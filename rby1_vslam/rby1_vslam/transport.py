"""ROS-free full-duplex transport with bounded queues and reconnect isolation."""

from collections import Counter, deque
from dataclasses import replace
import socket
import threading
import time
import uuid

from .wire import Packet, ProtocolError, encode_packet, read_packet


UPC_KINDS = frozenset({'stereo', 'imu', 'static_tf'})
LAB_KINDS = frozenset({'tracking_odom', 'slam_odom', 'tracking_status'})
POSE_KINDS = frozenset({'tracking_odom', 'slam_odom'})


class QueueOverflow(RuntimeError):
    pass


class StereoSynchronizer:
    """Bounded four-topic synchronizer; each CameraInfo matches its image stamp.

    Image stamps may differ by ``slop_ns`` between the two sensors. Original
    stamps are never rewritten. Use one instance from the ROS executor thread.
    """

    def __init__(self, slop_ns=5_000_000, max_queue=8):
        if slop_ns < 0 or max_queue < 1:
            raise ValueError('invalid stereo synchronizer limits')
        self.slop_ns = slop_ns
        self.max_queue = max_queue
        self.queues = {name: {} for name in ('left', 'right', 'left_info', 'right_info')}
        self.dropped = 0

    def clear(self):
        for queue in self.queues.values():
            queue.clear()

    def add(self, name, stamp_ns, message):
        queue = self.queues[name]
        queue[stamp_ns] = message
        while len(queue) > self.max_queue:
            del queue[min(queue)]
            self.dropped += 1
        ready = []
        for left_stamp in sorted(self.queues['left']):
            if left_stamp not in self.queues['left_info']:
                continue
            candidates = [stamp for stamp in self.queues['right']
                          if abs(stamp - left_stamp) <= self.slop_ns
                          and stamp in self.queues['right_info']]
            if not candidates:
                continue
            right_stamp = min(candidates, key=lambda stamp: abs(stamp - left_stamp))
            ready.append((
                self.queues['left'].pop(left_stamp),
                self.queues['right'].pop(right_stamp),
                self.queues['left_info'].pop(left_stamp),
                self.queues['right_info'].pop(right_stamp),
            ))
        return ready


class BoundedMailbox:
    """Ordered IMU/poses/stereo and latest-only state. Access is thread safe.

    Stereo pairs are indivisible and use a short FIFO to absorb temporary stalls.
    Capacity one keeps only the latest pair.
    Larger FIFOs report capacity loss as ``dropped_oldest`` and age loss as
    ``expired``; they never grow without a bound or rewrite source timestamps.
    An IMU overflow invalidates the stream instead of silently hiding a gap.
    Returned poses use a small per-kind FIFO so a receive burst cannot silently
    collapse to one sample.  If that bounded FIFO fills, the oldest pose of the
    same kind is discarded and reported explicitly in ``dropped_oldest``.
    """

    def __init__(self, max_imu=512, stereo_max_age_sec=0.5, pose_queue_size=32,
                 stereo_queue_size=3):
        if (max_imu < 1 or pose_queue_size < 1 or
                type(stereo_queue_size) is not int or stereo_queue_size < 1):
            raise ValueError('mailbox queue sizes must be positive')
        self._lock = threading.Lock()
        self._imu = deque()
        self._poses = deque()
        self._stereo = deque()
        self._pose_counts = Counter()
        self._latest = {}
        self.max_imu = max_imu
        self.pose_queue_size = pose_queue_size
        self.stereo_queue_size = stereo_queue_size
        self._stereo_high_watermark = 0
        self.stereo_max_age_sec = stereo_max_age_sec
        self.dropped_stereo = 0
        self.imu_overflows = 0
        # Lifetime counters intentionally survive clear()/reconnect.  They are
        # exposed through bridge_status to identify capacity/age losses.
        self._enqueued = Counter()
        self._drained = Counter()
        self._replaced = Counter()
        self._expired = Counter()
        self._dropped_oldest = Counter()
        self._discarded_on_clear = Counter()

    def _drop_oldest_pose(self, kind):
        for index, (packet, _) in enumerate(self._poses):
            if packet.kind == kind:
                del self._poses[index]
                self._pose_counts[kind] -= 1
                if not self._pose_counts[kind]:
                    del self._pose_counts[kind]
                self._dropped_oldest[kind] += 1
                return
        raise RuntimeError('pose queue accounting is inconsistent')

    def _expire_stereo(self, now):
        while (self._stereo and
               now - self._stereo[0][1] > self.stereo_max_age_sec):
            self._stereo.popleft()
            self.dropped_stereo += 1
            self._expired['stereo'] += 1

    def put(self, packet):
        with self._lock:
            if packet.kind == 'imu':
                if len(self._imu) >= self.max_imu:
                    self.imu_overflows += 1
                    self._discarded_on_clear.update(
                        packet.kind for packet in self._imu)
                    self._imu.clear()
                    raise QueueOverflow('IMU queue overflow; session must reset')
                self._imu.append(packet)
            elif packet.kind in POSE_KINDS:
                if self._pose_counts[packet.kind] >= self.pose_queue_size:
                    self._drop_oldest_pose(packet.kind)
                self._poses.append((packet, time.monotonic()))
                self._pose_counts[packet.kind] += 1
            elif packet.kind == 'stereo':
                now = time.monotonic()
                self._expire_stereo(now)
                if len(self._stereo) >= self.stereo_queue_size:
                    self._stereo.popleft()
                    self.dropped_stereo += 1
                    counter = (self._replaced if self.stereo_queue_size == 1
                               else self._dropped_oldest)
                    counter['stereo'] += 1
                self._stereo.append((packet, now))
                self._stereo_high_watermark = max(
                    self._stereo_high_watermark, len(self._stereo))
            else:
                if packet.kind in self._latest:
                    self._replaced[packet.kind] += 1
                self._latest[packet.kind] = (packet, time.monotonic())
            self._enqueued[packet.kind] += 1

    def drain(self, max_imu=32, max_stereo=None):
        if max_stereo is not None and max_stereo < 1:
            raise ValueError('max_stereo must be positive or None')
        with self._lock:
            result = []
            # Calibration precedes the first frame after every reconnect.
            if 'static_tf' in self._latest:
                result.append(self._latest.pop('static_tf')[0])
                self._drained['static_tf'] += 1
            for _ in range(min(max_imu, len(self._imu))):
                result.append(self._imu.popleft())
                self._drained['imu'] += 1
            while self._poses:
                packet, _ = self._poses.popleft()
                self._pose_counts[packet.kind] -= 1
                if not self._pose_counts[packet.kind]:
                    del self._pose_counts[packet.kind]
                result.append(packet)
                self._drained[packet.kind] += 1
            self._expire_stereo(time.monotonic())
            stereo_count = (len(self._stereo) if max_stereo is None
                            else min(max_stereo, len(self._stereo)))
            for _ in range(stereo_count):
                result.append(self._stereo.popleft()[0])
                self._drained['stereo'] += 1
            for kind, (packet, queued_at) in self._latest.items():
                result.append(packet)
                self._drained[kind] += 1
            self._latest.clear()
            return result

    def clear(self):
        with self._lock:
            self._discarded_on_clear.update(packet.kind for packet in self._imu)
            self._discarded_on_clear.update(
                packet.kind for packet, _ in self._poses)
            self._discarded_on_clear.update(
                packet.kind for packet, _ in self._stereo)
            self._discarded_on_clear.update(self._latest.keys())
            self._imu.clear()
            self._poses.clear()
            self._stereo.clear()
            self._pose_counts.clear()
            self._latest.clear()

    def has_pending(self):
        with self._lock:
            return bool(self._imu or self._poses or self._stereo or self._latest)

    def metrics(self):
        """Return a JSON-safe snapshot without changing queue behavior."""
        with self._lock:
            queued = dict(Counter(packet.kind for packet in self._imu))
            queued.update(self._pose_counts)
            queued.update({kind: 1 for kind in self._latest})
            if self._stereo:
                queued['stereo'] = len(self._stereo)
            return {
                'enqueued': dict(self._enqueued),
                'drained': dict(self._drained),
                'replaced': dict(self._replaced),
                'expired': dict(self._expired),
                'dropped_oldest': dict(self._dropped_oldest),
                'discarded_on_clear': dict(self._discarded_on_clear),
                'queued': queued,
                'stereo_capacity': self.stereo_queue_size,
                'stereo_high_watermark': self._stereo_high_watermark,
                'stereo_oldest_age_ms': (
                    max(0.0, time.monotonic() - self._stereo[0][1]) * 1000
                    if self._stereo else None),
            }


class SocketLink:
    """One writer and one reader share each full-duplex TCP session.

    ROS callbacks only enqueue/dequeue packets.  Keeping socket writes and reads
    on different threads prevents a large stereo ``sendall`` from starving the
    small pose packets travelling in the opposite direction.
    """

    def __init__(self, role, host='127.0.0.1', bind_host='0.0.0.0', port=7447,
                 timeout_sec=2.0, reconnect_delay_sec=1.0, max_imu=512,
                 stereo_max_age_sec=0.5, pose_queue_size=32,
                 receive_callback=None,
                 stereo_tx_queue_size=3, stereo_rx_queue_size=3,
                 socket_send_buffer_bytes=0):
        if role not in ('upc', 'lab'):
            raise ValueError('role must be upc or lab')
        if timeout_sec < 0.5 or reconnect_delay_sec < 0.01 or not 0 <= port <= 65535:
            raise ValueError('invalid socket timing or port')
        if max_imu < 1:
            raise ValueError('max_imu must be positive')
        if pose_queue_size < 1:
            raise ValueError('pose_queue_size must be positive')
        if stereo_max_age_sec <= 0:
            raise ValueError('stereo_max_age_sec must be positive')
        if (type(stereo_tx_queue_size) is not int or stereo_tx_queue_size < 1 or
                type(stereo_rx_queue_size) is not int or stereo_rx_queue_size < 1):
            raise ValueError('stereo queue sizes must be positive integers')
        if type(socket_send_buffer_bytes) is not int or socket_send_buffer_bytes < 0:
            raise ValueError('socket_send_buffer_bytes must be a nonnegative integer')
        self.role = role
        self.host, self.bind_host, self.port = host, bind_host, port
        self.timeout_sec, self.reconnect_delay_sec = timeout_sec, reconnect_delay_sec
        self.socket_send_buffer_bytes = socket_send_buffer_bytes
        self._initial_send_buffer_bytes = None
        self.outbox = BoundedMailbox(
            max_imu, stereo_max_age_sec, pose_queue_size, stereo_tx_queue_size)
        self.inbox = BoundedMailbox(
            max_imu, stereo_max_age_sec, pose_queue_size, stereo_rx_queue_size)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._reset = threading.Event()
        self._outbox_ready = threading.Event()
        self._thread = None
        self._socket = None
        self._listener = None
        self._persistent = {}
        self._connection_count = 0
        self._state = {'connected': False, 'session_id': '', 'reason': 'starting'}
        self._receive_callback = receive_callback

    def _notify_receive(self):
        if self._receive_callback is None:
            return
        try:
            self._receive_callback()
        except Exception:
            # Waking the ROS executor is an optimization; its 10 ms fallback
            # timer must continue to work if the notification itself fails.
            pass

    def start(self):
        if self._thread is not None:
            raise RuntimeError('link already started')
        self._thread = threading.Thread(target=self._run, name='rby1-vslam-tcp', daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        self._reset.set()
        self._outbox_ready.set()
        with self._lock:
            sockets = [self._socket, self._listener]
        for sock in sockets:
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                sock.close()
        if self._thread is not None:
            self._thread.join(timeout=self.timeout_sec + 1.0)

    def state(self):
        with self._lock:
            return dict(self._state, role=self.role,
                        dropped_stereo=self.outbox.dropped_stereo + self.inbox.dropped_stereo,
                        imu_overflows=self.outbox.imu_overflows + self.inbox.imu_overflows,
                        connection_count=self._connection_count,
                        socket_send_buffer_bytes=self.socket_send_buffer_bytes,
                        initial_send_buffer_bytes=self._initial_send_buffer_bytes,
                        tx_mailbox=self.outbox.metrics(),
                        rx_mailbox=self.inbox.metrics())

    def invalidate(self, reason):
        with self._lock:
            self._state.update(connected=False, reason=str(reason))
            self.outbox.clear()
            self.inbox.clear()
            self._reset.set()
            self._outbox_ready.set()

    def send(self, packet, persistent=False):
        allowed = UPC_KINDS if self.role == 'upc' else LAB_KINDS
        if packet.kind not in allowed:
            raise ProtocolError('message kind is not allowed in this direction')
        with self._lock:
            if persistent:
                if packet.kind != 'static_tf':
                    raise ValueError('only static_tf may persist across sessions')
                self._persistent[packet.kind] = packet
            if not self._state['connected']:
                return False
            try:
                self.outbox.put(packet)
            except QueueOverflow as exc:
                self.invalidate(str(exc))
                return False
            self._outbox_ready.set()
            return True

    def receive(self):
        with self._lock:
            if not self._state['connected']:
                return []
            session = self._state['session_id']
            return [p for p in self.inbox.drain() if p.session_id == session]

    def _set_connected(self, session):
        with self._lock:
            self.outbox.clear()
            self.inbox.clear()
            self._reset.clear()
            self._connection_count += 1
            self._state.update(connected=True, session_id=session, reason='connected')
            for packet in self._persistent.values():
                self.outbox.put(packet)
            self._outbox_ready.set()

    def _configure(self, sock):
        sock.settimeout(self.timeout_sec)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        # Zero leaves Linux TCP send-buffer autotuning enabled. An explicit
        # value sets a fixed limit instead.
        if self.socket_send_buffer_bytes:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF,
                            self.socket_send_buffer_bytes)
        with self._lock:
            self._initial_send_buffer_bytes = sock.getsockopt(
                socket.SOL_SOCKET, socket.SO_SNDBUF)

    def _handshake(self, sock):
        if self.role == 'lab':
            session = uuid.uuid4().hex
            sock.sendall(encode_packet(Packet('hello', {'role': 'lab'}, session_id=session)))
            reply = read_packet(sock, self.timeout_sec)
            if reply != Packet('hello', {'role': 'upc'}, session_id=session):
                raise ProtocolError('invalid UPC handshake')
        else:
            reply = read_packet(sock, self.timeout_sec)
            if reply.kind != 'hello' or reply.payload != {'role': 'lab'}:
                raise ProtocolError('invalid LAB handshake')
            session = reply.session_id
            sock.sendall(encode_packet(Packet('hello', {'role': 'upc'}, session_id=session)))
        self._set_connected(session)
        return session

    def _send_session(self, sock, session):
        heartbeat_sec = min(0.5, self.timeout_sec / 3)
        last_send = time.monotonic()
        while not self._stop.is_set() and not self._reset.is_set():
            # Clear before draining so a concurrent producer cannot lose its
            # wakeup between the drain and the following wait.
            self._outbox_ready.clear()
            # A reset may race with clear().  Recheck before doing any more
            # work so that the consumed wakeup cannot delay reconnect/close.
            if self._stop.is_set() or self._reset.is_set():
                return
            # Leave waiting frames in the bounded FIFO, rather than hiding an
            # entire burst in a local batch. Recheck age between video sends.
            for packet in self.outbox.drain(max_stereo=1):
                if self._reset.is_set() or self._stop.is_set():
                    return
                outbound = replace(packet, session_id=session)
                sock.sendall(encode_packet(outbound))
                last_send = time.monotonic()
            now = time.monotonic()
            if now - last_send >= heartbeat_sec:
                sock.sendall(encode_packet(Packet('ping', {}, session_id=session)))
                last_send = now
            if self.outbox.has_pending():
                continue
            if self._stop.is_set() or self._reset.is_set():
                return
            self._outbox_ready.wait(max(0.0, heartbeat_sec - (time.monotonic() - last_send)))

    def _receive_session(self, sock, session):
        allowed = LAB_KINDS if self.role == 'upc' else UPC_KINDS
        while not self._stop.is_set() and not self._reset.is_set():
            packet = read_packet(sock, self.timeout_sec)
            if packet.session_id != session:
                raise ProtocolError('packet from another session')
            if packet.kind == 'ping':
                if packet.payload:
                    raise ProtocolError('invalid heartbeat')
                continue
            if packet.kind not in allowed:
                raise ProtocolError('message kind is not allowed in this direction')
            queued = False
            with self._lock:
                if not self._reset.is_set():
                    self.inbox.put(packet)
                    queued = True
            if queued:
                # Never run an application callback while transport locks are held.
                self._notify_receive()

    def _session(self, sock):
        self._configure(sock)
        session = self._handshake(sock)
        errors = []

        def fail(exc):
            with self._lock:
                if self._reset.is_set() or self._stop.is_set():
                    return
                errors.append(exc)
                self.invalidate(str(exc))
            # Wake the opposite worker immediately if it is blocked in socket I/O.
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

        def receive():
            try:
                self._receive_session(sock, session)
            except (OSError, EOFError, ValueError, TimeoutError, QueueOverflow) as exc:
                # An explicit invalidate/close owns its existing reason.  Only
                # spontaneous I/O/protocol failures replace it.
                fail(exc)

        receiver = threading.Thread(
            target=receive, name=f'rby1-vslam-{self.role}-rx', daemon=True)
        receiver.start()
        try:
            self._send_session(sock, session)
        except (OSError, EOFError, ValueError, TimeoutError, QueueOverflow) as exc:
            fail(exc)
        finally:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            receiver.join(timeout=self.timeout_sec + 1.0)
        if receiver.is_alive():
            raise TimeoutError('TCP receive worker did not stop')
        if errors:
            raise errors[0]

    def _run(self):
        while not self._stop.is_set():
            sock = None
            try:
                if self.role == 'lab':
                    with self._lock:
                        if self._listener is None:
                            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            try:
                                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                                listener.bind((self.bind_host, self.port))
                                listener.listen(1)
                                listener.settimeout(0.2)
                            except OSError:
                                listener.close()
                                raise
                            self._listener = listener
                            self.port = listener.getsockname()[1]
                        listener = self._listener
                    try:
                        sock, _ = listener.accept()
                    except socket.timeout:
                        continue
                else:
                    sock = socket.create_connection((self.host, self.port), self.timeout_sec)
                with self._lock:
                    self._socket = sock
                self._session(sock)
            except (OSError, EOFError, ValueError, TimeoutError, QueueOverflow) as exc:
                self.invalidate(str(exc))
            finally:
                if sock is not None:
                    sock.close()
                with self._lock:
                    self._socket = None
                    self._state['connected'] = False
                    self.outbox.clear()
                    self.inbox.clear()
            self._stop.wait(self.reconnect_delay_sec)
