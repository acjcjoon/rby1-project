"""ROS-free duplex transport with bounded queues and reconnect isolation."""

from collections import Counter, deque
from dataclasses import replace
import select
import socket
import threading
import time
import uuid

from .wire import Packet, ProtocolError, encode_packet, read_packet


UPC_KINDS = frozenset({'stereo', 'imu', 'static_tf'})
LAB_KINDS = frozenset({'tracking_odom', 'slam_odom', 'tracking_status'})
TRACE_KINDS = frozenset({'stereo', 'tracking_odom', 'slam_odom'})


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
    """Ordered IMU, latest-only other messages. All access is thread safe.

    Stereo pairs are indivisible; replacing one never mixes left/right images.
    An IMU overflow invalidates the stream instead of silently hiding a gap.
    """

    def __init__(self, max_imu=512, stereo_max_age_sec=0.5):
        self._lock = threading.Lock()
        self._imu = deque()
        self._latest = {}
        self.max_imu = max_imu
        self.stereo_max_age_sec = stereo_max_age_sec
        self.dropped_stereo = 0
        self.imu_overflows = 0
        # Lifetime counters intentionally survive clear()/reconnect.  They are
        # exposed through bridge_status so a passive recorder can distinguish
        # producer gaps from latest-only mailbox replacement.
        self._enqueued = Counter()
        self._drained = Counter()
        self._replaced = Counter()
        self._expired = Counter()

    def put(self, packet):
        with self._lock:
            if packet.kind == 'imu':
                if len(self._imu) >= self.max_imu:
                    self.imu_overflows += 1
                    self._imu.clear()
                    raise QueueOverflow('IMU queue overflow; session must reset')
                self._imu.append(packet)
            else:
                if packet.kind in self._latest:
                    self._replaced[packet.kind] += 1
                    if packet.kind == 'stereo':
                        self.dropped_stereo += 1
                self._latest[packet.kind] = (packet, time.monotonic())
            self._enqueued[packet.kind] += 1

    def drain(self, max_imu=32):
        with self._lock:
            result = []
            # Calibration precedes the first frame after every reconnect.
            if 'static_tf' in self._latest:
                result.append(self._latest.pop('static_tf')[0])
                self._drained['static_tf'] += 1
            for _ in range(min(max_imu, len(self._imu))):
                result.append(self._imu.popleft())
                self._drained['imu'] += 1
            for kind, (packet, queued_at) in self._latest.items():
                if kind == 'stereo' and time.monotonic() - queued_at > self.stereo_max_age_sec:
                    self.dropped_stereo += 1
                    self._expired[kind] += 1
                    continue
                result.append(packet)
                self._drained[kind] += 1
            self._latest.clear()
            return result

    def clear(self):
        with self._lock:
            self._imu.clear()
            self._latest.clear()

    def metrics(self):
        """Return a JSON-safe snapshot without changing queue behavior."""
        with self._lock:
            queued = dict(Counter(packet.kind for packet in self._imu))
            queued.update({kind: 1 for kind in self._latest})
            return {
                'enqueued': dict(self._enqueued),
                'drained': dict(self._drained),
                'replaced': dict(self._replaced),
                'expired': dict(self._expired),
                'queued': queued,
            }


class SocketLink:
    """One worker owns all sockets; ROS callbacks only enqueue/dequeue packets."""

    def __init__(self, role, host='127.0.0.1', bind_host='0.0.0.0', port=7447,
                 timeout_sec=2.0, reconnect_delay_sec=1.0, max_imu=512,
                 stereo_max_age_sec=0.5, trace_callback=None):
        if role not in ('upc', 'lab'):
            raise ValueError('role must be upc or lab')
        if timeout_sec < 0.5 or reconnect_delay_sec < 0.01 or not 0 <= port <= 65535:
            raise ValueError('invalid socket timing or port')
        if max_imu < 1:
            raise ValueError('max_imu must be positive')
        if stereo_max_age_sec <= 0:
            raise ValueError('stereo_max_age_sec must be positive')
        self.role = role
        self.host, self.bind_host, self.port = host, bind_host, port
        self.timeout_sec, self.reconnect_delay_sec = timeout_sec, reconnect_delay_sec
        self.outbox = BoundedMailbox(max_imu, stereo_max_age_sec)
        self.inbox = BoundedMailbox(max_imu, stereo_max_age_sec)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._reset = threading.Event()
        self._thread = None
        self._socket = None
        self._listener = None
        self._persistent = {}
        self._connection_count = 0
        self._state = {'connected': False, 'session_id': '', 'reason': 'starting'}
        self._trace_callback = trace_callback

    def _trace(self, event, packet, wall_ns, monotonic_ns, **fields):
        if self._trace_callback is None or packet.kind not in TRACE_KINDS:
            return
        try:
            self._trace_callback({
                'event': event, 'packet': packet, 'wall_ns': int(wall_ns),
                'monotonic_ns': int(monotonic_ns), **fields,
            })
        except Exception:
            # Diagnostics must never reset or delay the transport session.
            pass

    def start(self):
        if self._thread is not None:
            raise RuntimeError('link already started')
        self._thread = threading.Thread(target=self._run, name='rby1-vslam-tcp', daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        self._reset.set()
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
                        tx_mailbox=self.outbox.metrics(),
                        rx_mailbox=self.inbox.metrics())

    def invalidate(self, reason):
        with self._lock:
            self._state.update(connected=False, reason=str(reason))
            self.outbox.clear()
            self.inbox.clear()
            self._reset.set()

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

    def _configure(self, sock):
        sock.settimeout(self.timeout_sec)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        # Limit the amount of stale video hidden inside the kernel send queue.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256 * 1024)

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

    def _session(self, sock):
        self._configure(sock)
        session = self._handshake(sock)
        allowed = LAB_KINDS if self.role == 'upc' else UPC_KINDS
        last_receive = last_send = time.monotonic()
        while not self._stop.is_set() and not self._reset.is_set():
            for packet in self.outbox.drain():
                if self._reset.is_set() or self._stop.is_set():
                    break
                outbound = replace(packet, session_id=session)
                encoded = encode_packet(outbound)
                started_ns = time.monotonic_ns()
                sock.sendall(encoded)
                finished_mono_ns = time.monotonic_ns()
                self._trace('socket_sent', outbound, time.time_ns(), finished_mono_ns,
                            io_duration_ns=finished_mono_ns - started_ns,
                            wire_bytes=len(encoded))
                last_send = time.monotonic()
            now = time.monotonic()
            if now - last_send >= min(0.5, self.timeout_sec / 3):
                sock.sendall(encode_packet(Packet('ping', {}, session_id=session)))
                last_send = now
            readable, _, _ = select.select([sock], [], [], 0.005)
            if readable:
                packet = read_packet(sock, self.timeout_sec)
                last_receive = time.monotonic()
                if packet.session_id != session:
                    raise ProtocolError('packet from another session')
                if packet.kind == 'ping':
                    if packet.payload:
                        raise ProtocolError('invalid heartbeat')
                    continue
                if packet.kind not in allowed:
                    raise ProtocolError('message kind is not allowed in this direction')
                self._trace('socket_received', packet, time.time_ns(), time.monotonic_ns())
                with self._lock:
                    if not self._reset.is_set():
                        self.inbox.put(packet)
            if time.monotonic() - last_receive > self.timeout_sec:
                raise TimeoutError('peer heartbeat timed out')

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
