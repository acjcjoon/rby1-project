"""ROS-free regression tests: run with python -m pytest test/test_transport.py."""

import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rby1_vslam.transport import BoundedMailbox, QueueOverflow, SocketLink, StereoSynchronizer
from rby1_vslam.wire import (
    HEADER, MAGIC, MAX_BLOB_BYTES, VERSION, Packet, ProtocolError,
    decode_body, decode_header, encode_packet, read_packet,
)


def eventually(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.005)
    raise AssertionError('condition did not become true before timeout')


def test_fragmented_binary_stereo_roundtrip():
    blob = bytes(range(256)) * 3
    payload = {
        'left': {'message': {'encoding': 'mono8'}, 'offset': 0, 'length': 384},
        'right': {'message': {'encoding': 'mono8'}, 'offset': 384, 'length': 384},
        'left_info': {}, 'right_info': {},
    }
    expected = Packet('stereo', payload, blob, 'session')
    first, second = socket.socketpair()
    serialized = encode_packet(expected)

    def send_fragments():
        for start in range(0, len(serialized), 7):
            first.sendall(serialized[start:start + 7])

    sender = threading.Thread(target=send_fragments)
    try:
        sender.start()
        assert read_packet(second) == expected
        sender.join(timeout=1)
    finally:
        first.close()
        second.close()


@pytest.mark.parametrize('header', [
    HEADER.pack(b'BAD!', VERSION, 0, 0, 1, 0),
    HEADER.pack(MAGIC, 99, 0, 0, 1, 0),
    HEADER.pack(MAGIC, VERSION, 1, 0, 1, 0),
    HEADER.pack(MAGIC, VERSION, 0, 0, 0, 0),
    HEADER.pack(MAGIC, VERSION, 0, 0, 1, MAX_BLOB_BYTES + 1),
])
def test_invalid_header_rejected_before_allocation(header):
    with pytest.raises(ProtocolError):
        decode_header(header)


@pytest.mark.parametrize('metadata', [
    b'{"kind":"ping","kind":"imu","session_id":"x","payload":{}}',
    b'{"kind":"ping","session_id":"x","payload":{"x":NaN}}',
    b'{"kind":"command","session_id":"x","payload":{}}',
    b'{"kind":"ping","session_id":"","payload":{}}',
    b'{"kind":[],"session_id":"x","payload":{}}',
    b'{"kind":"ping","session_id":"x","payload":{},"extra":1}',
])
def test_invalid_metadata_rejected(metadata):
    with pytest.raises(ProtocolError):
        decode_body(metadata, b'')


def test_binary_rejected_for_non_image_and_stereo_offsets_checked():
    with pytest.raises(ProtocolError):
        encode_packet(Packet('imu', {}, b'bad', 's'))
    with pytest.raises(ProtocolError):
        encode_packet(Packet('stereo', {
            'left': {'message': {}, 'offset': 0, 'length': 2},
            'right': {'message': {}, 'offset': 1, 'length': 2},
            'left_info': {}, 'right_info': {},
        }, b'abcd', 's'))


def test_partial_frame_has_one_deadline():
    first, second = socket.socketpair()
    try:
        first.sendall(encode_packet(Packet('ping', {}, session_id='s'))[:5])
        start = time.monotonic()
        with pytest.raises(TimeoutError):
            read_packet(second, timeout_sec=0.05)
        assert time.monotonic() - start < 0.5
    finally:
        first.close()
        second.close()


def test_mailbox_latest_stereo_and_ordered_imu():
    mailbox = BoundedMailbox(max_imu=3)
    for index in range(3):
        mailbox.put(Packet('stereo', {'index': index}))
        mailbox.put(Packet('imu', {'index': index}))
    drained = mailbox.drain()
    assert [packet.payload['index'] for packet in drained if packet.kind == 'imu'] == [0, 1, 2]
    assert [packet.payload['index'] for packet in drained if packet.kind == 'stereo'] == [2]
    assert mailbox.dropped_stereo == 2
    assert mailbox.metrics() == {
        'enqueued': {'stereo': 3, 'imu': 3},
        'drained': {'imu': 3, 'stereo': 1},
        'replaced': {'stereo': 2},
        'expired': {},
        'dropped_oldest': {},
        'discarded_on_clear': {},
        'queued': {},
    }


def test_imu_overflow_is_explicit_and_clears_gap():
    mailbox = BoundedMailbox(max_imu=1)
    mailbox.put(Packet('imu', {'index': 1}))
    with pytest.raises(QueueOverflow):
        mailbox.put(Packet('imu', {'index': 2}))
    assert mailbox.imu_overflows == 1
    assert mailbox.drain() == []


def test_mailbox_reports_pending_items_left_by_bounded_imu_drain():
    mailbox = BoundedMailbox(max_imu=3)
    for index in range(3):
        mailbox.put(Packet('imu', {'index': index}))
    assert mailbox.has_pending()
    assert [packet.payload['index'] for packet in mailbox.drain(max_imu=2)] == [0, 1]
    assert mailbox.has_pending()
    assert [packet.payload['index'] for packet in mailbox.drain()] == [2]
    assert not mailbox.has_pending()


def test_old_stereo_is_dropped_whole():
    mailbox = BoundedMailbox(stereo_max_age_sec=-1)
    mailbox.put(Packet('stereo', {'left': 'a', 'right': 'b'}))
    assert mailbox.drain() == []
    assert mailbox.dropped_stereo == 1
    assert mailbox.metrics()['expired'] == {'stereo': 1}


def test_mailbox_preserves_pose_burst_in_arrival_order():
    mailbox = BoundedMailbox()
    mailbox.put(Packet('tracking_odom', {'index': 1}))
    mailbox.put(Packet('slam_odom', {'index': 10}))
    mailbox.put(Packet('tracking_odom', {'index': 2}))
    drained = mailbox.drain()
    assert [(packet.kind, packet.payload['index']) for packet in drained] == [
        ('tracking_odom', 1), ('slam_odom', 10), ('tracking_odom', 2)]
    assert mailbox.metrics()['replaced'] == {}
    assert mailbox.metrics()['dropped_oldest'] == {}


def test_mailbox_pose_limit_drops_oldest_of_same_kind_explicitly():
    mailbox = BoundedMailbox(pose_queue_size=2)
    mailbox.put(Packet('tracking_odom', {'index': 1}))
    mailbox.put(Packet('slam_odom', {'index': 10}))
    mailbox.put(Packet('tracking_odom', {'index': 2}))
    mailbox.put(Packet('tracking_odom', {'index': 3}))
    assert mailbox.metrics()['queued'] == {'tracking_odom': 2, 'slam_odom': 1}
    assert [(packet.kind, packet.payload['index']) for packet in mailbox.drain()] == [
        ('slam_odom', 10), ('tracking_odom', 2), ('tracking_odom', 3)]
    assert mailbox.metrics()['dropped_oldest'] == {'tracking_odom': 1}


def test_mailbox_counts_packets_discarded_by_session_clear():
    mailbox = BoundedMailbox()
    mailbox.put(Packet('imu', {'index': 1}))
    mailbox.put(Packet('tracking_odom', {'index': 2}))
    mailbox.put(Packet('tracking_status', {'index': 3}))
    mailbox.clear()
    assert mailbox.metrics()['discarded_on_clear'] == {
        'imu': 1, 'tracking_odom': 1, 'tracking_status': 1}


def test_stereo_sync_preserves_matching_calibration_and_stamps():
    sync = StereoSynchronizer(slop_ns=5, max_queue=2)
    assert sync.add('left', 100, 'left100') == []
    assert sync.add('right', 103, 'right103') == []
    assert sync.add('left_info', 99, 'wrong_info') == []
    assert sync.add('right_info', 103, 'info103') == []
    assert sync.add('left_info', 100, 'info100') == [
        ('left100', 'right103', 'info100', 'info103')]
    for stamp in (200, 210, 220):
        sync.add('left', stamp, stamp)
    assert len(sync.queues['left']) == 2
    assert sync.dropped == 1
    sync.clear()
    assert all(not queue for queue in sync.queues.values())


def test_session_runs_one_reader_and_one_writer_concurrently():
    reader_started = threading.Event()
    writer_started = threading.Event()

    class ProbeSocket:
        def shutdown(self, _):
            pass

    class ProbeLink(SocketLink):
        def _configure(self, sock):
            pass

        def _handshake(self, sock):
            self._set_connected('probe-session')
            return 'probe-session'

        def _receive_session(self, sock, session):
            assert session == 'probe-session'
            reader_started.set()
            assert writer_started.wait(1.0)
            self._reset.set()
            self._outbox_ready.set()

        def _send_session(self, sock, session):
            assert session == 'probe-session'
            writer_started.set()
            assert reader_started.wait(1.0)
            assert self._reset.wait(1.0)

    link = ProbeLink('upc')
    link._session(ProbeSocket())
    assert reader_started.is_set()
    assert writer_started.is_set()


def test_receive_progresses_while_large_stereo_send_is_blocked():
    """A blocked video write must not starve pose traffic in the other direction."""
    send_started = threading.Event()
    release_send = threading.Event()
    pose_ready = threading.Event()
    session_errors = []
    session = 'full-duplex-session'
    incoming, peer = socket.socketpair()

    class BlockingSendSocket:
        """Read from a real socket, but hold sendall until the test releases it."""

        sent_bytes = 0

        def fileno(self):
            return incoming.fileno()

        def recv(self, size):
            return incoming.recv(size)

        def sendall(self, data):
            self.sent_bytes = len(data)
            send_started.set()
            if not release_send.wait(2.0):
                raise TimeoutError('test did not release blocked send')

        def shutdown(self, how):
            release_send.set()
            incoming.shutdown(how)

    class ProbeLink(SocketLink):
        def _configure(self, sock):
            pass

        def _handshake(self, sock):
            self._set_connected(session)
            return session

    sock = BlockingSendSocket()
    link = ProbeLink('upc', receive_callback=pose_ready.set)

    def run_session():
        try:
            link._session(sock)
        except Exception as exc:  # Preserve worker failures for the main test thread.
            session_errors.append(exc)

    worker = threading.Thread(target=run_session)
    try:
        worker.start()
        eventually(lambda: link.state()['connected'])
        blob = b'x' * (1024 * 1024)
        stereo = Packet('stereo', {
            'left': {'message': {}, 'offset': 0, 'length': len(blob) // 2},
            'right': {
                'message': {}, 'offset': len(blob) // 2,
                'length': len(blob) - len(blob) // 2,
            },
            'left_info': {}, 'right_info': {},
        }, blob)
        assert link.send(stereo)
        assert send_started.wait(1.0)
        assert sock.sent_bytes > len(blob)

        peer.sendall(encode_packet(Packet(
            'tracking_odom', {'sequence': 7}, session_id=session)))
        assert pose_ready.wait(1.0)
        eventually(lambda: link.state()['rx_mailbox']['enqueued'].get(
            'tracking_odom', 0) == 1)

        # The writer remains inside sendall, yet the independent reader has
        # already decoded, queued, and notified the pose packet.
        assert worker.is_alive()
        assert not release_send.is_set()
        received = link.receive()
        assert [packet.payload['sequence'] for packet in received] == [7]
    finally:
        link.invalidate('test complete')
        release_send.set()
        worker.join(timeout=2.0)
        peer.close()
        incoming.close()
    assert not worker.is_alive()
    assert session_errors == []


def test_receive_notification_failure_is_isolated_from_transport():
    def fail():
        raise RuntimeError('guard condition unavailable')

    link = SocketLink('upc', receive_callback=fail)
    link._notify_receive()


def test_duplex_reconnect_replays_static_only_and_rejects_commands():
    lab_trace = []
    upc_trace = []
    lab = SocketLink('lab', bind_host='127.0.0.1', port=0, reconnect_delay_sec=0.02,
                     trace_callback=lab_trace.append)
    lab.start()
    upc = None
    try:
        port = eventually(lambda: lab.port)
        upc = SocketLink('upc', host='127.0.0.1', port=port, reconnect_delay_sec=0.02,
                         trace_callback=upc_trace.append)
        upc.send(Packet('static_tf', {'transforms': []}), persistent=True)
        upc.start()
        eventually(lambda: lab.state()['connected'] and upc.state()['connected'])
        first_session = upc.state()['session_id']
        first_static = eventually(lab.receive)
        assert first_static[0].kind == 'static_tf'
        assert first_static[0].session_id == first_session
        assert lab.send(Packet('tracking_odom', {'sequence': 42}))
        assert eventually(upc.receive)[0].payload['sequence'] == 42
        eventually(lambda: lab_trace and upc_trace)
        assert lab_trace[-1]['event'] == 'socket_sent'
        assert lab_trace[-1]['packet'].kind == 'tracking_odom'
        assert lab_trace[-1]['wire_bytes'] > 0
        assert lab_trace[-1]['io_duration_ns'] >= 0
        assert upc_trace[-1]['event'] == 'socket_received'
        assert upc_trace[-1]['packet'].kind == 'tracking_odom'
        for sequence in range(5):
            assert upc.send(Packet('imu', {'sequence': sequence}))
        received = []

        def received_imu():
            received.extend(lab.receive())
            return len(received) == 5

        eventually(received_imu)
        assert [packet.payload['sequence'] for packet in received] == list(range(5))
        with pytest.raises(ProtocolError):
            upc.send(Packet('tracking_odom', {}))
        upc.invalidate('test reset')
        eventually(lambda: upc.state()['connected'] and upc.state()['session_id'] != first_session)
        second_session = upc.state()['session_id']
        replay = eventually(lab.receive)
        assert len(replay) == 1 and replay[0].kind == 'static_tf'
        assert replay[0].session_id == second_session
        assert upc.receive() == []
    finally:
        if upc is not None:
            upc.close()
        lab.close()


def test_duplex_pose_burst_is_delivered_without_latest_only_replacement():
    lab_trace = []
    pose_ready = threading.Event()
    lab = SocketLink('lab', bind_host='127.0.0.1', port=0, reconnect_delay_sec=0.02,
                     pose_queue_size=64, trace_callback=lab_trace.append)
    lab.start()
    upc = None
    try:
        port = eventually(lambda: lab.port)
        upc = SocketLink('upc', host='127.0.0.1', port=port, reconnect_delay_sec=0.02,
                         pose_queue_size=64, receive_callback=pose_ready.set)
        upc.start()
        eventually(lambda: lab.state()['connected'] and upc.state()['connected'])
        for sequence in range(50):
            assert lab.send(Packet('tracking_odom', {'sequence': sequence}))
        eventually(lambda: sum(
            trace['event'] == 'socket_sent' and trace['packet'].kind == 'tracking_odom'
            for trace in lab_trace) == 50)
        assert pose_ready.wait(1.0)
        eventually(lambda: upc.state()['rx_mailbox']['enqueued'].get(
            'tracking_odom', 0) == 50)
        received = upc.receive()
        assert [packet.payload['sequence'] for packet in received] == list(range(50))
        assert lab.state()['tx_mailbox']['dropped_oldest'] == {}
        assert upc.state()['rx_mailbox']['dropped_oldest'] == {}
    finally:
        if upc is not None:
            upc.close()
        lab.close()


def test_wrong_direction_peer_packet_disconnects():
    lab = SocketLink('lab', bind_host='127.0.0.1', port=0, reconnect_delay_sec=0.02)
    lab.start()
    peer = None
    try:
        port = eventually(lambda: lab.port)
        peer = socket.create_connection(('127.0.0.1', port))
        hello = read_packet(peer)
        peer.sendall(encode_packet(Packet('hello', {'role': 'upc'}, session_id=hello.session_id)))
        eventually(lambda: lab.state()['connected'])
        peer.sendall(encode_packet(Packet('tracking_odom', {}, session_id=hello.session_id)))
        eventually(lambda: not lab.state()['connected'])
        assert 'direction' in lab.state()['reason']
        assert lab.receive() == []
    finally:
        if peer is not None:
            peer.close()
        lab.close()
