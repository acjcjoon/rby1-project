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


def test_imu_overflow_is_explicit_and_clears_gap():
    mailbox = BoundedMailbox(max_imu=1)
    mailbox.put(Packet('imu', {'index': 1}))
    with pytest.raises(QueueOverflow):
        mailbox.put(Packet('imu', {'index': 2}))
    assert mailbox.imu_overflows == 1
    assert mailbox.drain() == []


def test_old_stereo_is_dropped_whole():
    mailbox = BoundedMailbox(stereo_max_age_sec=-1)
    mailbox.put(Packet('stereo', {'left': 'a', 'right': 'b'}))
    assert mailbox.drain() == []
    assert mailbox.dropped_stereo == 1


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


def test_duplex_reconnect_replays_static_only_and_rejects_commands():
    lab = SocketLink('lab', bind_host='127.0.0.1', port=0, reconnect_delay_sec=0.02)
    lab.start()
    upc = None
    try:
        port = eventually(lambda: lab.port)
        upc = SocketLink('upc', host='127.0.0.1', port=port, reconnect_delay_sec=0.02)
        upc.send(Packet('static_tf', {'transforms': []}), persistent=True)
        upc.start()
        eventually(lambda: lab.state()['connected'] and upc.state()['connected'])
        first_session = upc.state()['session_id']
        first_static = eventually(lab.receive)
        assert first_static[0].kind == 'static_tf'
        assert first_static[0].session_id == first_session
        assert lab.send(Packet('tracking_odom', {'sequence': 42}))
        assert eventually(upc.receive)[0].payload['sequence'] == 42
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
