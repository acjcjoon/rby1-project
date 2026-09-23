"""Versioned, bounded JSON + binary wire format; intentionally independent of ROS.

This is a protocol for a trusted robot LAN, not an authenticated Internet service.
Only the message kinds below are accepted. No Python objects or ROS CDR cross it.
"""

from dataclasses import dataclass
import json
import select
import struct
import time


MAGIC = b'RBV1'
VERSION = 1
HEADER = struct.Struct('!4sBBHII')
MAX_METADATA_BYTES = 256 * 1024
MAX_BLOB_BYTES = 16 * 1024 * 1024
KINDS = frozenset({
    'hello', 'ping', 'stereo', 'imu', 'static_tf', 'clock',
    'tracking_odom', 'slam_odom', 'tracking_status',
})


class ProtocolError(ValueError):
    """Malformed or unsupported packet. The connection must be discarded."""


@dataclass(frozen=True)
class Packet:
    kind: str
    payload: dict
    blob: bytes = b''
    session_id: str = ''


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError('duplicate JSON key')
        result[key] = value
    return result


def _invalid_constant(value):
    raise ProtocolError('nonfinite JSON number: ' + value)


def _validate(packet):
    if packet.kind not in KINDS or not isinstance(packet.payload, dict):
        raise ProtocolError('unknown kind or invalid payload')
    if not isinstance(packet.session_id, str) or not 1 <= len(packet.session_id) <= 64:
        raise ProtocolError('invalid session identifier')
    if packet.kind != 'stereo' and packet.blob:
        raise ProtocolError('binary data is only allowed for stereo')
    if packet.kind == 'stereo':
        if set(packet.payload) != {'left', 'right', 'left_info', 'right_info'}:
            raise ProtocolError('stereo fields do not match the protocol')
        offset = 0
        for key in ('left', 'right'):
            description = packet.payload[key]
            if not isinstance(description, dict):
                raise ProtocolError('invalid image description')
            if set(description) != {'message', 'offset', 'length'}:
                raise ProtocolError('invalid image description fields')
            size = description['length']
            if type(size) is not int or size < 1:
                raise ProtocolError('invalid image byte length')
            if type(description['offset']) is not int or description['offset'] != offset:
                raise ProtocolError('image data must be contiguous')
            message = description['message']
            if not isinstance(message, dict) or 'data' in message:
                raise ProtocolError('image metadata must exclude data')
            offset += size
        if offset != len(packet.blob):
            raise ProtocolError('image lengths do not cover the binary payload')
        if not all(isinstance(packet.payload[k], dict) for k in ('left_info', 'right_info')):
            raise ProtocolError('invalid camera info')


def encode_packet(packet):
    _validate(packet)
    try:
        metadata = json.dumps(
            {'kind': packet.kind, 'session_id': packet.session_id, 'payload': packet.payload},
            separators=(',', ':'), ensure_ascii=True, allow_nan=False,
        ).encode('utf-8')
    except (TypeError, ValueError, RecursionError) as exc:
        raise ProtocolError('payload is not finite JSON') from exc
    if not 0 < len(metadata) <= MAX_METADATA_BYTES or len(packet.blob) > MAX_BLOB_BYTES:
        raise ProtocolError('packet exceeds protocol size limits')
    return HEADER.pack(MAGIC, VERSION, 0, 0, len(metadata), len(packet.blob)) + metadata + packet.blob


def decode_header(header):
    if len(header) != HEADER.size:
        raise ProtocolError('invalid header length')
    magic, version, flags, reserved, json_size, blob_size = HEADER.unpack(header)
    if magic != MAGIC or version != VERSION or flags or reserved:
        raise ProtocolError('unsupported protocol header')
    if not 0 < json_size <= MAX_METADATA_BYTES or blob_size > MAX_BLOB_BYTES:
        raise ProtocolError('packet exceeds protocol size limits')
    return json_size, blob_size


def decode_body(metadata, blob):
    if not 0 < len(metadata) <= MAX_METADATA_BYTES or len(blob) > MAX_BLOB_BYTES:
        raise ProtocolError('packet exceeds protocol size limits')
    try:
        envelope = json.loads(
            metadata.decode('utf-8'), object_pairs_hook=_unique_object,
            parse_constant=_invalid_constant,
        )
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ProtocolError('invalid JSON metadata') from exc
    if not isinstance(envelope, dict) or set(envelope) != {'kind', 'session_id', 'payload'}:
        raise ProtocolError('invalid envelope')
    packet = Packet(envelope['kind'], envelope['payload'], bytes(blob), envelope['session_id'])
    try:
        _validate(packet)
    except (KeyError, TypeError) as exc:
        raise ProtocolError('malformed payload') from exc
    return packet


def _recv_exact(sock, count, deadline):
    result = bytearray()
    while len(result) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('incomplete packet timed out')
        readable, _, _ = select.select([sock], [], [], remaining)
        if not readable:
            raise TimeoutError('incomplete packet timed out')
        data = sock.recv(min(count - len(result), 256 * 1024))
        if not data:
            raise EOFError('peer closed TCP connection')
        result.extend(data)
    return bytes(result)


def read_packet(sock, timeout_sec=2.0):
    """Read exactly one packet with one deadline, including fragmented payloads."""
    deadline = time.monotonic() + timeout_sec
    json_size, blob_size = decode_header(_recv_exact(sock, HEADER.size, deadline))
    return decode_body(
        _recv_exact(sock, json_size, deadline),
        _recv_exact(sock, blob_size, deadline),
    )
