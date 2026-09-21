import json

import pytest

from rby1_navigation.navigation_protocol import (
    ProtocolError,
    encode_state,
    parse_command,
)


def test_move_to_protocol_uses_relative_metre_and_radian_values():
    command = parse_command(
        json.dumps({
            'version': 1,
            'command': 'move_to',
            'command_id': 'move-1',
            'x': 0.4,
            'y': -0.2,
            'yaw': 0.5,
            'timeout_sec': 12.0,
        }),
        default_timeout_sec=30.0,
        max_timeout_sec=120.0,
    )

    assert command.action == 'move_to'
    assert command.command_id == 'move-1'
    assert (command.x, command.y, command.yaw) == (0.4, -0.2, 0.5)
    assert command.timeout_sec == 12.0


@pytest.mark.parametrize('field,value', [
    ('x', float('nan')),
    ('y', float('inf')),
    ('yaw', None),
    ('timeout_sec', 0.0),
])
def test_move_to_protocol_rejects_invalid_numbers(field, value):
    payload = {
        'version': 1,
        'command': 'move_to',
        'command_id': 'move-1',
        'x': 0.0,
        'y': 0.0,
        'yaw': 0.0,
        'timeout_sec': 10.0,
    }
    payload[field] = value

    with pytest.raises(ProtocolError):
        parse_command(
            json.dumps(payload),
            default_timeout_sec=30.0,
            max_timeout_sec=120.0,
        )


def test_state_protocol_contains_correlated_progress_and_errors():
    payload = json.loads(encode_state(
        command_id='move-1',
        state='running',
        progress=0.5,
        message='driving',
        stamp_ns=123,
        position_error=0.2,
        yaw_error=0.1,
    ))

    assert payload == {
        'version': 1,
        'command_id': 'move-1',
        'state': 'running',
        'progress': 0.5,
        'message': 'driving',
        'stamp_ns': 123,
        'position_error': 0.2,
        'yaw_error': 0.1,
    }
