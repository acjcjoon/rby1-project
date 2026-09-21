"""Validation and serialization for planner/navigation JSON messages."""

from dataclasses import dataclass
import json
import math
import re


PROTOCOL_VERSION = 1
COMMAND_ID_PATTERN = re.compile(r'^[A-Za-z0-9_.:-]{1,128}$')
TERMINAL_STATES = {'succeeded', 'rejected', 'failed', 'canceled'}
VALID_STATES = {'idle', 'accepted', 'running'} | TERMINAL_STATES


class ProtocolError(ValueError):
    """Raised when a command or state violates the topic contract."""


@dataclass(frozen=True)
class NavigationCommand:
    action: str
    command_id: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    timeout_sec: float = 0.0


def _finite(value, label, *, positive=False):
    if isinstance(value, bool):
        raise ProtocolError(f'{label} must be a finite number')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f'{label} must be a finite number') from exc
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = 'positive ' if positive else ''
        raise ProtocolError(f'{label} must be a {qualifier}finite number')
    return result


def parse_command(data, *, default_timeout_sec, max_timeout_sec):
    try:
        payload = json.loads(data)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ProtocolError('command must be valid JSON') from exc
    if not isinstance(payload, dict):
        raise ProtocolError('command payload must be an object')
    if payload.get('version') != PROTOCOL_VERSION:
        raise ProtocolError('unsupported command protocol version')

    command_id = str(payload.get('command_id', '')).strip()
    if not COMMAND_ID_PATTERN.fullmatch(command_id):
        raise ProtocolError('command_id has an invalid format')
    action = str(payload.get('command', '')).strip().lower()
    if action == 'cancel':
        return NavigationCommand(action='cancel', command_id=command_id)
    if action != 'move_to':
        raise ProtocolError(f'unsupported navigation command: {action!r}')

    timeout_sec = _finite(
        payload.get('timeout_sec', default_timeout_sec),
        'timeout_sec',
        positive=True,
    )
    if timeout_sec > max_timeout_sec:
        raise ProtocolError(
            f'timeout_sec exceeds maximum {max_timeout_sec:.3f}'
        )
    return NavigationCommand(
        action='move_to',
        command_id=command_id,
        x=_finite(payload.get('x'), 'x'),
        y=_finite(payload.get('y'), 'y'),
        yaw=_finite(payload.get('yaw'), 'yaw'),
        timeout_sec=timeout_sec,
    )


def encode_state(
    *,
    command_id,
    state,
    progress,
    message,
    stamp_ns,
    position_error=None,
    yaw_error=None,
):
    state = str(state).strip().lower()
    if state not in VALID_STATES:
        raise ProtocolError(f'invalid navigation state: {state!r}')
    command_id = str(command_id).strip()
    if command_id and not COMMAND_ID_PATTERN.fullmatch(command_id):
        raise ProtocolError('command_id has an invalid format')
    progress = _finite(progress, 'progress')
    if not 0.0 <= progress <= 1.0:
        raise ProtocolError('progress must be in [0, 1]')

    payload = {
        'version': PROTOCOL_VERSION,
        'command_id': command_id,
        'state': state,
        'progress': progress,
        'message': str(message),
        'stamp_ns': int(stamp_ns),
    }
    if position_error is not None:
        payload['position_error'] = _finite(
            position_error,
            'position_error',
        )
    if yaw_error is not None:
        payload['yaw_error'] = _finite(yaw_error, 'yaw_error')
    return json.dumps(
        payload,
        allow_nan=False,
        separators=(',', ':'),
        sort_keys=True,
    )
