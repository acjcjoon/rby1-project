"""ROS topic client used by planner Tasks for relative base navigation."""

from __future__ import annotations

import json
import math
import threading
import time
import uuid

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from .navigation_protocol import (
    NavigationCommandState,
    NavigationCommandStatus,
)


PROTOCOL_VERSION = 1


def _finite(value: object, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{label} must be a finite number')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must be a finite number') from exc
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = 'positive ' if positive else ''
        raise ValueError(f'{label} must be a {qualifier}finite number')
    return result


class NavigationClient:
    """Publish move/cancel commands and cache correlated navigation states."""

    MAX_CACHED_COMMANDS = 100

    def __init__(
        self,
        node,
        *,
        command_topic: str = '/rby1/navigation/command',
        state_topic: str = '/rby1/navigation/state',
    ) -> None:
        command_topic = str(command_topic).strip()
        state_topic = str(state_topic).strip()
        if not command_topic or not state_topic:
            raise ValueError('navigation topics must not be empty')

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.command_topic = command_topic
        self.state_topic = state_topic
        self._node = node
        self._lock = threading.RLock()
        self._states: dict[str, NavigationCommandState] = {}
        self._received_at: dict[str, float] = {}
        self._publisher = node.create_publisher(String, command_topic, qos)
        self._subscription = node.create_subscription(
            String,
            state_topic,
            self._state_callback,
            qos,
        )

    def send_move_to(
        self,
        x: float,
        y: float,
        yaw: float,
        timeout_sec: float,
    ) -> str:
        """Send one body-relative SE(2) target; yaw uses radians."""
        x = _finite(x, 'x')
        y = _finite(y, 'y')
        yaw = _finite(yaw, 'yaw')
        timeout_sec = _finite(timeout_sec, 'timeout_sec', positive=True)
        command_id = uuid.uuid4().hex
        with self._lock:
            self._states[command_id] = NavigationCommandState(
                NavigationCommandStatus.PENDING,
                0.0,
                'waiting for navigation acknowledgement',
            )
            self._received_at[command_id] = time.monotonic()
            self._prune_locked()
        self._publish({
            'version': PROTOCOL_VERSION,
            'command': 'move_to',
            'command_id': command_id,
            'x': x,
            'y': y,
            'yaw': yaw,
            'timeout_sec': timeout_sec,
        })
        return command_id

    def cancel(self, command_id: str) -> None:
        command_id = str(command_id).strip()
        if not command_id:
            raise ValueError('command_id must not be empty')
        self._publish({
            'version': PROTOCOL_VERSION,
            'command': 'cancel',
            'command_id': command_id,
        })

    def poll(self, command_id: str) -> NavigationCommandState:
        with self._lock:
            state = self._states.get(str(command_id))
        if state is None:
            raise KeyError(f'unknown navigation command: {command_id}')
        return state

    def _publish(self, payload: dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(
            payload,
            allow_nan=False,
            separators=(',', ':'),
            sort_keys=True,
        )
        self._publisher.publish(message)

    def _state_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('state payload must be an object')
            if payload.get('version') != PROTOCOL_VERSION:
                raise ValueError('unsupported navigation state version')
            command_id = str(payload.get('command_id', '')).strip()
            if not command_id:
                # Idle heartbeats intentionally have no command ID.
                return
            status = NavigationCommandStatus(payload.get('state'))
            progress = _finite(payload.get('progress', 0.0), 'progress')
            if not 0.0 <= progress <= 1.0:
                raise ValueError('progress must be in [0, 1]')
            detail = str(payload.get('message', ''))
            state = NavigationCommandState(status, progress, detail)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._node.get_logger().warning(
                f'Ignored invalid navigation state: {exc}'
            )
            return

        with self._lock:
            current = self._states.get(command_id)
            if current is None or current.status.terminal:
                return
            self._states[command_id] = state
            self._received_at[command_id] = time.monotonic()
            self._prune_locked()

    def _prune_locked(self) -> None:
        overflow = len(self._states) - self.MAX_CACHED_COMMANDS
        if overflow <= 0:
            return
        oldest = sorted(
            self._received_at,
            key=self._received_at.get,
        )[:overflow]
        for command_id in oldest:
            self._states.pop(command_id, None)
            self._received_at.pop(command_id, None)


__all__ = ['NavigationClient', 'PROTOCOL_VERSION']
