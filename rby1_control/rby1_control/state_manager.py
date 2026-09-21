"""Thread-safe robot-state ownership for the control node."""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class RobotStatus:
    """Immutable snapshot assembled from driver and SDK feedback."""

    captured_at: float
    robot_state_updated_at: Optional[float]
    control_state: Optional[int]
    stream_enabled: Optional[bool]
    emo_active: Optional[bool]
    collision_active: Optional[bool]
    power_enabled: Optional[bool]
    power_updated_at: Optional[float]
    servo_enabled: Optional[bool]
    servo_updated_at: Optional[float]


class StateManager:
    """Own robot status independently of ROS callbacks and SDK threads."""

    def __init__(self, *, clock=time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.RLock()
        self._robot_state_updated_at: Optional[float] = None
        self._control_state: Optional[int] = None
        self._stream_enabled: Optional[bool] = None
        self._emo_active: Optional[bool] = None
        self._collision_active: Optional[bool] = None
        self._power_enabled: Optional[bool] = None
        self._power_updated_at: Optional[float] = None
        self._servo_enabled: Optional[bool] = None
        self._servo_updated_at: Optional[float] = None

    def update_driver_state(
        self,
        *,
        control_state: int,
        stream_enabled: bool,
        emo_active: bool,
        collision_active: bool,
    ) -> Dict[str, Tuple[object, object]]:
        """Store one driver state sample and return changed fields."""

        values = {
            'control_state': int(control_state),
            'stream_enabled': bool(stream_enabled),
            'emo_active': bool(emo_active),
            'collision_active': bool(collision_active),
        }
        changes: Dict[str, Tuple[object, object]] = {}
        with self._lock:
            for field, value in values.items():
                private_name = f'_{field}'
                previous = getattr(self, private_name)
                if previous != value:
                    changes[field] = (previous, value)
                setattr(self, private_name, value)
            self._robot_state_updated_at = self._clock()
        return changes

    def update_power_servo(
        self,
        power_enabled: bool,
        servo_enabled: bool,
    ) -> Dict[str, Tuple[object, object]]:
        """Store one asynchronous SDK query result."""

        now = self._clock()
        changes: Dict[str, Tuple[object, object]] = {}
        with self._lock:
            power = bool(power_enabled)
            servo = bool(servo_enabled)
            if self._power_enabled != power:
                changes['power_enabled'] = (self._power_enabled, power)
            if self._servo_enabled != servo:
                changes['servo_enabled'] = (self._servo_enabled, servo)
            self._power_enabled = power
            self._servo_enabled = servo
            self._power_updated_at = now
            self._servo_updated_at = now
        return changes

    def snapshot(self) -> RobotStatus:
        with self._lock:
            return RobotStatus(
                captured_at=self._clock(),
                robot_state_updated_at=self._robot_state_updated_at,
                control_state=self._control_state,
                stream_enabled=self._stream_enabled,
                emo_active=self._emo_active,
                collision_active=self._collision_active,
                power_enabled=self._power_enabled,
                power_updated_at=self._power_updated_at,
                servo_enabled=self._servo_enabled,
                servo_updated_at=self._servo_updated_at,
            )

    def safety_reason(
        self,
        *,
        maximum_age_sec: float,
        now: Optional[float] = None,
    ) -> Optional[str]:
        """Return a motion-safety rejection reason or ``None``."""

        status = self.snapshot()
        captured_at = status.captured_at if now is None else float(now)
        updated_at = status.robot_state_updated_at
        if updated_at is None:
            return 'robot state is unavailable or stale'
        age = captured_at - updated_at
        if (
            not math.isfinite(age)
            or age < 0.0
            or age > float(maximum_age_sec)
        ):
            return 'robot state is unavailable or stale'
        if status.emo_active is not False:
            return 'EMO state is active or unknown'
        if status.collision_active is not False:
            return 'collision state is active or unknown'
        if status.control_state not in (2, 3):
            return 'Control Manager is not ENABLE/EXECUTING'
        return None

