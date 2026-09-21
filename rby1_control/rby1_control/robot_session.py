"""Lifecycle boundary for robot connectivity and state aggregation."""
from __future__ import annotations

from typing import Callable, Optional

from .power_servo_state_adaptor import PowerServoStateAdaptor
from .state_manager import RobotStatus, StateManager


class RobotSession:
    """Combine driver feedback and asynchronous SDK feedback for one robot."""

    def __init__(
        self,
        *,
        state_manager: StateManager,
        robot_address: str,
        robot_model: str,
        power_pattern: str,
        servo_pattern: str,
        poll_period_sec: float,
        on_event: Callable[[str, str], None],
        adaptor_factory=PowerServoStateAdaptor,
    ) -> None:
        self.state_manager = state_manager
        self._on_event = on_event
        address = str(robot_address).strip()
        self._adaptor: Optional[PowerServoStateAdaptor] = None
        if address:
            self._adaptor = adaptor_factory(
                address=address,
                model=robot_model,
                power_pattern=power_pattern,
                servo_pattern=servo_pattern,
                poll_period_sec=poll_period_sec,
                on_state=self._power_servo_state,
                on_error=lambda detail: self._on_event('warning', detail),
            )
            self._on_event(
                'info',
                f'Power/Servo SDK session configured for {address}.',
            )
        else:
            self._on_event(
                'warning',
                'robot_address is empty; SDK Power/Servo feedback is disabled.',
            )

    def poll(self) -> None:
        if self._adaptor is not None:
            self._adaptor.poll()

    def close(self) -> None:
        if self._adaptor is not None:
            self._adaptor.close()

    def update_driver_state(self, message) -> dict:
        return self.state_manager.update_driver_state(
            control_state=int(message.control_manager_state),
            stream_enabled=bool(message.robot_stream_state),
            emo_active=bool(message.emo_state),
            collision_active=bool(message.collision),
        )

    def snapshot(self) -> RobotStatus:
        return self.state_manager.snapshot()

    def _power_servo_state(
        self,
        power_enabled: bool,
        servo_enabled: bool,
    ) -> None:
        changes = self.state_manager.update_power_servo(
            power_enabled,
            servo_enabled,
        )
        for field, (_previous, value) in changes.items():
            label = 'Power' if field == 'power_enabled' else 'Servo'
            self._on_event(
                'info',
                f'{label} feedback changed to {"ON" if value else "OFF"}.',
            )

