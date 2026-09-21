"""Mobile-base command arbitration and final ``cmd_vel`` ownership."""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional, Tuple

from geometry_msgs.msg import Twist

from .backend_contract import VelocityCommand


class MobileBaseController:
    """Merge raw navigation/manual inputs into the only final velocity topic."""

    def __init__(
        self,
        node,
        *,
        cmd_raw_topic: str,
        cmd_vel_topic: str,
        command_timeout_sec: float,
        publish_rate_hz: float,
        publish_zero_when_idle: bool,
        motion_guard: Callable[[], Optional[str]],
        on_event: Callable[[str, str], None],
        clock=time.monotonic,
    ) -> None:
        self._node = node
        self._clock = clock
        self._motion_guard = motion_guard
        self._on_event = on_event
        self.command_timeout_sec = float(command_timeout_sec)
        self.publish_zero_when_idle = bool(publish_zero_when_idle)
        self._lock = threading.RLock()
        self._command = VelocityCommand()
        self._last_command_update = self._clock()
        self._last_published = VelocityCommand()
        self._last_logged_command: Optional[VelocityCommand] = None
        self._last_guard_reason: Optional[str] = None

        self.publisher = node.create_publisher(Twist, cmd_vel_topic, 10)
        self.raw_subscription = node.create_subscription(
            Twist,
            cmd_raw_topic,
            self._raw_callback,
            10,
        )
        self.timer = node.create_timer(
            1.0 / float(publish_rate_hz),
            self.publish_cycle,
        )

    @property
    def cmd_vel_topic(self) -> str:
        return self.publisher.topic_name

    @property
    def cmd_raw_topic(self) -> str:
        return self.raw_subscription.topic_name

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        values = (float(vx), float(vy), float(wz))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('velocity command must contain finite values')
        with self._lock:
            self._command = VelocityCommand(*values)
            self._last_command_update = self._clock()

    def stop(self, *, publish_immediately: bool = True) -> None:
        with self._lock:
            self._command = VelocityCommand()
            self._last_command_update = self._clock()
        if publish_immediately:
            self.publish(VelocityCommand())

    def current_command(self) -> Tuple[VelocityCommand, bool]:
        with self._lock:
            command = self._command
            age = self._clock() - self._last_command_update
        return command, age > self.command_timeout_sec

    def publish_cycle(self) -> None:
        command, stale = self.current_command()
        if stale:
            command = VelocityCommand()
        reason = None if command.stopped else self._motion_guard()
        if reason is not None:
            command = VelocityCommand()
            if reason != self._last_guard_reason:
                self._on_event('error', f'Mobile-base command blocked: {reason}.')
            self._last_guard_reason = reason
        else:
            self._last_guard_reason = None
        if (
            self.publish_zero_when_idle
            or not command.stopped
            or not self._last_published.stopped
        ):
            self.publish(command)

    def publish(self, command: VelocityCommand) -> None:
        message = Twist()
        message.linear.x = command.vx
        message.linear.y = command.vy
        message.angular.z = command.wz
        self.publisher.publish(message)
        self._last_published = command
        if command != self._last_logged_command:
            self._last_logged_command = command
            self._on_event(
                'info',
                'cmd_vel: '
                f'vx={command.vx:+.3f}, '
                f'vy={command.vy:+.3f}, '
                f'wz={command.wz:+.3f}',
            )

    def subscriber_count(self) -> int:
        return self._node.count_subscribers(self.publisher.topic_name)

    def _raw_callback(self, message: Twist) -> None:
        try:
            self.set_velocity(
                message.linear.x,
                message.linear.y,
                message.angular.z,
            )
        except ValueError as exc:
            self._on_event('error', f'Invalid cmd_raw rejected: {exc}')

