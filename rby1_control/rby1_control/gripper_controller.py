"""ROS-independent open/close policy for the gripper driver."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Optional, Sequence, Tuple


GripperPair = Tuple[float, float]


class GripperCommandError(RuntimeError):
    """Raised when a high-level gripper command is unsafe or invalid."""


@dataclass(frozen=True)
class GripperControllerStatus:
    """Current backend-side view of the gripper driver and active goal."""

    ready: Optional[bool]
    positions: Optional[GripperPair]
    target: Optional[GripperPair]
    motion_active: bool
    state_fresh: bool
    error: Optional[str]
    feedback_sequence: int


def _ratio_pair(values: Sequence[float], label: str) -> GripperPair:
    if len(values) != 2:
        raise ValueError(f'{label} must contain [right, left]')
    if any(isinstance(value, bool) for value in values):
        raise ValueError(f'{label} must contain numbers')
    pair = (float(values[0]), float(values[1]))
    if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in pair):
        raise ValueError(
            f'{label} values must be finite and within [0.0, 1.0]'
        )
    return pair


class GripperController:
    """Translate open/close requests into full ``[right, left]`` targets.

    The hardware driver owns calibration and Dynamixel I/O. This controller
    owns the application policy: command validation, partial-side target
    merging, feedback freshness, and completion timeout tracking.
    """

    def __init__(
        self,
        *,
        open_ratio: float = 0.0,
        close_ratio: float = 1.0,
        state_timeout_sec: float = 1.0,
        command_timeout_sec: float = 5.0,
        goal_tolerance: float = 0.05,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        open_pair = _ratio_pair((open_ratio, open_ratio), 'open_ratio')
        close_pair = _ratio_pair((close_ratio, close_ratio), 'close_ratio')
        if open_pair[0] >= close_pair[0]:
            raise ValueError('open_ratio must be smaller than close_ratio')
        for value, label in (
            (state_timeout_sec, 'state_timeout_sec'),
            (command_timeout_sec, 'command_timeout_sec'),
        ):
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f'{label} must be positive')
        if (
            not math.isfinite(float(goal_tolerance))
            or float(goal_tolerance) < 0.0
            or float(goal_tolerance) > 1.0
        ):
            raise ValueError('goal_tolerance must be within [0.0, 1.0]')

        self._open_ratio = open_pair[0]
        self._close_ratio = close_pair[0]
        self._state_timeout_sec = float(state_timeout_sec)
        self._command_timeout_sec = float(command_timeout_sec)
        self._goal_tolerance = float(goal_tolerance)
        self._clock = clock
        self._lock = threading.RLock()

        self._ready: Optional[bool] = None
        self._positions: Optional[GripperPair] = None
        self._state_updated_at: Optional[float] = None
        self._feedback_sequence = 0
        self._target: Optional[GripperPair] = None
        self._command_started_at: Optional[float] = None
        self._motion_active = False
        self._error: Optional[str] = None

    def update_ready(self, ready: bool) -> Optional[str]:
        """Store driver readiness and return a newly raised failure, if any."""

        with self._lock:
            self._ready = bool(ready)
            if self._ready or not self._motion_active:
                return None
            self._motion_active = False
            self._error = 'gripper driver became not ready during a command'
            return self._error

    def update_state(self, positions: Sequence[float]) -> bool:
        """Store measured close ratios and report goal completion once."""

        measured = _ratio_pair(positions, 'gripper state')
        now = self._clock()
        with self._lock:
            self._positions = measured
            self._state_updated_at = now
            self._feedback_sequence += 1
            self._expire_locked(now)
            if not self._motion_active or self._target is None:
                return False
            if not self._goal_reached(measured, self._target):
                return False
            self._motion_active = False
            self._command_started_at = None
            self._error = None
            return True

    def command(self, action: str, side: str = 'both') -> GripperPair:
        """Create and track one open/close target in right-left order."""

        normalized_action = str(action).strip().lower()
        normalized_side = str(side).strip().lower()
        if normalized_action not in ('open', 'close'):
            raise GripperCommandError('action must be open or close')
        if normalized_side not in ('right', 'left', 'both'):
            raise GripperCommandError('side must be right, left, or both')
        ratio = (
            self._open_ratio
            if normalized_action == 'open'
            else self._close_ratio
        )
        return self._command_ratio_locked_entry(ratio, normalized_side)

    def command_ratio(
        self,
        ratio: float,
        side: str = 'both',
    ) -> GripperPair:
        """Create and track one normalized position target."""

        if isinstance(ratio, bool):
            raise GripperCommandError('gripper ratio must be within [0, 1]')
        try:
            normalized_ratio = float(ratio)
        except (TypeError, ValueError) as exc:
            raise GripperCommandError(
                'gripper ratio must be within [0, 1]'
            ) from exc
        if (
            not math.isfinite(normalized_ratio)
            or not 0.0 <= normalized_ratio <= 1.0
        ):
            raise GripperCommandError('gripper ratio must be within [0, 1]')

        normalized_side = str(side).strip().lower()
        if normalized_side not in ('right', 'left', 'both'):
            raise GripperCommandError('side must be right, left, or both')
        return self._command_ratio_locked_entry(
            normalized_ratio,
            normalized_side,
        )

    def complete_after_settle(self, feedback_after: int) -> None:
        """Finish Task waiting while preserving the active hardware target."""

        if isinstance(feedback_after, bool):
            raise GripperCommandError(
                'gripper feedback marker must be a nonnegative integer'
            )
        marker = int(feedback_after)
        if marker < 0:
            raise GripperCommandError(
                'gripper feedback marker must be a nonnegative integer'
            )

        now = self._clock()
        with self._lock:
            if self._ready is not True:
                self._fail_locked(
                    'gripper driver became not ready during settle'
                )
                raise GripperCommandError(str(self._error))
            if not self._state_fresh_locked(now):
                self._fail_locked(
                    'gripper state became stale during settle'
                )
                raise GripperCommandError(str(self._error))
            if self._feedback_sequence <= marker:
                self._fail_locked(
                    'no new gripper feedback arrived after the command'
                )
                raise GripperCommandError(str(self._error))
            if self._error is not None:
                raise GripperCommandError(self._error)

            # Completion here means the Scenario may advance. The target is
            # deliberately retained so the driver continues position holding.
            self._motion_active = False
            self._command_started_at = None

    def cancel_and_hold(self) -> Optional[GripperPair]:
        """Stop completion tracking and hold the latest fresh position."""

        now = self._clock()
        with self._lock:
            hold_target = (
                self._positions
                if self._state_fresh_locked(now)
                else None
            )
            if hold_target is not None:
                self._target = hold_target
            self._motion_active = False
            self._command_started_at = None
            return hold_target

    def tick(self) -> Optional[str]:
        """Advance timeout tracking and return a newly raised failure."""

        with self._lock:
            return self._expire_locked(self._clock())

    def status(self) -> GripperControllerStatus:
        """Return one immutable view for the backend transport snapshot."""

        now = self._clock()
        with self._lock:
            return GripperControllerStatus(
                ready=self._ready,
                positions=self._positions,
                target=self._target,
                motion_active=self._motion_active,
                state_fresh=self._state_fresh_locked(now),
                error=self._error,
                feedback_sequence=self._feedback_sequence,
            )

    def _command_ratio_locked_entry(
        self,
        ratio: float,
        normalized_side: str,
    ) -> GripperPair:
        now = self._clock()
        with self._lock:
            self._expire_locked(now)
            if self._ready is not True:
                raise GripperCommandError(
                    'gripper driver is not ready; calibrate and enable torque'
                )
            if not self._state_fresh_locked(now):
                raise GripperCommandError(
                    'gripper state is unavailable or stale; check the driver'
                )
            assert self._positions is not None

            if normalized_side == 'both':
                target = (ratio, ratio)
            else:
                # Preserve an in-flight target on the opposite hand. This
                # prevents a quick second command from undoing the first one.
                base = (
                    self._target
                    if self._motion_active and self._target is not None
                    else self._positions
                )
                mutable = [base[0], base[1]]
                mutable[0 if normalized_side == 'right' else 1] = ratio
                target = (mutable[0], mutable[1])

            self._target = target
            self._error = None
            if self._goal_reached(self._positions, target):
                self._motion_active = False
                self._command_started_at = None
            else:
                self._motion_active = True
                self._command_started_at = now
            return target

    def _fail_locked(self, message: str) -> str:
        self._motion_active = False
        self._command_started_at = None
        self._error = str(message)
        return self._error

    def _state_fresh_locked(self, now: float) -> bool:
        return (
            self._positions is not None
            and self._state_updated_at is not None
            and now - self._state_updated_at <= self._state_timeout_sec
        )

    def _expire_locked(self, now: float) -> Optional[str]:
        if not self._motion_active or self._command_started_at is None:
            return None
        if not self._state_fresh_locked(now):
            return self._fail_locked(
                'gripper state became stale during a command'
            )
        if now - self._command_started_at <= self._command_timeout_sec:
            return None
        return self._fail_locked(
            'gripper command timed out before reaching its target'
        )

    def _goal_reached(
        self,
        measured: GripperPair,
        target: GripperPair,
    ) -> bool:
        return all(
            abs(actual - expected) <= self._goal_tolerance
            for actual, expected in zip(measured, target)
        )
