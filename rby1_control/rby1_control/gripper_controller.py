"""ROS-independent position trajectory policy for the gripper driver."""

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
    """Current backend-side view of the driver, target, and trajectory."""

    ready: Optional[bool]
    positions: Optional[GripperPair]
    target: Optional[GripperPair]
    motion_active: bool
    state_fresh: bool
    error: Optional[str]
    feedback_sequence: int
    command_reference: Optional[GripperPair]
    reference_velocity: GripperPair
    profile_active: bool
    profile_completed_at: Optional[float]
    sample_pending: bool
    planned_duration_sec: Optional[float]
    command_deadline: Optional[float]


@dataclass(frozen=True)
class _TrapezoidProfile:
    """One-dimensional, zero-end-velocity trapezoidal profile."""

    start: float
    goal: float
    started_at: float
    acceleration: float
    peak_velocity: float
    acceleration_time: float
    cruise_time: float
    duration: float

    @classmethod
    def create(
        cls,
        *,
        start: float,
        goal: float,
        started_at: float,
        maximum_velocity: float,
        acceleration: float,
    ) -> Optional['_TrapezoidProfile']:
        distance = abs(goal - start)
        if distance <= 0.0:
            return None

        velocity_time = maximum_velocity / acceleration
        velocity_distance = acceleration * velocity_time * velocity_time
        if distance <= velocity_distance:
            acceleration_time = math.sqrt(distance / acceleration)
            peak_velocity = acceleration * acceleration_time
            cruise_time = 0.0
        else:
            acceleration_time = velocity_time
            peak_velocity = maximum_velocity
            cruise_time = (
                distance - velocity_distance
            ) / maximum_velocity
        duration = 2.0 * acceleration_time + cruise_time
        return cls(
            start=start,
            goal=goal,
            started_at=started_at,
            acceleration=acceleration,
            peak_velocity=peak_velocity,
            acceleration_time=acceleration_time,
            cruise_time=cruise_time,
            duration=duration,
        )

    @property
    def ends_at(self) -> float:
        return self.started_at + self.duration

    def sample(self, now: float) -> Tuple[float, float, bool]:
        # Sampling absolute monotonic time means delayed timer callbacks skip
        # obsolete points rather than building a command backlog.
        sample_time = float(now)
        if sample_time >= self.ends_at:
            return self.goal, 0.0, True
        elapsed = min(max(sample_time - self.started_at, 0.0), self.duration)
        direction = 1.0 if self.goal >= self.start else -1.0
        distance = abs(self.goal - self.start)
        acceleration_distance = (
            0.5
            * self.acceleration
            * self.acceleration_time
            * self.acceleration_time
        )
        cruise_end = self.acceleration_time + self.cruise_time

        if elapsed < self.acceleration_time:
            travelled = 0.5 * self.acceleration * elapsed * elapsed
            velocity = self.acceleration * elapsed
        elif elapsed < cruise_end:
            cruise_elapsed = elapsed - self.acceleration_time
            travelled = (
                acceleration_distance
                + self.peak_velocity * cruise_elapsed
            )
            velocity = self.peak_velocity
        elif elapsed < self.duration:
            remaining = self.duration - elapsed
            travelled = (
                distance
                - 0.5 * self.acceleration * remaining * remaining
            )
            velocity = self.acceleration * remaining
        else:
            return self.goal, 0.0, True

        # Floating-point roundoff must never command past either endpoint.
        travelled = min(max(travelled, 0.0), distance)
        position = self.start + direction * travelled
        lower = min(self.start, self.goal)
        upper = max(self.start, self.goal)
        return min(max(position, lower), upper), direction * velocity, False


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


def _positive_float(value: float, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{label} must be positive')
    try:
        normalized = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must be positive') from exc
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f'{label} must be positive')
    return normalized


class GripperController:
    """Translate gripper requests into acceleration-limited position samples.

    The hardware driver owns calibration and Dynamixel I/O. This controller
    owns command validation, independent right/left profiles, feedback
    freshness, and completion timeout tracking. ``command`` and
    ``command_ratio`` keep returning the final target for compatibility;
    callers publish interpolated samples obtained via ``take_command_target``.
    """

    def __init__(
        self,
        *,
        open_ratio: float = 0.0,
        close_ratio: float = 1.0,
        state_timeout_sec: float = 1.0,
        command_timeout_sec: float = 5.0,
        goal_tolerance: float = 0.05,
        default_speed_ratio_per_sec: float = 0.5,
        max_speed_ratio_per_sec: float = 1.0,
        acceleration_ratio_per_sec2: float = 1.0,
        tracking_timeout_margin_sec: Optional[float] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        open_pair = _ratio_pair((open_ratio, open_ratio), 'open_ratio')
        close_pair = _ratio_pair((close_ratio, close_ratio), 'close_ratio')
        if open_pair[0] >= close_pair[0]:
            raise ValueError('open_ratio must be smaller than close_ratio')
        state_timeout = _positive_float(
            state_timeout_sec,
            'state_timeout_sec',
        )
        command_timeout = _positive_float(
            command_timeout_sec,
            'command_timeout_sec',
        )
        default_speed = _positive_float(
            default_speed_ratio_per_sec,
            'default_speed_ratio_per_sec',
        )
        maximum_speed = _positive_float(
            max_speed_ratio_per_sec,
            'max_speed_ratio_per_sec',
        )
        acceleration = _positive_float(
            acceleration_ratio_per_sec2,
            'acceleration_ratio_per_sec2',
        )
        if default_speed > maximum_speed:
            raise ValueError(
                'default_speed_ratio_per_sec must not exceed '
                'max_speed_ratio_per_sec'
            )
        if tracking_timeout_margin_sec is None:
            timeout_margin = None
        else:
            if isinstance(tracking_timeout_margin_sec, bool):
                raise ValueError(
                    'tracking_timeout_margin_sec must be nonnegative'
                )
            timeout_margin = float(tracking_timeout_margin_sec)
            if not math.isfinite(timeout_margin) or timeout_margin < 0.0:
                raise ValueError(
                    'tracking_timeout_margin_sec must be nonnegative'
                )
        if (
            not math.isfinite(float(goal_tolerance))
            or float(goal_tolerance) < 0.0
            or float(goal_tolerance) > 1.0
        ):
            raise ValueError('goal_tolerance must be within [0.0, 1.0]')

        self._open_ratio = open_pair[0]
        self._close_ratio = close_pair[0]
        self._state_timeout_sec = state_timeout
        self._command_timeout_sec = command_timeout
        self._goal_tolerance = float(goal_tolerance)
        self._default_speed_ratio_per_sec = default_speed
        self._max_speed_ratio_per_sec = maximum_speed
        self._acceleration_ratio_per_sec2 = acceleration
        self._tracking_timeout_margin_sec = timeout_margin
        self._clock = clock
        self._lock = threading.RLock()

        self._ready: Optional[bool] = None
        self._positions: Optional[GripperPair] = None
        self._state_updated_at: Optional[float] = None
        self._feedback_sequence = 0
        self._target: Optional[GripperPair] = None
        self._command_reference: Optional[GripperPair] = None
        self._reference_velocity: GripperPair = (0.0, 0.0)
        self._profiles: list[Optional[_TrapezoidProfile]] = [None, None]
        self._sample_pending = False
        self._profile_completed_at: Optional[float] = None
        self._planned_duration_sec: Optional[float] = None
        self._command_started_at: Optional[float] = None
        self._command_deadline: Optional[float] = None
        self._motion_active = False
        self._error: Optional[str] = None

    def update_ready(self, ready: bool) -> Optional[str]:
        """Store driver readiness and return a newly raised failure, if any."""

        now = self._clock()
        with self._lock:
            self._ready = bool(ready)
            if self._ready or not self._motion_active:
                return None
            return self._fail_locked(
                'gripper driver became not ready during a command',
                now,
            )

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

            # Feedback at the final target supersedes any remaining planned
            # samples. This preserves the existing immediate-completion
            # behavior when hardware is already at the requested position.
            self._clear_profiles_locked()
            self._command_reference = self._target
            self._sample_pending = True
            self._profile_completed_at = now
            self._motion_active = False
            self._command_started_at = None
            self._command_deadline = None
            self._error = None
            return True

    def command(
        self,
        action: str,
        side: str = 'both',
        speed: Optional[float] = None,
    ) -> GripperPair:
        """Create one open/close target in right-left order."""

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
        return self._command_ratio_locked_entry(
            ratio,
            normalized_side,
            self._normalize_speed(speed),
        )

    def command_ratio(
        self,
        ratio: float,
        side: str = 'both',
        speed: Optional[float] = None,
    ) -> GripperPair:
        """Create one normalized position target."""

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
            self._normalize_speed(speed),
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
                    'gripper driver became not ready during settle',
                    now,
                )
                raise GripperCommandError(str(self._error))
            if not self._state_fresh_locked(now):
                self._fail_locked(
                    'gripper state became stale during settle',
                    now,
                )
                raise GripperCommandError(str(self._error))
            if self._feedback_sequence <= marker:
                self._fail_locked(
                    'no new gripper feedback arrived after the command',
                    now,
                )
                raise GripperCommandError(str(self._error))
            if self._error is not None:
                raise GripperCommandError(self._error)

            # The owner starts settle only after profile_active becomes false.
            # Retain the final target so the driver continues position holding.
            self._clear_profiles_locked()
            if self._target is not None:
                self._command_reference = self._target
                self._sample_pending = True
            self._profile_completed_at = now
            self._motion_active = False
            self._command_started_at = None
            self._command_deadline = None

    def begin_settle(self, duration_sec: float) -> float:
        """Start post-profile settling and return its tracking deadline.

        Position timeout tracking remains active during settling, as does the
        stricter feedback-freshness check. The deadline includes the configured
        tracking margin so a valid slow trajectory cannot time out merely
        because settling follows it.
        """

        if isinstance(duration_sec, bool):
            raise GripperCommandError(
                'gripper settle duration must be finite and nonnegative'
            )
        try:
            duration = float(duration_sec)
        except (TypeError, ValueError) as exc:
            raise GripperCommandError(
                'gripper settle duration must be finite and nonnegative'
            ) from exc
        if not math.isfinite(duration) or duration < 0.0:
            raise GripperCommandError(
                'gripper settle duration must be finite and nonnegative'
            )

        now = self._clock()
        with self._lock:
            if self._profile_active_locked():
                raise GripperCommandError(
                    'gripper trajectory must reach its endpoint before settle'
                )
            if self._profile_completed_at is None or self._target is None:
                raise GripperCommandError(
                    'no completed gripper trajectory is available to settle'
                )
            if self._ready is not True:
                self._fail_locked(
                    'gripper driver became not ready before settle',
                    now,
                )
                raise GripperCommandError(str(self._error))
            if not self._state_fresh_locked(now):
                self._fail_locked(
                    'gripper state became stale before settle',
                    now,
                )
                raise GripperCommandError(str(self._error))
            if self._error is not None:
                raise GripperCommandError(self._error)

            margin = (
                self._tracking_timeout_margin_sec
                if self._tracking_timeout_margin_sec is not None
                else self._command_timeout_sec
            )
            settle_deadline = now + duration + margin
            if self._command_deadline is None:
                self._command_deadline = settle_deadline
            else:
                self._command_deadline = max(
                    self._command_deadline,
                    settle_deadline,
                )
            if self._command_started_at is None:
                self._command_started_at = now
            self._motion_active = True
            return self._command_deadline

    def cancel_and_hold(self) -> Optional[GripperPair]:
        """Cancel all profiles and hold the latest fresh measured position."""

        now = self._clock()
        with self._lock:
            hold_target = (
                self._positions
                if self._state_fresh_locked(now)
                else None
            )
            self._clear_profiles_locked()
            self._sample_pending = False
            if hold_target is not None:
                self._target = hold_target
                self._command_reference = hold_target
                self._sample_pending = True
            else:
                self._target = None
                self._command_reference = None
            self._profile_completed_at = None
            self._planned_duration_sec = None
            self._motion_active = False
            self._command_started_at = None
            self._command_deadline = None
            return hold_target

    def tick(self) -> Optional[str]:
        """Advance to the latest trajectory sample and check for failure.

        Only the sample at the current monotonic time is retained. Call
        :meth:`take_command_target` to consume it for publication.
        """

        now = self._clock()
        with self._lock:
            failure = self._expire_locked(now)
            if failure is not None:
                return failure
            self._advance_profiles_locked(now)
            return None

    def take_command_target(self) -> Optional[GripperPair]:
        """Consume the latest position reference, dropping obsolete samples."""

        with self._lock:
            if not self._sample_pending or self._command_reference is None:
                return None
            self._sample_pending = False
            return self._command_reference

    # A more explicit alias for callers that name timer outputs as samples.
    take_command_sample = take_command_target

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
                command_reference=self._command_reference,
                reference_velocity=self._reference_velocity,
                profile_active=self._profile_active_locked(),
                profile_completed_at=self._profile_completed_at,
                sample_pending=self._sample_pending,
                planned_duration_sec=self._planned_duration_sec,
                command_deadline=self._command_deadline,
            )

    def _command_ratio_locked_entry(
        self,
        ratio: float,
        normalized_side: str,
        speed: float,
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

            # First bring any in-flight side to the newest reference. A new
            # command may use an idle side, but never replaces an active
            # trajectory on any selected side. Sampling first deliberately
            # permits a command whose previous profile ends exactly now.
            self._advance_profiles_locked(now)
            selected = (
                (0, 1)
                if normalized_side == 'both'
                else (0,) if normalized_side == 'right' else (1,)
            )
            active_selected = [
                ('right', 'left')[index]
                for index in selected
                if self._profiles[index] is not None
            ]
            if active_selected:
                raise GripperCommandError(
                    'gripper command rejected: selected side already has '
                    'an active trajectory: '
                    + ', '.join(active_selected)
                )

            if self._command_reference is None:
                self._command_reference = self._positions

            references = list(self._command_reference)
            targets = list(
                self._target
                if self._target is not None
                else self._command_reference
            )

            for index in selected:
                # While a side is moving, continue from its latest commanded
                # reference. Otherwise use fresh feedback as the new start.
                if self._profiles[index] is None:
                    references[index] = self._positions[index]
                targets[index] = ratio
                self._profiles[index] = _TrapezoidProfile.create(
                    start=references[index],
                    goal=ratio,
                    started_at=now,
                    maximum_velocity=speed,
                    acceleration=self._acceleration_ratio_per_sec2,
                )

            self._command_reference = (references[0], references[1])
            self._reference_velocity = tuple(
                0.0 if index in selected else self._reference_velocity[index]
                for index in range(2)
            )  # type: ignore[assignment]
            self._target = (targets[0], targets[1])
            self._sample_pending = True
            self._error = None
            self._profile_completed_at = None
            self._command_started_at = now
            self._planned_duration_sec = max(
                (
                    max(profile.ends_at - now, 0.0)
                    for profile in self._profiles
                    if profile is not None
                ),
                default=0.0,
            )
            if self._tracking_timeout_margin_sec is None:
                timeout = self._command_timeout_sec
            else:
                timeout = max(
                    self._command_timeout_sec,
                    self._planned_duration_sec
                    + self._tracking_timeout_margin_sec,
                )
            self._command_deadline = now + timeout

            if self._profile_active_locked():
                self._motion_active = True
            elif self._goal_reached(self._positions, self._target):
                self._motion_active = False
                self._profile_completed_at = now
                self._command_started_at = None
                self._command_deadline = None
            else:
                # A zero-distance command reference can still await feedback
                # for the final target (for example, after measured noise).
                self._motion_active = True
                self._profile_completed_at = now
            return self._target

    def _normalize_speed(self, speed: Optional[float]) -> float:
        if speed is None:
            return self._default_speed_ratio_per_sec
        if isinstance(speed, bool):
            raise GripperCommandError(
                'gripper speed must be positive and not exceed the maximum'
            )
        try:
            normalized = float(speed)
        except (TypeError, ValueError) as exc:
            raise GripperCommandError(
                'gripper speed must be positive and not exceed the maximum'
            ) from exc
        if (
            not math.isfinite(normalized)
            or normalized <= 0.0
            or normalized > self._max_speed_ratio_per_sec
        ):
            raise GripperCommandError(
                'gripper speed must be positive and not exceed '
                f'{self._max_speed_ratio_per_sec:g} ratio/s'
            )
        return normalized

    def _advance_profiles_locked(self, now: float) -> None:
        if (
            self._command_reference is None
            or not self._profile_active_locked()
        ):
            return

        references = list(self._command_reference)
        velocities = list(self._reference_velocity)
        changed = False
        for index, profile in enumerate(self._profiles):
            if profile is None:
                continue
            position, velocity, complete = profile.sample(now)
            if position != references[index] or velocity != velocities[index]:
                changed = True
            references[index] = position
            velocities[index] = velocity
            if complete:
                self._profiles[index] = None
                references[index] = profile.goal
                velocities[index] = 0.0
                changed = True

        self._command_reference = (references[0], references[1])
        self._reference_velocity = (velocities[0], velocities[1])
        if changed:
            self._sample_pending = True
        if not self._profile_active_locked():
            self._profile_completed_at = now
            # Always make the exact endpoint available for publication.
            self._sample_pending = True

    def _clear_profiles_locked(self) -> None:
        self._profiles = [None, None]
        self._reference_velocity = (0.0, 0.0)

    def _profile_active_locked(self) -> bool:
        return any(profile is not None for profile in self._profiles)

    def _fail_locked(self, message: str, now: float) -> str:
        hold_target = (
            self._positions
            if self._state_fresh_locked(now)
            else None
        )
        self._clear_profiles_locked()
        self._target = hold_target
        self._command_reference = hold_target
        self._sample_pending = hold_target is not None
        self._profile_completed_at = None
        self._planned_duration_sec = None
        self._motion_active = False
        self._command_started_at = None
        self._command_deadline = None
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
                'gripper state became stale during a command',
                now,
            )
        if self._command_deadline is None or now <= self._command_deadline:
            return None
        return self._fail_locked(
            'gripper command timed out before reaching its target',
            now,
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
