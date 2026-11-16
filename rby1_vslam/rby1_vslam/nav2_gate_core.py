"""ROS-independent watchdog for Nav2 velocity commands and cancel barriers."""

from dataclasses import dataclass, field
import math


@dataclass
class GateLimits:
    cmd_timeout_sec: float = 0.25
    odom_timeout_sec: float = 0.5
    pose_timeout_sec: float = 0.5
    localization_timeout_sec: float = 0.75
    depth_timeout_sec: float = 0.5
    slice_timeout_sec: float = 1.0
    integrated_depth_timeout_sec: float = 1.0
    future_tolerance_sec: float = 0.05
    max_linear_velocity: float = 0.15
    max_angular_velocity: float = 0.25
    max_pose_jump_m: float = 0.5
    max_pose_jump_rad: float = 0.8

    def __post_init__(self):
        if any(not math.isfinite(float(v)) or v <= 0 for v in vars(self).values()):
            raise ValueError('gate limits must be positive and finite')


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class VelocityGate:
    """Fail closed on sensor faults; command silence alone is a normal idle stop."""

    observation_sources = ('depth', 'slice', 'integrated_depth')

    def __init__(self, limits=None, *, require_nvblox=False, forward_only=False):
        self.limits = limits or GateLimits()
        self.require_nvblox = bool(require_nvblox)
        self.forward_only = bool(forward_only)
        self.enabled = False
        self.state = 'disabled'
        self.detail = 'waiting for explicit enable'
        self.command = None
        self.command_received = None
        self.armed_at = None
        self.goal_active = False
        self.goal_started_at = None
        self.session = ''
        self.localization_ok = False
        self.localization_received = None
        self.station_mode = True
        self.initialized = False
        self.wheel = None  # source ROS time, receipt monotonic time
        self.depth = None  # source ROS time, receipt monotonic time
        self.slice = None
        self.integrated_depth = None
        self.estimate = None  # wheel-propagated planar pose, frame, source time, receipt time
        self.competing_publisher = False
        self.last_ros_now = None
        # Startup has the same cancellation barrier as a later fault.
        self.cancel_pending = True
        self.cancel_serial = 1

    def _clear_command(self):
        self.command = None
        self.command_received = None

    def stop(self, detail, state='disabled'):
        self.enabled = False
        self.goal_active = False
        self.goal_started_at = None
        self.state, self.detail = state, detail
        self._clear_command()
        self.cancel_pending = True
        self.cancel_serial += 1

    def fault(self, detail):
        if self.enabled:
            self.stop(detail + '; explicit re-enable required', 'fault')

    def cancellation_complete(self, serial):
        if serial == self.cancel_serial:
            self.cancel_pending = False

    def set_competing_publisher(self, present):
        self.competing_publisher = bool(present)
        if present:
            self.fault('another publisher owns cmd_raw')

    def update_localization(self, healthy, initialized, navigation_ready, station_mode,
                            session, now):
        # session identifies the wheel estimator, never the camera tracker.
        # Visual outages and tracker resets do not interrupt a healthy transit.
        if session != self.session:
            self.fault('localization estimator session changed')
            self.estimate = None
            self._clear_command()
        self.session = session
        self.localization_received = now
        self.initialized = bool(initialized)
        self.station_mode = bool(station_mode)
        self.localization_ok = bool(healthy and initialized and navigation_ready
                                    and not station_mode and session)
        if not self.localization_ok:
            self.fault('station hold is active' if station_mode else
                       'wheel localization is not ready for navigation')

    def invalidate_wheel(self, reason):
        self.wheel = None
        self.fault(reason)

    def invalidate_estimate(self, reason):
        self.estimate = None
        self.fault(reason)

    def _stamp_valid(self, stamp, received, ros_now, timeout):
        return (_finite((stamp, received, ros_now)) and stamp > 0
                and -self.limits.future_tolerance_sec <= ros_now - stamp <= timeout)

    def update_wheel(self, stamp, received, ros_now):
        if not self._stamp_valid(stamp, received, ros_now, self.limits.odom_timeout_sec):
            self.invalidate_wheel('wheel odometry is stale or uses an invalid/future clock')
            return False
        if self.wheel is not None and stamp <= self.wheel[0]:
            if stamp < self.wheel[0]:
                self.invalidate_wheel('wheel odometry clock moved backwards')
            return False  # repeated samples must not refresh the watchdog
        self.wheel = stamp, received
        return True

    def invalidate_observation(self, source, reason):
        if source not in self.observation_sources:
            raise ValueError('unknown obstacle observation source')
        setattr(self, source, None)
        if self.require_nvblox:
            self.fault(reason)

    def update_observation(self, source, stamp, received, ros_now):
        if source not in self.observation_sources:
            raise ValueError('unknown obstacle observation source')
        timeout = getattr(self.limits, source + '_timeout_sec')
        if not self._stamp_valid(stamp, received, ros_now, timeout):
            self.invalidate_observation(source, 'nvblox ' + source + ' has an invalid/stale timestamp')
            return False
        old = getattr(self, source)
        if old is not None and stamp <= old[0]:
            if stamp < old[0]:
                self.invalidate_observation(source, 'nvblox ' + source + ' clock moved backwards')
            return False
        setattr(self, source, (stamp, received))
        return True

    def update_estimate(self, pose, frame, stamp, received, ros_now):
        if (len(pose) != 3 or not _finite(pose) or not frame or
                not self._stamp_valid(stamp, received, ros_now, self.limits.pose_timeout_sec)):
            self.invalidate_estimate('localized base pose is invalid, stale, or from a future clock')
            return False
        if self.estimate is not None:
            old_pose, old_frame, old_stamp, _ = self.estimate
            if stamp <= old_stamp:
                if stamp < old_stamp:
                    self.invalidate_estimate('localized pose clock moved backwards')
                return False
            if (frame != old_frame or
                    math.hypot(pose[0] - old_pose[0], pose[1] - old_pose[1]) >
                    self.limits.max_pose_jump_m or
                    abs(_wrap(pose[2] - old_pose[2])) > self.limits.max_pose_jump_rad):
                self.fault('localized pose jumped during transit or its map frame changed')
        self.estimate = tuple(pose), frame, stamp, received
        return True

    def update_command(self, command, received):
        # Discard messages received while disabled. No old cached command can
        # be released by a subsequent enable request. A fresh action status
        # transition must precede commands, including a smoother's residuals.
        if (not self.enabled or not self.goal_active or received <= self.armed_at
                or received <= self.goal_started_at):
            return False
        if len(command) != 3 or not _finite((*command, received)):
            self.fault('Nav2 produced an invalid velocity')
            return False
        vx, vy, wz = command
        if self.forward_only:
            if vx < -1e-6 or abs(vy) > 1e-6:
                self.fault('Nav2 produced reverse/lateral velocity in forward-only mode')
                return False
            vx, vy = max(0.0, vx), 0.0
        speed = math.hypot(vx, vy)
        scale = min(1.0, self.limits.max_linear_velocity / max(speed, 1e-12))
        self.command = (vx * scale, vy * scale,
                        max(-self.limits.max_angular_velocity,
                            min(self.limits.max_angular_velocity, wz)))
        self.command_received = received
        return True

    def set_goal_active(self, active, now):
        active = bool(active and self.enabled)
        if active != self.goal_active:
            self._clear_command()
            self.goal_started_at = now if active else None
        self.goal_active = active

    @staticmethod
    def _fresh(now, received, limit):
        return received is not None and 0 <= now - received <= limit

    def guard(self, now, ros_now):
        p = self.limits
        if not _finite((now, ros_now)):
            return 'invalid local clock'
        if self.competing_publisher:
            return 'another cmd_raw publisher is running'
        if self.station_mode:
            return 'station hold is active; leave station mode before enabling navigation'
        if (not self.localization_ok or
                not self._fresh(now, self.localization_received, p.localization_timeout_sec)):
            return 'initialized wheel localization is unavailable or stale'
        if (self.wheel is None or not self._fresh(now, self.wheel[1], p.odom_timeout_sec) or
                not self._stamp_valid(self.wheel[0], now, ros_now, p.odom_timeout_sec)):
            return 'wheel odometry is unavailable, stale, or from a different clock'
        if (self.estimate is None or not self._fresh(now, self.estimate[3], p.pose_timeout_sec) or
                not self._stamp_valid(self.estimate[2], now, ros_now, p.pose_timeout_sec)):
            return 'wheel-propagated base pose is unavailable, stale, or from a different clock'
        if self.require_nvblox:
            for source in self.observation_sources:
                observation = getattr(self, source)
                timeout = getattr(p, source + '_timeout_sec')
                if (observation is None or not self._fresh(now, observation[1], timeout) or
                        not self._stamp_valid(observation[0], now, ros_now, timeout)):
                    return 'nvblox ' + source + ' is unavailable, stale, or from a different clock'
        return ''

    def enable(self, now, ros_now):
        if self.enabled:
            reason = self.guard(now, ros_now)
            if reason:
                self.fault(reason)
                return False, reason
            return True, 'already enabled'
        if self.cancel_pending:
            return False, 'Nav2 cancellation has not completed; see navigation_status'
        reason = self.guard(now, ros_now)
        if reason:
            return False, reason
        self.enabled = True
        self.armed_at = now
        self.goal_active = False
        self.goal_started_at = None
        self._clear_command()
        self.state, self.detail = 'idle', 'enabled; waiting for a new Nav2 command'
        return True, self.detail

    def tick(self, now, ros_now):
        if (self.last_ros_now is not None and math.isfinite(ros_now) and
                ros_now < self.last_ros_now - 1e-6):
            self.fault('ROS clock moved backwards')
            self.wheel, self.estimate = None, None
            self.depth, self.slice = None, None
            self.integrated_depth = None
            self.localization_ok = False
        self.last_ros_now = ros_now
        if not self.enabled:
            return 0.0, 0.0, 0.0
        reason = self.guard(now, ros_now)
        if reason:
            self.fault(reason)
            return 0.0, 0.0, 0.0
        if not self.goal_active:
            self.state, self.detail = 'idle', 'enabled; waiting for a new active Nav2 goal'
            return 0.0, 0.0, 0.0
        if (self.command is None or
                not self._fresh(now, self.command_received, self.limits.cmd_timeout_sec)):
            self.state, self.detail = 'idle', 'Nav2 command absent or timed out; output zero'
            return 0.0, 0.0, 0.0
        self.state, self.detail = 'forwarding', 'forwarding bounded Nav2 velocity'
        return self.command


@dataclass
class CancelEndpoint:
    acknowledged: bool = False
    requested_ids: set = field(default_factory=set)
    statuses: dict = field(default_factory=dict)
    detail: str = 'waiting for cancellation service'

    def status(self, statuses):
        # Each message is the server's complete current snapshot. Preserve
        # terminal evidence only for UUIDs in this cancellation barrier, since
        # action servers can expire completed goals from later snapshots.
        terminal = {key: value for key, value in self.statuses.items()
                    if key in self.requested_ids and value in (4, 5, 6)}
        self.statuses = dict(statuses)
        for key, value in terminal.items():
            self.statuses.setdefault(key, value)

    def response(self, return_code, goal_ids):
        if return_code != 0:
            self.acknowledged = False
            self.detail = 'cancel rejected with return_code={}'.format(return_code)
            return False
        self.acknowledged = True
        self.requested_ids.update(goal_ids)
        self.detail = 'waiting for canceled goals to reach a terminal state'
        return True

    @property
    def complete(self):
        # An accepted cancel request can still be CANCELING (3), with the
        # controller generating commands. Wait until terminal status (4..6).
        return (self.acknowledged
                and all(self.statuses.get(key) in (4, 5, 6) for key in self.requested_ids)
                and not any(value in (0, 1, 2, 3) for value in self.statuses.values()))

    @property
    def active(self):
        return any(value in (0, 1, 2, 3) for value in self.statuses.values())
