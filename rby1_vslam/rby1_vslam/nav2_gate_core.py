"""ROS-independent watchdog for Nav2 velocity commands and cancel barriers."""

from dataclasses import dataclass, field
import math


@dataclass
class GateLimits:
    cmd_timeout_sec: float = 0.25
    odom_timeout_sec: float = 0.5
    pose_timeout_sec: float = 0.5
    bridge_timeout_sec: float = 0.75
    localization_timeout_sec: float = 0.75
    future_tolerance_sec: float = 0.05
    max_linear_velocity: float = 0.15
    max_angular_velocity: float = 0.35
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

    def __init__(self, limits=None):
        self.limits = limits or GateLimits()
        self.enabled = False
        self.state = 'disabled'
        self.detail = 'waiting for explicit enable'
        self.command = None
        self.command_received = None
        self.armed_at = None
        self.bridge_ok = False
        self.bridge_received = None
        self.session = ''
        self.localization_ok = False
        self.localization_received = None
        self.localization_session = ''
        self.wheel = None  # source ROS time, receipt monotonic time
        self.slam = None  # planar pose, frame, source time, receipt time
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

    def update_bridge(self, connected, tracking_ok, session, now):
        if session != self.session:
            self.fault('TCP bridge session changed')
            self.slam = None
            self.localization_ok = False
            self.localization_received = None
            self._clear_command()
        self.session = session
        self.bridge_received = now
        self.bridge_ok = bool(connected and tracking_ok and session)
        if not self.bridge_ok:
            self.fault('TCP bridge disconnected or visual tracking lost')

    def update_localization(self, healthy, session, now):
        self.localization_received = now
        self.localization_session = session
        self.localization_ok = bool(healthy and session and session == self.session)
        if not self.localization_ok:
            self.fault('map-to-odom correction is unavailable or belongs to another session')

    def invalidate_wheel(self, reason):
        self.wheel = None
        self.fault(reason)

    def invalidate_slam(self, reason):
        self.slam = None
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

    def update_slam(self, pose, frame, stamp, received, ros_now):
        if (len(pose) != 3 or not _finite(pose) or not frame or
                not self._stamp_valid(stamp, received, ros_now, self.limits.pose_timeout_sec)):
            self.invalidate_slam('base SLAM pose is invalid, stale, or from a future clock')
            return False
        if self.slam is not None:
            old_pose, old_frame, old_stamp, _ = self.slam
            if stamp <= old_stamp:
                if stamp < old_stamp:
                    self.invalidate_slam('SLAM pose clock moved backwards')
                return False
            if (frame != old_frame or
                    math.hypot(pose[0] - old_pose[0], pose[1] - old_pose[1]) >
                    self.limits.max_pose_jump_m or
                    abs(_wrap(pose[2] - old_pose[2])) > self.limits.max_pose_jump_rad):
                self.fault('SLAM pose jumped or its map frame changed')
        self.slam = tuple(pose), frame, stamp, received
        return True

    def update_command(self, command, received):
        # Discard messages received while disabled. No old cached command can
        # be released by a subsequent enable request.
        if not self.enabled or received <= self.armed_at:
            return False
        if len(command) != 3 or not _finite((*command, received)):
            self.fault('Nav2 produced an invalid velocity')
            return False
        vx, vy, wz = command
        speed = math.hypot(vx, vy)
        scale = min(1.0, self.limits.max_linear_velocity / max(speed, 1e-12))
        self.command = (vx * scale, vy * scale,
                        max(-self.limits.max_angular_velocity,
                            min(self.limits.max_angular_velocity, wz)))
        self.command_received = received
        return True

    @staticmethod
    def _fresh(now, received, limit):
        return received is not None and 0 <= now - received <= limit

    def guard(self, now, ros_now):
        p = self.limits
        if not _finite((now, ros_now)):
            return 'invalid local clock'
        if self.competing_publisher:
            return 'another cmd_raw publisher is running'
        if not self.bridge_ok or not self._fresh(now, self.bridge_received, p.bridge_timeout_sec):
            return 'bridge/tracking is unavailable or stale'
        if (not self.localization_ok or self.localization_session != self.session or
                not self._fresh(now, self.localization_received, p.localization_timeout_sec)):
            return 'map-to-odom correction is unavailable or stale'
        if (self.wheel is None or not self._fresh(now, self.wheel[1], p.odom_timeout_sec) or
                not self._stamp_valid(self.wheel[0], now, ros_now, p.odom_timeout_sec)):
            return 'wheel odometry is unavailable, stale, or from a different clock'
        if (self.slam is None or not self._fresh(now, self.slam[3], p.pose_timeout_sec) or
                not self._stamp_valid(self.slam[2], now, ros_now, p.pose_timeout_sec)):
            return 'base SLAM pose is unavailable, stale, or from a different clock'
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
        self._clear_command()
        self.state, self.detail = 'idle', 'enabled; waiting for a new Nav2 command'
        return True, self.detail

    def tick(self, now, ros_now):
        if (self.last_ros_now is not None and math.isfinite(ros_now) and
                ros_now < self.last_ros_now - 1e-6):
            self.fault('ROS clock moved backwards')
            self.wheel, self.slam = None, None
            self.localization_ok = False
        self.last_ros_now = ros_now
        if not self.enabled:
            return 0.0, 0.0, 0.0
        reason = self.guard(now, ros_now)
        if reason:
            self.fault(reason)
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
