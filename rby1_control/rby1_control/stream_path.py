"""Time-parameterised Cartesian path used by streamed Task steps.

The planner authors a path as a short list of TCP waypoints
``[x, y, z, roll, pitch, yaw]`` (metres, degrees, base frame). The control
backend turns it into a plan that starts at the measured TCP pose, allocates
the requested total duration across the segments in proportion to the time
each one needs at the velocity limits, and interpolates a target pose for any
time along it. All velocity checks live here so they can be unit-tested
without ROS.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

Pose = Tuple[float, float, float, float, float, float]
Quaternion = Tuple[float, float, float, float]


def rpy_deg_to_quaternion(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> Quaternion:
    """ZYX Euler (base-frame roll/pitch/yaw, degrees) to ``(x, y, z, w)``.

    Same convention as ``ManipulatorController.rpy_deg_to_quaternion`` so a
    streamed target matches what the regular Cartesian command would send.
    """

    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_distance_rad(a: Quaternion, b: Quaternion) -> float:
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(max(0.0, min(1.0, dot)))


def slerp(a: Quaternion, b: Quaternion, s: float) -> Quaternion:
    """Spherical interpolation along the shorter arc; ``s`` in [0, 1]."""

    s = max(0.0, min(1.0, float(s)))
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-value for value in b)
        dot = -dot
    dot = min(1.0, dot)
    if dot > 0.9995:
        result = tuple(x + s * (y - x) for x, y in zip(a, b))
    else:
        theta = math.acos(dot)
        sin_theta = math.sin(theta)
        wa = math.sin((1.0 - s) * theta) / sin_theta
        wb = math.sin(s * theta) / sin_theta
        result = tuple(wa * x + wb * y for x, y in zip(a, b))
    norm = math.sqrt(sum(value * value for value in result))
    if norm < 1.0e-12:
        raise ValueError('interpolated quaternion has zero length')
    return tuple(value / norm for value in result)


def _pose(values: Sequence[object], label: str) -> Pose:
    if isinstance(values, (str, bytes)) or len(values) != 6:
        raise ValueError(f'{label} must contain 6 values')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label} contains a non-finite value')
    return result


@dataclass(frozen=True)
class StreamPathSegment:
    start_pose: Pose
    end_pose: Pose
    start_quaternion: Quaternion
    end_quaternion: Quaternion
    linear_distance_m: float
    angular_distance_rad: float
    start_time_sec: float
    end_time_sec: float

    def target_at(self, t: float) -> Tuple[float, ...]:
        span = self.end_time_sec - self.start_time_sec
        s = 1.0 if span <= 0.0 else (t - self.start_time_sec) / span
        s = max(0.0, min(1.0, s))
        position = tuple(
            start + s * (end - start)
            for start, end in zip(self.start_pose[:3], self.end_pose[:3])
        )
        return position + slerp(self.start_quaternion, self.end_quaternion, s)


@dataclass(frozen=True)
class StreamPathPlan:
    """Piecewise-linear TCP path with slerp orientation and fixed timing."""

    segments: Tuple[StreamPathSegment, ...]
    duration_sec: float
    linear_distance_m: float
    angular_distance_rad: float
    required_time_sec: float

    def target_at(self, t: float) -> Tuple[float, ...]:
        """Return ``(x, y, z, qx, qy, qz, qw)`` for time ``t`` (clamped)."""

        t = max(0.0, min(float(self.duration_sec), float(t)))
        for segment in self.segments:
            if t <= segment.end_time_sec:
                return segment.target_at(t)
        return self.segments[-1].target_at(self.duration_sec)

    @property
    def final_pose(self) -> Pose:
        return self.segments[-1].end_pose


def build_stream_path_plan(
    start_pose: Sequence[object],
    waypoints: Sequence[Sequence[object]],
    duration_sec: float,
    linear_velocity_limit: float,
    angular_velocity_limit: float,
    *,
    backend_linear_velocity_limit: float | None = None,
    backend_angular_velocity_limit: float | None = None,
) -> StreamPathPlan:
    """Validate a path and allocate ``duration_sec`` across its segments.

    Raises ``ValueError`` when the path cannot be followed within the
    velocity limits, when the command's own limits exceed the backend's hard
    caps, or when the inputs are malformed. Time is allocated to each segment
    in proportion to the time it needs at the limits, so the TCP moves at a
    uniform fraction of the limits along the whole path.
    """

    duration = float(duration_sec)
    linear_cap = float(linear_velocity_limit)
    angular_cap = float(angular_velocity_limit)
    for label, value in (
        ('duration', duration),
        ('linear velocity limit', linear_cap),
        ('angular velocity limit', angular_cap),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{label} must be a positive finite number')
    if (
        backend_linear_velocity_limit is not None
        and linear_cap > float(backend_linear_velocity_limit) + 1.0e-9
    ):
        raise ValueError(
            f'path linear velocity limit {linear_cap:.3f} m/s exceeds the '
            f'backend limit {float(backend_linear_velocity_limit):.3f} m/s'
        )
    if (
        backend_angular_velocity_limit is not None
        and angular_cap > float(backend_angular_velocity_limit) + 1.0e-9
    ):
        raise ValueError(
            f'path angular velocity limit {angular_cap:.3f} rad/s exceeds '
            f'the backend limit '
            f'{float(backend_angular_velocity_limit):.3f} rad/s'
        )

    poses = [_pose(start_pose, 'start pose')]
    if isinstance(waypoints, (str, bytes)) or len(waypoints) == 0:
        raise ValueError('path requires at least one waypoint')
    for index, waypoint in enumerate(waypoints):
        poses.append(_pose(waypoint, f'waypoint {index}'))

    quaternions = [rpy_deg_to_quaternion(*pose[3:6]) for pose in poses]
    linear = []
    angular = []
    required = []
    for (a, b), (qa, qb) in zip(
        zip(poses, poses[1:]),
        zip(quaternions, quaternions[1:]),
    ):
        distance = math.sqrt(sum(
            (end - start) ** 2 for start, end in zip(a[:3], b[:3])
        ))
        rotation = quaternion_distance_rad(qa, qb)
        linear.append(distance)
        angular.append(rotation)
        required.append(max(distance / linear_cap, rotation / angular_cap))

    required_total = sum(required)
    if required_total > duration + 1.0e-9:
        raise ValueError(
            f'path needs at least {required_total:.2f} s at the velocity '
            f'limits ({linear_cap:.3f} m/s, {angular_cap:.3f} rad/s); the '
            f'requested duration {duration:.2f} s is too short'
        )

    count = len(required)
    if required_total <= 0.0:
        allocations = [duration / count] * count
    else:
        allocations = [duration * value / required_total for value in required]

    segments = []
    start_time = 0.0
    for index in range(count):
        end_time = start_time + allocations[index]
        if index == count - 1:
            end_time = duration
        segments.append(StreamPathSegment(
            start_pose=poses[index],
            end_pose=poses[index + 1],
            start_quaternion=quaternions[index],
            end_quaternion=quaternions[index + 1],
            linear_distance_m=linear[index],
            angular_distance_rad=angular[index],
            start_time_sec=start_time,
            end_time_sec=end_time,
        ))
        start_time = end_time

    return StreamPathPlan(
        segments=tuple(segments),
        duration_sec=duration,
        linear_distance_m=sum(linear),
        angular_distance_rad=sum(angular),
        required_time_sec=required_total,
    )


__all__ = [
    'StreamPathPlan',
    'StreamPathSegment',
    'build_stream_path_plan',
    'quaternion_distance_rad',
    'rpy_deg_to_quaternion',
    'slerp',
]
