"""ROS-free math for the Measurements tab.

Tag-to-tag relative pose
    Both tags are read in one common frame (``base``) from the same camera,
    so ``T_tag1_tag2 = inv(T_base_tag1) * T_base_tag2`` is independent of the
    base->camera chain as long as the robot holds still while sampling.

Base odometry delta
    Wheel odometry increments are composed between Start and Finish. A single
    sample jump larger than a physical limit (driver restart / odom reset) is
    skipped and counted instead of being added to the result.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import re
from statistics import pstdev
from typing import List, Optional, Sequence, Tuple

from .observation import (
    ObjectObservation,
    PoseTuple,
    average_observations,
    compose_pose_right,
    invert_pose,
    quaternion_to_rpy_deg,
)


TAG_NAME_PATTERN = re.compile(r'^\s*tag_?(\d+)\s*$', re.IGNORECASE)


def parse_tag_name(text: str) -> str:
    """Return the CameraClient object id (``tag_5``) for ``tag5``/``tag_5``."""

    match = TAG_NAME_PATTERN.match(str(text))
    if match is None:
        raise ValueError(f'{text!r} is not a tag name; use the form tagN')
    return f'tag_{int(match.group(1))}'


def relative_pose(first: PoseTuple, second: PoseTuple) -> PoseTuple:
    """Return ``second`` expressed in the ``first`` frame."""

    inverse_position, inverse_orientation = invert_pose(*first)
    return compose_pose_right(
        inverse_position,
        inverse_orientation,
        second[0],
        second[1],
    )


@dataclass(frozen=True)
class TagRelativeResult:
    """Tag2 pose in the Tag1 frame, averaged over ``sample_count`` pairs."""

    camera: str
    tag1: str
    tag2: str
    xyz: Tuple[float, float, float]
    rpy_deg: Tuple[float, float, float]
    distance: float
    xyz_std: Tuple[float, float, float]
    sample_count: int
    # Tag2 - Tag1 in the common (base) frame, for reference.
    base_delta_xyz: Tuple[float, float, float]


def average_relative_pose(
    pairs: Sequence[Tuple[ObjectObservation, ObjectObservation]],
    *,
    camera: str,
    outlier_count: int = 0,
) -> TagRelativeResult:
    """Average per-pair relative poses (tag1 -> tag2) with outlier trimming."""

    if not pairs:
        raise ValueError('no tag pairs were sampled')
    tag1 = pairs[0][0].object_id
    tag2 = pairs[0][1].object_id
    relative: List[ObjectObservation] = []
    base_deltas = []
    for first, second in pairs:
        position, orientation = relative_pose(
            (first.position, first.orientation_xyzw),
            (second.position, second.orientation_xyzw),
        )
        relative.append(ObjectObservation(
            object_id=f'{tag1}->{tag2}',
            frame_id=tag1,
            stamp_ns=max(first.stamp_ns, second.stamp_ns),
            received_at_ns=max(first.received_at_ns, second.received_at_ns),
            position=position,
            orientation_xyzw=orientation,
            confidence=1.0,
        ))
        base_deltas.append(tuple(
            b - a for a, b in zip(first.position, second.position)
        ))
    trim = min(max(0, int(outlier_count)), len(relative) - 1)
    mean = average_observations(relative, outlier_count=trim)
    xyz = tuple(float(v) for v in mean.position)
    xyz_std = tuple(
        pstdev(sample.position[axis] for sample in relative)
        if len(relative) > 1 else 0.0
        for axis in range(3)
    )
    base_delta = tuple(
        sum(delta[axis] for delta in base_deltas) / len(base_deltas)
        for axis in range(3)
    )
    return TagRelativeResult(
        camera=camera,
        tag1=tag1,
        tag2=tag2,
        xyz=xyz,  # type: ignore[arg-type]
        rpy_deg=quaternion_to_rpy_deg(mean.orientation_xyzw),
        distance=math.sqrt(sum(v * v for v in xyz)),
        xyz_std=xyz_std,  # type: ignore[arg-type]
        sample_count=len(relative),
        base_delta_xyz=base_delta,  # type: ignore[arg-type]
    )


# ----------------------------------------------------------------------
# Base odometry
# ----------------------------------------------------------------------
Pose2D = Tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def se2_between(first: Pose2D, second: Pose2D) -> Pose2D:
    """Return ``second`` in the ``first`` body frame (``inv(first)*second``)."""

    dx = second[0] - first[0]
    dy = second[1] - first[1]
    cosine, sine = math.cos(first[2]), math.sin(first[2])
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        normalize_angle(second[2] - first[2]),
    )


def se2_compose(first: Pose2D, second: Pose2D) -> Pose2D:
    cosine, sine = math.cos(first[2]), math.sin(first[2])
    return (
        first[0] + cosine * second[0] - sine * second[1],
        first[1] + sine * second[0] + cosine * second[1],
        normalize_angle(first[2] + second[2]),
    )


@dataclass
class OdometryDelta:
    """Accumulated motion since Start, in the Start body frame."""

    # Start-body frame: x forward, y left (base_footprint axes at Start).
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    # The same displacement expressed in the odom frame.
    odom_dx: float = 0.0
    odom_dy: float = 0.0
    path_length: float = 0.0
    duration_sec: float = 0.0
    sample_count: int = 0
    skipped_jumps: int = 0
    start_pose: Optional[Pose2D] = None
    end_pose: Optional[Pose2D] = None

    @property
    def planar_distance(self) -> float:
        return math.hypot(self.x, self.y)


@dataclass
class OdometryTracker:
    """Compose wheel-odometry increments between Start and Finish.

    ``max_step_m`` / ``max_step_rad`` reject a single-sample jump that a base
    cannot physically make between two odometry messages.
    """

    max_step_m: float = 0.25
    max_step_rad: float = 0.5
    active: bool = False
    _last: Optional[Pose2D] = None
    _last_time: Optional[float] = None
    _start_time: Optional[float] = None
    _accumulated: Pose2D = (0.0, 0.0, 0.0)
    _odom_start: Optional[Pose2D] = None
    _delta: OdometryDelta = field(default_factory=OdometryDelta)
    latest_pose: Optional[Pose2D] = None
    latest_time: Optional[float] = None
    # time.monotonic() of the latest message, for a stream-freshness display.
    latest_received: Optional[float] = None

    def start(self) -> None:
        if self.latest_pose is None:
            raise RuntimeError('no base odometry has been received yet')
        self.active = True
        self._last = self.latest_pose
        self._start_time = self.latest_time
        self._accumulated = (0.0, 0.0, 0.0)
        self._odom_start = self.latest_pose
        self._delta = OdometryDelta(start_pose=self.latest_pose)

    def finish(self) -> OdometryDelta:
        if not self.active:
            raise RuntimeError('odometry measurement has not been started')
        self.active = False
        return self.current()

    def cancel(self) -> None:
        self.active = False

    def update(
        self,
        pose: Pose2D,
        stamp_sec: float,
        received_at: Optional[float] = None,
    ) -> None:
        self.latest_received = received_at
        self.latest_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        self.latest_time = float(stamp_sec)
        if not self.active or self._last is None:
            return
        step = se2_between(self._last, self.latest_pose)
        self._last = self.latest_pose
        self._delta.sample_count += 1
        if (
            math.hypot(step[0], step[1]) > self.max_step_m
            or abs(step[2]) > self.max_step_rad
        ):
            self._delta.skipped_jumps += 1
            return
        self._accumulated = se2_compose(self._accumulated, step)
        self._delta.path_length += math.hypot(step[0], step[1])

    def current(self) -> OdometryDelta:
        delta = self._delta
        x, y, yaw = self._accumulated
        delta.x, delta.y, delta.yaw = x, y, yaw
        start = self._odom_start or (0.0, 0.0, 0.0)
        cosine, sine = math.cos(start[2]), math.sin(start[2])
        delta.odom_dx = cosine * x - sine * y
        delta.odom_dy = sine * x + cosine * y
        delta.end_pose = self.latest_pose
        if self._start_time is not None and self.latest_time is not None:
            delta.duration_sec = max(0.0, self.latest_time - self._start_time)
        return delta


def format_tag_result(result: TagRelativeResult) -> str:
    x, y, z = result.xyz
    roll, pitch, yaw = result.rpy_deg
    sx, sy, sz = result.xyz_std
    bx, by, bz = result.base_delta_xyz
    return '\n'.join((
        f'{result.tag2} in {result.tag1} frame  (camera {result.camera}, '
        f'{result.sample_count} samples)',
        f'  xyz  = [{x:+.4f}, {y:+.4f}, {z:+.4f}] m',
        f'  rpy  = [{roll:+.2f}, {pitch:+.2f}, {yaw:+.2f}] deg',
        f'  |d|  = {result.distance:.4f} m',
        f'  std  = [{sx * 1000:.1f}, {sy * 1000:.1f}, {sz * 1000:.1f}] mm',
        f'  base-frame tag2-tag1 = [{bx:+.4f}, {by:+.4f}, {bz:+.4f}] m',
    ))


def format_odometry_delta(delta: OdometryDelta) -> str:
    lines = [
        f'Start frame (x fwd, y left): '
        f'xy = [{delta.x:+.4f}, {delta.y:+.4f}] m',
        f'Odom frame: xy = [{delta.odom_dx:+.4f}, {delta.odom_dy:+.4f}] m',
    ]
    if delta.skipped_jumps:
        lines.append(
            f'WARNING: {delta.skipped_jumps} odometry jump(s) skipped '
            '(odom reset?)'
        )
    return '\n'.join(lines)


__all__ = [
    'OdometryDelta',
    'OdometryTracker',
    'TagRelativeResult',
    'average_relative_pose',
    'format_odometry_delta',
    'format_tag_result',
    'parse_tag_name',
    'relative_pose',
    'yaw_from_quaternion',
]
