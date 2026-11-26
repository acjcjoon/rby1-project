"""Planner Task authoring with planner-only execution steps.

Ordinary motion commands use planner-owned protocol value objects. Camera and
odometry-navigation commands are planner-only descriptors
resolved by :class:`PlannerTaskRunner` during execution.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
import re
from typing import (
    TYPE_CHECKING,
    Iterable,
    Optional,
    Sequence,
    Union,
    cast,
)

import yaml

from .control_commands import (
    BODY_JOINT_GROUPS,
    CARTESIAN_ARMS,
    CARTESIAN_GROUPS,
    JOINT_GROUP_DOF,
    CommandKind,
    Impedance,
    JointImpedance,
    STREAM_PATH_KINDS,
    Task as _CanonicalTask,
    TaskCommand,
    TaskDefinition,
    DEFAULT_IMPEDANCE_PRESET,
    ImpedanceSpec,
    arc_waypoints,
    resolve_impedance,
    set_impedance_preset_loader,
    check_impedance_exit,
    impedance_from_dict,
    impedance_tag,
    impedance_tag_for,
    impedance_to_dict,
    list_sum,
    _values,
)

from .observation import (
    CartesianTarget,
    ObjectObservation,
    compose_pose_right,
    compose_pose_yaw_only,
    normalize_quaternion,
    observation_to_cartesian_target,
)


if TYPE_CHECKING:
    from .camera import CameraClient


# 손목 카메라: 팔 -> camera_source / 카메라 중심 프레임.
# 이름은 rby1_camera/config/camera_system.yaml 의 카메라별 블록과 같다.
WRIST_CAMERA_SOURCE_BY_ARM = {
    'right_arm': 'd405_right',
    'left_arm': 'd405_left',
}
WRIST_CAMERA_FRAME_BY_ARM = {
    'right_arm': 'd405_right_camera_center',
    'left_arm': 'd405_left_camera_center',
}
# 팔 -> end effector 프레임 (base 정렬 목표를 카메라 중심에서 옮길 방향).
END_EFFECTOR_FRAME_BY_ARM = {
    'right_arm': 'ee_right',
    'left_arm': 'ee_left',
}


def wrist_camera_source(arm: str, camera_source: Optional[str] = None) -> str:
    """camera_source 가 None 이면 arm 에 달린 손목 카메라를 고른다."""
    if camera_source is not None:
        return camera_source
    try:
        return WRIST_CAMERA_SOURCE_BY_ARM[str(arm).strip()]
    except KeyError:
        raise ValueError(
            f'{arm!r} has no wrist camera; pass camera_source explicitly'
        ) from None


def wrist_camera_frame(arm: str, camera_frame: Optional[str] = None) -> str:
    """camera_frame 이 None 이면 arm 에 달린 손목 카메라 중심 프레임을 고른다."""
    if camera_frame is not None:
        return camera_frame
    try:
        return WRIST_CAMERA_FRAME_BY_ARM[str(arm).strip()]
    except KeyError:
        raise ValueError(
            f'{arm!r} has no wrist camera; pass camera_frame explicitly'
        ) from None


def end_effector_frame(arm: str) -> str:
    """arm 의 end effector 프레임 (ee_left / ee_right)."""
    try:
        return END_EFFECTOR_FRAME_BY_ARM[str(arm).strip()]
    except KeyError:
        raise ValueError(f'{arm!r} has no end effector frame') from None


def _finite_number(
    value: object,
    label: str,
    *,
    positive: bool = False,
) -> float:
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


def _finite_values(
    values: Sequence[object],
    count: int,
    label: str,
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f'{label} must contain {count} finite numbers')
    result = tuple(_finite_number(value, label) for value in values)
    if len(result) != count:
        raise ValueError(f'{label} must contain {count} finite numbers')
    return result


@dataclass(frozen=True)
class CameraLinearAbsoluteStep:
    """Deferred object lookup followed by a canonical Cartesian motion."""

    group: str
    object_id: str
    minimum_time: float
    linear_velocity: float
    angular_velocity: float
    acceleration_scaling: float
    camera_source: str = 'd405_right'
    detection_timeout_sec: float = 3.0
    max_age_sec: Optional[float] = None
    minimum_confidence: Optional[float] = None
    reuse_previous_observation: bool = False
    yaw_only: bool = False
    yaw_symmetry_deg: Optional[float] = None
    log_target_and_actual: bool = False
    object_to_end_effector_position: tuple[float, ...] = (0.0, 0.0, 0.0)
    object_to_end_effector_orientation_xyzw: tuple[float, ...] = (
        0.0,
        0.0,
        0.0,
        1.0,
    )
    # Set by the camera contact helpers only: the resolved target is tracked
    # through the arm IK with joint impedance.
    impedance: Optional[JointImpedance] = None

    def __post_init__(self) -> None:
        group = str(self.group).strip()
        object_id = str(self.object_id).strip()
        camera_source = str(self.camera_source).strip()
        if not object_id:
            raise ValueError('object_id must not be empty')
        if not camera_source:
            raise ValueError('camera_source must not be empty')

        minimum_time = _finite_number(
            self.minimum_time,
            'minimum_time',
            positive=True,
        )
        linear_velocity = _finite_number(
            self.linear_velocity,
            'linear_velocity',
            positive=True,
        )
        angular_velocity = _finite_number(
            self.angular_velocity,
            'angular_velocity',
            positive=True,
        )
        acceleration_scaling = _finite_number(
            self.acceleration_scaling,
            'acceleration_scaling',
            positive=True,
        )
        detection_timeout_sec = _finite_number(
            self.detection_timeout_sec,
            'detection_timeout_sec',
            positive=True,
        )
        max_age_sec = (
            None
            if self.max_age_sec is None
            else _finite_number(
                self.max_age_sec,
                'max_age_sec',
                positive=True,
            )
        )
        minimum_confidence = (
            None
            if self.minimum_confidence is None
            else _finite_number(
                self.minimum_confidence,
                'minimum_confidence',
            )
        )
        if (
            minimum_confidence is not None
            and not 0.0 <= minimum_confidence <= 1.0
        ):
            raise ValueError(
                'minimum_confidence must be in the range [0, 1]'
            )
        if not isinstance(self.reuse_previous_observation, bool):
            raise ValueError('reuse_previous_observation must be a boolean')
        if not isinstance(self.yaw_only, bool):
            raise ValueError('yaw_only must be a boolean')
        yaw_symmetry_deg = (
            None
            if self.yaw_symmetry_deg is None
            else _finite_number(
                self.yaw_symmetry_deg,
                'yaw_symmetry_deg',
                positive=True,
            )
        )
        if yaw_symmetry_deg is not None:
            if not self.yaw_only:
                raise ValueError('yaw_symmetry_deg requires yaw_only=True')
            symmetry_order = 360.0 / yaw_symmetry_deg
            if (
                yaw_symmetry_deg > 360.0
                or not math.isclose(
                    symmetry_order,
                    round(symmetry_order),
                    rel_tol=0.0,
                    abs_tol=1.0e-9,
                )
            ):
                raise ValueError(
                    'yaw_symmetry_deg must divide 360 degrees exactly'
                )
        if not isinstance(self.log_target_and_actual, bool):
            raise ValueError('log_target_and_actual must be a boolean')
        offset_position = _finite_values(
            self.object_to_end_effector_position,
            3,
            'object_to_end_effector_position',
        )
        offset_orientation = normalize_quaternion(
            self.object_to_end_effector_orientation_xyzw
        )
        # Position stays anchored to the detected tag axes. Yaw symmetry only
        # selects an equivalent EE orientation; it must not rotate that offset.

        # Reuse the canonical command validation for arm and motion limits.
        TaskCommand(
            kind=CommandKind.LINEAR_ABSOLUTE,
            group=group,
            values=(0.0,) * 6,
            minimum_time=minimum_time,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            acceleration_scaling=acceleration_scaling,
            impedance=self.impedance,
        )

        object.__setattr__(self, 'group', group)
        object.__setattr__(self, 'object_id', object_id)
        object.__setattr__(self, 'camera_source', camera_source)
        object.__setattr__(self, 'minimum_time', minimum_time)
        object.__setattr__(self, 'linear_velocity', linear_velocity)
        object.__setattr__(self, 'angular_velocity', angular_velocity)
        object.__setattr__(
            self,
            'acceleration_scaling',
            acceleration_scaling,
        )
        object.__setattr__(
            self,
            'detection_timeout_sec',
            detection_timeout_sec,
        )
        object.__setattr__(self, 'max_age_sec', max_age_sec)
        object.__setattr__(
            self,
            'minimum_confidence',
            minimum_confidence,
        )
        object.__setattr__(self, 'yaw_symmetry_deg', yaw_symmetry_deg)
        object.__setattr__(
            self,
            'object_to_end_effector_position',
            offset_position,
        )
        object.__setattr__(
            self,
            'object_to_end_effector_orientation_xyzw',
            offset_orientation,
        )

    def resolve(self, values: Sequence[object]) -> TaskCommand:
        """Create the canonical command sent to the control backend."""

        return TaskCommand(
            kind=CommandKind.LINEAR_ABSOLUTE,
            group=self.group,
            values=tuple(values),
            minimum_time=self.minimum_time,
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
            acceleration_scaling=self.acceleration_scaling,
            impedance=self.impedance,
        )

    def impedance_exit_role(self) -> tuple[str, tuple[str, ...]]:
        if self.impedance is not None:
            return ('contact', (self.group,))
        return ('position', (self.group,))

    def resolve_observation(
        self,
        observation: ObjectObservation,
    ) -> TaskCommand:
        """Compose the configured object-to-EE transform and resolve it."""

        if not isinstance(observation, ObjectObservation):
            raise TypeError('camera must return an ObjectObservation')
        compose = compose_pose_yaw_only if self.yaw_only else compose_pose_right
        position, orientation = compose(
            observation.position,
            observation.orientation_xyzw,
            self.object_to_end_effector_position,
            self.object_to_end_effector_orientation_xyzw,
        )
        target = observation_to_cartesian_target(
            replace(
                observation,
                position=position,
                orientation_xyzw=orientation,
            ),
            target_frame='base',
        )
        return self.resolve(target)


@dataclass(frozen=True)
class CameraLinearAbsolutePrintStep(CameraLinearAbsoluteStep):
    """Resolve and print the final TCP target without commanding motion."""

    def impedance_exit_role(self) -> tuple[str, tuple[str, ...]]:
        return ('other', ())


@dataclass(frozen=True)
class CameraFrameLinearAbsoluteStep(CameraLinearAbsoluteStep):
    """Move an arm so a mounted camera frame reaches a tag-relative pose."""

    controlled_frame: str = ''
    preserve_end_effector_orientation: bool = True

    def __post_init__(self) -> None:
        super().__post_init__()
        controlled_frame = str(self.controlled_frame).strip()
        if not controlled_frame:
            raise ValueError('controlled_frame must not be empty')
        if not isinstance(self.preserve_end_effector_orientation, bool):
            raise ValueError(
                'preserve_end_effector_orientation must be a boolean'
            )
        object.__setattr__(self, 'controlled_frame', controlled_frame)

    @property
    def object_to_controlled_frame_position(self) -> tuple[float, ...]:
        return self.object_to_end_effector_position

    @property
    def object_to_controlled_frame_orientation_xyzw(
        self,
    ) -> tuple[float, ...]:
        return self.object_to_end_effector_orientation_xyzw


@dataclass(frozen=True)
class MoveToStep:
    """Body-relative odometry target executed by ``rby1_navigation``."""

    x: float
    y: float
    yaw: float
    timeout_sec: float = 30.0

    def __post_init__(self) -> None:
        object.__setattr__(self, 'x', _finite_number(self.x, 'x'))
        object.__setattr__(self, 'y', _finite_number(self.y, 'y'))
        object.__setattr__(self, 'yaw', _finite_number(self.yaw, 'yaw'))
        object.__setattr__(
            self,
            'timeout_sec',
            _finite_number(
                self.timeout_sec,
                'timeout_sec',
                positive=True,
            ),
        )


VSLAM_STEP_ACTIONS = ('start', 'correct', 'operator', 'save_map')


@dataclass(frozen=True)
class VslamStep:
    """cuVSLAM stack operation executed through the planner's VSLAM client.

    ``start``     start the stack in ``mode`` and wait for visual tracking.
    ``correct``   stop the base and wait for a station correction.
    ``operator``  hand the base to the operator (Base tab jog) until
                  "Finish mapping" is pressed in the VSLAM tab.
    ``save_map``  save the cuVSLAM map; an empty ``map_name`` becomes
                  ``map_<date>_<time>`` when the step runs.
    """

    action: str
    mode: str = 'mapping'
    map_name: str = ''
    timeout_sec: float = 120.0

    def __post_init__(self) -> None:
        action = str(self.action).strip()
        if action not in VSLAM_STEP_ACTIONS:
            raise ValueError(
                f'vslam action must be one of {VSLAM_STEP_ACTIONS}'
            )
        mode = str(self.mode).strip()
        if mode not in ('mapping', 'localization'):
            raise ValueError("vslam mode must be 'mapping' or 'localization'")
        map_name = str(self.map_name or '').strip()
        if map_name and not re.match(
            r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$', map_name
        ):
            raise ValueError(
                'vslam map_name must use letters, digits, "_", "." or "-"'
            )
        if action == 'start' and mode == 'localization' and not map_name:
            raise ValueError('vslam localization start requires map_name')
        object.__setattr__(self, 'action', action)
        object.__setattr__(self, 'mode', mode)
        object.__setattr__(self, 'map_name', map_name)
        object.__setattr__(
            self,
            'timeout_sec',
            _finite_number(self.timeout_sec, 'timeout_sec', positive=True),
        )


@dataclass(frozen=True)
class CameraMoveToTagStep:
    """Deferred camera observation followed by relative base navigation."""

    camera_source: str
    object_id: str
    desired_tag_x: float
    desired_tag_y: float
    desired_tag_frame: str = 'base'
    relative_yaw: float = 0.0
    detection_timeout_sec: float = 3.0
    navigation_timeout_sec: float = 30.0
    max_translation_m: float = 0.8
    max_yaw_rad: float = 0.5
    max_age_sec: Optional[float] = None
    minimum_confidence: Optional[float] = None
    threshold_x_minus: float = 0.0
    threshold_y_plus: float = 0.0
    # 목표점을 base XY 에서 shift_toward_frame 쪽으로 shift_toward_m 만큼 옮긴다.
    shift_toward_frame: Optional[str] = None
    shift_toward_m: float = 0.0

    def __post_init__(self) -> None:
        camera_source = str(self.camera_source).strip()
        object_id = str(self.object_id).strip()
        desired_tag_frame = str(self.desired_tag_frame).strip()
        shift_toward_m = _finite_number(self.shift_toward_m, 'shift_toward_m')
        if shift_toward_m < 0.0:
            raise ValueError('shift_toward_m must be nonnegative')
        shift_toward_frame = (
            None
            if self.shift_toward_frame is None
            else str(self.shift_toward_frame).strip() or None
        )
        if shift_toward_m > 0.0 and shift_toward_frame is None:
            raise ValueError('shift_toward_m requires shift_toward_frame')
        object.__setattr__(self, 'shift_toward_m', shift_toward_m)
        object.__setattr__(self, 'shift_toward_frame', shift_toward_frame)
        if not camera_source:
            raise ValueError('camera_source must not be empty')
        if not object_id:
            raise ValueError('object_id must not be empty')
        if not desired_tag_frame:
            raise ValueError('desired_tag_frame must not be empty')

        for name in ('threshold_x_minus', 'threshold_y_plus'):
            value = _finite_number(getattr(self, name), name)
            if value < 0.0:
                raise ValueError(f'{name} must be nonnegative')
            object.__setattr__(self, name, value)

        desired_tag_x = _finite_number(
            self.desired_tag_x,
            'desired_tag_x',
        )
        desired_tag_y = _finite_number(
            self.desired_tag_y,
            'desired_tag_y',
        )
        relative_yaw = _finite_number(self.relative_yaw, 'relative_yaw')
        detection_timeout_sec = _finite_number(
            self.detection_timeout_sec,
            'detection_timeout_sec',
            positive=True,
        )
        navigation_timeout_sec = _finite_number(
            self.navigation_timeout_sec,
            'navigation_timeout_sec',
            positive=True,
        )
        max_translation_m = _finite_number(
            self.max_translation_m,
            'max_translation_m',
            positive=True,
        )
        max_yaw_rad = _finite_number(
            self.max_yaw_rad,
            'max_yaw_rad',
            positive=True,
        )
        if abs(relative_yaw) > max_yaw_rad:
            raise ValueError(
                'relative_yaw must not exceed max_yaw_rad'
            )
        max_age_sec = (
            None
            if self.max_age_sec is None
            else _finite_number(
                self.max_age_sec,
                'max_age_sec',
                positive=True,
            )
        )
        minimum_confidence = (
            None
            if self.minimum_confidence is None
            else _finite_number(
                self.minimum_confidence,
                'minimum_confidence',
            )
        )
        if (
            minimum_confidence is not None
            and not 0.0 <= minimum_confidence <= 1.0
        ):
            raise ValueError(
                'minimum_confidence must be in the range [0, 1]'
            )

        object.__setattr__(self, 'camera_source', camera_source)
        object.__setattr__(self, 'object_id', object_id)
        object.__setattr__(self, 'desired_tag_frame', desired_tag_frame)
        object.__setattr__(self, 'desired_tag_x', desired_tag_x)
        object.__setattr__(self, 'desired_tag_y', desired_tag_y)
        object.__setattr__(self, 'relative_yaw', relative_yaw)
        object.__setattr__(
            self,
            'detection_timeout_sec',
            detection_timeout_sec,
        )
        object.__setattr__(
            self,
            'navigation_timeout_sec',
            navigation_timeout_sec,
        )
        object.__setattr__(
            self,
            'max_translation_m',
            max_translation_m,
        )
        object.__setattr__(self, 'max_yaw_rad', max_yaw_rad)
        object.__setattr__(self, 'max_age_sec', max_age_sec)
        object.__setattr__(
            self,
            'minimum_confidence',
            minimum_confidence,
        )


@dataclass(frozen=True)
class RewindStep:
    """Repeat the complete Task, either forever or for a fixed total count."""

    repeat_count: Optional[int] = None

    def __post_init__(self) -> None:
        if self.repeat_count is None:
            return
        if (
            isinstance(self.repeat_count, bool)
            or not isinstance(self.repeat_count, int)
            or self.repeat_count < 1
        ):
            raise ValueError('rewind repeat_count must be a positive integer')


@dataclass(frozen=True)
class BackgroundGripperStep:
    """Gripper command the runner starts without waiting for it to finish.

    The next Task step (typically an arm motion) starts right away, so the
    gripper moves while the arm moves. The runner still watches the command:
    a failure fails the Task on the next tick, the next background gripper
    step waits until this one has finished (the backend runs one gripper
    command at a time), and the Task does not complete while it is pending.

    Only for gestures with empty hands. Never use it to grasp or release an
    object: the following motion does not wait for the grip.
    """

    command: TaskCommand

    def __post_init__(self) -> None:
        if not isinstance(self.command, TaskCommand) or self.command.kind not in (
            CommandKind.GRIPPER_OPEN,
            CommandKind.GRIPPER_CLOSE,
            CommandKind.GRIPPER_SET,
        ):
            raise ValueError('background step requires a gripper command')


DynamicTaskStep = Union[
    TaskCommand,
    BackgroundGripperStep,
    CameraFrameLinearAbsoluteStep,
    CameraLinearAbsoluteStep,
    CameraLinearAbsolutePrintStep,
    CameraMoveToTagStep,
    MoveToStep,
    RewindStep,
    VslamStep,
]


@dataclass(frozen=True)
class DynamicTaskDefinition:
    """Immutable planner Task containing at least one deferred command."""

    name: str
    commands: tuple[DynamicTaskStep, ...]
    description: str = ''

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('Task name must be nonempty')
        if not isinstance(self.description, str):
            raise ValueError('Task description must be a string')
        commands = tuple(self.commands)
        if not all(
            isinstance(
                command,
                (
                    TaskCommand,
                    CameraLinearAbsoluteStep,
                    CameraMoveToTagStep,
                    MoveToStep,
                    VslamStep,
                    BackgroundGripperStep,
                    RewindStep,
                ),
            )
            for command in commands
        ):
            raise ValueError('Planner Task commands are invalid')
        rewind_indexes = [
            index
            for index, command in enumerate(commands)
            if isinstance(command, RewindStep)
        ]
        if len(rewind_indexes) > 1:
            raise ValueError('Planner Task can contain only one rewind')
        if rewind_indexes and rewind_indexes[0] != len(commands) - 1:
            raise ValueError('rewind must be the final Task step')
        if rewind_indexes and len(commands) == 1:
            raise ValueError('rewind requires at least one preceding Task step')
        if not any(
            isinstance(
                command,
                (
                    CameraLinearAbsoluteStep,
                    CameraMoveToTagStep,
                    MoveToStep,
                    VslamStep,
                    BackgroundGripperStep,
                    RewindStep,
                ),
            )
            for command in commands
        ):
            raise ValueError(
                'DynamicTaskDefinition requires a planner-only command'
            )
        object.__setattr__(self, 'commands', commands)


RunnableTaskDefinition = Union[TaskDefinition, DynamicTaskDefinition]


class Task(_CanonicalTask):
    """Canonical Task builder extended with planner-only commands."""

    def open_gripper_background(
        self,
        side: str = 'both',
        speed: Optional[float] = None,
    ) -> 'Task':
        """Start opening the gripper and continue with the next step at once.

        See :class:`BackgroundGripperStep`; for empty-hand gestures only.
        """
        command = _CanonicalTask('gripper').open_gripper(side, speed).task_list[0]
        self.task_list.append(BackgroundGripperStep(command))
        return self

    def close_gripper_background(
        self,
        side: str = 'both',
        speed: Optional[float] = None,
        *,
        settle_time_sec: float = 0.3,
    ) -> 'Task':
        """Start closing the gripper and continue with the next step at once.

        See :class:`BackgroundGripperStep`; for empty-hand gestures only.
        """
        command = _CanonicalTask('gripper').close_gripper(
            side,
            speed,
            settle_time_sec=settle_time_sec,
        ).task_list[0]
        self.task_list.append(BackgroundGripperStep(command))
        return self

    def move_to(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_sec: float = 30.0,
    ) -> 'Task':
        """Drive to a body-relative odometry target.

        ``x`` and ``y`` are metres in the base frame at step start. ``yaw`` is
        a relative rotation in radians. The Task completes only after the nav
        state topic reports ``succeeded``.
        """
        self.task_list.append(MoveToStep(
            x=x,
            y=y,
            yaw=yaw,
            timeout_sec=timeout_sec,
        ))
        return self

    def vslam_start(
        self,
        mode: str = 'mapping',
        map_name: str = '',
        *,
        timeout_sec: float = 120.0,
    ) -> 'Task':
        """Start cuVSLAM in the Isaac container and wait for tracking.

        If the stack already runs in the same ``mode`` it is reused.
        """
        self.task_list.append(VslamStep(
            'start', mode=mode, map_name=map_name, timeout_sec=timeout_sec,
        ))
        return self

    def vslam_station_correct(self, *, timeout_sec: float = 30.0) -> 'Task':
        """Stop the base and wait for a converged station correction."""
        self.task_list.append(VslamStep('correct', timeout_sec=timeout_sec))
        return self

    def vslam_operator_mapping(self) -> 'Task':
        """Let the operator drive (Base tab) until "Finish mapping".

        The Task stays active; points are saved from the VSLAM tab.
        """
        self.task_list.append(VslamStep('operator'))
        return self

    def vslam_save_map(
        self,
        map_name: str = '',
        *,
        timeout_sec: float = 180.0,
    ) -> 'Task':
        """Save the cuVSLAM map (empty name = ``map_<date>_<time>``)."""
        self.task_list.append(VslamStep(
            'save_map', map_name=map_name, timeout_sec=timeout_sec,
        ))
        return self

    def move_base_to_detected_tag(
        self,
        camera_source: str,
        object_id: str,
        *,
        desired_tag_x: float,
        desired_tag_y: float,
        desired_tag_frame: str = 'base',
        threshold_x_minus: float = 0.0,
        threshold_y_plus: float = 0.0,
        relative_yaw: float = 0.0,
        detection_timeout_sec: float = 3.0,
        navigation_timeout_sec: float = 30.0,
        max_translation_m: float = 0.8,
        max_yaw_rad: float = 0.5,
        max_age_sec: Optional[float] = None,
        minimum_confidence: Optional[float] = None,
        shift_toward_frame: Optional[str] = None,
        shift_toward_m: float = 0.0,
    ) -> 'Task':
        """Defer a tag observation used to calculate a relative base goal.

        ``shift_toward_m`` moves the desired point that far in base XY toward
        the origin of ``shift_toward_frame`` (looked up at execution time).

        ``desired_tag_x`` and ``desired_tag_y`` describe where the tag should
        appear in ``desired_tag_frame`` after navigation. That point is
        transformed to the base frame at execution time before the SE(2) goal
        is calculated by PlannerTaskRunner.

        Thresholds are nonnegative metres in the base axes at step start,
        independent of the axes of ``desired_tag_frame``. The permitted
        tag-minus-goal error is X in [-threshold_x_minus, 0] and Y in
        [0, threshold_y_plus]. Only the excess beyond this rectangle is
        corrected. With zero translation and yaw the step skips navigation.
        The reference frame must remain fixed relative to base during the
        move; Z is ignored. Zero thresholds request exact XY alignment.
        """

        self.task_list.append(CameraMoveToTagStep(
            camera_source=camera_source,
            object_id=object_id,
            desired_tag_x=desired_tag_x,
            desired_tag_y=desired_tag_y,
            desired_tag_frame=desired_tag_frame,
            threshold_x_minus=threshold_x_minus,
            threshold_y_plus=threshold_y_plus,
            relative_yaw=relative_yaw,
            detection_timeout_sec=detection_timeout_sec,
            navigation_timeout_sec=navigation_timeout_sec,
            max_translation_m=max_translation_m,
            max_yaw_rad=max_yaw_rad,
            max_age_sec=max_age_sec,
            minimum_confidence=minimum_confidence,
            shift_toward_frame=shift_toward_frame,
            shift_toward_m=shift_toward_m,
        ))
        return self

    def rewind(self, repeat_count: Optional[int] = None) -> 'Task':
        """Repeat this Task from its first step.

        With no argument the Task repeats until stopped.  A positive integer
        is the total number of Task runs, including the first run.
        """

        if not self.task_list:
            raise ValueError('rewind requires at least one preceding Task step')
        if any(isinstance(command, RewindStep) for command in self.task_list):
            raise ValueError('Task can contain only one rewind')
        self.task_list.append(RewindStep(repeat_count=repeat_count))
        return self

    def camera_linear_absolute(
        self,
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
        camera_source: Optional[str] = None,
        detection_timeout_sec: float = 3.0,
        max_age_sec: Optional[float] = None,
        minimum_confidence: Optional[float] = None,
        reuse_previous_observation: bool = False,
        yaw_only: bool = False,
        yaw_symmetry_deg: Optional[float] = None,
        log_target_and_actual: bool = False,
        object_to_end_effector_position: Sequence[object] = (
            0.0,
            0.0,
            0.0,
        ),
        object_to_end_effector_orientation_xyzw: Sequence[object] = (
            0.0,
            0.0,
            0.0,
            1.0,
        ),
        impedance: ImpedanceSpec = None,
    ) -> 'Task':
        """Add a Cartesian target that will be captured during execution.

        The identity offset targets the tag pose exactly. Supply a calibrated
        object-to-end-effector transform for a real approach or grasp.

        Set ``reuse_previous_observation`` only on an immediately following
        camera step for the same object. This creates another absolute target
        from the first step's observation without detecting the object again.

        With ``yaw_only``, object roll and pitch are ignored: yaw rotates the
        X/Y offset, while the Z offset is added without rotation.

        ``yaw_symmetry_deg`` treats yaw values separated by that period as
        equivalent and lets the runner choose the one nearest the current EE
        yaw. It requires ``yaw_only=True`` and affects EE orientation only:
        the X/Y offset is always rotated by the detected object yaw, not by
        the selected equivalent EE yaw.

        Position control by default. ``impedance=True`` (the "default"
        preset), a preset name or a ``JointImpedance`` tracks the resolved
        target with joint impedance (arms only); a later position-controlled
        step of the same arm is then required in the same Task.
        """

        return self._append_camera_linear_absolute(
            CameraLinearAbsoluteStep,
            arm,
            object_id,
            tcp_motion,
            camera_source=camera_source,
            detection_timeout_sec=detection_timeout_sec,
            max_age_sec=max_age_sec,
            minimum_confidence=minimum_confidence,
            reuse_previous_observation=reuse_previous_observation,
            yaw_only=yaw_only,
            yaw_symmetry_deg=yaw_symmetry_deg,
            log_target_and_actual=log_target_and_actual,
            object_to_end_effector_position=(
                object_to_end_effector_position
            ),
            object_to_end_effector_orientation_xyzw=(
                object_to_end_effector_orientation_xyzw
            ),
            impedance=impedance,
        )

    def camera_linear_absolute_print(
        self,
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
        camera_source: Optional[str] = None,
        detection_timeout_sec: float = 3.0,
        max_age_sec: Optional[float] = None,
        minimum_confidence: Optional[float] = None,
        yaw_only: bool = False,
        yaw_symmetry_deg: Optional[float] = None,
        object_to_end_effector_position: Sequence[object] = (
            0.0,
            0.0,
            0.0,
        ),
        object_to_end_effector_orientation_xyzw: Sequence[object] = (
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    ) -> 'Task':
        """Print the resolved base-frame TCP target without moving the robot.

        Detection, TF lookup and object-to-end-effector composition are the
        same as :meth:`camera_linear_absolute`. Only backend motion submission
        is skipped by the planner runner.
        """

        return self._append_camera_linear_absolute(
            CameraLinearAbsolutePrintStep,
            arm,
            object_id,
            tcp_motion,
            camera_source=camera_source,
            detection_timeout_sec=detection_timeout_sec,
            max_age_sec=max_age_sec,
            minimum_confidence=minimum_confidence,
            reuse_previous_observation=False,
            yaw_only=yaw_only,
            yaw_symmetry_deg=yaw_symmetry_deg,
            log_target_and_actual=False,
            object_to_end_effector_position=(
                object_to_end_effector_position
            ),
            object_to_end_effector_orientation_xyzw=(
                object_to_end_effector_orientation_xyzw
            ),
        )

    def camera_frame_linear_absolute(
        self,
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
        camera_source: Optional[str] = None,
        controlled_frame: str,
        detection_timeout_sec: float = 3.0,
        max_age_sec: Optional[float] = None,
        minimum_confidence: Optional[float] = None,
        yaw_only: bool = True,
        preserve_end_effector_orientation: bool = True,
        log_target_and_actual: bool = False,
        object_to_controlled_frame_position: Sequence[object] = (
            0.0,
            0.0,
            0.0,
        ),
        object_to_controlled_frame_orientation_xyzw: Sequence[object] = (
            0.0,
            0.0,
            0.0,
            1.0,
        ),
    ) -> 'Task':
        """Move an EE so its mounted camera frame reaches the tag goal.

        With ``preserve_end_effector_orientation``, only the controlled
        camera-frame position is targeted and the current EE orientation is
        retained. The EE-to-camera mount transform is read from TF at runtime.
        """

        return self._append_camera_linear_absolute(
            CameraFrameLinearAbsoluteStep,
            arm,
            object_id,
            tcp_motion,
            camera_source=camera_source,
            detection_timeout_sec=detection_timeout_sec,
            max_age_sec=max_age_sec,
            minimum_confidence=minimum_confidence,
            reuse_previous_observation=False,
            yaw_only=yaw_only,
            yaw_symmetry_deg=None,
            log_target_and_actual=log_target_and_actual,
            object_to_end_effector_position=(
                object_to_controlled_frame_position
            ),
            object_to_end_effector_orientation_xyzw=(
                object_to_controlled_frame_orientation_xyzw
            ),
            controlled_frame=controlled_frame,
            preserve_end_effector_orientation=(
                preserve_end_effector_orientation
            ),
        )

    def _append_camera_linear_absolute(
        self,
        step_type: type[CameraLinearAbsoluteStep],
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
        camera_source: Optional[str],
        detection_timeout_sec: float,
        max_age_sec: Optional[float],
        minimum_confidence: Optional[float],
        reuse_previous_observation: bool,
        yaw_only: bool,
        yaw_symmetry_deg: Optional[float],
        log_target_and_actual: bool,
        object_to_end_effector_position: Sequence[object],
        object_to_end_effector_orientation_xyzw: Sequence[object],
        controlled_frame: Optional[str] = None,
        preserve_end_effector_orientation: bool = True,
        impedance: ImpedanceSpec = None,
    ) -> 'Task':
        probe = _CanonicalTask('_camera_command_validation')
        probe._linear(
            CommandKind.LINEAR_ABSOLUTE,
            arm,
            (0.0,) * 6,
            tcp_motion,
            impedance,
        )
        motion = probe.task_list[0]
        step_options: dict[str, object] = {}
        if controlled_frame is not None:
            step_options.update({
                'controlled_frame': controlled_frame,
                'preserve_end_effector_orientation': (
                    preserve_end_effector_orientation
                ),
            })
        self.task_list.append(step_type(
            group=str(motion.group),
            object_id=object_id,
            minimum_time=float(motion.minimum_time),
            linear_velocity=float(motion.linear_velocity),
            angular_velocity=float(motion.angular_velocity),
            acceleration_scaling=float(motion.acceleration_scaling),
            camera_source=wrist_camera_source(motion.group, camera_source),
            detection_timeout_sec=detection_timeout_sec,
            max_age_sec=max_age_sec,
            minimum_confidence=minimum_confidence,
            reuse_previous_observation=reuse_previous_observation,
            yaw_only=yaw_only,
            yaw_symmetry_deg=yaw_symmetry_deg,
            log_target_and_actual=log_target_and_actual,
            object_to_end_effector_position=tuple(
                object_to_end_effector_position
            ),
            object_to_end_effector_orientation_xyzw=tuple(
                object_to_end_effector_orientation_xyzw
            ),
            impedance=motion.impedance,
            **step_options,
        ))
        return self

    # ------------------------------------------------------------------
    # Camera contact patterns: tag-relative contact with impedance, an
    # optional gripper action, then a position-controlled retreat.
    # ``camera_options`` are the ``camera_linear_absolute`` keywords.
    # ------------------------------------------------------------------
    def _camera_contact(
        self,
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        impedance: ImpedanceSpec,
        camera_options: dict,
    ) -> 'Task':
        impedance = resolve_impedance(impedance)
        if impedance is None:
            raise ValueError(
                'camera contact helpers require impedance (True, a preset '
                'name or JointImpedance)'
            )
        if arm not in CARTESIAN_ARMS:
            raise ValueError(
                f'Cartesian impedance supports only {list(CARTESIAN_ARMS)}'
            )
        bound = impedance.bind(arm)
        self.camera_linear_absolute(arm, object_id, tcp_motion, **camera_options)
        self.task_list[-1] = replace(self.task_list[-1], impedance=bound)
        return self

    def camera_press(
        self,
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec,
        retreat: Sequence[object],
        **camera_options: object,
    ) -> 'Task':
        """Push toward a tag-relative target with impedance, then retreat."""

        self._camera_contact(arm, object_id, tcp_motion, impedance, camera_options)
        self.linear_relative(
            arm,
            tuple(_values(retreat, 3, 'retreat offset')) + (0.0, 0.0, 0.0),
            tcp_motion,
        )
        return self

    def camera_grasp(
        self,
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec,
        gripper_side: str,
        retreat: Sequence[object],
        gripper_speed: Optional[float] = None,
        **camera_options: object,
    ) -> 'Task':
        """Reach a tag-relative grasp pose compliantly, close and lift."""

        self._camera_contact(arm, object_id, tcp_motion, impedance, camera_options)
        self.close_gripper(gripper_side, gripper_speed)
        self.linear_relative(
            arm,
            tuple(_values(retreat, 3, 'retreat offset')) + (0.0, 0.0, 0.0),
            tcp_motion,
        )
        return self

    def camera_place_on_surface(
        self,
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec,
        gripper_side: str,
        retreat: Sequence[object],
        gripper_speed: Optional[float] = None,
        **camera_options: object,
    ) -> 'Task':
        """Lower a held object onto a tag-relative pose, release and lift."""

        self._camera_contact(arm, object_id, tcp_motion, impedance, camera_options)
        self.open_gripper(gripper_side, gripper_speed)
        self.linear_relative(
            arm,
            tuple(_values(retreat, 3, 'retreat offset')) + (0.0, 0.0, 0.0),
            tcp_motion,
        )
        return self

    def extend(
        self,
        other: Union[
            _CanonicalTask,
            TaskDefinition,
            DynamicTaskDefinition,
            Iterable[DynamicTaskStep],
        ],
    ) -> 'Task':
        if isinstance(other, _CanonicalTask):
            commands = other.task_list
        elif isinstance(other, (TaskDefinition, DynamicTaskDefinition)):
            commands = other.commands
        else:
            commands = list(other)
        if not all(
            isinstance(
                command,
                (
                    TaskCommand,
                    CameraLinearAbsoluteStep,
                    CameraMoveToTagStep,
                    MoveToStep,
                    VslamStep,
                    BackgroundGripperStep,
                    RewindStep,
                ),
            )
            for command in commands
        ):
            raise ValueError('extend accepts only planner Task commands')
        self.task_list.extend(commands)
        return self

    def build(self) -> RunnableTaskDefinition:
        commands = tuple(self.task_list)
        check_impedance_exit(commands)
        if any(
            isinstance(
                command,
                (
                    CameraLinearAbsoluteStep,
                    CameraMoveToTagStep,
                    MoveToStep,
                    VslamStep,
                    BackgroundGripperStep,
                    RewindStep,
                ),
            )
            for command in commands
        ):
            return DynamicTaskDefinition(
                name=self.name,
                description=self.description,
                commands=commands,
            )
        return TaskDefinition(
            name=self.name,
            description=self.description,
            commands=commands,
        )


def get_object_position(
    camera: 'CameraClient',
    object_id: str,
    *,
    max_age_sec: Optional[float] = None,
    minimum_confidence: Optional[float] = None,
    newer_than_ns: Optional[int] = None,
) -> CartesianTarget:
    """Return one fresh, Task-ready Cartesian object-position snapshot.

    The returned order and units match ``Task.linear_absolute`` exactly:
    ``(x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg)``. This helper never waits
    for perception; ``CameraClient`` raises ``ObservationUnavailable`` when a
    fresh detection or its TF is not currently available.
    """

    getter = getattr(camera, 'require_object_position', None)
    if not callable(getter):
        raise TypeError(
            'camera must provide require_object_position(object_id, ...)'
        )

    # TaskCommand does not carry a reference-frame field; the control backend
    # executes LINEAR_ABSOLUTE in base. Never let a camera-frame value cross
    # this boundary looking like a base-frame command.
    query: dict[str, object] = {'target_frame': 'base'}
    if max_age_sec is not None:
        query['max_age_sec'] = max_age_sec
    if minimum_confidence is not None:
        query['minimum_confidence'] = minimum_confidence
    if newer_than_ns is not None:
        query['newer_than_ns'] = newer_than_ns

    values = getter(object_id, **query)
    if isinstance(values, (str, bytes)):
        raise ValueError('camera Cartesian position must contain 6 numbers')
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            'camera Cartesian position must contain 6 finite numbers'
        ) from exc
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise ValueError(
            'camera Cartesian position must contain 6 finite numbers'
        )
    return cast(CartesianTarget, result)

def _load_task_settings() -> dict:
    """소스 작업 공간 또는 설치된 패키지에서 공통 설정을 매번 읽는다."""
    package_root = Path(__file__).resolve().parents[1]
    if (package_root / 'package.xml').is_file():
        # 소스/symlink 설치에서는 편집 중인 YAML을 그대로 사용한다.
        settings_path = package_root / 'config' / 'settings.yaml'
    else:
        from ament_index_python.packages import get_package_share_directory

        settings_path = (
            Path(get_package_share_directory('rby1_planner'))
            / 'config' / 'settings.yaml'
        )
    try:
        with settings_path.open(encoding='utf-8') as stream:
            settings = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'Cannot load planner settings: {settings_path}: {exc}') from exc
    if not isinstance(settings, dict):
        raise ValueError('settings.yaml must contain a mapping')
    return settings


def load_impedance_preset(name: str = DEFAULT_IMPEDANCE_PRESET) -> JointImpedance:
    """settings.yaml 의 impedance_presets 에서 이름으로 임피던스를 읽는다.

    값은 Reload Tasks 때마다 다시 읽힌다. stiffness 는 관절 수만큼의 벡터이며
    (팔 7개) 실제 사용하는 스텝의 그룹에 바인딩된다.
    """
    presets = _load_task_settings().get('impedance_presets')
    if not isinstance(presets, dict) or not presets:
        raise ValueError('settings.yaml has no impedance_presets section')
    key = str(name).strip()
    raw = presets.get(key)
    if not isinstance(raw, dict):
        raise ValueError(
            f'impedance preset {key!r} is not defined in settings.yaml '
            f'(available: {sorted(presets)})'
        )
    unknown = set(raw) - {'stiffness', 'damping_ratio', 'torque_limit', 'hold_time'}
    if unknown:
        raise ValueError(
            f'impedance preset {key!r} has unknown keys {sorted(unknown)}'
        )
    if 'stiffness' not in raw:
        raise ValueError(f'impedance preset {key!r} needs stiffness')
    try:
        return JointImpedance(
            stiffness=list(raw['stiffness']),
            damping_ratio=raw.get('damping_ratio', 1.0),
            torque_limit=raw.get('torque_limit', 10.0),
            hold_time=raw.get('hold_time', 0.0),
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(f'impedance preset {key!r}: {exc}') from exc


set_impedance_preset_loader(load_impedance_preset)


# ---------------------------------------------------------------------------
# 장비(stations) 설정 — config/settings.yaml
#
#   current_location: LH1          # Task 구성 시작 지점 (지점 참조, 생략 가능)
#   stations:
#     LH1:
#       kind: plain                # plain | door_switch
#       enabled: true              # 생략 시 true. false 면 이동/파지에 쓸 수 없다
#       points: {place: {position: [x, y], side: front|left|right}}
#                  side: fixed = 경유 지점 전용. 그 지점으로 갈 때 허리를 돌리지 않는다
#       place: {point: place, tag: tag_3, offset: [dx, dy, dz]}
#     ZS:
#       kind: door_switch
#       points: {work: {...}, via: {...}, place: {...}}
#       route: [work, via, place]  # 작업 위치 -> 놓는 위치 (복귀는 역순)
#       place: {...}
#       door: {open: {tag: tag_6, ...}, close: {tag: tag_7, ...}}
#       switch: {tag: tag_5, ...}
#
# 지점 참조: 'LH1' (= LH1 의 place 지점) 또는 'ZS.work' 처럼 '장비.지점'.
# 좌표는 모든 장비가 공통 좌표계 하나를 쓴다 (base 방향은 모든 지점에서 같다).
# 태그 ID 는 모든 장비를 통틀어 한 곳에서만 쓸 수 있다.
# ---------------------------------------------------------------------------
STATION_KINDS = ('plain', 'door_switch')
FIXED_SIDE = 'fixed'   # 경유 지점 전용: 도착할 때 허리 방향을 바꾸지 않는다
POINT_SIDES = ('left', 'right', 'front', FIXED_SIDE)


def _finite_list(value: object, count: int, what: str) -> tuple[float, ...]:
    if (
        not isinstance(value, list) or len(value) != count
        or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in value
        )
    ):
        raise ValueError(f'{what} must be a list of {count} finite numbers')
    return tuple(float(v) for v in value)


def _yaml_number(value: object, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{what} must be a finite number')
    return float(value)


# 지점에 head_pitch_deg 가 없을 때의 머리 pitch (deg).
POINT_HEAD_PITCH_DEFAULT_DEG = 0.0


def _head_pitch_value(value: object, what: str) -> float:
    """머리 pitch 절대각(deg). 관절 한계(±90°) 안이어야 한다."""
    pitch = _yaml_number(value, what)
    if not -90.0 < pitch < 90.0:
        raise ValueError(f'{what} must be between -90 and 90 degrees')
    return pitch


def _load_move_head_pitch_deg() -> float:
    """settings.yaml head_pitch_deg.move: 이동 중 머리 pitch 절대각(deg)을 매번 읽는다."""
    head = _load_task_settings().get('head_pitch_deg')
    if not isinstance(head, dict):
        raise ValueError('settings.yaml head_pitch_deg must be a mapping with move')
    return _head_pitch_value(head.get('move'), 'settings.yaml head_pitch_deg.move')


def _is_tag_id(value: object) -> bool:
    return (
        isinstance(value, str) and value.startswith('tag_')
        and value.removeprefix('tag_').isdigit()
    )


def _station_tags(name: str, station: dict) -> list[tuple[str, str]]:
    """(용도, 태그 ID) 목록. 용도 예: 'ZS.place', 'ZS.work.door.open'."""
    found: list[tuple[str, object]] = [(f'{name}.place', station['place'].get('tag'))]
    work = station.get('work')
    work = work if isinstance(work, dict) else {}
    door = work.get('door')
    if isinstance(door, dict):
        for action in ('open', 'close'):
            if isinstance(door.get(action), dict):
                found.append((f'{name}.work.door.{action}', door[action].get('tag')))
    switch = work.get('switch')
    if isinstance(switch, dict):
        found.append((f'{name}.work.switch', switch.get('tag')))
    for usage, tag in found:
        if tag is not None and not _is_tag_id(tag):
            raise ValueError(f'Station tag {usage} has invalid tag ID {tag!r}')
    return [(usage, tag) for usage, tag in found if tag is not None]


def _validate_station(name: object, station: object) -> None:
    if not isinstance(name, str) or not name.strip() or '.' in name:
        raise ValueError(f'settings.yaml station names must be nonempty strings without ".": {name!r}')
    if not isinstance(station, dict):
        raise ValueError(f'Station {name!r} must be a mapping')
    if station.get('kind') not in STATION_KINDS:
        raise ValueError(f'Station {name!r} kind must be one of {STATION_KINDS}')
    if not isinstance(station.get('enabled', True), bool):
        raise ValueError(f'Station {name!r} enabled must be true or false')
    points = station.get('points')
    if not isinstance(points, dict) or not points:
        raise ValueError(f'Station {name!r} points must be a nonempty mapping')
    for point_name, point in points.items():
        where = f'Location {name}.{point_name}'
        if not isinstance(point_name, str) or not point_name.strip() or '.' in point_name:
            raise ValueError(f'Station {name!r} point names must be nonempty strings without "."')
        if not isinstance(point, dict):
            raise ValueError(f'{where} must contain position and side')
        _finite_list(point.get('position'), 2, f'{where} position [x, y]')
        if point.get('side') not in POINT_SIDES:
            raise ValueError(f'{where} side must be left, right, front, or fixed')
        if 'head_pitch_deg' in point:   # 없으면 0° (POINT_HEAD_PITCH_DEFAULT_DEG)
            _head_pitch_value(point['head_pitch_deg'], f'{where} head_pitch_deg')
    place = station.get('place')
    if not isinstance(place, dict):
        raise ValueError(f'Station {name!r} place must be a mapping')
    if place.get('point') not in points:
        raise ValueError(f'Station {name!r} place.point must name one of its points')
    # 출입/작업 지점은 허리 방향이 정해져 있어야 한다 (fixed 는 경유 지점만).
    for point_name in (place['point'], 'work'):
        if point_name in points and points[point_name]['side'] == FIXED_SIDE:
            raise ValueError(
                f'Location {name}.{point_name} side cannot be fixed (waypoints only)'
            )
    if place.get('tag') is not None:
        _finite_list(place.get('offset'), 3, f'Station {name!r} place offset [x, y, z] in metres')
    if 'head_pitch_deg' in place:
        raise ValueError(
            f'Station {name!r} place.head_pitch_deg moved to points.<point>.head_pitch_deg'
        )
    if station['kind'] == 'door_switch':
        route = station.get('route')
        if (
            not isinstance(route, list) or len(route) < 2
            or any(p not in points for p in route)
        ):
            raise ValueError(f'Station {name!r} route must list at least two of its points')
        if route[-1] != place['point']:
            raise ValueError(f'Station {name!r} route must end at place.point')
        # work: work 지점에서 하는 문/스위치 작업 (door, switch)
        if 'work' not in points:
            raise ValueError(f'Station {name!r} door_switch needs a work point')
        work = station.get('work')
        if not isinstance(work, dict):
            raise ValueError(f'Station {name!r} work must be a mapping with door and switch')
        _yaml_number(work.get('wall_yaw_deg'), f'Station {name!r} work.wall_yaw_deg')
        door = work.get('door')
        if not isinstance(door, dict) or not all(isinstance(door.get(a), dict) for a in ('open', 'close')):
            raise ValueError(f'Station {name!r} work.door must contain open and close mappings')
        _finite_list(door.get('hinge_to_handle_wall'), 3, f'Station {name!r} work.door.hinge_to_handle_wall [x, y, z]')
        _yaml_number(door.get('swing_deg'), f'Station {name!r} work.door.swing_deg')
        for action in ('open', 'close'):
            target = door[action]
            if not _is_tag_id(target.get('tag')):
                raise ValueError(f'Station {name!r} work.door.{action}.tag must be a tag ID')
            _finite_list(target.get('base_align_xy'), 2, f'Station {name!r} work.door.{action}.base_align_xy [x, y]')
            _finite_list(target.get('ee_offset_tag'), 3, f'Station {name!r} work.door.{action}.ee_offset_tag [x, y, z]')
        switch = work.get('switch')
        if not isinstance(switch, dict):
            raise ValueError(f'Station {name!r} work.switch must be a mapping')
        if not _is_tag_id(switch.get('tag')):
            raise ValueError(f'Station {name!r} work.switch.tag must be a tag ID')
        if 'button_offset_in_tag' in switch:
            _finite_list(switch['button_offset_in_tag'], 2, f'Station {name!r} work.switch.button_offset_in_tag [x, y]')
        _yaml_number(switch.get('approach_standoff_m'), f'Station {name!r} work.switch.approach_standoff_m')
        if _yaml_number(switch.get('press_depth_m'), f'Station {name!r} work.switch.press_depth_m') <= 0.0:
            raise ValueError(f'Station {name!r} work.switch.press_depth_m must be positive')


def _load_move_locations() -> dict:
    """settings.yaml 의 stations 를 검증해 돌려준다.

    태그 ID 가 두 곳 이상에서 쓰이면 오류다 (태그 하나 = 용도 하나).
    """
    stations = _load_task_settings().get('stations')
    if not isinstance(stations, dict) or not stations:
        raise ValueError('settings.yaml stations must be a nonempty mapping')
    owners: dict[str, str] = {}
    for name, station in stations.items():
        _validate_station(name, station)
        for usage, tag in _station_tags(name, station):
            if tag in owners:
                raise ValueError(f'Tag {tag!r} is used by both {owners[tag]} and {usage}')
            owners[tag] = usage
    return stations


def _resolve_location(stations: dict, ref: object) -> tuple[str, str]:
    """지점 참조('LH1' 또는 'ZS.work')를 (장비, 지점) 이름으로 바꾼다."""
    if not isinstance(ref, str) or not ref.strip():
        raise ValueError('location name must be a nonempty string')
    station_name, _, point_name = ref.partition('.')
    station = stations.get(station_name)
    if station is None:
        raise ValueError(f'Unknown location {ref!r} in settings.yaml')
    if station.get('enabled', True) is not True:
        raise ValueError(
            f'Location {ref!r} is disabled until its position and tag are calibrated'
        )
    point_name = point_name or station['place']['point']
    if point_name not in station['points']:
        raise ValueError(f'Unknown location {ref!r} in settings.yaml')
    return station_name, point_name


def _load_station(name: str) -> dict:
    """설정을 읽어 장비 하나의 설정을 돌려준다 (비활성 장비는 오류)."""
    if not isinstance(name, str) or '.' in name:
        raise ValueError(f'station name expected (no point), got {name!r}')
    stations = _load_move_locations()
    station_name, _ = _resolve_location(stations, name)
    return stations[station_name]


def _move_location(stations: dict, ref: str) -> tuple[float, float, str]:
    """이동에 사용할 지점의 좌표(공통 좌표계)와 허리 방향."""
    station_name, point_name = _resolve_location(stations, ref)
    point = stations[station_name]['points'][point_name]
    x, y = (float(v) for v in point['position'])
    return x, y, point['side']


def _point_head_pitch(stations: dict, ref: str) -> float:
    """지점의 머리 pitch 절대각(deg): 그 지점에 도착해 작업할 때의 각도 (없으면 0°)."""
    station_name, point_name = _resolve_location(stations, ref)
    point = stations[station_name]['points'][point_name]
    return float(point.get('head_pitch_deg', POINT_HEAD_PITCH_DEFAULT_DEG))


def _load_point_head_pitch(ref: str) -> float:
    """설정을 읽어 지점('LH1' = place 지점, 'ZS.work')의 머리 pitch 절대각을 돌려준다."""
    return _point_head_pitch(_load_move_locations(), ref)


def _location_tag_id(stations: dict, ref: str) -> str:
    """지점의 집기/놓기 태그 ID. place 지점이 아니면(경유/작업 지점) 오류."""
    station_name, point_name = _resolve_location(stations, ref)
    place = stations[station_name]['place']
    if point_name != place['point']:
        raise ValueError(f'Location {ref!r} has no place tag (waypoint only)')
    if place.get('tag') is None:
        raise ValueError(f'Location {ref!r} has no place tag yet')
    return place['tag']


def _load_location_tag_id(ref: str) -> str:
    """설정을 읽어 지점의 집기/놓기 태그 ID 를 돌려준다."""
    return _location_tag_id(_load_move_locations(), ref)


def _load_current_location() -> Optional[str]:
    """settings.yaml 최상위 current_location (Task 구성 시작 지점). 파일은 바꾸지 않는다."""
    settings = _load_task_settings()
    stations = _load_move_locations()
    current_location = settings.get('current_location')
    if current_location is None:
        return None
    if not isinstance(current_location, str) or not current_location.strip():
        raise ValueError('settings.yaml current_location must be a location name')
    _resolve_location(stations, current_location)
    return current_location


def _load_tag_approach_offset(object_id: str) -> tuple[float, float, float]:
    """집기/놓기 태그에서 지점 중앙까지의 접근·접촉 공용 보정(m)을 읽는다.

    XY는 태그 축 기준으로 기록하고 기존 yaw_only=True 규칙에 따라
    실행 시 태그 yaw로 base 축에 변환한다. Z는 base 수직 방향이며,
    동일한 XYZ 보정을 접근·접촉 목표 양쪽에 적용한다.
    """
    if not isinstance(object_id, str) or not object_id.strip():
        raise ValueError('object_id must be a nonempty string')
    object_id = object_id.strip()
    for name, station in _load_move_locations().items():
        place = station['place']
        if place.get('tag') == object_id:
            return _finite_list(
                place.get('offset'), 3,
                f'Station {name!r} place offset [x, y, z] in metres',
            )
    raise ValueError(f'Unknown place tag {object_id!r} in settings.yaml')


__all__ = [
    'BODY_JOINT_GROUPS',
    'BackgroundGripperStep',
    'CARTESIAN_ARMS',
    'CARTESIAN_GROUPS',
    'CameraFrameLinearAbsoluteStep',
    'CameraLinearAbsolutePrintStep',
    'CameraLinearAbsoluteStep',
    'CameraMoveToTagStep',
    'Impedance',
    'JointImpedance',
    'JOINT_GROUP_DOF',
    'CommandKind',
    'DynamicTaskDefinition',
    'DynamicTaskStep',
    'MoveToStep',
    'VslamStep',
    'RewindStep',
    'RunnableTaskDefinition',
    'STREAM_PATH_KINDS',
    'Task',
    'TaskCommand',
    'TaskDefinition',
    'arc_waypoints',
    'check_impedance_exit',
    'get_object_position',
    'impedance_from_dict',
    'impedance_tag',
    'impedance_tag_for',
    'impedance_to_dict',
    'list_sum',
    'load_impedance_preset',
    'resolve_impedance',
]
