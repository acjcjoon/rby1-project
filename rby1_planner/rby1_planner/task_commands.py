"""Planner Task authoring with planner-only execution steps.

Ordinary motion commands use planner-owned protocol value objects. Camera and
odometry-navigation commands are planner-only descriptors
resolved by :class:`PlannerTaskRunner` during execution.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import (
    TYPE_CHECKING,
    Iterable,
    Optional,
    Sequence,
    Union,
    cast,
)

from .control_commands import (
    BODY_JOINT_GROUPS,
    CARTESIAN_ARMS,
    JOINT_GROUP_DOF,
    CommandKind,
    Task as _CanonicalTask,
    TaskCommand,
    TaskDefinition,
    list_sum,
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

    def __post_init__(self) -> None:
        group = str(self.group).strip()
        object_id = str(self.object_id).strip()
        if not object_id:
            raise ValueError('object_id must not be empty')

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
        if (
            yaw_symmetry_deg is not None
            and (
                abs(offset_position[0]) > 1.0e-12
                or abs(offset_position[1]) > 1.0e-12
            )
        ):
            raise ValueError(
                'yaw symmetry currently requires zero X/Y object offset'
            )

        # Reuse the canonical command validation for arm and motion limits.
        TaskCommand(
            kind=CommandKind.LINEAR_ABSOLUTE,
            group=group,
            values=(0.0,) * 6,
            minimum_time=minimum_time,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            acceleration_scaling=acceleration_scaling,
        )

        object.__setattr__(self, 'group', group)
        object.__setattr__(self, 'object_id', object_id)
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
        )

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


DynamicTaskStep = Union[
    TaskCommand,
    CameraLinearAbsoluteStep,
    CameraLinearAbsolutePrintStep,
    MoveToStep,
    RewindStep,
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
                    MoveToStep,
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
                (CameraLinearAbsoluteStep, MoveToStep, RewindStep),
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
        yaw. It currently requires ``yaw_only=True`` and a zero X/Y offset.
        """

        return self._append_camera_linear_absolute(
            CameraLinearAbsoluteStep,
            arm,
            object_id,
            tcp_motion,
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
        )

    def camera_linear_absolute_print(
        self,
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
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

    def _append_camera_linear_absolute(
        self,
        step_type: type[CameraLinearAbsoluteStep],
        arm: str,
        object_id: str,
        tcp_motion: Sequence[object],
        *,
        detection_timeout_sec: float,
        max_age_sec: Optional[float],
        minimum_confidence: Optional[float],
        reuse_previous_observation: bool,
        yaw_only: bool,
        yaw_symmetry_deg: Optional[float],
        log_target_and_actual: bool,
        object_to_end_effector_position: Sequence[object],
        object_to_end_effector_orientation_xyzw: Sequence[object],
    ) -> 'Task':
        probe = _CanonicalTask('_camera_command_validation')
        probe.linear_absolute(arm, (0.0,) * 6, tcp_motion)
        motion = probe.task_list[0]
        self.task_list.append(step_type(
            group=str(motion.group),
            object_id=object_id,
            minimum_time=float(motion.minimum_time),
            linear_velocity=float(motion.linear_velocity),
            angular_velocity=float(motion.angular_velocity),
            acceleration_scaling=float(motion.acceleration_scaling),
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
        ))
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
                    MoveToStep,
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
        if any(
            isinstance(
                command,
                (CameraLinearAbsoluteStep, MoveToStep, RewindStep),
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
    object_id: str = 'tag_4',
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


__all__ = [
    'BODY_JOINT_GROUPS',
    'CARTESIAN_ARMS',
    'CameraLinearAbsolutePrintStep',
    'CameraLinearAbsoluteStep',
    'JOINT_GROUP_DOF',
    'CommandKind',
    'DynamicTaskDefinition',
    'DynamicTaskStep',
    'MoveToStep',
    'RewindStep',
    'RunnableTaskDefinition',
    'Task',
    'TaskCommand',
    'TaskDefinition',
    'get_object_position',
    'list_sum',
]
