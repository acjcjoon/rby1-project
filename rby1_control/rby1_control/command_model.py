"""Validated control-protocol command value objects.

Task source uses operator-friendly units:
- joint positions: degrees
- Cartesian positions: metres
- Cartesian orientation: roll/pitch/yaw degrees in the base frame
- velocity/acceleration fields: the native rby1_msgs units
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Dict, Mapping, Optional, Sequence, Tuple, Union


JOINT_GROUP_DOF = {
    "right_arm": 7,
    "left_arm": 7,
    "torso": 6,
    "head": 2,
}
CARTESIAN_ARMS = ("right_arm", "left_arm")
# Groups that accept a single Cartesian target (linear_absolute/relative).
# The torso TCP is its last link, commanded in the base frame like the arms.
# Streamed paths stay arm-only: the backend IK solves arm joints only.
CARTESIAN_GROUPS = CARTESIAN_ARMS + ("torso",)
BODY_JOINT_GROUPS = ("torso", "right_arm", "left_arm")
# Groups one JOINT_ABSOLUTE_MULTI command may carry (head moves with the body).
MULTI_JOINT_GROUPS = BODY_JOINT_GROUPS + ("head",)
GRIPPER_SIDES = ("right", "left", "both")
DEFAULT_GRIPPER_SETTLE_TIME_SEC = 1.0


class CommandKind(str, Enum):
    JOINT_ABSOLUTE = "joint_absolute"
    JOINT_ABSOLUTE_MULTI = "joint_absolute_multi"
    JOINT_RELATIVE = "joint_relative"
    LINEAR_ABSOLUTE = "linear_absolute"
    LINEAR_RELATIVE = "linear_relative"
    BASE_VELOCITY = "base_velocity"
    GRIPPER_OPEN = "gripper_open"
    GRIPPER_CLOSE = "gripper_close"
    GRIPPER_SET = "gripper_set"
    DELAY = "delay"
    # Streamed Cartesian path: one TCP follows a waypoint list through the
    # driver's persistent stream (stream_cartesian). The relative form is
    # authored; the Task runner resolves it to the absolute form once, from
    # the pose measured when the step starts.
    LINEAR_PATH_STREAM_RELATIVE = "linear_path_stream_relative"
    LINEAR_PATH_STREAM_ABSOLUTE = "linear_path_stream_absolute"


STREAM_PATH_KINDS = (
    CommandKind.LINEAR_PATH_STREAM_RELATIVE,
    CommandKind.LINEAR_PATH_STREAM_ABSOLUTE,
)


def _number(value: object, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a finite number") from exc
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "positive " if positive else ""
        raise ValueError(f"{label} must be a {qualifier}finite number")
    return result


def _values(values: Sequence[object], count: int, label: str) -> Tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{label} must contain {count} numeric values")
    result = tuple(_number(value, label) for value in values)
    if len(result) != count:
        raise ValueError(f"{label} requires {count} values; got {len(result)}")
    return result


def list_sum(left: Sequence[object], right: Sequence[object]) -> list[float]:
    """Element-wise addition with strict length and numeric validation."""

    if len(left) != len(right):
        raise ValueError("list_sum operands must have the same length")
    return [
        _number(a, "list_sum value") + _number(b, "list_sum value")
        for a, b in zip(left, right)
    ]


# ---------------------------------------------------------------------------
# Impedance control parameters
#
# A joint Task step is position-controlled unless it carries a
# ``JointImpedance`` value. The control backend maps it onto the driver's
# ``JointCommand.use_impedance`` path and, after the command finishes,
# re-anchors the controller reference to the measured joint state. Cartesian
# steps are always position-controlled.
# ---------------------------------------------------------------------------

UNBOUND_IMPEDANCE_GROUP = ""
MAX_JOINT_STIFFNESS = 1000.0
MAX_JOINT_TORQUE_LIMIT = 50.0
MAX_DAMPING_RATIO = 5.0
# Extra time the controller keeps the impedance target after the
# trajectory ends (JointCommand.control_hold_time) before completing.
MAX_IMPEDANCE_HOLD_TIME_SEC = 10.0


def _bounded_values(
    values: Sequence[object],
    count: int,
    label: str,
    *,
    maximum: float,
) -> Tuple[float, ...]:
    result = _values(values, count, label)
    for value in result:
        if value < 0.0 or value > maximum:
            raise ValueError(
                f"{label} values must be within [0, {maximum:g}]"
            )
    return result


@dataclass(frozen=True)
class JointImpedance:
    """Joint-space impedance parameters for one or more joint groups.

    The driver tracks the planned trajectory with
    ``tau = K (q_d - q) + D (qd_d - qd) + gravity`` and clips each joint
    torque to ``torque_limit``. ``stiffness`` is always stored per group as a
    vector with one value per joint. Authoring may pass a flat vector for a
    single-group command; the Task builder binds it to that command's group.
    A mapping ``{group: vector}`` is required for multi-group commands.

    ``hold_time`` keeps the impedance target active for that many seconds
    after the trajectory ends (a contact dwell); the backend then re-anchors.

    The same value is accepted by the Cartesian contact helpers: the control
    backend solves the arm IK for the TCP target and tracks the resulting
    joint target with this joint impedance.
    """

    stiffness: Tuple[Tuple[str, Tuple[float, ...]], ...]
    damping_ratio: float = 1.0
    torque_limit: float = 10.0
    hold_time: float = 0.0

    def __post_init__(self) -> None:
        raw = self.stiffness
        if isinstance(raw, Mapping):
            items = []
            for group, values in raw.items():
                group_name = str(group)
                dof = JOINT_GROUP_DOF.get(group_name)
                if dof is None:
                    raise ValueError(f"unknown joint group: {group_name}")
                items.append((
                    group_name,
                    _bounded_values(
                        values,
                        dof,
                        f"{group_name} stiffness",
                        maximum=MAX_JOINT_STIFFNESS,
                    ),
                ))
            if not items:
                raise ValueError("joint impedance stiffness is empty")
            normalized = tuple(items)
        elif (
            isinstance(raw, (tuple, list))
            and raw
            and all(
                isinstance(item, tuple)
                and len(item) == 2
                and isinstance(item[0], str)
                for item in raw
            )
        ):
            items = []
            for group_name, values in raw:
                if group_name == UNBOUND_IMPEDANCE_GROUP:
                    if len(raw) != 1:
                        raise ValueError(
                            "unbound joint impedance must contain one vector"
                        )
                    vector = tuple(
                        _number(value, "stiffness") for value in values
                    )
                    if not vector:
                        raise ValueError("joint impedance stiffness is empty")
                    for value in vector:
                        if value < 0.0 or value > MAX_JOINT_STIFFNESS:
                            raise ValueError(
                                "stiffness values must be within "
                                f"[0, {MAX_JOINT_STIFFNESS:g}]"
                            )
                    items.append((group_name, vector))
                    continue
                dof = JOINT_GROUP_DOF.get(group_name)
                if dof is None:
                    raise ValueError(f"unknown joint group: {group_name}")
                items.append((
                    group_name,
                    _bounded_values(
                        values,
                        dof,
                        f"{group_name} stiffness",
                        maximum=MAX_JOINT_STIFFNESS,
                    ),
                ))
            normalized = tuple(items)
        else:
            if isinstance(raw, (str, bytes)):
                raise ValueError("stiffness must be a vector or a mapping")
            vector = tuple(_number(value, "stiffness") for value in raw)
            if not vector:
                raise ValueError("joint impedance stiffness is empty")
            for value in vector:
                if value < 0.0 or value > MAX_JOINT_STIFFNESS:
                    raise ValueError(
                        "stiffness values must be within "
                        f"[0, {MAX_JOINT_STIFFNESS:g}]"
                    )
            normalized = ((UNBOUND_IMPEDANCE_GROUP, vector),)

        seen = set()
        for group_name, _ in normalized:
            if group_name in seen:
                raise ValueError(f"duplicate stiffness group: {group_name}")
            seen.add(group_name)

        damping_ratio = _number(self.damping_ratio, "damping_ratio", positive=True)
        if damping_ratio > MAX_DAMPING_RATIO:
            raise ValueError(
                f"damping_ratio cannot exceed {MAX_DAMPING_RATIO:g}"
            )
        torque_limit = _number(self.torque_limit, "torque_limit", positive=True)
        if torque_limit > MAX_JOINT_TORQUE_LIMIT:
            raise ValueError(
                f"torque_limit cannot exceed {MAX_JOINT_TORQUE_LIMIT:g} Nm"
            )
        hold_time = _number(self.hold_time, "hold_time")
        if hold_time < 0.0 or hold_time > MAX_IMPEDANCE_HOLD_TIME_SEC:
            raise ValueError(
                "hold_time must be within "
                f"[0, {MAX_IMPEDANCE_HOLD_TIME_SEC:g}] s"
            )

        object.__setattr__(self, "stiffness", normalized)
        object.__setattr__(self, "damping_ratio", damping_ratio)
        object.__setattr__(self, "torque_limit", torque_limit)
        object.__setattr__(self, "hold_time", hold_time)

    @property
    def is_bound(self) -> bool:
        return all(
            group != UNBOUND_IMPEDANCE_GROUP for group, _ in self.stiffness
        )

    @property
    def groups(self) -> Tuple[str, ...]:
        return tuple(group for group, _ in self.stiffness)

    def stiffness_for(self, group: str) -> Tuple[float, ...]:
        for name, values in self.stiffness:
            if name == group:
                return values
        raise KeyError(group)

    def bind(self, group: str) -> "JointImpedance":
        """Return this impedance bound to one joint group."""

        dof = JOINT_GROUP_DOF.get(group)
        if dof is None:
            raise ValueError(f"unknown joint group: {group}")
        if self.is_bound:
            if self.groups != (group,):
                raise ValueError(
                    f"joint impedance is bound to {list(self.groups)}, "
                    f"not to {group!r}"
                )
            return self
        vector = self.stiffness[0][1]
        if len(vector) != dof:
            raise ValueError(
                f"{group} stiffness requires {dof} values; got {len(vector)}"
            )
        return JointImpedance(
            stiffness={group: vector},
            damping_ratio=self.damping_ratio,
            torque_limit=self.torque_limit,
            hold_time=self.hold_time,
        )



# Joint impedance is the only impedance type. Cartesian steps carry it too:
# the control backend solves the arm IK and tracks the joint target with it,
# so the driver's trajectory-less Cartesian spring builder is never used.
Impedance = JointImpedance


def impedance_to_dict(impedance: Optional[Impedance]) -> Optional[dict]:
    """Wire representation of an impedance value (``None`` stays ``None``)."""

    if impedance is None:
        return None
    if isinstance(impedance, JointImpedance):
        if not impedance.is_bound:
            raise ValueError("joint impedance must be bound to a joint group")
        return {
            "type": "joint",
            "stiffness": {
                group: list(values) for group, values in impedance.stiffness
            },
            "damping_ratio": impedance.damping_ratio,
            "torque_limit": impedance.torque_limit,
            "hold_time": impedance.hold_time,
        }
    raise TypeError("impedance must be JointImpedance")


def impedance_from_dict(payload: object) -> Optional[Impedance]:
    """Parse the wire representation produced by :func:`impedance_to_dict`."""

    if payload is None:
        return None
    if not isinstance(payload, Mapping):
        raise ValueError("impedance payload must be an object")
    kind = str(payload.get("type", ""))
    if kind == "joint":
        stiffness = payload.get("stiffness")
        if not isinstance(stiffness, Mapping):
            raise ValueError("joint impedance stiffness must be an object")
        return JointImpedance(
            stiffness={
                str(group): tuple(values) for group, values in stiffness.items()
            },
            damping_ratio=payload.get("damping_ratio", 1.0),
            torque_limit=payload.get("torque_limit", 10.0),
            hold_time=payload.get("hold_time", 0.0),
        )
    raise ValueError(f"unknown impedance type: {kind!r}")


def impedance_tag(impedance: Optional[Impedance]) -> str:
    """Short label used in UI previews and runner status lines."""

    if isinstance(impedance, JointImpedance):
        return "JS impedance"
    return ""


def impedance_tag_for(kind: object, impedance: Optional[Impedance]) -> str:
    """Tag by command kind: Cartesian steps run their impedance via IK."""

    if impedance is None:
        return ""
    if kind in (CommandKind.LINEAR_ABSOLUTE, CommandKind.LINEAR_RELATIVE):
        return "CS impedance"
    return impedance_tag(impedance)


def impedance_exit_role(step: object) -> Tuple[str, Tuple[str, ...]]:
    """Classify a Task step for the contact-exit rule.

    Returns ``("contact", groups)`` for Cartesian impedance steps,
    ``("position", groups)`` for every other arm/joint motion and
    ``("other", ())`` for steps that do not move the manipulator.
    Planner-only steps provide the same answer through an
    ``impedance_exit_role()`` method.
    """

    custom = getattr(step, "impedance_exit_role", None)
    if callable(custom) and not isinstance(step, TaskCommand):
        role, groups = custom()
        return str(role), tuple(str(group) for group in groups)
    if not isinstance(step, TaskCommand):
        return ("other", ())
    if step.kind is CommandKind.JOINT_ABSOLUTE_MULTI:
        return ("position", tuple(group for group, _ in step.joint_targets))
    if step.kind in (CommandKind.LINEAR_ABSOLUTE, CommandKind.LINEAR_RELATIVE):
        if step.impedance is not None:
            return ("contact", (str(step.group),))
        return ("position", (str(step.group),))
    if step.kind in (CommandKind.JOINT_ABSOLUTE, CommandKind.JOINT_RELATIVE):
        return ("position", (str(step.group),))
    if step.kind in STREAM_PATH_KINDS:
        return ("position", (str(step.group),))
    return ("other", ())


def check_impedance_exit(steps: Sequence[object]) -> None:
    """Require a position-controlled exit after a Cartesian impedance contact.

    After a contact step the controller is re-anchored to the measured pose,
    but the arm is still touching the surface. Every arm that ran a Cartesian
    impedance step must therefore end the Task with at least one later
    position-controlled step (for example a ``linear_relative`` retreat).
    """

    last_contact: Dict[str, int] = {}
    last_position: Dict[str, int] = {}
    for index, step in enumerate(steps):
        role, groups = impedance_exit_role(step)
        if role == "contact":
            for group in groups:
                last_contact[group] = index
        elif role == "position":
            for group in groups:
                last_position[group] = index
    for group, contact_index in sorted(last_contact.items()):
        exit_index = last_position.get(group)
        if exit_index is None or exit_index < contact_index:
            raise ValueError(
                f"Cartesian impedance step {contact_index + 1} ({group}) must "
                f"be followed by a position-controlled {group} step "
                "(for example a linear_relative retreat) before the Task ends"
            )


@dataclass(frozen=True)
class TaskCommand:
    kind: CommandKind
    group: Optional[str] = None
    values: Tuple[float, ...] = ()
    joint_targets: Tuple[Tuple[str, Tuple[float, ...]], ...] = ()
    minimum_time: Optional[float] = None
    velocity_limit: Optional[float] = None
    acceleration_limit: Optional[float] = None
    linear_velocity: Optional[float] = None
    angular_velocity: Optional[float] = None
    acceleration_scaling: Optional[float] = None
    seconds: Optional[float] = None
    impedance: Optional[Impedance] = None
    # Stream path only: ordered TCP poses [x, y, z, roll, pitch, yaw]. For the
    # relative kind each entry is an offset from the start pose (not from the
    # previous waypoint); for the absolute kind each entry is a base-frame pose.
    waypoints: Tuple[Tuple[float, ...], ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.kind, CommandKind):
            raise ValueError("Task command kind is invalid")
        self._validate_impedance()

        if self.kind in STREAM_PATH_KINDS:
            self._validate_stream_path()
            return
        if self.waypoints:
            raise ValueError("only stream path commands carry waypoints")

        if self.kind is CommandKind.DELAY:
            _number(self.seconds, "seconds", positive=True)
            if (
                self.group is not None
                or self.values
                or self.joint_targets
                or any(value is not None for value in (
                    self.minimum_time,
                    self.velocity_limit,
                    self.acceleration_limit,
                    self.linear_velocity,
                    self.angular_velocity,
                    self.acceleration_scaling,
                ))
            ):
                raise ValueError("delay cannot contain a motion target")
            return

        if self.kind is CommandKind.BASE_VELOCITY:
            if self.group != "base":
                raise ValueError("base velocity command group must be 'base'")

            _values(self.values, 3, "base velocity")
            _number(self.seconds, "seconds", positive=True)

            if self.joint_targets:
                raise ValueError(
                    "base velocity command cannot contain joint targets"
            )

            if any(value is not None for value in (
                self.minimum_time,
                self.velocity_limit,
                self.acceleration_limit,
                self.linear_velocity,
                self.angular_velocity,
                self.acceleration_scaling,
            )):
                raise ValueError(
                    "base velocity command contains incompatible fields"
                )

            return

        if self.kind in (
            CommandKind.GRIPPER_OPEN,
            CommandKind.GRIPPER_CLOSE,
            CommandKind.GRIPPER_SET,
        ):
            if self.group not in GRIPPER_SIDES:
                raise ValueError(
                    "gripper side must be 'right', 'left', or 'both'"
                )
            if self.kind is CommandKind.GRIPPER_SET:
                ratio = _values(self.values, 1, "gripper ratio")[0]
                if not 0.0 <= ratio <= 1.0:
                    raise ValueError("gripper ratio must be within [0.0, 1.0]")
            elif self.values:
                raise ValueError("gripper open/close cannot contain a ratio")

            if self.kind is CommandKind.GRIPPER_OPEN:
                if self.seconds is not None:
                    raise ValueError("gripper open does not use settle time")
            else:
                _number(self.seconds, "gripper settle time", positive=True)

            if self.velocity_limit is not None:
                _number(
                    self.velocity_limit,
                    "velocity_limit",
                    positive=True,
                )

            if self.joint_targets or any(value is not None for value in (
                self.minimum_time,
                self.acceleration_limit,
                self.linear_velocity,
                self.angular_velocity,
                self.acceleration_scaling,
            )):
                raise ValueError(
                    "gripper command contains incompatible motion fields"
                )
            return

        if (
            self.kind is not CommandKind.JOINT_ABSOLUTE_MULTI
            and self.group is None
        ):
            raise ValueError("motion command group is missing")
        _number(self.minimum_time, "minimum_time", positive=True)

        if self.kind in (CommandKind.JOINT_ABSOLUTE, CommandKind.JOINT_RELATIVE):
            if self.joint_targets:
                raise ValueError(
                    "single-group joint command contains multiple targets"
                )
            dof = JOINT_GROUP_DOF.get(self.group)
            if dof is None:
                raise ValueError(f"unknown joint group: {self.group}")
            _values(self.values, dof, f"{self.group} target")
            _number(self.velocity_limit, "velocity_limit", positive=True)
            _number(self.acceleration_limit, "acceleration_limit", positive=True)
            if any(value is not None for value in (
                self.linear_velocity,
                self.angular_velocity,
                self.acceleration_scaling,
                self.seconds,
            )):
                raise ValueError("joint command contains Cartesian/delay fields")
            return

        if self.kind is CommandKind.JOINT_ABSOLUTE_MULTI:
            if self.group is not None or self.values:
                raise ValueError("multi-group joint command contains a single target")
            if (
                not isinstance(self.joint_targets, tuple)
                or len(self.joint_targets) < 2
            ):
                raise ValueError(
                    "multi-group joint command requires at least two targets"
                )
            seen = set()
            for item in self.joint_targets:
                if not isinstance(item, tuple) or len(item) != 2:
                    raise ValueError("multi-group joint target is invalid")
                group, values = item
                if group not in MULTI_JOINT_GROUPS:
                    raise ValueError(
                        f"unsupported multi-group joint target: {group}"
                    )
                if group in seen:
                    raise ValueError(
                        f"duplicate multi-group joint target: {group}"
                    )
                seen.add(group)
                _values(values, JOINT_GROUP_DOF[group], f"{group} target")
            if self.impedance is not None and "head" in seen:
                raise ValueError("head cannot use joint impedance")
            _number(self.velocity_limit, "velocity_limit", positive=True)
            _number(self.acceleration_limit, "acceleration_limit", positive=True)
            if any(value is not None for value in (
                self.linear_velocity,
                self.angular_velocity,
                self.acceleration_scaling,
                self.seconds,
            )):
                raise ValueError(
                    "multi-group joint command contains Cartesian/delay fields"
                )
            return

        if self.group not in CARTESIAN_GROUPS:
            raise ValueError(f"unknown Cartesian group: {self.group}")
        if self.joint_targets:
            raise ValueError("Cartesian command contains joint targets")
        _values(self.values, 6, f"{self.group} TCP target")
        _number(self.linear_velocity, "linear_velocity", positive=True)
        _number(self.angular_velocity, "angular_velocity", positive=True)
        scaling = _number(
            self.acceleration_scaling,
            "acceleration_scaling",
            positive=True,
        )
        if scaling > 1.0:
            raise ValueError("acceleration_scaling cannot exceed 1.0")
        if any(value is not None for value in (
            self.velocity_limit,
            self.acceleration_limit,
            self.seconds,
        )):
            raise ValueError("Cartesian command contains joint/delay fields")

    def _validate_stream_path(self) -> None:
        if self.group not in CARTESIAN_ARMS:
            raise ValueError(f"unknown Cartesian arm: {self.group}")
        if self.values or self.joint_targets:
            raise ValueError(
                "stream path command carries waypoints, not a single target"
            )
        if isinstance(self.waypoints, (str, bytes)) or not self.waypoints:
            raise ValueError("stream path requires at least one waypoint")
        normalized = tuple(
            _values(waypoint, 6, f"{self.group} path waypoint")
            for waypoint in self.waypoints
        )
        object.__setattr__(self, "waypoints", normalized)
        # minimum_time is the total path duration; the velocity fields are
        # caps that the control backend enforces on the generated path.
        _number(self.minimum_time, "minimum_time", positive=True)
        _number(self.linear_velocity, "linear_velocity", positive=True)
        _number(self.angular_velocity, "angular_velocity", positive=True)
        if any(value is not None for value in (
            self.velocity_limit,
            self.acceleration_limit,
            self.acceleration_scaling,
            self.seconds,
        )):
            raise ValueError("stream path command contains joint/delay fields")

    def _validate_impedance(self) -> None:
        impedance = self.impedance
        if impedance is None:
            return
        if self.kind in (
            CommandKind.JOINT_ABSOLUTE,
            CommandKind.JOINT_RELATIVE,
            CommandKind.JOINT_ABSOLUTE_MULTI,
        ):
            if not isinstance(impedance, JointImpedance):
                raise ValueError(
                    "joint commands accept only JointImpedance"
                )
            if not impedance.is_bound:
                raise ValueError(
                    "joint impedance must be bound to the command's group(s)"
                )
            if self.kind is CommandKind.JOINT_ABSOLUTE_MULTI:
                expected = tuple(group for group, _ in self.joint_targets)
            else:
                expected = (str(self.group),)
            if sorted(impedance.groups) != sorted(expected):
                raise ValueError(
                    f"joint impedance groups {list(impedance.groups)} do not "
                    f"match command groups {list(expected)}"
                )
            return
        if self.kind in (
            CommandKind.LINEAR_ABSOLUTE,
            CommandKind.LINEAR_RELATIVE,
        ):
            # The control backend solves the arm IK for the TCP target and
            # tracks the joint target with this joint impedance.
            if not isinstance(impedance, JointImpedance):
                raise ValueError(
                    "Cartesian commands accept only JointImpedance"
                )
            arm = str(self.group)
            if arm not in CARTESIAN_ARMS:
                raise ValueError(
                    f"Cartesian impedance supports only {list(CARTESIAN_ARMS)}, "
                    f"not {arm}"
                )
            if not impedance.is_bound or impedance.groups != (arm,):
                raise ValueError(
                    f"Cartesian impedance must be bound to {arm}"
                )
            return
        raise ValueError(f"{self.kind.value} does not support impedance")

    @property
    def uses_impedance(self) -> bool:
        return self.impedance is not None

    @property
    def timeout_seconds(self) -> float:
        if self.kind is CommandKind.DELAY:
            return float(self.seconds or 0.0) + 1.0
        if self.kind in (
            CommandKind.GRIPPER_CLOSE,
            CommandKind.GRIPPER_SET,
        ):
            return float(self.seconds or 0.0) + 2.0
        if self.kind in STREAM_PATH_KINDS:
            # Total path duration plus stream enable/disable transitions.
            return float(self.minimum_time or 0.0) * 1.5 + 10.0
        # Fallback for callers that do not have a fresh robot state.  The
        # TaskRunner refines this using the actual travel distance and the
        # command's velocity limits.
        hold_time = (
            float(self.impedance.hold_time)
            if self.impedance is not None
            else 0.0
        )
        return max(
            10.0,
            float(self.minimum_time or 0.0) * 3.0 + hold_time + 5.0,
        )
