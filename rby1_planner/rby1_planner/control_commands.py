"""Small IDE-authored Task language for RB-Y1.

Task source uses operator-friendly units:
- joint positions: degrees
- Cartesian positions: metres
- Cartesian orientation: roll/pitch/yaw degrees in the base frame
- velocity/acceleration fields: the native rby1_msgs units
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple, Union


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


def arc_waypoints(
    radius_m: float,
    center_direction_deg: float,
    sweep_deg: float,
    step_deg: float,
    *,
    clockwise: bool = True,
    yaw_follows_arc: bool = False,
) -> list[Tuple[float, ...]]:
    """Return waypoints on a horizontal arc as offsets from the start pose.

    The arc centre lies ``radius_m`` from the start pose in the base-frame XY
    direction ``center_direction_deg`` (0 = +x, 90 = +y). The TCP sweeps
    ``sweep_deg`` about that centre in ``step_deg`` increments, clockwise when
    viewed from above (negative rotation about base +z) unless ``clockwise``
    is False. Every entry is ``[dx, dy, 0, 0, 0, dyaw]`` measured from the
    start pose, which is what ``Task.linear_path_relative`` expects. ``dyaw``
    follows the swept angle only when ``yaw_follows_arc`` is True; otherwise
    the orientation is kept fixed in the base frame.
    """

    radius = _number(radius_m, "radius_m", positive=True)
    sweep = _number(sweep_deg, "sweep_deg", positive=True)
    step = _number(step_deg, "step_deg", positive=True)
    center_direction = _number(center_direction_deg, "center_direction_deg")
    count = max(1, int(round(sweep / step)))
    actual_step = sweep / count
    sign = -1.0 if clockwise else 1.0
    theta0 = math.radians(center_direction + 180.0)

    def point(sweep_angle_deg: float) -> Tuple[float, float]:
        theta = theta0 + sign * math.radians(sweep_angle_deg)
        return radius * math.cos(theta), radius * math.sin(theta)

    origin = point(0.0)
    waypoints = []
    for index in range(1, count + 1):
        swept = index * actual_step
        current = point(swept)
        waypoints.append((
            current[0] - origin[0],
            current[1] - origin[1],
            0.0,
            0.0,
            0.0,
            sign * swept if yaw_follows_arc else 0.0,
        ))
    return waypoints


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


# ---------------------------------------------------------------------------
# Impedance by name.
#
# Task authoring may name an impedance instead of building one:
#   impedance=None / False   -> position control (default everywhere)
#   impedance=True           -> the "default" preset
#   impedance="contact"      -> the preset with that name
#   impedance=JointImpedance(...) -> exactly that value
# Presets live in rby1_planner/config/settings.yaml under
# ``impedance_presets`` and are re-read on every Reload Tasks. The loader is
# registered by task_commands so this module stays free of file access.
# ---------------------------------------------------------------------------
DEFAULT_IMPEDANCE_PRESET = "default"
ImpedanceSpec = Union[None, bool, str, JointImpedance]
_impedance_preset_loader = None


def set_impedance_preset_loader(loader) -> None:
    """Register ``loader(name) -> JointImpedance`` used for named presets."""

    global _impedance_preset_loader
    _impedance_preset_loader = loader


def resolve_impedance(spec: ImpedanceSpec) -> Optional[JointImpedance]:
    """Turn an authoring ``impedance=`` value into a JointImpedance or None."""

    if spec is None or spec is False:
        return None
    if isinstance(spec, JointImpedance):
        return spec
    if spec is True:
        name = DEFAULT_IMPEDANCE_PRESET
    elif isinstance(spec, str) and spec.strip():
        name = spec.strip()
    else:
        raise ValueError(
            "impedance must be None, True, a preset name or JointImpedance; "
            f"got {spec!r}"
        )
    if _impedance_preset_loader is None:
        raise ValueError(
            f"impedance preset {name!r} requested but no preset loader is "
            "registered"
        )
    impedance = _impedance_preset_loader(name)
    if not isinstance(impedance, JointImpedance):
        raise ValueError(f"impedance preset {name!r} is invalid")
    return impedance


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
                    "gripper velocity_limit",
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


@dataclass(frozen=True)
class TaskDefinition:
    name: str
    commands: Tuple[TaskCommand, ...]
    description: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("Task name must be nonempty")
        if not isinstance(self.description, str):
            raise ValueError("Task description must be a string")
        if not all(isinstance(command, TaskCommand) for command in self.commands):
            raise ValueError("Task commands are invalid")


class Task:
    """Mutable authoring helper; ``build()`` creates an immutable Task."""

    def __init__(self, name: str, description: str = "") -> None:
        self.name = str(name)
        self.description = str(description)
        self.task_list: list[TaskCommand] = []

    # robot commands
    def joint_absolute(
        self,
        group: str,
        positions: Sequence[object],
        joint_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec = None,
    ) -> "Task":
        """Move one joint group; ``impedance`` switches it to joint impedance.

        ``impedance`` accepts ``JointImpedance`` with a flat stiffness vector
        (one value per joint of ``group``); it is bound to ``group`` here.
        """

        return self._joint(
            CommandKind.JOINT_ABSOLUTE,
            group,
            positions,
            joint_motion,
            impedance,
        )

    def joint_relative(
        self,
        group: str,
        offsets: Sequence[object],
        joint_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec = None,
    ) -> "Task":
        return self._joint(
            CommandKind.JOINT_RELATIVE,
            group,
            offsets,
            joint_motion,
            impedance,
        )

    def joint_absolute_multi(
        self,
        targets: Mapping[str, Sequence[object]],
        joint_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec = None,
    ) -> "Task":
        """Move two or more body joint groups in one synchronized command.

        ``impedance`` must carry a ``{group: stiffness_vector}`` mapping for
        exactly the commanded groups.
        """

        if not isinstance(targets, Mapping) or len(targets) < 2:
            raise ValueError("targets must contain at least two joint groups")
        impedance = resolve_impedance(impedance)
        if impedance is not None and not impedance.is_bound:
            # A preset is one stiffness vector: apply it to every commanded
            # group (each group must have the same joint count).
            vector = impedance.stiffness[0][1]
            impedance = JointImpedance(
                stiffness={str(group): vector for group in targets},
                damping_ratio=impedance.damping_ratio,
                torque_limit=impedance.torque_limit,
                hold_time=impedance.hold_time,
            )
        if impedance is not None and not impedance.is_bound:
            raise ValueError(
                "multi-group joint impedance requires a {group: stiffness} "
                "mapping"
            )
        minimum_time, velocity_limit, acceleration_limit = _values(
            joint_motion,
            3,
            "Joint motion settings",
        )
        normalized = []
        for group, positions in targets.items():
            if group not in MULTI_JOINT_GROUPS:
                raise ValueError(
                    f"unsupported multi-group joint target: {group}"
                )
            normalized.append((
                group,
                _values(positions, JOINT_GROUP_DOF[group], f"{group} target"),
            ))
        self.task_list.append(TaskCommand(
            kind=CommandKind.JOINT_ABSOLUTE_MULTI,
            joint_targets=tuple(normalized),
            minimum_time=_number(minimum_time, "minimum_time", positive=True),
            velocity_limit=_number(
                velocity_limit,
                "velocity_limit",
                positive=True,
            ),
            acceleration_limit=_number(
                acceleration_limit,
                "acceleration_limit",
                positive=True,
            ),
            impedance=impedance,
        ))
        return self

    def whole_body_joint_absolute(
        self,
        *,
        torso: Optional[Sequence[object]] = None,
        right_arm: Optional[Sequence[object]] = None,
        left_arm: Optional[Sequence[object]] = None,
        head: Optional[Sequence[object]] = None,
        joint_motion: Sequence[object],
        impedance: ImpedanceSpec = None,
    ) -> "Task":
        """Move the specified body joint groups in one command.

        Omitted or None groups receive no new target. At least one group is
        required; multiple groups move together in one synchronized command.
        ``head`` rides along with the body; the control backend caps its
        velocity/acceleration to the head limits, and it cannot use impedance.
        """

        targets = {
            group: positions
            for group, positions in (
                ("torso", torso),
                ("right_arm", right_arm),
                ("left_arm", left_arm),
                ("head", head),
            )
            if positions is not None
        }
        if not targets:
            raise ValueError(
                "at least one of torso, right_arm, left_arm, or head must be "
                "provided"
            )
        if len(targets) == 1:
            group, positions = next(iter(targets.items()))
            return self.joint_absolute(
                group,
                positions,
                joint_motion,
                impedance=impedance,
            )
        return self.joint_absolute_multi(
            targets,
            joint_motion,
            impedance=impedance,
        )

    def _joint(
        self,
        kind: CommandKind,
        group: str,
        values: Sequence[object],
        joint_motion: Sequence[object],
        impedance: ImpedanceSpec = None,
    ) -> "Task":
        dof = JOINT_GROUP_DOF.get(group)
        if dof is None:
            raise ValueError(f"unknown joint group: {group}")
        impedance = resolve_impedance(impedance)
        if impedance is not None:
            impedance = impedance.bind(group)
        minimum_time, velocity_limit, acceleration_limit = _values(
            joint_motion,
            3,
            "Joint motion settings",
        )
        self.task_list.append(TaskCommand(
            kind=kind,
            group=group,
            values=_values(values, dof, f"{group} target"),
            minimum_time=_number(minimum_time, "minimum_time", positive=True),
            velocity_limit=_number(velocity_limit, "velocity_limit", positive=True),
            acceleration_limit=_number(
                acceleration_limit,
                "acceleration_limit",
                positive=True,
            ),
            impedance=impedance,
        ))
        return self

    def linear_absolute(
        self,
        arm: str,
        pose: Sequence[object],
        tcp_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec = None,
    ) -> "Task":
        """Move one TCP to a base-frame pose.

        Position control by default. ``impedance=True`` (the "default"
        preset), a preset name or a ``JointImpedance`` tracks the IK-solved
        joint target with joint impedance instead (arms only, within the
        control node's 10 cm / 20 deg contact caps). An impedance step must
        be followed later in the same Task by a position-controlled step of
        the same arm, for example a ``linear_relative`` retreat.

        ``arm`` is ``right_arm``, ``left_arm`` or ``torso`` (TCP = the torso's
        last link). A torso step moves only the torso joints; the arms keep
        their joint angles and travel with it.
        """

        return self._linear(
            CommandKind.LINEAR_ABSOLUTE,
            arm,
            pose,
            tcp_motion,
            impedance,
        )

    def linear_relative(
        self,
        arm: str,
        offset: Sequence[object],
        tcp_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec = None,
    ) -> "Task":
        """Move one TCP by a base-frame offset from its measured pose.

        Accepts the same groups as ``linear_absolute``; for example
        ``linear_relative('torso', (0, 0, 0.05, 0, 0, 0), tcp_motion)`` raises
        the torso TCP by 5 cm along base z.
        """

        return self._linear(
            CommandKind.LINEAR_RELATIVE,
            arm,
            offset,
            tcp_motion,
            impedance,
        )

    def linear_path_relative(
        self,
        arm: str,
        offsets: Sequence[Sequence[object]],
        path_motion: Sequence[object],
    ) -> "Task":
        """Stream one TCP along base-frame offsets from its measured pose.

        Each entry of ``offsets`` is ``[dx, dy, dz, droll, dpitch, dyaw]``
        from the pose measured when the step starts (not from the previous
        waypoint). ``path_motion`` is ``[total duration s, linear velocity
        limit m/s, angular velocity limit rad/s]``. The runner turns the
        driver stream on for this step only and off again afterwards.
        """

        return self._linear_path(
            CommandKind.LINEAR_PATH_STREAM_RELATIVE,
            arm,
            offsets,
            path_motion,
        )

    def linear_path_absolute(
        self,
        arm: str,
        poses: Sequence[Sequence[object]],
        path_motion: Sequence[object],
    ) -> "Task":
        """Stream one TCP through base-frame poses (see linear_path_relative)."""

        return self._linear_path(
            CommandKind.LINEAR_PATH_STREAM_ABSOLUTE,
            arm,
            poses,
            path_motion,
        )

    def _linear_path(
        self,
        kind: CommandKind,
        arm: str,
        waypoints: Sequence[Sequence[object]],
        path_motion: Sequence[object],
    ) -> "Task":
        duration, linear_velocity, angular_velocity = _values(
            path_motion,
            3,
            "path motion settings",
        )
        if isinstance(waypoints, (str, bytes)):
            raise ValueError("path waypoints must be a sequence of poses")
        normalized = tuple(
            _values(waypoint, 6, f"{arm} path waypoint")
            for waypoint in waypoints
        )
        if not normalized:
            raise ValueError("path requires at least one waypoint")
        self.task_list.append(TaskCommand(
            kind=kind,
            group=arm,
            waypoints=normalized,
            minimum_time=_number(duration, "path duration", positive=True),
            linear_velocity=_number(
                linear_velocity,
                "linear_velocity",
                positive=True,
            ),
            angular_velocity=_number(
                angular_velocity,
                "angular_velocity",
                positive=True,
            ),
        ))
        return self

    def _linear(
        self,
        kind: CommandKind,
        arm: str,
        values: Sequence[object],
        tcp_motion: Sequence[object],
        impedance: ImpedanceSpec = None,
    ) -> "Task":
        """Append a Cartesian step; ``impedance`` is reserved for the helpers.

        With ``impedance`` the control backend solves the arm IK for the TCP
        target and tracks the joint target with joint impedance (bounded by
        ``torque_limit``), then re-anchors to the measured joint state.
        """

        impedance = resolve_impedance(impedance)
        if impedance is not None:
            if arm not in CARTESIAN_ARMS:
                raise ValueError(
                    f"Cartesian impedance supports only {list(CARTESIAN_ARMS)}"
                )
            impedance = impedance.bind(arm)
        (
            minimum_time,
            linear_velocity,
            angular_velocity,
            acceleration_scaling,
        ) = _values(tcp_motion, 4, "TCP motion settings")
        self.task_list.append(TaskCommand(
            kind=kind,
            group=arm,
            values=_values(values, 6, f"{arm} TCP target"),
            minimum_time=_number(minimum_time, "minimum_time", positive=True),
            linear_velocity=_number(
                linear_velocity,
                "linear_velocity",
                positive=True,
            ),
            angular_velocity=_number(
                angular_velocity,
                "angular_velocity",
                positive=True,
            ),
            acceleration_scaling=_number(
                acceleration_scaling,
                "acceleration_scaling",
                positive=True,
            ),
            impedance=impedance,
        ))
        return self

    # ------------------------------------------------------------------
    # Contact patterns.  These are the only way to put impedance on a
    # Cartesian step: a short compliant move past the expected contact,
    # optionally a gripper action, then a position-controlled retreat so
    # the Task never ends pressed against a surface.
    # ------------------------------------------------------------------
    @staticmethod
    def _retreat_offset(
        offset: Sequence[object],
        retreat: Optional[Sequence[object]],
        retreat_margin_m: float,
    ) -> Tuple[float, float, float]:
        contact = _values(offset, 3, "contact offset")
        if retreat is not None:
            return _values(retreat, 3, "retreat offset")
        margin = _number(retreat_margin_m, "retreat_margin_m")
        if margin < 0.0:
            raise ValueError("retreat_margin_m must be nonnegative")
        depth = math.sqrt(sum(value * value for value in contact))
        if depth <= 0.0:
            raise ValueError("contact offset must not be zero")
        scale = -(depth + margin) / depth
        # ``+ 0.0`` folds -0.0 into 0.0 for readable previews.
        return tuple(value * scale + 0.0 for value in contact)

    def _contact_relative(
        self,
        arm: str,
        contact: Sequence[float],
        tcp_motion: Sequence[object],
        impedance: ImpedanceSpec,
    ) -> "Task":
        impedance = resolve_impedance(impedance)
        if impedance is None:
            raise ValueError(
                "contact helpers require impedance (True, a preset name or "
                "JointImpedance)"
            )
        return self._linear(
            CommandKind.LINEAR_RELATIVE,
            arm,
            tuple(contact) + (0.0, 0.0, 0.0),
            tcp_motion,
            impedance,
        )

    def press(
        self,
        arm: str,
        offset: Sequence[object],
        tcp_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec,
        retreat: Optional[Sequence[object]] = None,
        retreat_margin_m: float = 0.02,
    ) -> "Task":
        """Push along a base-frame offset with impedance, then retreat.

        ``offset`` is ``[dx, dy, dz]`` in metres from the measured TCP pose;
        it should reach slightly past the expected contact point. The move is
        tracked with ``impedance`` (force bounded by its ``torque_limit``),
        held for ``impedance.hold_time`` and followed by a position-controlled
        ``linear_relative`` retreat. The default retreat reverses ``offset``
        plus ``retreat_margin_m``.
        """

        contact = _values(offset, 3, "contact offset")
        exit_offset = self._retreat_offset(contact, retreat, retreat_margin_m)
        self._contact_relative(arm, contact, tcp_motion, impedance)
        self.linear_relative(
            arm,
            tuple(exit_offset) + (0.0, 0.0, 0.0),
            tcp_motion,
        )
        return self

    def grasp(
        self,
        arm: str,
        offset: Sequence[object],
        tcp_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec,
        gripper_side: str,
        gripper_speed: Optional[float] = None,
        retreat: Optional[Sequence[object]] = None,
        retreat_margin_m: float = 0.02,
    ) -> "Task":
        """Approach an object compliantly, close the gripper and lift."""

        contact = _values(offset, 3, "grasp offset")
        exit_offset = self._retreat_offset(contact, retreat, retreat_margin_m)
        self._contact_relative(arm, contact, tcp_motion, impedance)
        self.close_gripper(gripper_side, gripper_speed)
        self.linear_relative(
            arm,
            tuple(exit_offset) + (0.0, 0.0, 0.0),
            tcp_motion,
        )
        return self

    def place_on_surface(
        self,
        arm: str,
        descend: Sequence[object],
        tcp_motion: Sequence[object],
        *,
        impedance: ImpedanceSpec,
        gripper_side: str,
        gripper_speed: Optional[float] = None,
        retreat: Optional[Sequence[object]] = None,
        retreat_margin_m: float = 0.02,
    ) -> "Task":
        """Lower a held object onto a surface, release it and retreat."""

        contact = _values(descend, 3, "descend offset")
        exit_offset = self._retreat_offset(contact, retreat, retreat_margin_m)
        self._contact_relative(arm, contact, tcp_motion, impedance)
        self.open_gripper(gripper_side, gripper_speed)
        self.linear_relative(
            arm,
            tuple(exit_offset) + (0.0, 0.0, 0.0),
            tcp_motion,
        )
        return self

    def open_gripper(
        self,
        side: str = "both",
        speed: Optional[float] = None,
    ) -> "Task":
        """Open the selected gripper and wait for its position target."""

        self.task_list.append(TaskCommand(
            kind=CommandKind.GRIPPER_OPEN,
            group=str(side).strip().lower(),
            velocity_limit=(
                None
                if speed is None
                else _number(speed, "gripper speed", positive=True)
            ),
        ))
        return self

    def close_gripper(
        self,
        side: str = "both",
        speed: Optional[float] = None,
        *,
        settle_time_sec: float = DEFAULT_GRIPPER_SETTLE_TIME_SEC,
    ) -> "Task":
        """Close the gripper and complete after fresh feedback settles."""

        self.task_list.append(TaskCommand(
            kind=CommandKind.GRIPPER_CLOSE,
            group=str(side).strip().lower(),
            velocity_limit=(
                None
                if speed is None
                else _number(speed, "gripper speed", positive=True)
            ),
            seconds=_number(
                settle_time_sec,
                "gripper settle time",
                positive=True,
            ),
        ))
        return self

    def set_gripper(
        self,
        side: str,
        ratio: float,
        speed: Optional[float] = None,
        *,
        settle_time_sec: float = DEFAULT_GRIPPER_SETTLE_TIME_SEC,
    ) -> "Task":
        """Set a normalized close ratio and complete after fresh feedback."""

        self.task_list.append(TaskCommand(
            kind=CommandKind.GRIPPER_SET,
            group=str(side).strip().lower(),
            values=(_number(ratio, "gripper ratio"),),
            velocity_limit=(
                None
                if speed is None
                else _number(speed, "gripper speed", positive=True)
            ),
            seconds=_number(
                settle_time_sec,
                "gripper settle time",
                positive=True,
            ),
        ))
        return self

    # base commands
    def base_velocity(
            self,
            vx: float,
            vy: float,
            wz: float,
            seconds: float,
    ) -> "Task":
        self.task_list.append(TaskCommand(
            kind=CommandKind.BASE_VELOCITY,
            group="base",
            values=(
                _number(vx, "vx"),
                _number(vy, "vy"),
                _number(wz, "wz"),
            ),
            seconds=_number(seconds, "seconds", positive=True),
        ))

        return self

    def delay(self, milliseconds: float) -> "Task":
        seconds = _number(milliseconds, "milliseconds", positive=True) / 1000.0
        self.task_list.append(TaskCommand(kind=CommandKind.DELAY, seconds=seconds))
        return self

    def extend(
        self,
        other: Union["Task", TaskDefinition, Iterable[TaskCommand]],
    ) -> "Task":
        if isinstance(other, Task):
            commands = other.task_list
        elif isinstance(other, TaskDefinition):
            commands = other.commands
        else:
            commands = list(other)
        if not all(isinstance(command, TaskCommand) for command in commands):
            raise ValueError("extend accepts only Task commands")
        self.task_list.extend(commands)
        return self

    def build(self) -> TaskDefinition:
        check_impedance_exit(self.task_list)
        return TaskDefinition(
            name=self.name,
            description=self.description,
            commands=tuple(self.task_list),
        )


def default_catalog_path() -> Path:
    override = os.environ.get("RBY1_TASK_CATALOG")
    if override:
        return Path(override).expanduser()
    config_home = os.environ.get("XDG_CONFIG_HOME")
    root = Path(config_home).expanduser() if config_home else Path.home() / ".config"
    return root / "rby1_planner" / "task_catalog.json"


def _command_dict(command: TaskCommand) -> dict:
    payload = asdict(command)
    payload["kind"] = command.kind.value
    payload["values"] = list(command.values)
    if command.joint_targets:
        payload["joint_targets"] = {
            group: list(values)
            for group, values in command.joint_targets
        }
    else:
        payload.pop("joint_targets", None)
    payload["impedance"] = impedance_to_dict(command.impedance)
    if command.waypoints:
        payload["waypoints"] = [list(values) for values in command.waypoints]
    else:
        payload.pop("waypoints", None)
    return {key: value for key, value in payload.items() if value is not None}


def save_catalog(
        tasks: Mapping[str, TaskDefinition], path: Optional[Path] = None) -> Path:
    target = Path(path) if path is not None else default_catalog_path()
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "tasks": [
            {
                "name": task.name,
                "description": task.description,
                "commands": [_command_dict(command) for command in task.commands],
            }
            for _, task in sorted(tasks.items())
        ],
    }
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=str(target.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, target)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return target


def load_catalog(path: Optional[Path] = None) -> Dict[str, TaskDefinition]:
    target = Path(path) if path is not None else default_catalog_path()
    with target.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise ValueError("unsupported Task catalog schema")
    raw_tasks = payload.get("tasks")
    if not isinstance(raw_tasks, list):
        raise ValueError("Task catalog tasks must be a list")
    tasks: Dict[str, TaskDefinition] = {}
    for raw_task in raw_tasks:
        if not isinstance(raw_task, dict):
            raise ValueError("Task catalog entry must be an object")
        commands = []
        for raw_command in raw_task.get("commands", []):
            if not isinstance(raw_command, dict):
                raise ValueError("Task command must be an object")
            item = dict(raw_command)
            item["kind"] = CommandKind(item["kind"])
            item["values"] = tuple(item.get("values", ()))
            raw_joint_targets = item.get("joint_targets", {})
            if not isinstance(raw_joint_targets, dict):
                raise ValueError("Task multi-group joint targets must be an object")
            item["joint_targets"] = tuple(
                (group, tuple(values))
                for group, values in raw_joint_targets.items()
            )
            item["impedance"] = impedance_from_dict(item.get("impedance"))
            raw_waypoints = item.get("waypoints", ())
            if not isinstance(raw_waypoints, (list, tuple)):
                raise ValueError("Task path waypoints must be a list")
            item["waypoints"] = tuple(
                tuple(values) for values in raw_waypoints
            )
            commands.append(TaskCommand(**item))
        task = TaskDefinition(
            name=raw_task.get("name", ""),
            description=raw_task.get("description", ""),
            commands=tuple(commands),
        )
        if task.name in tasks:
            raise ValueError(f"duplicate Task name: {task.name}")
        tasks[task.name] = task
    return tasks


def delete_catalog_task(name: str, path: Optional[Path] = None) -> Dict[str, TaskDefinition]:
    tasks = load_catalog(path)
    if name not in tasks:
        raise KeyError(name)
    del tasks[name]
    save_catalog(tasks, path)
    return tasks


def compile_task_source(path: Optional[Path] = None) -> Dict[str, TaskDefinition]:
    """Reload ``task.py``, validate every Task and atomically save the catalog."""

    import importlib
    from . import task as task_source

    module = importlib.reload(task_source)
    authored = module.build_tasks()
    if not isinstance(authored, Mapping):
        raise ValueError("build_tasks() must return a name-to-Task mapping")
    result: Dict[str, TaskDefinition] = {}
    for name, value in authored.items():
        if not isinstance(name, str) or not name:
            raise ValueError("Task registry names must be nonempty strings")
        if isinstance(value, Task):
            definition = value.build()
        elif isinstance(value, TaskDefinition):
            definition = value
        else:
            raise ValueError(f"Task '{name}' is not a Task")
        if name != definition.name:
            raise ValueError(f"Task registry key '{name}' does not match '{definition.name}'")
        if name in result:
            raise ValueError(f"duplicate Task name: {name}")
        result[name] = definition
    save_catalog(result, path)
    return result
