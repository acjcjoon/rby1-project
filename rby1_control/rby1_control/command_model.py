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
from typing import Optional, Sequence, Tuple


JOINT_GROUP_DOF = {
    "right_arm": 7,
    "left_arm": 7,
    "torso": 6,
    "head": 2,
}
CARTESIAN_ARMS = ("right_arm", "left_arm")
BODY_JOINT_GROUPS = ("torso", "right_arm", "left_arm")
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

    def __post_init__(self) -> None:
        if not isinstance(self.kind, CommandKind):
            raise ValueError("Task command kind is invalid")

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

            if self.joint_targets or any(value is not None for value in (
                self.minimum_time,
                self.velocity_limit,
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
                if group not in BODY_JOINT_GROUPS:
                    raise ValueError(
                        f"unsupported multi-group joint target: {group}"
                    )
                if group in seen:
                    raise ValueError(
                        f"duplicate multi-group joint target: {group}"
                    )
                seen.add(group)
                _values(values, JOINT_GROUP_DOF[group], f"{group} target")
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

        if self.group not in CARTESIAN_ARMS:
            raise ValueError(f"unknown Cartesian arm: {self.group}")
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

    @property
    def timeout_seconds(self) -> float:
        if self.kind is CommandKind.DELAY:
            return float(self.seconds or 0.0) + 1.0
        if self.kind in (
            CommandKind.GRIPPER_CLOSE,
            CommandKind.GRIPPER_SET,
        ):
            return float(self.seconds or 0.0) + 2.0
        # Fallback for callers that do not have a fresh robot state.  The
        # TaskRunner refines this using the actual travel distance and the
        # command's velocity limits.
        return max(10.0, float(self.minimum_time or 0.0) * 3.0 + 5.0)


