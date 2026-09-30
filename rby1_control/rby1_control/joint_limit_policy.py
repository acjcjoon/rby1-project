"""Validated joint-limit policy shared by Joint and Cartesian commands."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple

import yaml


class JointLimitPolicyError(ValueError):
    """Raised when a joint-limit policy is missing or malformed."""


@dataclass(frozen=True)
class JointLimit:
    """One named joint's physical position limits in radians."""

    name: str
    lower_rad: float
    upper_rad: float


@dataclass(frozen=True)
class JointLimitViolation:
    """First measured or requested position outside a selected limit."""

    group: str
    index: int
    joint_name: str
    value_deg: float
    lower_deg: float
    upper_deg: float


class JointLimitPolicy:
    """Load hard limits and derive conservative Cartesian soft limits."""

    def __init__(
        self,
        groups: Mapping[str, Sequence[JointLimit]],
        *,
        model: str,
    ) -> None:
        self.model = str(model)
        self._groups: Dict[str, Tuple[JointLimit, ...]] = {
            str(group): tuple(limits)
            for group, limits in groups.items()
        }

    @classmethod
    def from_yaml(
        cls,
        path: Path,
        *,
        expected_joint_names: Optional[
            Mapping[str, Sequence[str]]
        ] = None,
    ) -> "JointLimitPolicy":
        """Load and strictly validate a versioned radian-limit YAML file."""
        config_path = Path(path)
        try:
            with config_path.open("r", encoding="utf-8") as stream:
                payload = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as exc:
            raise JointLimitPolicyError(
                f"cannot load joint-limit policy {config_path}: {exc}"
            ) from exc

        if not isinstance(payload, dict):
            raise JointLimitPolicyError(
                "joint-limit policy root must be a mapping"
            )
        if payload.get("schema_version") != 1:
            raise JointLimitPolicyError(
                "joint-limit policy schema_version must be 1"
            )
        if payload.get("unit") != "rad":
            raise JointLimitPolicyError(
                "joint-limit policy unit must be 'rad'"
            )

        model = payload.get("model")
        if not isinstance(model, str) or not model.strip():
            raise JointLimitPolicyError(
                "joint-limit policy model must be a non-empty string"
            )

        raw_groups = payload.get("joint_limits")
        if not isinstance(raw_groups, dict) or not raw_groups:
            raise JointLimitPolicyError(
                "joint-limit policy joint_limits must be a non-empty mapping"
            )

        groups: Dict[str, Tuple[JointLimit, ...]] = {}
        for raw_group, raw_limits in raw_groups.items():
            group = str(raw_group)
            if not isinstance(raw_limits, list) or not raw_limits:
                raise JointLimitPolicyError(
                    f"joint_limits.{group} must be a non-empty list"
                )

            parsed = []
            seen_names = set()
            for index, raw_limit in enumerate(raw_limits):
                location = f"joint_limits.{group}[{index}]"
                if not isinstance(raw_limit, dict):
                    raise JointLimitPolicyError(
                        f"{location} must be a mapping"
                    )
                name = raw_limit.get("name")
                if not isinstance(name, str) or not name:
                    raise JointLimitPolicyError(
                        f"{location}.name must be a non-empty string"
                    )
                if name in seen_names:
                    raise JointLimitPolicyError(
                        f"duplicate joint name in {group}: {name}"
                    )
                seen_names.add(name)

                lower = cls._finite_number(
                    raw_limit.get("lower"),
                    f"{location}.lower",
                )
                upper = cls._finite_number(
                    raw_limit.get("upper"),
                    f"{location}.upper",
                )
                if lower >= upper:
                    raise JointLimitPolicyError(
                        f"{location} lower must be smaller than upper"
                    )
                parsed.append(JointLimit(name, lower, upper))
            groups[group] = tuple(parsed)

        if expected_joint_names is not None:
            expected_groups = set(expected_joint_names)
            actual_groups = set(groups)
            if actual_groups != expected_groups:
                missing = sorted(expected_groups - actual_groups)
                extra = sorted(actual_groups - expected_groups)
                raise JointLimitPolicyError(
                    "joint-limit groups do not match the control model: "
                    f"missing={missing}, extra={extra}"
                )
            for group, expected_names in expected_joint_names.items():
                actual_names = tuple(
                    limit.name for limit in groups[group]
                )
                if actual_names != tuple(expected_names):
                    raise JointLimitPolicyError(
                        f"joint names/order for {group} do not match the "
                        "control model"
                    )

        return cls(groups, model=model.strip())

    @staticmethod
    def _finite_number(value: object, location: str) -> float:
        if isinstance(value, bool):
            raise JointLimitPolicyError(
                f"{location} must be a finite number"
            )
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exc:
            raise JointLimitPolicyError(
                f"{location} must be a finite number"
            ) from exc
        if not math.isfinite(numeric):
            raise JointLimitPolicyError(
                f"{location} must be a finite number"
            )
        return numeric

    def limits(self, group: str) -> Tuple[JointLimit, ...]:
        """Return the configured hard limits for one joint group."""
        try:
            return self._groups[str(group)]
        except KeyError as exc:
            raise JointLimitPolicyError(
                f"no joint limits defined for group: {group}"
            ) from exc

    def limits_rad(
        self,
        group: str,
    ) -> Tuple[Tuple[float, float], ...]:
        """Return hard lower/upper pairs in radians."""
        return tuple(
            (limit.lower_rad, limit.upper_rad)
            for limit in self.limits(group)
        )

    def limits_deg(
        self,
        group: str,
        *,
        margin_deg: float = 0.0,
    ) -> Tuple[Tuple[float, float], ...]:
        """Return degree limits inset by ``margin_deg`` on both sides."""
        margin = self._finite_number(margin_deg, "margin_deg")
        if margin < 0.0:
            raise JointLimitPolicyError(
                "margin_deg must be greater than or equal to zero"
            )

        result = []
        for limit in self.limits(group):
            lower = math.degrees(limit.lower_rad) + margin
            upper = math.degrees(limit.upper_rad) - margin
            if lower > upper:
                raise JointLimitPolicyError(
                    f"margin_deg={margin} collapses the range for "
                    f"{limit.name}"
                )
            result.append((lower, upper))
        return tuple(result)

    def first_violation(
        self,
        group: str,
        values_deg: Sequence[float],
        *,
        margin_deg: float = 0.0,
    ) -> Optional[JointLimitViolation]:
        """Return the first out-of-range value, or ``None`` when safe."""
        limits = self.limits(group)
        if len(values_deg) != len(limits):
            raise JointLimitPolicyError(
                f"{group} value count does not match joint limits"
            )
        numeric_values = tuple(
            self._finite_number(value, f"{group}[{index}]")
            for index, value in enumerate(values_deg)
        )
        limits_deg = self.limits_deg(group, margin_deg=margin_deg)
        for index, (value, bounds, limit) in enumerate(
            zip(numeric_values, limits_deg, limits)
        ):
            lower, upper = bounds
            if value < lower or value > upper:
                return JointLimitViolation(
                    group=str(group),
                    index=index,
                    joint_name=limit.name,
                    value_deg=value,
                    lower_deg=lower,
                    upper_deg=upper,
                )
        return None
