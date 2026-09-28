"""ROS-independent contracts shared by the Qt UI and both backends."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Dict, Mapping, Optional, Sequence, Tuple


@dataclass(frozen=True)
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def stopped(self) -> bool:
        return (
            abs(self.vx) < 1e-9
            and abs(self.vy) < 1e-9
            and abs(self.wz) < 1e-9
        )


@dataclass(frozen=True)
class BackendSnapshot:
    namespace: str
    cmd_vel_topic: str
    cmd_vel_subscribers: int
    control_state: Optional[int]
    power_enabled: Optional[bool]
    servo_enabled: Optional[bool]
    stream_enabled: Optional[bool]
    emo_active: Optional[bool]
    collision_active: Optional[bool]
    services_enabled: bool
    rby1_msgs_available: bool
    service_ready: Dict[str, bool]
    command: VelocityCommand
    command_stale: bool
    gripper_ready: Optional[bool] = None
    gripper_state_fresh: bool = False
    gripper_positions: Optional[Tuple[float, float]] = None
    gripper_target: Optional[Tuple[float, float]] = None
    gripper_motion_active: bool = False
    gripper_error: Optional[str] = None
    gripper_default_speed_ratio_per_sec: Optional[float] = None
    gripper_max_speed_ratio_per_sec: Optional[float] = None
    gripper_acceleration_ratio_per_sec2: Optional[float] = None
    gripper_trajectory_rate_hz: Optional[float] = None
    gripper_power_voltages: Optional[Tuple[float, float]] = None
    gripper_power_state_fresh: bool = False
    gripper_power_12v: bool = False


class TaskCommandStatus(str, Enum):
    PENDING = "pending"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


@dataclass(frozen=True)
class TaskCommandState:
    status: TaskCommandStatus
    message: str = ""
    timeout_remaining_sec: Optional[float] = None

    def __post_init__(self) -> None:
        value = self.timeout_remaining_sec
        if value is None:
            return
        if isinstance(value, bool):
            raise ValueError(
                "timeout_remaining_sec must be a nonnegative finite number"
            )
        try:
            normalized = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "timeout_remaining_sec must be a nonnegative finite number"
            ) from exc
        if not math.isfinite(normalized) or normalized < 0.0:
            raise ValueError(
                "timeout_remaining_sec must be a nonnegative finite number"
            )
        object.__setattr__(self, "timeout_remaining_sec", normalized)


@dataclass(frozen=True)
class TaskBackendState:
    """One consistent capture of the state needed by Task execution.

    Joint values and Cartesian RPY values use the same operator units shown
    in the UI: degrees and ``[m, m, m, deg, deg, deg]`` respectively.
    Timestamps are from ``time.monotonic()``.
    """

    captured_at: float
    robot_state_updated_at: Optional[float]
    control_state: Optional[int]
    stream_enabled: Optional[bool]
    emo_active: Optional[bool]
    collision_active: Optional[bool]
    motion_active: bool
    joint_groups: Mapping[str, Optional[Sequence[float]]]
    joint_updated_at: Mapping[str, Optional[float]]
    joint_order_verified: Mapping[str, bool]
    cartesian: Mapping[str, Optional[Sequence[float]]]
    cartesian_updated_at: Mapping[str, Optional[float]]
    driver_safety_verified: bool
    driver_safety_updated_at: Optional[float]
