"""Arm inverse kinematics on the RB-Y1 SDK dynamics model.

The driver's own ``stream_cartesian`` IK does not converge on this robot, so
streamed Cartesian paths are converted to joint targets here and sent through
``stream_joint``. The model is the robot's own description as served by the
SDK (``Robot.get_robot_model()``), stored under ``config/`` so no robot
connection is needed. Forward kinematics of that model matches the driver's
``get_cartesian_pose`` service.

Only the commanded arm's seven joints move; the torso and every other joint
are held at their measured values. A damped least-squares step on the space
Jacobian converges in a handful of iterations for the small per-tick motions
of a streamed path.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

from .manipulator_controller import JOINT_NAMES


DEFAULT_ARM_LINKS = {
    'right_arm': 'ee_right',
    'left_arm': 'ee_left',
}


@dataclass(frozen=True)
class IkResult:
    arm: str
    q_full: np.ndarray
    q_arm_rad: Tuple[float, ...]
    converged: bool
    iterations: int
    position_error_m: float
    orientation_error_rad: float

    @property
    def q_arm_deg(self) -> Tuple[float, ...]:
        return tuple(math.degrees(value) for value in self.q_arm_rad)


def rotation_error_vector(target_rotation, current_rotation) -> np.ndarray:
    """Return the axis-angle vector (space frame) taking current to target."""

    relative = np.asarray(target_rotation) @ np.asarray(current_rotation).T
    cos_angle = max(-1.0, min(1.0, (np.trace(relative) - 1.0) / 2.0))
    angle = math.acos(cos_angle)
    if angle < 1.0e-9:
        return np.zeros(3)
    axis = np.array([
        relative[2, 1] - relative[1, 2],
        relative[0, 2] - relative[2, 0],
        relative[1, 0] - relative[0, 1],
    ])
    sin_angle = math.sin(angle)
    if sin_angle < 1.0e-9:
        # Near pi: fall back to the symmetric part of the rotation.
        diagonal = np.diag(relative)
        axis = np.sqrt(np.maximum((diagonal + 1.0) / 2.0, 0.0))
        return angle * axis / max(np.linalg.norm(axis), 1.0e-12)
    return angle / (2.0 * sin_angle) * axis


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def pose_to_transform(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = quaternion_to_rotation(qx, qy, qz, qw)
    transform[:3, 3] = (x, y, z)
    return transform


class ArmKinematics:
    """Forward and inverse kinematics for one arm on the SDK dynamics model."""

    def __init__(
        self,
        dynamics,
        *,
        base_link: str = 'base',
        arm_links: Optional[Mapping[str, str]] = None,
    ) -> None:
        self._dynamics = dynamics
        self.joint_names: Tuple[str, ...] = tuple(dynamics.get_joint_names())
        self.link_names: Tuple[str, ...] = tuple(dynamics.get_link_names())
        if base_link not in self.link_names:
            raise ValueError(f'model has no link named {base_link!r}')
        self._base_index = self.link_names.index(base_link)
        self._state = dynamics.make_state(
            list(self.link_names),
            list(self.joint_names),
        )
        self.dof = int(dynamics.get_dof())
        self._joint_index: Dict[str, int] = {
            name: index for index, name in enumerate(self.joint_names)
        }
        links = dict(DEFAULT_ARM_LINKS if arm_links is None else arm_links)
        self._arm_links: Dict[str, int] = {}
        self._arm_columns: Dict[str, np.ndarray] = {}
        for arm, link in links.items():
            if link not in self.link_names:
                raise ValueError(f'model has no link named {link!r}')
            names = JOINT_NAMES.get(arm)
            if names is None:
                raise ValueError(f'unknown arm: {arm}')
            missing = [name for name in names if name not in self._joint_index]
            if missing:
                raise ValueError(f'model lacks joints {missing} for {arm}')
            self._arm_links[arm] = self.link_names.index(link)
            self._arm_columns[arm] = np.array(
                [self._joint_index[name] for name in names],
                dtype=int,
            )
        zero = np.zeros(self.dof)
        self._state.set_q(zero)
        self.q_lower = np.asarray(dynamics.get_limit_q_lower(self._state)).reshape(-1)
        self.q_upper = np.asarray(dynamics.get_limit_q_upper(self._state)).reshape(-1)

    @classmethod
    def from_model_file(
        cls,
        path: Path,
        *,
        base_link: str = 'base',
        arm_links: Optional[Mapping[str, str]] = None,
    ) -> 'ArmKinematics':
        """Load the SDK model description (``Robot.get_robot_model()``)."""

        try:
            from rby1_sdk import dynamics as sdk_dynamics
        except ImportError as exc:  # pragma: no cover - environment specific
            raise RuntimeError('rby1_sdk is not importable') from exc
        data = Path(path).read_text(encoding='utf-8')
        configuration = sdk_dynamics.load_robot_from_urdf_data(data, base_link)
        robot = None
        for name in ('Robot_26', 'Robot_24', 'Robot_18', 'Robot'):
            constructor = getattr(sdk_dynamics, name, None)
            if constructor is None:
                continue
            try:
                robot = constructor(configuration)
                break
            except Exception:
                continue
        if robot is None:
            raise RuntimeError(f'could not build a dynamics model from {path}')
        return cls(robot, base_link=base_link, arm_links=arm_links)

    @property
    def arms(self) -> Tuple[str, ...]:
        return tuple(self._arm_links)

    def arm_columns(self, arm: str) -> np.ndarray:
        return self._arm_columns[str(arm)]

    def joint_vector(
        self,
        joint_positions_rad: Mapping[str, float],
    ) -> np.ndarray:
        """Build the full model vector; joints not given stay at zero."""

        q = np.zeros(self.dof)
        for name, value in joint_positions_rad.items():
            index = self._joint_index.get(str(name))
            if index is not None:
                q[index] = float(value)
        return q

    def joint_vector_from_groups(
        self,
        groups_deg: Mapping[str, Optional[Sequence[float]]],
    ) -> np.ndarray:
        """Build the full vector from ``{group: degrees}`` as cached by the node."""

        positions: Dict[str, float] = {}
        for group, values in groups_deg.items():
            names = JOINT_NAMES.get(str(group))
            if names is None or values is None:
                continue
            if len(values) != len(names):
                raise ValueError(f'{group} has {len(values)} values; expected {len(names)}')
            for name, value in zip(names, values):
                positions[name] = math.radians(float(value))
        return self.joint_vector(positions)

    def arm_values(self, q_full: np.ndarray, arm: str) -> Tuple[float, ...]:
        return tuple(float(value) for value in np.asarray(q_full)[self._arm_columns[arm]])

    def forward(self, q_full: np.ndarray, arm: str) -> np.ndarray:
        """Return the 4x4 base-to-end-effector transform for ``arm``."""

        self._state.set_q(np.asarray(q_full, dtype=float).reshape(-1))
        self._dynamics.compute_forward_kinematics(self._state)
        return np.array(self._dynamics.compute_transformation(
            self._state,
            self._base_index,
            self._arm_links[arm],
        ))

    def solve(
        self,
        arm: str,
        q_seed_full: np.ndarray,
        target_transform: np.ndarray,
        *,
        max_iterations: int = 30,
        position_tolerance_m: float = 5.0e-4,
        orientation_tolerance_rad: float = 2.0e-3,
        damping: float = 1.0e-3,
        max_step_rad: float = 0.05,
    ) -> IkResult:
        """Damped least-squares IK moving only the ``arm`` joints.

        The result is reported as not converged when the tolerances are not
        met within ``max_iterations``; callers must then stop rather than
        send the partial solution.
        """

        arm = str(arm)
        columns = self._arm_columns[arm]
        link_index = self._arm_links[arm]
        target = np.asarray(target_transform, dtype=float)
        q = np.array(q_seed_full, dtype=float).reshape(-1).copy()
        lower = self.q_lower[columns]
        upper = self.q_upper[columns]
        position_error = math.inf
        orientation_error = math.inf
        iterations = 0
        for iteration in range(int(max_iterations) + 1):
            iterations = iteration
            self._state.set_q(q)
            self._dynamics.compute_forward_kinematics(self._state)
            current = np.array(self._dynamics.compute_transformation(
                self._state,
                self._base_index,
                link_index,
            ))
            position_delta = target[:3, 3] - current[:3, 3]
            rotation_delta = rotation_error_vector(target[:3, :3], current[:3, :3])
            position_error = float(np.linalg.norm(position_delta))
            orientation_error = float(np.linalg.norm(rotation_delta))
            if (
                position_error <= position_tolerance_m
                and orientation_error <= orientation_tolerance_rad
            ):
                return IkResult(
                    arm=arm,
                    q_full=q,
                    q_arm_rad=tuple(float(value) for value in q[columns]),
                    converged=True,
                    iterations=iterations,
                    position_error_m=position_error,
                    orientation_error_rad=orientation_error,
                )
            if iteration == int(max_iterations):
                break
            jacobian = np.asarray(self._dynamics.compute_space_jacobian(
                self._state,
                self._base_index,
                link_index,
            ))[:, columns]
            # Space-frame twist: [angular; linear] with the linear part
            # expressed at the base origin (v = p_dot - w x p).
            twist = np.concatenate([
                rotation_delta,
                position_delta - np.cross(rotation_delta, current[:3, 3]),
            ])
            gain = jacobian @ jacobian.T + float(damping) * np.eye(6)
            step = jacobian.T @ np.linalg.solve(gain, twist)
            step = np.clip(step, -float(max_step_rad), float(max_step_rad))
            q[columns] = np.clip(q[columns] + step, lower, upper)
        return IkResult(
            arm=arm,
            q_full=q,
            q_arm_rad=tuple(float(value) for value in q[columns]),
            converged=False,
            iterations=iterations,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
        )


__all__ = [
    'ArmKinematics',
    'IkResult',
    'pose_to_transform',
    'quaternion_to_rotation',
    'rotation_error_vector',
]
