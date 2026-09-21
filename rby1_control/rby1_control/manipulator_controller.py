"""Manipulator feedback, snapshots, and rotation utilities."""
from __future__ import annotations

import math
import threading
import time
from typing import Dict, List, Optional, Tuple


JOINT_NAMES = {
    'right_arm': tuple(f'right_arm_{index}' for index in range(7)),
    'left_arm': tuple(f'left_arm_{index}' for index in range(7)),
    'torso': tuple(f'torso_{index}' for index in range(6)),
    'head': tuple(f'head_{index}' for index in range(2)),
}


class ManipulatorController:
    """Own cached manipulator state used by the control-node command layer."""

    def __init__(self, *, clock=time.monotonic) -> None:
        self._clock = clock
        self.lock = threading.RLock()
        self.joint_groups_deg: Dict[str, Optional[List[float]]] = {
            group: None for group in JOINT_NAMES
        }
        self.joint_updated_at = {group: None for group in JOINT_NAMES}
        self.joint_order_verified = {group: False for group in JOINT_NAMES}
        self.cartesian_state: Dict[str, Optional[List[float]]] = {
            'right_arm': None,
            'left_arm': None,
        }
        self.cartesian_updated_at = {
            arm: None for arm in self.cartesian_state
        }
        self.cartesian_quaternion_state: Dict[
            str,
            Optional[Tuple[float, float, float, float]],
        ] = {arm: None for arm in self.cartesian_state}
        self.cartesian_snapshot: Dict[str, Optional[List[float]]] = {
            arm: None for arm in self.cartesian_state
        }

    def update_joint_state(self, group: str, message) -> bool:
        names = JOINT_NAMES.get(group)
        if names is None:
            return False
        positions = {
            str(name): float(position)
            for name, position in zip(message.name, message.position)
        }
        ordered_by_name = True
        try:
            ordered = [positions[name] for name in names]
        except KeyError:
            if len(message.position) < len(names):
                return False
            ordered_by_name = False
            ordered = [float(value) for value in message.position[:len(names)]]
        with self.lock:
            self.joint_groups_deg[group] = [
                math.degrees(value) for value in ordered
            ]
            self.joint_updated_at[group] = self._clock()
            self.joint_order_verified[group] = ordered_by_name
        return True

    def update_cartesian(self, arm: str, transform) -> List[float]:
        quaternion = self.normalize_quaternion(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        rpy = self.quaternion_to_rpy_deg(*quaternion)
        pose = [
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
            *rpy,
        ]
        with self.lock:
            self.cartesian_state[arm] = pose
            self.cartesian_quaternion_state[arm] = quaternion
            self.cartesian_updated_at[arm] = self._clock()
        return pose

    def motion_state(self) -> Dict[str, object]:
        with self.lock:
            return {
                'joint_groups': {
                    group: list(values) if values is not None else None
                    for group, values in self.joint_groups_deg.items()
                },
                'cartesian': {
                    arm: list(values) if values is not None else None
                    for arm, values in self.cartesian_state.items()
                },
            }

    @staticmethod
    def quaternion_to_rpy_deg(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> Tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return tuple(math.degrees(value) for value in (roll, pitch, yaw))

    @staticmethod
    def rpy_deg_to_quaternion(
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> Tuple[float, float, float, float]:
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

    @staticmethod
    def normalize_quaternion(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> Tuple[float, float, float, float]:
        values = tuple(float(value) for value in (x, y, z, w))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('quaternion contains a non-finite value')
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1.0e-12:
            raise ValueError('quaternion has zero length')
        return tuple(value / norm for value in values)

    @staticmethod
    def multiply_quaternions(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @staticmethod
    def axis_angle_deg_to_quaternion(axis_index: int, angle_deg: float):
        if axis_index < 0 or axis_index >= 3:
            raise ValueError(f'invalid rotation axis {axis_index}')
        half_angle = math.radians(float(angle_deg)) * 0.5
        vector = [0.0, 0.0, 0.0]
        vector[axis_index] = math.sin(half_angle)
        return vector[0], vector[1], vector[2], math.cos(half_angle)

