"""Arm IK on the SDK model stored in config (skipped without rby1_sdk)."""
import math
from pathlib import Path

import numpy as np
import pytest


pytest.importorskip('rby1_sdk')

from rby1_control.arm_kinematics import (  # noqa: E402
    ArmKinematics,
    pose_to_transform,
    rotation_error_vector,
)
from rby1_control.stream_path import rpy_deg_to_quaternion  # noqa: E402


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'rby1m_v1_3_sdk_model.urdf'
)

# Measured joint state (deg) from the door grasp posture on the real robot.
GROUPS_DEG = {
    'torso': [0.06, 45.37, -90.96, 45.15, 0.03, -90.0],
    'right_arm': [-55.48, -44.23, 44.42, -46.12, -68.75, -9.22, -8.10],
    'left_arm': [-37.02, 61.23, -44.05, -95.22, -16.82, 35.95, -84.38],
    'head': [0.0, 0.0],
}


@pytest.fixture(scope='module')
def kinematics():
    return ArmKinematics.from_model_file(MODEL_PATH)


def test_model_exposes_both_arms_and_joint_order(kinematics):
    assert kinematics.arms == ('right_arm', 'left_arm')
    assert kinematics.dof == 26
    columns = kinematics.arm_columns('right_arm')
    assert [kinematics.joint_names[i] for i in columns] == [
        f'right_arm_{k}' for k in range(7)
    ]


def test_rotation_error_vector_matches_axis_angle():
    angle = math.radians(30.0)
    rotation = np.array([
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    error = rotation_error_vector(rotation, np.eye(3))
    assert error == pytest.approx((0.0, 0.0, angle))
    assert rotation_error_vector(np.eye(3), np.eye(3)) == pytest.approx((0.0,) * 3)


def test_ik_tracks_a_small_offset_with_fixed_orientation(kinematics):
    q = kinematics.joint_vector_from_groups(GROUPS_DEG)
    start = kinematics.forward(q, 'right_arm')
    target = start.copy()
    target[:3, 3] += (0.0, 0.01, 0.0)
    result = kinematics.solve('right_arm', q, target)
    assert result.converged
    assert result.iterations <= 15
    assert result.position_error_m < 5.0e-4
    assert result.orientation_error_rad < 2.0e-3
    reached = kinematics.forward(result.q_full, 'right_arm')
    assert reached[:3, 3] == pytest.approx(target[:3, 3], abs=5.0e-4)
    # Only the right arm moved.
    others = np.ones(kinematics.dof, dtype=bool)
    others[kinematics.arm_columns('right_arm')] = False
    assert np.allclose(result.q_full[others], q[others])


def test_ik_reports_non_convergence_for_an_unreachable_target(kinematics):
    q = kinematics.joint_vector_from_groups(GROUPS_DEG)
    start = kinematics.forward(q, 'right_arm')
    target = start.copy()
    target[:3, 3] += (0.0, -3.0, 0.0)
    result = kinematics.solve('right_arm', q, target, max_iterations=20)
    assert not result.converged
    assert result.position_error_m > 1.0


def test_pose_to_transform_matches_rpy_convention(kinematics):
    q = kinematics.joint_vector_from_groups(GROUPS_DEG)
    start = kinematics.forward(q, 'right_arm')
    from rby1_control.manipulator_controller import ManipulatorController
    rotation = start[:3, :3]
    trace = np.trace(rotation)
    w = math.sqrt(max(0.0, 1.0 + trace)) / 2.0
    x = (rotation[2, 1] - rotation[1, 2]) / (4.0 * w)
    y = (rotation[0, 2] - rotation[2, 0]) / (4.0 * w)
    z = (rotation[1, 0] - rotation[0, 1]) / (4.0 * w)
    rpy = ManipulatorController.quaternion_to_rpy_deg(x, y, z, w)
    rebuilt = pose_to_transform(*start[:3, 3], *rpy_deg_to_quaternion(*rpy))
    assert rotation_error_vector(start[:3, :3], rebuilt[:3, :3]) == pytest.approx(
        (0.0,) * 3, abs=1.0e-6,
    )


def test_quaternion_to_rpy_round_trips_exact_gimbal_lock():
    import math
    from rby1_control.manipulator_controller import ManipulatorController
    for q in ((0.5, 0.5, 0.5, -0.5), (0.5, -0.5, 0.5, 0.5), (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))):
        roll, pitch, yaw = ManipulatorController.quaternion_to_rpy_deg(*q)
        assert abs(pitch) == pytest.approx(90.0)
        back = ManipulatorController.rpy_deg_to_quaternion(roll, pitch, yaw)
        assert abs(sum(a * b for a, b in zip(q, back))) == pytest.approx(1.0, abs=1e-9)
