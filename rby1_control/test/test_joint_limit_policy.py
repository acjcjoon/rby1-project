from pathlib import Path

import pytest

from rby1_control.joint_limit_policy import (
    JointLimitPolicy,
    JointLimitPolicyError,
)
from rby1_control.manipulator_controller import JOINT_NAMES


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'joint_limits_rby1m_v1_3.yaml'
)


def load_policy():
    return JointLimitPolicy.from_yaml(
        CONFIG_PATH,
        expected_joint_names=JOINT_NAMES,
    )


def test_yaml_preserves_existing_joint_space_hard_limits():
    policy = load_policy()

    right_arm_1 = policy.limits('right_arm')[1]
    left_arm_1 = policy.limits('left_arm')[1]

    assert right_arm_1.lower_rad == pytest.approx(-3.141592654)
    assert right_arm_1.upper_rad == pytest.approx(0.017453293)
    assert left_arm_1.lower_rad == pytest.approx(-0.017453293)
    assert left_arm_1.upper_rad == pytest.approx(3.141592654)

    hard_safe = [0.0, 1.0, 0.0, -10.0, 0.0, 0.0, 0.0]
    assert policy.first_violation('right_arm', hard_safe) is None

    hard_violation = list(hard_safe)
    hard_violation[1] = 1.1
    violation = policy.first_violation('right_arm', hard_violation)
    assert violation is not None
    assert violation.joint_name == 'right_arm_1'


def test_cartesian_margin_is_applied_to_both_bounds():
    policy = load_policy()
    limits = policy.limits_deg('right_arm', margin_deg=5.0)

    assert limits[1][0] == pytest.approx(-175.0)
    assert limits[1][1] == pytest.approx(-4.0)
    assert limits[3][0] == pytest.approx(-145.0)
    assert limits[3][1] == pytest.approx(-4.0)

    safe = [0.0, -5.0, 0.0, -5.0, 0.0, 0.0, 0.0]
    assert policy.first_violation(
        'right_arm',
        safe,
        margin_deg=5.0,
    ) is None

    unsafe = list(safe)
    unsafe[1] = -3.9
    violation = policy.first_violation(
        'right_arm',
        unsafe,
        margin_deg=5.0,
    )
    assert violation is not None
    assert violation.joint_name == 'right_arm_1'


def test_policy_fails_closed_for_missing_file(tmp_path):
    with pytest.raises(JointLimitPolicyError, match='cannot load'):
        JointLimitPolicy.from_yaml(
            tmp_path / 'missing.yaml',
            expected_joint_names=JOINT_NAMES,
        )


def test_policy_fails_closed_for_wrong_joint_order(tmp_path):
    malformed = tmp_path / 'wrong-order.yaml'
    malformed.write_text(
        '\n'.join(
            [
                'schema_version: 1',
                'model: test',
                'unit: rad',
                'joint_limits:',
                '  right_arm:',
                '    - {name: right_arm_1, lower: -1, upper: 1}',
            ]
        ),
        encoding='utf-8',
    )

    with pytest.raises(JointLimitPolicyError, match='joint names/order'):
        JointLimitPolicy.from_yaml(
            malformed,
            expected_joint_names={
                'right_arm': JOINT_NAMES['right_arm'],
            },
        )
