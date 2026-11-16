"""Wire format of impedance commands and the backend control mode."""
import pytest

from rby1_control.backend_contract import TaskBackendState
from rby1_control.command_model import (
    CommandKind,
    JointImpedance,
    TaskCommand,
    impedance_from_dict,
)
from rby1_control.topic_protocol import (
    task_backend_state_from_dict,
    task_backend_state_to_dict,
    task_command_from_dict,
    task_command_to_dict,
)


def _cartesian(**overrides):
    base = dict(
        kind=CommandKind.LINEAR_ABSOLUTE,
        group='right_arm',
        values=(0.4, -0.2, 0.9, 0.0, 0.0, 90.0),
        minimum_time=2.0,
        linear_velocity=0.05,
        angular_velocity=0.2,
        acceleration_scaling=0.3,
    )
    base.update(overrides)
    return TaskCommand(**base)


def test_topic_protocol_round_trips_impedance_and_control_mode():
    joint = TaskCommand(
        kind=CommandKind.JOINT_ABSOLUTE_MULTI,
        joint_targets=(('torso', (0.0,) * 6), ('right_arm', (0.0,) * 7)),
        minimum_time=3.0,
        velocity_limit=0.5,
        acceleration_limit=0.5,
        impedance=JointImpedance(
            {'torso': [200.0] * 6, 'right_arm': [100.0] * 7},
            torque_limit=12.0,
        ),
    )
    # A Cartesian step carries joint impedance: the node solves the arm IK
    # and tracks the joint target with it.
    contact = _cartesian(
        impedance=JointImpedance(
            {'right_arm': [80.0] * 7}, torque_limit=5.0, hold_time=0.5,
        ),
    )
    for command in (joint, contact):
        assert task_command_from_dict(task_command_to_dict(command)) == command
    assert task_command_to_dict(contact)['impedance']['hold_time'] == 0.5

    state = TaskBackendState(
        captured_at=1.0,
        robot_state_updated_at=0.9,
        control_state=2,
        stream_enabled=False,
        emo_active=False,
        collision_active=False,
        motion_active=True,
        joint_groups={},
        joint_updated_at={},
        joint_order_verified={},
        cartesian={},
        cartesian_updated_at={},
        driver_safety_verified=False,
        driver_safety_updated_at=None,
        control_mode='impedance_cartesian',
    )
    assert task_backend_state_from_dict(task_backend_state_to_dict(state)) == state
    assert task_backend_state_from_dict({}).control_mode == 'idle'


def test_cartesian_impedance_rules_on_the_wire():
    with pytest.raises(ValueError, match='unknown impedance type'):
        impedance_from_dict({'type': 'cartesian', 'translation': [1, 1, 1]})
    # Legacy payloads without hold_time decode to a zero dwell.
    legacy = impedance_from_dict({
        'type': 'joint', 'stiffness': {'right_arm': [100.0] * 7},
    })
    assert legacy.hold_time == 0.0
    with pytest.raises(ValueError, match='bound to right_arm'):
        _cartesian(impedance=JointImpedance({'left_arm': [100.0] * 7}))
    with pytest.raises(ValueError, match='supports only'):
        _cartesian(
            group='torso',
            impedance=JointImpedance({'torso': [100.0] * 6}),
        )
