import pytest

from rby1_control.command_model import CommandKind, JointImpedance, TaskCommand


def _multi(**extra):
    return TaskCommand(
        kind=CommandKind.JOINT_ABSOLUTE_MULTI,
        joint_targets=(('torso', (0.0,) * 6), ('head', (0.0, 2.0))),
        minimum_time=5.0,
        velocity_limit=2.0,
        acceleration_limit=3.0,
        **extra,
    )


def test_multi_command_accepts_head_with_body():
    assert [g for g, _ in _multi().joint_targets] == ['torso', 'head']


def test_multi_command_rejects_head_impedance():
    with pytest.raises(ValueError, match='head'):
        _multi(impedance=JointImpedance(
            stiffness={'torso': (100.0,) * 6, 'head': (10.0,) * 2},
        ))
