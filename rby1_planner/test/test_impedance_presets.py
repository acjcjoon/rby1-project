"""Named impedance presets and the impedance= argument on Cartesian steps."""
import pytest

from rby1_planner import task_commands
from rby1_planner.control_commands import (
    CommandKind,
    JointImpedance,
    resolve_impedance,
)
from rby1_planner.task_commands import Task, load_impedance_preset


PRESETS = {
    'impedance_presets': {
        'default': {
            'stiffness': [80, 80, 60, 60, 30, 30, 20],
            'damping_ratio': 1.0,
            'torque_limit': 10.0,
            'hold_time': 0.0,
        },
        'contact': {
            'stiffness': [80, 80, 60, 60, 30, 30, 20],
            'torque_limit': 5.0,
            'hold_time': 0.5,
        },
    },
}
MOTION = (2.0, 0.05, 0.2, 0.5)


@pytest.fixture(autouse=True)
def _settings(monkeypatch):
    monkeypatch.setattr(task_commands, '_load_task_settings', lambda: PRESETS)


def test_repository_settings_define_default_and_contact():
    import yaml
    from pathlib import Path
    path = Path(task_commands.__file__).resolve().parents[1] / 'config' / 'settings.yaml'
    presets = yaml.safe_load(path.read_text())['impedance_presets']
    assert {'default', 'contact'} <= set(presets)


def test_resolution_rules():
    assert resolve_impedance(None) is None
    assert resolve_impedance(False) is None
    default = resolve_impedance(True)
    assert default.torque_limit == 10.0 and not default.is_bound
    contact = resolve_impedance('contact')
    assert contact.torque_limit == 5.0 and contact.hold_time == 0.5
    explicit = JointImpedance(stiffness=[1.0] * 7, torque_limit=3.0)
    assert resolve_impedance(explicit) is explicit
    with pytest.raises(ValueError, match="not defined"):
        resolve_impedance('missing')
    with pytest.raises(ValueError, match='must be None, True'):
        resolve_impedance(3)


def test_preset_validation_errors_are_reported(monkeypatch):
    bad = {'impedance_presets': {'default': {'stiffness': [1, 2], 'torque_limt': 3}}}
    monkeypatch.setattr(task_commands, '_load_task_settings', lambda: bad)
    with pytest.raises(ValueError, match='unknown keys'):
        load_impedance_preset('default')
    monkeypatch.setattr(task_commands, '_load_task_settings', lambda: {})
    with pytest.raises(ValueError, match='no impedance_presets'):
        load_impedance_preset('default')


def test_linear_steps_default_to_position_control():
    task = Task('t')
    task.linear_relative('right_arm', (0, -0.03, 0, 0, 0, 0), MOTION)
    task.linear_absolute('left_arm', (0.4, 0.2, 1.0, 0, 0, 0), MOTION)
    task.camera_linear_absolute('right_arm', 'tag_4', MOTION)
    assert all(step.impedance is None for step in task.build().commands)


@pytest.mark.parametrize('spec', [True, 'contact'])
def test_linear_relative_impedance_binds_preset_to_arm(spec):
    task = Task('t')
    task.linear_relative('right_arm', (0, -0.03, 0, 0, 0, 0), MOTION, impedance=spec)
    task.linear_relative('right_arm', (0, 0.05, 0, 0, 0, 0), MOTION)
    step = task.build().commands[0]
    assert step.kind is CommandKind.LINEAR_RELATIVE
    assert step.impedance.groups == ('right_arm',)


def test_camera_linear_absolute_impedance_and_exit_rule():
    task = Task('t')
    task.camera_linear_absolute('right_arm', 'tag_4', MOTION, impedance='contact')
    with pytest.raises(ValueError, match='must be followed by a position-controlled'):
        task.build()
    task.linear_relative('right_arm', (0, 0.05, 0, 0, 0, 0), MOTION)
    step = task.build().commands[0]
    assert step.impedance.groups == ('right_arm',)
    assert step.impedance.torque_limit == 5.0


def test_torso_cartesian_impedance_is_rejected():
    with pytest.raises(ValueError, match='Cartesian impedance supports only'):
        Task('t').linear_relative('torso', (0, 0, 0.02, 0, 0, 0), MOTION, impedance=True)


def test_joint_steps_accept_presets():
    task = Task('t')
    task.joint_relative('right_arm', [0, 0, 0, -10, 0, 0, 0], (3.0, 0.5, 0.5), impedance=True)
    task.joint_absolute_multi(
        {'right_arm': [0] * 7, 'left_arm': [0] * 7}, (3.0, 0.5, 0.5), impedance='default',
    )
    first, second = task.build().commands
    assert first.impedance.groups == ('right_arm',)
    assert sorted(second.impedance.groups) == ['left_arm', 'right_arm']
    with pytest.raises(ValueError, match='torso stiffness'):
        Task('t').joint_absolute_multi(
            {'torso': [0] * 6, 'right_arm': [0] * 7}, (3.0, 0.5, 0.5), impedance=True,
        )


def test_contact_helpers_accept_preset_names():
    task = Task('t').press('right_arm', (0, -0.02, 0), MOTION, impedance='contact')
    contact, retreat = task.build().commands
    assert contact.impedance.hold_time == 0.5 and retreat.impedance is None
    with pytest.raises(ValueError, match='require impedance'):
        Task('t').press('right_arm', (0, -0.02, 0), MOTION, impedance=None)
