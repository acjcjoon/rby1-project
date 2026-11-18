"""The torso is a Cartesian group for single-target linear Task steps."""
import threading
import time
from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')

from rby1_control.backend_contract import TaskCommandStatus  # noqa: E402
from rby1_control.command_model import (  # noqa: E402
    CARTESIAN_ARMS,
    CARTESIAN_GROUPS,
    CommandKind,
    TaskCommand,
)
from rby1_control import control_node as control_node_module  # noqa: E402
from rby1_control.control_node import RBY1ControlNode  # noqa: E402
from rby1_control.manipulator_controller import ManipulatorController  # noqa: E402


def _linear(group, kind=CommandKind.LINEAR_ABSOLUTE):
    return TaskCommand(
        kind=kind,
        group=group,
        values=(0.0, 0.0, 1.1, 0.0, 0.0, 0.0),
        minimum_time=3.0,
        linear_velocity=0.1,
        angular_velocity=0.3,
        acceleration_scaling=0.5,
    )


def test_cartesian_groups_extend_the_arms_with_the_torso():
    assert CARTESIAN_ARMS == ('right_arm', 'left_arm')
    assert CARTESIAN_GROUPS == ('right_arm', 'left_arm', 'torso')


@pytest.mark.parametrize(
    'kind', [CommandKind.LINEAR_ABSOLUTE, CommandKind.LINEAR_RELATIVE],
)
def test_linear_commands_accept_the_torso(kind):
    assert _linear('torso', kind).group == 'torso'


def test_linear_commands_reject_other_groups():
    with pytest.raises(ValueError, match='unknown Cartesian group: head'):
        _linear('head')


def test_stream_paths_remain_arm_only():
    with pytest.raises(ValueError, match='unknown Cartesian arm: torso'):
        TaskCommand(
            kind=CommandKind.LINEAR_PATH_STREAM_ABSOLUTE,
            group='torso',
            waypoints=((0.0,) * 6,),
            minimum_time=1.0,
            linear_velocity=0.1,
            angular_velocity=0.1,
        )


def test_manipulator_controller_caches_the_torso_tcp():
    controller = ManipulatorController()
    assert set(controller.cartesian_state) == {'right_arm', 'left_arm', 'torso'}
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=0.0, y=0.0, z=1.1),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    pose = controller.update_cartesian('torso', transform)
    assert pose[:3] == [0.0, 0.0, 1.1]
    assert controller.motion_state()['cartesian']['torso'][:3] == [0.0, 0.0, 1.1]


def test_task_linear_absolute_sends_a_torso_cartesian_goal(monkeypatch):
    class Vector:
        def __init__(self):
            self.x = self.y = self.z = 0.0
            self.w = 1.0

    class CartesianCommand:
        def __init__(self):
            self.ref_link = ''
            self.target_link = ''
            self.transform = SimpleNamespace(
                translation=Vector(), rotation=Vector(),
            )

    class Rby1CartesianCommand:
        class Goal:
            def __init__(self):
                self.torso = None
                self.right_arm = None
                self.left_arm = None

    monkeypatch.setattr(control_node_module, 'CartesianCommand', CartesianCommand)
    monkeypatch.setattr(
        control_node_module, 'Rby1CartesianCommand', Rby1CartesianCommand,
    )

    node = object.__new__(RBY1ControlNode)
    node._lock = threading.RLock()
    node._task_command_sequence = 0
    node._task_commands = {}
    node._active_gripper_task_id = None
    node.cartesian_links = {
        'right_arm': ('base', 'ee_right'),
        'left_arm': ('base', 'ee_left'),
        'torso': ('base', 'link_torso_5'),
    }
    node.cartesian_action_client = SimpleNamespace(server_is_ready=lambda: True)
    node._motion_command_allowed = lambda: True
    checked = []
    node._cartesian_feedback_ready = lambda arm: checked.append(('tcp', arm)) or True
    node._cartesian_joint_preflight_ready = (
        lambda arm: checked.append(('joints', arm)) or True
    )
    sent = {}
    node._send_motion_goal = lambda **kwargs: sent.update(kwargs)

    command_id = node.start_task_command(_linear('torso'))

    assert node._task_commands[command_id].status is TaskCommandStatus.PENDING
    assert checked == [('tcp', 'torso'), ('joints', 'torso')]
    goal = sent['goal']
    assert goal.right_arm is None and goal.left_arm is None
    assert goal.torso.ref_link == 'base'
    assert goal.torso.target_link == 'link_torso_5'
    assert goal.torso.transform.translation.z == pytest.approx(1.1)
    assert goal.torso.minimum_time == pytest.approx(3.0)
    assert goal.torso.linear_velocity_limit == pytest.approx(0.1)
    # The runtime joint soft-limit guard follows the torso joints.
    assert sent['motion_mode'] == 'cartesian'
    assert sent['motion_groups'] == ('torso',)


def _manual_node(monkeypatch):
    """Node stub for the manual (operator UI) Cartesian path."""

    class Vector:
        def __init__(self):
            self.x = self.y = self.z = 0.0
            self.w = 1.0

    class CartesianCommand:
        def __init__(self):
            self.ref_link = ''
            self.target_link = ''
            self.transform = SimpleNamespace(
                translation=Vector(), rotation=Vector(),
            )

    class Rby1CartesianCommand:
        class Goal:
            def __init__(self):
                self.torso = None
                self.right_arm = None
                self.left_arm = None

    monkeypatch.setattr(control_node_module, 'CartesianCommand', CartesianCommand)
    monkeypatch.setattr(
        control_node_module, 'Rby1CartesianCommand', Rby1CartesianCommand,
    )

    node = object.__new__(RBY1ControlNode)
    node._lock = threading.RLock()
    node.cartesian_links = {
        'right_arm': ('base', 'ee_right'),
        'left_arm': ('base', 'ee_left'),
        'torso': ('base', 'link_torso_5'),
    }
    node.cartesian_action_client = SimpleNamespace(server_is_ready=lambda: True)
    node.cartesian_action_name = 'robot_cartesian'
    node.manual_cartesian_linear_velocity_limit = 0.1
    node.manual_cartesian_angular_velocity_limit = 0.3
    node.manual_cartesian_acceleration_scaling = 0.5
    node._motion_command_allowed = lambda: True
    node.checked = []
    node._cartesian_feedback_ready = (
        lambda arm: node.checked.append(('tcp', arm)) or True
    )
    node._cartesian_joint_preflight_ready = (
        lambda arm: node.checked.append(('joints', arm)) or True
    )
    node.events = []
    node._push_event = lambda level, text: node.events.append((level, text))
    node.sent = {}
    node._send_motion_goal = lambda **kwargs: node.sent.update(kwargs)
    return node


def test_manual_move_cartesian_sends_a_torso_goal(monkeypatch):
    node = _manual_node(monkeypatch)

    node.move_cartesian('torso', [0.0, 0.0, 1.05, 0.0, 0.0, 0.0], 3.0)

    assert node.checked == [('tcp', 'torso'), ('joints', 'torso')]
    goal = node.sent['goal']
    assert goal.right_arm is None and goal.left_arm is None
    assert goal.torso.ref_link == 'base'
    assert goal.torso.target_link == 'link_torso_5'
    assert goal.torso.transform.translation.z == pytest.approx(1.05)
    assert node.sent['motion_groups'] == ('torso',)


def test_manual_torso_cartesian_rejects_impedance(monkeypatch):
    node = _manual_node(monkeypatch)

    node.move_cartesian(
        'torso',
        [0.0, 0.0, 1.05, 0.0, 0.0, 0.0],
        3.0,
        impedance={'stiffness': {'torso': [100.0] * 6}},
    )

    assert node.sent == {}
    assert any(
        level == 'error' and 'supports only the arms' in text
        for level, text in node.events
    )


def test_manual_cartesian_rejects_the_head(monkeypatch):
    node = _manual_node(monkeypatch)

    node.move_cartesian('head', [0.0] * 6, 3.0)

    assert node.sent == {}
    assert node.events[-1][0] == 'error'
