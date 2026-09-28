import math

import pytest

from rby1_planner.control_commands import CommandKind, Task, TaskCommand
from rby1_planner.control_protocol import (
    task_command_from_dict,
    task_command_to_dict,
)


def test_gripper_task_api_keeps_default_speed_optional():
    definition = (
        Task('default-gripper')
        .open_gripper('left')
        .close_gripper('right')
        .set_gripper('both', 0.6)
        .build()
    )

    assert [command.velocity_limit for command in definition.commands] == [
        None,
        None,
        None,
    ]
    assert definition.commands[1].seconds == pytest.approx(1.0)
    assert definition.commands[2].seconds == pytest.approx(1.0)


def test_gripper_task_api_stores_positional_speed_in_velocity_limit():
    definition = (
        Task('speed-gripper')
        .open_gripper('left', 0.2)
        .close_gripper('right', 0.3, settle_time_sec=1.5)
        .set_gripper('both', 0.6, 0.4, settle_time_sec=2.0)
        .build()
    )

    assert [
        command.velocity_limit for command in definition.commands
    ] == pytest.approx([0.2, 0.3, 0.4])
    assert definition.commands[1].seconds == pytest.approx(1.5)
    assert definition.commands[2].seconds == pytest.approx(2.0)


@pytest.mark.parametrize('speed', [0.0, -0.1, math.nan, math.inf, True])
def test_gripper_task_api_rejects_nonpositive_or_nonfinite_speed(speed):
    with pytest.raises(ValueError, match='gripper speed'):
        Task('invalid-gripper').close_gripper('left', speed)


def test_gripper_command_allows_only_its_velocity_motion_field():
    command = TaskCommand(
        kind=CommandKind.GRIPPER_CLOSE,
        group='left',
        velocity_limit=0.25,
        seconds=1.0,
    )
    assert command.velocity_limit == pytest.approx(0.25)

    with pytest.raises(ValueError, match='incompatible motion fields'):
        TaskCommand(
            kind=CommandKind.GRIPPER_CLOSE,
            group='left',
            velocity_limit=0.25,
            acceleration_limit=1.0,
            seconds=1.0,
        )


def test_gripper_speed_round_trips_through_task_protocol():
    command = TaskCommand(
        kind=CommandKind.GRIPPER_SET,
        group='left',
        values=(0.5,),
        velocity_limit=0.25,
        seconds=1.0,
    )

    assert task_command_from_dict(task_command_to_dict(command)) == command


def test_control_client_only_sends_gripper_speed_when_provided():
    pytest.importorskip('std_msgs.msg')
    from rby1_planner.control_client import ControlClient

    client = object.__new__(ControlClient)
    calls = []
    client._send = lambda operation, arguments: calls.append(
        (operation, arguments)
    )

    client.open_gripper('left')
    client.close_gripper('right', 0.35)

    assert calls == [
        ('open_gripper', {'side': 'left'}),
        ('close_gripper', {'side': 'right', 'speed': 0.35}),
    ]
