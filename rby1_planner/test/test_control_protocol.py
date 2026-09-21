from rby1_planner.control_commands import CommandKind, TaskCommand
from rby1_planner.control_protocol import (
    COMMAND_KIND,
    decode_message,
    encode_message,
    task_command_from_dict,
    task_command_to_dict,
)


def test_planner_owned_control_protocol_round_trip():
    command = TaskCommand(
        kind=CommandKind.GRIPPER_SET,
        group='left',
        values=(0.5,),
        seconds=1.0,
    )
    assert task_command_from_dict(task_command_to_dict(command)) == command

    encoded = encode_message(
        COMMAND_KIND,
        {'operation': 'stop', 'arguments': {}},
    )
    assert decode_message(encoded, expected_kind=COMMAND_KIND)['operation'] == (
        'stop'
    )
