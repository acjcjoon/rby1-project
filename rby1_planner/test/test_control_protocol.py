import pytest

from rby1_planner.backend_contract import (
    BackendSnapshot,
    TaskCommandState,
    TaskCommandStatus,
    VelocityCommand,
)
from rby1_planner.control_commands import CommandKind, TaskCommand
from rby1_planner.control_protocol import (
    COMMAND_KIND,
    backend_snapshot_from_dict,
    backend_snapshot_to_dict,
    decode_message,
    encode_message,
    task_command_state_from_dict,
    task_command_state_to_dict,
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


def test_gripper_speed_config_snapshot_round_trip_and_legacy_defaults():
    snapshot = BackendSnapshot(
        namespace='/rby1',
        cmd_vel_topic='/rby1/cmd_vel',
        cmd_vel_subscribers=1,
        control_state=2,
        power_enabled=True,
        servo_enabled=True,
        stream_enabled=False,
        emo_active=False,
        collision_active=False,
        services_enabled=True,
        rby1_msgs_available=True,
        service_ready={},
        command=VelocityCommand(),
        command_stale=False,
        gripper_default_speed_ratio_per_sec=1.5,
        gripper_max_speed_ratio_per_sec=2.0,
        gripper_acceleration_ratio_per_sec2=4.0,
        gripper_trajectory_rate_hz=50.0,
    )
    assert backend_snapshot_from_dict(backend_snapshot_to_dict(snapshot)) == (
        snapshot
    )

    legacy = backend_snapshot_from_dict({})
    assert legacy.gripper_default_speed_ratio_per_sec is None
    assert legacy.gripper_max_speed_ratio_per_sec is None
    assert legacy.gripper_acceleration_ratio_per_sec2 is None
    assert legacy.gripper_trajectory_rate_hz is None


def test_planner_task_state_timeout_hint_round_trips_and_accepts_legacy():
    state = TaskCommandState(
        TaskCommandStatus.PENDING,
        'moving',
        timeout_remaining_sec=3.25,
    )
    assert (
        task_command_state_from_dict(task_command_state_to_dict(state))
        == state
    )

    legacy = task_command_state_from_dict({
        'status': 'pending',
        'message': 'legacy backend',
    })
    assert legacy.timeout_remaining_sec is None


@pytest.mark.parametrize('value', [-1.0, float('nan'), float('inf'), True])
def test_planner_task_state_rejects_invalid_timeout_hint(value):
    with pytest.raises(ValueError, match='timeout_remaining_sec'):
        task_command_state_from_dict({
            'status': 'pending',
            'timeout_remaining_sec': value,
        })


def test_stream_path_command_round_trips_waypoints():
    command = TaskCommand(
        kind=CommandKind.LINEAR_PATH_STREAM_RELATIVE,
        group="right_arm",
        waypoints=((0.01, 0.0, 0.0, 0.0, 0.0, 0.0), (0.02, 0.01, 0.0, 0.0, 0.0, 0.0)),
        minimum_time=4.0,
        linear_velocity=0.05,
        angular_velocity=0.3,
    )
    payload = task_command_to_dict(command)
    assert payload["waypoints"] == [
        [0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.02, 0.01, 0.0, 0.0, 0.0, 0.0],
    ]
    restored = task_command_from_dict(payload)
    assert restored == command
    assert restored.waypoints == command.waypoints


def test_task_builder_linear_path_relative_validates_inputs():
    from rby1_planner.control_commands import Task

    task = Task("path").linear_path_relative(
        "right_arm",
        [(0.0, 0.01, 0.0, 0.0, 0.0, 0.0), [0.0, 0.02, 0.0, 0.0, 0.0, 0.0]],
        (3.0, 0.05, 0.3),
    )
    command = task.task_list[0]
    assert command.kind is CommandKind.LINEAR_PATH_STREAM_RELATIVE
    assert command.waypoints == (
        (0.0, 0.01, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.02, 0.0, 0.0, 0.0, 0.0),
    )
    assert command.minimum_time == pytest.approx(3.0)
    assert command.timeout_seconds == pytest.approx(3.0 * 1.5 + 10.0)
    with pytest.raises(ValueError, match="at least one waypoint"):
        Task("empty").linear_path_relative("right_arm", [], (3.0, 0.05, 0.3))
    with pytest.raises(ValueError, match="6 values"):
        Task("short").linear_path_relative("right_arm", [(0.0, 0.0)], (3.0, 0.05, 0.3))
    with pytest.raises(ValueError, match="unknown Cartesian arm"):
        Task("arm").linear_path_relative("torso", [(0.0,) * 6], (3.0, 0.05, 0.3))
