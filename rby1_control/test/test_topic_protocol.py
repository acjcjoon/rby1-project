import json

import pytest

from rby1_control.backend_contract import (
    BackendSnapshot,
    TaskBackendState,
    TaskCommandState,
    TaskCommandStatus,
    VelocityCommand,
)
from rby1_control.command_model import CommandKind, TaskCommand
from rby1_control.topic_protocol import (
    COMMAND_KIND,
    backend_snapshot_from_dict,
    backend_snapshot_to_dict,
    decode_message,
    encode_message,
    task_backend_state_from_dict,
    task_backend_state_to_dict,
    task_command_state_from_dict,
    task_command_state_to_dict,
    task_command_from_dict,
    task_command_to_dict,
)


def test_command_envelope_is_versioned_and_round_trips():
    encoded = encode_message(
        COMMAND_KIND,
        {
            "source": "planner",
            "request_id": "command-1",
            "operation": "set_velocity",
            "arguments": {"vx": 0.15, "vy": 0.0, "wz": 0.0},
        },
    )

    decoded = decode_message(encoded, expected_kind=COMMAND_KIND)
    assert decoded["schema_version"] == 1
    assert decoded["source"] == "planner"
    assert decoded["arguments"]["vx"] == pytest.approx(0.15)


def test_decode_rejects_unknown_protocol_version():
    encoded = json.dumps({"schema_version": 99, "kind": COMMAND_KIND})
    with pytest.raises(ValueError, match="unsupported"):
        decode_message(encoded)


def test_decode_rejects_non_finite_json_number():
    encoded = '{"schema_version":1,"kind":"command","vx":NaN}'
    with pytest.raises(ValueError, match="non-finite"):
        decode_message(encoded)


@pytest.mark.parametrize(
    "command",
    [
        TaskCommand(
            kind=CommandKind.JOINT_ABSOLUTE,
            group="torso",
            values=(0.0, 45.0, -90.0, 45.0, 0.0, 0.0),
            minimum_time=5.0,
            velocity_limit=0.2,
            acceleration_limit=0.2,
        ),
        TaskCommand(
            kind=CommandKind.JOINT_ABSOLUTE_MULTI,
            joint_targets=(
                ("right_arm", (0.0,) * 7),
                ("left_arm", (0.0,) * 7),
            ),
            minimum_time=5.0,
            velocity_limit=0.2,
            acceleration_limit=0.2,
        ),
        TaskCommand(
            kind=CommandKind.LINEAR_ABSOLUTE,
            group="right_arm",
            values=(0.4, -0.2, 0.9, 0.0, 0.0, 90.0),
            minimum_time=2.0,
            linear_velocity=0.05,
            angular_velocity=0.2,
            acceleration_scaling=0.3,
        ),
        TaskCommand(
            kind=CommandKind.GRIPPER_OPEN,
            group="left",
        ),
        TaskCommand(
            kind=CommandKind.GRIPPER_CLOSE,
            group="left",
            seconds=1.0,
        ),
        TaskCommand(
            kind=CommandKind.GRIPPER_SET,
            group="left",
            values=(0.6,),
            seconds=1.0,
        ),
    ],
)
def test_task_command_wire_round_trip(command):
    assert task_command_from_dict(task_command_to_dict(command)) == command


@pytest.mark.parametrize(
    ("kind", "values", "seconds"),
    [
        (CommandKind.GRIPPER_OPEN, (), None),
        (CommandKind.GRIPPER_CLOSE, (), 1.0),
        (CommandKind.GRIPPER_SET, (0.6,), 1.0),
    ],
)
def test_gripper_velocity_limit_wire_round_trip(kind, values, seconds):
    command = TaskCommand(
        kind=kind,
        group="left",
        values=values,
        velocity_limit=0.25,
        seconds=seconds,
    )

    payload = task_command_to_dict(command)

    assert payload["velocity_limit"] == pytest.approx(0.25)
    assert task_command_from_dict(payload) == command


@pytest.mark.parametrize(
    "velocity_limit",
    [0.0, -0.1, float("nan"), float("inf"), float("-inf"), True],
)
def test_gripper_rejects_non_positive_or_non_finite_velocity_limit(
    velocity_limit,
):
    with pytest.raises(ValueError, match="velocity_limit"):
        TaskCommand(
            kind=CommandKind.GRIPPER_OPEN,
            group="left",
            velocity_limit=velocity_limit,
        )


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("minimum_time", 1.0),
        ("acceleration_limit", 1.0),
        ("linear_velocity", 1.0),
        ("angular_velocity", 1.0),
        ("acceleration_scaling", 1.0),
    ],
)
def test_gripper_velocity_limit_does_not_allow_other_motion_fields(
    field,
    value,
):
    arguments = {
        "kind": CommandKind.GRIPPER_OPEN,
        "group": "left",
        "velocity_limit": 0.25,
        field: value,
    }
    with pytest.raises(ValueError, match="incompatible motion fields"):
        TaskCommand(**arguments)


def test_backend_snapshot_wire_round_trip():
    snapshot = BackendSnapshot(
        namespace="/rby1",
        cmd_vel_topic="/rby1/cmd_vel",
        cmd_vel_subscribers=1,
        control_state=2,
        power_enabled=True,
        servo_enabled=True,
        stream_enabled=False,
        emo_active=False,
        collision_active=False,
        services_enabled=True,
        rby1_msgs_available=True,
        service_ready={"power": True},
        command=VelocityCommand(0.1, 0.0, 0.0),
        command_stale=False,
        gripper_ready=True,
        gripper_state_fresh=True,
        gripper_positions=(0.25, 0.75),
        gripper_target=(0.0, 1.0),
        gripper_motion_active=True,
        gripper_error=None,
        gripper_default_speed_ratio_per_sec=1.5,
        gripper_max_speed_ratio_per_sec=2.0,
        gripper_acceleration_ratio_per_sec2=4.0,
        gripper_trajectory_rate_hz=50.0,
        gripper_power_voltages=(12.0, 12.0),
        gripper_power_state_fresh=True,
        gripper_power_12v=True,
    )
    assert backend_snapshot_from_dict(backend_snapshot_to_dict(snapshot)) == snapshot


def test_backend_snapshot_accepts_state_without_gripper_fields():
    snapshot = backend_snapshot_from_dict({})
    assert snapshot.gripper_ready is None
    assert snapshot.gripper_state_fresh is False
    assert snapshot.gripper_positions is None
    assert snapshot.gripper_target is None
    assert snapshot.gripper_motion_active is False
    assert snapshot.gripper_default_speed_ratio_per_sec is None
    assert snapshot.gripper_max_speed_ratio_per_sec is None
    assert snapshot.gripper_acceleration_ratio_per_sec2 is None
    assert snapshot.gripper_trajectory_rate_hz is None
    assert snapshot.gripper_power_voltages is None
    assert snapshot.gripper_power_state_fresh is False
    assert snapshot.gripper_power_12v is False


def test_task_backend_state_wire_round_trip():
    state = TaskBackendState(
        captured_at=10.0,
        robot_state_updated_at=9.9,
        control_state=2,
        stream_enabled=False,
        emo_active=False,
        collision_active=False,
        motion_active=False,
        joint_groups={"torso": (0.0,) * 6},
        joint_updated_at={"torso": 9.9},
        joint_order_verified={"torso": True},
        cartesian={"right_arm": (0.0,) * 6},
        cartesian_updated_at={"right_arm": 9.9},
        driver_safety_verified=False,
        driver_safety_updated_at=None,
    )
    assert task_backend_state_from_dict(task_backend_state_to_dict(state)) == state


def test_task_command_state_timeout_hint_round_trips_and_is_optional():
    state = TaskCommandState(
        TaskCommandStatus.PENDING,
        "moving",
        timeout_remaining_sec=12.5,
    )
    assert (
        task_command_state_from_dict(task_command_state_to_dict(state))
        == state
    )

    legacy = task_command_state_from_dict({
        "status": "pending",
        "message": "legacy backend",
    })
    assert legacy.timeout_remaining_sec is None
    assert "timeout_remaining_sec" not in task_command_state_to_dict(legacy)


@pytest.mark.parametrize(
    "value",
    [-0.1, float("nan"), float("inf"), float("-inf"), True, "bad"],
)
def test_task_command_state_rejects_invalid_timeout_hint(value):
    with pytest.raises(ValueError, match="timeout_remaining_sec"):
        task_command_state_from_dict({
            "status": "pending",
            "timeout_remaining_sec": value,
        })
