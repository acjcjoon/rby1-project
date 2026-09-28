import sys
from types import ModuleType

import pytest


try:
    from rby1_control.control_transport import ControlTransport
except ModuleNotFoundError as exc:
    if exc.name not in ("std_msgs", "std_msgs.msg"):
        raise

    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")

    class String:
        pass

    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    from rby1_control.control_transport import ControlTransport


class FakeControlNode:
    def __init__(self):
        self.gripper_commands = []

    def command_gripper(self, action, side="both", *, speed=None):
        self.gripper_commands.append((action, side, speed))


def make_transport():
    node = FakeControlNode()
    transport = object.__new__(ControlTransport)
    transport._node = node
    return transport, node


@pytest.mark.parametrize(
    ("operation", "action"),
    [
        ("open_gripper", "open"),
        ("close_gripper", "close"),
    ],
)
def test_manual_gripper_speed_is_forwarded(operation, action):
    transport, node = make_transport()

    result = transport._dispatch_transport_command(
        operation,
        {"side": "left", "speed": 0.25},
        request_id="",
    )

    assert result == "accepted"
    assert node.gripper_commands == [(action, "left", 0.25)]


@pytest.mark.parametrize(
    ("operation", "action"),
    [
        ("open_gripper", "open"),
        ("close_gripper", "close"),
    ],
)
def test_manual_gripper_without_speed_forwards_none(operation, action):
    transport, node = make_transport()

    transport._dispatch_transport_command(
        operation,
        {"side": "right"},
        request_id="",
    )

    assert node.gripper_commands == [(action, "right", None)]


@pytest.mark.parametrize(
    "speed",
    [
        0.0,
        -0.1,
        float("nan"),
        float("inf"),
        float("-inf"),
        True,
        None,
        "not-a-number",
    ],
)
def test_manual_gripper_rejects_invalid_speed(speed):
    transport, node = make_transport()

    with pytest.raises(ValueError, match="speed must be"):
        transport._dispatch_transport_command(
            "open_gripper",
            {"speed": speed},
            request_id="",
        )

    assert node.gripper_commands == []
