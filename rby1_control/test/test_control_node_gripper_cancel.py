import pytest


pytest.importorskip('rclpy')

from rby1_control.backend_contract import (  # noqa: E402
    TaskCommandState,
    TaskCommandStatus,
)
from rby1_control.command_model import CommandKind  # noqa: E402
from rby1_control.control_node import RBY1ControlNode  # noqa: E402
from rby1_control.gripper_controller import GripperController  # noqa: E402


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class _Publisher:
    def __init__(self):
        self.targets = []

    def publish(self, message):
        self.targets.append(tuple(message.data))


class _NotReadyCancelClient:
    def service_is_ready(self):
        return False

    def call_async(self, request):
        raise AssertionError(
            'an unavailable cancel service must not be called'
        )


def _make_node(cancel_client):
    clock = _Clock()
    controller = GripperController(
        state_timeout_sec=10.0,
        clock=clock,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))

    node = object.__new__(RBY1ControlNode)
    node._gripper_controller = controller
    node.gripper_command_pub = _Publisher()
    node._active_gripper_task_id = None
    node._active_gripper_task_kind = None
    node._active_gripper_feedback_marker = None
    node._active_gripper_settle_duration_sec = None
    node._active_gripper_settle_deadline = None
    node._task_commands = {}
    node.cancel_control_client = cancel_client
    node.cancel_control_service = 'cancel_control'
    node._pending_futures = []
    node._push_event = lambda *args: None
    return node, controller, clock


@pytest.mark.parametrize(
    'cancel_client',
    [None, _NotReadyCancelClient()],
    ids=['service-disabled', 'service-not-ready'],
)
def test_cancel_motion_stops_manual_gripper_before_service_return(
    cancel_client,
):
    node, controller, clock = _make_node(cancel_client)
    node.command_gripper('close', 'left', speed=0.5)
    clock.now = 0.5
    node._check_gripper_command()
    node.gripper_command_pub.targets.clear()

    node.cancel_motion()

    status = controller.status()
    assert status.motion_active is False
    assert status.profile_active is False
    assert node.gripper_command_pub.targets == [(0.0, 0.0)]

    clock.now = 2.0
    node._check_gripper_command()
    assert node.gripper_command_pub.targets == [(0.0, 0.0)]


def test_cancel_motion_cancels_active_gripper_task_and_clears_state():
    node, controller, clock = _make_node(None)
    controller.command('close', 'left', speed=0.5)
    controller.take_command_target()
    # Close/Set Tasks remain active for settling after position feedback has
    # completed the profile. Cancellation must clear that pending endpoint too.
    controller.update_state((0.0, 1.0))
    assert controller.status().motion_active is False
    assert controller.status().sample_pending is True
    command_id = 'task-1'
    node._task_commands[command_id] = TaskCommandState(
        TaskCommandStatus.PENDING,
        'Gripper command pending',
    )
    node._active_gripper_task_id = command_id
    node._active_gripper_task_kind = CommandKind.GRIPPER_CLOSE
    node._active_gripper_feedback_marker = 1
    node._active_gripper_settle_duration_sec = 1.0

    node.cancel_motion()

    assert node._task_commands[command_id].status is TaskCommandStatus.CANCELED
    assert node._active_gripper_task_id is None
    assert node._active_gripper_task_kind is None
    assert node._active_gripper_feedback_marker is None
    assert node._active_gripper_settle_duration_sec is None
    assert node._active_gripper_settle_deadline is None
    assert controller.status().motion_active is False
    assert controller.status().profile_active is False
    assert node.gripper_command_pub.targets == [(0.0, 1.0)]

    clock.now = 2.0
    node._check_gripper_command()
    assert node.gripper_command_pub.targets == [(0.0, 1.0)]
