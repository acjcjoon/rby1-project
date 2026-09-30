import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')

from rby1_control.backend_contract import (  # noqa: E402
    TaskCommandState,
    TaskCommandStatus,
)
from rby1_control.control_node import RBY1ControlNode  # noqa: E402
from rby1_control.joint_limit_policy import JointLimitPolicy  # noqa: E402
from rby1_control.manipulator_controller import (  # noqa: E402
    JOINT_NAMES,
    ManipulatorController,
)


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'joint_limits_rby1m_v1_3.yaml'
)


def _joint_state(group, values_deg):
    return SimpleNamespace(
        name=list(JOINT_NAMES[group]),
        position=[math.radians(value) for value in values_deg],
    )


def _make_safety_node(*, mode='cartesian', arm='right_arm'):
    node = object.__new__(RBY1ControlNode)
    node.manipulator_controller = ManipulatorController()
    node._joint_groups_deg = node.manipulator_controller.joint_groups_deg
    node._joint_updated_at = node.manipulator_controller.joint_updated_at
    node._joint_order_verified = (
        node.manipulator_controller.joint_order_verified
    )
    node.joint_limit_policy = JointLimitPolicy.from_yaml(
        CONFIG_PATH,
        expected_joint_names=JOINT_NAMES,
    )
    node.cartesian_joint_limit_margin_deg = 5.0
    node.cartesian_joint_state_timeout_sec = 0.1
    node._lock = threading.RLock()
    node._motion_busy = True
    node._active_motion_mode = mode
    node._active_motion_groups = (arm,)
    node._cartesian_safety_stop_latched = False
    node._active_task_command_id = None
    node._task_commands = {}
    node._cancel_motion_on_accept = False
    node._active_goal_handle = None
    node._active_cancel_requested = False
    node.events = []
    node.stop_calls = []
    node._push_event = lambda level, message: node.events.append(
        (level, message)
    )
    node.cancel_active_motion = lambda: node.stop_calls.append('action')
    node._request_driver_motion_cancel = (
        lambda callback: node.stop_calls.append('driver') or True
    )
    return node


RIGHT_SAFE = [0.0, -5.0, 0.0, -5.0, 0.0, 0.0, 0.0]
LEFT_SAFE = [0.0, 5.0, 0.0, -5.0, 0.0, 0.0, 0.0]


def test_active_cartesian_arm_violation_cancels_once():
    node = _make_safety_node()
    node._joint_state_callback(
        'right_arm',
        _joint_state('right_arm', RIGHT_SAFE),
    )
    assert node.stop_calls == []

    unsafe = list(RIGHT_SAFE)
    unsafe[1] = -3.9
    node._joint_state_callback(
        'right_arm',
        _joint_state('right_arm', unsafe),
    )
    node._joint_state_callback(
        'right_arm',
        _joint_state('right_arm', unsafe),
    )

    assert node.stop_calls == ['action', 'driver']
    assert node._cartesian_safety_stop_latched is True
    assert any('right_arm_1' in message for _, message in node.events)


def test_opposite_arm_and_joint_space_motion_are_not_stopped():
    opposite = _make_safety_node(arm='right_arm')
    unsafe_left = list(LEFT_SAFE)
    unsafe_left[1] = 3.9
    opposite._joint_state_callback(
        'left_arm',
        _joint_state('left_arm', unsafe_left),
    )
    assert opposite.stop_calls == []

    joint_space = _make_safety_node(mode='joint')
    unsafe_right = list(RIGHT_SAFE)
    unsafe_right[1] = -3.9
    joint_space._joint_state_callback(
        'right_arm',
        _joint_state('right_arm', unsafe_right),
    )
    assert joint_space.stop_calls == []


def test_stale_active_arm_feedback_triggers_watchdog_stop():
    node = _make_safety_node()
    node._joint_state_callback(
        'right_arm',
        _joint_state('right_arm', RIGHT_SAFE),
    )
    with node.manipulator_controller.lock:
        node._joint_updated_at['right_arm'] = time.monotonic() - 0.2

    assert node._enforce_active_cartesian_joint_safety() is True
    assert node.stop_calls == ['action', 'driver']
    assert any('stale' in message for _, message in node.events)


def test_preflight_rejects_soft_limit_violation():
    node = _make_safety_node()
    unsafe = list(RIGHT_SAFE)
    unsafe[3] = -3.9
    node.manipulator_controller.update_joint_state(
        'right_arm',
        _joint_state('right_arm', unsafe),
    )

    assert node._cartesian_joint_preflight_ready('right_arm') is False
    assert node.stop_calls == []
    assert any('rejected' in message for _, message in node.events)


def test_task_safety_violation_is_failed_before_cancel():
    node = _make_safety_node()
    command_id = 'task-1'
    node._active_task_command_id = command_id
    node._task_commands[command_id] = TaskCommandState(
        TaskCommandStatus.PENDING,
        'pending',
    )
    node._joint_state_callback(
        'right_arm',
        _joint_state('right_arm', RIGHT_SAFE),
    )

    unsafe = list(RIGHT_SAFE)
    unsafe[1] = -3.9
    node._joint_state_callback(
        'right_arm',
        _joint_state('right_arm', unsafe),
    )

    result = node._task_commands[command_id]
    assert result.status is TaskCommandStatus.FAILED
    assert 'joint safety stop' in result.message


def test_safety_latch_cancels_goal_accepted_after_violation():
    node = object.__new__(RBY1ControlNode)
    node._pending_futures = []
    node._active_goal_handle = None
    node._cancel_motion_on_accept = False
    node._cartesian_safety_stop_latched = True
    node._active_cancel_requested = False
    node._push_event = lambda *args: None
    node._discard_future = lambda future: None
    canceled = []
    node._request_active_goal_cancel = canceled.append

    class ResultFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    result_future = ResultFuture()
    goal_handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: result_future,
    )
    response_future = SimpleNamespace(result=lambda: goal_handle)

    node._motion_goal_response(
        response_future,
        'Cartesian right_arm',
    )

    assert canceled == [goal_handle]
    assert result_future in node._pending_futures
