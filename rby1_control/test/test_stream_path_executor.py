"""Control-node stream path executor: safety aborts and driver feedback."""
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
from rby1_control.command_model import CommandKind, TaskCommand  # noqa: E402
from rby1_control import control_node as control_node_module  # noqa: E402
from rby1_control.control_node import RBY1ControlNode  # noqa: E402
from rby1_control.joint_limit_policy import JointLimitPolicy  # noqa: E402
from rby1_control.manipulator_controller import (  # noqa: E402
    JOINT_NAMES,
    ManipulatorController,
)
from rby1_control.arm_kinematics import IkResult  # noqa: E402
import numpy as np  # noqa: E402


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'joint_limits_rby1m_v1_3.yaml'
)
RIGHT_SAFE = [0.0, -5.0, 0.0, -5.0, 0.0, 0.0, 0.0]
START = (0.4, -0.2, 0.9, 0.0, 0.0, -90.0)


class FakeFuture:
    def __init__(self, result=None, error=None):
        self._result = result
        self._error = error
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def fire(self):
        for callback in list(self.callbacks):
            callback(self)


class FakeGoalHandle:
    def __init__(self, accepted=True, success=True):
        self.accepted = accepted
        self.result_future = FakeFuture(
            SimpleNamespace(result=SimpleNamespace(success=success))
        )

    def get_result_async(self):
        return self.result_future


class FakeStreamClient:
    def __init__(self):
        self.goals = []
        self.futures = []
        self.ready = True

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        future = FakeFuture(FakeGoalHandle())
        self.futures.append(future)
        return future


class FakeTimer:
    def __init__(self):
        self.canceled = False

    def cancel(self):
        self.canceled = True


class FakeKinematics:
    """IK stand-in: the arm 'joints' are the target position and RPY."""

    dof = 26
    arms = ('right_arm', 'left_arm')

    def __init__(self):
        self.solve_calls = 0
        self.converged = True
        self.fail_after_calls = None

    def joint_vector_from_groups(self, groups_deg):
        q = np.zeros(self.dof)
        q[:7] = [math.radians(v) for v in groups_deg['right_arm']]
        return q

    def arm_columns(self, arm):
        return np.arange(7)

    def arm_values(self, q_full, arm):
        return tuple(float(v) for v in q_full[:7])

    def solve(self, arm, q_seed_full, target, **kwargs):
        self.solve_calls += 1
        q = np.array(q_seed_full, dtype=float)
        # Encode the Cartesian target in the first joints so tests can
        # inspect what was streamed; keep the step small like real IK.
        q[0] = float(target[0, 3])
        q[1] = float(target[1, 3])
        q[2] = float(target[2, 3])
        converged = self.converged and not (
            self.fail_after_calls is not None
            and self.solve_calls > self.fail_after_calls
        )
        return IkResult(
            arm=arm,
            q_full=q,
            q_arm_rad=tuple(float(v) for v in q[:7]),
            converged=converged,
            iterations=3,
            position_error_m=0.0 if converged else 0.5,
            orientation_error_rad=0.0,
        )


def _joint_state(group, values_deg):
    return SimpleNamespace(
        name=list(JOINT_NAMES[group]),
        position=[math.radians(value) for value in values_deg],
    )


def _make_node():
    node = object.__new__(RBY1ControlNode)
    node.manipulator_controller = ManipulatorController()
    node._joint_groups_deg = node.manipulator_controller.joint_groups_deg
    node._joint_updated_at = node.manipulator_controller.joint_updated_at
    node._joint_order_verified = (
        node.manipulator_controller.joint_order_verified
    )
    node._cartesian_state = node.manipulator_controller.cartesian_state
    node._cartesian_updated_at = (
        node.manipulator_controller.cartesian_updated_at
    )
    node.joint_limit_policy = JointLimitPolicy.from_yaml(
        CONFIG_PATH,
        expected_joint_names=JOINT_NAMES,
    )
    node.cartesian_joint_limit_margin_deg = 5.0
    node.cartesian_joint_state_timeout_sec = 0.5
    node.cartesian_links = {'right_arm': ('base', 'ee_right')}
    node.stream_path_kinematics = FakeKinematics()
    node._stream_path_kinematics_error = None
    node.stream_path_ik_max_iterations = 30
    node.stream_path_ik_position_tolerance_m = 0.0005
    node.stream_path_ik_orientation_tolerance_rad = 0.002
    # The fake IK encodes Cartesian targets as joint values, so the per-tick
    # step guard is relaxed here and tightened by the test that exercises it.
    node.stream_path_max_joint_step_rad = 10.0
    node._joint_feedback_ready = lambda group: True
    node.stream_path_rate_hz = 20.0
    node.stream_path_tick_minimum_time_sec = 0.1
    node.stream_path_watchdog_sec = 0.25
    node.stream_path_settle_sec = 0.3
    node.stream_path_linear_velocity_limit = 0.1
    node.stream_path_angular_velocity_limit = 0.5
    node._lock = threading.RLock()
    node._motion_busy = False
    node._active_motion_kind = None
    node._active_motion_mode = None
    node._active_motion_groups = ()
    node._active_goal_handle = None
    node._active_task_command_id = None
    node._active_control_mode = 'position'
    node._active_impedance_context = None
    node._cancel_motion_on_accept = False
    node._active_cancel_requested = False
    node._cartesian_safety_stop_latched = False
    node._task_command_sequence = 0
    node._task_commands = {}
    node._pending_futures = []
    node._stream_path = None
    node._stream_path_timer = None
    node.stream_enabled = True
    node.services_enabled = True
    node.stream_joint_action_client = FakeStreamClient()
    node.events = []
    node.stream_requests = []
    node.driver_cancels = 0
    node.safety_reason = None
    node._push_event = lambda level, message: node.events.append(
        (level, message)
    )
    node.request_stream = lambda enabled, value=0.0: (
        node.stream_requests.append(enabled)
    )
    node._request_driver_motion_cancel = (
        lambda callback: node.driver_cancels.__class__
        and setattr(node, 'driver_cancels', node.driver_cancels + 1) or True
    )
    node._live_motion_safety_reason = lambda now=None: node.safety_reason
    node._motion_command_allowed = (
        lambda: node.safety_reason is None and not node._motion_busy
    )
    node.create_timer = lambda period, callback: FakeTimer()
    node.destroy_timer = lambda timer: None
    node._cartesian_feedback_ready = lambda arm: True
    node._joint_state_callback('right_arm', _joint_state('right_arm', RIGHT_SAFE))
    node.manipulator_controller.cartesian_state['right_arm'] = START
    node.manipulator_controller.cartesian_updated_at['right_arm'] = (
        time.monotonic()
    )
    return node


def _path_command(duration=10.0, waypoints=None):
    return TaskCommand(
        kind=CommandKind.LINEAR_PATH_STREAM_ABSOLUTE,
        group='right_arm',
        waypoints=waypoints or (
            (0.4, -0.1, 0.9, 0.0, 0.0, -90.0),
            (0.5, -0.1, 0.9, 0.0, 0.0, -90.0),
        ),
        minimum_time=duration,
        linear_velocity=0.05,
        angular_velocity=0.3,
    )


@pytest.fixture(autouse=True)
def _real_messages(monkeypatch):
    """Stand-ins for rby1_msgs so the executor can build goals without ROS."""

    class Vector:
        def __init__(self):
            self.x = self.y = self.z = 0.0
            self.w = 1.0

    class Transform:
        def __init__(self):
            self.translation = Vector()
            self.rotation = Vector()

    class CartesianCommand:
        def __init__(self):
            self.ref_link = ''
            self.target_link = ''
            self.transform = Transform()
            self.minimum_time = 0.0
            self.linear_velocity_limit = 0.0
            self.angular_velocity_limit = 0.0
            self.use_impedance = False

    class JointCommand:
        def __init__(self):
            self.position = []
            self.minimum_time = 0.0
            self.use_impedance = False
            self.use_group_joint = False

    class StreamJointCommand:
        def __init__(self):
            self.torso = JointCommand()
            self.right_arm = JointCommand()
            self.left_arm = JointCommand()
            self.head = JointCommand()

    class StreamJoint:
        class Goal:
            def __init__(self):
                self.command = None

    monkeypatch.setattr(control_node_module, 'JointCommand', JointCommand)
    monkeypatch.setattr(
        control_node_module, 'StreamJointCommand', StreamJointCommand,
    )
    monkeypatch.setattr(control_node_module, 'StreamJoint', StreamJoint)


def test_start_sends_first_target_and_marks_motion_busy():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())

    assert node._task_commands[command_id].status is TaskCommandStatus.PENDING
    assert node._motion_busy is True
    assert node._active_motion_mode == 'stream_path'
    client = node.stream_joint_action_client
    assert len(client.goals) == 1
    target = client.goals[0].command.right_arm
    assert len(target.position) == 7
    # The fake IK stores the Cartesian target in the first joints; the first
    # target is sent a few microseconds after the start time.
    assert target.position[0] == pytest.approx(START[0], abs=1e-3)
    assert target.position[1] == pytest.approx(START[1], abs=1e-3)
    assert target.minimum_time == pytest.approx(0.1)
    # Pre-flight solved both waypoints before the first tick.
    assert node.stream_path_kinematics.solve_calls == 3
    # The runtime guard refuses a second motion while the path is active.
    with pytest.raises(RuntimeError):
        node._start_stream_path_task_command(_path_command())


def test_start_requires_stream_on_and_rejects_fast_paths():
    node = _make_node()
    node.stream_enabled = False
    with pytest.raises(RuntimeError, match='stream control is not active'):
        node._start_stream_path_task_command(_path_command())
    node.stream_enabled = True
    with pytest.raises(RuntimeError, match='needs at least'):
        node._start_stream_path_task_command(_path_command(duration=1.0))
    assert node._stream_path is None and node._motion_busy is False


def test_preflight_ik_failure_rejects_before_motion():
    node = _make_node()
    node.stream_path_kinematics.converged = False
    with pytest.raises(RuntimeError, match='waypoint 1/2: IK did not converge'):
        node._start_stream_path_task_command(_path_command())
    assert node._stream_path is None and node._motion_busy is False
    assert node.stream_joint_action_client.goals == []


def test_missing_kinematics_rejects_with_reason():
    node = _make_node()
    node.stream_path_kinematics = None
    node._stream_path_kinematics_error = 'model file missing'
    with pytest.raises(RuntimeError, match='model file missing'):
        node._start_stream_path_task_command(_path_command())


def test_tick_ik_failure_aborts_and_closes_stream():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    node.stream_path_kinematics.fail_after_calls = (
        node.stream_path_kinematics.solve_calls
    )
    node._stream_path_tick()
    state = node._task_commands[command_id]
    assert state.status is TaskCommandStatus.FAILED
    assert 'IK did not converge' in state.message
    assert node.stream_requests == [False]
    assert node.driver_cancels == 1


def test_tick_rejects_a_joint_limit_violation_in_the_solution():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    path = node._stream_path
    # Stream a target far outside the joint range (fake IK copies x into q0).
    path['plan'] = SimpleNamespace(
        duration_sec=10.0,
        final_pose=START,
        target_at=lambda t: (9.0, START[1], START[2], 0.0, 0.0, 0.0, 1.0),
    )
    node.stream_path_max_joint_step_rad = 100.0
    node._stream_path_tick()
    state = node._task_commands[command_id]
    assert state.status is TaskCommandStatus.FAILED
    assert 'outside the CS soft limit' in state.message


def test_tick_rejects_a_joint_jump_between_targets():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    path = node._stream_path
    path['plan'] = SimpleNamespace(
        duration_sec=10.0,
        final_pose=START,
        target_at=lambda t: (START[0] + 0.3, START[1], START[2], 0.0, 0.0, 0.0, 1.0),
    )
    node.stream_path_max_joint_step_rad = 0.05
    node._stream_path_tick()
    state = node._task_commands[command_id]
    assert state.status is TaskCommandStatus.FAILED
    assert 'exceeds the per-tick limit' in state.message


def test_driver_abort_stops_sending_closes_stream_and_fails_task():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    client = node.stream_joint_action_client
    future = client.futures[0]
    future._result.result_future._result.result.success = False
    future.fire()
    future._result.result_future.fire()

    state = node._task_commands[command_id]
    assert state.status is TaskCommandStatus.FAILED
    assert 'aborted by the driver' in state.message
    assert node._stream_path is None and node._motion_busy is False
    assert node.stream_requests == [False]
    assert node.driver_cancels == 1
    sent = len(client.goals)
    node._stream_path_tick()
    assert len(client.goals) == sent


def test_rejected_goal_fails_task():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    future = node.stream_joint_action_client.futures[0]
    future._result.accepted = False
    future.fire()
    assert node._task_commands[command_id].status is TaskCommandStatus.FAILED
    assert 'rejected' in node._task_commands[command_id].message


def test_runtime_safety_reason_aborts_next_tick():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    node.safety_reason = 'collision state is active or unknown'
    node._stream_path_tick()
    state = node._task_commands[command_id]
    assert state.status is TaskCommandStatus.FAILED
    assert 'collision state' in state.message
    assert node.stream_requests == [False]


def test_joint_soft_limit_violation_aborts_from_joint_state_callback():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    violating = list(RIGHT_SAFE)
    violating[1] = 500.0
    node._joint_state_callback('right_arm', _joint_state('right_arm', violating))
    state = node._task_commands[command_id]
    assert state.status is TaskCommandStatus.FAILED
    assert 'joint safety' in state.message


def test_tick_watchdog_aborts_after_a_stall():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    node._stream_path['last_tick_at'] = time.monotonic() - 1.0
    node._stream_path_tick()
    state = node._task_commands[command_id]
    assert state.status is TaskCommandStatus.FAILED
    assert 'watchdog' in state.message


def test_stream_dropped_aborts_without_driver_cancel():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    node.stream_enabled = False
    node._stream_path_tick()
    assert node._task_commands[command_id].status is TaskCommandStatus.FAILED
    assert node.driver_cancels == 0


def test_cancel_motion_and_cancel_active_motion_stop_the_path():
    node = _make_node()
    command_id = node._start_stream_path_task_command(_path_command())
    node.cancel_active_motion()
    assert node._task_commands[command_id].status is TaskCommandStatus.CANCELED
    assert node._stream_path is None

    node = _make_node()
    node._cancel_gripper_motion = lambda: None
    command_id = node._start_stream_path_task_command(_path_command())
    node.cancel_motion()
    assert node._task_commands[command_id].status is TaskCommandStatus.CANCELED
    assert node.stream_requests == [False]


def test_path_completes_after_settle_and_sends_final_target():
    node = _make_node()
    # 0.2 m at 0.05 m/s needs 4 s; 5 s is accepted.
    command_id = node._start_stream_path_task_command(_path_command(duration=5.0))
    path = node._stream_path
    path['started_at'] = time.monotonic() - 5.5
    node._stream_path_tick()
    assert node._task_commands[command_id].status is TaskCommandStatus.PENDING
    path['finished_at'] = time.monotonic() - 1.0
    node._stream_path_tick()
    state = node._task_commands[command_id]
    assert state.status is TaskCommandStatus.SUCCEEDED
    assert node._stream_path is None and node._motion_busy is False
    # Success leaves the stream to the Task runner: no OFF request here.
    assert node.stream_requests == []
    last = node.stream_joint_action_client.goals[-1].command.right_arm
    assert last.position[0] == pytest.approx(0.5)
