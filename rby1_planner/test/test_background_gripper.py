"""Background gripper steps: the arm moves while the gripper command runs."""
import pytest

from rby1_planner.backend_contract import (
    TaskBackendState,
    TaskCommandState,
    TaskCommandStatus,
)
from rby1_planner.control_commands import CommandKind, TaskCommand
from rby1_planner.task import build_tasks, wave_both_arms
from rby1_planner.task_commands import (
    BackgroundGripperStep,
    DynamicTaskDefinition,
    Task,
)
from rby1_planner.task_lifecycle import TaskRunStatus
from rby1_planner.task_runner import PlannerTaskRunner

GRIPPER_KINDS = (
    CommandKind.GRIPPER_OPEN,
    CommandKind.GRIPPER_CLOSE,
    CommandKind.GRIPPER_SET,
)
ARM = [0.0] * 7
JOINT_MOTION = [2.0, 1.0, 1.0]


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.active = True

    def cancel(self):
        self.active = False

    def reset(self):
        self.active = True


class FakeNode:
    def create_timer(self, _period_sec, callback):
        self.timer = FakeTimer(callback)
        return self.timer


class FakeBackend:
    """Every command stays pending until the test finishes it."""

    def __init__(self):
        self.now = 100.0
        self.commands = []
        self.command_states = {}
        self.canceled = []
        self.cancel_motion_calls = 0
        self.stop_calls = 0
        self.stream_requests = []

    def task_state(self):
        pending = any(
            state.status is TaskCommandStatus.PENDING
            for state in self.command_states.values()
        )
        return TaskBackendState(
            captured_at=self.now,
            robot_state_updated_at=self.now - 0.1,
            control_state=2,
            stream_enabled=False,
            emo_active=False,
            collision_active=False,
            motion_active=pending,
            joint_groups={'right_arm': (0.0,) * 7},
            joint_updated_at={'right_arm': self.now - 0.1},
            joint_order_verified={'right_arm': True},
            cartesian={},
            cartesian_updated_at={},
            driver_safety_verified=True,
            driver_safety_updated_at=self.now - 0.1,
        )

    def start_task_command(self, command):
        if command.kind in GRIPPER_KINDS and any(
            self.commands[index].kind in GRIPPER_KINDS
            and state.status is TaskCommandStatus.PENDING
            for index, state in enumerate(self.command_states.values())
        ):
            raise RuntimeError('another gripper Task command is active')
        command_id = f'command-{len(self.commands)}'
        self.commands.append(command)
        self.command_states[command_id] = TaskCommandState(
            TaskCommandStatus.PENDING,
            'running',
        )
        return command_id

    def finish(self, index, status=TaskCommandStatus.SUCCEEDED, message='done'):
        self.command_states[f'command-{index}'] = TaskCommandState(
            status,
            message,
        )

    def poll_task_command(self, command_id):
        return self.command_states[command_id]

    def cancel_task_command(self, command_id):
        self.canceled.append(command_id)
        self.command_states[command_id] = TaskCommandState(
            TaskCommandStatus.CANCELED,
            'canceled',
        )

    def cancel_motion(self):
        self.cancel_motion_calls += 1

    def set_velocity(self, vx, vy, wz):
        pass

    def stop(self, publish_immediately=True):
        self.stop_calls += 1

    def request_stream(self, enabled):
        self.stream_requests.append(enabled)


def _runner(backend, results=None):
    return PlannerTaskRunner(
        FakeNode(),
        backend,
        clock=lambda: backend.now,
        on_finished=(results.append if results is not None else None),
    )


def test_builders_wrap_validated_gripper_commands():
    task = (
        Task('g')
        .open_gripper_background('left')
        .close_gripper_background('both', settle_time_sec=0.2)
    )
    opened, closed = task.task_list
    assert isinstance(opened, BackgroundGripperStep)
    assert opened.command.kind is CommandKind.GRIPPER_OPEN
    assert opened.command.group == 'left'
    assert closed.command.kind is CommandKind.GRIPPER_CLOSE
    assert closed.command.seconds == pytest.approx(0.2)
    assert isinstance(task.build(), DynamicTaskDefinition)
    with pytest.raises(ValueError, match='requires a gripper command'):
        BackgroundGripperStep(TaskCommand(kind=CommandKind.DELAY, seconds=1.0))
    with pytest.raises(ValueError):
        Task('bad').open_gripper_background('middle')


def test_arm_step_starts_while_the_background_gripper_is_still_running():
    backend = FakeBackend()
    results = []
    runner = _runner(backend, results)
    task = (
        Task('wave')
        .open_gripper_background('both')
        .joint_absolute('right_arm', ARM, JOINT_MOTION)
        .build()
    )
    runner.start(task)
    assert [c.kind for c in backend.commands] == [CommandKind.GRIPPER_OPEN]
    # The gripper is still pending (backend reports motion_active), yet the
    # arm command is sent on the next tick without an idle failure.
    runner.tick()
    assert [c.kind for c in backend.commands] == [
        CommandKind.GRIPPER_OPEN,
        CommandKind.JOINT_ABSOLUTE,
    ]
    assert runner.active
    # The arm finishes first: the Task still waits for the gripper result.
    backend.finish(1)
    runner.tick()
    assert runner.active and results == []
    backend.finish(0)
    runner.tick()
    assert not runner.active
    assert results[-1].status is TaskRunStatus.SUCCEEDED


def test_background_gripper_failure_fails_the_task_and_cancels_the_arm():
    backend = FakeBackend()
    results = []
    runner = _runner(backend, results)
    task = (
        Task('wave')
        .close_gripper_background('both')
        .joint_absolute('right_arm', ARM, JOINT_MOTION)
        .build()
    )
    runner.start(task)
    runner.tick()
    assert len(backend.commands) == 2
    backend.finish(0, TaskCommandStatus.FAILED, 'gripper tracking timeout')
    runner.tick()
    assert not runner.active
    assert results[-1].status is TaskRunStatus.FAILED
    assert 'background gripper command did not complete' in results[-1].message
    assert 'gripper tracking timeout' in results[-1].message
    assert 'command-1' in backend.canceled


def test_next_background_gripper_waits_for_the_previous_one():
    backend = FakeBackend()
    runner = _runner(backend)
    task = (
        Task('wave')
        .open_gripper_background('both')
        .close_gripper_background('both')
        .joint_absolute('right_arm', ARM, JOINT_MOTION)
        .build()
    )
    runner.start(task)
    runner.tick()
    runner.tick()
    assert [c.kind for c in backend.commands] == [CommandKind.GRIPPER_OPEN]
    backend.finish(0)
    runner.tick()
    assert [c.kind for c in backend.commands] == [
        CommandKind.GRIPPER_OPEN,
        CommandKind.GRIPPER_CLOSE,
    ]
    runner.tick()
    assert backend.commands[-1].kind is CommandKind.JOINT_ABSOLUTE


def test_background_gripper_timeout_fails_the_task():
    backend = FakeBackend()
    results = []
    runner = _runner(backend, results)
    task = (
        Task('wave')
        .open_gripper_background('both')
        .joint_absolute('right_arm', ARM, JOINT_MOTION)
        .build()
    )
    runner.start(task)
    backend.now += 600.0
    runner.tick()
    assert not runner.active
    assert results[-1].status is TaskRunStatus.FAILED
    assert 'background gripper command timed out' in results[-1].message


def test_stop_cancels_the_background_gripper_and_the_arm():
    backend = FakeBackend()
    runner = _runner(backend)
    task = (
        Task('wave')
        .open_gripper_background('both')
        .joint_absolute('right_arm', ARM, JOINT_MOTION)
        .build()
    )
    runner.start(task)
    runner.tick()
    runner.stop()
    assert not runner.active
    assert set(backend.canceled) == {'command-0', 'command-1'}


# wave_both_arms 의 토르소 자세 (ready_pose 높이 정면 / 약 4.5 cm 일어선 자세)
WAVE_TORSO_DOWN = [0.065048, 45.375276, -90.959835, 45.153373, 0.026976, 0.0]
WAVE_TORSO_UP = [0.065048, 40.0, -80.0, 39.568814, 0.026976, 0.0]


def test_wave_both_arms_moves_torso_and_grippers_with_the_arms():
    task = wave_both_arms(count=2)
    assert isinstance(task, DynamicTaskDefinition)
    steps = task.commands
    assert steps[0].kind is CommandKind.GRIPPER_CLOSE
    assert steps[0].group == 'both'
    neutral = dict(steps[1].joint_targets)
    assert set(neutral) == {'torso', 'right_arm', 'left_arm'}
    assert neutral['torso'] == pytest.approx(WAVE_TORSO_DOWN)
    assert neutral['torso'][5] == pytest.approx(0.0)  # facing front
    body = steps[2:-1]
    assert len(body) == 2 * 4
    for cycle in range(2):
        opened, left, closed, right = body[4 * cycle:4 * cycle + 4]
        assert isinstance(opened, BackgroundGripperStep)
        assert opened.command.kind is CommandKind.GRIPPER_OPEN
        assert dict(left.joint_targets)['torso'] == pytest.approx(WAVE_TORSO_UP)
        assert isinstance(closed, BackgroundGripperStep)
        assert closed.command.kind is CommandKind.GRIPPER_CLOSE
        assert dict(right.joint_targets)['torso'] == pytest.approx(
            WAVE_TORSO_DOWN,
        )
    assert dict(steps[-1].joint_targets) == neutral
    # The torso only unfolds: roll and yaw joints stay where they are.
    for index in (0, 4, 5):
        assert WAVE_TORSO_UP[index] == pytest.approx(WAVE_TORSO_DOWN[index])
    with pytest.raises(ValueError, match='count'):
        wave_both_arms(count=0)
    assert build_tasks()['wave_both_arms'].name == 'wave_both_arms'
