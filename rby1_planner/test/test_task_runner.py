import pytest

from rby1_planner.backend_contract import (
    TaskBackendState,
    TaskCommandState,
    TaskCommandStatus,
)
from rby1_planner.control_commands import CommandKind, TaskCommand
from rby1_planner.observation import ObjectObservation
from rby1_planner.navigation_protocol import (
    NavigationCommandState,
    NavigationCommandStatus,
)
from rby1_planner.task_commands import Task
from rby1_planner.task_runner import PlannerTaskRunner


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
    def __init__(self):
        self.now = 100.0
        self.stream_enabled = False
        self.commands = []
        self.command_states = {}
        self.velocity_calls = []
        self.stop_calls = 0
        self.cancel_calls = 0
        self.stream_requests = []
        self.motion_active = False
        self.cancel_error = None
        self.cartesian = {'right_arm': (0.0,) * 6}

    def task_state(self):
        return TaskBackendState(
            captured_at=self.now,
            robot_state_updated_at=self.now - 0.1,
            control_state=2,
            stream_enabled=self.stream_enabled,
            emo_active=False,
            collision_active=False,
            motion_active=self.motion_active,
            joint_groups={'right_arm': (0.0,) * 7},
            joint_updated_at={'right_arm': self.now - 0.1},
            joint_order_verified={'right_arm': True},
            cartesian=self.cartesian,
            cartesian_updated_at={
                arm: self.now - 0.1 for arm in self.cartesian
            },
            driver_safety_verified=True,
            driver_safety_updated_at=self.now - 0.1,
        )

    def start_task_command(self, command):
        command_id = f'command-{len(self.commands)}'
        self.commands.append(command)
        self.command_states[command_id] = TaskCommandState(
            TaskCommandStatus.SUCCEEDED,
            'done',
        )
        return command_id

    def poll_task_command(self, command_id):
        return self.command_states[command_id]

    def cancel_task_command(self, command_id):
        self.command_states[command_id] = TaskCommandState(
            TaskCommandStatus.CANCELED,
            'canceled',
        )

    def cancel_motion(self):
        self.cancel_calls += 1
        if self.cancel_error is not None:
            raise self.cancel_error

    def set_velocity(self, vx, vy, wz):
        self.velocity_calls.append((vx, vy, wz))

    def stop(self, publish_immediately=True):
        assert publish_immediately
        self.stop_calls += 1

    def request_stream(self, enabled):
        self.stream_requests.append(enabled)


class FakeCamera:
    def __init__(self):
        self.marker = 1_000_000_000
        self.marker_calls = 0
        self.observation = None
        self.observation_error = None
        self.observation_calls = []
        self.reason = 'no transform for the new detection'

    def capture_marker(self):
        self.marker_calls += 1
        result = self.marker
        self.marker += 1
        return result

    def get_observation(self, object_id, **kwargs):
        self.observation_calls.append((object_id, kwargs))
        if self.observation_error is not None:
            raise self.observation_error
        return self.observation

    def unavailable_reason(self, _object_id):
        return self.reason


class FakeNavigation:
    def __init__(self):
        self.commands = []
        self.canceled = []
        self.states = {}

    def send_move_to(self, x, y, yaw, timeout_sec):
        command_id = f'nav-{len(self.commands)}'
        self.commands.append((x, y, yaw, timeout_sec))
        self.states[command_id] = NavigationCommandState(
            NavigationCommandStatus.PENDING,
        )
        return command_id

    def poll(self, command_id):
        return self.states[command_id]

    def cancel(self, command_id):
        self.canceled.append(command_id)


def _camera_observation(
    *,
    object_id='tag_0',
    stamp_ns=1_000_000_001,
    position=(0.3, -0.2, 0.8),
    orientation_xyzw=(0.0, 0.0, 2 ** -0.5, 2 ** -0.5),
):
    return ObjectObservation(
        object_id=object_id,
        frame_id='base',
        stamp_ns=stamp_ns,
        received_at_ns=stamp_ns,
        position=position,
        orientation_xyzw=orientation_xyzw,
        confidence=1.0,
    )


def test_runner_uses_ros_timer_and_resolves_relative_joint_command():
    node = FakeNode()
    backend = FakeBackend()
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    assert not node.timer.active

    task = Task('relative').joint_relative(
        'right_arm',
        [1, 2, 3, 4, 5, 6, 7],
        [2.0, 0.2, 0.2],
    )
    runner.start(task.build())

    assert node.timer.active
    assert backend.commands[0].values == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    runner.tick()
    assert not runner.active
    assert not node.timer.active
    assert messages[-1] == 'Task completed: relative'


def test_runner_rewinds_for_requested_total_run_count():
    node = FakeNode()
    backend = FakeBackend()
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('repeat-three').joint_relative(
        'right_arm',
        [1, 2, 3, 4, 5, 6, 7],
        [2.0, 0.2, 0.2],
    ).rewind(3).build()

    runner.start(task)
    runner.tick()
    runner.tick()
    runner.tick()
    runner.tick()
    runner.tick()
    runner.tick()

    assert len(backend.commands) == 3
    assert not runner.active
    assert 'Task rewind: repeat-three run 2/3' in messages
    assert 'Task rewind: repeat-three run 3/3' in messages
    assert messages[-1] == 'Task completed: repeat-three'


def test_runner_rewinds_forever_until_stopped():
    node = FakeNode()
    backend = FakeBackend()
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('repeat-forever').joint_relative(
        'right_arm',
        [1, 2, 3, 4, 5, 6, 7],
        [2.0, 0.2, 0.2],
    ).rewind().build()

    runner.start(task)
    runner.tick()
    runner.tick()
    runner.tick()

    assert len(backend.commands) == 2
    assert runner.active
    assert 'Task rewind: repeat-forever run 2/infinite' in messages
    assert 'Task rewind: repeat-forever run 3/infinite' in messages

    runner.stop()
    assert not runner.active


def test_move_to_publishes_once_and_waits_for_navigation_success():
    node = FakeNode()
    backend = FakeBackend()
    navigation = FakeNavigation()
    runner = PlannerTaskRunner(
        node,
        backend,
        navigation=navigation,
        clock=lambda: backend.now,
    )

    runner.start(Task('navigate').move_to(0.3, -0.1, 0.2).build())
    assert backend.stream_requests == [True]
    backend.stream_enabled = True
    runner.tick()
    assert navigation.commands == [(0.3, -0.1, 0.2, 30.0)]
    assert runner.active

    navigation.states['nav-0'] = NavigationCommandState(
        NavigationCommandStatus.RUNNING,
        0.5,
        'moving',
    )
    runner.tick()
    assert runner.active
    assert navigation.commands == [(0.3, -0.1, 0.2, 30.0)]

    navigation.states['nav-0'] = NavigationCommandState(
        NavigationCommandStatus.SUCCEEDED,
        1.0,
        'goal reached',
    )
    runner.tick()
    assert backend.stream_requests[-1] is False
    backend.stream_enabled = False
    runner.tick()
    assert not runner.active


def test_stopping_move_to_publishes_navigation_cancel():
    node = FakeNode()
    backend = FakeBackend()
    navigation = FakeNavigation()
    runner = PlannerTaskRunner(
        node,
        backend,
        navigation=navigation,
        clock=lambda: backend.now,
    )
    runner.start(Task('navigate').move_to(0.3, 0.0, 0.0).build())
    backend.stream_enabled = True
    runner.tick()

    runner.stop()

    assert navigation.canceled == ['nav-0']
    assert not runner.active


def test_move_to_timeout_cancels_navigation_once():
    node = FakeNode()
    backend = FakeBackend()
    navigation = FakeNavigation()
    runner = PlannerTaskRunner(
        node,
        backend,
        navigation=navigation,
        clock=lambda: backend.now,
    )
    runner.start(
        Task('navigate').move_to(
            0.3,
            0.0,
            0.0,
            timeout_sec=1.0,
        ).build()
    )
    backend.stream_enabled = True
    runner.tick()

    backend.now += 6.1
    runner.tick()

    assert navigation.canceled == ['nav-0']
    assert not runner.active


def test_runner_forwards_common_gripper_commands_to_backend():
    node = FakeNode()
    backend = FakeBackend()
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)
    task = Task('gripper').set_gripper('left', 0.6).build()

    runner.start(task)

    assert len(backend.commands) == 1
    command = backend.commands[0]
    assert command.kind is CommandKind.GRIPPER_SET
    assert command.group == 'left'
    assert command.values == pytest.approx((0.6,))
    assert command.seconds == pytest.approx(1.0)

    runner.tick()
    assert not runner.active


def test_stop_while_waiting_for_base_stream_requests_stream_off():
    node = FakeNode()
    backend = FakeBackend()
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)
    task = Task('base').base_velocity(0.1, 0.0, 0.0, 1.0).build()

    runner.start(task)
    assert backend.stream_requests == [True]
    runner.stop()

    assert backend.stop_calls == 1
    assert backend.stream_requests == [True, False]
    assert not runner.active


def test_runner_delay_is_non_blocking():
    node = FakeNode()
    backend = FakeBackend()
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)
    runner.start(Task('wait').delay(100).build())

    assert runner.active
    backend.now += 0.05
    runner.tick()
    assert runner.active
    backend.now += 0.06
    runner.tick()
    assert not runner.active


def test_runner_turns_stream_off_before_joint_motion():
    node = FakeNode()
    backend = FakeBackend()
    backend.stream_enabled = True
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)
    task = Task('joint').joint_absolute(
        'right_arm',
        [0.0] * 7,
        [2.0, 0.2, 0.2],
    ).build()

    runner.start(task)

    assert backend.stream_requests == [False]
    assert backend.commands == []

    backend.stream_enabled = False
    runner.tick()

    assert len(backend.commands) == 1


def test_runner_turns_stream_off_before_camera_capture():
    node = FakeNode()
    backend = FakeBackend()
    backend.stream_enabled = True
    camera = FakeCamera()
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        clock=lambda: backend.now,
    )
    task = Task('camera').camera_linear_absolute(
        'right_arm',
        'tag_0',
        [1.0, 0.1, 0.2, 0.5],
    ).build()

    runner.start(task)

    assert backend.stream_requests == [False]
    assert camera.marker_calls == 0

    backend.stream_enabled = False
    runner.tick()

    assert camera.marker_calls == 1


def test_runner_rejects_stale_robot_state_before_start():
    node = FakeNode()
    backend = FakeBackend()
    backend.now = 10.0

    original_task_state = backend.task_state

    def stale_state():
        state = original_task_state()
        return TaskBackendState(
            **{
                **state.__dict__,
                'robot_state_updated_at': 8.0,
            }
        )

    backend.task_state = stale_state
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)

    with pytest.raises(RuntimeError, match='robot state is stale'):
        runner.start(Task('wait').delay(100).build())


def test_base_velocity_is_refreshed_and_stopped():
    node = FakeNode()
    backend = FakeBackend()
    backend.stream_enabled = True
    runner = PlannerTaskRunner(node, backend, clock=lambda: backend.now)
    runner.start(Task('base').base_velocity(0.1, 0.0, 0.0, 0.1).build())

    backend.now += 0.05
    runner.tick()
    assert len(backend.velocity_calls) == 2

    backend.now += 0.06
    runner.tick()
    assert backend.stop_calls == 1
    assert backend.stream_requests == [False]

    backend.stream_enabled = False
    runner.tick()
    assert not runner.active


def test_base_motion_fails_safe_if_manipulation_becomes_active():
    node = FakeNode()
    backend = FakeBackend()
    backend.stream_enabled = True
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    runner.start(Task('base').base_velocity(0.1, 0.0, 0.0, 1.0).build())

    backend.motion_active = True
    backend.now += 0.05
    runner.tick()

    assert not runner.active
    assert backend.stop_calls == 1
    assert backend.stream_requests[-1] is False
    assert messages[-1].startswith('Task failed (base):')


def test_stop_finishes_runner_when_cancellation_raises():
    node = FakeNode()
    backend = FakeBackend()
    backend.cancel_error = RuntimeError('cancel transport unavailable')
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    runner.start(Task('delay').delay(1.0).build())

    runner.stop()

    assert not runner.active
    assert not node.timer.active
    assert 'Task cancel warning: cancel transport unavailable' in messages
    assert messages[-1] == 'Task stopped'


def test_camera_step_resolves_only_after_previous_motion_completes():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('camera-sequence')
    task.joint_absolute(
        'right_arm',
        (1.0,) * 7,
        (1.0, 0.2, 0.2),
    )
    task.camera_linear_absolute(
        'right_arm',
        'tag_0',
        (2.0, 0.05, 0.2, 0.4),
        detection_timeout_sec=1.0,
        max_age_sec=0.25,
        minimum_confidence=0.7,
    )
    task.joint_absolute(
        'right_arm',
        (2.0,) * 7,
        (1.0, 0.2, 0.2),
    )

    runner.start(task.build())
    assert len(backend.commands) == 1
    assert camera.marker_calls == 0
    assert camera.observation_calls == []

    backend.command_states['command-0'] = TaskCommandState(
        TaskCommandStatus.PENDING,
        'moving',
    )
    runner.tick()
    assert camera.marker_calls == 0

    backend.command_states['command-0'] = TaskCommandState(
        TaskCommandStatus.SUCCEEDED,
        'done',
    )
    backend.motion_active = True
    runner.tick()
    assert camera.marker_calls == 0

    backend.now += 0.05
    runner.tick()
    assert camera.marker_calls == 0

    backend.motion_active = False
    backend.now += 0.05
    runner.tick()
    assert camera.marker_calls == 1
    assert camera.observation_calls == []
    assert len(backend.commands) == 1

    runner.tick()
    assert camera.marker_calls == 1
    assert camera.observation_calls == [(
        'tag_0',
        {
            'target_frame': 'base',
            'newer_than_ns': 1_000_000_000,
            'max_age_sec': 0.25,
            'minimum_confidence': 0.7,
        },
    )]
    assert len(backend.commands) == 1

    camera.observation = _camera_observation()
    runner.tick()
    assert len(backend.commands) == 2
    resolved = backend.commands[1]
    assert type(resolved) is TaskCommand
    assert resolved.kind is CommandKind.LINEAR_ABSOLUTE
    assert resolved.group == 'right_arm'
    assert resolved.values == pytest.approx(
        (0.3, -0.2, 0.8, 0.0, 0.0, 90.0)
    )
    assert resolved.minimum_time == pytest.approx(2.0)
    assert resolved.linear_velocity == pytest.approx(0.05)
    assert resolved.angular_velocity == pytest.approx(0.2)
    assert resolved.acceleration_scaling == pytest.approx(0.4)

    runner.tick()
    assert len(backend.commands) == 3
    assert backend.commands[2].kind is CommandKind.JOINT_ABSOLUTE
    assert camera.marker_calls == 1

    runner.tick()
    assert not runner.active
    assert messages[-1] == 'Task completed: camera-sequence'


def test_camera_yaw_symmetry_selects_equivalent_yaw_nearest_current_ee():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    camera.observation = _camera_observation(
        orientation_xyzw=(0.0, 0.0, 1.0, 0.0),
    )
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('symmetric-yaw').camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
        yaw_only=True,
        yaw_symmetry_deg=180.0,
        object_to_end_effector_position=(0.0, 0.0, 0.1),
    )

    runner.start(task.build())
    runner.tick()

    assert len(backend.commands) == 1
    assert backend.commands[0].values[5] == pytest.approx(0.0)
    assert any(
        'raw=+180.000 deg' in message
        and 'selected=+0.000 deg' in message
        for message in messages
    )


@pytest.mark.parametrize(
    'current_yaw,raw_yaw,expected_yaw',
    [
        (0.0, 0.0, 0.0),
        (0.0, 180.0, 0.0),
        (0.0, -179.0, 1.0),
        (160.0, 170.0, 170.0),
        (-170.0, 170.0, 170.0),
    ],
)
def test_rectangular_yaw_symmetry_math_selects_shortest_equivalent(
    current_yaw,
    raw_yaw,
    expected_yaw,
):
    delta = PlannerTaskRunner._wrap_periodic_degrees(
        raw_yaw - current_yaw,
        180.0,
    )
    selected = PlannerTaskRunner._normalize_degrees(current_yaw + delta)

    assert selected == pytest.approx(expected_yaw)


def test_camera_contact_reuses_the_approach_yaw_symmetry_branch():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    camera.observation = _camera_observation(
        orientation_xyzw=(0.0, 0.0, 1.0, 0.0),
    )
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        clock=lambda: backend.now,
    )
    task = Task('symmetric-approach-contact')
    for reuse, height in ((False, 0.2), (True, 0.1)):
        task.camera_linear_absolute(
            'right_arm',
            'tag_0',
            (1.0, 0.05, 0.2, 0.4),
            reuse_previous_observation=reuse,
            yaw_only=True,
            yaw_symmetry_deg=180.0,
            object_to_end_effector_position=(0.0, 0.0, height),
        )

    runner.start(task.build())
    runner.tick()
    assert backend.commands[0].values[5] == pytest.approx(0.0)

    runner.tick()
    backend.cartesian['right_arm'] = (0.0, 0.0, 0.0, 0.0, 0.0, 170.0)
    backend.now += 0.05
    runner.tick()

    assert len(backend.commands) == 2
    assert backend.commands[1].values[5] == pytest.approx(0.0)


def test_camera_step_times_out_without_sending_a_motion():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('camera-timeout').camera_linear_absolute(
        'right_arm',
        'tag_5',
        (1.0, 0.05, 0.2, 0.4),
        detection_timeout_sec=0.5,
    )

    runner.start(task.build())
    assert camera.marker_calls == 1
    assert backend.commands == []

    backend.now += 0.5
    runner.tick()

    assert not runner.active
    assert backend.commands == []
    assert 'camera detection timed out for \'tag_5\'' in messages[-1]
    assert camera.reason in messages[-1]


def test_camera_capture_waits_for_fresh_idle_state_and_times_out_safely():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('idle-timeout')
    task.joint_absolute(
        'right_arm',
        (1.0,) * 7,
        (1.0, 0.2, 0.2),
    )
    task.camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
    )

    runner.start(task.build())
    backend.motion_active = True
    runner.tick()
    assert camera.marker_calls == 0

    backend.now += runner.CAMERA_IDLE_TIMEOUT_SEC
    runner.tick()

    assert not runner.active
    assert camera.marker_calls == 0
    assert len(backend.commands) == 1
    assert 'fresh idle state' in messages[-1]


def test_camera_step_composes_offset_in_the_rotated_object_frame():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    camera.observation = _camera_observation(position=(1.0, 2.0, 3.0))
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        clock=lambda: backend.now,
    )
    task = Task('camera-offset').camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
        object_to_end_effector_position=(1.0, 0.0, 0.0),
        object_to_end_effector_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    runner.start(task.build())
    runner.tick()

    assert len(backend.commands) == 1
    assert backend.commands[0].values == pytest.approx(
        (1.0, 3.0, 3.0, 0.0, 0.0, 90.0)
    )


def test_camera_step_averages_distinct_frames_and_discards_outlier():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        on_status=messages.append,
        clock=lambda: backend.now,
        camera_average_enabled=True,
        camera_average_sample_count=4,
        camera_average_outlier_count=1,
    )
    task = Task('camera-average').camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
    )

    runner.start(task.build())
    for index, x in enumerate((0.9, 1.0, 1.1, 9.0), start=1):
        camera.observation = _camera_observation(
            stamp_ns=1_000_000_000 + index,
            position=(x, 2.0, 3.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        runner.tick()

    assert len(backend.commands) == 1
    assert backend.commands[0].values == pytest.approx(
        (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    )
    assert "Camera average 'tag_0': collected=4, kept=3, discarded=1" in messages


def test_camera_step_logs_target_and_actual_tcp_after_motion_succeeds():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    camera.observation = _camera_observation(
        position=(0.3, -0.2, 0.8),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('camera-log').camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
        log_target_and_actual=True,
    )

    runner.start(task.build())
    runner.tick()
    runner.tick()

    assert (
        'CAM : [0.300000, -0.200000, 0.800000, 0.000000, 0.000000, '
        '0.000000]\n'
        'REAL: [0.000000, 0.000000, 0.000000, 0.000000, 0.000000, '
        '0.000000]'
    ) in messages


def test_camera_print_step_outputs_only_clean_camera_target_without_motion():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    camera.observation = _camera_observation(position=(1.0, 2.0, 3.0))
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('camera-print').camera_linear_absolute_print(
        'left_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
        object_to_end_effector_position=(1.0, 0.0, 0.0),
        object_to_end_effector_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    runner.start(task.build())
    runner.tick()

    assert backend.commands == []
    output = next(message for message in messages if message.startswith('CAM :'))
    assert output == (
        'CAM : [1.000000, 3.000000, 3.000000, 0.000000, 0.000000, '
        '90.000000]'
    )
    assert not any(message.startswith('REAL:') for message in messages)

    runner.tick()
    assert not runner.active


@pytest.mark.parametrize(
    'observation',
    [
        (0.1, 0.2, 0.3),
        (0.1, 0.2, 0.3, 0.0, 0.0, float('nan')),
        'not-a-pose',
    ],
)
def test_camera_step_rejects_invalid_cartesian_output(observation):
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    camera.observation = observation
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('bad-camera-output').camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
    )

    runner.start(task.build())
    runner.tick()

    assert not runner.active
    assert backend.commands == []
    assert messages[-1].startswith('Task failed (bad-camera-output):')


def test_camera_step_requires_an_injected_camera_client():
    node = FakeNode()
    backend = FakeBackend()
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('missing-camera').camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
    )

    runner.start(task.build())

    assert not runner.active
    assert backend.commands == []
    assert 'requires a camera client' in messages[-1]


def test_camera_step_does_not_reuse_wait_state_after_stop():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        clock=lambda: backend.now,
    )

    first = Task('first-camera').camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
    )
    runner.start(first.build())
    assert camera.marker_calls == 1
    runner.stop()

    second = Task('second-camera').camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
    )
    runner.start(second.build())

    assert runner.active
    assert camera.marker_calls == 2


def test_consecutive_camera_steps_capture_separate_markers():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    camera.observation = _camera_observation()
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        clock=lambda: backend.now,
    )
    task = Task('two-camera-steps')
    task.camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
    )
    task.camera_linear_absolute(
        'right_arm',
        'tag_1',
        (1.0, 0.05, 0.2, 0.4),
    )

    runner.start(task.build())
    assert camera.marker_calls == 1
    runner.tick()
    assert len(backend.commands) == 1

    runner.tick()

    assert camera.marker_calls == 1
    backend.now += 0.05
    runner.tick()

    assert camera.marker_calls == 2
    assert camera.observation_calls[0][1]['newer_than_ns'] == 1_000_000_000


def test_camera_step_can_reuse_previous_observation_for_absolute_target():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    camera.observation = _camera_observation(
        position=(1.0, 2.0, 3.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        clock=lambda: backend.now,
    )
    task = Task('approach-contact')
    task.camera_linear_absolute(
        'left_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
        object_to_end_effector_position=(0.0, 0.0, 0.2),
    )
    task.camera_linear_absolute(
        'left_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
        reuse_previous_observation=True,
        object_to_end_effector_position=(0.0, 0.0, 0.1),
    )

    runner.start(task.build())
    runner.tick()
    runner.tick()
    backend.now += 0.05
    runner.tick()

    assert camera.marker_calls == 1
    assert len(camera.observation_calls) == 1
    assert [command.values for command in backend.commands] == pytest.approx([
        (1.0, 2.0, 3.2, 0.0, 0.0, 0.0),
        (1.0, 2.0, 3.1, 0.0, 0.0, 0.0),
    ])


def test_camera_lookup_exception_fails_without_backend_motion():
    node = FakeNode()
    backend = FakeBackend()
    camera = FakeCamera()
    camera.observation_error = RuntimeError('camera transport failed')
    messages = []
    runner = PlannerTaskRunner(
        node,
        backend,
        camera=camera,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    task = Task('camera-error').camera_linear_absolute(
        'right_arm',
        'tag_0',
        (1.0, 0.05, 0.2, 0.4),
    )

    runner.start(task.build())
    runner.tick()

    assert not runner.active
    assert backend.commands == []
    assert 'camera transport failed' in messages[-1]
