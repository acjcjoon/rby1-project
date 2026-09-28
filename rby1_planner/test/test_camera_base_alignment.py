"""Camera-centred base alignment, without ROS or robot commands."""
import math

import pytest

from rby1_planner.control_commands import CommandKind
from rby1_planner.navigation_protocol import (
    NavigationCommandState,
    NavigationCommandStatus,
)
from rby1_planner.observation import rpy_deg_to_quaternion
from rby1_planner.task import (
    _configured_tag_approach_contact_move,
    object_handover_demo7,
)
from rby1_planner.task_commands import CameraMoveToTagStep, Task
from rby1_planner.task_runner import PlannerTaskRunner
from test_task_runner import (
    FakeBackend,
    FakeCamera,
    FakeNavigation,
    FakeNode,
    _camera_observation,
)


class FrameCamera(FakeCamera):
    def __init__(self):
        super().__init__()
        self.position = (0.2, -0.4, 0.95)
        self.orientation = rpy_deg_to_quaternion((30.0, 20.0, -90.0))
        self.lookups = []
        self.lookup_error = None

    def lookup_frame_pose(self, target, source):
        self.lookups.append((target, source))
        if self.lookup_error is not None:
            raise self.lookup_error
        return self.position, self.orientation


def alignment_task(**overrides):
    options = dict(
        desired_tag_frame='d405_camera_center',
        desired_tag_x=0.0,
        desired_tag_y=0.0,
        threshold_x_minus=0.05,
        threshold_y_plus=0.03,
    )
    options.update(overrides)
    return Task('alignment').move_base_to_detected_tag(
        'd435', 'tag_4', **options,
    )


def start_alignment(error, *, task=None, camera=None, navigation=True):
    backend = FakeBackend()
    # Deliberately different from the camera centre: using ee_left must fail
    # these expected navigation targets even though both belong to the arm.
    backend.cartesian['left_arm'] = (0.3, -0.3, 0.92, 0.0, 0.0, -180.0)
    camera = camera or FrameCamera()
    camera.observation = _camera_observation(
        object_id='tag_4',
        position=(camera.position[0] + error[0],
                  camera.position[1] + error[1], 0.7),
    )
    wrist_camera = FakeCamera()
    wrist_camera.observation = _camera_observation(
        object_id='tag_4',
        position=(0.21, -0.37, 0.71),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    nav = FakeNavigation() if navigation else None
    messages = []
    runner = PlannerTaskRunner(
        FakeNode(), backend,
        cameras={'d435': camera, 'd405': wrist_camera},
        navigation=nav,
        on_status=messages.append,
        clock=lambda: backend.now,
    )
    runner.start((task or alignment_task()).build())
    runner.tick()
    return runner, backend, camera, wrist_camera, nav, messages


@pytest.mark.parametrize('error, expected_move', [
    ((0.0, 0.0), None),
    ((-0.04, 0.02), None),
    ((-0.05, 0.03), None),  # Inclusive bounds, including float roundoff.
    ((-0.08, 0.02), (-0.03, 0.0)),
    ((-0.02, 0.08), (0.0, 0.05)),
    ((-0.08, 0.09), (-0.03, 0.06)),
    ((0.02, 0.02), (0.02, 0.0)),  # +X has no tolerance.
    ((-0.02, -0.01), (0.0, -0.01)),  # -Y has no tolerance.
    ((0.02, -0.01), (0.02, -0.01)),
])
def test_camera_alignment_uses_base_axes_and_minimum_translation(
    error, expected_move,
):
    runner, backend, camera, _, nav, messages = start_alignment(error)
    assert camera.lookups == [('base', 'd405_camera_center')]
    assert camera.observation_calls[0][1]['target_frame'] == 'base'
    assert backend.commands == []
    if expected_move is None:
        assert nav.commands == []
        assert True not in backend.stream_requests
        runner.tick()
        assert not runner.active
        assert any('skipping base navigation' in message for message in messages)
    else:
        assert nav.commands == []  # Wait for Stream ON acknowledgement.
        assert backend.stream_requests == [True]
        backend.stream_enabled = True
        runner.tick()
        assert nav.commands == [pytest.approx((*expected_move, 0.0, 30.0))]
        runner.tick()
        assert len(nav.commands) == 1
        assert len(camera.lookups) == 1
        assert camera.marker_calls == 1


def test_zero_thresholds_keep_exact_alignment():
    runner, backend, _, _, nav, _ = start_alignment(
        (-0.04, 0.02),
        task=alignment_task(threshold_x_minus=0.0, threshold_y_plus=0.0),
    )
    backend.stream_enabled = True
    runner.tick()
    assert nav.commands == [pytest.approx((-0.04, 0.02, 0.0, 30.0))]


def test_existing_base_goal_and_nonzero_yaw_are_preserved():
    yaw = 0.2
    runner, backend, _, _, nav, _ = start_alignment(
        (0.3, 0.1),
        task=alignment_task(
            desired_tag_frame='base', desired_tag_x=0.4, desired_tag_y=-0.2,
            threshold_x_minus=0.0, threshold_y_plus=0.0, relative_yaw=yaw,
        ),
    )
    backend.stream_enabled = True
    runner.tick()
    expected_x = 0.5 - (math.cos(yaw) * 0.4 + math.sin(yaw) * 0.2)
    expected_y = -0.3 - (math.sin(yaw) * 0.4 - math.cos(yaw) * 0.2)
    assert nav.commands == [pytest.approx((expected_x, expected_y, yaw, 30.0))]


def test_camera_local_offset_is_transformed_before_base_thresholds():
    camera = FrameCamera()
    camera.orientation = rpy_deg_to_quaternion((0.0, 0.0, -90.0))
    runner, backend, _, _, nav, _ = start_alignment(
        (0.02, -0.08), camera=camera,
        task=alignment_task(desired_tag_x=0.1),
    )
    # Local +X 10cm is base -Y 10cm. Residual +Y 2cm is allowed;
    # residual +X 2cm still needs correction even with a rotated camera.
    backend.stream_enabled = True
    runner.tick()
    assert nav.commands == [pytest.approx((0.02, 0.0, 0.0, 30.0))]


@pytest.mark.parametrize('field', ['threshold_x_minus', 'threshold_y_plus'])
@pytest.mark.parametrize('value', [-0.01, float('nan'), float('inf'), True])
def test_invalid_thresholds_are_rejected_before_execution(field, value):
    with pytest.raises(ValueError, match=field):
        alignment_task(**{field: value})


def test_in_bounds_does_not_require_a_navigation_client():
    runner, backend, _, _, _, messages = start_alignment(
        (-0.02, 0.01), navigation=False,
    )
    runner.tick()
    assert not runner.active
    assert backend.stream_requests == []
    assert messages[-1] == 'Task completed: alignment'


def test_required_move_without_navigation_fails_without_enabling_stream():
    runner, backend, _, _, _, messages = start_alignment(
        (-0.2, 0.0), navigation=False,
    )
    assert not runner.active
    assert True not in backend.stream_requests
    assert 'navigation client is unavailable' in messages[-1]


def test_missing_camera_mount_tf_fails_without_navigation():
    camera = FrameCamera()
    camera.lookup_error = RuntimeError('camera mount TF unavailable')
    runner, backend, _, _, nav, messages = start_alignment(
        (-0.04, 0.02), camera=camera,
    )
    assert not runner.active
    assert nav.commands == []
    assert True not in backend.stream_requests
    assert 'camera mount TF unavailable' in messages[-1]


def test_distance_limit_applies_to_actual_correction():
    runner, backend, _, _, nav, _ = start_alignment((-0.82, 0.0))
    backend.stream_enabled = True
    runner.tick()
    assert nav.commands == [pytest.approx((-0.77, 0.0, 0.0, 30.0))]

    runner, backend, _, _, nav, messages = start_alignment((-0.9, 0.0))
    assert not runner.active
    assert nav.commands == []
    assert True not in backend.stream_requests
    assert 'exceeds safety limit' in messages[-1]


@pytest.mark.parametrize('error', [(-0.04, 0.02), (-0.08, 0.09)])
def test_alignment_continues_to_fresh_d405_approach_contact_and_lift(error):
    task = alignment_task().open_gripper('left')
    task.extend(_configured_tag_approach_contact_move(
        'tag_4', (0.0, 0.0, 0.15), (0.0, 0.0, 0.05),
        (3.0, 1.0, 1.0, 1.0),
    ))
    task.close_gripper('left')
    task.linear_relative(
        'left_arm', (0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        (2.0, 2.0, 2.0, 1.0),
    )
    runner, backend, head, wrist, nav, messages = start_alignment(error, task=task)
    for _ in range(40):
        if not runner.active:
            break
        if backend.stream_requests:
            backend.stream_enabled = backend.stream_requests[-1]
        for command_id in nav.states:
            nav.states[command_id] = NavigationCommandState(
                NavigationCommandStatus.SUCCEEDED,
            )
        if backend.commands:
            command = backend.commands[-1]
            if command.kind is CommandKind.LINEAR_ABSOLUTE:
                backend.cartesian['left_arm'] = command.values
        backend.now += 0.5
        runner.tick()
    assert not runner.active, messages
    assert messages[-1] == 'Task completed: alignment'
    assert head.marker_calls == wrist.marker_calls == 1
    assert len(wrist.observation_calls) == 1  # Contact reuses the approach sample.
    assert [command.kind for command in backend.commands] == [
        CommandKind.GRIPPER_OPEN,
        CommandKind.LINEAR_ABSOLUTE,
        CommandKind.LINEAR_ABSOLUTE,
        CommandKind.GRIPPER_CLOSE,
        CommandKind.LINEAR_ABSOLUTE,
    ]
    assert backend.commands[1].values[:3] == pytest.approx((0.21, -0.37, 0.86))
    assert backend.commands[2].values[:3] == pytest.approx((0.21, -0.37, 0.76))
    assert backend.commands[4].values[:3] == pytest.approx((0.21, -0.37, 0.86))


def test_demo7_wires_camera_centre_and_threshold_arguments():
    definition = object_handover_demo7(
        threshold_x_minus=0.05, threshold_y_plus=0.03,
    )
    base_step = next(
        command for command in definition.commands
        if isinstance(command, CameraMoveToTagStep)
    )
    assert base_step.camera_source == 'd435'
    assert base_step.desired_tag_frame == 'd405_camera_center'
    assert (base_step.desired_tag_x, base_step.desired_tag_y) == (0.0, 0.0)
    assert (base_step.threshold_x_minus, base_step.threshold_y_plus) == (0.05, 0.03)
