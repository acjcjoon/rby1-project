"""Tag-axis centre corrections shared by approach and contact; no robot IO."""
from dataclasses import replace
import math

import pytest
import yaml

from rby1_planner import task as task_source
from rby1_planner import task_commands
from rby1_planner.observation import rpy_deg_to_quaternion
from rby1_planner.task_commands import CameraLinearAbsoluteStep, Task
from rby1_planner.task_runner import PlannerTaskRunner
from test_task_runner import FakeBackend, FakeCamera, FakeNode, _camera_observation


OFFSETS = {
    'tag_0': [0.0, 0.0, 0.0],
    'tag_1': [0.01, 0.02, -0.003],
    'tag_2': [0.0, 0.0, 0.0],
    'tag_3': [0.05, -0.06250, -0.009],
    'tag_4': [0.0, 0.0, 0.0],
}


@pytest.fixture
def settings_file(tmp_path, monkeypatch):
    (tmp_path / 'package.xml').write_text('<package/>', encoding='utf-8')
    (tmp_path / 'config').mkdir()
    path = tmp_path / 'config' / 'settings.yaml'
    path.write_text(yaml.safe_dump({
        'tag': {name: {'offset': offset} for name, offset in OFFSETS.items()},
    }), encoding='utf-8')
    monkeypatch.setattr(
        task_commands, '__file__', str(tmp_path / 'rby1_planner' / 'task_commands.py'),
    )
    return path


def camera_steps(definition):
    return [step for step in definition.commands if isinstance(step, CameraLinearAbsoluteStep)]


@pytest.mark.parametrize('object_id', OFFSETS)
@pytest.mark.parametrize('operation, contact_z', [
    ('align_base_and_pick_up_object', 0.05),
    ('align_base_and_put_down_object', 0.065),
])
def test_both_camera_moves_get_same_tag_correction(
    settings_file, monkeypatch, object_id, operation, contact_z,
):
    builder = getattr(task_source, operation)
    definition = builder(object_id)
    approach, contact = camera_steps(definition)
    dx, dy, dz = OFFSETS[object_id]
    assert approach.object_to_end_effector_position == pytest.approx(
        (dx, dy, contact_z + 0.075 + dz),
    )
    assert contact.object_to_end_effector_position == pytest.approx((dx, dy, contact_z + dz))
    assert approach.object_id == contact.object_id == object_id
    assert approach.camera_source == contact.camera_source == 'd405'
    assert not approach.reuse_previous_observation
    assert contact.reuse_previous_observation
    assert approach.yaw_symmetry_deg == contact.yaw_symmetry_deg == 180.0

    # All other fields and commands (base, gripper, lift, return pose)
    # remain identical to building the same task with no correction.
    monkeypatch.setattr(task_source, '_load_tag_approach_offset', lambda _: (0.0, 0.0, 0.0))
    baseline = builder(object_id)
    expected = tuple(
        replace(step, object_to_end_effector_position=tuple(
            value + offset
            for value, offset in zip(step.object_to_end_effector_position, OFFSETS[object_id])
        ))
        if isinstance(step, CameraLinearAbsoluteStep) else step
        for step in baseline.commands
    )
    assert definition.commands == expected


def test_custom_approach_is_added_once_without_mutating_arguments(settings_file):
    approach_offset = [0.01, -0.02, 0.2]
    contact_offset = [0.0, 0.0, 0.065]
    args = ('tag_3', approach_offset, contact_offset, (3.0, 1.0, 1.0, 1.0))
    first = task_source._configured_tag_approach_contact_move(*args)
    second = task_source._configured_tag_approach_contact_move(*args)
    approach, contact = camera_steps(first)
    assert approach.object_to_end_effector_position == pytest.approx((0.06, -0.0825, 0.191))
    assert contact.object_to_end_effector_position == pytest.approx((0.05, -0.0625, 0.056))
    assert first == second
    assert approach_offset == [0.01, -0.02, 0.2]
    assert contact_offset == [0.0, 0.0, 0.065]


def test_yaml_is_read_again_when_task_is_rebuilt(settings_file):
    first = task_source.align_base_and_put_down_object('tag_3')
    settings_file.write_text(yaml.safe_dump({
        'tag': {'tag_3': {'offset': [0.01, 0.02, 0.03]}},
    }), encoding='utf-8')
    second = task_source.align_base_and_put_down_object('tag_3')
    assert camera_steps(first)[0].object_to_end_effector_position == pytest.approx((0.05, -0.0625, 0.131))
    assert camera_steps(second)[0].object_to_end_effector_position == pytest.approx((0.01, 0.02, 0.17))
    assert camera_steps(first)[1].object_to_end_effector_position == pytest.approx((0.05, -0.0625, 0.056))
    assert camera_steps(second)[1].object_to_end_effector_position == pytest.approx((0.01, 0.02, 0.095))


@pytest.mark.parametrize('text', [
    '', '[]', 'tag: [', 'tag: []', 'tag: {}',
    'locations: {}', 'tag: {tag_0: {offset: [0, 0, 0]}}',
    'tag: {tag_3: null}', 'tag: {tag_3: {}}',
    'tag: {tag_3: {offset: [0, 0]}}',
    'tag: {tag_3: {offset: [0, 0, 0, 0]}}',
    'tag: {tag_3: {offset: "0,0,0"}}',
    'tag: {tag_3: {offset: [true, 0, 0]}}',
    'tag: {tag_3: {offset: [0, .nan, 0]}}',
    'tag: {tag_3: {offset: [0, 0, .inf]}}',
    'tag: {tag_3: {offset: [0, 0, "0.1"]}}',
])
def test_bad_or_missing_tag_config_fails_before_motion(settings_file, text):
    settings_file.write_text(text, encoding='utf-8')
    with pytest.raises(ValueError):
        task_source.align_base_and_put_down_object('tag_3')


def test_missing_settings_file_is_not_silently_ignored(settings_file):
    settings_file.unlink()
    with pytest.raises(ValueError, match='Cannot load planner settings'):
        task_source.align_base_and_put_down_object('tag_3')


@pytest.mark.parametrize('object_id', ['', ' ', None])
def test_invalid_tag_id_is_rejected(settings_file, object_id):
    with pytest.raises(ValueError, match='object_id'):
        task_source._load_tag_approach_offset(object_id)


@pytest.mark.parametrize('rpy, expected_xy, expected_yaw', [
    ((0.0, 0.0, 0.0), (1.05, 1.9375), 0.0),
    ((20.0, -15.0, 180.0), (0.95, 2.0625), 0.0),
    ((0.0, 0.0, 90.0), (1.0625, 2.05), -90.0),
    ((0.0, 0.0, -90.0), (0.9375, 1.95), -90.0),
])
def test_runner_keeps_centre_xy_when_selecting_and_reusing_ee_yaw(
    settings_file, rpy, expected_xy, expected_yaw,
):
    backend = FakeBackend()
    backend.cartesian['left_arm'] = (0.0,) * 6
    camera = FakeCamera()
    camera.observation = _camera_observation(
        object_id='tag_3', position=(1.0, 2.0, 0.8),
        orientation_xyzw=rpy_deg_to_quaternion(rpy),
    )
    raw_observation = camera.observation
    messages = []
    runner = PlannerTaskRunner(
        FakeNode(), backend, camera=camera,
        on_status=messages.append, clock=lambda: backend.now,
    )
    definition = task_source._configured_tag_approach_contact_move(
        'tag_3', (0.0, 0.0, 0.14), (0.0, 0.0, 0.065),
        (3.0, 1.0, 1.0, 1.0),
    )
    runner.start(definition)
    for _ in range(40):
        if not runner.active:
            break
        if backend.commands:
            backend.cartesian['left_arm'] = backend.commands[-1].values
            # Contact must use the captured pose without accumulating offsets
            # or re-detecting after the arm has moved.
            camera.observation = _camera_observation(
                object_id='tag_3', position=(9.0, 9.0, 9.0),
            )
        backend.now += 0.5
        runner.tick()
    assert not runner.active, messages
    assert messages[-1] == f'Task completed: {definition.name}'
    assert len(backend.commands) == 2
    assert backend.commands[0].values == pytest.approx((*expected_xy, 0.931, 0.0, 0.0, expected_yaw))
    assert backend.commands[1].values == pytest.approx((*expected_xy, 0.856, 0.0, 0.0, expected_yaw))
    assert tuple(c - a for c, a in zip(backend.commands[1].values[:3], backend.commands[0].values[:3])) == pytest.approx((0.0, 0.0, -0.075))
    assert raw_observation.position == (1.0, 2.0, 0.8)
    assert camera.marker_calls == len(camera.observation_calls) == 1


@pytest.mark.parametrize('options, message', [
    ({'yaw_only': False}, 'yaw_only'),
    ({'yaw_symmetry_deg': 200.0}, 'divide 360'),
])
def test_other_yaw_symmetry_constraints_are_preserved(options, message):
    kwargs = dict(yaw_only=True, yaw_symmetry_deg=180.0)
    kwargs.update(options)
    with pytest.raises(ValueError, match=message):
        Task('invalid').camera_linear_absolute(
            'left_arm', 'tag_3', (3.0, 1.0, 1.0, 1.0),
            object_to_end_effector_position=(0.05, -0.0625, 0.131),
            **kwargs,
        )


@pytest.mark.parametrize('base_yaw_deg', [0.0, 45.0, 90.0, 180.0, -90.0])
def test_same_physical_centre_when_base_heading_changes(settings_file, base_yaw_deg):
    # A fixed tag and fixed centre in the world, viewed from a rotated base.
    tag_world = (0.4, -0.2, 0.7)
    tag_yaw_world = 30.0
    base_world = (0.2, -0.1)

    def rotate_xy(x, y, degrees):
        angle = math.radians(degrees)
        return (math.cos(angle) * x - math.sin(angle) * y,
                math.sin(angle) * x + math.cos(angle) * y)

    tag_base_xy = rotate_xy(
        tag_world[0] - base_world[0], tag_world[1] - base_world[1],
        -base_yaw_deg,
    )
    observation = _camera_observation(
        object_id='tag_3', position=(*tag_base_xy, tag_world[2]),
        orientation_xyzw=rpy_deg_to_quaternion((0.0, 0.0, tag_yaw_world - base_yaw_deg)),
    )
    offset_world_xy = rotate_xy(*OFFSETS['tag_3'][:2], tag_yaw_world)
    centre_world = tuple(t + d for t, d in zip(tag_world, (*offset_world_xy, OFFSETS['tag_3'][2])))
    approach, contact = camera_steps(task_source.align_base_and_put_down_object('tag_3'))
    for command, height in ((approach, 0.14), (contact, 0.065)):
        goal_base = command.resolve_observation(observation).values[:3]
        goal_world_xy = rotate_xy(*goal_base[:2], base_yaw_deg)
        goal_world = (goal_world_xy[0] + base_world[0],
                      goal_world_xy[1] + base_world[1], goal_base[2])
        assert goal_world == pytest.approx((centre_world[0], centre_world[1], centre_world[2] + height))
