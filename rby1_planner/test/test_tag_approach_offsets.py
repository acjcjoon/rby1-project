"""Tag-axis centre corrections shared by approach and contact; no robot IO."""
from dataclasses import replace
import math

import pytest
import yaml

from rby1_planner import task as task_source
from rby1_planner import task_commands
from rby1_planner.observation import rpy_deg_to_quaternion
from rby1_planner.task_commands import (
    CameraLinearAbsoluteStep,
    CameraMoveToTagStep,
    Task,
)
from rby1_planner.task_runner import PlannerTaskRunner
from station_settings import stations_settings
from test_task_runner import FakeBackend, FakeCamera, FakeNode, _camera_observation


OFFSETS = {
    'tag_0': [0.0, 0.0, 0.0],
    'tag_1': [0.01, 0.02, -0.003],
    'tag_2': [0.0, 0.0, 0.0],
    'tag_3': [0.05, -0.06250, -0.009],
}
LOCATION_TAGS = {
    'A': 'tag_2',
    'B': 'tag_1',
    'C': 'tag_3',
    'D': 'tag_0',
}
LOCATION_FIELDS = {
    'A': {'position': [0.0, 0.0], 'side': 'right'},
    'B': {'position': [-0.5, 0.0], 'side': 'right'},
    'C': {'position': [0.0, 0.81], 'side': 'left'},
    'D': {'position': [0.6, 0.81], 'side': 'left'},
}


def nested_settings(location_tags=LOCATION_TAGS, offsets=OFFSETS):
    locations = {}
    for location_name, tag_id in location_tags.items():
        locations[location_name] = {
            **LOCATION_FIELDS[location_name],
            tag_id: {'offset': offsets[tag_id]},
        }
    return stations_settings(locations)


@pytest.fixture
def settings_file(tmp_path, monkeypatch):
    (tmp_path / 'package.xml').write_text('<package/>', encoding='utf-8')
    (tmp_path / 'config').mkdir()
    path = tmp_path / 'config' / 'settings.yaml'
    path.write_text(yaml.safe_dump(nested_settings()), encoding='utf-8')
    monkeypatch.setattr(
        task_commands, '__file__', str(tmp_path / 'rby1_planner' / 'task_commands.py'),
    )
    return path


def _aligned(in_place_name):
    """_pick_or_place 와 같은 순서: base 정렬 -> 손목 카메라 파지/배치 -> 초기 자세."""
    def build(object_id, *, arm):
        task = Task(f'aligned_{in_place_name}')
        task.extend(task_source.align_base_to_tag(object_id, arm=arm))
        task.extend(getattr(task_source, in_place_name)(object_id, arm=arm))
        task.extend(task_source.object_gripping_initial_pose())
        return task.build()
    return build


align_and_pick_up = _aligned('pick_up_object_in_place')
align_and_put_down = _aligned('put_down_object_in_place')


def camera_steps(definition):
    return [step for step in definition.commands if isinstance(step, CameraLinearAbsoluteStep)]


@pytest.mark.parametrize('object_id', OFFSETS)
@pytest.mark.parametrize('operation', [
    align_and_pick_up,
    align_and_put_down,
])
def test_both_camera_moves_get_same_tag_correction(
    settings_file, monkeypatch, object_id, operation,
):
    builder = operation
    definition = builder(object_id, arm='right_arm')
    approach, contact = camera_steps(definition)
    dx, dy, dz = OFFSETS[object_id]
    assert approach.object_to_end_effector_position == pytest.approx(
        (dx, dy, 0.065 + 0.075 + dz),
    )
    assert contact.object_to_end_effector_position == pytest.approx((dx, dy, 0.065 + dz))
    assert approach.object_id == contact.object_id == object_id
    assert approach.camera_source == contact.camera_source == 'd405_right'
    assert not approach.reuse_previous_observation
    assert contact.reuse_previous_observation
    assert approach.yaw_symmetry_deg == contact.yaw_symmetry_deg == 180.0

    # All other fields and commands (base, gripper, lift, return pose)
    # remain identical to building the same task with no correction.
    monkeypatch.setattr(task_source, '_load_tag_approach_offset', lambda _: (0.0, 0.0, 0.0))
    baseline = builder(object_id, arm='right_arm')
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
    first = task_source.move_hand_to_tag(*args, arm='right_arm')
    second = task_source.move_hand_to_tag(*args, arm='right_arm')
    approach, contact = camera_steps(first)
    assert approach.object_to_end_effector_position == pytest.approx((0.06, -0.0825, 0.191))
    assert contact.object_to_end_effector_position == pytest.approx((0.05, -0.0625, 0.056))
    assert first == second
    assert approach_offset == [0.01, -0.02, 0.2]
    assert contact_offset == [0.0, 0.0, 0.065]


def test_yaml_is_read_again_when_task_is_rebuilt(settings_file):
    first = align_and_put_down('tag_3', arm='right_arm')
    settings_file.write_text(yaml.safe_dump(nested_settings(
        {'C': 'tag_3'},
        {'tag_3': [0.01, 0.02, 0.03]},
    )), encoding='utf-8')
    second = align_and_put_down('tag_3', arm='right_arm')
    assert camera_steps(first)[0].object_to_end_effector_position == pytest.approx((0.05, -0.0625, 0.131))
    assert camera_steps(second)[0].object_to_end_effector_position == pytest.approx((0.01, 0.02, 0.17))
    assert camera_steps(first)[1].object_to_end_effector_position == pytest.approx((0.05, -0.0625, 0.056))
    assert camera_steps(second)[1].object_to_end_effector_position == pytest.approx((0.01, 0.02, 0.095))


@pytest.mark.parametrize('text', [
    '',
    '[]',
    'stations: [',
    'stations: []',
    'stations: {}',
    'locations: {C: {position: [0, 0], side: left, tag_3: {offset: [0, 0, 0]}}}',
    'stations: {C: null}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: [0, 0]}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: [0, 0, 0, 0]}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: "0,0,0"}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: [true, 0, 0]}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: [0, .nan, 0]}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: [0, 0, .inf]}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: [0, 0, "0.1"]}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_name, offset: [0, 0, 0]}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: [0, 0, 0]}}, D: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: [0, 0, 0]}}}',
    'stations: {C: {kind: shelf, points: {place: {position: [0, 0], side: left}}, place: {point: place, tag: tag_3, offset: [0, 0, 0]}}}',
    'stations: {C: {kind: plain, points: {place: {position: [0, 0], side: left}}, place: {point: other, tag: tag_3, offset: [0, 0, 0]}}}',
])
def test_bad_or_missing_tag_config_fails_before_motion(settings_file, text):
    settings_file.write_text(text, encoding='utf-8')
    with pytest.raises(ValueError):
        align_and_put_down('tag_3', arm='right_arm')


def test_missing_settings_file_is_not_silently_ignored(settings_file):
    settings_file.unlink()
    with pytest.raises(ValueError, match='Cannot load planner settings'):
        align_and_put_down('tag_3', arm='right_arm')


@pytest.mark.parametrize('object_id', ['', ' ', None])
def test_invalid_tag_id_is_rejected(settings_file, object_id):
    with pytest.raises(ValueError, match='object_id'):
        task_source._load_tag_approach_offset(object_id)


def test_location_to_tag_mapping_drives_the_handover(settings_file):
    location_tags = {
        'LH1': 'tag_3',
        'LH2': 'tag_0',
    }
    locations = {
        'LH1': {'position': [0.0, 0.81], 'side': 'left'},
        'LH2': {'position': [0.6, 0.81], 'side': 'left'},
    }
    for name, tag_id in location_tags.items():
        locations[name][tag_id] = {'offset': [0.0, 0.0, 0.0]}
    settings_file.write_text(
        yaml.safe_dump(stations_settings(locations)), encoding='utf-8',
    )

    definition = task_source.transfer('LH1', 'LH2')
    alignment_steps = [
        step for step in definition.commands
        if isinstance(step, CameraMoveToTagStep)
    ]
    location_order = ('LH1', 'LH2')
    assert [step.object_id for step in alignment_steps] == [
        location_tags[name] for name in location_order
    ]


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
    definition = task_source.move_hand_to_tag(
        'tag_3', (0.0, 0.0, 0.14), (0.0, 0.0, 0.065),
        (3.0, 1.0, 1.0, 1.0), arm='right_arm',
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
    approach, contact = camera_steps(align_and_put_down('tag_3', arm='right_arm'))
    for command, height in ((approach, 0.14), (contact, 0.065)):
        goal_base = command.resolve_observation(observation).values[:3]
        goal_world_xy = rotate_xy(*goal_base[:2], base_yaw_deg)
        goal_world = (goal_world_xy[0] + base_world[0],
                      goal_world_xy[1] + base_world[1], goal_base[2])
        assert goal_world == pytest.approx((centre_world[0], centre_world[1], centre_world[2] + height))


@pytest.mark.parametrize('operation', [
    align_and_pick_up,
    align_and_put_down,
])
@pytest.mark.parametrize('arm, gripper, source, frame', [
    ('right_arm', 'right', 'd405_right', 'd405_right_camera_center'),
    ('left_arm', 'left', 'd405_left', 'd405_left_camera_center'),
])
def test_pick_and_place_derive_gripper_and_camera_from_arm(
    settings_file, operation, arm, gripper, source, frame,
):
    definition = operation('tag_1', arm=arm)
    base_step = next(
        step for step in definition.commands
        if isinstance(step, CameraMoveToTagStep)
    )
    approach, contact = camera_steps(definition)
    assert base_step.camera_source == 'd435'
    assert base_step.desired_tag_frame == frame
    assert approach.group == contact.group == arm
    assert approach.camera_source == contact.camera_source == source
    gripper_sides = {
        step.group
        for step in definition.commands
        if 'GRIPPER' in str(getattr(step, 'kind', ''))
    }
    assert gripper_sides == {gripper}


@pytest.mark.parametrize('call', [
    lambda: task_source.align_base_to_tag('tag_1'),
    lambda: task_source.pick_up_object_in_place('tag_1'),
    lambda: task_source.put_down_object_in_place('tag_1'),
])
def test_pick_and_place_require_explicit_arm(settings_file, call):
    with pytest.raises(TypeError, match="arm"):
        call()


def test_pick_and_place_reject_non_arm_group(settings_file):
    for call in (
        task_source.align_base_to_tag,
        task_source.pick_up_object_in_place,
        task_source.put_down_object_in_place,
    ):
        with pytest.raises(ValueError):
            call('tag_1', arm='torso')


def test_camera_source_explicit_and_unknown_arm():
    task = Task('explicit').camera_linear_absolute(
        'left_arm', 'tag_0', (1.0, 0.05, 0.2, 0.4), camera_source='d435',
    )
    assert task.task_list[0].camera_source == 'd435'
    with pytest.raises(ValueError, match='no wrist camera'):
        task_commands.wrist_camera_source('torso')


def test_test1_moves_only_the_left_arm_and_repeats(settings_file):
    from rby1_planner.task_commands import MoveToStep, RewindStep

    definition = task_source.test1()
    commands = definition.commands
    assert not any(isinstance(step, (CameraMoveToTagStep, MoveToStep)) for step in commands)
    assert isinstance(commands[-1], RewindStep) and commands[-1].repeat_count == 10
    # 첫 스텝(허리 오른쪽 준비 자세) 이후로는 왼팔/왼 그리퍼만 움직인다.
    moving_groups = {
        step.group for step in commands[1:]
        if getattr(step, 'group', None) is not None
    }
    assert moving_groups == {'left_arm', 'left'}
    wrist_steps = camera_steps(definition)
    assert [step.object_id for step in wrist_steps[::2]] == ['tag_3', 'tag_0', 'tag_0', 'tag_3']
    assert {step.camera_source for step in wrist_steps} == {'d405_left'}
    shifts = [
        step.values[:3] for step in commands
        if str(getattr(step, 'kind', '')).endswith('LINEAR_RELATIVE')
        and step.values[0] != 0.0
    ]
    assert shifts == [(-0.25, 0.0, 0.0), (0.15, 0.0, 0.0)]
