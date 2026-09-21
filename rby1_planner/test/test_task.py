import pytest

from rby1_planner.control_commands import CommandKind, TaskDefinition
from rby1_planner.task import (
    build_tasks,
    left_gripper_sim_test,
    move_left_ee_to_detected_object,
    object_gripping_initial_pose,
    object_gripping_initial_pose_and_print_tcp,
    object_handover_demo,
    object_handover_demo2,
    object_handover_demo4,
    object_handover_demo5,
    object_handover_demo6,
    torso_rotation_right,
)
from rby1_planner.task_commands import (
    CameraLinearAbsolutePrintStep,
    CameraLinearAbsoluteStep,
    DynamicTaskDefinition,
    MoveToStep,
    RewindStep,
    Task,
)
from rby1_planner.task_registry import load_tasks_from_source


def test_move_to_builds_a_deferred_relative_navigation_step():
    definition = Task('navigate').move_to(
        0.4,
        -0.2,
        0.5,
        timeout_sec=12.0,
    ).build()

    assert isinstance(definition, DynamicTaskDefinition)
    assert definition.commands == (
        MoveToStep(0.4, -0.2, 0.5, 12.0),
    )


def test_rewind_builds_infinite_and_finite_planner_tasks():
    infinite = Task('infinite').delay(1).rewind().build()
    finite = Task('finite').delay(1).rewind(10).build()

    assert isinstance(infinite, DynamicTaskDefinition)
    assert infinite.commands[-1] == RewindStep()
    assert finite.commands[-1] == RewindStep(repeat_count=10)


@pytest.mark.parametrize('repeat_count', [0, -1, 1.5, True, '10'])
def test_rewind_rejects_invalid_repeat_count(repeat_count):
    with pytest.raises(ValueError, match='positive integer'):
        Task('invalid').delay(1).rewind(repeat_count)


def test_rewind_must_be_the_single_final_step():
    with pytest.raises(ValueError, match='preceding'):
        Task('empty').rewind()

    task = Task('duplicate').delay(1).rewind()
    with pytest.raises(ValueError, match='only one'):
        task.rewind()

    with pytest.raises(ValueError, match='final'):
        Task('not-final').delay(1).rewind().delay(1).build()


def test_demo6_is_configured_for_infinite_rewind():
    definition = object_handover_demo6()

    assert isinstance(definition, DynamicTaskDefinition)
    assert definition.commands[-1] == RewindStep()


@pytest.mark.parametrize('values', [
    (float('nan'), 0.0, 0.0, 1.0),
    (0.0, float('inf'), 0.0, 1.0),
    (0.0, 0.0, float('nan'), 1.0),
    (0.0, 0.0, 0.0, 0.0),
])
def test_move_to_rejects_invalid_values(values):
    x, y, yaw, timeout_sec = values
    with pytest.raises(ValueError):
        Task('invalid').move_to(
            x,
            y,
            yaw,
            timeout_sec=timeout_sec,
        )


def test_initial_pose_preserves_the_measured_whole_body_target():
    definition = object_gripping_initial_pose().build()

    assert isinstance(definition, TaskDefinition)
    assert len(definition.commands) == 1
    command = definition.commands[0]
    assert command.kind is CommandKind.JOINT_ABSOLUTE_MULTI
    targets = dict(command.joint_targets)
    assert targets['torso'] == pytest.approx((
        0.059126,
        45.376237,
        -90.963053,
        45.154560,
        0.021390,
        -0.000280,
    ))
    assert targets['right_arm'] == pytest.approx((
        -34.913810,
        -63.837794,
        46.172754,
        -106.640370,
        16.002640,
        45.214678,
        83.061208,
    ))
    assert targets['left_arm'] == pytest.approx((
        -33.643824,
        63.218668,
        -44.626985,
        -106.563932,
        -17.501234,
        44.556740,
        -84.095907,
    ))


def test_torso_rotation_preserves_the_measured_target():
    definition = torso_rotation_right()

    assert isinstance(definition, TaskDefinition)
    command = definition.commands[0]
    assert command.kind is CommandKind.JOINT_ABSOLUTE
    assert command.group == 'torso'
    assert command.values == pytest.approx((
        0.065071,
        45.375357,
        -90.960025,
        45.153579,
        0.027261,
        -90.0,
    ))


def test_camera_motion_factory_builds_a_deferred_planner_step():
    definition = move_left_ee_to_detected_object(
        object_id='tag_7',
        tcp_motion=(2.0, 0.05, 0.2, 0.2),
        object_to_ee_position=(0.0, 0.0, 0.1),
        object_to_ee_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        detection_timeout_sec=4.0,
        max_age_sec=0.25,
        minimum_confidence=0.8,
    )

    assert isinstance(definition, DynamicTaskDefinition)
    assert len(definition.commands) == 2
    assert definition.commands[0].kind is CommandKind.DELAY
    assert definition.commands[0].seconds == pytest.approx(2.0)
    step = definition.commands[1]
    assert isinstance(step, CameraLinearAbsoluteStep)
    assert step.group == 'left_arm'
    assert step.object_id == 'tag_7'
    assert step.detection_timeout_sec == pytest.approx(4.0)
    assert step.max_age_sec == pytest.approx(0.25)
    assert step.minimum_confidence == pytest.approx(0.8)
    assert step.object_to_end_effector_position == (0.0, 0.0, 0.1)
    assert step.object_to_end_effector_orientation_xyzw == (
        0.0,
        0.0,
        0.0,
        1.0,
    )


def test_camera_print_factory_builds_a_non_motion_camera_step():
    definition = Task('camera-print').camera_linear_absolute_print(
        'left_arm',
        'tag_7',
        (2.0, 0.05, 0.2, 0.4),
        object_to_end_effector_position=(0.0, 0.0, 0.1),
    ).build()

    assert isinstance(definition, DynamicTaskDefinition)
    assert len(definition.commands) == 1
    step = definition.commands[0]
    assert isinstance(step, CameraLinearAbsolutePrintStep)
    assert step.group == 'left_arm'
    assert step.object_id == 'tag_7'
    assert step.object_to_end_effector_position == (0.0, 0.0, 0.1)


def test_initial_pose_then_camera_print_task_stops_after_tcp_output():
    definition = object_gripping_initial_pose_and_print_tcp()

    assert isinstance(definition, DynamicTaskDefinition)
    assert definition.name == 'object_gripping_initial_pose_and_print_tcp'
    assert len(definition.commands) == 3
    assert definition.commands[0].kind is CommandKind.JOINT_ABSOLUTE_MULTI
    assert definition.commands[1].kind is CommandKind.DELAY
    assert definition.commands[1].seconds == pytest.approx(2.0)
    assert isinstance(
        definition.commands[2],
        CameraLinearAbsolutePrintStep,
    )
    assert definition.commands[2].object_id == 'tag_0'


def test_handover_demo_places_the_dynamic_camera_step_between_motions():
    definition = object_handover_demo2()

    assert isinstance(definition, DynamicTaskDefinition)
    assert len(definition.commands) == 9
    assert definition.commands[0].kind is CommandKind.JOINT_ABSOLUTE_MULTI
    assert definition.commands[1].kind is CommandKind.DELAY
    assert isinstance(definition.commands[2], CameraLinearAbsoluteStep)
    assert definition.commands[2].group == 'left_arm'
    assert definition.commands[3].kind is CommandKind.GRIPPER_CLOSE
    assert definition.commands[3].group == 'left'
    assert definition.commands[4].kind is CommandKind.LINEAR_RELATIVE
    assert definition.commands[4].group == 'left_arm'
    assert definition.commands[4].values == (
        0.0,
        0.0,
        0.07,
        0.0,
        0.0,
        0.0,
    )
    assert definition.commands[5].kind is CommandKind.JOINT_ABSOLUTE
    assert definition.commands[5].group == 'torso'
    assert definition.commands[6].kind is CommandKind.LINEAR_RELATIVE
    assert definition.commands[6].group == 'left_arm'
    assert definition.commands[6].values == (
        0.0,
        0.0,
        -0.07,
        0.0,
        0.0,
        0.0,
    )
    assert definition.commands[7].kind is CommandKind.GRIPPER_OPEN
    assert definition.commands[7].group == 'left'
    assert definition.commands[8].kind is CommandKind.LINEAR_RELATIVE
    assert definition.commands[8].group == 'left_arm'


def test_demo4_uses_one_detection_for_two_absolute_targets_per_tag():
    definition = object_handover_demo4()

    camera_steps = [
        (index, command)
        for index, command in enumerate(definition.commands)
        if isinstance(command, CameraLinearAbsoluteStep)
    ]
    assert [command.object_id for _, command in camera_steps] == [
        'tag_4',
        'tag_4',
        'tag_0',
        'tag_0',
    ]

    for approach, contact in zip(camera_steps[::2], camera_steps[1::2]):
        approach_index, approach_command = approach
        contact_index, contact_command = contact
        assert contact_index == approach_index + 1
        assert not approach_command.reuse_previous_observation
        assert contact_command.reuse_previous_observation
        assert approach_command.yaw_only
        assert contact_command.yaw_only
        assert contact_command.group == 'left_arm'
        assert contact_command.object_to_end_effector_position[2] == pytest.approx(
            approach_command.object_to_end_effector_position[2] - 0.05
        )


def test_demo5_prints_tag4_across_y_and_yaw_offsets_then_returns():
    definition = object_handover_demo5()

    print_steps = [
        command for command in definition.commands
        if isinstance(command, CameraLinearAbsolutePrintStep)
    ]
    relative_steps = [
        command for command in definition.commands
        if getattr(command, 'kind', None) is CommandKind.LINEAR_RELATIVE
    ]

    assert len(print_steps) == 5
    assert all(step.object_id == 'tag_4' for step in print_steps)
    assert all(
        step.object_to_end_effector_position == (0.0, 0.0, 0.0)
        for step in print_steps
    )
    assert [step.values for step in relative_steps] == [
        (0.0, -0.05, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.10, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0, 0.0, 30.0),
        (0.0, 0.0, 0.0, 0.0, 0.0, -60.0),
    ]
    assert definition.commands[-1].kind is CommandKind.JOINT_ABSOLUTE
    assert definition.commands[-1].group == 'left_arm'


def test_non_camera_handover_demo_uses_only_the_left_manipulator():
    definition = object_handover_demo()

    assert isinstance(definition, TaskDefinition)
    gripper_commands = [
        command
        for command in definition.commands
        if command.kind in {
            CommandKind.GRIPPER_OPEN,
            CommandKind.GRIPPER_CLOSE,
            CommandKind.GRIPPER_SET,
        }
    ]
    assert [command.kind for command in gripper_commands] == [
        CommandKind.GRIPPER_OPEN,
        CommandKind.GRIPPER_CLOSE,
        CommandKind.GRIPPER_OPEN,
    ]
    assert all(command.group == 'left' for command in gripper_commands)
    assert all(
        command.group == 'left_arm'
        for command in definition.commands
        if command.kind in {
            CommandKind.LINEAR_ABSOLUTE,
            CommandKind.LINEAR_RELATIVE,
        }
    )


def test_left_gripper_sim_test_has_the_requested_command_sequence():
    definition = left_gripper_sim_test()

    assert isinstance(definition, TaskDefinition)
    assert [command.kind for command in definition.commands] == [
        CommandKind.GRIPPER_OPEN,
        CommandKind.DELAY,
        CommandKind.GRIPPER_SET,
        CommandKind.DELAY,
        CommandKind.GRIPPER_CLOSE,
        CommandKind.DELAY,
        CommandKind.GRIPPER_OPEN,
    ]
    assert all(
        command.group == 'both'
        for command in definition.commands
        if command.kind in {
            CommandKind.GRIPPER_OPEN,
            CommandKind.GRIPPER_CLOSE,
            CommandKind.GRIPPER_SET,
        }
    )
    assert definition.commands[2].values == (0.6,)
    assert definition.commands[2].seconds == pytest.approx(1.0)
    assert definition.commands[4].seconds == pytest.approx(1.0)
    assert [definition.commands[index].seconds for index in (1, 3, 5)] == [
        pytest.approx(0.5),
        pytest.approx(0.5),
        pytest.approx(0.5),
    ]


def test_configured_camera_move_uses_safe_mock_approach_pose():
    step = build_tasks()['move_left_ee_to_detected_object'].commands[1]

    assert isinstance(step, CameraLinearAbsoluteStep)
    assert step.group == 'left_arm'
    assert step.object_to_end_effector_position == (0.0, 0.0, 0.07)
    assert step.object_to_end_effector_orientation_xyzw == (
        0.0,
        0.0,
        0.0,
        1.0,
    )
    assert step.minimum_time == pytest.approx(5.0)
    assert step.linear_velocity == pytest.approx(0.20)
    assert step.angular_velocity == pytest.approx(0.20)
    assert step.acceleration_scaling == pytest.approx(0.30)


def test_build_tasks_exposes_registered_operator_tasks():
    tasks = build_tasks()

    assert set(tasks) == {
        'move_left_ee_to_detected_object',
        'left_gripper_sim_test',
        'object_gripping_initial_pose_and_print_tcp',
        'object_handover_demo2',
        'object_handover_demo',
        'object_handover_demo3',
        'object_handover_demo4',
        'object_handover_demo5',
        'object_handover_demo6',
        'object_gripping_initial_pose',
        'object_gripping_initial_pose_and_move',
        'move_right_10cm',
        'ready_pose',
        'torso_rotation_left',
        'torso_rotation_right',
    }
    assert all(name == task.name for name, task in tasks.items())


def test_planner_registry_reloads_dynamic_tasks_without_serializing_them():
    tasks = load_tasks_from_source()

    assert set(tasks) == {
        'move_left_ee_to_detected_object',
        'left_gripper_sim_test',
        'object_gripping_initial_pose_and_print_tcp',
        'object_handover_demo2',
        'object_handover_demo',
        'object_handover_demo3',
        'object_handover_demo4',
        'object_handover_demo5',
        'object_handover_demo6',
        'object_gripping_initial_pose',
        'object_gripping_initial_pose_and_move',
        'move_right_10cm',
        'ready_pose',
        'torso_rotation_left',
        'torso_rotation_right',
    }
    assert isinstance(
        tasks['move_left_ee_to_detected_object'],
        DynamicTaskDefinition,
    )
    assert isinstance(tasks['object_handover_demo2'], DynamicTaskDefinition)
    assert isinstance(
        tasks['object_gripping_initial_pose_and_print_tcp'],
        DynamicTaskDefinition,
    )
    assert isinstance(tasks['object_gripping_initial_pose'], TaskDefinition)


@pytest.mark.parametrize(
    'kwargs,match',
    [
        ({'object_id': ''}, 'object_id'),
        ({'detection_timeout_sec': 0.0}, 'positive finite'),
        ({'max_age_sec': -1.0}, 'positive finite'),
        ({'minimum_confidence': 1.1}, 'range'),
        (
            {'object_to_end_effector_position': (0.0, 0.0)},
            '3 finite numbers',
        ),
        (
            {
                'object_to_end_effector_orientation_xyzw': (
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            },
            'zero quaternion',
        ),
    ],
)
def test_camera_step_rejects_invalid_perception_settings(kwargs, match):
    values = {
        'arm': 'right_arm',
        'object_id': 'tag_0',
        'tcp_motion': (1.0, 0.1, 0.2, 0.5),
    }
    values.update(kwargs)

    with pytest.raises(ValueError, match=match):
        Task('invalid').camera_linear_absolute(**values)


def test_camera_step_accepts_rectangular_yaw_symmetry():
    definition = Task('symmetric').camera_linear_absolute(
        'left_arm',
        'tag_0',
        (1.0, 0.1, 0.2, 0.5),
        yaw_only=True,
        yaw_symmetry_deg=180.0,
        object_to_end_effector_position=(0.0, 0.0, 0.1),
    ).build()

    step = definition.commands[0]
    assert isinstance(step, CameraLinearAbsoluteStep)
    assert step.yaw_symmetry_deg == pytest.approx(180.0)


def test_camera_yaw_symmetry_rejects_unsafe_configuration():
    base = {
        'arm': 'left_arm',
        'object_id': 'tag_0',
        'tcp_motion': (1.0, 0.1, 0.2, 0.5),
        'yaw_only': True,
        'yaw_symmetry_deg': 180.0,
    }

    with pytest.raises(ValueError, match='yaw_only'):
        Task('invalid').camera_linear_absolute(
            **{**base, 'yaw_only': False},
        )
    with pytest.raises(ValueError, match='divide 360'):
        Task('invalid').camera_linear_absolute(
            **{**base, 'yaw_symmetry_deg': 200.0},
        )
    with pytest.raises(ValueError, match='zero X/Y'):
        Task('invalid').camera_linear_absolute(
            **base,
            object_to_end_effector_position=(0.01, 0.0, 0.1),
        )
