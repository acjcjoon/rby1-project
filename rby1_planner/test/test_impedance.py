"""Impedance authoring, contact helpers, exit rule, wire format, runner."""
import pytest

from rby1_planner.control_commands import (
    CommandKind,
    JointImpedance,
    Task as CanonicalTask,
    TaskCommand,
    check_impedance_exit,
    impedance_tag,
    impedance_tag_for,
    load_catalog,
    save_catalog,
)
from rby1_planner.control_protocol import (
    task_backend_state_from_dict,
    task_backend_state_to_dict,
    task_command_from_dict,
    task_command_to_dict,
)
from rby1_planner.task_commands import (
    CameraLinearAbsoluteStep,
    DynamicTaskDefinition,
    Task,
)
from rby1_planner.task_runner import PlannerTaskRunner

from test_task_runner import FakeBackend, FakeNode


JOINT_MOTION = (3.0, 0.5, 0.5)
TCP_MOTION = (2.0, 0.05, 0.2, 0.3)
JS = JointImpedance([100.0] * 7, torque_limit=10.0)
CONTACT = JointImpedance([80.0] * 7, torque_limit=5.0, hold_time=0.5)


def _cartesian(**overrides):
    base = dict(
        kind=CommandKind.LINEAR_ABSOLUTE,
        group='right_arm',
        values=(0.0,) * 6,
        minimum_time=2.0,
        linear_velocity=0.05,
        angular_velocity=0.2,
        acceleration_scaling=0.3,
    )
    base.update(overrides)
    return TaskCommand(**base)


# --- joint authoring --------------------------------------------------------

def test_joint_builder_binds_flat_stiffness_to_the_command_group():
    task = Task('js').joint_absolute(
        'right_arm', [0.0] * 7, JOINT_MOTION, impedance=JS,
    )
    command = task.task_list[0]
    assert command.impedance.groups == ('right_arm',)
    assert command.impedance.stiffness_for('right_arm') == (100.0,) * 7
    assert command.impedance.torque_limit == 10.0
    assert command.impedance.hold_time == 0.0
    assert command.uses_impedance


def test_joint_builder_rejects_wrong_length_or_bound_group():
    with pytest.raises(ValueError, match='requires 6 values'):
        Task('bad').joint_absolute('torso', [0.0] * 6, JOINT_MOTION, impedance=JS)
    bound = JointImpedance({'left_arm': [50.0] * 7})
    with pytest.raises(ValueError, match='bound to'):
        Task('bad').joint_relative(
            'right_arm', [0.0] * 7, JOINT_MOTION, impedance=bound,
        )


def test_multi_group_joint_impedance_vector_must_fit_every_group():
    # A single vector is applied to every commanded group, so a 7-value arm
    # vector cannot be used for the 6-joint torso; use a mapping instead.
    targets = {'torso': [0.0] * 6, 'right_arm': [0.0] * 7}
    with pytest.raises(ValueError, match='torso stiffness'):
        Task('bad').joint_absolute_multi(targets, JOINT_MOTION, impedance=JS)
    mapping = JointImpedance({'torso': [200.0] * 6, 'right_arm': [100.0] * 7})
    command = Task('ok').whole_body_joint_absolute(
        torso=[0.0] * 6,
        right_arm=[0.0] * 7,
        joint_motion=JOINT_MOTION,
        impedance=mapping,
    ).task_list[0]
    assert command.kind is CommandKind.JOINT_ABSOLUTE_MULTI
    assert sorted(command.impedance.groups) == ['right_arm', 'torso']


def test_joint_impedance_rejects_out_of_range_values():
    with pytest.raises(ValueError, match='torque_limit'):
        JointImpedance([10.0] * 7, torque_limit=500.0)
    with pytest.raises(ValueError, match='within'):
        JointImpedance([2000.0] * 7)
    with pytest.raises(ValueError, match='hold_time'):
        JointImpedance([10.0] * 7, hold_time=-1.0)
    with pytest.raises(ValueError, match='hold_time'):
        JointImpedance([10.0] * 7, hold_time=11.0)


def test_mode_is_implied_by_the_command_kind():
    with pytest.raises(ValueError, match='must be None, True'):
        Task('bad').joint_absolute(
            'right_arm', [0.0] * 7, JOINT_MOTION, impedance=3,
        )
    with pytest.raises(ValueError, match='does not support impedance'):
        TaskCommand(
            kind=CommandKind.DELAY, seconds=1.0, impedance=JS.bind('right_arm'),
        )


# --- Cartesian contact (impedance tracked through the arm IK) ---------------

def test_plain_cartesian_builders_are_position_only_unless_asked():
    task = Task('t')
    task.linear_absolute('right_arm', [0.0] * 6, TCP_MOTION)
    task.linear_relative('right_arm', [0.0] * 6, TCP_MOTION)
    task.camera_linear_absolute('left_arm', 'tag_0', TCP_MOTION)
    assert all(step.impedance is None for step in task.task_list)
    task = Task('t')
    task.linear_absolute('right_arm', [0.0] * 6, TCP_MOTION, impedance=JS)
    task.linear_relative('right_arm', [0.0] * 6, TCP_MOTION, impedance=JS)
    task.camera_linear_absolute('left_arm', 'tag_0', TCP_MOTION, impedance=JS)
    assert [step.impedance.groups for step in task.task_list] == [
        ('right_arm',), ('right_arm',), ('left_arm',),
    ]


def test_cartesian_command_accepts_joint_impedance_bound_to_its_arm():
    command = _cartesian(impedance=JS.bind('right_arm'))
    assert command.uses_impedance
    assert impedance_tag_for(command.kind, command.impedance) == 'CS impedance'
    with pytest.raises(ValueError, match='bound to right_arm'):
        _cartesian(impedance=JS)
    with pytest.raises(ValueError, match='bound to right_arm'):
        _cartesian(impedance=JS.bind('left_arm'))
    with pytest.raises(ValueError, match='supports only'):
        _cartesian(group='torso', impedance=JointImpedance([100.0] * 6).bind('torso'))


def test_press_helper_builds_contact_then_retreat():
    commands = Task('press').press(
        'right_arm', [0.0, 0.0, -0.03], TCP_MOTION, impedance=CONTACT,
    ).build().commands
    assert [command.kind for command in commands] == [
        CommandKind.LINEAR_RELATIVE,
        CommandKind.LINEAR_RELATIVE,
    ]
    assert commands[0].impedance == CONTACT.bind('right_arm')
    assert commands[0].impedance.hold_time == 0.5
    assert commands[0].values == (0.0, 0.0, -0.03, 0.0, 0.0, 0.0)
    assert commands[1].impedance is None
    assert commands[1].values == pytest.approx((0.0, 0.0, 0.05, 0.0, 0.0, 0.0))


def test_grasp_and_place_helpers_act_between_contact_and_retreat():
    grasp = Task('grasp').grasp(
        'left_arm', [0.0, 0.0, -0.02], TCP_MOTION,
        impedance=CONTACT, gripper_side='left',
    ).build().commands
    assert [command.kind for command in grasp] == [
        CommandKind.LINEAR_RELATIVE,
        CommandKind.GRIPPER_CLOSE,
        CommandKind.LINEAR_RELATIVE,
    ]
    assert grasp[0].impedance == CONTACT.bind('left_arm')
    assert grasp[2].values == pytest.approx((0.0, 0.0, 0.04, 0.0, 0.0, 0.0))

    place = Task('place').place_on_surface(
        'left_arm', [0.0, 0.0, -0.02], TCP_MOTION,
        impedance=CONTACT, gripper_side='left', retreat=[0.0, 0.0, 0.08],
    ).build().commands
    assert [command.kind for command in place] == [
        CommandKind.LINEAR_RELATIVE,
        CommandKind.GRIPPER_OPEN,
        CommandKind.LINEAR_RELATIVE,
    ]
    assert place[2].values == (0.0, 0.0, 0.08, 0.0, 0.0, 0.0)


def test_contact_helpers_reject_torso_and_non_joint_impedance():
    with pytest.raises(ValueError, match='supports only'):
        Task('bad').press('torso', [0.0, 0.0, -0.03], TCP_MOTION, impedance=CONTACT)
    with pytest.raises(ValueError, match='require impedance'):
        Task('bad').press('right_arm', [0.0, 0.0, -0.03], TCP_MOTION, impedance=None)
    with pytest.raises(ValueError, match='must be None, True'):
        Task('bad').press('right_arm', [0.0, 0.0, -0.03], TCP_MOTION, impedance=3)


def test_camera_grasp_helper_builds_contact_close_and_lift():
    definition = Task('camera_grasp').camera_grasp(
        'left_arm', 'tag_0', TCP_MOTION,
        impedance=CONTACT, gripper_side='left', retreat=[0.0, 0.0, 0.075],
        yaw_only=True, object_to_end_effector_position=(0.0, 0.0, 0.06),
    ).build()
    assert isinstance(definition, DynamicTaskDefinition)
    commands = definition.commands
    step = commands[0]
    assert isinstance(step, CameraLinearAbsoluteStep)
    assert step.impedance == CONTACT.bind('left_arm')
    assert step.yaw_only is True
    assert step.resolve((0.0,) * 6).impedance == CONTACT.bind('left_arm')
    assert commands[1].kind is CommandKind.GRIPPER_CLOSE
    assert commands[2].kind is CommandKind.LINEAR_RELATIVE
    assert commands[2].impedance is None


def test_contact_step_without_position_exit_is_rejected_at_build():
    task = Task('contact')._contact_relative(
        'right_arm', (0.0, 0.0, -0.03), TCP_MOTION, CONTACT,
    )
    with pytest.raises(ValueError, match='followed by a position-controlled'):
        task.build()
    task.open_gripper('right')
    with pytest.raises(ValueError, match='right_arm'):
        task.build()
    task.linear_relative('right_arm', [0.0, 0.0, 0.05, 0.0, 0.0, 0.0], TCP_MOTION)
    task.build()


def test_exit_rule_accepts_joint_moves_and_ignores_other_steps():
    contact = Task('x')._contact_relative(
        'right_arm', (0.0, 0.0, -0.03), TCP_MOTION, CONTACT,
    ).task_list[0]
    exit_move = Task('y').joint_relative(
        'right_arm', [0.0] * 7, JOINT_MOTION,
    ).task_list[0]
    delay = Task('z').delay(500).task_list[0]
    check_impedance_exit([contact, delay, exit_move])
    with pytest.raises(ValueError):
        check_impedance_exit([exit_move, contact, delay])


def test_impedance_tags():
    assert impedance_tag(JS) == 'JS impedance'
    assert impedance_tag(None) == ''
    assert impedance_tag_for(CommandKind.JOINT_ABSOLUTE, JS) == 'JS impedance'
    assert impedance_tag_for(CommandKind.LINEAR_RELATIVE, JS) == 'CS impedance'
    assert impedance_tag_for(CommandKind.LINEAR_RELATIVE, None) == ''


# --- wire format -----------------------------------------------------------

@pytest.mark.parametrize(
    'command',
    [
        TaskCommand(
            kind=CommandKind.JOINT_ABSOLUTE,
            group='right_arm',
            values=(0.0,) * 7,
            minimum_time=3.0,
            velocity_limit=0.5,
            acceleration_limit=0.5,
            impedance=JS.bind('right_arm'),
        ),
        _cartesian(impedance=CONTACT.bind('right_arm')),
    ],
)
def test_impedance_round_trips_over_the_control_protocol(command):
    payload = task_command_to_dict(command)
    assert payload['impedance']['type'] == 'joint'
    assert payload['impedance']['hold_time'] == command.impedance.hold_time
    assert task_command_from_dict(payload) == command


def test_legacy_payloads_decode_without_hold_time_or_impedance():
    payload = task_command_to_dict(_cartesian())
    assert 'impedance' not in payload
    assert task_command_from_dict(payload).impedance is None
    payload = task_command_to_dict(_cartesian(impedance=JS.bind('right_arm')))
    del payload['impedance']['hold_time']
    assert task_command_from_dict(payload).impedance.hold_time == 0.0
    payload['impedance'] = {'type': 'cartesian', 'translation': [1, 1, 1]}
    with pytest.raises(ValueError, match='unknown impedance type'):
        task_command_from_dict(payload)


def test_control_mode_round_trips_and_defaults_to_idle():
    state = FakeBackend().task_state()
    assert state.control_mode == 'idle'
    payload = task_backend_state_to_dict(state)
    payload['control_mode'] = 'impedance_cartesian'
    assert task_backend_state_from_dict(payload).control_mode == 'impedance_cartesian'
    del payload['control_mode']
    assert task_backend_state_from_dict(payload).control_mode == 'idle'


def test_catalog_round_trips_impedance(tmp_path):
    task = CanonicalTask('press').press(
        'right_arm', [0.0, 0.0, -0.03], TCP_MOTION, impedance=CONTACT,
    ).build()
    path = tmp_path / 'catalog.json'
    save_catalog({'press': task}, path)
    assert load_catalog(path)['press'] == task


# --- runner ----------------------------------------------------------------

def _runner(backend, statuses):
    return PlannerTaskRunner(
        FakeNode(),
        backend,
        on_status=statuses.append,
    )


def test_runner_sends_authored_joint_impedance():
    backend = FakeBackend()
    statuses = []
    runner = _runner(backend, statuses)
    task = Task('js').joint_relative(
        'right_arm', [0.0] * 7, JOINT_MOTION, impedance=JS,
    ).build()

    runner.start(task)
    while runner.active:
        runner.tick()

    assert backend.commands[0].kind is CommandKind.JOINT_ABSOLUTE
    assert backend.commands[0].impedance == JS.bind('right_arm')
    assert any('[JS impedance:' in status for status in statuses)


def test_runner_resolves_contact_step_and_keeps_its_impedance():
    backend = FakeBackend()
    statuses = []
    runner = _runner(backend, statuses)
    task = Task('press').press(
        'right_arm', [0.0, 0.0, -0.03], TCP_MOTION, impedance=CONTACT,
    ).build()

    runner.start(task)
    while runner.active:
        runner.tick()

    assert backend.commands[0].kind is CommandKind.LINEAR_ABSOLUTE
    assert backend.commands[0].impedance == CONTACT.bind('right_arm')
    assert backend.commands[1].impedance is None
    assert any('[CS impedance:' in status and 'hold=0.50 s' in status
               for status in statuses)


def test_runner_budgets_hold_time_and_reanchor_for_impedance():
    backend = FakeBackend()
    runner = _runner(backend, [])
    state = backend.task_state()
    base = TaskCommand(
        kind=CommandKind.JOINT_ABSOLUTE,
        group='right_arm',
        values=(0.0,) * 7,
        minimum_time=3.0,
        velocity_limit=0.5,
        acceleration_limit=0.5,
    )
    plain = runner._command_timeout_seconds(base, state)
    with_impedance = runner._command_timeout_seconds(
        TaskCommand(**{**base.__dict__, 'impedance': CONTACT.bind('right_arm')}),
        state,
    )
    assert with_impedance == pytest.approx(
        plain
        + (PlannerTaskRunner.IMPEDANCE_REANCHOR_MARGIN_SEC + CONTACT.hold_time)
        * PlannerTaskRunner.MOTION_TIMEOUT_SCALE
    )
