from itertools import combinations

import pytest

from rby1_planner.control_commands import CommandKind, Task as CanonicalTask
from rby1_planner.control_protocol import (
    task_command_from_dict,
    task_command_to_dict,
)
from rby1_planner.task_commands import Task as PlannerTask


TARGETS = {
    'torso': (1.0,) * 6,
    'right_arm': (2.0,) * 7,
    'left_arm': (3.0,) * 7,
}
JOINT_MOTION = (5.0, 1.0, 1.0)
GROUP_COMBINATIONS = [
    groups
    for count in (1, 2, 3)
    for groups in combinations(TARGETS, count)
]


@pytest.fixture(params=[CanonicalTask, PlannerTask], ids=['canonical', 'planner'])
def task(request):
    return request.param('optional-body-joints')


@pytest.mark.parametrize('groups', GROUP_COMBINATIONS)
def test_whole_body_moves_only_selected_groups(task, groups):
    targets = {group: TARGETS[group] for group in groups}
    assert task.whole_body_joint_absolute(
        **targets, joint_motion=JOINT_MOTION,
    ) is task

    commands = task.build().commands
    assert len(commands) == 1
    command = commands[0]
    if len(groups) == 1:
        assert command.kind is CommandKind.JOINT_ABSOLUTE
        assert command.group == groups[0]
        assert command.values == TARGETS[groups[0]]
        assert command.joint_targets == ()
    else:
        assert command.kind is CommandKind.JOINT_ABSOLUTE_MULTI
        assert command.group is None
        assert command.joint_targets == tuple(targets.items())
        assert command.values == ()
    assert (
        command.minimum_time,
        command.velocity_limit,
        command.acceleration_limit,
    ) == JOINT_MOTION
    assert task_command_from_dict(task_command_to_dict(command)) == command


def test_whole_body_none_omits_torso_target(task):
    task.whole_body_joint_absolute(
        torso=None,
        right_arm=TARGETS['right_arm'],
        left_arm=TARGETS['left_arm'],
        joint_motion=JOINT_MOTION,
    )
    assert dict(task.build().commands[0].joint_targets) == {
        'right_arm': TARGETS['right_arm'],
        'left_arm': TARGETS['left_arm'],
    }


@pytest.mark.parametrize('targets', [{}, dict.fromkeys(TARGETS)])
def test_whole_body_rejects_no_targets(task, targets):
    with pytest.raises(ValueError, match='at least one'):
        task.whole_body_joint_absolute(**targets, joint_motion=JOINT_MOTION)
    assert task.task_list == []


@pytest.mark.parametrize('targets', [
    {'left_arm': ()},
    {'torso': (0.0,) * 7},
    {'right_arm': (0.0,) * 6},
    {'left_arm': (float('nan'),) * 7},
    {'left_arm': (float('inf'),) * 7},
    {'right_arm': TARGETS['right_arm'], 'left_arm': ()},
])
def test_whole_body_preserves_target_validation(task, targets):
    with pytest.raises(ValueError):
        task.whole_body_joint_absolute(**targets, joint_motion=JOINT_MOTION)
    assert task.task_list == []


@pytest.mark.parametrize('groups', [('left_arm',), ('right_arm', 'left_arm')])
@pytest.mark.parametrize('joint_motion', [(0.0, 1.0, 1.0), (5.0, -1.0, 1.0)])
def test_whole_body_preserves_motion_validation(task, groups, joint_motion):
    with pytest.raises(ValueError):
        task.whole_body_joint_absolute(
            **{group: TARGETS[group] for group in groups},
            joint_motion=joint_motion,
        )
    assert task.task_list == []


def test_whole_body_still_requires_joint_motion(task):
    with pytest.raises(TypeError, match='joint_motion'):
        task.whole_body_joint_absolute(left_arm=TARGETS['left_arm'])
    assert task.task_list == []
