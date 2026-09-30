"""Named-area moves are constructed offline; no robot or ROS node is started."""
import sys
from types import ModuleType

import pytest
import yaml

from rby1_planner import task as task_source
from rby1_planner import task_commands
from rby1_planner.control_commands import CommandKind
from rby1_planner.task_commands import MoveToStep


@pytest.fixture
def settings_file(tmp_path, monkeypatch):
    # Make the loader use an isolated source-package layout.
    (tmp_path / 'package.xml').write_text('<package/>', encoding='utf-8')
    (tmp_path / 'config').mkdir()
    path = tmp_path / 'config' / 'settings.yaml'
    monkeypatch.setattr(
        task_commands, '__file__', str(tmp_path / 'rby1_planner' / 'task_commands.py'),
    )
    path.write_text(yaml.safe_dump({
        'locations': {
            'A': {'position': [0.0, 0.0], 'side': 'right'},
            'B': {'position': [0.0, 0.87], 'side': 'left'},
        },
    }), encoding='utf-8')
    return path


@pytest.mark.parametrize('start, end, previous', [
    ('A', 'B', 'turn_left_and_move_left'),
    ('B', 'A', 'turn_right_and_move_right'),
])
def test_default_locations_match_existing_move_commands(settings_file, start, end, previous):
    definition = task_source.turn_and_move(start, end)
    assert definition.name == 'turn_and_move'
    assert definition.commands == getattr(task_source, previous)().commands


@pytest.mark.parametrize('start, end, expected_xy, side', [
    ('A', 'B', (0.4, 0.87), 'left'),
    ('B', 'A', (-0.4, -0.87), 'right'),
    ('C', 'D', (0.5, -0.2), 'left'),
])
def test_xy_difference_and_destination_side(settings_file, start, end, expected_xy, side):
    settings_file.write_text(yaml.safe_dump({
        'locations': {
            'A': {'position': [0.2, -0.3], 'side': 'right'},
            'B': {'position': [0.6, 0.57], 'side': 'left'},
            'C': {'position': [-0.2, 0.5], 'side': 'right'},
            'D': {'position': [0.3, 0.3], 'side': 'left'},
        },
    }), encoding='utf-8')
    commands = task_source.turn_and_move(start, end).commands
    assert len(commands) == 3
    pose = getattr(task_source, f'object_gripping_initial_pose_torso_{side}')()
    assert commands[0] == pose.commands[0]
    assert isinstance(commands[1], MoveToStep)
    assert (commands[1].x, commands[1].y) == pytest.approx(expected_xy)
    assert commands[1].yaw == 0.0
    assert commands[1].timeout_sec == 15.0
    assert commands[2].kind is CommandKind.DELAY
    assert commands[2].seconds == 1.0


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('start, end, expected_xy', [
    ('A', 'B', (0.4, 0.87)),
    ('B', 'A', (-0.4, -0.87)),
    ('B', 'B', (0.0, 0.0)),
])
def test_same_side_skips_both_preparation_poses(
    settings_file, monkeypatch, side, start, end, expected_xy,
):
    settings_file.write_text(yaml.safe_dump({
        'locations': {
            'A': {'position': [0.0, 0.0], 'side': side},
            'B': {'position': [0.4, 0.87], 'side': side},
        },
    }), encoding='utf-8')

    def unexpected_pose():
        pytest.fail('Same-side movement must not call a preparation pose')

    for name in ('object_gripping_initial_pose_torso_left',
                 'object_gripping_initial_pose_torso_right'):
        monkeypatch.setattr(task_source, name, unexpected_pose)

    commands = task_source.turn_and_move(start, end).commands
    assert len(commands) == 2
    assert isinstance(commands[0], MoveToStep)
    assert (commands[0].x, commands[0].y) == pytest.approx(expected_xy)
    assert commands[0].yaw == 0.0
    assert commands[0].timeout_sec == 15.0
    assert commands[1].kind is CommandKind.DELAY
    assert commands[1].seconds == 1.0


def test_settings_are_reloaded_on_each_call(settings_file):
    assert task_source.turn_and_move('A', 'B').commands[1].y == 0.87
    settings_file.write_text(yaml.safe_dump({
        'locations': {
            'A': {'position': [0, 0], 'side': 'right'},
            'B': {'position': [0.1, 0.5], 'side': 'right'},
        },
    }), encoding='utf-8')
    commands = task_source.turn_and_move('A', 'B').commands
    assert len(commands) == 2
    assert commands[0] == MoveToStep(0.1, 0.5, 0.0, 15.0)


@pytest.mark.parametrize('start, end', [
    ('missing', 'A'), ('A', 'missing'), ('', 'A'), ('A', None),
])
def test_invalid_location_names_are_rejected(settings_file, start, end):
    with pytest.raises(ValueError, match='location'):
        task_source.turn_and_move(start, end)


@pytest.mark.parametrize('text', [
    '', '[]', 'locations: []', 'locations: {}', 'locations: [',
    'locations: {A: null, B: {position: [0, 1], side: left}}',
    'locations: {A: {position: [0, 0], side: right}, B: {side: left}}',
    'locations: {A: {position: [0, 0], side: right}, B: {position: [0], side: left}}',
    'locations: {A: {position: [0, 0], side: right}, B: {position: [0, 1], side: front}}',
    'locations: {A: {position: [0, 0], side: right}, B: {position: [0, 1]}}',
    'locations: {A: {position: [0, 0], side: right}, B: {position: [true, 1], side: left}}',
    'locations: {A: {position: [0, 0], side: right}, B: {position: [.nan, 1], side: left}}',
    'locations: {A: {position: [0, 0], side: right}, B: {position: [0, .inf], side: left}}',
    'locations: {A: {position: [0, 0], side: right}, B: {position: ["0", 1], side: left}}',
])
def test_invalid_yaml_is_rejected(settings_file, text):
    settings_file.write_text(text, encoding='utf-8')
    with pytest.raises(ValueError):
        task_source.turn_and_move('A', 'B')


def test_missing_source_settings_does_not_fall_back_to_old_install(settings_file):
    settings_file.unlink()
    with pytest.raises(ValueError, match='Cannot load planner settings'):
        task_source.turn_and_move('A', 'B')


def test_installed_package_uses_share_config(settings_file, tmp_path, monkeypatch):
    # No package.xml beside this pretend installed Python module.
    monkeypatch.setattr(
        task_commands, '__file__', str(tmp_path / 'site-packages' / 'rby1_planner' / 'task_commands.py'),
    )
    ament_index = ModuleType('ament_index_python')
    packages = ModuleType('ament_index_python.packages')
    calls = []
    def package_share(name):
        calls.append(name)
        return str(tmp_path)
    packages.get_package_share_directory = package_share
    ament_index.packages = packages
    monkeypatch.setitem(sys.modules, 'ament_index_python', ament_index)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    definition = task_source.turn_and_move('A', 'B')
    assert calls == ['rby1_planner']
    assert definition.commands[1] == MoveToStep(0.0, 0.87, 0.0, 15.0)
