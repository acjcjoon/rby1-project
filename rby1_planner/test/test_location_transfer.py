"""Automatic moves between authored transfers; no robot or ROS is started."""
import importlib

import pytest
import yaml

from rby1_planner import task as task_source
from rby1_planner import task_commands
from rby1_planner.task_commands import Task


@pytest.fixture
def settings_file(tmp_path, monkeypatch):
    (tmp_path / 'package.xml').write_text('<package/>', encoding='utf-8')
    (tmp_path / 'config').mkdir()
    path = tmp_path / 'config' / 'settings.yaml'
    locations = {
        'A': {'position': [0.0, 0.0], 'side': 'right'},
        'B': {'position': [-0.5, 0.0], 'side': 'right'},
        'C': {'position': [0.0, 0.81], 'side': 'left'},
        'D': {'position': [0.6, 0.81], 'side': 'left'},
    }
    for index, (name, tag_id) in enumerate(
        zip(locations, ('tag_2', 'tag_1', 'tag_3', 'tag_0')),
    ):
        locations[name][tag_id] = {
            'offset': [-0.01 * index, 0.02 * index, -0.003 * index],
        }
    locations['current_location'] = None
    path.write_text(yaml.safe_dump({'locations': locations}), encoding='utf-8')
    monkeypatch.setattr(
        task_commands, '__file__',
        str(tmp_path / 'rby1_planner' / 'task_commands.py'),
    )
    monkeypatch.setattr(task_source, 'current_position', None)
    return path


def set_initial_location(path, name):
    settings = yaml.safe_load(path.read_text(encoding='utf-8'))
    settings['locations']['current_location'] = name
    path.write_text(yaml.safe_dump(settings), encoding='utf-8')


def test_automatic_transfers_match_the_existing_demo(settings_file):
    original_settings = settings_file.read_bytes()
    expected = task_source.object_handover_demo_final_tmp()
    actual = Task(expected.name)
    actual.extend(task_source.object_gripping_initial_pose_torso_right())
    for start, end in (('A', 'C'), ('B', 'D'), ('C', 'A'), ('D', 'B')):
        actual.extend(task_source.pick_up_move_and_put_down_object2(start, end))
        assert task_source.current_position == end
    actual.extend(task_source.turn_and_move('B', 'A'))

    # Includes the same tag-specific offsets and intermediate movements.
    assert actual.build() == expected
    assert settings_file.read_bytes() == original_settings


def test_matching_start_skips_the_intermediate_move(settings_file):
    task_source.pick_up_move_and_put_down_object2('A', 'C')
    actual = task_source.pick_up_move_and_put_down_object2('C', 'D')
    expected = task_source.pick_up_move_and_put_down_object('C', 'D')

    assert actual.commands == expected.commands
    assert task_source.current_position == 'D'


def test_initial_location_from_yaml_adds_the_first_move(settings_file):
    set_initial_location(settings_file, 'B')
    original_settings = settings_file.read_bytes()
    actual = task_source.pick_up_move_and_put_down_object2('A', 'C')
    expected_commands = (
        task_source.turn_and_move('B', 'A').commands
        + task_source.pick_up_move_and_put_down_object('A', 'C').commands
    )

    assert actual.commands == expected_commands
    assert task_source.current_position == 'C'
    assert settings_file.read_bytes() == original_settings


@pytest.mark.parametrize('start, end', [
    ('missing', 'D'), ('B', 'missing'), ('', 'D'), ('B', None),
])
def test_invalid_transfer_does_not_advance_the_position(
    settings_file, start, end,
):
    task_source.pick_up_move_and_put_down_object2('A', 'C')
    with pytest.raises(ValueError, match='location'):
        task_source.pick_up_move_and_put_down_object2(start, end)

    assert task_source.current_position == 'C'
    actual = task_source.pick_up_move_and_put_down_object2('B', 'D')
    assert actual.commands == (
        task_source.turn_and_move('C', 'B').commands
        + task_source.pick_up_move_and_put_down_object('B', 'D').commands
    )


def test_bad_destination_offset_does_not_advance_the_position(settings_file):
    task_source.pick_up_move_and_put_down_object2('A', 'C')
    settings = yaml.safe_load(settings_file.read_text(encoding='utf-8'))
    settings['locations']['D']['tag_0']['offset'] = [0.0, 0.0, 'invalid']
    settings_file.write_text(yaml.safe_dump(settings), encoding='utf-8')

    with pytest.raises(ValueError, match='offset'):
        task_source.pick_up_move_and_put_down_object2('B', 'D')
    assert task_source.current_position == 'C'


@pytest.mark.parametrize('location', ['missing', '', True, []])
def test_invalid_initial_location_is_rejected(settings_file, location):
    set_initial_location(settings_file, location)

    with pytest.raises(ValueError, match='location'):
        task_source.pick_up_move_and_put_down_object2('A', 'C')
    assert task_source.current_position is None


def test_source_reload_resets_to_the_yaml_initial_location(settings_file):
    set_initial_location(settings_file, 'B')
    first = task_source.pick_up_move_and_put_down_object2('A', 'C')
    assert task_source.current_position == 'C'

    importlib.reload(task_source)
    assert task_source.current_position is None
    second = task_source.pick_up_move_and_put_down_object2('A', 'C')
    assert second == first


def test_location_metadata_does_not_break_the_task_registry(settings_file):
    assert set(task_commands._load_move_locations()) == {'A', 'B', 'C', 'D'}
    tasks = task_source.build_tasks()
    assert tasks['object_handover_demo_final'].commands == (
        tasks['object_handover_demo_final_tmp'].commands
    )
