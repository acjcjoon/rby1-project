"""Named-area moves are constructed offline; no robot or ROS node is started."""
import sys
from types import ModuleType

import pytest
import yaml

from rby1_planner import task as task_source
from rby1_planner import task_commands
from rby1_planner.control_commands import CommandKind
from rby1_planner.task_commands import MoveToStep, Task
from station_settings import HEAD_PITCH_DEG, stations_settings


def head_move():
    """turn_and_move 가 이동 전에 넣는 머리 절대 이동 명령 (head_pitch_deg.move).
    도착 후 머리는 건드리지 않는다 (뒤따르는 장비 작업이 지점 각도로 맞춘다)."""
    return task_source.head_pitch_pose(HEAD_PITCH_DEG['move']).commands[0]


@pytest.fixture
def settings_file(tmp_path, monkeypatch):
    # Make the loader use an isolated source-package layout.
    (tmp_path / 'package.xml').write_text('<package/>', encoding='utf-8')
    (tmp_path / 'config').mkdir()
    path = tmp_path / 'config' / 'settings.yaml'
    monkeypatch.setattr(
        task_commands, '__file__', str(tmp_path / 'rby1_planner' / 'task_commands.py'),
    )
    path.write_text(yaml.safe_dump(stations_settings({
            'A': {'position': [0.0, 0.0], 'side': 'right'},
            'B': {'position': [0.0, 0.87], 'side': 'left'},
        })), encoding='utf-8')
    return path


@pytest.mark.parametrize('start, end, side, y', [
    ('A', 'B', 'left', 0.87),
    ('B', 'A', 'right', -0.87),
])
def test_default_locations_turn_then_move(settings_file, start, end, side, y):
    definition = task_source.turn_and_move(start, end)
    expected = Task('expected')
    expected.extend(task_source.head_pitch_pose(HEAD_PITCH_DEG['move']))
    expected.extend(task_source.object_gripping_initial_pose(side))
    expected.move_to(0.0, y, 0.0, timeout_sec=15.0)
    assert definition.name == 'turn_and_move'
    assert definition.commands == expected.build().commands


@pytest.mark.parametrize('start, end, expected_xy, side', [
    ('A', 'B', (0.4, 0.87), 'left'),
    ('B', 'A', (-0.4, -0.87), 'right'),
    ('C', 'D', (0.5, -0.2), 'left'),
])
def test_xy_difference_and_destination_side(settings_file, start, end, expected_xy, side):
    settings_file.write_text(yaml.safe_dump(stations_settings({
            'A': {'position': [0.2, -0.3], 'side': 'right'},
            'B': {'position': [0.6, 0.57], 'side': 'left'},
            'C': {'position': [-0.2, 0.5], 'side': 'right'},
            'D': {'position': [0.3, 0.3], 'side': 'left'},
        })), encoding='utf-8')
    commands = task_source.turn_and_move(start, end).commands
    assert len(commands) == 3
    pose = task_source.object_gripping_initial_pose(side)
    assert commands[0] == head_move()
    assert commands[1] == pose.commands[0]
    assert isinstance(commands[2], MoveToStep)
    assert (commands[2].x, commands[2].y) == pytest.approx(expected_xy)
    assert commands[2].yaw == 0.0
    assert commands[2].timeout_sec == 15.0


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('start, end, expected_xy', [
    ('A', 'B', (0.4, 0.87)),
    ('B', 'A', (-0.4, -0.87)),
    ('B', 'B', (0.0, 0.0)),
])
def test_same_side_skips_both_preparation_poses(
    settings_file, monkeypatch, side, start, end, expected_xy,
):
    settings_file.write_text(yaml.safe_dump(stations_settings({
            'A': {'position': [0.0, 0.0], 'side': side},
            'B': {'position': [0.4, 0.87], 'side': side},
        })), encoding='utf-8')

    def unexpected_pose(*args, **kwargs):
        pytest.fail('Same-side movement must not call a preparation pose')

    for name in ('object_gripping_initial_pose', 'torso_align'):
        monkeypatch.setattr(task_source, name, unexpected_pose)

    commands = task_source.turn_and_move(start, end).commands
    assert len(commands) == 2
    assert commands[0] == head_move()
    assert isinstance(commands[1], MoveToStep)
    assert (commands[1].x, commands[1].y) == pytest.approx(expected_xy)
    assert commands[1].yaw == 0.0
    assert commands[1].timeout_sec == 15.0


def test_settings_are_reloaded_on_each_call(settings_file):
    assert task_source.turn_and_move('A', 'B').commands[2].y == 0.87
    settings_file.write_text(yaml.safe_dump(stations_settings({
            'A': {'position': [0, 0], 'side': 'right'},
            'B': {'position': [0.1, 0.5], 'side': 'right'},
        })), encoding='utf-8')
    commands = task_source.turn_and_move('A', 'B').commands
    assert len(commands) == 2
    assert commands[1] == MoveToStep(0.1, 0.5, 0.0, 15.0)


def test_head_is_set_to_move_angle_before_moving_only(settings_file):
    data = yaml.safe_load(settings_file.read_text(encoding='utf-8'))
    data['head_pitch_deg'] = {'move': 5.0}
    data['stations']['B']['points']['place']['head_pitch_deg'] = 25.0   # 도착 지점 각도는 쓰지 않는다
    settings_file.write_text(yaml.safe_dump(data), encoding='utf-8')
    commands = task_source.turn_and_move('A', 'B').commands
    heads = [c for c in commands if getattr(c, 'group', None) == 'head']
    assert [c.kind for c in heads] == [CommandKind.JOINT_ABSOLUTE]
    assert [c.values for c in heads] == [(0.0, 5.0)]
    assert commands[0] == heads[0]
    assert isinstance(commands[-1], MoveToStep)


def test_move_head_false_leaves_head_alone(settings_file):
    commands = task_source.turn_and_move('A', 'B', move_head=False).commands
    assert not [c for c in commands if getattr(c, 'group', None) == 'head']


@pytest.mark.parametrize('head_pitch', [None, {}, {'move': 95.0}, {'move': 'x'}])
def test_missing_or_bad_head_pitch_is_rejected(settings_file, head_pitch):
    data = yaml.safe_load(settings_file.read_text(encoding='utf-8'))
    if head_pitch is None:
        data.pop('head_pitch_deg')
    else:
        data['head_pitch_deg'] = head_pitch
    settings_file.write_text(yaml.safe_dump(data), encoding='utf-8')
    with pytest.raises(ValueError, match='head_pitch_deg'):
        task_source.turn_and_move('A', 'B')


@pytest.mark.parametrize('value', [None, 90.0, '30'])
def test_bad_point_head_pitch_is_rejected(settings_file, value):
    data = yaml.safe_load(settings_file.read_text(encoding='utf-8'))
    data['stations']['A']['points']['place']['head_pitch_deg'] = value
    settings_file.write_text(yaml.safe_dump(data), encoding='utf-8')
    with pytest.raises(ValueError, match='head_pitch_deg'):
        task_source.turn_and_move('A', 'B')


def test_point_without_head_pitch_defaults_to_zero(settings_file):
    data = yaml.safe_load(settings_file.read_text(encoding='utf-8'))
    data['stations']['B']['points']['place'].pop('head_pitch_deg')
    settings_file.write_text(yaml.safe_dump(data), encoding='utf-8')
    assert task_commands._load_point_head_pitch('B') == 0.0


@pytest.mark.parametrize('start, end', [
    ('missing', 'A'), ('A', 'missing'), ('', 'A'), ('A', None),
])
def test_invalid_location_names_are_rejected(settings_file, start, end):
    with pytest.raises(ValueError, match='location'):
        task_source.turn_and_move(start, end)


@pytest.mark.parametrize('text', [
    '',
    '[]',
    'stations: []',
    'stations: {}',
    'stations: [',
    'locations: {A: {position: [0, 0], side: right}, B: {position: [0, 1], side: left}}',
    'stations: {A: null, B: {kind: plain, points: {place: {position: [0, 1], side: left}}, place: {point: place}}}',
    'stations: {A: {kind: plain, points: {place: {position: [0, 0], side: right}}, place: {point: place}}, B: {kind: plain, points: {place: {side: left}}, place: {point: place}}}',
    'stations: {A: {kind: plain, points: {place: {position: [0, 0], side: right}}, place: {point: place}}, B: {kind: plain, points: {place: {position: [0], side: left}}, place: {point: place}}}',
    'stations: {A: {kind: plain, points: {place: {position: [0, 0], side: right}}, place: {point: place}}, B: {kind: plain, points: {place: {position: [0, 1], side: up}}, place: {point: place}}}',
    'stations: {A: {kind: plain, points: {place: {position: [0, 0], side: right}}, place: {point: place}}, B: {kind: plain, points: {place: {position: [0, 1]}}, place: {point: place}}}',
    'stations: {A: {kind: plain, points: {place: {position: [0, 0], side: right}}, place: {point: place}}, B: {kind: plain, points: {place: {position: [true, 1], side: left}}, place: {point: place}}}',
    'stations: {A: {kind: plain, points: {place: {position: [0, 0], side: right}}, place: {point: place}}, B: {kind: plain, points: {place: {position: [.nan, 1], side: left}}, place: {point: place}}}',
    'stations: {A: {kind: plain, points: {place: {position: [0, 0], side: right}}, place: {point: place}}, B: {kind: plain, points: {place: {position: [0, .inf], side: left}}, place: {point: place}}}',
    'stations: {A: {kind: plain, points: {place: {position: [0, 0], side: right}}, place: {point: place}}, B: {kind: plain, points: {place: {position: ["0", 1], side: left}}, place: {point: place}}}',
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
    # 장비 위치와 머리 각도를 읽을 때마다 설치 share 경로를 쓴다
    assert calls and set(calls) == {'rby1_planner'}
    assert definition.commands[2] == MoveToStep(0.0, 0.87, 0.0, 15.0)
