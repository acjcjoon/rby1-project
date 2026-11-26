"""장비 사이 운반(transfer) 구성; 로봇/ROS 없음."""
import copy
from pathlib import Path

import pytest
import yaml

from rby1_planner import task as task_source
from rby1_planner import task_commands
from rby1_planner.task_commands import CameraMoveToTagStep, MoveToStep, Task
from station_settings import HEAD_PITCH_DEG, POINT_HEAD_PITCH_DEG, stations_settings

REAL_SETTINGS = Path(task_commands.__file__).resolve().parents[1] / 'config' / 'settings.yaml'
REAL = yaml.safe_load(REAL_SETTINGS.read_text(encoding='utf-8'))


@pytest.fixture
def settings_file(tmp_path, monkeypatch):
    """plain 장비 4개 + 실제 ZS(door_switch) 설정."""
    (tmp_path / 'package.xml').write_text('<package/>', encoding='utf-8')
    (tmp_path / 'config').mkdir()
    path = tmp_path / 'config' / 'settings.yaml'
    locations = {
        'INBOX': {'position': [0.0, 0.0], 'side': 'right'},
        'STORE': {'position': [-0.5, 0.0], 'side': 'right'},
        'LH1': {'position': [0.4, 0.0], 'side': 'front'},
        'LH2': {'position': [0.4, 0.81], 'side': 'right'},
    }
    for index, (name, tag_id) in enumerate(
        zip(locations, ('tag_12', 'tag_1', 'tag_3', 'tag_0')),
    ):
        locations[name][tag_id] = {
            'offset': [-0.01 * index, 0.02 * index, -0.003 * index],
        }
    settings = stations_settings(locations)
    settings['stations']['ZS'] = copy.deepcopy(REAL['stations']['ZS'])
    settings['impedance_presets'] = REAL['impedance_presets']
    path.write_text(yaml.safe_dump(settings), encoding='utf-8')
    monkeypatch.setattr(
        task_commands, '__file__',
        str(tmp_path / 'rby1_planner' / 'task_commands.py'),
    )
    return path


def _at(station, arm, in_place):
    """_pick_or_place(plain) 와 같은 순서: 태그 조회 -> base 정렬 -> 파지/배치 -> 초기 자세."""
    tag = task_commands._load_location_tag_id(station)
    task = Task('at')
    task.extend(task_source.align_base_to_tag(tag, arm=arm))
    task.extend(in_place(tag, arm=arm))
    task.extend(task_source.object_gripping_initial_pose())
    return task.build()


def pick_up_at(station, arm):
    return _at(station, arm, task_source.pick_up_object_in_place)


def put_down_at(station, arm):
    return _at(station, arm, task_source.put_down_object_in_place)


def work_head():
    """plain 장비 집기/놓기 시작 때 맞추는 머리 각도 (집기/놓기 지점의 head_pitch_deg)."""
    return task_source.head_pitch_pose(POINT_HEAD_PITCH_DEG)


def expected_plain_transfer(source, destination, start_side):
    task = Task('expected')
    task.extend(task_source.object_gripping_initial_pose(start_side))
    task.extend(work_head())
    task.extend(pick_up_at(source, arm='left_arm'))
    task.extend(task_source.turn_and_move(source, destination))
    task.extend(work_head())
    task.extend(put_down_at(destination, arm='left_arm'))
    return task.build().commands


def test_plain_transfer_picks_moves_and_places(settings_file):
    original = settings_file.read_bytes()
    actual = task_source.transfer('LH1', 'LH2')
    assert actual.name == 'transfer(LH1,LH2)'
    assert actual.commands == expected_plain_transfer('LH1', 'LH2', 'front')
    moves = [c for c in actual.commands if isinstance(c, MoveToStep)]
    assert [(m.x, m.y) for m in moves] == [pytest.approx((0.0, 0.81))]
    assert [c.object_id for c in actual.commands if isinstance(c, CameraMoveToTagStep)] == ['tag_3', 'tag_0']
    assert settings_file.read_bytes() == original


def test_start_adds_the_first_move(settings_file):
    actual = task_source.transfer('INBOX', 'LH1', start='STORE')
    task = Task('expected')
    task.extend(task_source.object_gripping_initial_pose('right'))  # STORE side
    task.extend(task_source.turn_and_move('STORE', 'INBOX'))
    expected_rest = expected_plain_transfer('INBOX', 'LH1', 'right')[1:]
    assert actual.commands == task.build().commands + expected_rest


def test_start_at_source_adds_no_move(settings_file):
    assert (
        task_source.transfer('LH1', 'LH2', start='LH1').commands
        == task_source.transfer('LH1', 'LH2').commands
    )


@pytest.mark.parametrize('source, destination, start, message', [
    ('missing', 'LH2', None, 'location'),
    ('LH1', 'missing', None, 'location'),
    ('', 'LH2', None, 'location'),
    ('LH1', 'LH2', 'missing', 'location'),
    ('LH1', 'LH1', None, 'differ'),
    ('LH1', 'ZS.work', None, 'station name'),
])
def test_invalid_transfer_is_rejected(settings_file, source, destination, start, message):
    with pytest.raises(ValueError, match=message):
        task_source.transfer(source, destination, start=start)


def test_disabled_station_is_rejected(settings_file):
    settings = yaml.safe_load(settings_file.read_text(encoding='utf-8'))
    settings['stations']['LH2']['enabled'] = False
    settings_file.write_text(yaml.safe_dump(settings), encoding='utf-8')
    with pytest.raises(ValueError, match='disabled'):
        task_source.transfer('LH1', 'LH2')


def test_bad_destination_offset_is_rejected(settings_file):
    settings = yaml.safe_load(settings_file.read_text(encoding='utf-8'))
    settings['stations']['LH2']['place']['offset'] = [0.0, 0.0, 'invalid']
    settings_file.write_text(yaml.safe_dump(settings), encoding='utf-8')
    with pytest.raises(ValueError, match='offset'):
        task_source.transfer('LH1', 'LH2')


def test_go_to_uses_station_entry_points(settings_file):
    assert task_source.go_to('LH1', 'LH1').commands == ()
    # door_switch 장비의 출입 지점은 work
    assert (
        task_source.go_to('LH1', 'ZS').commands
        == task_source.turn_and_move('LH1', 'ZS.work').commands
    )
    assert (
        task_source.go_to('ZS', 'LH2').commands
        == task_source.turn_and_move('ZS.work', 'LH2').commands
    )


def test_door_switch_place_sequence(settings_file):
    commands = task_source.place_at_station('ZS').commands
    open_door = task_source.open_door('ZS').commands
    close_door = task_source.close_door('ZS').commands
    zs_points = REAL['stations']['ZS']['points']
    zs_work, zs_pitch = zs_points['work']['head_pitch_deg'], zs_points['place']['head_pitch_deg']
    # 문 base 정렬 전에 머리를 work 지점 각도로 맞춘다
    assert commands[0] == task_source.head_pitch_pose(zs_work).commands[0]
    assert commands[1:1 + len(open_door)] == open_door
    assert commands[-len(close_door):] == close_door
    moves = [(round(c.x, 3), round(c.y, 3)) for c in commands if isinstance(c, MoveToStep)]
    # work -> via -> place, place -> via -> work
    assert moves == [(-0.75, 0.0), (0.0, 0.7), (0.0, -0.7), (0.75, 0.0)]
    put_down = put_down_at('ZS', arm='left_arm').commands
    start = next(i for i in range(len(commands)) if commands[i:i + len(put_down)] == put_down)
    before, after = commands[start - 1], commands[start + len(put_down)]
    assert before.group == after.group == 'head'
    # ZS 놓기는 place 지점 각도로 숙였다가 바로 복귀 이동이라 move 각도로 든다
    assert before.values == (0.0, zs_pitch)
    assert after.values == (0.0, HEAD_PITCH_DEG['move'])
    # 경유 구간에서는 머리를 바꾸지 않는다: work -> move -> ZS -> move -> work
    pitches = [c.values[1] for c in commands if getattr(c, 'group', None) == 'head']
    assert pitches == [zs_work, HEAD_PITCH_DEG['move'], zs_pitch, HEAD_PITCH_DEG['move'], zs_work]
    aligned = [c.object_id for c in commands if isinstance(c, CameraMoveToTagStep)]
    assert aligned == ['tag_6', 'tag_2', 'tag_7']


def test_door_switch_pick_is_symmetric(settings_file):
    place = task_source.place_at_station('ZS').commands
    pick = task_source.pick_at_station('ZS').commands
    put_down = put_down_at('ZS', arm='left_arm').commands
    pick_up = pick_up_at('ZS', arm='left_arm').commands
    start = next(i for i in range(len(place)) if place[i:i + len(put_down)] == put_down)
    assert pick == place[:start] + pick_up + place[start + len(put_down):]


def test_transfer_into_door_switch_station(settings_file):
    actual = task_source.transfer('LH1', 'ZS').commands
    task = Task('expected')
    task.extend(task_source.object_gripping_initial_pose('front'))
    task.extend(work_head())
    task.extend(pick_up_at('LH1', arm='left_arm'))
    task.extend(task_source.go_to('LH1', 'ZS'))
    task.extend(task_source.place_at_station('ZS'))
    assert actual == task.build().commands


def test_demo_goes_back_to_lh1_between_transfers(settings_file):
    actual = task_source.transfer_demo()
    task = Task('expected')
    task.extend(task_source.transfer('LH1', 'ZS'))
    task.extend(task_source.transfer('LH1', 'LH2', start='ZS'))
    assert actual.commands == task.build().commands
    second = task_source.transfer('LH1', 'LH2', start='ZS').commands
    go_back = task_source.go_to('ZS', 'LH1').commands
    assert go_back and second[1:1 + len(go_back)] == go_back


def test_door_is_released_before_leaving_the_station(settings_file):
    commands = task_source.close_door('ZS').commands
    release, retreat = commands[-2:]
    assert release.kind.value == 'gripper_open' and release.group == 'right'
    assert retreat.group == 'right_arm' and retreat.values[1] < 0.0


def test_real_settings_task_registry_builds():
    tasks = task_source.build_tasks()
    for name in ('transfer_demo(LH1_1->ZS, LH1_2->LH2)', 'pick_at_station(ZS)'):
        assert tasks[name].commands, name
