"""문/스위치 장비 값이 settings.yaml stations 에서 읽히는지 (로봇/ROS 없음)."""
import copy
import math
from pathlib import Path

import pytest
import yaml

from rby1_planner import task as task_source
from rby1_planner import task_commands
from rby1_planner.task_commands import CameraLinearAbsoluteStep, CameraMoveToTagStep

REAL_SETTINGS = Path(task_commands.__file__).resolve().parents[1] / 'config' / 'settings.yaml'


@pytest.fixture
def settings(tmp_path, monkeypatch):
    """실제 settings.yaml 을 임시 패키지로 복사해 두고, write(dict) 로 바꿔 쓴다."""
    (tmp_path / 'package.xml').write_text('<package/>', encoding='utf-8')
    (tmp_path / 'config').mkdir()
    path = tmp_path / 'config' / 'settings.yaml'
    data = yaml.safe_load(REAL_SETTINGS.read_text(encoding='utf-8'))
    monkeypatch.setattr(
        task_commands, '__file__', str(tmp_path / 'rby1_planner' / 'task_commands.py'),
    )

    class Settings:
        def __init__(self):
            self.data = copy.deepcopy(data)
            self.write()

        @property
        def zs(self):
            return self.data['stations']['ZS']

        def write(self):
            path.write_text(yaml.safe_dump(self.data), encoding='utf-8')

    return Settings()


def test_real_settings_load():
    stations = task_commands._load_move_locations()
    assert list(stations)[:3] == ['LH1', 'LH2', 'ZS']
    assert task_commands._load_location_tag_id('ZS') == 'tag_2'
    assert task_commands._move_location(stations, 'ZS.work')[:2] == (0.0, 0.0)


@pytest.mark.parametrize('action', ['open', 'close'])
def test_door_alignment_and_hand_target_come_from_yaml(settings, action):
    target = settings.zs['work']['door'][action]
    target.update(tag='tag_11', base_align_xy=[0.5, -0.2], ee_offset_tag=[0.01, 0.07, 0.03])
    settings.write()

    (align,) = task_source.align_base_to_door('ZS', action).commands
    assert isinstance(align, CameraMoveToTagStep)
    assert align.object_id == 'tag_11'
    assert (align.desired_tag_x, align.desired_tag_y) == (0.5, -0.2)

    (hand,) = task_source.move_hand_to_door_handle('ZS', action).commands
    assert isinstance(hand, CameraLinearAbsoluteStep)
    assert hand.object_id == 'tag_11'
    # 태그 축 값을 그대로 쓴다
    assert hand.object_to_end_effector_position == pytest.approx((0.01, 0.07, 0.03))


def test_switch_press_and_retreat_come_from_yaml(settings):
    settings.zs['work']['switch'].update(
        tag='tag_12', button_offset_in_tag=[0.01, -0.02],
        approach_standoff_m=0.1, press_depth_m=0.03,
    )
    settings.write()

    commands = task_source.push_switch('ZS').commands
    approach, press, retreat = [c for c in commands if isinstance(c, CameraLinearAbsoluteStep)]
    assert approach.object_id == press.object_id == retreat.object_id == 'tag_12'
    # 접근 -> 누름(태그 z 로 press_depth_m 만큼 벽 쪽) -> 후퇴, 모두 태그 기준
    assert approach.object_to_end_effector_position == pytest.approx((0.01, -0.02, 0.1))
    assert press.object_to_end_effector_position == pytest.approx((0.01, -0.02, 0.07))
    assert retreat.object_to_end_effector_position == pytest.approx((0.01, -0.02, 0.1))
    assert [c.reuse_previous_observation for c in (approach, press, retreat)] == [False, True, True]
    # base 좌표 직선 이동은 쓰지 않는다
    assert not [c for c in commands if str(getattr(c, 'kind', '')).endswith('LINEAR_RELATIVE')]


def test_door_arc_uses_hinge_geometry(settings):
    # 벽 좌표: 경첩 -> 손잡이가 벽에서 로봇 쪽으로 0.2 m (wall_yaw_deg 90 = 벽이 base +y)
    settings.zs['work']['door'].update(hinge_to_handle_wall=[0.0, 0.0, 0.2], swing_deg=60.0)
    settings.zs['work']['wall_yaw_deg'] = 90.0
    settings.write()

    (opening,) = task_source.door_open_stream('ZS').commands
    (closing,) = task_source.door_close_stream('ZS').commands
    # 손잡이 -> 경첩 방향 = atan2(0.2, 0) = 90°, 반지름 0.2, 60° 만큼
    assert opening.waypoints == tuple(task_commands.arc_waypoints(0.2, 90.0, 60.0, 2.0, clockwise=True))
    assert closing.waypoints == tuple(task_commands.arc_waypoints(0.2, 30.0, 60.0, 2.0, clockwise=False))
    end = opening.waypoints[-1]
    assert math.hypot(end[0], end[1]) == pytest.approx(2 * 0.2 * math.sin(math.radians(30.0)))


def test_wall_yaw_rotates_hand_moves_and_door_arc(settings):
    # 벽이 base 정면(+x)에 있는 배치: 벽 좌표 z(벽 -> 로봇) = base -x, x(벽 따라) = base -y
    settings.zs['work']['wall_yaw_deg'] = 0.0
    settings.zs['work']['door'].update(hinge_to_handle_wall=[0.0, 0.0, 0.2], swing_deg=60.0)
    settings.write()

    moves = [
        c.values[:3] for c in task_source.move_hand_from_door_to_switch('ZS').commands
        if str(getattr(c, 'kind', '')).endswith('LINEAR_RELATIVE')
    ]
    assert moves == [pytest.approx((-0.056, 0.0, 0.0)), pytest.approx((0.0, -0.15, 0.0))]
    release = task_source.close_door('ZS').commands[-1]
    assert release.values[:3] == pytest.approx((-0.10, 0.0, 0.0))
    (opening,) = task_source.door_open_stream('ZS').commands
    # 손잡이 -> 경첩 방향 = 벽 쪽(+x) = 0°
    assert opening.waypoints == tuple(task_commands.arc_waypoints(0.2, 0.0, 60.0, 2.0, clockwise=True))


def test_current_layout_keeps_measured_base_moves(settings):
    # 지금 배치(wall_yaw_deg 90)에서는 예전 base 좌표 측정값과 같다
    moves = [
        c.values[:3] for c in task_source.move_hand_from_door_to_switch('ZS').commands
        if str(getattr(c, 'kind', '')).endswith('LINEAR_RELATIVE')
    ]
    assert moves == [(0.0, -0.056, 0.0), (0.15, 0.0, 0.0)]
    close = [
        c.values[:3] for c in task_source.close_door('ZS').commands
        if str(getattr(c, 'kind', '')).endswith('LINEAR_RELATIVE')
    ]
    assert close == [(0.0, -0.19, 0.0), (-0.15, 0.0, 0.0), (0.0, -0.10, 0.0)]


def test_head_pitch_comes_from_place(settings):
    settings.zs['points']['place']['head_pitch_deg'] = 40.0
    settings.write()
    work = task_commands._load_point_head_pitch('ZS.work')
    heads = [
        c for c in task_source.place_at_station('ZS').commands
        if getattr(c, 'group', None) == 'head'
    ]
    assert {c.kind.value for c in heads} == {'joint_absolute'}
    pitches = [c.values[1] for c in heads]
    move = task_commands._load_move_head_pitch_deg()
    # 문 작업(work) -> 경유 이동(move) -> 놓기(절대값 40) -> 복귀 이동(move) -> 문 닫기(work)
    assert pitches == [work, move, 40.0, move, work]


@pytest.mark.parametrize('change, message', [
    (lambda zs: zs['work']['door'].pop('close'), 'door'),
    (lambda zs: zs['work']['door'].update(swing_deg='90'), 'swing_deg'),
    (lambda zs: zs['work']['door']['open'].update(base_align_xy=[0.6]), 'base_align_xy'),
    (lambda zs: zs['work']['door']['close'].update(tag='door'), 'tag'),
    (lambda zs: zs['work']['switch'].update(press_depth_m=0.0), 'press_depth_m'),
    (lambda zs: zs['work'].pop('wall_yaw_deg'), 'wall_yaw_deg'),
    (lambda zs: zs['work']['door'].update(hinge_to_handle_wall=[0.1, 0.0]), 'hinge_to_handle_wall'),
    (lambda zs: zs['work']['switch'].update(tag='tag_2'), 'tag_2'),          # place 태그와 중복
    (lambda zs: zs['work']['door']['close'].update(tag='tag_6'), 'tag_6'),   # open 태그와 중복
    (lambda zs: zs.update(route=['work', 'via']), 'route'),          # place 로 끝나지 않음
    (lambda zs: zs['points']['place'].update(head_pitch_deg=None), 'head_pitch_deg'),
    (lambda zs: zs['points']['via'].update(head_pitch_deg='2'), 'head_pitch_deg'),
    (lambda zs: zs['place'].update(head_pitch_deg=47), 'head_pitch_deg'),     # 예전 위치는 거부
])
def test_bad_door_switch_settings_fail_on_reload(settings, change, message):
    change(settings.zs)
    settings.write()
    with pytest.raises(ValueError, match=message):
        task_source.open_door('ZS')


def test_disabled_or_unknown_station_is_rejected(settings):
    settings.zs['enabled'] = False
    settings.write()
    with pytest.raises(ValueError, match='disabled'):
        task_source.push_switch('ZS')
    with pytest.raises(ValueError, match='Unknown location'):
        task_source.push_switch('NOPE')
    with pytest.raises(ValueError, match='station name'):
        task_source.push_switch('ZS.work')
