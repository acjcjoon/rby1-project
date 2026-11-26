# Planner Scenario UI용 Task 정의
from __future__ import annotations

import math
from typing import Optional, Sequence

from .task_commands import (
    FIXED_SIDE,
    JointImpedance,
    RunnableTaskDefinition,
    Task,
    _load_move_head_pitch_deg,
    _load_point_head_pitch,
    _load_location_tag_id,
    _load_station,
    _load_move_locations,
    _load_tag_approach_offset,
    _move_location,
    arc_waypoints,
    end_effector_frame,
    list_sum,
    wrist_camera_frame,
)


# Task 작성 단위: 초, 미터, 도
# TCP 움직임 설정: [최소 시간, 선속도, 각속도, 가속도 비율]
# 기본 손이 없다: arm='right_arm' / 'left_arm' 을 반드시 넘긴다 (빠지면 Reload 시 오류).
# 그리퍼와 손목 카메라(camera_source / camera_frame)는 arm 에서 자동으로 정해진다.

# 운반(장비에서 집기/놓기) 손. 왼팔 고정.
TRANSFER_ARM = 'left_arm'
SWITCH_DOOR_ARM = 'right_arm'
SWITCH_DOOR_GRIPPER = 'right'
_GRIPPER_BY_ARM = {'right_arm': 'right', 'left_arm': 'left'}


def _gripper_for(arm: str) -> str:
    try:
        return _GRIPPER_BY_ARM[arm]
    except KeyError:
        raise ValueError(f"arm must be 'right_arm' or 'left_arm', got {arm!r}") from None


# 기존 이름 유지: 프리셋 이름을 가리킨다.
JS_SOFT_ARM = 'default'
JS_CONTACT_ARM = 'contact'


# ---------------------------------------------------------------------------
# 문 / 스위치 장비 설정 읽기 (settings.yaml stations.<장비>.work.door / switch)
# ---------------------------------------------------------------------------
DOOR_ACTIONS = ('open', 'close')

def _door(station: str, action: str) -> dict:
    """stations.<station>.work.door (open/close + 공통 경첩 기하)."""
    if action not in DOOR_ACTIONS:
        raise ValueError(f"door action must be 'open' or 'close', got {action!r}")
    door = (_load_station(station).get('work') or {}).get('door')
    if not isinstance(door, dict):
        raise ValueError(f'Station {station!r} has no door')
    return door

def _wall_to_base(station: str, offset_wall: Sequence[float]) -> tuple[float, float, float]:
    """벽 좌표 [x 벽 따라 수평, y 위, z 벽에서 로봇 쪽] -> base 좌표 [x, y, z].

    벽 방향은 stations.<station>.work.wall_yaw_deg (base XY, 0 = +x, 90 = +y).
    벽 축: z = 벽 -> 로봇 (-wall 방향), y = 위, x = y × z.
    """
    yaw = math.radians(float(_load_station(station)['work']['wall_yaw_deg']))
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = (float(v) for v in offset_wall)
    base = (x * s - z * c, -x * c - z * s, y)
    return tuple(round(v, 12) + 0.0 for v in base)

def _wall_move(station: str, offset_wall: Sequence[float]) -> tuple[float, ...]:
    """벽 좌표 이동량 -> linear_relative 용 base 이동 [dx, dy, dz, 0, 0, 0]."""
    return (*_wall_to_base(station, offset_wall), 0.0, 0.0, 0.0)

def _switch(station: str) -> dict:
    """stations.<station>.work.switch."""
    switch = (_load_station(station).get('work') or {}).get('switch')
    if not isinstance(switch, dict):
        raise ValueError(f'Station {station!r} has no switch')
    return switch

# ===========================================================================
# 1. 자세 — 관절 목표 하나로 끝나는 시작/준비 자세
# ===========================================================================
def ready_pose() -> Task:
    task = Task('ready_pose')
    task.whole_body_joint_absolute(
        torso= [0.000000, 45.000000, -89.999782, 44.999782, 0.000000, 0.000218],
        right_arm =[-0.000218, -5.000411, 0.000435, -119.999420, 0.000000, 30.000146, 0.000220],
        left_arm  = [0.000000, 5.000193, -0.000218, -119.999637, 0.000000, 29.999707, -0.000220],
        joint_motion=[5.0, 1.0, 1.0],
    )
    return task.build()

def _grip_torso(side: str) -> tuple:
    """허리 관절 6개 값: 높이(torso_0~4)는 공통, yaw(torso_5)만 side 로 고른다."""
    # height = (0.065048, 45.375276, -90.959835, 45.153373, 0.026976)  # torso_0~4
    height = [0.058956, 39.346066, -78.884882, 39.102191, 0.031110]
    yaw_deg = {'front': 0.0, 'left': 90.0, 'right': -90.0}
    try:
        yaw = yaw_deg[side]
    except KeyError:
        raise ValueError(f"torso side must be 'front', 'left' or 'right', got {side!r}") from None
    return (*height, yaw)

def _head_target(head_pitch_deg: Optional[float]) -> Optional[tuple]:
    """머리 관절 2개 값 (yaw 0, pitch). None 이면 머리를 움직이지 않는다."""
    return None if head_pitch_deg is None else (0.0, float(head_pitch_deg))

def torso_align(
    side: str = 'front',
    joint_motion: Sequence[float] = (5.0, 2.0, 3.0),
    *,
    head_pitch_deg: Optional[float] = None,
) -> Task:
    """허리만 정렬한다 (팔은 그대로, 물체를 든 채 회전 가능).

    side: front / left / right. joint_motion: [최소 시간 s, 속도, 가속도].
    head_pitch_deg: 주면 머리 pitch 도 같은 스텝에서 같이 맞춘다
    (머리 속도/가속도는 control 이 머리 상한으로 낮춘다).
    """
    task = Task(f'torso_align({side})')
    task.whole_body_joint_absolute(
        torso=_grip_torso(side), head=_head_target(head_pitch_deg), joint_motion=joint_motion,
    )
    return task.build()

def object_gripping_initial_pose(
    torso: Optional[str] = None,
    joint_motion: Optional[Sequence[float]] = None,
    *,
    head_pitch_deg: Optional[float] = None,
) -> Task:
    """양팔 파지 준비 자세.

    torso=None 이면 팔만 움직인다 (허리 그대로).
    torso='front' / 'left' / 'right' 이면 허리와 팔을 한 스텝에서 같이 움직인다.
    joint_motion: [최소 시간 s, 속도, 가속도]. 생략 시 팔만 (1.5, 2.0, 3.0),
    허리 포함 (5.0, 2.0, 3.0).
    head_pitch_deg: 주면 머리 pitch 도 같은 스텝에서 같이 맞춘다
    (머리 속도/가속도는 control 이 머리 상한으로 낮춘다).
    """
    right_arm = (-38.280283, -61.948097, 45.740109, -95.376122, 15.278687, 36.629077, 83.445557)
    left_arm = (-37.023491, 61.226480, -44.046256, -95.222313, -16.824243, 35.951221, -84.381812)
    if joint_motion is None:
        joint_motion = (1.5, 2.0, 3.0) if torso is None else (5.0, 2.0, 3.0)

    name = 'object_gripping_initial_pose' if torso is None else f'object_gripping_initial_pose({torso})'
    task = Task(name)
    task.whole_body_joint_absolute(
        torso=None if torso is None else _grip_torso(torso),
        right_arm=right_arm, left_arm=left_arm, head=_head_target(head_pitch_deg),
        joint_motion=joint_motion,
    )
    return task.build()

def pose_before_base_movement(side: str) -> Task:
    """door_switch 작업 위치로 들어가기 전 자세: 허리 side + 오른팔을 스위치 탐색에 방해되지 않게.

    side: 허리 방향 (front / left / right) = 들어갈 작업 위치(work)의 side.
    직전 허리 방향이 다르면(경유 지점이 fixed) 이 스텝에서 허리도 돌므로 5 s.
    """
    task = Task(f'pose_before_base_movement({side})')
    # 오른팔: object_gripping_initial_pose 에서 TCP 를 (-y 0.12 -> +x 0.07) 옮긴 자세
    # (허리 기준, 손 방향 동일, SDK 모델 IK 계산값. 계산은 허리 left 에서). 이전 값:
    # right_arm=[-16.485584, -81.212678, 36.201365, -117.319844, 5.521069, 37.464697, 72.280151]
    # 왼팔은 object_gripping_initial_pose 의 왼팔과 같다.
    task.whole_body_joint_absolute(
        torso=_grip_torso(side),
        right_arm=[-8.030944, -85.718373, 30.622439, -119.996443, 3.002524, 34.710418, 68.873201],
        left_arm=(-37.023491, 61.226480, -44.046256, -95.222313, -16.824243, 35.951221, -84.381812),
        joint_motion=(5.0, 2.0, 3.0),
    )
    return task.build()

def door_handle_grasp_pose() -> Task:
    task = Task('door_handle_grasp_pose')
    task.whole_body_joint_absolute(
        torso=[0.060044, 43.042258, -86.290755, 42.812307, 0.030457, 90.000870],
        right_arm=[-40.690314, -67.544163, 41.962987, -86.445651, -75.910693, -3.345776, -23.483716],
        left_arm=(-37.023491, 61.226480, -44.046256, -95.222313, -16.824243, 35.951221, -84.381812),
        joint_motion=(5.0, 1.0, 1.0),
    )
    return task.build()

def search_pose() -> Task:
    task = Task('search_pose')
    task.whole_body_joint_absolute(
        torso=[0.0, 0.0, 0.0, 0.0, 0.0, 90.0],   # 허리 쭉 편 자세 (키 최대). yaw 90 = left (ZS 작업 위치 방향)
        right_arm = [-2.639050, -48.778294, 9.475941, -105.823940, -48.357757, 5.870969, -18.970059],
        joint_motion=(5.0, 1.0, 1.0),             # 허리가 크게 움직이니 최소 시간 2.5 -> 5 s
    )
    return task.build()

def search_switch_pose() -> Task:
    task = Task('search_switch_pose')
    # base 180° 회전 배치: torso_5 를 -89.999130 -> +90.000870 (상체는 이전과 같은 방향)
    task.whole_body_joint_absolute(
        # torso=[0.060044, 43.042258, -86.290755, 42.812307, 0.030457, 90.000870],
        torso=[0.0, 0.0, 0.0, 0.0, 0.0, 90.0],
        right_arm = [-32.177210, -57.932216, 44.146889, -91.228664, -64.357215, -9.066141, -33.238335],        
        left_arm=(-37.023491, 61.226480, -44.046256, -95.222313, -16.824243, 35.951221, -84.381812),
        joint_motion=(2.5, 1.0, 1.0),
    )
    return task.build()

def head_pitch_pose(pitch_deg: float, joint_motion: Sequence[float] = (1.5, 1.5, 1.5)) -> Task:
    """머리를 pitch 절대각(deg, + = 숙임)으로 맞춘다. yaw(head_0)는 0.

    joint_motion: [최소 시간 s, 속도, 가속도]. 모든 머리 이동이 이 기본값을 쓴다.
    각도 값은 settings.yaml head_pitch_deg.move /
    stations.<장비>.points.<지점>.head_pitch_deg.
    """
    task = Task(f'head_pitch_pose({float(pitch_deg):g})')
    task.joint_absolute('head', [0.0, float(pitch_deg)], joint_motion)
    return task.build()

# ===========================================================================
# 2. 기본 동작 — base 이동, 태그 기준 손 접근, 문 원호
# ===========================================================================
def turn_and_move(
    start_point: str,
    end_point: str,
    *,
    move_head: bool = True,
    current_side: Optional[str] = None,
) -> RunnableTaskDefinition:
    """허리 방향이 바뀌면 먼저 맞추고(머리 이동도 같은 스텝) start -> end 로 base 를 직선 이동한다.

    도착 지점 side 가 fixed 면 허리를 그대로 둔다.
    current_side: 지금 허리 방향. 생략하면 start 지점의 side (fixed 지점에서 출발하면 필수).
    """
    locations = _load_move_locations()

    start_x, start_y, start_side = _move_location(locations, start_point)
    end_x, end_y, side = _move_location(locations, end_point)
    if current_side is not None:
        start_side = current_side
    if start_side == FIXED_SIDE:
        raise ValueError(
            f'turn_and_move from {start_point!r} (side: fixed) needs current_side'
        )
    head_pitch = _load_move_head_pitch_deg() if move_head else None

    task = Task('turn_and_move')
    if side != FIXED_SIDE and start_side != side:
        task.extend(object_gripping_initial_pose(side, head_pitch_deg=head_pitch))
    elif head_pitch is not None:
        task.extend(head_pitch_pose(head_pitch))

    task.move_to(end_x - start_x, end_y - start_y, 0.0, timeout_sec=15.0)
    return task.build()

def align_base_to_door(
    station: str = 'ZS',
    action: str = 'open',
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
) -> RunnableTaskDefinition:
    """D435 로 문 손잡이 태그(open/close)를 보고 측정해 둔 상대 위치가 되도록 base 를 이동한다."""
    target = _door(station, action)[action]
    x, y = target['base_align_xy']
    task = Task(f'align_base_to_door({action})')
    task.move_base_to_detected_tag(
        camera_source='d435', object_id=target['tag'],
        desired_tag_x=float(x),
        desired_tag_y=float(y),
        desired_tag_frame='d435_camera_center',
        threshold_x_minus=threshold_x_minus, threshold_y_plus=threshold_y_plus,
        relative_yaw=0.0, detection_timeout_sec=5.0, navigation_timeout_sec=30.0,
        max_translation_m=0.8,
    )
    return task.build()

def align_base_to_tag(
    object_id: str,
    *,
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
    arm: str,
    camera_frame: Optional[str] = None,
    shift_toward_ee_m: float = 0.002,
) -> RunnableTaskDefinition:
    """D435 로 태그를 보고 base 를 정렬한다 (태그가 arm 손목 카메라 아래에 오도록).

    arm 필수: 정렬 목표가 그 팔의 손목 카메라 프레임이다. 뒤따르는
    pick_up/put_down_object_in_place 에는 같은 태그와 같은 arm 을 넘길 것.
    shift_toward_ee_m: 목표점을 카메라 중심에서 그 팔 end effector 쪽으로
    base XY 에서 이만큼 옮긴다 (m, 기본 2 mm, 0 이면 카메라 중심 그대로).
    """
    camera_frame = wrist_camera_frame(arm, camera_frame)
    task = Task(f'align_base_to_tag_{object_id}')
    task.move_base_to_detected_tag(
        camera_source='d435', object_id=object_id,
        desired_tag_x=0.0, desired_tag_y=0.0, desired_tag_frame=camera_frame,
        threshold_x_minus=threshold_x_minus, threshold_y_plus=threshold_y_plus,
        relative_yaw=0.0, detection_timeout_sec=5.0, navigation_timeout_sec=30.0,
        max_translation_m=0.8,
        shift_toward_frame=end_effector_frame(arm), shift_toward_m=shift_toward_ee_m,
    )
    return task.build()

def move_hand_to_door_handle(
    station: str = 'ZS',
    action: str = 'open',
    tcp_motion: Sequence[float] = (1.5, 0.2, 0.6, 0.5),
) -> RunnableTaskDefinition:
    """D435 로 손잡이 태그(open/close)를 보고 오른손 TCP 를 ee_offset_tag 지점으로 옮긴다.

    ee_offset_tag = 태그 축 기준 손 위치 [x 벽 따라 수평, y 위, z 벽에서 로봇 쪽] m.
    태그 기준이라 로봇이 문의 어느 쪽에 서든 같은 값을 쓴다.
    tcp_motion: [최소 시간 s, 선속도, 각속도, 가속도 비율].
    """
    # 태그 -> 손 자세: 태그 z 축 기준 +90° (손목 롤). 손가락(손 -z)은 벽을 향하고 손 x 축이 위.
    tag_to_ee_orientation_xyzw = (0.0, 0.0, 0.7071068, 0.7071068)
    target = _door(station, action)[action]
    task = Task(f'move_hand_to_door_handle({action})')
    task.camera_linear_absolute(
        SWITCH_DOOR_ARM, target['tag'], tcp_motion,
        camera_source='d435',
        detection_timeout_sec=5.0,
        log_target_and_actual=True,
        object_to_end_effector_position=tuple(float(v) for v in target['ee_offset_tag']),
        object_to_end_effector_orientation_xyzw=tag_to_ee_orientation_xyzw,
    )
    return task.build()

def move_hand_to_tag(
    object_id: str,
    approach_offset: Sequence[object],
    contact_offset: Sequence[object],
    tcp_motion: Sequence[object],
    *,
    arm: str,
    camera_source: Optional[str] = None,
) -> RunnableTaskDefinition:
    approach = tuple(float(value) for value in approach_offset)
    contact = tuple(float(value) for value in contact_offset)

    if len(approach) != 3 or len(contact) != 3:
        raise ValueError('approach/contact offsets must contain 3 numbers')
    tag_offset = _load_tag_approach_offset(object_id)
    approach = tuple(value + offset for value, offset in zip(approach, tag_offset))
    contact = tuple(value + offset for value, offset in zip(contact, tag_offset))
    tag_to_ee_orientation_xyzw = (0.0, 0.0, 0.0, 1.0)  # 태그 -> 손 자세: 태그와 같은 방향 (회전 없음).

    task = Task(f'move_{object_id}_approach_contact')
    task.delay(500)
    task.camera_linear_absolute(
        arm, object_id, tcp_motion,
        camera_source=camera_source,
        detection_timeout_sec=3.0,
        yaw_only=True,
        yaw_symmetry_deg=180.0,
        log_target_and_actual=True,
        object_to_end_effector_position=approach,
        object_to_end_effector_orientation_xyzw=tag_to_ee_orientation_xyzw,
    )
    task.camera_linear_absolute(
        arm, object_id, tcp_motion,
        camera_source=camera_source,
        detection_timeout_sec=3.0,
        reuse_previous_observation=True,
        yaw_only=True,
        yaw_symmetry_deg=180.0,
        log_target_and_actual=True,
        object_to_end_effector_position=contact,
        object_to_end_effector_orientation_xyzw=tag_to_ee_orientation_xyzw,
    )
    return task.build()

def _door_arc(station: str) -> tuple[float, float, float]:
    """(반지름 m, 열기 시작 경첩 방향 deg, swing deg)."""
    door = _door(station, 'open')
    hx, hy, _ = _wall_to_base(station, door['hinge_to_handle_wall'])
    radius = math.hypot(hx, hy)
    open_direction = math.degrees(math.atan2(-hy, -hx)) % 360.0
    return radius, open_direction, float(door['swing_deg'])

def door_open_stream(
    station: str = 'ZS',
    open_angle_deg: Optional[float] = None,
    step_deg: float = 2.0,
    yaw_follows_door: bool = False,
    path_motion: Sequence[float] = (10.0, 0.05, 0.3),
) -> Task:
    """손잡이를 잡은 오른손으로 경첩 중심 원호를 시계 방향으로 따라 문을 연다.

    path_motion: [전체 시간 s, 선속도 상한 m/s, 각속도 상한 rad/s].
    """
    radius, open_direction, swing = _door_arc(station)
    task = Task('door_open_stream')
    task.linear_path_relative(
        'right_arm',
        arc_waypoints(
            radius,
            open_direction,
            swing if open_angle_deg is None else open_angle_deg,
            step_deg,
            clockwise=True,
            yaw_follows_arc=yaw_follows_door,
        ),
        path_motion,
    )
    return task.build()

def door_close_stream(
    station: str = 'ZS',
    close_angle_deg: Optional[float] = None,
    step_deg: float = 2.0,
    yaw_follows_door: bool = False,
    path_motion: Sequence[float] = (10.0, 0.05, 0.3),
) -> Task:
    """swing 만큼 열린 문을 닫는다: door_open_stream 경로를 반시계로 되감는다.

    path_motion: [전체 시간 s, 선속도 상한 m/s, 각속도 상한 rad/s].
    """
    radius, open_direction, swing = _door_arc(station)
    task = Task('door_close_stream')
    task.linear_path_relative(
        'right_arm',
        arc_waypoints(
            radius,
            (open_direction - swing) % 360.0,
            swing if close_angle_deg is None else close_angle_deg,
            step_deg,
            clockwise=False,
            yaw_follows_arc=yaw_follows_door,
        ),
        path_motion,
    )
    return task.build()

# ===========================================================================
# 3. 장비 동작 — 집기/놓기, 문 열기, 스위치 누르기
# ===========================================================================
def _pick_place_offsets() -> tuple[list, list, float]:
    """제자리 파지/배치 공용 (접근 지점, 접촉 지점, 상승 거리)."""
    contact = [0.0, 0.0, 0.065]  # 태그 기준 접촉 지점 (+ settings.yaml 태그 오프셋)
    lift_m = 0.075               # 접근 높이 = 접촉 + 이 값, 파지/배치 후 상승 거리
    return list_sum(contact, [0.0, 0.0, lift_m]), contact, lift_m

def pick_up_object_in_place(
    object_id: str,
    *,
    arm: str,
    camera_source: Optional[str] = None,
    approach_motion: Sequence[float] = (1.5, 1.0, 1.0, 1.0),
    lift_motion: Sequence[float] = (1.0, 1.0, 1.0, 1.0),
) -> RunnableTaskDefinition:
    """base 이동 없이 현재 자세에서 손목 카메라로 태그를 보고 그리퍼 열기 -> 접근·접촉 -> 파지 -> 상승.

    approach_motion / lift_motion: [최소 시간 s, 선속도 m/s, 각속도 rad/s, 가속도 비율].
    """
    gripper = _gripper_for(arm)
    approach_offset, contact_offset, lift_m = _pick_place_offsets()
    task = Task(f'pick_up_object_in_place_{object_id}')
    task.open_gripper(gripper)
    task.extend(move_hand_to_tag(
        object_id, approach_offset, contact_offset, approach_motion,
        arm=arm, camera_source=camera_source,
    ))
    task.close_gripper(gripper)
    task.linear_relative(arm, (0.0, 0.0, lift_m, 0.0, 0.0, 0.0), lift_motion)
    return task.build()

def put_down_object_in_place(
    object_id: str,
    *,
    arm: str,
    camera_source: Optional[str] = None,
    approach_motion: Sequence[float] = (3.0, 1.0, 1.0, 1.0),
    lift_motion: Sequence[float] = (3.0, 1.0, 1.0, 1.0),
) -> RunnableTaskDefinition:
    """base 이동 없이 현재 자세에서 손목 카메라로 태그를 보고 접근·접촉 -> 놓기 -> 상승.

    approach_motion / lift_motion: [최소 시간 s, 선속도 m/s, 각속도 rad/s, 가속도 비율].
    """
    gripper = _gripper_for(arm)
    approach_offset, contact_offset, lift_m = _pick_place_offsets()
    task = Task(f'put_down_object_in_place_{object_id}')
    task.extend(move_hand_to_tag(
        object_id, approach_offset, contact_offset, approach_motion,
        arm=arm, camera_source=camera_source,
    ))
    task.open_gripper(gripper)
    task.linear_relative(arm, (0.0, 0.0, lift_m, 0.0, 0.0, 0.0), lift_motion)
    return task.build()

def open_door(station: str = 'ZS') -> RunnableTaskDefinition:
    """그리퍼 열기 -> 탐색 자세 -> 손잡이 기준 base 정렬 -> 손잡이로 손 이동 -> 잡기 -> 원호로 열기."""
    task = Task('open_door')
    task.open_gripper('right')
    task.extend(search_pose())
    task.extend(align_base_to_door(station, 'open'))
    task.extend(move_hand_to_door_handle(station, 'open'))
    task.close_gripper('right')
    task.extend(door_open_stream(station))
    return task.build()

def open_door_at_work(station: str = 'ZS') -> RunnableTaskDefinition:
    """작업 위치(work)에서 문 열기만 단독 실행한다 (Scenario UI 용).

    _door_switch_visit 처럼 머리를 work 지점 pitch 로 맞춘 뒤 open_door 를 한다.
    base_align_xy 가 그 머리 pitch 기준이기 때문이다. 로봇은 work 지점 근처에서 시작한다.
    """
    task = Task(f'open_door({station})')
    task.extend(head_pitch_pose(_load_point_head_pitch(f'{station}.work')))
    task.extend(open_door(station))
    return task.build()

def push_switch(
    station: str = 'ZS',
    approach_motion: Sequence[float] = (3.0, 0.1, 0.3, 0.5),
    press_motion: Sequence[float] = (3.0, 0.3, 0.3, 0.5),
) -> RunnableTaskDefinition:
    """search_switch_pose 에서 D405 로 스위치 태그를 보고 접근한 뒤 직선으로 누르고 돌아온다.

    approach_motion / press_motion: [최소 시간 s, 선속도 m/s, 각속도 rad/s, 가속도 비율].
    """
    # 태그 -> 손 자세: 태그 z 축 기준 +90° (손목 롤). 손가락(손 -z)은 벽을 향하고 손 x 축이 위.
    tag_to_ee_orientation_xyzw = (0.0, 0.0, 0.7071068, 0.7071068)
    switch = _switch(station)
    bx, by = (float(v) for v in switch.get('button_offset_in_tag', (0.0, 0.0)))
    standoff = float(switch['approach_standoff_m'])
    pressed = standoff - float(switch['press_depth_m'])
    task = Task('push_switch')
    task.extend(search_switch_pose())
    task.close_gripper(SWITCH_DOOR_GRIPPER)
    # 접근 -> 누름 -> 후퇴: 모두 태그 기준 목표. 누름/후퇴는 접근 때의 태그 관측을 재사용한다.
    for z, motion, reuse in (
        (standoff, approach_motion, False),
        (pressed, press_motion, True),
        (standoff, press_motion, True),
    ):
        task.camera_linear_absolute(
            SWITCH_DOOR_ARM, switch['tag'], motion,
            detection_timeout_sec=5.0,
            reuse_previous_observation=reuse,
            log_target_and_actual=True,
            object_to_end_effector_position=(bx, by, z),
            object_to_end_effector_orientation_xyzw=tag_to_ee_orientation_xyzw,
        )
    # 탐색 자세로 복귀
    task.extend(search_switch_pose())
    return task.build()

def move_hand_from_door_to_switch(
    station: str = 'ZS',
    hand_motion: Sequence[float] = (2.0, 0.1, 0.3, 0.5),
) -> RunnableTaskDefinition:
    """문을 연 뒤: 손잡이를 놓고 오른손을 스위치 쪽으로 옮긴다 (벽 좌표 직선 이동, ZS 측정값).

    hand_motion: [최소 시간 s, 선속도 m/s, 각속도 rad/s, 가속도 비율].
    """
    task = Task('move_hand_from_door_to_switch')
    task.open_gripper(SWITCH_DOOR_GRIPPER)
    task.linear_relative(SWITCH_DOOR_ARM, _wall_move(station, (0.0, 0.0, 0.056)), hand_motion)  # 벽에서 떼기
    task.close_gripper(SWITCH_DOOR_GRIPPER)
    task.linear_relative(SWITCH_DOOR_ARM, _wall_move(station, (0.15, 0.0, 0.0)), hand_motion)   # 벽 따라 스위치 쪽
    return task.build()

def close_door(
    station: str = 'ZS',
    hand_motion: Sequence[float] = (2.0, 0.1, 0.3, 0.5),
) -> RunnableTaskDefinition:
    """스위치를 누른 뒤(search_switch_pose): 닫기 손잡이로 손을 옮겨 잡고 원호로 닫은 뒤 놓는다.

    base 는 align_base_to_door(station, 'close') 로 맞춰져 있어야 한다.
    hand_motion: 스위치 -> 손잡이, 손 빼기 직선 이동 [최소 시간 s, 선속도 m/s, 각속도 rad/s, 가속도 비율].
    """
    task = Task('close_door')
    # 스위치 -> 닫기 손잡이 쪽 (벽 좌표): 벽에서 떼고 벽 따라 이동
    task.linear_relative(SWITCH_DOOR_ARM, _wall_move(station, (0.0, 0.0, 0.19)), hand_motion)
    task.linear_relative(SWITCH_DOOR_ARM, _wall_move(station, (-0.15, 0.0, 0.0)), hand_motion)
    task.open_gripper(SWITCH_DOOR_GRIPPER)
    task.extend(move_hand_to_door_handle(station, 'close'))
    task.close_gripper(SWITCH_DOOR_GRIPPER)
    task.extend(door_close_stream(station))
    # 손잡이를 놓고 벽에서 손을 뺀다 (벽 좌표 z 0.10 m). 이후 허리 회전/base 이동이 이어져도 안전하도록.
    # 실기 확인 전 값.
    task.open_gripper(SWITCH_DOOR_GRIPPER)
    task.linear_relative(SWITCH_DOOR_ARM, _wall_move(station, (0.0, 0.0, 0.10)), hand_motion)
    return task.build()

# ===========================================================================
# 4. 장비 단위 — 장비 사이 이동, 장비에서 집기/놓기 (장비 kind 에 따라 분기)
# ===========================================================================
def _entry_point(station: str) -> str:
    """장비에 들어가고 나오는 지점 참조. plain = 집기/놓기 지점, door_switch = 작업 위치(work)."""
    config = _load_station(station)
    if config['kind'] == 'door_switch':
        return f'{station}.work'
    return f"{station}.{config['place']['point']}"

def go_to(from_station: str, to_station: str) -> RunnableTaskDefinition:
    """장비 출입 지점 사이를 직선 이동한다 (허리 방향이 다르면 먼저 맞춘다)."""
    task = Task(f'go_to({from_station},{to_station})')
    if from_station != to_station:
        task.extend(turn_and_move(_entry_point(from_station), _entry_point(to_station)))
    return task.build()

def align_at_station(station: str, *, set_torso: bool = True) -> RunnableTaskDefinition:
    """장비의 집기/놓기 지점에서 base 정렬만 한다 (집기/놓기 없음, 왼팔 손목 카메라 기준).

    허리 방향(front/left/right)마다 함수를 따로 둘 필요 없다: 정렬 목표(손목 카메라 위치)는
    실행 시점의 TF(base <- 손목 카메라)로 읽으므로 지금 허리 yaw 가 그대로 반영된다.
    단, 정렬 전에 허리가 그 지점 side 로 돌아가 있어야 한다 (D435 가 태그를 보고
    손목 카메라가 태그 위로 갈 수 있도록).

    set_torso=True: 그 지점 side 로 허리+팔 파지 준비 자세 + 머리 pitch 를 한 스텝에서 맞춘다
    (Scenario 에서 단독 실행용). False: 머리만 맞춘다 (허리는 이미 맞춰져 있다고 본다).
    """
    ref = f"{station}.{_load_station(station)['place']['point']}"
    _, _, side = _move_location(_load_move_locations(), ref)
    head_pitch = _load_point_head_pitch(ref)
    task = Task(f'align_at_station({station})')
    if set_torso and side != FIXED_SIDE:
        task.extend(object_gripping_initial_pose(side, head_pitch_deg=head_pitch))
    else:
        task.extend(head_pitch_pose(head_pitch))
    task.extend(align_base_to_tag(_load_location_tag_id(ref), arm=TRANSFER_ARM))
    return task.build()

def _pick_or_place(station: str, action: str) -> RunnableTaskDefinition:
    """장비의 집기/놓기 지점에서 집거나 놓는다.

    머리: 집기/놓기 지점의 head_pitch_deg 로 맞춘다 (UI 에서 바로 실행해도 같은 각도에서 태그를 본다).
    머리 속도는 head_pitch_pose 기본값 하나로 관리한다.
    """
    task = Task(f'{action}_at_station({station})')
    task.extend(align_at_station(station, set_torso=False))
    tag = _load_location_tag_id(station)
    if action == 'pick':
        task.extend(pick_up_object_in_place(tag, arm=TRANSFER_ARM))
    else:
        task.extend(put_down_object_in_place(tag, arm=TRANSFER_ARM))
    task.extend(object_gripping_initial_pose())
    return task.build()

def _door_switch_visit(
    station: str,
    action: str,
    hand_motion: Sequence[float] = (2.0, 0.1, 0.3, 0.5),
) -> RunnableTaskDefinition:
    """door_switch 장비: 문 열기 -> 스위치 -> route 따라 이동 -> 집기/놓기 -> 복귀 -> 스위치 -> 문 닫기.

    작업 위치(work)에서 시작해 작업 위치에서 끝난다. 집기/놓기 손은 왼팔, 문/스위치는 오른팔.
    hand_motion: 스위치를 누른 뒤 손 빼기 직선 이동 [최소 시간 s, 선속도 m/s, 각속도 rad/s, 가속도 비율].
    """
    config = _load_station(station)
    route = [f'{station}.{point}' for point in config['route']]
    route_side = {f'{station}.{point}': config['points'][point]['side'] for point in config['route']}
    work_pitch = _load_point_head_pitch(f'{station}.work')
    move_pitch = _load_move_head_pitch_deg()

    task = Task(f'{action}_at_station({station})')

    task.extend(head_pitch_pose(work_pitch))
    task.extend(open_door(station))
    task.extend(move_hand_from_door_to_switch(station))
    task.extend(push_switch(station))
    # 스위치를 누른 뒤 오른손을 벽에서 뺀다 (벽 좌표 z 0.25 m).
    task.linear_relative(SWITCH_DOOR_ARM, _wall_move(station, (0.0, 0.0, 0.25)), hand_motion)

    # 허리 방향을 따라가며 이동한다: side: fixed 경유 지점에서는 허리를 그대로 둔다.
    side = route_side[route[0]]
    task.extend(head_pitch_pose(move_pitch))
    for start, end in zip(route, route[1:]):
        task.extend(turn_and_move(start, end, move_head=False, current_side=side))
        if route_side[end] != FIXED_SIDE:
            side = route_side[end]

    task.extend(_pick_or_place(station, action))

    task.extend(head_pitch_pose(move_pitch))
    back = route[::-1]
    for start, end in zip(back, back[1:]):
        if end == back[-1]:
            # 작업 위치로 들어가기 전: 오른손을 스위치 탐색에 방해되지 않는 자세로 둔다.
            # 허리도 작업 위치의 side 로 맞추므로(경유 지점이 fixed 면 여기서 회전)
            # 마지막 구간은 허리가 그대로다.
            side = route_side[end]
            task.extend(pose_before_base_movement(side))
        task.extend(turn_and_move(start, end, move_head=False, current_side=side))
        if route_side[end] != FIXED_SIDE:
            side = route_side[end]

    task.extend(head_pitch_pose(work_pitch))
    task.extend(align_base_to_door(station, 'close'))
    task.extend(push_switch(station))
    task.extend(close_door(station))
    return task.build()

def pick_at_station(station: str) -> RunnableTaskDefinition:
    """장비에서 물체를 집는다. 출입 지점에서 시작해 출입 지점에서 끝난다."""
    if _load_station(station)['kind'] == 'door_switch':
        return _door_switch_visit(station, 'pick')
    return _pick_or_place(station, 'pick')

def place_at_station(station: str) -> RunnableTaskDefinition:
    """장비에 물체를 놓는다. 출입 지점에서 시작해 출입 지점에서 끝난다."""
    if _load_station(station)['kind'] == 'door_switch':
        return _door_switch_visit(station, 'place')
    return _pick_or_place(station, 'place')

# ===========================================================================
# 5. 최상위 — 장비 사이 운반
# ===========================================================================
def transfer(source: str, destination: str, *, start: Optional[str] = None) -> RunnableTaskDefinition:
    """source 장비에서 집어 destination 장비에 놓는다 (왼팔).

    start: 로봇이 지금 있는 장비. 생략하면 이미 source 출입 지점에 있다고 본다.
    시작할 때 현재 지점의 허리 방향으로 파지 준비 자세를 잡는다.
    """
    if source == destination:
        raise ValueError(f'source and destination must differ, got {source!r}')
    here = source if start is None else start
    _, _, side = _move_location(_load_move_locations(), _entry_point(here))
    task = Task(f'transfer({source},{destination})')
    task.extend(object_gripping_initial_pose(side))
    task.extend(go_to(here, source))
    task.extend(pick_at_station(source))
    task.extend(go_to(source, destination))
    task.extend(place_at_station(destination))
    return task.build()

# ===========================================================================
# 6. 데모
# ===========================================================================
def transfer_demo() -> RunnableTaskDefinition:
    """LH1_1 -> ZS 운반 뒤 ZS 에서 LH1_2 로 돌아와 LH1_2 -> LH2 운반 (왼팔). 로봇은 LH1_1 에서 시작한다."""
    task = Task('transfer_demo(LH1_1->ZS, LH1_2->LH2)')
    task.extend(transfer('LH1_1', 'ZS'))
    task.extend(transfer('LH1_2', 'LH2', start='ZS'))  # ZS -> LH1_2 이동부터
    return task.build()

def lh1_to_lh2_demo() -> RunnableTaskDefinition:
    """LH1_1 에서 plate 를 집어 LH1_2 로 이동해 놓는다 (왼팔). 로봇은 LH1_1 에서 시작한다."""
    task = Task('lh1_to_lh2_demo(LH1_1->LH1_2)')
    task.extend(transfer('LH1_1', 'LH1_2'))
    return task.build()

def impedance_demo_joint() -> Task:
    """오른팔 팔꿈치를 관절 임피던스로 10도 굽혔다가 위치제어로 되돌린다.

    움직이는 동안 손으로 밀면 유연하게 밀린다(토크 상한 10 Nm). 완료 후 re-anchor
    되므로 반복 실행하면 팔꿈치 각도가 조금씩 이동할 수 있다.
    """
    task = Task('impedance_demo_joint')
    task.joint_relative(
        'right_arm',
        [0.0, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0],
        [3.0, 0.5, 0.5],
        impedance=JS_SOFT_ARM,
    )
    task.delay(1000)
    task.joint_relative(
        'right_arm',
        [0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0],
        [3.0, 0.5, 0.5],
    )
    return task.build()

def impedance_demo_press() -> Task:
    """현재 자세에서 base -z로 3 cm 접촉 이동 후 5 cm(3 + 여유 2) 올라온다.

    말단 아래 1~2 cm에 부드러운 대상(완충재, 버튼)을 두고 실행한다. 접촉 스텝은
    control 노드가 IK로 푼 관절 목표를 JS_CONTACT_ARM 으로 추종하므로 궤적을 따라
    내려가며, 막히면 토크 상한만큼만 누르고 hold_time 뒤 re-anchor 된다. 후퇴는
    위치제어다.
    """
    task = Task('impedance_demo_press')
    task.press(
        'right_arm',
        [0.0, 0.0, -0.03],
        [2.0, 0.05, 0.2, 0.3],
        impedance=JS_CONTACT_ARM,
    )
    return task.build()

# 양팔 흔들기 (인사)
def wave_both_arms(count: int = 3) -> RunnableTaskDefinition:
    """그리퍼를 닫고 정면을 본 채 양팔을 든 뒤 ``count`` 번 왼쪽-오른쪽으로 흔든다.

    왼쪽 스윙 동안 몸이 일어서며 그리퍼를 펴고, 오른쪽 스윙 동안 ready_pose 높이로
    내려오며 그리퍼를 오므린다. 닫힌 채 중립 자세(정면, ready_pose 높이)에서 끝난다.
    """
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('count must be a positive integer')
    # 토르소: DOWN = ready_pose 높이에서 정면(yaw 0), UP = 약 4.5 cm 일어선 자세.
    torso_down = [0.065048, 45.375276, -90.959835, 45.153373, 0.026976, 0.0]
    torso_up = [0.065048, 40.0, -80.0, 39.568814, 0.026976, 0.0]
    neutral = {
        'torso': torso_down,
        'right_arm': [-71.65, -31.12, -29.76, -101.88, -17.24, 3.04, 0.15],
        'left_arm': [-65.01, 31.67, 29.12, -106.92, 17.47, 6.39, 0.48],
    }
    left_up = {
        'torso': torso_up,
        'right_arm': [-71.65, -31.12, -46.56, -97.27, -17.24, 3.04, 0.15],
        'left_arm': [-66.78, 31.71, 0.90, -117.12, 17.46, 6.35, 0.44],
    }
    right_down = {
        'torso': torso_down,
        'right_arm': [-71.65, -31.12, -1.54, -112.08, -17.24, 3.04, 0.15],
        'left_arm': [-66.77, 31.72, 45.92, -102.31, 17.50, 6.35, 0.52],
    }
    # 관절 이동 설정: [최소 시간 s, 속도 rad/s, 가속도 rad/s^2]
    # 속도는 관절 모델 한계(2.09 rad/s), 가속도는 토르소 한계 5 rad/s^2 를 넘으면 거부된다.
    approach_motion = (4.0, 1.0, 1.0)
    swing_motion = (1.5, 1.0, 2.0)
    # 그리퍼를 닫은 뒤 완료로 보기까지 기다리는 시간(s). 물체를 잡지 않으므로 짧게 둔다.
    # 닫기(약 1 s) + 이 값이 스윙 시간보다 짧아야 다음 펴기가 기다리지 않는다.
    gripper_settle_sec = 0.3

    task = Task('wave_both_arms')
    task.close_gripper('both', settle_time_sec=gripper_settle_sec)
    task.joint_absolute_multi(neutral, approach_motion)
    for _ in range(count):
        task.open_gripper_background('both')
        task.joint_absolute_multi(left_up, swing_motion)
        task.close_gripper_background('both', settle_time_sec=gripper_settle_sec)
        task.joint_absolute_multi(right_down, swing_motion)
    task.joint_absolute_multi(neutral, swing_motion)
    return task.build()

# ===========================================================================
# 8. VSLAM
# ===========================================================================
def offline_map_creation(map_name: str = '') -> RunnableTaskDefinition:
    """운영 전 오프라인 맵 생성. 머리를 이동 각도로 -> cuVSLAM mapping 시작 -> 제자리 보정
    -> 작업자가 Base 탭으로 지점을 돌며 VSLAM 탭에서 보정 + Point 저장 -> Finish mapping -> 지도 저장.

    map_name 이 비어 있으면 map_<날짜>_<시각> 으로 저장한다. 끝나도 VSLAM 은 켜 둔다.
    """
    task = Task('offline_map_creation')
    task.extend(head_pitch_pose(_load_move_head_pitch_deg()))   # 주행 때와 같은 머리 각도로 지도를 만든다
    task.vslam_start('mapping')
    task.vslam_station_correct()        # 출발 지점에서 localization 초기화
    task.vslam_operator_mapping()       # 수동 주행 + Point 저장 (VSLAM 탭 Finish mapping 으로 종료)
    task.vslam_save_map(map_name)
    return task.build()

# ===========================================================================
# 7. 테스트
# ===========================================================================
# test0: base 정렬 후 파지 (허리 자세 무관)
def test0() -> RunnableTaskDefinition:
    """허리 유지. 팔만 초기 자세 -> D435 로 tag_3 보고 base 정렬 -> 왼팔 손목 카메라로 접근·파지·상승 -> 팔 초기 자세."""
    task = Task('test0')
    task.extend(object_gripping_initial_pose())   # 팔만, 허리 그대로
    task.extend(align_base_to_tag('tag_3', arm='left_arm'))
    task.extend(pick_up_object_in_place('tag_3', arm='left_arm'))
    task.extend(object_gripping_initial_pose())
    return task.build()

# test1: 왼손 좌우 픽앤플레이스 반복 (base/허리 고정, 왼팔만 이동)
def test1() -> RunnableTaskDefinition:
    """허리 왼쪽 고정. 왼팔: tag_1 잡기 -> 왼쪽(-x) 15 cm -> tag_2 놓기/다시 잡기 -> 오른쪽(+x) 15 cm -> tag_1 놓기, 10회 반복."""
    shift_motion = (3.0, 0.1, 0.3, 0.5)
    runs = 10                        # 첫 실행 포함 총 반복 횟수

    task = Task('test1')

    task.extend(object_gripping_initial_pose('left'))

    task.extend(pick_up_object_in_place('tag_1', arm='left_arm'))
    task.linear_relative('left_arm', (-0.25, 0.0, 0.0, 0.0, 0.0, 0.0), shift_motion)  # 왼쪽으로
    task.extend(put_down_object_in_place('tag_2', arm='left_arm'))

    task.extend(pick_up_object_in_place('tag_2', arm='left_arm'))
    task.linear_relative('left_arm', (0.15, 0.0, 0.0, 0.0, 0.0, 0.0), shift_motion)  # 오른쪽으로 복귀
    task.extend(put_down_object_in_place('tag_1', arm='left_arm'))
    
    task.rewind(runs)
    return task.build()

# test2: LH1_1 <-> LH2 왕복 픽앤플레이스 1회 (왼팔)
def test2() -> RunnableTaskDefinition:
    """LH1_1 -> LH2 -> LH1_1 왕복 픽앤플레이스 1회 (왼팔)."""
    task = Task('test2')
    task.extend(transfer('LH1_1', 'LH2'))  # 가기
    task.extend(transfer('LH2', 'LH1_1'))  # 오기
    return task.build()

# move_base: base 만 상대 이동 (Measurements 탭 Start frame xy 를 그대로 넣으면 된다)
def test_move_base(x: float, y: float, yaw_deg: float = 0.0) -> RunnableTaskDefinition:
    """지금 base 기준 x 전방 / y 왼쪽 [m] 만큼 이동, yaw_deg 만큼 회전. 팔/허리는 그대로."""
    task = Task('test_move_base')
    task.move_to(x, y, math.radians(yaw_deg), timeout_sec=15.0)
    return task.build()

# ===========================================================================
# Scenario UI 등록
# ===========================================================================
def build_tasks() -> dict[str, RunnableTaskDefinition]:
    # Scenario 목록은 이 순서 그대로 표시된다 (레벨 순).
    return {
        # --- 1. 자세 ---
        'ready_pose': ready_pose(),
        'object_gripping_initial_pose': object_gripping_initial_pose(),
        'object_gripping_initial_pose(front)': object_gripping_initial_pose('front'),
        'object_gripping_initial_pose(left)': object_gripping_initial_pose('left'),
        'object_gripping_initial_pose(right)': object_gripping_initial_pose('right'),
        'search_pose': search_pose(),
        'search_switch_pose': search_switch_pose(),
        'door_handle_grasp_pose': door_handle_grasp_pose(),

        # --- 2. 기본 동작 --- (문 열기/닫기, 스위치는 place_at / pick_at 안에서만 쓴다)
        # ZS 문 태그 기준 base 정렬. 태그는 settings.yaml stations.ZS.work.door.<open|close>.tag
        'align_base_to_door(open)': align_base_to_door('ZS', 'open'),
        'align_base_to_door(close)': align_base_to_door('ZS', 'close'),
        'open_door(ZS)': open_door_at_work('ZS'),   # 단독 문 열기 (시늉 확인용)
        'door_open_stream': door_open_stream('ZS'),  # 지금 손 위치에서 바로 원호로 문 열기 (자세 이동 없음)

        # --- 4. 장비 단위 ---
        # 집기/놓기 지점 base 정렬만 (허리 side 는 settings.yaml 지점 side 로 자동)
        'align_at_station(LH1_1)': align_at_station('LH1_1'),
        'align_at_station(LH2)': align_at_station('LH2'),
        'align_at_station(ZS)': align_at_station('ZS'),
        'pick_at_station(ZS)': pick_at_station('ZS'),

        # --- 6. 데모 ---
        'transfer_demo(LH1_1->ZS, LH1_2->LH2)': transfer_demo(),
        'lh1_to_lh2_demo(LH1_1->LH1_2)': lh1_to_lh2_demo(),
        'impedance_demo_press': impedance_demo_press(),
        'impedance_demo_joint': impedance_demo_joint(),
        'wave_both_arms': wave_both_arms(),

        # --- 8. VSLAM ---
        'offline_map_creation': offline_map_creation(),

        # --- 7. 테스트 ---
        'test0': test0(),
        'test1': test1(),
        'test2': test2(),
        'test_move_base': test_move_base(0.2964, -0.3427),
    }


__all__ = [
    'Task', 'build_tasks',
    'JS_SOFT_ARM', 'JS_CONTACT_ARM',
    # 1. 자세
    'ready_pose', 'torso_align', 'object_gripping_initial_pose', 'head_pitch_pose',
    'pose_before_base_movement', 'door_handle_grasp_pose', 'search_pose', 'search_switch_pose',
    # 2. 기본 동작
    'turn_and_move', 'align_base_to_door', 'align_base_to_tag',
    'move_hand_to_door_handle', 'move_hand_to_tag',
    'door_open_stream', 'door_close_stream',
    # 3. 장비 동작
    'open_door', 'open_door_at_work', 'push_switch',
    'move_hand_from_door_to_switch', 'close_door',
    # 4. 장비 단위 / 5. 운반
    'go_to', 'align_at_station', 'pick_at_station', 'place_at_station', 'transfer', 'transfer_demo', 'lh1_to_lh2_demo',
    # 6. 데모
    'impedance_demo_press', 'impedance_demo_joint', 'wave_both_arms',
    # 8. VSLAM
    'offline_map_creation',
]
