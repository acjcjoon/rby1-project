# Planner Scenario UI용 Task 정의
from __future__ import annotations

import math
from typing import Optional, Sequence

from .task_commands import (
    JointImpedance,
    RunnableTaskDefinition,
    Task,
    _load_current_location,
    _load_location_tag_id,
    _load_move_locations,
    _load_tag_approach_offset,
    _move_location,
    arc_waypoints,
    list_sum,
    wrist_camera_frame,
)


# Task 작성 단위: 초, 미터, 도
# TCP 움직임 설정: [최소 시간, 선속도, 각속도, 가속도 비율]
# 손목 D405 는 양손에 있다 (camera_source d405_right / d405_left).
# 파지/배치 helper(align_base_and_*, *_object_at, pick_up_move_and_put_down_object*)는
# 기본 손이 없다: arm='right_arm' / 'left_arm' 을 반드시 넘긴다 (빠지면 Reload 시 오류).
# 그리퍼와 손목 카메라(camera_source / camera_frame)는 arm 에서 자동으로 정해진다.

# 스위치/문 Task 전용 손. 해당 Task 의 오프셋/자세는 ee_right 기준으로 측정한 값이다.
SWITCH_DOOR_ARM = 'right_arm'
SWITCH_DOOR_GRIPPER = 'right'

_GRIPPER_BY_ARM = {'right_arm': 'right', 'left_arm': 'left'}


def _gripper_for(arm: str) -> str:
    try:
        return _GRIPPER_BY_ARM[arm]
    except KeyError:
        raise ValueError(f"arm must be 'right_arm' or 'left_arm', got {arm!r}") from None


# 태그 기준 D405 손(ee) 자세: 태그와 같은 방향
_OBJECT_TO_D405_EE_ORIENTATION_XYZW = (0.0, 0.0, 0.0, 1.0)

# 연속 작업을 구성할 때의 예정 지점. Reload Tasks 시 다시 초기화한다.
current_position: str | None = None


# ---------------------------------------------------------------------------
# 임피던스 프리셋 (관절 임피던스 한 종류)
#
# 모든 이동은 기본이 위치제어다. 아래 빌더에 impedance= 를 주면 그 스텝만 관절
# 임피던스로 실행된다:
#   관절: joint_absolute / joint_relative / whole_body_joint_absolute
#   Cartesian: linear_absolute / linear_relative / camera_linear_absolute,
#              접촉 헬퍼 press / grasp / place_on_surface / camera_press / ...
# impedance 값: None(생략) = 위치제어, True = settings.yaml 의 "default" 프리셋,
#   "contact" 같은 문자열 = 그 이름의 프리셋, JointImpedance(...) = 그 값 그대로.
# 프리셋 값은 rby1_planner/config/settings.yaml 의 impedance_presets 에서 고치고
# Reload Tasks 하면 반영된다.
# Cartesian 임피던스는 control 노드가 목표 TCP 의 팔 IK 를 풀어 그 관절 목표를 관절
# 임피던스로 추종한다(팔만, 현재 위치에서 10 cm / 20° 이내). 끝나면 측정 관절값으로
# 위치 목표를 다시 잡고(re-anchor), 같은 Task 안에서 그 팔의 위치제어 스텝(예: 후퇴
# linear_relative)이 뒤에 반드시 있어야 한다(없으면 Reload 시 오류).
# ---------------------------------------------------------------------------
# 기존 이름 유지: 프리셋 이름을 가리킨다.
JS_SOFT_ARM = 'default'
JS_CONTACT_ARM = 'contact'

def torso_up() -> Task:
    task = Task('torso_up')
    # [dx, dy, dz, droll, dpitch, dyaw], tcp_motion = [최소 시간, 선속도, 각속도, 가속도 비율]
    task.linear_relative('torso', (0.0, 0.0, -0.02, 0.0, 0.0, 0.0), [3.0, 0.05, 0.2, 0.5])
    return task.build()


def ready_pose() -> Task:
    task = Task('ready_pose')
    task.whole_body_joint_absolute(
        torso=[0.060044, 40.599160, -81.392810, 40.360071, 0.032198, -89.999565],
        right_arm = [1.885080, -79.650879, 35.902233, -105.252939, -66.071558, -14.386377, -22.038574],
        left_arm  = [-2.613022,  30.087802, -11.978428, -120.0,  32.776268, 33.400665,  27.435807],
        joint_motion=[5.0, 1.0, 1.0],
    )
    return task.build()
#left_arm  = [1.885080,  79.650879, -35.902233, -105.252939,  66.071558, -14.386377,  22.038574]
#위의 right arm과 대칭인 left arm좌표
def object_gripping_initial_pose() -> Task:
    task = Task('object_gripping_initial_pose')
    task.whole_body_joint_absolute(
        torso=(0.059126, 45.376237, -90.963053, 45.154560, 0.021390, -0.000280),
        right_arm=(-34.913810, -63.837794, 46.172754, -106.640370, 16.002640, 45.214678, 83.061208),
        left_arm=(-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907),
        joint_motion=(5.0, 1.0, 1.0),
    )
    return task.build()


def object_gripping_initial_pose_temp() -> Task:
    task = Task('object_gripping_initial_pose_temp')
    task.whole_body_joint_absolute(
        right_arm=(-38.280283, -61.948097, 45.740109, -95.376122, 15.278687, 36.629077, 83.445557),
        left_arm=(-37.023491, 61.226480, -44.046256, -95.222313, -16.824243, 35.951221, -84.381812),
        joint_motion=(2.0, 2.0, 4.0),
    )
    return task.build()


def object_gripping_initial_pose_torso_right() -> Task:
    task = Task('object_gripping_initial_pose_torso_right')
    task.whole_body_joint_absolute(
        torso=[0.060044, 40.599160, -81.392810, 40.360071, 0.032198, -89.999565],        right_arm=(-38.280283, -61.948097, 45.740109, -95.376122, 15.278687, 36.629077, 83.445557),
        left_arm=(-37.023491, 61.226480, -44.046256, -95.222313, -16.824243, 35.951221, -84.381812),
        joint_motion=(5.0, 2.0, 3.0),
    )
    return task.build()


def object_gripping_initial_pose_torso_left() -> Task:
    task = Task('object_gripping_initial_pose_torso_left')
    task.whole_body_joint_absolute(
        torso=(0.065048, 45.375276, -90.959835, 45.153373, 0.026976, 90.00),
        right_arm=(-38.280283, -61.948097, 45.740109, -95.376122, 15.278687, 36.629077, 83.445557),
        left_arm=(-37.023491, 61.226480, -44.046256, -95.222313, -16.824243, 35.951221, -84.381812),
        joint_motion=(5.0, 2.0, 3.0),
    )
    return task.build()

def pose_before_base_movement() -> Task:
    task = Task('pose_before_base_movement')
    # 오른팔: object_gripping_initial_pose_torso_left 에서 TCP 를 base (-y 0.12 -> +x 0.07) 옮긴 자세
    # (손 방향 동일, SDK 모델 IK 계산값). 이전 값:
    # right_arm=[-16.485584, -81.212678, 36.201365, -117.319844, 5.521069, 37.464697, 72.280151]
    task.whole_body_joint_absolute(
        torso=(0.065048, 45.375276, -90.959835, 45.153373, 0.026976, 90.00),
        right_arm=[-8.030944, -85.718373, 30.622439, -119.996443, 3.002524, 34.710418, 68.873201],
        left_arm=(-37.023491, 61.226480, -44.046256, -95.222313, -16.824243, 35.951221, -84.381812),
        joint_motion=(5.0, 2.0, 3.0),
    )
    return task.build()



def object_gripping_initial_pose_front() -> Task:
    """팔은 움직이지 않고 허리만 정면(torso_5 = 0)을 보게 한다. 높이는 torso_left/right 와 같다."""
    task = Task('object_gripping_initial_pose_front')
    task.whole_body_joint_absolute(
        torso=(0.065048, 45.375276, -90.959835, 45.153373, 0.026976, 0.0),
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


def _configured_tag_approach_contact_move(object_id: str, approach_offset: Sequence[object], contact_offset: Sequence[object], tcp_motion: Sequence[object], *, arm: str, camera_source: Optional[str] = None) -> RunnableTaskDefinition:
    approach = tuple(float(value) for value in approach_offset)
    contact = tuple(float(value) for value in contact_offset)
    if len(approach) != 3 or len(contact) != 3:
        raise ValueError('approach/contact offsets must contain 3 numbers')
    tag_offset = _load_tag_approach_offset(object_id)
    approach = tuple(value + offset for value, offset in zip(approach, tag_offset))
    contact = tuple(value + offset for value, offset in zip(contact, tag_offset))
    # 접근·접촉은 동일한 중앙을 기준으로 한다. XY는 태그 yaw로 변환하며
    # yaw_only=True에 따라 Z는 base 수직 방향으로 더한다.
    task = Task(f'move_{object_id}_approach_contact')
    task.delay(2000)
    task.camera_linear_absolute(
        arm, object_id, tcp_motion,
        camera_source=camera_source,
        detection_timeout_sec=3.0,
        yaw_only=True,
        yaw_symmetry_deg=180.0,
        log_target_and_actual=True,
        object_to_end_effector_position=approach,
        object_to_end_effector_orientation_xyzw=_OBJECT_TO_D405_EE_ORIENTATION_XYZW,
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
        object_to_end_effector_orientation_xyzw=_OBJECT_TO_D405_EE_ORIENTATION_XYZW,
    )
    return task.build()


def turn_and_move(start_point: str, end_point: str) -> RunnableTaskDefinition:
    global current_position

    locations = _load_move_locations()
    start_x, start_y, start_side = _move_location(locations, start_point)
    end_x, end_y, side = _move_location(locations, end_point)
    task = Task('turn_and_move')
    if start_side != side:
        if side == 'left':
            task.extend(object_gripping_initial_pose_torso_left())
        elif side == 'front':
            task.extend(object_gripping_initial_pose_front())
        else:
            task.extend(object_gripping_initial_pose_torso_right())
    task.move_to(end_x - start_x, end_y - start_y, 0.0, timeout_sec=15.0)
    task.delay(1000)
    definition = task.build()
    current_position = end_point
    return definition


def turn_left_and_move_left()-> RunnableTaskDefinition:
    task = Task('turn_left_and_move_left')
    task.extend(object_gripping_initial_pose_torso_left())
    task.move_to(0.0, 0.87, 0.0, timeout_sec=15.0)
    task.delay(1000)
    return task.build()


def turn_right_and_move_right()-> RunnableTaskDefinition:
    task = Task('turn_right_and_move_right')
    task.extend(object_gripping_initial_pose_torso_right())
    task.move_to(0.0, -0.87, 0.0, timeout_sec=15.0)
    task.delay(1000)
    return task.build()


# 제자리 파지/배치 공통 값 (base 정렬 버전도 같은 값을 쓴다)
PICK_PLACE_CONTACT_OFFSET = (0.0, 0.0, 0.065)  # 태그 기준 접촉 지점 (+ settings.yaml 태그 오프셋)
PICK_PLACE_LIFT_M = 0.075                      # 접근 높이 = 접촉 + 이 값, 파지/배치 후 상승 거리
PICK_PLACE_APPROACH_MOTION = (3.0, 1.0, 1.0, 1.0)
PICK_LIFT_MOTION = (1.0, 1.0, 1.0, 1.0)
PLACE_LIFT_MOTION = (3.0, 1.0, 1.0, 1.0)


def _pick_place_offsets() -> tuple[list, list]:
    contact = list(PICK_PLACE_CONTACT_OFFSET)
    return list_sum(contact, [0.0, 0.0, PICK_PLACE_LIFT_M]), contact


def pick_up_object_in_place(
    object_id: str, *, arm: str, camera_source: Optional[str] = None,
) -> RunnableTaskDefinition:
    """base 이동 없이 현재 자세에서 손목 카메라로 태그를 보고 그리퍼 열기 -> 접근·접촉 -> 파지 -> 상승."""
    gripper = _gripper_for(arm)
    approach_offset, contact_offset = _pick_place_offsets()
    task = Task(f'pick_up_object_in_place_{object_id}')
    task.open_gripper(gripper)
    task.extend(_configured_tag_approach_contact_move(
        object_id, approach_offset, contact_offset, PICK_PLACE_APPROACH_MOTION,
        arm=arm, camera_source=camera_source,
    ))
    task.close_gripper(gripper)
    task.linear_relative(arm, (0.0, 0.0, PICK_PLACE_LIFT_M, 0.0, 0.0, 0.0), PICK_LIFT_MOTION)
    return task.build()


def put_down_object_in_place(
    object_id: str, *, arm: str, camera_source: Optional[str] = None,
) -> RunnableTaskDefinition:
    """base 이동 없이 현재 자세에서 손목 카메라로 태그를 보고 접근·접촉 -> 놓기 -> 상승."""
    gripper = _gripper_for(arm)
    approach_offset, contact_offset = _pick_place_offsets()
    task = Task(f'put_down_object_in_place_{object_id}')
    task.extend(_configured_tag_approach_contact_move(
        object_id, approach_offset, contact_offset, PICK_PLACE_APPROACH_MOTION,
        arm=arm, camera_source=camera_source,
    ))
    task.open_gripper(gripper)
    task.linear_relative(arm, (0.0, 0.0, PICK_PLACE_LIFT_M, 0.0, 0.0, 0.0), PLACE_LIFT_MOTION)
    return task.build()


def align_base_and_put_down_object(
    object_id: str,
    *,
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
    arm: str,
    camera_source: Optional[str] = None,
    camera_frame: Optional[str] = None,
) -> RunnableTaskDefinition:
    """동일 태그로 base 정렬 후 접근·접촉하고 물체를 놓는다.

    arm 필수. 그리퍼와 camera_source/camera_frame(생략 시)은 arm 에서 정한다.
    """
    camera_frame = wrist_camera_frame(arm, camera_frame)
    task = Task(f'align_base_and_put_down_object_{object_id}')

    task.move_base_to_detected_tag(
        camera_source='d435', object_id=object_id,
        desired_tag_x=0.0, desired_tag_y=0.0, desired_tag_frame=camera_frame,
        threshold_x_minus=threshold_x_minus, threshold_y_plus=threshold_y_plus,
        relative_yaw=0.0, detection_timeout_sec=5.0, navigation_timeout_sec=30.0,
        max_translation_m=0.8,
    )
    task.delay(1000)
    task.extend(put_down_object_in_place(object_id, arm=arm, camera_source=camera_source))
    task.extend(object_gripping_initial_pose_temp())
    return task.build()


def align_base_and_pick_up_object(
    object_id: str,
    *,
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
    arm: str,
    camera_source: Optional[str] = None,
    camera_frame: Optional[str] = None,
) -> RunnableTaskDefinition:
    """동일 태그로 base 정렬 후 그리퍼를 열고 접근·파지·상승한다.

    arm 필수. 그리퍼와 camera_source/camera_frame(생략 시)은 arm 에서 정한다.
    """
    camera_frame = wrist_camera_frame(arm, camera_frame)
    task = Task(f'align_base_and_pick_up_object_{object_id}')

    task.move_base_to_detected_tag(
        camera_source='d435', object_id=object_id,
        desired_tag_x=0.0, desired_tag_y=0.0, desired_tag_frame=camera_frame,
        threshold_x_minus=threshold_x_minus, threshold_y_plus=threshold_y_plus,
        relative_yaw=0.0, detection_timeout_sec=5.0, navigation_timeout_sec=30.0,
        max_translation_m=0.8,
    )
    task.delay(1000)
    task.extend(pick_up_object_in_place(object_id, arm=arm, camera_source=camera_source))
    task.extend(object_gripping_initial_pose_temp())
    return task.build()


# ---------------------------------------------------------------------------
# tag_6 기준 base 정렬 (D435 관측, 기준 프레임 d435_camera_center)
#
# 2026-11-12 로봇을 원하는 위치에 세워 두고 측정한 값 (6 s, 171 샘플 평균, 표준편차 < 0.5 mm):
#   ros2 run tf2_ros tf2_echo d435_camera_center d435_tag_pnp_6
#   -> Translation [0.589, -0.326, 0.027]  (z 는 사용하지 않음)
# 즉 tag_6 이 d435_camera_center 기준 전방 58.9 cm, 우측 32.6 cm 에 보이도록 base 를
# 옮긴다. 측정 위치에서 실행하면 이동하지 않는다. 기준 프레임이 머리에 달려 있으므로
# 측정할 때와 같은 머리/토르소 자세에서 실행해야 한다. 방향(yaw)은 맞추지 않는다.
# 값을 바꿀 때: 로봇을 원하는 위치에 두고 위 명령으로 다시 읽어 x, y 를 넣는다.
# ---------------------------------------------------------------------------
# base -y 1.5 cm 더 멀리 (이전: (0.600, -0.326)). 카메라 x 축이 39° 숙여져 있어
# base 수평 성분이 0.801 이므로 x 를 0.015 / 0.801 = 0.0187 m 늘렸다.
ALIGN_TAG6_DESIRED_XY_IN_D435 = (0.619, -0.326)


def align_base_to_tag6(
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
) -> RunnableTaskDefinition:
    """D435 로 tag_6 을 관측해 측정해 둔 상대 위치가 되도록 base 를 이동한다."""
    task = Task('align_base_to_tag6')
    task.move_base_to_detected_tag(
        camera_source='d435', object_id='tag_6',
        desired_tag_x=ALIGN_TAG6_DESIRED_XY_IN_D435[0],
        desired_tag_y=ALIGN_TAG6_DESIRED_XY_IN_D435[1],
        desired_tag_frame='d435_camera_center',
        threshold_x_minus=threshold_x_minus, threshold_y_plus=threshold_y_plus,
        relative_yaw=0.0, detection_timeout_sec=5.0, navigation_timeout_sec=30.0,
        max_translation_m=0.8,
    )
    return task.build()


# tag_7 기준 base 정렬: align_base_to_tag6 직후(base 이동 없음) 같은 머리/토르소 자세에서
# 2026-11-13 측정 (d435_camera_center -> d435_tag_pnp_7, 6 s, 144 샘플, 표준편차 < 0.2 mm):
#   Translation [0.4388, -0.1004, -0.0353]  (z 는 사용하지 않음)
# 따라서 align_base_to_tag6 -> align_base_to_tag7 순서로 실행하면 base 가 움직이지 않는다.
# ALIGN_TAG6_DESIRED_XY_IN_D435 를 바꾸면 이 값도 그 위치에서 다시 측정해야 한다.
ALIGN_TAG7_DESIRED_XY_IN_D435 = (0.4188, -0.1004)


def align_base_to_tag7(
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
) -> RunnableTaskDefinition:
    """D435 로 tag_7 을 관측해 align_base_to_tag6 과 같은 base 위치가 되도록 이동한다."""
    task = Task('align_base_to_tag7')
    task.move_base_to_detected_tag(
        camera_source='d435', object_id='tag_7',
        desired_tag_x=ALIGN_TAG7_DESIRED_XY_IN_D435[0],
        desired_tag_y=ALIGN_TAG7_DESIRED_XY_IN_D435[1],
        desired_tag_frame='d435_camera_center',
        threshold_x_minus=threshold_x_minus, threshold_y_plus=threshold_y_plus,
        relative_yaw=0.0, detection_timeout_sec=5.0, navigation_timeout_sec=30.0,
        max_translation_m=0.8,
    )
    return task.build()

def search_pose() -> Task:
    task = Task('search_pose')
    task.whole_body_joint_absolute(
        right_arm = [-2.639050, -48.778294, 9.475941, -105.823940, -48.357757, 5.870969, -18.970059],
        joint_motion=(5.0, 1.0, 1.0),
    )
    return task.build()

def search_switch_pose() -> Task:
    task = Task('search_switch_pose')
    # base 180° 회전 배치: torso_5 를 -89.999130 -> +90.000870 (상체는 이전과 같은 방향)
    task.whole_body_joint_absolute(
        torso=[0.060044, 43.042258, -86.290755, 42.812307, 0.030457, 90.000870],
        right_arm = [-32.177210, -57.932216, 44.146889, -91.228664, -64.357215, -9.066141, -33.238335],        
        left_arm=(-37.023491, 61.226480, -44.046256, -95.222313, -16.824243, 35.951221, -84.381812),
        joint_motion=(5.0, 1.0, 1.0),
    )
    return task.build()

# ---------------------------------------------------------------------------
# 스위치 누르기 (D405 로 tag_5 관측 -> 접근 -> 직선 누름 -> 후퇴)
#
# 2026-11-11 search_switch_pose 에서 측정한 기하:
#   tf2_echo base right_tag_pnp_5      : 태그 z 축 = base +y (로봇 쪽), 태그 y 축 = 위, 벽은 -y
#   tf2_echo right_tag_pnp_5 ee_right  : TCP 가 태그 평면 앞 0.123 m, 태그 y 로 -0.040 m,
#                                  태그 z 축 기준 약 +93° 회전 (손 x 축이 위)
# 접근만 카메라(태그 프레임 오프셋)로 하고, 누름과 후퇴는 base 좌표 linear_relative
# (위치제어 직선 이동)로 한다. 누름 거리는 아래 상수로 직접 정한다. 위치제어라 힘 제한이
# 없으므로 누름 깊이는 버튼 행정보다 조금만 크게 잡는다.
# ---------------------------------------------------------------------------
SWITCH_TAG_ID = 'tag_5'
# 태그 평면 안에서 버튼 중심 위치 (tag x, tag y). 태그가 버튼 위에 붙어 있으면 (0, 0).
SWITCH_BUTTON_OFFSET_IN_TAG = (0.0, 0.0)
# 접근: TCP 를 태그 평면에서 이만큼 떨어진 지점(태그 법선 방향)에 둔다.
SWITCH_APPROACH_STANDOFF_M = 0.08
# 태그 -> 손(ee_right) 자세: 태그 z 축 기준 +90°. 손가락이 벽(-z)을 향하고 손 x 축이 위.
SWITCH_TAG_TO_EE_ORIENTATION_XYZW = (0.0, 0.0, 0.7071068, 0.7071068)
# 누름: 접근 지점에서 base 좌표로 이만큼 직선 이동한다.
# base 180° 회전 배치: 벽이 base +y 쪽이다 (회전 전: (0.0, -0.02, 0.0)).
SWITCH_PRESS_OFFSET_BASE = (0.0, 0.01, 0.0)
# 후퇴: 누른 뒤 같은 거리만큼 되돌아온다 (회전 전: (0.0, 0.02, 0.0)).
SWITCH_RETREAT_OFFSET_BASE = (0.0, -0.01, 0.0)
# [최소 시간 s, 선속도 m/s, 각속도 rad/s, 가속도 비율]
SWITCH_APPROACH_MOTION = (3.0, 0.1, 0.3, 0.5)
SWITCH_PRESS_MOTION = (3.0, 0.3, 0.3, 0.5)


def search_and_push_switch() -> RunnableTaskDefinition:
    """search_switch_pose 에서 D405 로 tag_5 를 보고 접근한 뒤 직선으로 누르고 돌아온다."""
    bx, by = SWITCH_BUTTON_OFFSET_IN_TAG
    task = Task('search_and_push_switch')
    task.extend(search_switch_pose())
    task.close_gripper(SWITCH_DOOR_GRIPPER)
    task.camera_linear_absolute(
        SWITCH_DOOR_ARM, SWITCH_TAG_ID, SWITCH_APPROACH_MOTION,
        detection_timeout_sec=5.0,
        log_target_and_actual=True,
        object_to_end_effector_position=(bx, by, SWITCH_APPROACH_STANDOFF_M),
        object_to_end_effector_orientation_xyzw=SWITCH_TAG_TO_EE_ORIENTATION_XYZW,
    )
    # 2) 누름: 측정 TCP 에서 base 좌표로 직선 이동 (자세 유지)
    task.linear_relative(SWITCH_DOOR_ARM, SWITCH_PRESS_OFFSET_BASE + (0.0, 0.0, 0.0), SWITCH_PRESS_MOTION)
    # 3) 후퇴
    task.linear_relative(SWITCH_DOOR_ARM, SWITCH_RETREAT_OFFSET_BASE + (0.0, 0.0, 0.0), SWITCH_PRESS_MOTION)
    # 4) 탐색 자세로 복귀
    task.extend(search_switch_pose())
    return task.build()

def push_switch_and_close_door() -> RunnableTaskDefinition:
    bx, by = SWITCH_BUTTON_OFFSET_IN_TAG
    task = Task('push_switch_and_close_door')
    tcp_motion = [2.0, 0.1, 0.3, 0.5]
    task.extend(search_switch_pose())
    task.close_gripper(SWITCH_DOOR_GRIPPER)
    task.camera_linear_absolute(
        SWITCH_DOOR_ARM, SWITCH_TAG_ID, SWITCH_APPROACH_MOTION,
        detection_timeout_sec=5.0,
        log_target_and_actual=True,
        object_to_end_effector_position=(bx, by, SWITCH_APPROACH_STANDOFF_M),
        object_to_end_effector_orientation_xyzw=SWITCH_TAG_TO_EE_ORIENTATION_XYZW,
    )
    # 2) 누름: 측정 TCP 에서 base 좌표로 직선 이동 (자세 유지)
    task.linear_relative(SWITCH_DOOR_ARM, SWITCH_PRESS_OFFSET_BASE + (0.0, 0.0, 0.0), SWITCH_PRESS_MOTION)
    # 3) 후퇴
    task.linear_relative(SWITCH_DOOR_ARM, SWITCH_RETREAT_OFFSET_BASE + (0.0, 0.0, 0.0), SWITCH_PRESS_MOTION)
    # 4) 탐색 자세로 복귀
    task.extend(search_switch_pose())
    # base 180° 회전 배치 (회전 전: (0, 0.19, 0) -> (0.15, 0, 0))
    task.linear_relative(SWITCH_DOOR_ARM, (0.0, -0.19, 0.0, 0.0, 0.0, 0.0), tcp_motion)
    task.linear_relative(SWITCH_DOOR_ARM, (-0.15, 0.0, 0.0, 0.0, 0.0, 0.0), tcp_motion)
    task.open_gripper(SWITCH_DOOR_GRIPPER)
    task.extend(move_ee_to_tag7_offset())
    task.close_gripper(SWITCH_DOOR_GRIPPER)
    task.extend(door_close_right_arm_stream())
    return task.build()

def open_door_and_push_switch2()-> RunnableTaskDefinition:
    task = Task('open_door_and_push_switch2')
    tcp_motion = [2.0, 0.1, 0.3, 0.5]
    task.extend(search_and_open_door2())
    task.open_gripper(SWITCH_DOOR_GRIPPER)
    task.linear_relative(SWITCH_DOOR_ARM, (0.0, -0.056, 0.0, 0.0, 0.0, 0.0), tcp_motion)
    task.close_gripper(SWITCH_DOOR_GRIPPER)
    task.linear_relative(SWITCH_DOOR_ARM, (0.15, 0.0, 0.0, 0.0, 0.0, 0.0), tcp_motion)
    task.extend(search_and_push_switch())
    return task.build()


# ---------------------------------------------------------------------------
# tag_6 기준 오른손 이동 (D435 관측, Cartesian 절대 목표)
#
# 2026-11-12 측정 (tf2_echo base d435_tag_pnp_6, 5 s 평균):
#   d435_tag_pnp_6 축 = base 기준 x: -x, y: +z(위), z: +y(로봇 쪽).  (base 180° 회전 전)
#   base 180° 회전 배치에서는 x: +x, y: +z(위), z: -y(로봇 쪽).
#   (tag_4 를 D405 로 볼 때와 축 배치가 다르다: tag_4 는 y=-y, z=+z 였다.)
# 오프셋은 "현재 손에서 얼마나 움직이는가"가 아니라 "태그에서 어디에 멈추는가"다.
# ---------------------------------------------------------------------------
# 원하는 손(ee_right) 위치, base 축 기준 태그 대비 [dx, dy, dz] m.
# base 180° 회전 배치 값 (회전 전: (-0.02, 0.03, 0.09)). 태그 기준 위치는 회전 전과 같다.
TAG6_EE_OFFSET_BASE = (0.02, -0.02, 0.09)  # +y 2 cm 추가 (이전: (0.02, -0.03, 0.09))
# 위 base 오프셋을 태그 프레임으로 바꾼 값 (태그 x=+x, y=+z, z=-y 이므로 (dx, dz, −dy)).
TAG6_EE_OFFSET_IN_TAG = (
    TAG6_EE_OFFSET_BASE[0],
    TAG6_EE_OFFSET_BASE[2],
    -TAG6_EE_OFFSET_BASE[1],
)
# d435_tag_pnp_6 -> ee_right 회전 (xyzw): 태그 z 축 기준 +90°. 태그 기준이라 base 회전과 무관하다.
# 손가락이 벽(태그 -z)을 향하고 손이 수평(손 x 축 = 위)이 되는 이상적인 값이다.
TAG6_EE_ORIENTATION_XYZW = (0.0, 0.0, 0.7071068, 0.7071068)
TAG6_MOVE_MOTION = (3.0, 0.1, 0.3, 0.5)

def search_and_open_door2() -> RunnableTaskDefinition:
    task = Task('search_and_open_door2')
    task.open_gripper('right')
    task.extend(search_pose())
    task.extend(align_base_to_tag6())
    task.extend(move_ee_to_tag6_offset())
    task.close_gripper('right')
    task.extend(door_open_right_arm_stream())
    return task.build()
    
def move_ee_to_tag6_offset() -> RunnableTaskDefinition:
    """D435 로 tag_6 을 보고 오른손 TCP 를 태그 바로 위 6 cm (base 기준) 지점으로 옮긴다."""
    task = Task('move_ee_to_tag6_offset')
    task.camera_linear_absolute(
        SWITCH_DOOR_ARM, 'tag_6', TAG6_MOVE_MOTION,
        camera_source='d435',
        detection_timeout_sec=5.0,
        log_target_and_actual=True,
        object_to_end_effector_position=TAG6_EE_OFFSET_IN_TAG,
        object_to_end_effector_orientation_xyzw=TAG6_EE_ORIENTATION_XYZW,
    )
    return task.build()


# tag_7 기준 오른손 이동 (D435 관측). tag_6 과 같은 방식.
# 가정: d435_tag_pnp_7 축이 tag_6 과 같다. 측정 전.
# base 180° 회전 배치: 태그 축 base 기준 x: +x, y: +z, z: -y (회전 전 BASE: (-0.02, 0.026, 0.08)).
TAG7_EE_OFFSET_BASE = (0.02, -0.006, 0.08)  # +y 0.5 cm 추가 (이전: (0.02, -0.011, 0.08))
TAG7_EE_OFFSET_IN_TAG = (
    TAG7_EE_OFFSET_BASE[0],
    TAG7_EE_OFFSET_BASE[2],
    -TAG7_EE_OFFSET_BASE[1],
)
TAG7_EE_ORIENTATION_XYZW = (0.0, 0.0, 0.7071068, 0.7071068)
TAG7_MOVE_MOTION = (3.0, 0.1, 0.3, 0.5)


def move_ee_to_tag7_offset() -> RunnableTaskDefinition:
    """D435 로 tag_7 을 보고 오른손 TCP 를 TAG7_EE_OFFSET_BASE (base 기준) 지점으로 옮긴다."""
    task = Task('move_ee_to_tag7_offset')
    task.camera_linear_absolute(
        SWITCH_DOOR_ARM, 'tag_7', TAG7_MOVE_MOTION,
        camera_source='d435',
        detection_timeout_sec=5.0,
        log_target_and_actual=True,
        object_to_end_effector_position=TAG7_EE_OFFSET_IN_TAG,
        object_to_end_effector_orientation_xyzw=TAG7_EE_ORIENTATION_XYZW,
    )
    return task.build()


def pick_up_object_at(
    location_name: str,
    *,
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
    arm: str,
    **camera,
) -> RunnableTaskDefinition:
    """설정된 지점의 고정 태그를 기준으로 물체를 집는다.

    arm 필수. camera: align_base_and_pick_up_object 의 camera_source/camera_frame.
    """
    return align_base_and_pick_up_object(
        _load_location_tag_id(location_name),
        threshold_x_minus=threshold_x_minus,
        threshold_y_plus=threshold_y_plus,
        arm=arm,
        **camera,
    )


def put_down_object_at(
    location_name: str,
    *,
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
    arm: str,
    **camera,
) -> RunnableTaskDefinition:
    """설정된 지점의 고정 태그를 기준으로 물체를 내려놓는다.

    arm 필수. camera: align_base_and_put_down_object 의 camera_source/camera_frame.
    """
    return align_base_and_put_down_object(
        _load_location_tag_id(location_name),
        threshold_x_minus=threshold_x_minus,
        threshold_y_plus=threshold_y_plus,
        arm=arm,
        **camera,
    )


def pick_up_move_and_put_down_object(start_point: str, end_point: str, *, arm: str) -> RunnableTaskDefinition:
    task = Task('pick_up_move_and_put_down_object')
    task.extend(pick_up_object_at(start_point, arm=arm))
    task.extend(turn_and_move(start_point, end_point))
    task.extend(put_down_object_at(end_point, arm=arm))
    return task.build()


def pick_up_move_and_put_down_object2(
    start_point: str,
    end_point: str,
    *,
    arm: str,
) -> RunnableTaskDefinition:
    """이전 도착 지점에서 출발 지점으로 이동한 뒤 파지·운반·배치한다.

    current_position은 실제 로봇 위치가 아니라 명령 구성 중의 예정 지점이다.
    첫 호출에서는 YAML의 current_location을 사용하며, 값이 없으면 이미
    start_point에 있다고 간주한다. YAML 파일 자체는 변경하지 않는다.
    """
    global current_position
    previous_position = current_position
    try:
        locations = _load_move_locations()
        _move_location(locations, start_point)
        _move_location(locations, end_point)
        current_point = current_position
        if current_point is None:
            current_point = _load_current_location()
        if current_point is None:
            current_point = start_point
        _move_location(locations, current_point)

        task = Task('pick_up_move_and_put_down_object2')
        if current_point != start_point:
            task.extend(turn_and_move(current_point, start_point))
        task.extend(pick_up_move_and_put_down_object(start_point, end_point, arm=arm))
        definition = task.build()
    except Exception:
        current_position = previous_position
        raise

    # 명령 생성이 성공한 경우에만 다음 호출의 출발 위치를 갱신한다.
    current_position = end_point
    return definition


def object_handover_demo_final() -> RunnableTaskDefinition:
    task = Task('object_handover_demo_final')

    task.extend(object_gripping_initial_pose_torso_left())
    task.extend(pick_up_move_and_put_down_object('LH1', 'LH2', arm='left_arm'))

    return task.build()


def object_handover_demo_final_tmp() -> RunnableTaskDefinition:
    task = Task('object_handover_demo_final_tmp')

    task.extend(object_gripping_initial_pose_torso_left())
    task.extend(pick_up_move_and_put_down_object('LH1', 'LH2', arm='left_arm'))

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

# ---------------------------------------------------------------------------
# 문 손잡이 파지 자세
# ---------------------------------------------------------------------------
# object_gripping_initial_pose_torso_right 에서 오른팔만 손잡이 파지 관절값으로 변경
DOOR_HANDLE_RIGHT_ARM = [-55.483742, -44.230608, 44.420618, -46.120118, -68.745680, -9.224412, -8.102626]

# 실측 (2026-11-12): 문이 닫힌 상태에서 경첩 -> 손잡이 중심, base 기준 (m).
# 문 길이 23 cm 는 원호 계산에 쓰이지 않는다 (원호 반지름은 경첩~손잡이 중심 거리).
# base 180° 회전 배치 값 (회전 전: (-0.18975, 0.0325)).
DOOR_HANDLE_FROM_HINGE_BASE = (0.18975, -0.0325)
DOOR_RADIUS_M = math.hypot(*DOOR_HANDLE_FROM_HINGE_BASE)  # 0.1925 m
DOOR_SWING_DEG = 90.0
# 손잡이(현재 TCP)에서 경첩을 바라보는 방향, base XY 평면 기준 각도(도).
#   0 = +x(전방), 90 = +y(좌측), 270 = -y(우측), 180 = -x(후방)
# 열기 시작(닫힘): 경첩은 손 기준 -x 에서 +y 쪽으로 9.7° -> 170.28° (회전 전 350.28°).
# 시계 방향으로 연다 (base z 축은 그대로 위이므로 회전 방향은 바뀌지 않는다).
DOOR_OPEN_HINGE_DIRECTION_DEG = math.degrees(math.atan2(
    -DOOR_HANDLE_FROM_HINGE_BASE[1], -DOOR_HANDLE_FROM_HINGE_BASE[0])) % 360.0
# 닫기 시작(90° 열림): 문이 시계 방향으로 90° 돌았으므로 경첩 방향도 -90° -> 80.28° (회전 전 260.28°).
DOOR_CLOSE_HINGE_DIRECTION_DEG = (DOOR_OPEN_HINGE_DIRECTION_DEG - DOOR_SWING_DEG) % 360.0


# 스트림 경로 설정: [전체 소요 시간 s, 선속도 상한 m/s, 각속도 상한 rad/s]
# 60도 원호 길이는 약 0.24 m 이므로 12 s 면 평균 0.02 m/s. control 노드의
# stream_path_linear_velocity_limit(0.1 m/s)보다 큰 상한은 거부된다.
DOOR_STREAM_PATH_MOTION = (12.0, 0.05, 0.3)


def door_open_right_arm_stream(
    open_angle_deg: float = DOOR_SWING_DEG,
    step_deg: float = 2.0,
    yaw_follows_door: bool = False,
) -> Task:

    task = Task('door_open_right_arm_stream')
    task.linear_path_relative(
        'right_arm',
        arc_waypoints(
            DOOR_RADIUS_M,
            DOOR_OPEN_HINGE_DIRECTION_DEG,
            open_angle_deg,
            step_deg,
            clockwise=True,
            yaw_follows_arc=yaw_follows_door,
        ),
        DOOR_STREAM_PATH_MOTION,
    )
    return task.build()


def door_close_right_arm_stream(
    close_angle_deg: float = DOOR_SWING_DEG,
    step_deg: float = 2.0,
    yaw_follows_door: bool = False,
) -> Task:
    """90° 열린 문을 닫는다: door_open_right_arm_stream 경로를 반시계로 되감는다."""
    task = Task('door_close_right_arm_stream')
    task.linear_path_relative(
        'right_arm',
        arc_waypoints(
            DOOR_RADIUS_M,
            DOOR_CLOSE_HINGE_DIRECTION_DEG,
            close_angle_deg,
            step_deg,
            clockwise=False,
            yaw_follows_arc=yaw_follows_door,
        ),
        DOOR_STREAM_PATH_MOTION,
    )
    return task.build()


# ---------------------------------------------------------------------------
# 양팔 흔들기 (인사)
#
# 그리퍼를 닫고 정면을 본 채 양팔을 든 뒤, 두 팔을 같은 방향(왼쪽 -> 오른쪽)으로 함께
# 흔든다. 팔 흔들기는 주로 J3(어깨 yaw, 약 45도)와 J4(팔꿈치, 약 15도)로 이루어진다.
# 스텝마다 토르소와 두 팔이 한 명령으로 동기되어 움직이고, 목표에 도달한 뒤 다음
# 스텝으로 넘어가므로 양 끝에서 잠깐 멈췄다가 돌아온다.
#
# 토르소: 시작 중립 자세는 ready_pose 높이에서 yaw 만 0(정면)으로 둔 자세다. 왼쪽으로
# 흔들 때 약 4.5 cm 일어서고 오른쪽으로 흔들 때 ready_pose 높이로 내려온다. UP 은
# torso_1/2/3 을 같은 비율로 펴서 가슴의 기울기와 앞뒤 위치가 변하지 않는다(URDF FK).
#
# 그리퍼: 닫은 채로 시작해서 왼쪽 스윙 동안 펴고 오른쪽 스윙 동안 오므린다. 그리퍼는
# 백그라운드 스텝이라 팔과 동시에 움직인다(BackgroundGripperStep). 빈손 동작 전용이며
# 물체를 잡는 Task 에는 쓰지 않는다.
#
# LEFT/RIGHT 는 로봇을 마주 본 사람 기준이다(LEFT 에서 두 손이 로봇의 오른쪽으로
# 간다). 따라서 왼팔 LEFT 와 오른팔 RIGHT 가 손이 몸 중앙 쪽으로 오는 자세다.
# 왼팔 세 자세와 오른팔 RIGHT 자세는 UI Joint 탭의 Get q 실측값이다. 오른팔
# NEUTRAL/LEFT 는 실측이 아니라, 오른팔 RIGHT 실측값에 왼팔의 J3/J4 변화량을
# 좌우 대칭으로 적용해 계산한 값이다(나머지 관절은 RIGHT 실측값 그대로).
# ---------------------------------------------------------------------------
WAVE_LEFT_ARM_NEUTRAL = [-65.01, 31.67, 29.12, -106.92, 17.47, 6.39, 0.48]
WAVE_LEFT_ARM_LEFT = [-66.78, 31.71, 0.90, -117.12, 17.46, 6.35, 0.44]
WAVE_LEFT_ARM_RIGHT = [-66.77, 31.72, 45.92, -102.31, 17.50, 6.35, 0.52]
WAVE_RIGHT_ARM_RIGHT = [-71.65, -31.12, -1.54, -112.08, -17.24, 3.04, 0.15]
WAVE_RIGHT_ARM_NEUTRAL = [-71.65, -31.12, -29.76, -101.88, -17.24, 3.04, 0.15]
WAVE_RIGHT_ARM_LEFT = [-71.65, -31.12, -46.56, -97.27, -17.24, 3.04, 0.15]
# 토르소: DOWN = ready_pose 높이에서 정면(yaw 0), UP = 약 4.5 cm 일어선 자세.
WAVE_TORSO_DOWN = [0.065048, 45.375276, -90.959835, 45.153373, 0.026976, 0.0]
WAVE_TORSO_UP = [0.065048, 40.0, -80.0, 39.568814, 0.026976, 0.0]
# 관절 이동 설정: [최소 시간 s, 속도 rad/s, 가속도 rad/s^2]
# 속도는 관절 모델 한계(2.09 rad/s), 가속도는 토르소 한계 5 rad/s^2 를 넘으면 거부된다.
WAVE_APPROACH_JOINT_MOTION = (4.0, 1.0, 1.0)
WAVE_SWING_JOINT_MOTION = (1.5, 1.0, 2.0)
# 그리퍼를 닫은 뒤 완료로 보기까지 기다리는 시간(s). 물체를 잡지 않으므로 짧게 둔다.
# 닫기(약 1 s) + 이 값이 스윙 시간보다 짧아야 다음 펴기가 기다리지 않는다.
WAVE_GRIPPER_SETTLE_SEC = 0.3


def wave_both_arms(count: int = 3) -> RunnableTaskDefinition:
    """그리퍼를 닫고 정면을 본 채 양팔을 든 뒤 ``count`` 번 왼쪽-오른쪽으로 흔든다.

    왼쪽 스윙 동안 몸이 일어서며 그리퍼를 펴고, 오른쪽 스윙 동안 ready_pose 높이로
    내려오며 그리퍼를 오므린다. 닫힌 채 중립 자세(정면, ready_pose 높이)에서 끝난다.
    """
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('count must be a positive integer')
    neutral = {
        'torso': WAVE_TORSO_DOWN,
        'right_arm': WAVE_RIGHT_ARM_NEUTRAL,
        'left_arm': WAVE_LEFT_ARM_NEUTRAL,
    }
    left_up = {
        'torso': WAVE_TORSO_UP,
        'right_arm': WAVE_RIGHT_ARM_LEFT,
        'left_arm': WAVE_LEFT_ARM_LEFT,
    }
    right_down = {
        'torso': WAVE_TORSO_DOWN,
        'right_arm': WAVE_RIGHT_ARM_RIGHT,
        'left_arm': WAVE_LEFT_ARM_RIGHT,
    }
    task = Task('wave_both_arms')
    task.close_gripper('both', settle_time_sec=WAVE_GRIPPER_SETTLE_SEC)
    task.joint_absolute_multi(neutral, WAVE_APPROACH_JOINT_MOTION)
    for _ in range(count):
        task.open_gripper_background('both')
        task.joint_absolute_multi(left_up, WAVE_SWING_JOINT_MOTION)
        task.close_gripper_background(
            'both',
            settle_time_sec=WAVE_GRIPPER_SETTLE_SEC,
        )
        task.joint_absolute_multi(right_down, WAVE_SWING_JOINT_MOTION)
    task.joint_absolute_multi(neutral, WAVE_SWING_JOINT_MOTION)
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


# ---------------------------------------------------------------------------
# ZSSW -> AP -> ZS 이동 후 왼손으로 tag_3 기준 내려놓기
#
# 좌표(settings.yaml): ZSSW = 원점, AP = base -x 0.90 m, ZS = AP 에서 base +y 0.80 m.
# ZSSW/AP 는 side left(허리 +90°), ZS 는 side front 이므로 AP -> ZS 이동 직전에
# turn_and_move 가 object_gripping_initial_pose_front 로 허리를 0° 로 돌린다(팔은 그대로).
# 내려놓기는 기존 pick and place(put_down_object_at, arm='left_arm')와 같다: D435 로 tag_3 을 보고
# base 를 d405_left_camera_center 기준으로 정렬한 뒤 d405_left 로 접근·접촉하고 그리퍼를 연다.
# ---------------------------------------------------------------------------


# ZS 에서 머리 J2(head_1, pitch) 를 이만큼 돌렸다가 내려놓기 후 되돌린다 (deg).
ZS_HEAD_PITCH_OFFSET_DEG = 15.0
# 머리 관절 이동 설정: [최소 시간 s, 속도 rad/s, 가속도 rad/s^2]
# UI Joint 탭과 같은 값 (최소 시간 기본 3.0 s, manual_joint_velocity/acceleration_limit 0.2).
# control 노드는 head 가속도를 manual_joint_acceleration_limit(0.2) 이하만 허용한다.
ZS_HEAD_JOINT_MOTION = (3.0, 0.2, 0.2)


def move_zssw_to_zs_and_put_down() -> RunnableTaskDefinition:
    global current_position
    previous_position = current_position  # 다른 Task 의 current_position 에 영향 주지 않도록 복원
    try:
        task = Task('move_zssw_to_zs_and_put_down')
        task.extend(turn_and_move('ZSSW', 'AP'))
        task.extend(turn_and_move('AP', 'ZS'))
        # 머리 J2(head_1, pitch) +15° (tag_3 을 D435 로 보기 위해). 끝나면 -15° 로 원상복구.
        task.joint_relative('head', [0.0, ZS_HEAD_PITCH_OFFSET_DEG], ZS_HEAD_JOINT_MOTION)
        # 기존 pick and place 와 같은 방식: tag_3(settings.yaml ZS) 기준 왼손 내려놓기.
        # arm='left_arm' 이면 그리퍼(left)와 손목 카메라(d405_left / d405_left_camera_center)가 자동으로 정해진다.
        task.extend(put_down_object_at('ZS', arm='left_arm'))
        task.joint_relative('head', [0.0, -ZS_HEAD_PITCH_OFFSET_DEG], ZS_HEAD_JOINT_MOTION)
        return task.build()
    finally:
        current_position = previous_position
 

def move_zs_to_zssw() -> RunnableTaskDefinition:
    """ZS -> AP -> ZSSW 로 돌아온다 (move_zssw_to_zs_and_put_down 의 역경로).

    ZS(front) -> AP(left) 이므로 ZS 에서 출발하기 전에 turn_and_move 가
    object_gripping_initial_pose_torso_left 로 허리를 +90° 로 돌린다.
    """
    global current_position
    previous_position = current_position  # 다른 Task 의 current_position 에 영향 주지 않도록 복원
    try:
        task = Task('move_zs_to_zssw')
        task.extend(turn_and_move('ZS', 'AP'))
        task.extend(turn_and_move('AP', 'ZSSW'))
        return task.build()
    finally:
        current_position = previous_position

def push_switch_and_close_door_move_to_zs() -> RunnableTaskDefinition:
    global current_position
    previous_position = current_position  # 다른 Task 의 current_position 에 영향 주지 않도록 복원
    try:
        task = Task('push_switch_and_close_door_move_to_zs')
        tcp_motion = [2.0, 0.1, 0.3, 0.5]
        task.extend(open_door_and_push_switch2())
        task.linear_relative(SWITCH_DOOR_ARM, (0.0, -0.25, 0.0, 0.0, 0.0, 0.0), tcp_motion)
        task.extend(move_zssw_to_zs_and_put_down())
        # move_zs_to_zssw() 를 풀어 쓴 것: AP 도착 후 오른손을 -y 0.12 옮긴 자세로 ZSSW 까지 간다.
        # (ZS -> AP 의 torso_left 자세가 팔을 덮어쓰므로 그 뒤에 이동한다. AP -> ZSSW 는 둘 다
        #  side left 라 허리/팔을 다시 움직이지 않는다.)
        task.extend(turn_and_move('ZS', 'AP'))
        task.extend(pose_before_base_movement())
        # task.linear_relative(SWITCH_DOOR_ARM, (0.0, -0.12, 0.0, 0.0, 0.0, 0.0), tcp_motion)
        # task.linear_relative(SWITCH_DOOR_ARM, (0.07, 0.0, 0.0, 0.0, 0.0, 0.0), tcp_motion)
        task.extend(turn_and_move('AP', 'ZSSW'))
        task.extend(align_base_to_tag7())
        task.extend(push_switch_and_close_door())
        return task.build()
    finally:
        current_position = previous_position


# ---------------------------------------------------------------------------
# test1: 왼손 좌우 픽앤플레이스 반복 (base/허리 고정, 왼팔만 이동)
#
# 허리는 object_gripping_initial_pose_torso_left(+90°)로 고정. 가슴이 base +y 를 본다.
# tag_0 은 tag_3 의 가슴 기준 왼쪽에 둔다. 허리 +90° 에서 가슴 기준 왼쪽 = base -x.
# linear_relative 는 base 좌표이므로 -x 로 옮긴다. 물체를 든 높이 그대로 옆으로 옮긴다.
# (settings.yaml 의 LH1/LH2 좌표 배치와는 반대 방향이다.)
# ---------------------------------------------------------------------------
TEST1_SHIFT_MOTION = (3.0, 0.1, 0.3, 0.5)
TEST1_RUNS = 10                      # 첫 실행 포함 총 반복 횟수


def test1() -> RunnableTaskDefinition:
    """허리 왼쪽 고정. 왼팔: tag_3 잡기 -> 왼쪽(-x) 15 cm -> tag_0 놓기/다시 잡기 -> 오른쪽(+x) 15 cm -> tag_3 놓기, 10회 반복."""
    task = Task('test1')

    task.extend(object_gripping_initial_pose_torso_left())

    task.extend(pick_up_object_in_place('tag_3', arm='left_arm'))
    task.linear_relative('left_arm', (-0.25, 0.0, 0.0, 0.0, 0.0, 0.0), TEST1_SHIFT_MOTION)  # 왼쪽으로
    task.extend(put_down_object_in_place('tag_0', arm='left_arm'))

    task.extend(pick_up_object_in_place('tag_0', arm='left_arm'))
    task.linear_relative('left_arm', (0.15, 0.0, 0.0, 0.0, 0.0, 0.0), TEST1_SHIFT_MOTION)  # 오른쪽으로 복귀
    task.extend(put_down_object_in_place('tag_3', arm='left_arm'))
    
    task.rewind(TEST1_RUNS)
    return task.build()


def build_tasks() -> dict[str, RunnableTaskDefinition]:
    return {
        'impedance_demo_press': impedance_demo_press(),
        'torso_up': torso_up(),
        'impedance_demo_joint': impedance_demo_joint(),
        
        'object_gripping_initial_pose': object_gripping_initial_pose(),
        'object_gripping_initial_pose_torso_right': object_gripping_initial_pose_torso_right(),
        'object_gripping_initial_pose_torso_left': object_gripping_initial_pose_torso_left(),
        
        'ready_pose': ready_pose(),

        'object_handover_demo_final': object_handover_demo_final(),
        'object_handover_demo_final_tmp': object_handover_demo_final_tmp(),
        'test1': test1(),
        'door_handle_grasp_pose': door_handle_grasp_pose(),
        'door_open_right_arm_stream': door_open_right_arm_stream(),

        'wave_both_arms': wave_both_arms(),
        
        'align_base_to_tag6': align_base_to_tag6(),
        'align_base_to_tag7': align_base_to_tag7(),
        'search_pose': search_pose(),
        'search_switch_pose': search_switch_pose(),
        'move_ee_to_tag6_offset': move_ee_to_tag6_offset(),
        'search_and_push_switch': search_and_push_switch(),
        'search_and_open_door2': search_and_open_door2(),
        'open_door_and_push_switch2': open_door_and_push_switch2(),
        'push_switch_and_close_door': push_switch_and_close_door(),
        'object_gripping_initial_pose_front': object_gripping_initial_pose_front(),
        'move_zssw_to_zs_and_put_down': move_zssw_to_zs_and_put_down(),
        'move_zs_to_zssw': move_zs_to_zssw(),
        'push_switch_and_close_door_move_to_zs': push_switch_and_close_door_move_to_zs(),
    }

__all__ = [
    'Task', 'build_tasks', 'torso_up'
    'JS_SOFT_ARM', 'JS_CONTACT_ARM', 'impedance_demo_press', 'search_pose',
    'ready_pose',
    'impedance_demo_joint',
    'turn_and_move',
    'object_gripping_initial_pose',
    'ready_pose',
    'object_gripping_initial_pose_torso_right',
    'object_gripping_initial_pose_torso_left',
    'pick_up_object_at',
    'put_down_object_at',
    'pick_up_move_and_put_down_object',
    'pick_up_move_and_put_down_object2',
    'object_handover_demo_final',
    'door_handle_grasp_pose',
    'door_open_right_arm_stream',
    'wave_both_arms',
    'align_base_to_tag6',
    'search_switch_pose',
    'move_ee_to_tag6_offset',
    'search_and_push_switch',
    'search_and_open_door2',
    'open_door_and_push_switch2',
    'push_switch_and_close_door'
]
