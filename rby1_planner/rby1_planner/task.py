# Planner Scenario UI용 Task 정의
from __future__ import annotations

from typing import Sequence

from .task_commands import RunnableTaskDefinition, Task, list_sum


# Task 작성 단위: 초, 미터, 도
# TCP 움직임 설정: [최소 시간, 선속도, 각속도, 가속도 비율]
_PICKUP_TCP_MOTION = (5.0, 0.20, 0.20, 0.30)
_PICKUP_DISTANCE_M = 0.07

# tag_0 기준 ee_left 7 cm 상단 접근 설정
# 실물 적용 전 object<-EE 파지 변환 보정 필요
_OBJECT_TO_LEFT_EE_POSITION = (0.0, 0.0, _PICKUP_DISTANCE_M)
_OBJECT_TO_LEFT_EE_ORIENTATION_XYZW = (0.0, 0.0, 0.0, 1.0)


# 물체 파지용 초기 전신 자세
def object_gripping_initial_pose() -> Task:
    task = Task('object_gripping_initial_pose')
    task.whole_body_joint_absolute(
        torso=(0.059126, 45.376237, -90.963053, 45.154560, 0.021390, -0.000280),
        right_arm=(-34.913810, -63.837794, 46.172754, -106.640370, 16.002640, 45.214678, 83.061208),
        left_arm=(-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907),
        joint_motion=(5.0, 1.0, 1.0),
    )
    return task


# 물체 배치용 몸통 회전
def torso_rotation_right() -> Task:
    task = Task('torso_rotation_right')
    task.joint_absolute('torso', (0.065071, 45.375357, -90.960025, 45.153579, 0.027261, -90.0), (5.0, 1.0, 1.0))
    return task.build()

def torso_rotation_left() -> Task:
    task = Task('torso_rotation_left')
    task.joint_absolute('torso', (0.065071, 45.375357, -90.960025, 45.153579, 0.027261, 90.0), (5.0, 1.0, 1.0))
    return task.build()

# 감지 물체 기준 왼팔 말단 이동
def move_left_ee_to_detected_object(
    *, object_id: str, tcp_motion: Sequence[object], object_to_ee_position: Sequence[object],
    object_to_ee_orientation_xyzw: Sequence[object], detection_timeout_sec: float = 3.0,
    max_age_sec: float | None = None, minimum_confidence: float | None = None,
) -> RunnableTaskDefinition:
    task = Task('move_left_ee_to_detected_object')
    task.camera_linear_absolute(
        'left_arm', object_id, tcp_motion, detection_timeout_sec=detection_timeout_sec,
        max_age_sec=max_age_sec, minimum_confidence=minimum_confidence,
        object_to_end_effector_position=object_to_ee_position,
        object_to_end_effector_orientation_xyzw=object_to_ee_orientation_xyzw,
    )
    return task.build()


# 감지 물체 기준 왼팔 말단 목표 자세 출력
def move_left_ee_to_detected_object_print(
    *, object_id: str, tcp_motion: Sequence[object], object_to_ee_position: Sequence[object],
    object_to_ee_orientation_xyzw: Sequence[object], detection_timeout_sec: float = 3.0,
    max_age_sec: float | None = None, minimum_confidence: float | None = None,
) -> RunnableTaskDefinition:
    task = Task('move_left_ee_to_detected_object_print')
    task.camera_linear_absolute_print(
        'left_arm', object_id, tcp_motion, detection_timeout_sec=detection_timeout_sec,
        max_age_sec=max_age_sec, minimum_confidence=minimum_confidence,
        object_to_end_effector_position=object_to_ee_position,
        object_to_end_effector_orientation_xyzw=object_to_ee_orientation_xyzw,
    )
    return task.build()


# tag_0 기준 왼팔 말단 이동 설정
def _configured_detected_object_move() -> RunnableTaskDefinition:
    return move_left_ee_to_detected_object(
        object_id='tag_0', tcp_motion=_PICKUP_TCP_MOTION,
        object_to_ee_position=_OBJECT_TO_LEFT_EE_POSITION,
        object_to_ee_orientation_xyzw=_OBJECT_TO_LEFT_EE_ORIENTATION_XYZW,
        detection_timeout_sec=3.0,
    )


# tag_0 기준 왼팔 말단 목표 출력 설정
def _configured_detected_object_print() -> RunnableTaskDefinition:
    return move_left_ee_to_detected_object_print(
        object_id='tag_0', tcp_motion=_PICKUP_TCP_MOTION,
        object_to_ee_position=_OBJECT_TO_LEFT_EE_POSITION,
        object_to_ee_orientation_xyzw=_OBJECT_TO_LEFT_EE_ORIENTATION_XYZW,
        detection_timeout_sec=3.0,
    )


# 태그 기준 왼팔 말단 이동 설정
def _configured_tag_offset_move(object_id: str, position_offset: Sequence[object], tcp_motion: Sequence[object]) -> RunnableTaskDefinition:
    return move_left_ee_to_detected_object(
        object_id=object_id, tcp_motion=tcp_motion,
        object_to_ee_position=position_offset,
        object_to_ee_orientation_xyzw=_OBJECT_TO_LEFT_EE_ORIENTATION_XYZW,
        detection_timeout_sec=3.0,
    )


# 태그를 새로 감지하고 현재 base 기준 태그/TCP 좌표 출력
def _configured_tag_print(object_id: str, tcp_motion: Sequence[object]) -> RunnableTaskDefinition:
    return move_left_ee_to_detected_object_print(
        object_id=object_id, tcp_motion=tcp_motion,
        object_to_ee_position=(0.0, 0.0, 0.0),
        object_to_ee_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        detection_timeout_sec=3.0,
    )


# 태그 1회 감지 기반 접근 및 절대 좌표 접촉 이동
def _configured_tag_approach_contact_move(object_id: str, approach_offset: Sequence[object], contact_offset: Sequence[object], tcp_motion: Sequence[object]) -> RunnableTaskDefinition:
    approach = tuple(float(value) for value in approach_offset)
    contact = tuple(float(value) for value in contact_offset)
    if len(approach) != 3 or len(contact) != 3:
        raise ValueError('approach/contact offsets must contain 3 numbers')
    task = Task(f'move_{object_id}_approach_contact')
    task.delay(2000)
    task.camera_linear_absolute(
        'left_arm', object_id, tcp_motion,
        detection_timeout_sec=3.0,
        yaw_only=True,
        yaw_symmetry_deg=180.0,
        log_target_and_actual=True,
        object_to_end_effector_position=approach,
        object_to_end_effector_orientation_xyzw=_OBJECT_TO_LEFT_EE_ORIENTATION_XYZW,
    )
    task.camera_linear_absolute(
        'left_arm', object_id, tcp_motion,
        detection_timeout_sec=3.0,
        reuse_previous_observation=True,
        yaw_only=True,
        yaw_symmetry_deg=180.0,
        log_target_and_actual=True,
        object_to_end_effector_position=contact,
        object_to_end_effector_orientation_xyzw=_OBJECT_TO_LEFT_EE_ORIENTATION_XYZW,
    )
    return task.build()

# 파지 초기 자세 이동 후 왼팔 목표 자세 출력
def object_gripping_initial_pose_and_print_tcp() -> RunnableTaskDefinition:
    task = Task('object_gripping_initial_pose_and_print_tcp')
    task.extend(object_gripping_initial_pose())
    task.extend(_configured_detected_object_print())
    return task.build()


# 파지 초기 자세 이동 후 감지 물체 접근
def object_gripping_initial_pose_and_move() -> RunnableTaskDefinition:
    task = Task('object_gripping_initial_pose_and_move')
    task.extend(object_gripping_initial_pose())
    task.extend(_configured_detected_object_move())
    return task.build()


# 고정 좌표 기반 왼팔 물체 파지 및 전달 위치 배치
def object_handover_demo() -> RunnableTaskDefinition:
    task = Task('object_handover_demo')
    task.extend(object_gripping_initial_pose())

    ready_left_arm = [-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907]
    ready_joint_motion = [3.0, 1.0, 1.0]
    tcp_motion = [5.0, 1.0, 1.0, 1.0]
    pickup_distance = 0.10

    # 측정 초기 관절각 FK 결과: [x, y, z, roll, pitch, yaw]
    ready_left_tcp = [0.374034, 0.243053, 0.921057, 0.008670, 0.007028, -89.997544]
    left_facing_right_rpy = [-19.257234, -89.992555, -70.734096]
    handover_half_gap = 0.08
    left_handover_tcp = [ready_left_tcp[0], handover_half_gap, ready_left_tcp[2], *left_facing_right_rpy]

    task.open_gripper('left')
    task.linear_absolute('left_arm', list_sum(ready_left_tcp, [0.0, 0.0, -pickup_distance, 0.0, 0.0, 0.0]), tcp_motion)
    task.close_gripper('left')
    task.linear_absolute('left_arm', ready_left_tcp, tcp_motion)

    task.linear_absolute('left_arm', [*ready_left_tcp[:3], *left_facing_right_rpy], tcp_motion)
    task.linear_absolute('left_arm', left_handover_tcp, tcp_motion)
    task.open_gripper('left')
    task.linear_absolute('left_arm', [*ready_left_tcp[:3], *left_facing_right_rpy], tcp_motion)
    task.joint_absolute('left_arm', ready_left_arm, ready_joint_motion)
    return task.build()


# 카메라 기반 왼팔 물체 파지, 회전, 배치
def object_handover_demo2() -> RunnableTaskDefinition:
    task = Task('object_handover_demo2')
    task.extend(object_gripping_initial_pose())

    # 초기 전신 자세 완료 후 카메라 조회
    task.extend(_configured_detected_object_move())
    task.close_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, _PICKUP_DISTANCE_M, 0.0, 0.0, 0.0), _PICKUP_TCP_MOTION)
    task.linear_relative('left_arm', (0.0, 0.0, -_PICKUP_DISTANCE_M, 0.0, 0.0, 0.0), _PICKUP_TCP_MOTION)
    task.open_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, _PICKUP_DISTANCE_M, 0.0, 0.0, 0.0), _PICKUP_TCP_MOTION)
    return task.build()


# 고정 좌표 기반 왼팔 물체 파지 및 중앙 배치
def object_handover_demo3() -> RunnableTaskDefinition:
    task = Task('object_handover_demo3')

    task.extend(object_gripping_initial_pose())

    ready_left_arm = [-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907]
    ready_left_tcp = [0.374034, 0.243053, 0.921057, 0.008670, 0.007028, -89.997544]
    move_y_left_tcp = list_sum(ready_left_tcp, [0.0, 0.15, 0.0, 0.0, 0.0, 0.0])

    ready_joint_motion = [3.0, 1.0, 1.0]
    tcp_motion = [5.0, 1.0, 1.0, 1.0]
    pickup_distance = 0.10

    # 측정 초기 관절각 FK 결과: [x, y, z, roll, pitch, yaw]
    # handover_left_tcp = [ready_left_tcp[0], 0.08, ready_left_tcp[2], *ready_left_tcp[3:]]
    # 파지
    task.open_gripper('left')
    task.linear_absolute('left_arm', list_sum(ready_left_tcp, [0.0, 0.0, -pickup_distance, 0.0, 0.0, 0.0]), tcp_motion)
    task.close_gripper('left')
    task.linear_absolute('left_arm', list_sum(ready_left_tcp, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), tcp_motion)

    # y 방향 이동후 배치
    task.linear_absolute('left_arm', list_sum(move_y_left_tcp, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), tcp_motion)
    task.linear_absolute('left_arm', list_sum(move_y_left_tcp, [0.0, 0.0, -pickup_distance, 0.0, 0.0, 0.0]), tcp_motion)
    task.open_gripper('left')
    task.linear_absolute('left_arm', list_sum(move_y_left_tcp, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), tcp_motion)
    task.joint_absolute('left_arm', ready_left_arm, ready_joint_motion)
    return task.build()


# tag_0 파지 후 tag_4 배치 카메라 사용
def object_handover_demo4() -> RunnableTaskDefinition:
    task = Task('object_handover_demo4')

    ready_left_arm = [-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907]
    ready_left_tcp = [0.374034, 0.243053, 0.921057, 0.008670, 0.007028, -89.997544] # 허리 정면    
    # ready_left_tcp = [0.241868, -0.376696, 0.919114, -0.516431, 0.357345, -179.997283] # 허리우측 회전 

    ready_joint_motion = [3.0, 2.0, 2.0]
    tcp_motion = [2.0, 2.0, 2.0, 1.0]
    pickup_motion = [3.0, 1.0, 1.0, 1.0]

    move_x = 0.25 
    pickup_distance = 0.075
    tag4_contact_offset = [0.0, 0.0, 0.05]
    tag4_approach_offset = list_sum(tag4_contact_offset, [0.0, 0.0, pickup_distance])

    tag0_contact_offset = [0.0, 0.0, 0.0675]
    tag0_approach_offset = list_sum(tag0_contact_offset, [0.0, 0.0, pickup_distance])

    # 준비
    # task.extend(object_gripping_initial_pose())
    task.linear_absolute('left_arm', list_sum(ready_left_tcp, [0.0, 0.0, 0.00, 0.0, 0.0, 0.0]), tcp_motion)
    task.linear_absolute('left_arm', list_sum(ready_left_tcp, [0.0, 0.0, -0.05, 0.0, 0.0, 0.0]), tcp_motion)

    # tag_4 기준 파지
    task.open_gripper('left')
    task.extend(_configured_tag_approach_contact_move('tag_4', tag4_approach_offset, tag4_contact_offset, pickup_motion))
    task.close_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)

    # tag_0 기준 배치
    task.linear_relative('left_arm', (move_x, 0.0, 0.0, 0.0, 0.0, 0.0), tcp_motion)
    task.extend(_configured_tag_approach_contact_move('tag_0', tag0_approach_offset, tag0_contact_offset, pickup_motion))
    task.open_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)

    # 제자리
    task.joint_absolute('left_arm', ready_left_arm, ready_joint_motion)

    return task.build()


# tag_4 PnP 위치 편향 확인: 중앙, 오른쪽 5 mm, 왼쪽 5 mm에서 각각 측정
def object_handover_demo5() -> RunnableTaskDefinition:
    task = Task('object_handover_demo5')

    ready_left_arm = [-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907]
    # ready_left_tcp = [0.374034, 0.243053, 0.921057, 0.008670, 0.007028, -89.997544] # 허리 정면    
    ready_left_tcp = [0.241868, -0.376696, 0.919114, -0.516431, 0.357345, -179.997283] # 허리우측 회전 

    ready_joint_motion = [3.0, 1.0, 1.0]
    tcp_motion = [5.0, 1.0, 1.0, 1.0]

    # 준비
    # task.extend(object_gripping_initial_pose())
    task.linear_absolute('left_arm', list_sum(ready_left_tcp, [0.0, 0.0, 0.00, 0.0, 0.0, 0.0]), tcp_motion)
    task.linear_absolute('left_arm', list_sum(ready_left_tcp, [0.0, 0.0, -0.05, 0.0, 0.0, 0.0]), tcp_motion)

    # 시작 위치에서 tag_4 새 측정 및 출력
    task.extend(_configured_tag_print('tag_4', tcp_motion))

    # ROS base 기준 오른쪽(-Y) 5 mm 이동 후 새 측정 및 출력
    task.linear_relative('left_arm', (0.0, -0.05, 0.0, 0.0, 0.0, 0.0), tcp_motion)
    task.extend(_configured_tag_print('tag_4', tcp_motion))

    # 현재 위치에서 왼쪽(+Y) 10 mm 이동 후 새 측정 및 출력
    task.linear_relative('left_arm', (0.0, 0.10, 0.0, 0.0, 0.0, 0.0), tcp_motion)
    task.extend(_configured_tag_print('tag_4', tcp_motion))

    # 현재 yaw 기준 +30도, -30도에서 각각 새 측정 및 출력
    task.linear_relative('left_arm', (0.0, 0.0, 0.0, 0.0, 0.0, 10.0), tcp_motion)
    task.extend(_configured_tag_print('tag_4', tcp_motion))
    task.linear_relative('left_arm', (0.0, 0.0, 0.0, 0.0, 0.0, -20.0), tcp_motion)
    task.extend(_configured_tag_print('tag_4', tcp_motion))

    # 제자리
    task.joint_absolute('left_arm', ready_left_arm, ready_joint_motion)
    return task.build()

def object_handover_demo6() -> RunnableTaskDefinition:
    task = Task('object_handover_demo6')

    ready_left_arm = [-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907]
    # ready_left_tcp = [0.374034, 0.243053, 0.921057, 0.008670, 0.007028, -89.997544] # 허리 정면    
    ready_left_tcp_RS = [0.241868, -0.376696, 0.919114, -0.516431, 0.357345, -179.997283] # 허리 우측 회전 
    ready_left_tcp_LS = [-0.245620, 0.375166, 0.916662, -0.331338, -0.504770, 0.002247] # 허리 좌측 회전 

    ready_joint_motion = [3.0, 2.0, 2.0]
    tcp_motion = [2.0, 2.0, 2.0, 1.0]
    pickup_motion = [3.0, 1.0, 1.0, 1.0]

    pickup_distance = 0.075
    tag4_contact_offset = [0.0, 0.0, 0.05]
    tag4_approach_offset = list_sum(tag4_contact_offset, [0.0, 0.0, pickup_distance])

    tag0_contact_offset = [0.0, 0.0, 0.065]
    tag0_approach_offset = list_sum(tag0_contact_offset, [0.0, 0.0, pickup_distance])

    tag2_contact_offset = [0.0, 0.0, 0.065]
    tag2_approach_offset = list_sum(tag2_contact_offset, [0.0, 0.0, pickup_distance]) 

    # 준비
    task.extend(torso_rotation_right())
    task.joint_absolute('left_arm', ready_left_arm, ready_joint_motion)

    # 오른쪽에서 tag_4 기준 파지
    task.linear_absolute('left_arm', list_sum(ready_left_tcp_RS, [0.0, 0.0, 0.00, 0.0, 0.0, 0.0]), tcp_motion)
    task.linear_absolute('left_arm', list_sum(ready_left_tcp_RS, [0.0, 0.0, -0.05, 0.0, 0.0, 0.0]), tcp_motion)
    task.open_gripper('left')
    task.extend(_configured_tag_approach_contact_move('tag_4', tag4_approach_offset, tag4_contact_offset, pickup_motion))
    task.close_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)
    
    # 왼쪽 이동 준비 및 이동
    task.extend(torso_rotation_left())
    task.move_to(0.0, 0.75, 0.0, timeout_sec=15.0)
    task.delay(1000)

    # 왼쪽에서 tag_0 기준 배치
    task.linear_absolute('left_arm', list_sum(ready_left_tcp_LS, [0.0, 0.0, 0.00, 0.0, 0.0, 0.0]), tcp_motion)
    task.linear_absolute('left_arm', list_sum(ready_left_tcp_LS, [0.0, 0.0, -0.05, 0.0, 0.0, 0.0]), tcp_motion)
    task.extend(_configured_tag_approach_contact_move('tag_0', tag0_approach_offset, tag0_contact_offset, pickup_motion))
    task.open_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)

    # 제자리
    task.joint_absolute('left_arm', ready_left_arm, ready_joint_motion)
    
    # 왼쪽에서 tag_4 기준 파지
    task.linear_absolute('left_arm', list_sum(ready_left_tcp_LS, [0.0, 0.0, 0.00, 0.0, 0.0, 0.0]), tcp_motion)
    task.linear_absolute('left_arm', list_sum(ready_left_tcp_LS, [0.0, 0.0, -0.05, 0.0, 0.0, 0.0]), tcp_motion)
    task.open_gripper('left')
    task.extend(_configured_tag_approach_contact_move('tag_4', tag4_approach_offset, tag4_contact_offset, pickup_motion))
    task.close_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)

    # 오른쪽 이동 준비 및 이동
    task.extend(torso_rotation_right())
    task.move_to(0.0, -0.75, 0.0, timeout_sec=15.0)
    task.delay(1000)

    # 오른쪽에서 tag_2 기준 배치
    task.linear_absolute('left_arm', list_sum(ready_left_tcp_RS, [0.0, 0.0, 0.00, 0.0, 0.0, 0.0]), tcp_motion)
    task.linear_absolute('left_arm', list_sum(ready_left_tcp_RS, [0.0, 0.0, -0.05, 0.0, 0.0, 0.0]), tcp_motion)
    task.extend(_configured_tag_approach_contact_move('tag_2', tag2_approach_offset, tag2_contact_offset, pickup_motion))
    task.open_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)

    # 제자리
    task.joint_absolute('left_arm', ready_left_arm, ready_joint_motion)

    # Stop/EMO/failure가 발생할 때까지 demo6 전체를 무한 반복
    task.rewind(10)

    return task.build()


# D435로 태그를 거칠게 정렬한 뒤 D405 카메라 중심을 태그 위로 이동
def object_handover_demo7() -> RunnableTaskDefinition:
    task = Task('object_handover_demo7')

    target_tag = 'tag_4'
    desired_tag_x = 0.45
    desired_tag_y = 0.25
    d405_hover_height = 0.18

    ready_left_tcp = [
        0.374034,
        0.243053,
        0.921057,
        0.008670,
        0.007028,
        -89.997544,
    ]
    tcp_motion = [3.0, 0.20, 0.20, 0.30]

    # 팔과 몸통을 정면 안전 자세로 회수한 뒤 D435로 새 관측을 받는다.
    task.extend(object_gripping_initial_pose())
    task.move_base_to_detected_tag(
        camera_source='d435',
        object_id=target_tag,
        desired_tag_x=desired_tag_x,
        desired_tag_y=desired_tag_y,
        relative_yaw=0.0,
        detection_timeout_sec=5.0,
        navigation_timeout_sec=30.0,
        max_translation_m=0.8,
    )
    task.delay(1000)

    # base 이동 뒤 D405 관측 자세로 이동하고 반드시 새 D405 관측을 쓴다.
    task.linear_absolute('left_arm', ready_left_tcp, tcp_motion)
    task.camera_frame_linear_absolute(
        'left_arm',
        target_tag,
        tcp_motion,
        camera_source='d405',
        controlled_frame='d405_camera_center',
        detection_timeout_sec=5.0,
        yaw_only=True,
        preserve_end_effector_orientation=True,
        log_target_and_actual=True,
        object_to_controlled_frame_position=(
            0.0,
            0.0,
            d405_hover_height,
        ),
    )

    return task.build()

# 시작 자세 기준 오른쪽으로 10 cm 이동
def move_right_10cm() -> RunnableTaskDefinition:
    task = Task('move_right_10cm')
    task.move_to(0.0, 0.6, 0.0, timeout_sec=15.0)
    task.delay(1000)
    task.move_to(0.0, -0.6, 0.0, timeout_sec=15.0)
    task.delay(1000)

    # task.move_to(0.1, 0.0, 0.0, timeout_sec=10.0)
    # task.delay(1000)
    # task.move_to(-0.1, 0.0, 0.0, timeout_sec=10.0)
    # task.delay(1000)

    # task.move_to(0.1, 0.0, 0.0, timeout_sec=10.0)
    # task.delay(1000)
    # task.move_to(-0.1, 0.0, 0.0, timeout_sec=10.0)
    # task.delay(1000)

    # task.move_to(0.1, 0.0, 0.0, timeout_sec=10.0)
    # task.delay(1000)
    # task.move_to(-0.1, 0.0, 0.0, timeout_sec=10.0)
    # task.delay(1000)

    # task.move_to(0.1, 0.0, 0.0, timeout_sec=10.0)
    # task.delay(1000)
    # task.move_to(-0.1, 0.0, 0.0, timeout_sec=10.0)
    # task.delay(1000)  

    # task.move_to(0.1, 0.0, 0.0, timeout_sec=10.0)
    # task.move_to(-0.1, 0.0, 0.0, timeout_sec=10.0)
    return task.build()


# Isaac Sim 양쪽 그리퍼 동작 시험
def left_gripper_sim_test() -> RunnableTaskDefinition:
    task = Task('left_gripper_sim_test')
    task.open_gripper('both')
    task.delay(500)
    task.set_gripper('both', 0.6)
    task.delay(500)
    task.close_gripper('both')
    task.delay(500)
    task.open_gripper('both')
    return task.build()


# Cartesian 동작용 전신 준비 자세
def ready_pose() -> Task:
    task = Task('ready_pose')
    task.whole_body_joint_absolute(
        torso=[0.0, 45.0, -90.0, 45.0, 0.0, 0.0],
        right_arm=[0.0, -5.0, 0.0, -120.0, 0.0, 40.0, 0.0],
        left_arm=[0.0, 5.0, 0.0, -120.0, 0.0, 40.0, 0.0],
        joint_motion=[5.0, 0.20, 0.20],
    )
    return task


# Planner Scenario UI 표시 Task 목록
def build_tasks() -> dict[str, RunnableTaskDefinition]:
    return {
        'move_right_10cm': move_right_10cm(),
        'move_left_ee_to_detected_object': _configured_detected_object_move(),
        'object_gripping_initial_pose_and_print_tcp': object_gripping_initial_pose_and_print_tcp(),
        'object_handover_demo2': object_handover_demo2(),
        'object_handover_demo': object_handover_demo(),
        'object_handover_demo3': object_handover_demo3(),
        'object_handover_demo4': object_handover_demo4(),
        'object_handover_demo5': object_handover_demo5(),
        'object_handover_demo6': object_handover_demo6(),
        'object_handover_demo7': object_handover_demo7(),
        'object_gripping_initial_pose': object_gripping_initial_pose().build(),
        'ready_pose': ready_pose().build(),
        'object_gripping_initial_pose_and_move': object_gripping_initial_pose_and_move(),
        'left_gripper_sim_test': left_gripper_sim_test(),
        'torso_rotation_left': torso_rotation_left(),
        'torso_rotation_right': torso_rotation_right()
    }


__all__ = [
    'Task', 'build_tasks', 'left_gripper_sim_test',
    'move_left_ee_to_detected_object', 'move_left_ee_to_detected_object_print',
    'object_gripping_initial_pose', 'object_gripping_initial_pose_and_print_tcp',
    'object_handover_demo2', 'object_handover_demo', 'object_handover_demo3',
    'object_handover_demo4', 'object_handover_demo5', 'object_handover_demo6',
    'object_handover_demo7',
    'torso_rotation_left', 'torso_rotation_right', 'ready_pose',
    'object_gripping_initial_pose_and_move',
    'move_right_10cm',
]
