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
def torso_rotation() -> Task:
    task = Task('torso_rotation')
    task.joint_absolute('torso', (0.065071, 45.375357, -90.960025, 45.153579, 0.027261, -90.0), (5.0, 1.0, 1.0))
    return task


# 감지 물체 기준 왼팔 말단 이동
def move_left_ee_to_detected_object(
    *, object_id: str, tcp_motion: Sequence[object], object_to_ee_position: Sequence[object],
    object_to_ee_orientation_xyzw: Sequence[object], detection_timeout_sec: float = 3.0,
    max_age_sec: float | None = None, minimum_confidence: float | None = None,
) -> RunnableTaskDefinition:
    task = Task('move_left_ee_to_detected_object')
    task.delay(2000)
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
    task.delay(2000)
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


# 태그 1회 감지 기반 접근 및 base-frame Z 접촉 이동
def _configured_tag_approach_contact_move(object_id: str, approach_offset: Sequence[object], contact_offset: Sequence[object], tcp_motion: Sequence[object]) -> RunnableTaskDefinition:
    approach = tuple(float(value) for value in approach_offset)
    contact = tuple(float(value) for value in contact_offset)
    if len(approach) != 3 or len(contact) != 3:
        raise ValueError('approach/contact offsets must contain 3 numbers')
    if approach[:2] != contact[:2]:
        raise ValueError('approach/contact X and Y offsets must match for a Z-only move')

    task = Task(f'move_{object_id}_approach_contact')
    task.delay(2000)
    task.camera_linear_absolute(
        'left_arm', object_id, tcp_motion,
        detection_timeout_sec=3.0,
        object_to_end_effector_position=approach,
        object_to_end_effector_orientation_xyzw=_OBJECT_TO_LEFT_EE_ORIENTATION_XYZW,
    )
    task.linear_relative('left_arm', (0.0, 0.0, contact[2] - approach[2], 0.0, 0.0, 0.0), tcp_motion)
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
    task.extend(torso_rotation())
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


# tag_0 파지 후 tag_4 배치
def object_handover_demo4() -> RunnableTaskDefinition:
    task = Task('object_handover_demo4')

    ready_left_arm = [-33.643824, 63.218668, -44.626985, -106.563932, -17.501234, 44.556740, -84.095907]

    ready_joint_motion = [3.0, 1.0, 1.0]
    tcp_motion = [5.0, 1.0, 1.0, 1.0]
    pickup_motion = [5.0, 0.20, 0.20, 0.30]

    move_y = 0.25 
    pickup_distance = 0.10
    tag4_contact_offset = [0.0, 0.0, 0.05]
    tag4_approach_offset = list_sum(tag4_contact_offset, [0.0, 0.0, pickup_distance])

    tag0_contact_offset = [0.0, 0.0, 0.07]
    tag0_approach_offset = list_sum(tag0_contact_offset, [0.0, 0.0, pickup_distance])

    # 준비
    task.extend(object_gripping_initial_pose())

    # tag_4 기준 파지
    task.open_gripper('left')
    task.extend(_configured_tag_approach_contact_move('tag_4', tag4_approach_offset, tag4_contact_offset, pickup_motion))
    task.close_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)

    # tag_0 기준 배치
    task.linear_relative('left_arm', (0.0, move_y, 0.0, 0.0, 0.0, 0.0), tcp_motion)
    task.extend(_configured_tag_approach_contact_move('tag_0', tag0_approach_offset, tag0_contact_offset, pickup_motion))
    task.open_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)

    # 제자리
    task.joint_absolute('left_arm', ready_left_arm, ready_joint_motion)

    return task.build()


# Isaac Sim 왼쪽 그리퍼 동작 시험
def left_gripper_sim_test() -> RunnableTaskDefinition:
    task = Task('left_gripper_sim_test')
    task.open_gripper('left')
    task.delay(500)
    task.set_gripper('left', 0.6)
    task.delay(500)
    task.close_gripper('left')
    task.delay(500)
    task.open_gripper('left')
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
        'move_left_ee_to_detected_object': _configured_detected_object_move(),
        'object_gripping_initial_pose_and_print_tcp': object_gripping_initial_pose_and_print_tcp(),
        'object_handover_demo2': object_handover_demo2(),
        'object_handover_demo': object_handover_demo(),
        'object_handover_demo3': object_handover_demo3(),
        'object_handover_demo4': object_handover_demo4(),
        'object_gripping_initial_pose': object_gripping_initial_pose().build(),
        'ready_pose': ready_pose().build(),
        'object_gripping_initial_pose_and_move': object_gripping_initial_pose_and_move(),
        'left_gripper_sim_test': left_gripper_sim_test(),
    }


__all__ = [
    'Task', 'build_tasks', 'left_gripper_sim_test',
    'move_left_ee_to_detected_object', 'move_left_ee_to_detected_object_print',
    'object_gripping_initial_pose', 'object_gripping_initial_pose_and_print_tcp',
    'object_handover_demo2', 'object_handover_demo', 'object_handover_demo3', 'object_handover_demo4',
    'torso_rotation', 'ready_pose',
    'object_gripping_initial_pose_and_move',
]
