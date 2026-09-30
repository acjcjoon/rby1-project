# Planner Scenario UI용 Task 정의
from __future__ import annotations

from typing import Sequence

from .task_commands import (
    RunnableTaskDefinition,
    Task,
    _load_move_locations,
    _load_tag_approach_offset,
    _move_location,
    list_sum,
)


# Task 작성 단위: 초, 미터, 도
# TCP 움직임 설정: [최소 시간, 선속도, 각속도, 가속도 비율]
# tag_0 기준 ee_left 7 cm 상단 접근 설정
_OBJECT_TO_LEFT_EE_ORIENTATION_XYZW = (0.0, 0.0, 0.0, 1.0)

def ready_pose() -> Task:
    task = Task('ready_pose')
    task.whole_body_joint_absolute(
        torso=[0.0, 45.0, -90.0, 45.0, 0.0, 0.0],
        right_arm=[0.0, -5.0, 0.0, -120.0, 0.0, 40.0, 0.0],
        left_arm=[0.0, 5.0, 0.0, -120.0, 0.0, 40.0, 0.0],
        joint_motion=[5.0, 0.20, 0.20],
    )
    return task.build()

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
        joint_motion=(5.0, 2.0, 3.0),
    )
    return task.build()

def object_gripping_initial_pose_torso_right() -> Task:
    task = Task('object_gripping_initial_pose_torso_right')
    task.whole_body_joint_absolute(
        torso=(0.065048, 45.375276, -90.959835, 45.153373, 0.026976, -89.999782),
        right_arm=(-38.280283, -61.948097, 45.740109, -95.376122, 15.278687, 36.629077, 83.445557),
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

# 태그 1회 감지 기반 접근 및 절대 좌표 접촉 이동
def _configured_tag_approach_contact_move(object_id: str, approach_offset: Sequence[object], contact_offset: Sequence[object], tcp_motion: Sequence[object]) -> RunnableTaskDefinition:
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




def turn_and_move(start_point: str, end_point: str) -> RunnableTaskDefinition:
    locations = _load_move_locations()
    start_x, start_y, start_side = _move_location(locations, start_point)
    end_x, end_y, side = _move_location(locations, end_point)
    task = Task('turn_and_move')
    if start_side != side:
        if side == 'left':
            task.extend(object_gripping_initial_pose_torso_left())
        else:
            task.extend(object_gripping_initial_pose_torso_right())
    task.move_to(end_x - start_x, end_y - start_y, 0.0, timeout_sec=15.0)
    task.delay(1000)
    return task.build()


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

def align_base_and_put_down_object(
    object_id: str,
    *,
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
) -> RunnableTaskDefinition:
    """동일 태그로 base 정렬 후 접근·접촉하고 물체를 놓는다."""
    task = Task(f'align_base_and_put_down_object_{object_id}')
    tcp_motion = [3.0, 1.0, 1.0, 1.0]
    pickup_distance = 0.075
    contact_offset = [0.0, 0.0, 0.065]
    approach_offset = list_sum(contact_offset, [0.0, 0.0, pickup_distance])
    pickup_motion = [3.0, 1.0, 1.0, 1.0]

    task.move_base_to_detected_tag(
        camera_source='d435', object_id=object_id,
        desired_tag_x=0.0, desired_tag_y=0.0, desired_tag_frame='d405_camera_center',
        threshold_x_minus=threshold_x_minus, threshold_y_plus=threshold_y_plus,
        relative_yaw=0.0, detection_timeout_sec=5.0, navigation_timeout_sec=30.0,
        max_translation_m=0.8,
    )
    task.delay(1000)
    task.extend(_configured_tag_approach_contact_move(
        object_id, approach_offset, contact_offset, pickup_motion,
    ))
    task.open_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)
    task.extend(object_gripping_initial_pose_temp())
    return task.build()

def align_base_and_pick_up_object(
    object_id: str,
    *,
    threshold_x_minus: float = 0.0,
    threshold_y_plus: float = 0.0,
) -> RunnableTaskDefinition:
    """동일 태그로 base 정렬 후 그리퍼를 열고 접근·파지·상승한다."""
    task = Task(f'align_base_and_pick_up_object_{object_id}')
    tcp_motion = [3.0, 1.0, 1.0, 1.0]
    pickup_distance = 0.075
    contact_offset = [0.0, 0.0, 0.05] 
    approach_offset = list_sum(contact_offset, [0.0, 0.0, pickup_distance])
    pickup_motion = [3.0, 1.0, 1.0, 1.0]

    task.move_base_to_detected_tag(
        camera_source='d435', object_id=object_id,
        desired_tag_x=0.0, desired_tag_y=0.0, desired_tag_frame='d405_camera_center',
        threshold_x_minus=threshold_x_minus, threshold_y_plus=threshold_y_plus,
        relative_yaw=0.0, detection_timeout_sec=5.0, navigation_timeout_sec=30.0,
        max_translation_m=0.8,
    )
    task.delay(1000)
    task.open_gripper('left')
    task.extend(_configured_tag_approach_contact_move(
        object_id, approach_offset, contact_offset, pickup_motion,
    ))
    task.close_gripper('left')
    task.linear_relative('left_arm', (0.0, 0.0, pickup_distance, 0.0, 0.0, 0.0), tcp_motion)
    task.extend(object_gripping_initial_pose_temp())
    return task.build()

def object_handover_demo_final() -> RunnableTaskDefinition:
    task = Task('object_handover_demo_final')

    task.extend(object_gripping_initial_pose_torso_right())

    task.extend(align_base_and_pick_up_object('tag_4'))
    task.extend(turn_and_move('A', 'C'))
    task.extend(align_base_and_put_down_object('tag_3'))

    task.extend(turn_and_move('C', 'B'))
    task.extend(align_base_and_pick_up_object('tag_5'))
    task.extend(turn_and_move('B', 'D'))
    task.extend(align_base_and_put_down_object('tag_0'))

    task.extend(turn_and_move('D', 'C'))
    task.extend(align_base_and_pick_up_object('tag_4'))
    task.extend(turn_and_move('C', 'A'))
    task.extend(align_base_and_put_down_object('tag_2'))

    task.extend(turn_and_move('A', 'D'))
    task.extend(align_base_and_pick_up_object('tag_5'))
    task.extend(turn_and_move('D', 'B'))
    task.extend(align_base_and_put_down_object('tag_1'))

    task.extend(turn_and_move('B', 'A'))

    return task.build()

# Planner Scenario UI 표시 Task 목록
def build_tasks() -> dict[str, RunnableTaskDefinition]:
    return {
        'object_gripping_initial_pose': object_gripping_initial_pose(),
        'object_gripping_initial_pose_torso_right': object_gripping_initial_pose_torso_right(),
        'object_gripping_initial_pose_torso_left': object_gripping_initial_pose_torso_left(),
        'ready_pose': ready_pose(),
        'object_handover_demo_final': object_handover_demo_final(),
    }


__all__ = [
    'Task', 'build_tasks',
    'turn_and_move',
    'object_gripping_initial_pose',
    'ready_pose',
    'object_gripping_initial_pose_torso_right',
    'object_gripping_initial_pose_torso_left',
    'object_handover_demo_final'
    
]
