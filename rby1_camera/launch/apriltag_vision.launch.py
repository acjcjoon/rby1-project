import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _camera_nodes(namespace, config_file):
    """카메라 하나의 장착 TF / AprilTag 검출 / PnP 태그 TF 노드.

    모든 카메라는 자기 이름의 네임스페이스를 쓰므로 노드/토픽이 겹치지 않는다.
      입력: /<ns>/<ns>/color/{image_raw,camera_info}  (realsense camera_namespace=camera_name=<ns>)
      출력: /<ns>/detections, 태그 TF 이름은 camera_system.yaml 의 /<ns>/* 블록에서 정한다.
    """
    image_prefix = f'/{namespace}/{namespace}'
    return [
        Node(
            package='rby1_camera',
            executable='camera_mount_tf_publisher',
            namespace=namespace,
            name='camera_mount_tf_publisher',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            namespace=namespace,
            name='apriltag_node',
            parameters=[config_file],
            remappings=[
                ('image_rect', f'{image_prefix}/color/image_raw'),
                ('camera_info', f'{image_prefix}/color/camera_info'),
            ],
            output='screen'
        ),
        Node(
            package='rby1_camera',
            executable='depth_tag_tf_node.py',
            namespace=namespace,
            name='depth_tag_tf_node',
            parameters=[config_file],
            remappings=[
                ('detections', f'/{namespace}/detections'),
                ('depth_image',
                 f'{image_prefix}/aligned_depth_to_color/image_raw'),
                ('camera_info', f'{image_prefix}/color/camera_info'),
            ],
            output='screen'
        ),
    ]


def generate_launch_description():
    # 세 카메라의 장착 TF와 AprilTag 처리 설정을 한 파일에서 관리합니다.
    config_file = os.path.join(
        get_package_share_directory('rby1_camera'),
        'config',
        'camera_system.yaml'
    )

    # d405_right: 오른손 손목, d405_left: 왼손 손목, d435: 머리
    return LaunchDescription(
        _camera_nodes('d405_right', config_file)
        + _camera_nodes('d405_left', config_file)
        + _camera_nodes('d435', config_file)
    )
