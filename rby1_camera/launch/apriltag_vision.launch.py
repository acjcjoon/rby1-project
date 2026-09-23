import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 두 카메라의 장착 TF와 AprilTag 처리 설정을 한 파일에서 관리합니다.
    config_file = os.path.join(
        get_package_share_directory('rby1_camera'),
        'config',
        'camera_system.yaml'
    )

    # D405는 기존 플래너 인터페이스(/detections, tag_pnp_*)를 유지합니다.
    d405_camera_mount_tf = Node(
        package='rby1_camera',
        executable='camera_mount_tf_publisher',
        name='camera_mount_tf_publisher',
        parameters=[config_file],
        output='screen'
    )
    d405_apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[config_file],
        remappings=[
            ('image_rect', '/d405/d405/color/image_raw'),
            ('camera_info', '/d405/d405/color/camera_info'),
        ],
        output='screen'
    )
    d405_depth_tag_tf_node = Node(
        package='rby1_camera',
        executable='depth_tag_tf_node.py',
        name='depth_tag_tf_node',
        parameters=[config_file],
        remappings=[
            ('detections', '/detections'),
            ('depth_image',
             '/d405/d405/aligned_depth_to_color/image_raw'),
            ('camera_info', '/d405/d405/color/camera_info'),
        ],
        output='screen'
    )

    # D435는 독립 네임스페이스를 사용해 D405의 토픽/노드와 충돌하지 않습니다.
    d435_camera_mount_tf = Node(
        package='rby1_camera',
        executable='camera_mount_tf_publisher',
        namespace='d435',
        name='camera_mount_tf_publisher',
        parameters=[config_file],
        output='screen'
    )
    d435_apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        namespace='d435',
        name='apriltag_node',
        parameters=[config_file],
        remappings=[
            ('image_rect', '/d435/d435/color/image_raw'),
            ('camera_info', '/d435/d435/color/camera_info'),
        ],
        output='screen'
    )
    d435_depth_tag_tf_node = Node(
        package='rby1_camera',
        executable='depth_tag_tf_node.py',
        namespace='d435',
        name='depth_tag_tf_node',
        parameters=[config_file],
        remappings=[
            ('detections', '/d435/detections'),
            ('depth_image',
             '/d435/d435/aligned_depth_to_color/image_raw'),
            ('camera_info', '/d435/d435/color/camera_info'),
        ],
        output='screen'
    )

    return LaunchDescription([
        d405_camera_mount_tf,
        d405_apriltag_node,
        d405_depth_tag_tf_node,
        d435_camera_mount_tf,
        d435_apriltag_node,
        d435_depth_tag_tf_node,
    ])
