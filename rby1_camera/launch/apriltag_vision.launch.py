import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 통합 설정 파일: apriltag_node와 depth_tag_tf_node 파라미터를 모두 담고 있음.
    # (태그 family/size, apply_flip, trigger_mode, samples_per_trigger 등은 여기서 수정)
    config_file = os.path.join(
        get_package_share_directory('rby1_camera'),
        'config',
        'vision.yaml'
    )
    camera_mount_config = os.path.join(
        get_package_share_directory('rby1_camera'),
        'config',
        'camera_mount_tf.yaml'
    )

    # 로봇 EE의 TF 트리와 RealSense TF 트리를 연결합니다.
    camera_mount_tf = Node(
        package='rby1_camera',
        executable='camera_mount_tf_publisher',
        name='camera_mount_tf_publisher',
        parameters=[camera_mount_config],
        output='screen'
    )

    # 시스템에 깔려있는 공식 apriltag_ros 패키지를 호출합니다.
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[config_file],
        remappings=[
            # 리얼센스 카메라의 토픽 이름을 AprilTag 노드에 맞게 매핑(연결)
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info')
        ],
        output='screen'
    )

    # depth 기반 태그 포즈 노드:
    # apriltag_ros의 /detections(코너 픽셀)와 aligned depth를 결합해
    # TF(tag_depth_<id>)와 /tag_depth_poses(PoseArray)를 퍼블리시합니다.
    # ※ realsense2_camera를 align_depth.enable:=true 로 실행해야
    #    /camera/camera/aligned_depth_to_color/image_raw 토픽이 나옵니다.
    depth_tag_tf_node = Node(
        package='rby1_camera',
        executable='depth_tag_tf_node.py',
        name='depth_tag_tf_node',
        parameters=[config_file],
        remappings=[
            ('detections', '/detections'),
            ('depth_image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_mount_tf,
        apriltag_node,
        depth_tag_tf_node
    ])
