import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERAS = ('d405_right', 'd405_left', 'd435')


def _camera_nodes(namespace, config_file, *, mount_tf=True, detection=True):
    """카메라 하나의 장착 TF / AprilTag 검출 / PnP 태그 TF 노드.

    mount_tf: 장착 TF 노드 (항상 필요. VSLAM 도 D435 장착 TF 를 쓴다)
    detection: AprilTag 검출 + PnP 태그 TF 노드 쌍 (CPU 를 많이 쓰므로 planner 가
               태그 인식 스텝에서만 켠다)

    모든 카메라는 자기 이름의 네임스페이스를 쓰므로 노드/토픽이 겹치지 않는다.
      입력: /<ns>/<ns>/color/{image_raw,camera_info}  (realsense camera_namespace=camera_name=<ns>)
      출력: /<ns>/detections, 태그 TF 이름은 camera_system.yaml 의 /<ns>/* 블록에서 정한다.
    """
    image_prefix = f'/{namespace}/{namespace}'
    nodes = []
    if mount_tf:
        nodes.append(Node(
            package='rby1_camera',
            executable='camera_mount_tf_publisher',
            namespace=namespace,
            name='camera_mount_tf_publisher',
            parameters=[config_file],
            output='screen'
        ))
    if detection:
        nodes.append(Node(
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
        ))
        nodes.append(Node(
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
        ))
    return nodes


def _launch(context):
    # 세 카메라의 장착 TF와 AprilTag 처리 설정을 한 파일에서 관리합니다.
    config_file = os.path.join(
        get_package_share_directory('rby1_camera'),
        'config',
        'camera_system.yaml'
    )
    mount_tf = LaunchConfiguration('mount_tf').perform(context).strip().lower()
    if mount_tf not in ('true', 'false'):
        raise ValueError('mount_tf must be true or false')
    selection = LaunchConfiguration(
        'detection_cameras').perform(context).strip().lower()
    if selection == 'all':
        detection = set(CAMERAS)
    elif selection in ('', 'none'):
        detection = set()
    else:
        detection = {name.strip() for name in selection.split(',')}
        unknown = detection - set(CAMERAS)
        if unknown:
            raise ValueError(
                f'detection_cameras has unknown cameras: {sorted(unknown)}')

    # d405_right: 오른손 손목, d405_left: 왼손 손목, d435: 머리
    nodes = []
    for camera in CAMERAS:
        nodes += _camera_nodes(
            camera, config_file,
            mount_tf=mount_tf == 'true',
            detection=camera in detection,
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mount_tf', default_value='true',
            description='Publish the camera mount TF of all three cameras.'),
        DeclareLaunchArgument(
            'detection_cameras', default_value='all',
            description='AprilTag detection + PnP tag TF: all, none, or a '
                        'comma list of d405_right,d405_left,d435.'),
        OpaqueFunction(function=_launch),
    ])
