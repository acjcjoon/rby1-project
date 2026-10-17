"""Jetson-local camera, cuVSLAM, tracking health and camera-to-base pose."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    value = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    declarations = [
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('tracking_mode', default_value='0'),
        DeclareLaunchArgument('infra_profile', default_value='640,360,30'),
        DeclareLaunchArgument('camera_params_file', default_value=str(share / 'config/d435i.yaml')),
        DeclareLaunchArgument('vslam_params_file', default_value=str(share / 'config/isaac_vslam.yaml')),
        DeclareLaunchArgument('enable_image_denoising', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('tracking_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('pose_queue_size', default_value='32'),
        DeclareLaunchArgument(
            'planar_base_motion', default_value='true',
            description='Constrain the mobile-base VSLAM pose to z=roll=pitch=0.'),
        DeclareLaunchArgument('base_frame', default_value='base'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('start_localization_tf', default_value='false'),
        DeclareLaunchArgument('start_map_origin_alignment', default_value='true'),
        DeclareLaunchArgument('publish_mount_tf', default_value='true'),
        DeclareLaunchArgument('mount_x', default_value='0.0464'),
        DeclareLaunchArgument('mount_y', default_value='0.0'),
        DeclareLaunchArgument('mount_z', default_value='0.066'),
        DeclareLaunchArgument('mount_roll', default_value='0.0'),
        DeclareLaunchArgument('mount_pitch', default_value='-0.03490658503988659'),
        DeclareLaunchArgument('mount_yaw', default_value='0.0'),
    ]
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/d435i.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_camera')),
        launch_arguments={key: LaunchConfiguration(key) for key in (
            'serial_no', 'tracking_mode', 'infra_profile', 'camera_params_file', 'use_sim_time',
        )}.items(),
    )
    isaac = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/isaac.launch.py')),
        launch_arguments={key: LaunchConfiguration(key) for key in (
            'mode', 'map_path', 'tracking_mode', 'vslam_params_file', 'use_sim_time',
            'enable_image_denoising', 'tracking_timeout_sec',
            'start_rviz',
        )}.items(),
    )
    adapter = Node(
        package='rby1_vslam', executable='pose_adapter', name='pose_adapter',
        namespace='/rby1/vslam', output='screen', parameters=[{
            'use_sim_time': value('use_sim_time', bool),
            'base_frame': value('base_frame', str), 'camera_frame': 'd435_link',
            'publish_odom_tf': False,
            'planar_base_motion': value('planar_base_motion', bool),
            'pose_queue_size': value('pose_queue_size', int),
        }],
    )
    align_mapping_origin = PythonExpression([
        "'", LaunchConfiguration('mode'), "' == 'mapping' and '",
        LaunchConfiguration('start_map_origin_alignment'), "'.lower() == 'true'",
    ])
    origin = Node(
        package='rby1_vslam', executable='map_origin_initializer',
        name='map_origin_initializer', namespace='/rby1/vslam', output='screen',
        parameters=[{
            'use_sim_time': value('use_sim_time', bool),
            'enabled': ParameterValue(align_mapping_origin, value_type=bool),
            'odom_frame': value('odom_frame', str),
            'camera_frame': 'd435_link',
        }],
    )
    localization = Node(
        package='rby1_vslam', executable='localization_tf', name='localization_tf',
        namespace='/rby1/vslam', output='screen',
        condition=IfCondition(LaunchConfiguration('start_localization_tf')),
        parameters=[{
            'use_sim_time': value('use_sim_time', bool),
            'base_frame': value('base_frame', str),
            'odom_frame': value('odom_frame', str),
            'map_frame': 'vslam_map',
            'require_origin_alignment': ParameterValue(
                align_mapping_origin, value_type=bool),
        }],
    )
    mount = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='d435_vslam_mount', output='screen',
        condition=IfCondition(LaunchConfiguration('publish_mount_tf')),
        arguments=[
            '--x', LaunchConfiguration('mount_x'), '--y', LaunchConfiguration('mount_y'),
            '--z', LaunchConfiguration('mount_z'), '--roll', LaunchConfiguration('mount_roll'),
            '--pitch', LaunchConfiguration('mount_pitch'), '--yaw', LaunchConfiguration('mount_yaw'),
            '--frame-id', 'link_head_2', '--child-frame-id', 'd435_camera_center',
        ],
    )
    center = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='d435_vslam_center', output='screen',
        condition=IfCondition(LaunchConfiguration('publish_mount_tf')),
        arguments=['--x', '0.02185', '--y', '0.0175', '--z', '0.0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'd435_camera_center', '--child-frame-id', 'd435_link'],
    )
    return LaunchDescription(
        declarations + [camera, mount, center, isaac, adapter, origin, localization])
