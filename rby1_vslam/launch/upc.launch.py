"""UPC/Humble sensor acquisition, TCP client and timestamped camera-to-base pose."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    value = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    declarations = [
        DeclareLaunchArgument('lab_host', description='Reachable LAB PC IP or host name.'),
        DeclareLaunchArgument('port', default_value='7447'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('infra_profile', default_value='640,480,30'),
        DeclareLaunchArgument('camera_params_file', default_value=str(share / 'config/d435i.yaml')),
        DeclareLaunchArgument('enable_color', default_value='false'),
        DeclareLaunchArgument('enable_depth', default_value='false'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('forward_clock', default_value='false'),
        DeclareLaunchArgument('base_frame', default_value='base'),
        DeclareLaunchArgument('publish_mount_tf', default_value='true',
                              description='Disable if existing camera mount publisher is running.'),
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
            'serial_no', 'infra_profile', 'camera_params_file', 'enable_color', 'enable_depth'
        )}.items(),
    )
    bridge = Node(
        package='rby1_vslam', executable='bridge_node', name='upc_bridge',
        namespace='/rby1/vslam', output='screen',
        parameters=[str(share / 'config/bridge.yaml'), {
            'role': 'upc', 'lab_host': value('lab_host', str), 'port': value('port', int),
            'enable_imu': value('enable_imu', bool),
            'use_sim_time': value('use_sim_time', bool),
            'forward_clock': value('forward_clock', bool),
        }],
    )
    adapter = Node(
        package='rby1_vslam', executable='pose_adapter', name='pose_adapter',
        namespace='/rby1/vslam', output='screen', parameters=[{
            'use_sim_time': value('use_sim_time', bool),
            'base_frame': value('base_frame', str), 'camera_frame': 'd435_link',
            'input_odom_topic': '/rby1/vslam/camera_odometry',
            'input_slam_odom_topic': '/rby1/vslam/camera_slam_odometry',
            'output_odom_topic': '/rby1/vslam/odom',
            'output_slam_odom_topic': '/rby1/vslam/slam_odom',
        }],
    )
    # Same provisional mount as rby1_camera/config/camera_system.yaml. No import
    # or dependency on that package, and no D405/AprilTag nodes are started.
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
    return LaunchDescription(declarations + [camera, mount, center, bridge, adapter])
