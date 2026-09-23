"""LAB/Jazzy TCP server and Isaac ROS 4.5 cuVSLAM component."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _launch(context):
    share = Path(get_package_share_directory('rby1_vslam'))
    get = lambda name: LaunchConfiguration(name).perform(context)
    mode = get('mode')
    if mode not in ('mapping', 'localization', 'odometry'):
        raise ValueError('mode must be mapping, localization or odometry')
    map_path = get('map_path').strip()
    if mode == 'localization' and (not map_path or not Path(map_path).is_absolute()):
        raise ValueError('localization requires map_path:=/absolute/path inside the LAB environment')
    if mode == 'localization' and not Path(map_path).is_dir():
        raise ValueError(f'LAB map directory does not exist: {map_path}')
    use_sim_time = get('use_sim_time').lower() == 'true'
    use_imu = get('enable_imu').lower() == 'true'
    bridge = Node(
        package='rby1_vslam', executable='bridge_node', name='lab_bridge',
        namespace='/rby1/vslam', output='screen',
        parameters=[str(share / 'config/bridge.yaml'), {
            'role': 'lab', 'bind_host': get('bind_host'), 'port': int(get('port')),
            'tracking_odom_topic': '/rby1/vslam/camera_odometry',
            'slam_odom_topic': '/rby1/vslam/camera_slam_odometry',
            'require_localized': mode == 'localization',
            'use_sim_time': use_sim_time, 'enable_imu': use_imu,
            'forward_clock': get('forward_clock').lower() == 'true',
        }],
    )
    slam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode', name='visual_slam',
        namespace='',
        parameters=[get('vslam_params_file'), {
            'use_sim_time': use_sim_time, 'tracking_mode': 1 if use_imu else 0,
            'enable_localization_n_mapping': mode != 'odometry',
            'load_map_folder_path': map_path if mode == 'localization' else '',
            'localize_on_startup': False,
        }],
        remappings=[
            ('visual_slam/image_0', '/d435/d435/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', '/d435/d435/infra1/camera_info'),
            ('visual_slam/image_1', '/d435/d435/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', '/d435/d435/infra2/camera_info'),
            ('visual_slam/imu', '/d435/d435/imu'),
            ('visual_slam/tracking/odometry', '/rby1/vslam/camera_odometry'),
            ('visual_slam/vis/slam_odometry', '/rby1/vslam/camera_slam_odometry'),
        ],
    )
    container = ComposableNodeContainer(
        name='rby1_vslam_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', composable_node_descriptions=[slam], output='screen',
    )
    return [bridge, container]


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='7447'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('forward_clock', default_value='false'),
        DeclareLaunchArgument('vslam_params_file', default_value=str(share / 'config/isaac_vslam.yaml')),
        OpaqueFunction(function=_launch),
    ])
