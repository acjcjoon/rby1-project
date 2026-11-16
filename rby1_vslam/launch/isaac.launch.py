"""Isaac ROS 3.2 on Jetson; consume local D435i topics directly."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from rby1_vslam.map_tool import validate_path
from rby1_vslam.modes import isaac_parameters, parse_tracking_mode, uses_imu


def _boolean(value, name):
    if value.strip().lower() not in ('true', 'false'):
        raise ValueError(name + ' must be true or false')
    return value.strip().lower() == 'true'


def _median_kernel_size(value):
    try:
        kernel_size = int(value)
    except ValueError as error:
        raise ValueError('median_kernel_size must be an odd integer from 3 through 9') from error
    if kernel_size < 3 or kernel_size > 9 or kernel_size % 2 == 0:
        raise ValueError('median_kernel_size must be an odd integer from 3 through 9')
    return kernel_size


def _launch(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    mode = get('mode')
    tracking_mode = parse_tracking_mode(get('tracking_mode'))
    parameters = isaac_parameters(tracking_mode, mode)
    distro = os.environ.get('ROS_DISTRO', '')
    if distro and distro != 'humble':
        raise ValueError('This launch requires ROS 2 Humble and Isaac ROS 3.2')
    isaac_share = Path(get_package_share_directory('isaac_ros_visual_slam'))
    version = ET.parse(isaac_share / 'package.xml').getroot().findtext('version', '')
    if version.split('.')[:2] != ['3', '2']:
        raise ValueError(f'Install Isaac ROS release-3.2; found isaac_ros_visual_slam {version}')
    use_sim_time = _boolean(get('use_sim_time'), 'use_sim_time')
    use_median_filter = _boolean(get('enable_median_filter'), 'enable_median_filter')
    use_splitter = _boolean(get('use_realsense_splitter'), 'use_realsense_splitter')
    median_kernel_size = _median_kernel_size(get('median_kernel_size'))
    map_path = validate_path(get('map_path'), 'localize') if mode == 'localization' else ''
    parameters.update({
        'use_sim_time': use_sim_time,
        'enable_image_denoising': _boolean(get('enable_image_denoising'), 'enable_image_denoising'),
        'load_map_folder_path': map_path,
    })
    input_prefix = '/rby1/camera' if use_splitter else '/d435/d435'
    left_input_topic = input_prefix + '/infra1/image_rect_raw'
    right_input_topic = input_prefix + '/infra2/image_rect_raw'
    left_image_topic = ('/rby1/vslam/median/infra1/image_rect_raw'
                        if use_median_filter else left_input_topic)
    right_image_topic = ('/rby1/vslam/median/infra2/image_rect_raw'
                         if use_median_filter else right_input_topic)
    remappings = [
        ('visual_slam/image_0', left_image_topic),
        ('visual_slam/image_1', right_image_topic),
        ('visual_slam/camera_info_0', '/d435/d435/infra1/camera_info'),
        ('visual_slam/camera_info_1', '/d435/d435/infra2/camera_info'),
        ('visual_slam/tracking/odometry', '/rby1/vslam/camera_odometry'),
        ('visual_slam/vis/slam_odometry', '/rby1/vslam/camera_slam_odometry'),
    ]
    if uses_imu(tracking_mode):
        remappings.append(('visual_slam/imu', '/d435/d435/imu'))
    slam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam', namespace='',
        parameters=[get('vslam_params_file'), parameters], remappings=remappings,
    )
    actions = [
        ComposableNodeContainer(
            name='rby1_vslam_container', namespace='', package='rclcpp_components',
            executable='component_container_mt', composable_node_descriptions=[slam],
            output='screen', arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='rby1_vslam', executable='tracking_monitor', name='tracking_monitor',
            namespace='/rby1/vslam', output='screen', parameters=[{
                'use_sim_time': use_sim_time,
                'tracking_timeout_sec': float(get('tracking_timeout_sec')),
                'require_localized': mode == 'localization',
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='vslam_rviz', output='screen',
            arguments=['-d', str(Path(get_package_share_directory('rby1_vslam')) / 'config/vslam.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ]
    if use_median_filter:
        actions.append(Node(
            package='rby1_vslam', executable='median_filter',
            name='stereo_median_filter', namespace='/rby1/vslam', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'kernel_size': median_kernel_size,
                'left_input_topic': left_input_topic,
                'right_input_topic': right_input_topic,
                'left_output_topic': '/rby1/vslam/median/infra1/image_rect_raw',
                'right_output_topic': '/rby1/vslam/median/infra2/image_rect_raw',
            }],
        ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('tracking_mode', default_value='0',
                              description='Isaac ROS 3.2: 0=stereo, 1=stereo+IMU.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('tracking_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('enable_image_denoising', default_value='false'),
        DeclareLaunchArgument(
            'use_realsense_splitter', default_value='false',
            description='Consume emitter-OFF IR; jetson.launch.py starts the splitter.'),
        DeclareLaunchArgument(
            'enable_median_filter', default_value='true',
            description='Apply a spatial median filter to both rectified IR images.'),
        DeclareLaunchArgument(
            'median_kernel_size', default_value='3',
            description='Odd median kernel size from 3 through 9.'),
        DeclareLaunchArgument('vslam_params_file', default_value=str(share / 'config/isaac_vslam.yaml')),
        OpaqueFunction(function=_launch),
    ])
