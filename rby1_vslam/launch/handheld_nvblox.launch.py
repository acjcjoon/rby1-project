"""Handheld RGB reconstruction using cuVSLAM's full 6-DoF camera pose.

The host owns the RealSense driver. This launch owns only splitter, optional
median filtering, cuVSLAM and nvblox. No robot mount, wheel odometry, Nav2 or
station-correction node is started. Run it in a separate ROS domain from the
robot so that d435_link has exactly one dynamic TF parent.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _boolean(value, name):
    if value.strip().lower() not in ('true', 'false'):
        raise ValueError(name + ' must be true or false')
    return value.strip().lower() == 'true'


def _positive_float(value, name):
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(name + ' must be finite and positive')
    return number


def _launch(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    use_sim_time = _boolean(get('use_sim_time'), 'use_sim_time')
    use_median = _boolean(get('enable_median_filter'), 'enable_median_filter')
    kernel = int(get('median_kernel_size'))
    if kernel < 3 or kernel > 9 or kernel % 2 == 0:
        raise ValueError('median_kernel_size must be an odd integer from 3 through 9')
    voxel_size = _positive_float(get('voxel_size'), 'voxel_size')
    max_distance = _positive_float(
        get('max_integration_distance_m'), 'max_integration_distance_m')
    isaac_share = Path(get_package_share_directory('isaac_ros_visual_slam'))
    version = ET.parse(isaac_share / 'package.xml').getroot().findtext('version', '')
    if version.split('.')[:2] != ['3', '2']:
        raise ValueError('This handheld launch requires Isaac ROS 3.2; found ' + version)

    input_prefix = '/rby1/camera'
    image_prefix = '/rby1/vslam/median' if use_median else input_prefix
    slam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam', namespace='',
        parameters=[get('vslam_params_file'), {
            'use_sim_time': use_sim_time,
            'base_frame': 'd435_link',
            'odom_frame': 'vslam_odom',
            'map_frame': 'vslam_map',
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': False,
            'invert_odom_to_base_tf': False,
            'override_publishing_stamp': False,
            # Match the continuous odometry path in NVIDIA's 3.2 RealSense
            # example. A loop-closure pose jump would not deform old voxels.
            'enable_localization_n_mapping': False,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
            'enable_imu_fusion': False,
            'enable_slam_visualization': True,
        }],
        remappings=[
            ('visual_slam/image_0', image_prefix + '/infra1/image_rect_raw'),
            ('visual_slam/image_1', image_prefix + '/infra2/image_rect_raw'),
            ('visual_slam/camera_info_0', '/d435/d435/infra1/camera_info'),
            ('visual_slam/camera_info_1', '/d435/d435/infra2/camera_info'),
            ('visual_slam/tracking/odometry', '/rby1/vslam/camera_odometry'),
            ('visual_slam/vis/slam_odometry', '/rby1/vslam/camera_slam_odometry'),
        ],
    )
    container = ComposableNodeContainer(
        name='rby1_handheld_vslam_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[slam], output='screen',
    )
    mapper = Node(
        package='nvblox_ros', executable='nvblox_node', name='nvblox_node',
        output='screen', parameters=[
            get('nvblox_params_file'), get('handheld_params_file'), {
                'use_sim_time': use_sim_time,
                'voxel_size': voxel_size,
                'max_back_projection_distance': max_distance,
                'static_mapper.projective_integrator_max_integration_distance_m': max_distance,
            },
        ],
        remappings=[
            ('camera_0/depth/image', '/rby1/nvblox/depth'),
            ('camera_0/depth/camera_info', '/d435/d435/depth/camera_info'),
            ('camera_0/color/image', '/d435/d435/color/image_raw'),
            ('camera_0/color/camera_info', '/d435/d435/color/camera_info'),
        ],
    )
    monitor = Node(
        package='rby1_vslam', executable='tracking_monitor', name='tracking_monitor',
        namespace='/rby1/vslam', output='screen', parameters=[{
            'use_sim_time': use_sim_time,
            'tracking_timeout_sec': 0.5,
            'require_localized': False,
        }],
    )
    actions = [container, monitor, mapper]
    critical_nodes = [container, mapper]
    if use_median:
        median = Node(
            package='rby1_vslam', executable='median_filter',
            name='stereo_median_filter', namespace='/rby1/vslam', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'kernel_size': kernel,
                'left_input_topic': input_prefix + '/infra1/image_rect_raw',
                'right_input_topic': input_prefix + '/infra2/image_rect_raw',
                'left_output_topic': image_prefix + '/infra1/image_rect_raw',
                'right_output_topic': image_prefix + '/infra2/image_rect_raw',
            }],
        )
        actions.append(median)
        critical_nodes.append(median)
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Handheld reconstruction process exited'))],
    )) for node in critical_nodes]
    return handlers + actions


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('voxel_size', default_value='0.03',
                              description='Voxel edge length in metres; 0.05 reduces GPU/memory load.'),
        DeclareLaunchArgument('max_integration_distance_m', default_value='4.0'),
        DeclareLaunchArgument('enable_median_filter', default_value='true'),
        DeclareLaunchArgument('median_kernel_size', default_value='3'),
        DeclareLaunchArgument('vslam_params_file', default_value=str(share / 'config/isaac_vslam.yaml')),
        DeclareLaunchArgument('nvblox_params_file', default_value=str(share / 'config/nvblox.yaml')),
        DeclareLaunchArgument('handheld_params_file', default_value=str(share / 'config/handheld_nvblox.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/realsense_splitter.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        ),
        OpaqueFunction(function=_launch),
    ])
