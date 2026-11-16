"""Depth-only nvblox in the continuous wheel odom frame; no localization owner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    node = Node(
        package='nvblox_ros', executable='nvblox_node', name='nvblox_node',
        output='screen', parameters=[LaunchConfiguration('nvblox_params_file'), {
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
        remappings=[
            ('camera_0/depth/image', LaunchConfiguration('depth_topic')),
            ('camera_0/depth/camera_info', LaunchConfiguration('depth_info_topic')),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('nvblox_params_file', default_value=str(share / 'config/nvblox.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('depth_topic', default_value='/rby1/nvblox/depth'),
        DeclareLaunchArgument('depth_info_topic', default_value='/d435/d435/depth/camera_info'),
        node,
        RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='nvblox node exited'))],
        )),
    ])
