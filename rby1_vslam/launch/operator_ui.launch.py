"""Jetson/Humble RViz and named-waypoint operator UI for the physical robot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString


def generate_launch_description():
    rviz_source = (
        Path(get_package_share_directory('nav2_bringup')) /
        'rviz/nav2_namespaced_view.rviz'
    )
    rviz_config = ReplaceString(
        source_file=str(rviz_source),
        replacements={
            '<robot_namespace>': '/rby1/vslam/nav2',
            'Fixed Frame: map': 'Fixed Frame: vslam_map',
        },
    )
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=str(Path.home() / 'rby1_maps/rby1_vslam_waypoints.yaml'),
            description='Absolute waypoint YAML path on the Jetson.',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='planner_rviz',
            namespace='/rby1/vslam/nav2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': False,
            }],
        ),
        Node(
            package='rby1_vslam',
            executable='waypoint_ui',
            name='waypoint_ui',
            namespace='/rby1/vslam',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False,
                'base_frame': ParameterValue(
                    LaunchConfiguration('base_frame'), value_type=str),
                'waypoints_file': ParameterValue(
                    LaunchConfiguration('waypoints_file'), value_type=str),
            }],
        ),
    ])
