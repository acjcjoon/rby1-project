"""Station operator with live poses, base controls and named station goals."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString
from rby1_vslam.waypoint_store import default_waypoints_path


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
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('base_frame', default_value='base'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.2'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.4'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=str(default_waypoints_path(
                get_package_share_directory('rby1_vslam'))),
            description='Waypoint YAML in package data (source with symlink-install).',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(LaunchConfiguration('start_rviz')),
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
            name='station_operator',
            namespace='/rby1/vslam',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False,
                'base_frame': ParameterValue(
                    LaunchConfiguration('base_frame'), value_type=str),
                'waypoints_file': ParameterValue(
                    LaunchConfiguration('waypoints_file'), value_type=str),
                'max_linear_speed': ParameterValue(
                    LaunchConfiguration('max_linear_speed'), value_type=float),
                'max_angular_speed': ParameterValue(
                    LaunchConfiguration('max_angular_speed'), value_type=float),
            }],
        ),
    ])
