from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rby1_navigation'),
                'config',
                'navigation.yaml',
            ]),
        ),
        Node(
            package='rby1_navigation',
            executable='navigation_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
