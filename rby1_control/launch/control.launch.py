"""Launch the single production RB-Y1 control node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('rby1_control'))
    default_config = str(package_share / 'config' / 'default.yaml')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rby1',
        description='ROS namespace for the backend and robot interfaces.',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the control ROS parameter YAML file.',
    )
    robot_address_arg = DeclareLaunchArgument(
        'robot_address',
        default_value='',
        description='RB-Y1 SDK RPC address for asynchronous state queries.',
    )
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='m',
        description='RB-Y1 SDK model name (a, m, or ub).',
    )

    backend_node = Node(
        package='rby1_control',
        executable='rby1_control',
        name='rby1_control',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config'),
            {
                'robot_address': LaunchConfiguration('robot_address'),
                'robot_model': LaunchConfiguration('robot_model'),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        namespace_arg,
        config_arg,
        robot_address_arg,
        robot_model_arg,
        backend_node,
    ])
