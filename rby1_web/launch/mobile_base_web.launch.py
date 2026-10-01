"""UPC-only driver + existing safe control backend + browser operator."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_share = Path(get_package_share_directory('rby1_control'))
    driver_share = Path(get_package_share_directory('rby1_driver'))
    value = lambda key, kind: ParameterValue(LaunchConfiguration(key), value_type=kind)
    declarations = [
        DeclareLaunchArgument('namespace', default_value='rby1'),
        DeclareLaunchArgument('robot_address', default_value='192.168.30.1:50051'),
        DeclareLaunchArgument('robot_model', default_value='m'),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('driver_config', default_value=str(
            driver_share / 'config/driver_parameters.yaml')),
        DeclareLaunchArgument('control_config', default_value=str(
            control_share / 'config/default.yaml')),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.2'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.4'),
    ]
    driver = Node(
        package='rby1_driver', executable='rby1_ros2_driver',
        namespace=LaunchConfiguration('namespace'), output='screen',
        condition=IfCondition(LaunchConfiguration('start_driver')),
        parameters=[LaunchConfiguration('driver_config'), {
            'robot_ip': value('robot_address', str), 'model': value('robot_model', str),
        }],
    )
    control = Node(
        package='rby1_control', executable='rby1_control', name='rby1_control',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[LaunchConfiguration('control_config'), {
            'robot_address': value('robot_address', str),
            'robot_model': value('robot_model', str),
            'use_rby1_services': True,
            'cmd_vel_topic': 'cmd_vel', 'cmd_raw_topic': 'cmd_raw',
            'control_command_topic': 'control/command',
            'control_state_topic': 'control/state',
            'control_event_topic': 'control/event',
            'control_response_topic': 'control/response',
            'command_timeout_sec': 0.35,
        }],
    )
    web = Node(
        package='rby1_web', executable='mobile_base_web', name='mobile_base_web',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[{
            'host': value('web_host', str), 'port': value('web_port', int),
            'max_linear_speed': value('max_linear_speed', float),
            'max_angular_speed': value('max_angular_speed', float),
        }],
    )
    # Tear down the whole minimal stack if any required process exits.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(
            reason='Mobile-base stack process exited'))],
    )) for node in (driver, control, web)]
    return LaunchDescription(declarations + handlers + [driver, control, web])
