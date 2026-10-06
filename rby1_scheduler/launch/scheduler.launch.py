from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('rby1_scheduler'))
    mock = share / 'mock_server'
    config = DeclareLaunchArgument(
        'config',
        default_value=str(share / 'config' / 'scheduler.yaml'),
    )
    namespace = DeclareLaunchArgument('namespace', default_value='rby1')
    planner_mode = DeclareLaunchArgument('planner_mode', default_value='ros')
    planner_action = DeclareLaunchArgument(
        'planner_action_name',
        default_value='planner/execute_transfer',
    )
    input_path = DeclareLaunchArgument('server_input_path', default_value=str(mock / 'server_input.json'))
    output_path = DeclareLaunchArgument('scheduler_output_path', default_value=str(mock / 'scheduler_output.json'))
    events_path = DeclareLaunchArgument('scheduler_events_path', default_value=str(mock / 'scheduler_events.jsonl'))
    lab_path = DeclareLaunchArgument('lab_config_path', default_value=str(share / 'config' / 'lab.yaml'))
    scheduler = Node(
        package='rby1_scheduler', executable='scheduler_node', name='scheduler',
        namespace=LaunchConfiguration('namespace'), output='screen', emulate_tty=True,
        parameters=[LaunchConfiguration('config'), {
            'server_input_path': LaunchConfiguration('server_input_path'),
            'scheduler_output_path': LaunchConfiguration('scheduler_output_path'),
            'scheduler_events_path': LaunchConfiguration('scheduler_events_path'),
            'lab_config_path': LaunchConfiguration('lab_config_path'),
            'planner_mode': LaunchConfiguration('planner_mode'),
            'planner_action_name': LaunchConfiguration('planner_action_name'),
        }],
    )
    ui = Node(
        package='rby1_scheduler', executable='scheduler_ui', name='scheduler_ui',
        namespace=LaunchConfiguration('namespace'), output='screen', emulate_tty=True,
    )
    return LaunchDescription([
        config, namespace, planner_mode, planner_action,
        input_path, output_path, events_path, lab_path, scheduler, ui,
    ])
