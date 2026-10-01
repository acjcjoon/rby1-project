"""Web UI only; attach to the existing VSLAM physical/mapping stack."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rby1_vslam.waypoint_store import default_waypoints_path


def generate_launch_description():
    defaults = {
        'namespace': 'rby1', 'web_host': '0.0.0.0', 'web_port': '8080',
        'base_frame': 'base', 'map_frame': 'vslam_map',
        'waypoints_file': str(default_waypoints_path(
            get_package_share_directory('rby1_vslam'))),
        'navigate_action': '/rby1/vslam/nav2/navigate_to_pose',
        'enable_service': '/rby1/vslam/enable', 'cancel_service': '/rby1/vslam/cancel',
        'bridge_status_topic': '/rby1/vslam/bridge_status',
        'navigation_status_topic': '/rby1/vslam/navigation_status',
        'localization_status_topic': '/rby1/vslam/localization_status',
        'map_topic': '/rby1/vslam/nav2/map', 'path_topic': '/rby1/vslam/nav2/plan',
        'control_command_topic': '/rby1/control/command',
        'control_state_topic': '/rby1/control/state',
        'control_event_topic': '/rby1/control/event',
        'control_response_topic': '/rby1/control/response',
    }
    parameters = {key: ParameterValue(LaunchConfiguration(key), value_type=str)
                  for key in defaults if key not in ('namespace', 'web_host', 'web_port')}
    parameters.update({
        'operator_mode': 'vslam',
        'host': ParameterValue(LaunchConfiguration('web_host'), value_type=str),
        'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
        'max_linear_speed': 0.08, 'max_angular_speed': 0.2,
    })
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        Node(package='rby1_web', executable='operator_web', name='operator_web',
             namespace=LaunchConfiguration('namespace'), output='screen', parameters=[parameters]),
    ])
