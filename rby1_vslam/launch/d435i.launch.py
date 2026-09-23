"""UPC: exactly one D435i, using this repository's /d435/d435 names."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _camera(context):
    serial = LaunchConfiguration('serial_no').perform(context).strip().lstrip('_')
    if serial and not serial.isdigit():
        raise ValueError('serial_no must be empty or a numeric D435i serial number')
    return [Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        name='d435', namespace='d435', output='screen',
        parameters=[LaunchConfiguration('camera_params_file'), {
            'camera_name': 'd435', 'device_type': 'd435i',
            'serial_no': ParameterValue('_' + serial if serial else '', value_type=str),
            'enable_color': ParameterValue(LaunchConfiguration('enable_color'), value_type=bool),
            'enable_depth': ParameterValue(LaunchConfiguration('enable_depth'), value_type=bool),
            'depth_module.infra_profile': ParameterValue(
                LaunchConfiguration('infra_profile'), value_type=str),
        }],
    )]


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    return LaunchDescription([
        DeclareLaunchArgument('serial_no', default_value='', description='Optional D435i serial.'),
        DeclareLaunchArgument('camera_params_file', default_value=str(share / 'config/d435i.yaml')),
        DeclareLaunchArgument('infra_profile', default_value='640,480,30'),
        DeclareLaunchArgument('enable_color', default_value='false'),
        DeclareLaunchArgument('enable_depth', default_value='false'),
        OpaqueFunction(function=_camera),
    ])
