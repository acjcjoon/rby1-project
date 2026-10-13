"""Jetson D435i with Isaac ROS 3.2's RealSense 4.51.1 driver API."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rby1_vslam.modes import camera_profile, depth_module_profile, parse_tracking_mode, uses_imu, uses_rgbd


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false')
    return value == 'true'


def _camera(context):
    camera_share = Path(get_package_share_directory('realsense2_camera'))
    version = ET.parse(camera_share / 'package.xml').getroot().findtext('version', '')
    if version.split('.')[:3] != ['4', '51', '1']:
        raise ValueError('Use Isaac ROS 3.2 RealSense release/4.51.1-isaac; '
                         f'found realsense2_camera {version}. To reuse an existing camera '
                         'publisher use start_camera:=false and verify its topic names.')
    serial = LaunchConfiguration('serial_no').perform(context).strip().lstrip('_')
    if serial and not serial.isdigit():
        raise ValueError('serial_no must be empty or a numeric D435i serial number')
    tracking_mode = parse_tracking_mode(
        LaunchConfiguration('tracking_mode').perform(context))
    publish_all_sensors = _boolean(context, 'publish_all_sensors')
    use_imu = publish_all_sensors or uses_imu(tracking_mode)
    use_rgbd = publish_all_sensors or uses_rgbd(tracking_mode)
    use_stereo = publish_all_sensors or not uses_rgbd(tracking_mode)
    profile = depth_module_profile(
        tracking_mode, publish_all_sensors,
        LaunchConfiguration('infra_profile').perform(context),
        LaunchConfiguration('depth_profile').perform(context))
    return [Node(
        package='realsense2_camera', executable='realsense2_camera_node',
        # 4.51.1 publishes relative topics; this retains /d435/d435/infra1/... .
        name='d435', namespace='d435/d435', output='screen',
        parameters=[LaunchConfiguration('camera_params_file'), {
            'camera_name': 'd435', 'device_type': 'd435i',
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'serial_no': ParameterValue('_' + serial if serial else '', value_type=str),
            'enable_gyro': use_imu,
            'enable_accel': use_imu,
            'unite_imu_method': 2 if use_imu else 0,
            'enable_color': use_rgbd,
            'enable_depth': use_rgbd,
            'enable_infra1': use_stereo,
            'enable_infra2': use_stereo,
            'enable_sync': use_rgbd,
            'align_depth.enable': use_rgbd,
            'depth_module.emitter_enabled': 1 if use_rgbd else 0,
            'rgb_camera.profile': camera_profile(LaunchConfiguration('color_profile').perform(context)),
            'depth_module.profile': profile,
        }],
    )]


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    return LaunchDescription([
        DeclareLaunchArgument('serial_no', default_value='', description='Optional D435i serial.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera_params_file', default_value=str(share / 'config/d435i.yaml')),
        DeclareLaunchArgument(
            'tracking_mode', default_value='0',
            description='cuVSLAM mode: 0=stereo, 1=stereo+IMU, 2=RGB-D.'),
        DeclareLaunchArgument(
            'publish_all_sensors', default_value='false',
            description='Publish color, depth, stereo IR and IMU together.'),
        DeclareLaunchArgument('infra_profile', default_value='640,360,30'),
        DeclareLaunchArgument('color_profile', default_value='640,480,30'),
        DeclareLaunchArgument('depth_profile', default_value='640,480,30'),
        OpaqueFunction(function=_camera),
    ])
