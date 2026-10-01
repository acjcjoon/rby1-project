"""Physical RB-Y1 Jetson stack with TCP cuVSLAM on a separate LAB PC."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rby1_vslam.waypoint_store import default_waypoints_path


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false'):
        raise ValueError(name + ' must be true or false')
    return value == 'true'


def _launch(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('rby1_vslam'))
    model = get('robot_model').strip().lower()
    if model not in ('a', 'm'):
        raise ValueError('robot_model must be a or m')

    lab_host = get('lab_host').strip()
    if not lab_host or lab_host in ('127.0.0.1', 'localhost'):
        raise ValueError('lab_host must be the reachable LAN IP of the separate LAB PC')

    actions = []
    if _boolean(context, 'start_driver'):
        driver_share = Path(get_package_share_directory('rby1_driver'))
        actions.append(Node(
            package='rby1_driver', executable='rby1_ros2_driver',
            name='rby1_ros2_driver', namespace='/rby1', output='screen',
            parameters=[str(driver_share / 'config/driver_parameters.yaml'), {
                'use_sim_time': False,
                'robot_ip': get('robot_address'),
                'model': model,
            }],
        ))

    if _boolean(context, 'start_description'):
        description_share = Path(get_package_share_directory('rby1_description'))
        urdf = (description_share / 'urdf' / ('rby1' + model) /
                ('model_v' + get('robot_version').strip() + '.urdf'))
        if not urdf.is_file():
            raise ValueError('RBY1 URDF does not exist: ' + str(urdf))
        actions.extend([
            Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                name='robot_state_publisher', output='screen',
                parameters=[{'use_sim_time': False,
                             'robot_description': urdf.read_text(encoding='utf-8')}],
                remappings=[('joint_states', '/rby1/joint_states')],
            ),
            # Driver odometry ends at base_footprint; the vendor URDF starts at base.
            Node(
                package='tf2_ros', executable='static_transform_publisher',
                name='rby1_base_footprint', output='screen', arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'base_footprint', '--child-frame-id', 'base'],
            ),
        ])

    if _boolean(context, 'start_control'):
        control_share = Path(get_package_share_directory('rby1_control'))
        actions.append(Node(
            package='rby1_control', executable='rby1_control',
            name='rby1_control', namespace='/rby1', output='screen', emulate_tty=True,
            parameters=[str(control_share / 'config/default.yaml'), {
                'use_sim_time': False,
                'robot_address': get('robot_address'),
                'robot_model': model,
            }],
        ))

    # Camera, mount TF, TCP client and camera-to-base pose conversion.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/upc.launch.py')),
        launch_arguments={
            'lab_host': lab_host,
            'port': LaunchConfiguration('port'),
            'start_camera': LaunchConfiguration('start_camera'),
            'serial_no': LaunchConfiguration('serial_no'),
            'infra_profile': LaunchConfiguration('infra_profile'),
            'camera_params_file': LaunchConfiguration('camera_params_file'),
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_imu': LaunchConfiguration('enable_imu'),
            'tracking_timeout_sec': LaunchConfiguration('tracking_timeout_sec'),
            'base_frame': 'base',
            'publish_odom_tf': 'false',
            'publish_mount_tf': LaunchConfiguration('publish_mount_tf'),
            'mount_x': LaunchConfiguration('mount_x'),
            'mount_y': LaunchConfiguration('mount_y'),
            'mount_z': LaunchConfiguration('mount_z'),
            'mount_roll': LaunchConfiguration('mount_roll'),
            'mount_pitch': LaunchConfiguration('mount_pitch'),
            'mount_yaw': LaunchConfiguration('mount_yaw'),
        }.items(),
    ))

    if _boolean(context, 'start_navigation'):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/navigation.launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('navigation_params_file'),
                'occupancy_map': LaunchConfiguration('occupancy_map'),
                'enable': 'false',
                'base_frame': 'base',
                'odom_frame': 'odom',
                'wheel_odom_topic': '/rby1/odom',
                'scan_topic': LaunchConfiguration('scan_topic'),
                'use_scan': LaunchConfiguration('use_scan'),
                'start_gate': 'true',
            }.items(),
        ))

    if _boolean(context, 'start_ui'):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/operator_ui.launch.py')),
            launch_arguments={
                'base_frame': 'base',
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'ui_backend': LaunchConfiguration('ui_backend'),
                'web_host': LaunchConfiguration('web_host'),
                'web_port': LaunchConfiguration('web_port'),
                'start_rviz': LaunchConfiguration('start_rviz'),
            }.items(),
        ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'lab_host', description='Reachable wired-LAN IPv4 address of the LAB PC.'),
        DeclareLaunchArgument('port', default_value='7447'),
        DeclareLaunchArgument('robot_address', default_value='192.168.30.1:50051'),
        DeclareLaunchArgument('robot_model', default_value='m'),
        DeclareLaunchArgument('robot_version', default_value='1_3'),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('start_description', default_value='true'),
        DeclareLaunchArgument('start_control', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_ui', default_value='true'),
        DeclareLaunchArgument('ui_backend', default_value='web'),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('infra_profile', default_value='640,480,30'),
        DeclareLaunchArgument('camera_params_file',
                              default_value=str(share / 'config/d435i.yaml')),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('tracking_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('navigation_params_file',
                              default_value=str(share / 'config/navigation.yaml')),
        DeclareLaunchArgument('occupancy_map', default_value=''),
        DeclareLaunchArgument('use_scan', default_value='false',
                              description='false means Nav2 has no obstacle sensing/avoidance.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('waypoints_file',
                              default_value=str(default_waypoints_path(share))),
        DeclareLaunchArgument('publish_mount_tf', default_value='true'),
        DeclareLaunchArgument('mount_x', default_value='0.0464'),
        DeclareLaunchArgument('mount_y', default_value='0.0'),
        DeclareLaunchArgument('mount_z', default_value='0.066'),
        DeclareLaunchArgument('mount_roll', default_value='0.0'),
        DeclareLaunchArgument('mount_pitch', default_value='-0.03490658503988659'),
        DeclareLaunchArgument('mount_yaw', default_value='0.0'),
        OpaqueFunction(function=_launch),
    ])
