"""Jetson/Humble station navigation with wheel transit and held visual corrections."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
from rby1_vslam.navigation_config import configure_navigation


def _cleanup(context, path):
    del context
    Path(path).unlink(missing_ok=True)
    return []


def _check_omni_controller(settings):
    """Fail before starting Nav2 if this shell cannot discover our controller."""
    plugin = 'rby1_omni_controller::OmniPursuitController'
    controller = settings.get('controller_server', {}).get('ros__parameters', {})
    if not any(controller.get(name, {}).get('plugin') == plugin
               for name in controller.get('controller_plugins', ['FollowPath'])):
        return
    try:
        manifests, prefix = get_resource(
            'nav2_core__pluginlib__plugin', 'rby1_omni_controller')
        for manifest in manifests.splitlines():
            if not manifest.strip():
                continue
            root = ET.parse(Path(prefix) / manifest.strip()).getroot()
            for library in root.iter('library'):
                if not any(item.get('name', item.get('type')) == plugin
                           and item.get('base_class_type') == 'nav2_core::Controller'
                           for item in library.findall('class')):
                    continue
                library_path = library.get('path', '')
                installed_library = Path(prefix) / 'lib' / f'lib{library_path}.so'
                if not installed_library.is_file():
                    raise ValueError(f'plugin library is missing: {installed_library}')
                return
        raise ValueError('plugin XML does not export ' + plugin)
    except (LookupError, OSError, ValueError, ET.ParseError) as exc:
        raise RuntimeError(
            f'Omni controller is unavailable in this shell: {exc}\n'
            'Nav2 was not started. Build on the Jetson host, then source the overlay '
            'in the same terminal used to launch Nav2:\n'
            '  cd /home/nvidia/rby1_ros2_ws\n'
            '  source /opt/ros/humble/setup.bash\n'
            '  colcon build --symlink-install --packages-select rby1_omni_controller rby1_vslam\n'
            '  source /home/nvidia/rby1_ros2_ws/install/setup.bash\n'
            'If already built, the final source command is sufficient.'
        ) from exc


def _launch(context):
    share = Path(get_package_share_directory('rby1_vslam'))
    namespace = 'rby1/vslam/nav2'
    value = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    occupancy_map = LaunchConfiguration('occupancy_map').perform(context).strip()
    use_scan_text = LaunchConfiguration('use_scan').perform(context).strip().lower()
    if use_scan_text not in ('true', 'false'):
        raise ValueError('use_scan must be true or false')
    use_scan = use_scan_text == 'true'
    use_nvblox_text = LaunchConfiguration('use_nvblox').perform(context).strip().lower()
    if use_nvblox_text not in ('true', 'false'):
        raise ValueError('use_nvblox must be true or false')
    use_nvblox = use_nvblox_text == 'true'
    source_file = LaunchConfiguration('params_file').perform(context)
    with open(source_file, encoding='utf-8') as stream:
        settings = yaml.safe_load(stream)
    cleanup = []
    if occupancy_map or not use_scan or use_nvblox:
        if occupancy_map:
            map_file = Path(occupancy_map)
            if not map_file.is_absolute() or not map_file.is_file():
                raise ValueError('occupancy_map must be an existing absolute map.yaml path on Jetson')
        nvblox_settings = None
        if use_nvblox:
            # Resolve optional dependencies before starting any motion-related node.
            for package in ('nvblox_nav2', 'nvblox_msgs',
                            'nav2_regulated_pure_pursuit_controller'):
                get_package_share_directory(package)
            with open(LaunchConfiguration('nvblox_params_file').perform(context),
                      encoding='utf-8') as stream:
                nvblox_settings = yaml.safe_load(stream)
        settings = configure_navigation(
            settings, use_scan=use_scan, use_nvblox=use_nvblox,
            occupancy_map=occupancy_map, nvblox_settings=nvblox_settings)
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', prefix='rby1_vslam_nav2_',
                                         suffix='.yaml', delete=False) as stream:
            yaml.safe_dump(settings, stream)
            source_file = stream.name
        cleanup = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=_cleanup, kwargs={'path': source_file})]))]
    _check_omni_controller(settings)
    param_rewrites = {
        'use_sim_time': 'false',
        'robot_base_frame': LaunchConfiguration('base_frame'),
        'odom_topic': LaunchConfiguration('nav_odom_topic'),
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': LaunchConfiguration('scan_topic'),
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': LaunchConfiguration('scan_topic'),
        'local_costmap.local_costmap.ros__parameters.global_frame': LaunchConfiguration('odom_frame'),
        'behavior_server.ros__parameters.global_frame': LaunchConfiguration('odom_frame'),
    }
    if use_nvblox:
        param_rewrites.update({
            'local_costmap.local_costmap.ros__parameters.nvblox_layer.nav2_costmap_global_frame': LaunchConfiguration('odom_frame'),
            'local_costmap.local_costmap.ros__parameters.nvblox_layer.nvblox_map_slice_topic': LaunchConfiguration('nvblox_slice_topic'),
            'global_costmap.global_costmap.ros__parameters.nvblox_layer.nvblox_map_slice_topic': LaunchConfiguration('nvblox_slice_topic'),
        })
    params = ParameterFile(RewrittenYaml(
        source_file=source_file, root_key=namespace, param_rewrites=param_rewrites,
        convert_types=True), allow_substs=True)
    server_packages = {
        'controller_server': 'nav2_controller', 'smoother_server': 'nav2_smoother',
        'planner_server': 'nav2_planner', 'behavior_server': 'nav2_behaviors',
        'bt_navigator': 'nav2_bt_navigator', 'waypoint_follower': 'nav2_waypoint_follower',
        'velocity_smoother': 'nav2_velocity_smoother',
    }
    if occupancy_map:
        server_packages = {'map_server': 'nav2_map_server', **server_packages}
    # This vendor driver differentiates odom-frame XY directly into twist.
    # Nav2 requires twist in the child/body frame, especially for omni feedback.
    nodes = [Node(
        package='rby1_vslam', executable='wheel_odom_adapter', name='wheel_odom_adapter',
        namespace='/rby1/vslam', output='screen', parameters=[{
            'use_sim_time': False,
            'input_topic': value('wheel_odom_topic', str),
            'output_topic': value('nav_odom_topic', str),
            'input_twist_frame': value('wheel_twist_frame', str),
            'expected_odom_frame': value('odom_frame', str),
        }],
    )]
    for name, package in server_packages.items():
        extra_params = []
        remaps = []
        arguments = []
        if name == 'map_server':
            extra_params = [{'yaml_filename': occupancy_map, 'frame_id': 'vslam_map',
                             'topic_name': 'map', 'use_sim_time': False}]
        elif name in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', 'cmd_vel_nav')]
        elif name == 'velocity_smoother':
            remaps = [('cmd_vel', 'cmd_vel_nav'),
                      ('cmd_vel_smoothed', '/rby1/vslam/nav2_cmd_vel')]
        elif name == 'bt_navigator':
            extra_params = [{
                'default_nav_to_pose_bt_xml': str(share / 'config/navigate_to_pose.xml'),
                'default_nav_through_poses_bt_xml': str(share / 'config/navigate_through_poses.xml'),
            }]
        if name in ('planner_server', 'controller_server', 'bt_navigator'):
            arguments = ['--ros-args', '--log-level', LaunchConfiguration('nav2_log_level')]
        # Existing robot TF is global. Do not move /tf into the Nav2 namespace.
        nodes.append(Node(
            package=package, executable=name, name=name, namespace=namespace,
            parameters=[params] + extra_params, remappings=remaps,
            arguments=arguments, output='screen',
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=namespace, output='screen',
        parameters=[{'use_sim_time': False,
                     'autostart': True, 'node_names': list(server_packages)}],
    ))
    nodes.append(Node(
        package='rby1_vslam', executable='localization_tf', name='localization_tf',
        namespace='/rby1/vslam', output='screen', parameters=[
            LaunchConfiguration('localization_params_file'), {
            'use_sim_time': False,
            'base_frame': value('base_frame', str), 'odom_frame': value('odom_frame', str),
            'map_frame': 'vslam_map',
            'wheel_odom_topic': value('wheel_odom_topic', str),
            'require_origin_alignment': value('require_origin_alignment', bool),
        }],
        condition=IfCondition(LaunchConfiguration('start_localization_tf')),
    ))
    nodes.append(Node(
        package='rby1_vslam', executable='nav2_gate', name='nav2_gate',
        namespace='/rby1/vslam', output='screen', parameters=[{
            'use_sim_time': False,
            'enabled': value('enable', bool),
            'nav2_cmd_topic': '/rby1/vslam/nav2_cmd_vel',
            'cmd_raw_topic': '/rby1/cmd_raw',
            'base_frame': value('base_frame', str),
            'wheel_odom_topic': value('wheel_odom_topic', str),
            'require_nvblox': use_nvblox,
            'forward_only': use_nvblox,
            'nvblox_depth_topic': value('nvblox_depth_topic', str),
            'nvblox_slice_topic': value('nvblox_slice_topic', str),
            'nvblox_integrated_depth_topic': value('nvblox_integrated_depth_topic', str),
            'nvblox_slice_frame': value('odom_frame', str),
            'depth_timeout_sec': value('nvblox_depth_timeout_sec', float),
            'slice_timeout_sec': value('nvblox_slice_timeout_sec', float),
            'integrated_depth_timeout_sec': value('nvblox_integrated_depth_timeout_sec', float),
        }],
        condition=IfCondition(LaunchConfiguration('start_gate')),
    ))
    return cleanup + nodes


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/navigation.yaml')),
        DeclareLaunchArgument('use_nvblox', default_value='false',
                              description='Use nvblox obstacles and forward-only RPP with sensor watchdogs.'),
        DeclareLaunchArgument('nvblox_params_file',
                              default_value=str(share / 'config/navigation_nvblox.yaml')),
        DeclareLaunchArgument('nvblox_depth_topic', default_value='/rby1/nvblox/depth'),
        DeclareLaunchArgument('nvblox_slice_topic', default_value='/nvblox_node/static_map_slice'),
        DeclareLaunchArgument('nvblox_integrated_depth_topic',
                              default_value='/nvblox_node/back_projected_depth/d435_depth_optical_frame'),
        DeclareLaunchArgument('nvblox_depth_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('nvblox_slice_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument('nvblox_integrated_depth_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument('localization_params_file',
                              default_value=str(share / 'config/localization.yaml')),
        DeclareLaunchArgument('occupancy_map', default_value='',
                              description='Optional Jetson map.yaml already aligned with vslam_map.'),
        DeclareLaunchArgument('enable', default_value='false'),
        DeclareLaunchArgument('base_frame', default_value='base'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/rby1/odom'),
        DeclareLaunchArgument('nav_odom_topic', default_value='/rby1/vslam/wheel_odom'),
        DeclareLaunchArgument('wheel_twist_frame', default_value='odom',
                              description='Current vendor driver: odom. Use body for standard Odometry twist.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('use_scan', default_value='false',
                              description='Enable LaserScan obstacle input only when scan_topic is available.'),
        DeclareLaunchArgument('start_gate', default_value='true'),
        DeclareLaunchArgument('start_localization_tf', default_value='true',
                              description='Disable when localization already runs with the VSLAM container.'),
        DeclareLaunchArgument('require_origin_alignment', default_value='false',
                              description='Require the mapping origin to be accepted before station initialization.'),
        DeclareLaunchArgument(
            'nav2_log_level', default_value='info',
            description='planner/controller/bt_navigator log level (debug|info|warn|error|fatal)'),
        OpaqueFunction(function=_launch),
    ])
