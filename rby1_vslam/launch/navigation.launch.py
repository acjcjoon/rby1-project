"""UPC/Humble: Nav2, map->wheel-odom alignment, and guarded cmd_raw output."""
from pathlib import Path
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _cleanup(context, path):
    del context
    Path(path).unlink(missing_ok=True)
    return []


def _launch(context):
    share = Path(get_package_share_directory('rby1_vslam'))
    namespace = 'rby1/vslam/nav2'
    value = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    occupancy_map = LaunchConfiguration('occupancy_map').perform(context).strip()
    source_file = LaunchConfiguration('params_file').perform(context)
    cleanup = []
    if occupancy_map:
        map_file = Path(occupancy_map)
        if not map_file.is_absolute() or not map_file.is_file():
            raise ValueError('occupancy_map must be an existing absolute map.yaml path on UPC')
        with open(source_file, encoding='utf-8') as stream:
            settings = yaml.safe_load(stream)
        global_costmap = settings['global_costmap']['global_costmap']['ros__parameters']
        global_costmap['rolling_window'] = False
        global_costmap['plugins'] = ['static_layer', 'obstacle_layer', 'inflation_layer']
        global_costmap['static_layer'] = {
            'plugin': 'nav2_costmap_2d::StaticLayer',
            'map_subscribe_transient_local': True,
            'map_topic': '/rby1/vslam/nav2/map',
        }
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', prefix='rby1_vslam_nav2_',
                                         suffix='.yaml', delete=False) as stream:
            yaml.safe_dump(settings, stream)
            source_file = stream.name
        cleanup = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=_cleanup, kwargs={'path': source_file})]))]
    params = ParameterFile(RewrittenYaml(
        source_file=source_file, root_key=namespace,
        param_rewrites={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_base_frame': LaunchConfiguration('base_frame'),
            'odom_topic': LaunchConfiguration('wheel_odom_topic'),
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': LaunchConfiguration('scan_topic'),
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': LaunchConfiguration('scan_topic'),
            'local_costmap.local_costmap.ros__parameters.global_frame': LaunchConfiguration('odom_frame'),
            'behavior_server.ros__parameters.global_frame': LaunchConfiguration('odom_frame'),
        },
        convert_types=True), allow_substs=True)
    server_packages = {
        'controller_server': 'nav2_controller', 'smoother_server': 'nav2_smoother',
        'planner_server': 'nav2_planner', 'behavior_server': 'nav2_behaviors',
        'bt_navigator': 'nav2_bt_navigator', 'waypoint_follower': 'nav2_waypoint_follower',
        'velocity_smoother': 'nav2_velocity_smoother',
    }
    if occupancy_map:
        server_packages = {'map_server': 'nav2_map_server', **server_packages}
    nodes = []
    for name, package in server_packages.items():
        extra_params = []
        remaps = []
        if name == 'map_server':
            extra_params = [{'yaml_filename': occupancy_map, 'frame_id': 'vslam_map',
                             'topic_name': 'map', 'use_sim_time': value('use_sim_time', bool)}]
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
        # Existing robot TF is global. Do not move /tf into the Nav2 namespace.
        nodes.append(Node(
            package=package, executable=name, name=name, namespace=namespace,
            parameters=[params] + extra_params, remappings=remaps, output='screen',
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=namespace, output='screen',
        parameters=[{'use_sim_time': value('use_sim_time', bool),
                     'autostart': True, 'node_names': list(server_packages)}],
    ))
    nodes.append(Node(
        package='rby1_vslam', executable='localization_tf', name='localization_tf',
        namespace='/rby1/vslam', output='screen', parameters=[{
            'use_sim_time': value('use_sim_time', bool),
            'base_frame': value('base_frame', str), 'odom_frame': value('odom_frame', str),
            'map_frame': 'vslam_map',
        }],
    ))
    nodes.append(Node(
        package='rby1_vslam', executable='nav2_gate', name='nav2_gate',
        namespace='/rby1/vslam', output='screen', parameters=[{
            'use_sim_time': value('use_sim_time', bool),
            'enabled': value('enable', bool),
            'nav2_cmd_topic': '/rby1/vslam/nav2_cmd_vel',
            'cmd_raw_topic': '/rby1/cmd_raw',
            'base_frame': value('base_frame', str),
            'wheel_odom_topic': value('wheel_odom_topic', str),
        }],
    ))
    return cleanup + nodes


def generate_launch_description():
    share = Path(get_package_share_directory('rby1_vslam'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/navigation.yaml')),
        DeclareLaunchArgument('occupancy_map', default_value='',
                              description='Optional UPC map.yaml already aligned with vslam_map.'),
        DeclareLaunchArgument('enable', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('base_frame', default_value='base'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/rby1/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        OpaqueFunction(function=_launch),
    ])
