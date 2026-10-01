"""UPC VSLAM operator: browser by default, optional legacy Qt/RViz."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rby1_vslam.waypoint_store import default_waypoints_path


def _launch(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    backend = get('ui_backend').strip().lower()
    if backend not in ('web', 'qt'):
        raise ValueError('ui_backend must be web or qt')
    if backend == 'qt':
        share = Path(get_package_share_directory('rby1_vslam'))
        filename = 'operator_qt.launch.py'
        keys = ('base_frame', 'waypoints_file')
    else:
        share = Path(get_package_share_directory('rby1_web'))
        filename = 'operator_web.launch.py'
        keys = ('base_frame', 'waypoints_file', 'web_host', 'web_port')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / filename)),
        launch_arguments={key: LaunchConfiguration(key) for key in keys}.items(),
    )]
    rviz = get('start_rviz').strip().lower()
    if rviz not in ('true', 'false'):
        raise ValueError('start_rviz must be true or false')
    if backend == 'web' and rviz == 'true':
        from nav2_common.launch import ReplaceString
        from launch_ros.actions import Node
        source = Path(get_package_share_directory('nav2_bringup')) / 'rviz/nav2_namespaced_view.rviz'
        config = ReplaceString(source_file=str(source), replacements={
            '<robot_namespace>': '/rby1/vslam/nav2', 'Fixed Frame: map': 'Fixed Frame: vslam_map',
        })
        actions.append(Node(package='rviz2', executable='rviz2', name='planner_rviz',
                            namespace='/rby1/vslam/nav2', output='screen',
                            arguments=['-d', config], parameters=[{'use_sim_time': False}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base'),
        DeclareLaunchArgument('waypoints_file', default_value=str(default_waypoints_path(
            get_package_share_directory('rby1_vslam')))),
        DeclareLaunchArgument('ui_backend', default_value='web'),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        OpaqueFunction(function=_launch),
    ])
