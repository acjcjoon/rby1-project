"""LAB: select a saved map; issue map_tool localize after images arrive."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    path = Path(get_package_share_directory('rby1_vslam')) / 'launch/lab.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('map_path', description='Existing absolute map folder inside LAB environment.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments={
            'mode': 'localization', 'map_path': LaunchConfiguration('map_path'),
        }.items()),
    ])
