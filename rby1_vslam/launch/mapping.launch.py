"""LAB: build a cuVSLAM landmark map; UPC runs upc.launch.py separately."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = Path(get_package_share_directory('rby1_vslam')) / 'launch/lab.launch.py'
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), launch_arguments={'mode': 'mapping'}.items(),
    )])
