"""Route alternating D435i emitter frames with the Isaac ROS 3.2 splitter.

CameraInfo remains on the original /d435/d435 streams. Each selected image
retains its acquisition timestamp and matches the original CameraInfo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    splitter = ComposableNode(
        package='realsense_splitter', plugin='nvblox::RealsenseSplitterNode',
        name='realsense_splitter', namespace='/rby1/camera',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'input_qos': 'SENSOR_DATA', 'output_qos': 'SENSOR_DATA',
        }],
        remappings=[
            ('input/infra_1', '/d435/d435/infra1/image_rect_raw'),
            ('input/infra_1_metadata', '/d435/d435/infra1/metadata'),
            ('input/infra_2', '/d435/d435/infra2/image_rect_raw'),
            ('input/infra_2_metadata', '/d435/d435/infra2/metadata'),
            ('input/depth', '/d435/d435/depth/image_rect_raw'),
            ('input/depth_metadata', '/d435/d435/depth/metadata'),
            ('input/pointcloud', '/d435/d435/depth/color/points'),
            ('input/pointcloud_metadata', '/d435/d435/depth/metadata'),
            ('~/output/infra_1', '/rby1/camera/infra1/image_rect_raw'),
            ('~/output/infra_2', '/rby1/camera/infra2/image_rect_raw'),
            ('~/output/depth', '/rby1/nvblox/depth'),
            ('~/output/pointcloud', '/rby1/nvblox/points'),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        ComposableNodeContainer(
            name='rby1_camera_splitter_container', namespace='',
            package='rclcpp_components', executable='component_container_mt',
            composable_node_descriptions=[splitter], output='screen',
        ),
    ])
