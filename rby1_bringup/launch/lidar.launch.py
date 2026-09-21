from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_ip = LaunchConfiguration("host_ip")
    parent_frame = LaunchConfiguration("parent_frame")

    base_scan_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_scan_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.1850",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_scan",
        ],
    )

    # ============================================================
    # Front-Right LakiBeam1L
    # Sensor IP : 192.168.30.10
    # UDP port  : 2368
    # Position  : x=+0.2725, y=-0.2650, z=+0.1850 [m]
    # Yaw       : -45 deg
    # ============================================================
    front_right_lidar = Node(
        package="lakibeam1",
        executable="lakibeam1_scan_node",
        name="lakibeam_front_right",
        output="screen",
        parameters=[{
            "frame_id": "laser_front_right",
            "output_topic": "/scan_front_right",
            "hostip": host_ip,

            "sensorip": "192.168.30.10",
            "port": "2367",

            "inverted": False,
            "angle_offset": 0,
            "scanfreq": "30",
            "filter": "3",
            "laser_enable": "true",
            "scan_range_start": "45",
            "scan_range_stop": "315",
        }],
    )

    front_right_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="front_right_lidar_tf",
        output="screen",
        arguments=[
            "--x", "0.2725",
            "--y", "-0.2200",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "-0.785398",
            "--frame-id", parent_frame,
            "--child-frame-id", "laser_front_right",
        ],
    )

    # ============================================================
    # Rear-Left LakiBeam1L
    # Sensor IP : 192.168.30.11
    # UDP port  : 2367
    # Position  : x=-0.2725, y=+0.2650, z=+0.1850 [m]
    # Yaw       : +135 deg
    # ============================================================
    rear_left_lidar = Node(
        package="lakibeam1",
        executable="lakibeam1_scan_node",
        name="lakibeam_rear_left",
        output="screen",
        parameters=[{
            "frame_id": "laser_rear_left",
            "output_topic": "/scan_rear_left",
            "hostip": host_ip,

            "sensorip": "192.168.30.11",
            "port": "2368",

            "inverted": False,
            "angle_offset": 0,
            "scanfreq": "30",
            "filter": "3",
            "laser_enable": "true",
            "scan_range_start": "45",
            "scan_range_stop": "315",
        }],
    )

    rear_left_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rear_left_lidar_tf",
        output="screen",
        arguments=[
            "--x", "-0.2725",
            "--y", "0.2200",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "2.356194",
            "--frame-id", parent_frame,
            "--child-frame-id", "laser_rear_left",
        ],
    )

    scan_merger = Node(
        package="lakibeam1",
        executable="scan_merger_node.py",
        name="scan_merger_node",
        output="screen",
        parameters=[{
            "scan0_topic": "/scan_front_right",
            "scan1_topic": "/scan_rear_left",
            "output_topic": "/scan",
            "target_frame": "base_scan",

            "output_angle_min": -3.141592653589793,
            "output_angle_max": 3.141592653589793,
            "output_angle_increment": 0.0,

            "range_min": 0.05,
            "range_max": 20.0,

            "sync_slop": 0.02,
            "sync_queue_size": 20,
            "tf_timeout": 0.10,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "host_ip",
            default_value="192.168.30.2",
            description="UPC IP address receiving LiDAR UDP packets",
        ),

        DeclareLaunchArgument(
            "parent_frame",
            default_value="base_scan",
            description="Parent frame for the LiDAR static transforms",
        ),

        base_scan_tf,
        front_right_tf,
        rear_left_tf,
        front_right_lidar,
        rear_left_lidar,
        scan_merger,
    ])