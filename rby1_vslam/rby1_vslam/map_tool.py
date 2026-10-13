"""Jetson-local map services for Isaac ROS 3.2.

In release-3.2 save_map and localize_in_map return after the cuVSLAM result
is available. Check that response and the saved database, without depending
on release-4.5 asynchronous completion log strings. ROS imports stay lazy
so path/CLI validation can also run on a development PC.
"""

import argparse
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


def finite_float(value):
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("must be finite")
    return result


def positive_float(value):
    result = finite_float(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return result


def build_parser():
    parser = argparse.ArgumentParser(
        description="Save/localize cuVSLAM maps on Jetson (Isaac ROS 3.2).",
        epilog="Use the same filesystem/container and ROS_DOMAIN_ID as visual_slam. "
        "Stop navigation before map operations. "
        "Do not run concurrent map operations.",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("save", "localize", "reset"):
        sub = commands.add_parser(command)
        sub.add_argument("--service-prefix", default="/visual_slam")
        sub.add_argument("--node-name", default="/visual_slam",
                         help="Fully qualified Isaac component node name")
        sub.add_argument("--service-timeout", type=positive_float, default=15.0,
                         help="Service discovery timeout, seconds (default: 15)")
        sub.add_argument("--timeout", type=positive_float, default=120.0,
                         help="Tracking/operation timeout, seconds (default: 120)")
        if command != "reset":
            sub.add_argument("--path", required=True,
                             help="Absolute map folder in the Jetson/container filesystem")
        if command == "localize":
            for component in ("x", "y", "z", "roll", "pitch", "yaw"):
                sub.add_argument("--" + component, type=finite_float, default=0.0,
                                 help="Pose hint for Isaac base_frame in map; angles in radians")
    return parser


def validate_path(value, command):
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise ValueError("--path must be absolute inside the Jetson/container filesystem")
    if path.exists() and not path.is_dir():
        raise ValueError("--path is not a directory")
    if command == "localize" and not has_map_database(path):
        raise ValueError("localization needs an existing map folder containing .mdb files")
    if command == "save" and not path.parent.is_dir():
        raise ValueError("create the map parent directory first: {}".format(path.parent))
    if command == "save" and path.exists() and any(path.iterdir()):
        raise ValueError("save needs a new or empty map directory")
    return str(path)


def has_map_database(path):
    path = Path(path)
    return path.is_dir() and any(item.is_file() for item in path.glob("*.mdb"))


def pose_quaternion(roll, pitch, yaw):
    """Return ROS quaternion x,y,z,w for fixed-axis roll, pitch, yaw."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _spin_until(rclpy, node, predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not predicate():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(description)
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    if not rclpy.ok():
        raise RuntimeError("ROS shutdown while " + description)


def _call(rclpy, node, srv_type, name, request, discovery_timeout, response_timeout):
    client = node.create_client(srv_type, name)
    try:
        if not client.wait_for_service(timeout_sec=discovery_timeout):
            raise TimeoutError("Service not available: " + name)
        future = client.call_async(request)
        _spin_until(rclpy, node, future.done, response_timeout,
                    "No response from {}; operation may still be running on Jetson".format(name))
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()
    finally:
        node.destroy_client(client)


def run(args, ros_args=None):
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from isaac_ros_visual_slam_interfaces.srv import FilePath, LocalizeInMap, Reset

    package_share = Path(get_package_share_directory('isaac_ros_visual_slam'))
    version = ET.parse(package_share / 'package.xml').getroot().findtext('version', '')
    if version.split('.')[:2] != ['3', '2']:
        raise ValueError('map_tool requires Isaac ROS 3.2 completion semantics; found ' + version)

    prefix = args.service_prefix.rstrip("/")
    target_node = args.node_name.rstrip("/")
    if not prefix.startswith("/") or not target_node.startswith("/"):
        raise ValueError("--service-prefix and --node-name must be absolute ROS names")
    rclpy.init(args=ros_args or [])
    node = rclpy.create_node("rby1_vslam_map_tool")
    try:
        if args.command == "reset":
            response = _call(rclpy, node, Reset, prefix + "/reset", Reset.Request(),
                             args.service_timeout, args.timeout)
            if not response.success:
                raise RuntimeError("cuVSLAM reset rejected")
            print("cuVSLAM reset. Restart localization before resuming navigation.")
            return 0

        # Mapping disabled can otherwise produce a success response with no map,
        # or no usable SLAM instance for a localize request.
        request = GetParameters.Request()
        request.names = ["enable_localization_n_mapping"]
        response = _call(rclpy, node, GetParameters, target_node + "/get_parameters",
                         request, args.service_timeout, args.timeout)
        if (len(response.values) != 1 or
                response.values[0].type != ParameterType.PARAMETER_BOOL or
                not response.values[0].bool_value):
            raise RuntimeError("Isaac node must set enable_localization_n_mapping=true")

        tracking = {"ready": False}

        def on_status(message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            age = node.get_clock().now().nanoseconds * 1e-9 - stamp
            tracking["ready"] = message.vo_state == 1 and stamp > 0 and -0.05 <= age <= 0.5

        node.create_subscription(VisualSlamStatus, prefix + "/status", on_status,
                                 qos_profile_sensor_data)
        print("Waiting for a fresh successful visual tracking frame...", flush=True)
        _spin_until(rclpy, node, lambda: tracking["ready"], args.timeout,
                    "No successful visual tracking frame; check stereo images, calibration and TF")
        if args.command == "save":
            Path(args.path).mkdir(exist_ok=True)
            request = FilePath.Request()
            request.file_path = args.path
            service_type, suffix = FilePath, "/save_map"
        else:
            request = LocalizeInMap.Request()
            request.map_folder_path = args.path
            request.pose_hint.position.x = args.x
            request.pose_hint.position.y = args.y
            request.pose_hint.position.z = args.z
            q = pose_quaternion(args.roll, args.pitch, args.yaw)
            (request.pose_hint.orientation.x, request.pose_hint.orientation.y,
             request.pose_hint.orientation.z, request.pose_hint.orientation.w) = q
            service_type, suffix = LocalizeInMap, "/localize_in_map"

        response = _call(rclpy, node, service_type, prefix + suffix, request,
                         args.service_timeout, args.timeout)
        if not response.success:
            raise RuntimeError("cuVSLAM rejected the {} request; check Jetson logs".format(args.command))
        if args.command == "save":
            if not has_map_database(args.path):
                raise RuntimeError("Save completed but no .mdb files are visible; run this tool "
                                   "inside the same Jetson container/filesystem as cuVSLAM")
            print("Map saved: " + args.path)
        else:
            print("Localization completed in map: " + args.path)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if "--ros-args" in argv:
        separator = argv.index("--ros-args")
        argv, ros_args = argv[:separator], argv[separator:]
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command != "reset":
            args.path = validate_path(args.path, args.command)
        return run(args, ros_args)
    except KeyboardInterrupt:
        print("Interrupted; a server-side map operation may still be running.", file=sys.stderr)
        return 130
    except ImportError as exc:
        print("Run map_tool on Jetson after sourcing ROS Humble and Isaac ROS 3.2: " + str(exc),
              file=sys.stderr)
        return 2
    except (ValueError, RuntimeError, TimeoutError, OSError) as exc:
        print("map_tool: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
