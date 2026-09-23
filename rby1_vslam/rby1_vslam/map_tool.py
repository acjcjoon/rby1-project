"""LAB-side Isaac ROS 4.5 map operations with bounded completion checks.

Run in the same container/filesystem and ROS domain as visual_slam. Isaac 4.5
uses services, not the older SaveMap/LoadMapAndLocalize actions. Its save
callback ignores the internal boolean result, and localize returns before
completion. Consequently success=true alone is insufficient for either call.
We require the operation's completion log from /rosout, with a request-time
and operation-start filter. Upstream provides no request ID in its logs, so
concurrent map calls cannot be distinguished. This is tied to release-4.5;
keep the visual_slam logger at INFO and do not run concurrent map operations.

Verified upstream sources (release-4.5):
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-4.5/
isaac_ros_visual_slam/src/visual_slam_node.cpp
and src/impl/visual_slam_impl.cpp, interfaces/srv/{FilePath,LocalizeInMap,Reset}.srv.

All ROS and Isaac imports are lazy so installing the UPC bridge does not
require Isaac ROS; CLI validation and tests also run without ROS installed.
"""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Optional


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
        description="Save/localize cuVSLAM maps on the LAB PC (Isaac ROS 4.5).",
        epilog="Use the same filesystem/container and ROS_DOMAIN_ID as visual_slam. "
        "Keep its logger at INFO. Stop navigation before map operations. "
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
                             help="Absolute map folder in the LAB container/filesystem")
        if command == "localize":
            for component in ("x", "y", "z", "roll", "pitch", "yaw"):
                sub.add_argument("--" + component, type=finite_float, default=0.0,
                                 help="Pose hint for Isaac base_frame in map; angles in radians")
    return parser


def validate_path(value, command):
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise ValueError("--path must be absolute inside the LAB container/filesystem")
    if path.exists() and not path.is_dir():
        raise ValueError("--path is not a directory")
    if command == "localize" and not has_map_database(path):
        raise ValueError("localization needs an existing map folder containing .mdb files")
    if command == "save" and not path.parent.is_dir():
        raise ValueError("create the map parent directory first: {}".format(path.parent))
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


@dataclass
class CompletionLog:
    """Filter completion logs; the caller must serialize all map operations."""

    command: str
    path: str
    logger: str
    since_ns: int = 0
    started: bool = False
    done: bool = False
    error: Optional[str] = None

    def observe(self, logger, message, stamp_ns):
        if not self.since_ns or logger != self.logger or stamp_ns < self.since_ns:
            return
        if self.command == "save":
            start = "Saving map to " + self.path
            success = "Finished saving map"
            failures = ("Failed to save map", "SaveMap Error:", "Cannot save map because")
        else:
            start = "Starting async localization in map '" + self.path + "'"
            success = "Localization completed successfully"
            failures = ("Failed to localize in map.", "Localization failed", "Localization exception:")
        if message == start:
            self.started = True
        elif self.started and message == success:
            self.done = True
        elif self.started and any(message.startswith(item) for item in failures):
            self.error = message


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
                    "No response from {}; operation may still be running on LAB".format(name))
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()
    finally:
        node.destroy_client(client)


def run(args, ros_args=None):
    # LAB-only dependencies: do not move these imports to module scope.
    import rclpy
    from rcl_interfaces.msg import Log, ParameterType
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import (
        DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
    )
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from isaac_ros_visual_slam_interfaces.srv import FilePath, LocalizeInMap, Reset

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

        completion = CompletionLog(args.command, args.path,
                                   target_node.strip("/").replace("/", "."))

        def on_log(message):
            stamp = message.stamp.sec * 1_000_000_000 + message.stamp.nanosec
            completion.observe(message.name, message.msg, stamp)

        node.create_subscription(
            Log, "/rosout", on_log,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        tracking = {"ready": False}

        def on_status(message):
            tracking["ready"] = message.vo_state == 1

        node.create_subscription(VisualSlamStatus, prefix + "/status", on_status,
                                 qos_profile_sensor_data)
        print("Waiting for a fresh successful visual tracking frame...", flush=True)
        _spin_until(rclpy, node, lambda: tracking["ready"], args.timeout,
                    "No successful visual tracking frame; check stereo images, calibration and TF")
        # rcutils /rosout stamps use system time, also when /clock is in use.
        # The CLI must run on the LAB host/container, not a clock-skewed client.
        completion.since_ns = time.time_ns()
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
            raise RuntimeError("cuVSLAM rejected the {} request; check LAB logs".format(args.command))
        _spin_until(rclpy, node, lambda: completion.done or completion.error is not None,
                    args.timeout,
                    "{} completion not confirmed; keep images flowing, check LAB /rosout and "
                    "INFO logging. The operation may still be running".format(args.command))
        if completion.error:
            raise RuntimeError(completion.error)
        if args.command == "save":
            if not has_map_database(args.path):
                raise RuntimeError("Save completed but no .mdb files are visible; run this tool "
                                   "inside the same LAB container/filesystem as cuVSLAM")
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
        print("Run map_tool on LAB after sourcing ROS Jazzy and Isaac ROS 4.5: " + str(exc),
              file=sys.stderr)
        return 2
    except (ValueError, RuntimeError, TimeoutError, OSError) as exc:
        print("map_tool: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
