#!/usr/bin/env python3
"""Passive ROS metadata capture. Does not import or change the VSLAM pipeline."""
import argparse
import json
import math
from pathlib import Path
import socket
import time


CAMERA_TOPICS = [
    ('/d435/d435/infra1/image_rect_raw', 'sensor_msgs/msg/Image', 'sensor'),
    ('/d435/d435/infra2/image_rect_raw', 'sensor_msgs/msg/Image', 'sensor'),
    ('/d435/d435/infra1/camera_info', 'sensor_msgs/msg/CameraInfo', 'sensor'),
    ('/d435/d435/infra2/camera_info', 'sensor_msgs/msg/CameraInfo', 'sensor'),
    ('/d435/d435/imu', 'sensor_msgs/msg/Imu', 'sensor'),
    ('/rby1/vslam/camera_odometry', 'nav_msgs/msg/Odometry', 'sensor'),
    ('/rby1/vslam/camera_slam_odometry', 'nav_msgs/msg/Odometry', 'sensor'),
    ('/rby1/vslam/bridge_status', 'std_msgs/msg/String', 'reliable'),
]
UPC_TOPICS = [
    ('/rby1/vslam/slam_odom', 'nav_msgs/msg/Odometry', 'sensor'),
    ('/rby1/odom', 'nav_msgs/msg/Odometry', 'sensor'),
    ('/rby1/vslam/localization_status', 'std_msgs/msg/String', 'reliable'),
    ('/rby1/vslam/navigation_status', 'std_msgs/msg/String', 'reliable'),
    ('/rby1/vslam/nav2_cmd_vel', 'geometry_msgs/msg/Twist', 'reliable'),
    ('/rby1/cmd_raw', 'geometry_msgs/msg/Twist', 'reliable'),
    ('/rby1/cmd_vel', 'geometry_msgs/msg/Twist', 'reliable'),
]
LAB_TOPICS = [
    ('/visual_slam/status', 'isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus', 'sensor'),
]


def safe_json(value):
    """Preserve invalid numeric telemetry without emitting nonstandard JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(k): safe_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe_json(v) for v in value]
    return value


def message_metadata(message, message_type):
    """Keep stamps and small state, never serialize image pixels."""
    fields = {}
    header = getattr(message, 'header', None)
    if header is not None:
        fields.update(source_stamp_ns=int(header.stamp.sec) * 1_000_000_000
                      + int(header.stamp.nanosec), frame_id=str(header.frame_id))
    if message_type == 'sensor_msgs/msg/Image':
        fields.update(width=int(message.width), height=int(message.height),
                      step=int(message.step), encoding=str(message.encoding),
                      payload_bytes=len(message.data))
    elif message_type == 'nav_msgs/msg/Odometry':
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        fields.update(child_frame_id=str(message.child_frame_id),
                      position=[p.x, p.y, p.z], orientation=[q.x, q.y, q.z, q.w],
                      velocity=[v.linear.x, v.linear.y, v.angular.z])
    elif message_type == 'geometry_msgs/msg/Twist':
        fields['velocity'] = [message.linear.x, message.linear.y, message.angular.z]
    elif message_type == 'std_msgs/msg/String':
        try:
            fields['state'] = json.loads(message.data)
        except (ValueError, TypeError):
            fields['text'] = str(message.data)
    elif message_type.endswith('/VisualSlamStatus'):
        fields['vo_state'] = int(message.vo_state)
    return safe_json(fields)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', required=True, choices=('upc', 'lab'))
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--capture-id', required=True)
    args = parser.parse_args()

    # Deferred imports keep help and metadata unit checks usable without ROS.
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile
    from rosidl_runtime_py.utilities import get_message

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = Node('vslam_timing_observer_' + args.role,
                namespace='/rby1/diagnostics')
    host = socket.gethostname()
    topics = CAMERA_TOPICS + (UPC_TOPICS if args.role == 'upc' else LAB_TOPICS)
    counts = {}
    subscriptions = []
    stream = args.output.open('x', encoding='utf-8', buffering=1024 * 1024)

    def write(record):
        stream.write(json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n')

    def callback_for(topic, message_type):
        # A single-argument callback also supports Humble's executor, which
        # does not pass MessageInfo to subscription callbacks.
        def callback(message):
            # Timestamp before extracting fields; this is observer callback time.
            wall_ns, mono_ns = time.time_ns(), time.monotonic_ns()
            ros_ns = node.get_clock().now().nanoseconds
            counts[topic] = counts.get(topic, 0) + 1
            record = {
                'event': 'message', 'role': args.role, 'host': host,
                'capture_id': args.capture_id, 'topic': topic, 'type': message_type,
                'observer_index': counts[topic], 'observed_wall_ns': wall_ns,
                'observed_monotonic_ns': mono_ns, 'observed_ros_ns': ros_ns,
            }
            record.update(message_metadata(message, message_type))
            write(record)
        return callback

    write({'event': 'start', 'role': args.role, 'host': host,
           'capture_id': args.capture_id, 'schema_version': 1,
           'wall_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns()})
    for topic, message_type, qos_name in topics:
        try:
            cls = get_message(message_type)
        except (ImportError, ModuleNotFoundError, AttributeError) as exc:
            write({'event': 'subscription_unavailable', 'topic': topic,
                   'type': message_type, 'detail': str(exc)})
            node.get_logger().warning(f'Cannot observe {topic}: {exc}')
            continue
        qos = qos_profile_sensor_data if qos_name == 'sensor' else QoSProfile(depth=100)
        subscriptions.append(node.create_subscription(
            cls, topic, callback_for(topic, message_type), qos))
    steady = Clock(clock_type=ClockType.STEADY_TIME)

    def heartbeat():
        write({'event': 'observer_health', 'role': args.role, 'host': host,
               'capture_id': args.capture_id, 'wall_ns': time.time_ns(),
               'monotonic_ns': time.monotonic_ns(),
               'ros_ns': node.get_clock().now().nanoseconds, 'counts': dict(counts)})
        stream.flush()

    node.create_timer(1.0, heartbeat, clock=steady)
    stream.flush()
    node.get_logger().info(f'Recording {len(subscriptions)} topic observers to {args.output}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        write({'event': 'end', 'wall_ns': time.time_ns(),
               'monotonic_ns': time.monotonic_ns(), 'counts': counts})
        stream.flush()
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
