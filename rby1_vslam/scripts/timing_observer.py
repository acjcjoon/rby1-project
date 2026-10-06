#!/usr/bin/env python3
"""Passive ROS metadata capture. Does not import or change the VSLAM pipeline."""
import argparse
import json
import math
from pathlib import Path
import socket
import time


CAMERA_TOPICS = [
    # CameraInfo is published once per delivered stereo frame with the same
    # source stamp.  Observe it instead of copying ~18 MB/s of raw images into
    # this Python diagnostics process.
    ('/d435/d435/infra1/camera_info', 'sensor_msgs/msg/CameraInfo', 'sensor'),
    ('/d435/d435/infra2/camera_info', 'sensor_msgs/msg/CameraInfo', 'sensor'),
    ('/d435/d435/imu', 'sensor_msgs/msg/Imu', 'sensor'),
    ('/rby1/vslam/camera_odometry', 'nav_msgs/msg/Odometry', 'sensor'),
    ('/rby1/vslam/camera_slam_odometry', 'nav_msgs/msg/Odometry', 'sensor'),
    ('/rby1/vslam/bridge_status', 'std_msgs/msg/String', 'reliable'),
    ('/rby1/vslam/timing', 'std_msgs/msg/String', 'reliable'),
    ('/rosout', 'rcl_interfaces/msg/Log', 'rosout'),
]
UPC_TOPICS = [
    ('/rby1/vslam/odom', 'nav_msgs/msg/Odometry', 'sensor'),
    ('/rby1/vslam/slam_odom', 'nav_msgs/msg/Odometry', 'sensor'),
    ('/rby1/odom', 'nav_msgs/msg/Odometry', 'sensor'),
    ('/rby1/vslam/localization_status', 'std_msgs/msg/String', 'reliable'),
    ('/rby1/vslam/navigation_status', 'std_msgs/msg/String', 'reliable'),
    ('/rby1/vslam/navigation_event', 'std_msgs/msg/String', 'reliable'),
    ('/rby1/vslam/nav2_cmd_vel', 'geometry_msgs/msg/Twist', 'reliable'),
    ('/rby1/cmd_raw', 'geometry_msgs/msg/Twist', 'reliable'),
    ('/rby1/cmd_vel', 'geometry_msgs/msg/Twist', 'reliable'),
    ('/rby1/vslam/nav2/navigate_to_pose/_action/status',
     'action_msgs/msg/GoalStatusArray', 'action_status'),
    ('/rby1/vslam/nav2/navigate_to_pose/_action/feedback',
     'nav2_msgs/action/NavigateToPose_FeedbackMessage', 'reliable'),
    ('/rby1/vslam/nav2/navigate_through_poses/_action/status',
     'action_msgs/msg/GoalStatusArray', 'action_status'),
    ('/rby1/vslam/nav2/navigate_through_poses/_action/feedback',
     'nav2_msgs/action/NavigateThroughPoses_FeedbackMessage', 'reliable'),
    ('/tf', 'tf2_msgs/msg/TFMessage', 'reliable'),
    ('/parameter_events', 'rcl_interfaces/msg/ParameterEvent', 'parameter_events'),
]
NAV2_DEBUG_TOPICS = [
    ('/rby1/vslam/nav2/plan', 'nav_msgs/msg/Path', 'reliable'),
    ('/rby1/vslam/nav2/received_global_plan', 'nav_msgs/msg/Path', 'reliable'),
    ('/rby1/vslam/nav2/transformed_global_plan', 'nav_msgs/msg/Path', 'reliable'),
    ('/rby1/vslam/nav2/local_plan', 'nav_msgs/msg/Path', 'reliable'),
    ('/rby1/vslam/nav2/compute_path_to_pose/_action/status',
     'action_msgs/msg/GoalStatusArray', 'action_status'),
    ('/rby1/vslam/nav2/follow_path/_action/status',
     'action_msgs/msg/GoalStatusArray', 'action_status'),
    ('/rby1/vslam/nav2/follow_path/_action/feedback',
     'nav2_msgs/action/FollowPath_FeedbackMessage', 'reliable'),
]
LAB_TOPICS = [
    ('/visual_slam/status', 'isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus', 'sensor'),
    ('/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', 'reliable'),
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


def integer_scalar(value):
    """Normalize ROS uint8 scalars across Humble/Jazzy Python bindings."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        data = bytes(value)
        if len(data) != 1:
            raise ValueError(f'expected one-byte integer scalar, got {len(data)} bytes')
        return data[0]
    return int(value)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _quaternion_yaw(rotation):
    sin_yaw = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cos_yaw = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(sin_yaw, cos_yaw)


def _pose2d(pose):
    return [float(pose.position.x), float(pose.position.y),
            float(_quaternion_yaw(pose.orientation))]


def _duration_sec(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _parameter_value(value):
    """Convert rcl_interfaces/ParameterValue without importing ROS constants."""
    kind = integer_scalar(value.type)
    fields = {
        1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
        5: 'byte_array_value', 6: 'bool_array_value', 7: 'integer_array_value',
        8: 'double_array_value', 9: 'string_array_value',
    }
    if kind == 0:
        return None
    name = fields.get(kind)
    if name is None:
        return {'unsupported_parameter_type': kind}
    result = getattr(value, name)
    if kind == 5:
        return list(bytes(result))
    if kind >= 6:
        return list(result)
    return result


def _parameter(parameter):
    return {'name': str(parameter.name), 'value': _parameter_value(parameter.value)}


def message_metadata(message, message_type):
    """Keep stamps and small state, never serialize image pixels."""
    fields = {}
    header = getattr(message, 'header', None)
    if header is not None:
        fields.update(source_stamp_ns=_stamp_ns(header.stamp), frame_id=str(header.frame_id))
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
            if isinstance(fields['state'], dict):
                stamp = fields['state'].get('source_stamp_ns')
                if isinstance(stamp, int) and stamp > 0:
                    fields['source_stamp_ns'] = stamp
                if isinstance(fields['state'].get('stage'), str):
                    fields['timing_stage'] = fields['state']['stage']
        except (ValueError, TypeError):
            fields['text'] = str(message.data)
    elif message_type.endswith('/VisualSlamStatus'):
        fields['vo_state'] = int(message.vo_state)
        # Interface revisions expose additional execution counters under
        # different names. Preserve every scalar without binding this tool to
        # a single Isaac ROS release or serializing nested arrays.
        scalar = {}
        get_types = getattr(message, 'get_fields_and_field_types', None)
        for name in (get_types() if callable(get_types) else {}):
            value = getattr(message, name, None)
            if isinstance(value, (bool, int, float, str)):
                scalar[str(name)] = value
        if scalar:
            fields['visual_slam_scalars'] = scalar
    elif message_type == 'diagnostic_msgs/msg/DiagnosticArray':
        fields['diagnostics'] = [{
            'name': str(status.name), 'hardware_id': str(status.hardware_id),
            'level': integer_scalar(status.level), 'message': str(status.message),
            'values': {str(item.key): str(item.value) for item in status.values},
        } for status in message.status]
    elif message_type == 'rcl_interfaces/msg/Log':
        fields.update(log_level=int(message.level), log_name=str(message.name),
                      log_message=str(message.msg), log_file=str(message.file),
                      log_function=str(message.function), log_line=int(message.line))
    elif message_type == 'action_msgs/msg/GoalStatusArray':
        fields['goal_statuses'] = [{
            'goal_id': bytes(status.goal_info.goal_id.uuid).hex(),
            'status': int(status.status),
            'goal_stamp_ns': _stamp_ns(status.goal_info.stamp),
        } for status in message.status_list]
    elif message_type == 'nav_msgs/msg/Path':
        points = [_pose2d(item.pose) for item in message.poses]
        lengths = [math.hypot(right[0] - left[0], right[1] - left[1])
                   for left, right in zip(points, points[1:])]
        fields.update(
            pose_count=len(points), path_length_m=sum(lengths),
            direct_distance_m=(math.hypot(points[-1][0] - points[0][0],
                                          points[-1][1] - points[0][1])
                               if len(points) >= 2 else 0.0),
            max_step_m=max(lengths) if lengths else 0.0,
            start_pose=points[0] if points else None,
            end_pose=points[-1] if points else None,
        )
    elif message_type == 'tf2_msgs/msg/TFMessage':
        transforms = []
        interesting = {'vslam_map', 'odom', 'base', 'base_footprint'}
        for item in message.transforms:
            parent = str(item.header.frame_id).lstrip('/')
            child = str(item.child_frame_id).lstrip('/')
            if parent not in interesting or child not in interesting:
                continue
            transform = item.transform
            transforms.append({
                'source_stamp_ns': _stamp_ns(item.header.stamp),
                'parent_frame': parent, 'child_frame': child,
                'translation': [float(transform.translation.x),
                                float(transform.translation.y),
                                float(transform.translation.z)],
                'orientation': [float(transform.rotation.x), float(transform.rotation.y),
                                float(transform.rotation.z), float(transform.rotation.w)],
                'yaw': float(_quaternion_yaw(transform.rotation)),
            })
        fields['transforms'] = transforms
    elif message_type == 'rcl_interfaces/msg/ParameterEvent':
        fields.update(
            source_stamp_ns=_stamp_ns(message.stamp),
            parameter_node=str(message.node),
            new_parameters=[_parameter(item) for item in message.new_parameters],
            changed_parameters=[_parameter(item) for item in message.changed_parameters],
            deleted_parameters=[str(item.name) for item in message.deleted_parameters],
        )
    elif message_type.endswith('_FeedbackMessage'):
        feedback = message.feedback
        fields['goal_id'] = bytes(message.goal_id.uuid).hex()
        current_pose = getattr(feedback, 'current_pose', None)
        if current_pose is not None:
            fields['current_pose'] = _pose2d(current_pose.pose)
            fields['current_pose_frame'] = str(current_pose.header.frame_id)
            fields['current_pose_stamp_ns'] = _stamp_ns(current_pose.header.stamp)
        for name in ('navigation_time', 'estimated_time_remaining'):
            value = getattr(feedback, name, None)
            if value is not None:
                fields[name + '_sec'] = _duration_sec(value)
        for name in ('number_of_recoveries', 'number_of_poses_remaining'):
            value = getattr(feedback, name, None)
            if value is not None:
                fields[name] = int(value)
        for name in ('distance_remaining', 'distance_to_goal', 'speed'):
            value = getattr(feedback, name, None)
            if value is not None:
                fields[name + ('_m' if name != 'speed' else '_mps')] = float(value)
    return safe_json(fields)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', required=True, choices=('upc', 'lab'))
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--capture-id', required=True)
    parser.add_argument('--nav2-debug', action='store_true',
                        help='UPC only: observe child actions and subscriber-driven plan topics')
    args = parser.parse_args()
    if args.nav2_debug and args.role != 'upc':
        parser.error('--nav2-debug is UPC-only')

    # Deferred imports keep help and metadata unit checks usable without ROS.
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy import qos as rclpy_qos
    from rclpy.qos import qos_profile_sensor_data, QoSProfile
    from rosidl_runtime_py.utilities import get_message

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = Node('vslam_timing_observer_' + args.role,
                namespace='/rby1/diagnostics')
    host = socket.gethostname()
    topics = CAMERA_TOPICS + (UPC_TOPICS if args.role == 'upc' else LAB_TOPICS)
    if args.nav2_debug:
        topics += NAV2_DEBUG_TOPICS
    counts = {}
    current_bridge = {'session_id': '', 'connected': False}
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
            metadata = message_metadata(message, message_type)
            # /tf carries the entire robot tree at high rate. Keep only the
            # localization chain selected by message_metadata().
            if message_type == 'tf2_msgs/msg/TFMessage' and not metadata['transforms']:
                return
            record.update(metadata)
            state = record.get('state')
            if topic == '/rby1/vslam/bridge_status' and isinstance(state, dict):
                current_bridge['session_id'] = str(state.get('session_id') or '')
                current_bridge['connected'] = bool(state.get('connected', False))
            exact_session = (str(state.get('session_id') or '')
                             if topic == '/rby1/vslam/timing' and isinstance(state, dict)
                             else '')
            record['bridge_session_id'] = exact_session or current_bridge['session_id']
            record['bridge_connected'] = current_bridge['connected']
            write(record)
        return callback

    write({'event': 'start', 'role': args.role, 'host': host,
           'capture_id': args.capture_id, 'schema_version': 1,
           'nav2_debug': args.nav2_debug,
           'wall_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns()})
    for topic, message_type, qos_name in topics:
        try:
            cls = get_message(message_type)
        except (ImportError, ModuleNotFoundError, AttributeError) as exc:
            write({'event': 'subscription_unavailable', 'topic': topic,
                   'type': message_type, 'detail': str(exc)})
            node.get_logger().warning(f'Cannot observe {topic}: {exc}')
            continue
        if qos_name == 'sensor':
            qos = qos_profile_sensor_data
        elif qos_name == 'rosout':
            # Humble and the Isaac ROS Jazzy image do not consistently export
            # qos_profile_rosout_default from rclpy.qos.  A reliable volatile
            # reader is compatible with the standard transient-local rosout
            # publisher and records new log messages without version checks.
            qos = QoSProfile(depth=100)
        elif qos_name == 'parameter_events':
            qos = getattr(
                rclpy_qos, 'qos_profile_parameter_events', QoSProfile(depth=100))
        elif qos_name == 'action_status':
            qos = getattr(
                rclpy_qos, 'qos_profile_action_status_default', QoSProfile(depth=10))
        else:
            qos = QoSProfile(depth=100)
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
