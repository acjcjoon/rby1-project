"""Humble/Jazzy boundary: allowlisted standard messages over a bounded TCP link."""

from array import array
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .transport import SocketLink, StereoSynchronizer
from .wire import Packet, ProtocolError


DEFAULTS = {
    'role': 'upc', 'lab_host': '127.0.0.1', 'bind_host': '0.0.0.0', 'port': 7447,
    'socket_timeout_sec': 2.0, 'reconnect_delay_sec': 1.0, 'max_imu_queue': 512,
    'stereo_slop_ms': 5.0, 'stereo_queue_size': 8, 'stereo_max_age_sec': 0.5,
    'max_image_age_sec': 0.5, 'max_future_image_sec': 0.05,
    'enable_imu': True, 'forward_clock': False,
    'camera_root_frame': 'd435_link', 'camera_frame_prefix': 'd435_',
    'left_image_topic': '/d435/d435/infra1/image_rect_raw',
    'right_image_topic': '/d435/d435/infra2/image_rect_raw',
    'left_info_topic': '/d435/d435/infra1/camera_info',
    'right_info_topic': '/d435/d435/infra2/camera_info',
    'imu_topic': '/d435/d435/imu',
    'tracking_odom_topic': '/visual_slam/tracking/odometry',
    'slam_odom_topic': '/visual_slam/vis/slam_odometry',
    'tracking_status_topic': '/visual_slam/status', 'tracking_timeout_sec': 0.5,
    'diagnostics_topic': '/diagnostics', 'require_localized': False,
    'localization_timeout_sec': 1.0,
    'upc_tracking_odom_topic': '/rby1/vslam/camera_odometry',
    'upc_slam_odom_topic': '/rby1/vslam/camera_slam_odometry',
    'status_topic': '/rby1/vslam/bridge_status',
}


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _to_dict(message):
    return dict(message_to_ordereddict(message))


def _check_dictionary(value, template):
    """Reject missing/extra fields recursively, including nested message arrays."""
    if isinstance(template, dict):
        if not isinstance(value, dict) or set(value) != set(template):
            raise ProtocolError('ROS message fields do not match schema')
        for key, child in template.items():
            _check_dictionary(value[key], child)
    elif isinstance(template, (list, tuple)):
        if not isinstance(value, list) or len(value) > 2048:
            raise ProtocolError('invalid ROS array')
        if template:
            if len(value) != len(template):
                raise ProtocolError('invalid fixed array length')
            for item, child in zip(value, template):
                _check_dictionary(item, child)
    elif isinstance(template, str):
        if not isinstance(value, str) or len(value) > 1024:
            raise ProtocolError('invalid ROS string')
    elif isinstance(template, bool):
        if type(value) is not bool:
            raise ProtocolError('invalid ROS boolean')
    elif isinstance(template, int):
        if type(value) is not int:
            raise ProtocolError('invalid ROS integer')
    elif isinstance(template, float):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ProtocolError('invalid ROS floating point value')


def _from_dict(message_type, value):
    message = message_type()
    _check_dictionary(value, _to_dict(message))
    try:
        set_message_fields(message, value)
    except (AttributeError, TypeError, ValueError, AssertionError, OverflowError) as exc:
        raise ProtocolError('invalid typed ROS message') from exc
    return message


def _validate_image(image, info):
    if not (0 < image.width <= 4096 and 0 < image.height <= 4096):
        raise ProtocolError('invalid image dimensions')
    if image.encoding not in ('mono8', '8UC1') or image.step < image.width:
        raise ProtocolError('D435i bridge requires mono8 infrared images')
    if len(image.data) != image.step * image.height:
        raise ProtocolError('image byte count disagrees with dimensions')
    if (info.width, info.height) != (image.width, image.height):
        raise ProtocolError('camera calibration dimensions disagree with image')
    if image.header.frame_id != info.header.frame_id or not image.header.frame_id:
        raise ProtocolError('camera image/calibration frame mismatch')
    if _stamp_ns(image.header.stamp) != _stamp_ns(info.header.stamp):
        raise ProtocolError('camera info stamp must match image stamp')
    if not 0 <= image.header.stamp.nanosec < 1_000_000_000:
        raise ProtocolError('invalid camera timestamp')
    if not (info.k[0] > 0 and info.k[4] > 0):
        raise ProtocolError('camera is not calibrated')


def _stereo_packet(left, right, left_info, right_info):
    _validate_image(left, left_info)
    _validate_image(right, right_info)
    payload = {'left_info': _to_dict(left_info), 'right_info': _to_dict(right_info)}
    blobs = []
    offset = 0
    for name, message in (('left', left), ('right', right)):
        data = bytes(message.data)
        description = {
            'header': _to_dict(message.header), 'height': message.height,
            'width': message.width, 'encoding': message.encoding,
            'is_bigendian': message.is_bigendian, 'step': message.step,
        }
        payload[name] = {'message': description, 'offset': offset, 'length': len(data)}
        blobs.append(data)
        offset += len(data)
    return Packet('stereo', payload, b''.join(blobs))


def _decode_stereo(packet, slop_ns):
    left_info = _from_dict(CameraInfo, packet.payload['left_info'])
    right_info = _from_dict(CameraInfo, packet.payload['right_info'])
    images = []
    for name, info in (('left', left_info), ('right', right_info)):
        description = packet.payload[name]
        # The bulk pixel buffer never goes through JSON or set_message_fields.
        metadata = dict(description['message'], data=[])
        message = _from_dict(Image, metadata)
        start = description['offset']
        # Generated ROS setters accept array('B') without per-pixel Python type
        # checking; assigning a bytes sequence can otherwise iterate every byte.
        message.data = array('B', packet.blob[start:start + description['length']])
        _validate_image(message, info)
        images.append(message)
    if abs(_stamp_ns(images[0].header.stamp) - _stamp_ns(images[1].header.stamp)) > slop_ns:
        raise ProtocolError('stereo images exceed synchronization tolerance')
    return images[0], images[1], left_info, right_info


class BridgeNode(Node):
    def __init__(self):
        super().__init__('vslam_bridge')
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in DEFAULTS}
        self.role = self.cfg['role']
        self._session = ''
        self._camera_session = ''
        self._previous_state = None
        self._tracking_state = 0
        self._tracking_received_at = None
        self._localized = False
        self._localization_received_at = None
        self._requires_localization = self.cfg['require_localized']
        self._last_stereo_stamp = None
        self._lab_input_session = ''
        self._lab_first_image_stamp = None
        self._forwarded_clock_ns = None
        self._static_candidates = {}
        self._subscriptions = []
        self._publishers = {}
        self._synchronizer = StereoSynchronizer(
            int(self.cfg['stereo_slop_ms'] * 1_000_000), self.cfg['stereo_queue_size'])
        if self.cfg['tracking_timeout_sec'] <= 0:
            raise ValueError('tracking_timeout_sec must be positive')
        if (not math.isfinite(self.cfg['localization_timeout_sec'])
                or self.cfg['localization_timeout_sec'] <= 0):
            raise ValueError('localization_timeout_sec must be positive and finite')
        for key in ('max_image_age_sec', 'max_future_image_sec'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] < 0:
                raise ValueError(key + ' must be nonnegative and finite')
        if not self.cfg['camera_root_frame'] or not self.cfg['camera_frame_prefix']:
            raise ValueError('camera frame root and prefix must be nonempty')
        self.link = SocketLink(
            self.role, host=self.cfg['lab_host'], bind_host=self.cfg['bind_host'],
            port=self.cfg['port'], timeout_sec=self.cfg['socket_timeout_sec'],
            reconnect_delay_sec=self.cfg['reconnect_delay_sec'],
            max_imu=self.cfg['max_imu_queue'],
            stereo_max_age_sec=self.cfg['stereo_max_age_sec'])
        self._status_pub = self.create_publisher(String, self.cfg['status_topic'], 5)
        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if self.role == 'upc':
            for name, msg_type, param in (
                ('left', Image, 'left_image_topic'), ('right', Image, 'right_image_topic'),
                ('left_info', CameraInfo, 'left_info_topic'),
                ('right_info', CameraInfo, 'right_info_topic'),
            ):
                self._subscriptions.append(self.create_subscription(
                    msg_type, self.cfg[param],
                    lambda msg, name=name: self._on_camera(name, msg), sensor_qos))
            if self.cfg['enable_imu']:
                self._subscriptions.append(self.create_subscription(
                    Imu, self.cfg['imu_topic'], self._on_imu,
                    QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)))
            self._subscriptions.append(self.create_subscription(
                TFMessage, '/tf_static', self._on_static_tf, static_qos))
            if self.cfg['forward_clock']:
                self._subscriptions.append(self.create_subscription(
                    ClockMessage, '/clock', lambda msg: self.link.send(Packet('clock', _to_dict(msg))),
                    sensor_qos))
            for kind, param in (('tracking_odom', 'upc_tracking_odom_topic'),
                                ('slam_odom', 'upc_slam_odom_topic')):
                self._publishers[kind] = self.create_publisher(Odometry, self.cfg[param], 5)
        else:
            for name, msg_type, param in (
                ('left', Image, 'left_image_topic'), ('right', Image, 'right_image_topic'),
                ('left_info', CameraInfo, 'left_info_topic'),
                ('right_info', CameraInfo, 'right_info_topic'), ('imu', Imu, 'imu_topic'),
            ):
                self._publishers[name] = self.create_publisher(msg_type, self.cfg[param], 5)
            self._publishers['static_tf'] = self.create_publisher(TFMessage, '/tf_static', static_qos)
            if self.cfg['forward_clock']:
                self._publishers['clock'] = self.create_publisher(ClockMessage, '/clock', 5)
            for kind, param in (('tracking_odom', 'tracking_odom_topic'),
                                ('slam_odom', 'slam_odom_topic')):
                self._subscriptions.append(self.create_subscription(
                    Odometry, self.cfg[param],
                    lambda msg, kind=kind: self._on_odometry(kind, msg), sensor_qos))
            # UPC installs no Isaac ROS packages. This import only occurs on LAB.
            try:
                from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
                from diagnostic_msgs.msg import DiagnosticArray
            except ImportError as exc:
                raise RuntimeError('LAB requires isaac_ros_visual_slam_interfaces (Isaac ROS 4.5)') from exc
            self._subscriptions.append(self.create_subscription(
                VisualSlamStatus, self.cfg['tracking_status_topic'], self._on_tracking_status, sensor_qos))
            self._subscriptions.append(self.create_subscription(
                DiagnosticArray, self.cfg['diagnostics_topic'], self._on_diagnostics, 10))
        # Never wait on ROS time to process the first forwarded /clock packet.
        self._timer = self.create_timer(
            0.01, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._last_status_at = 0.0
        self.link.start()

    def _on_camera(self, name, message):
        state = self.link.state()
        if not state['connected']:
            return
        # A network reconnect can occur between timer callbacks. Clear partial
        # synchronizer state before any camera callback can publish into it.
        if state['session_id'] != self._camera_session:
            self._synchronizer.clear()
            self._last_stereo_stamp = None
            self._camera_session = state['session_id']
        try:
            for messages in self._synchronizer.add(name, _stamp_ns(message.header.stamp), message):
                stamp = _stamp_ns(messages[0].header.stamp)
                if self._last_stereo_stamp is not None and stamp <= self._last_stereo_stamp:
                    raise ProtocolError('camera timestamp moved backward or repeated')
                self._last_stereo_stamp = stamp
                self.link.send(_stereo_packet(*messages))
        except (ProtocolError, ValueError, TypeError) as exc:
            self.link.invalidate(str(exc))

    def _on_imu(self, message):
        self.link.send(Packet('imu', _to_dict(message)))

    def _camera_transforms(self, transforms):
        root = self.cfg['camera_root_frame']
        prefix = self.cfg['camera_frame_prefix']
        candidates = {}
        for transform in transforms:
            parent, child = transform.header.frame_id, transform.child_frame_id
            if (child != root and child.startswith(prefix)
                    and (parent == root or parent.startswith(prefix)) and parent != child):
                candidates[child] = transform
        reachable = {root}
        selected = []
        while candidates:
            children = [child for child, tf in candidates.items() if tf.header.frame_id in reachable]
            if not children:
                break
            for child in children:
                selected.append(candidates.pop(child))
                reachable.add(child)
        return selected

    def _on_static_tf(self, message):
        for transform in message.transforms:
            if transform.child_frame_id.startswith(self.cfg['camera_frame_prefix']):
                self._static_candidates[transform.child_frame_id] = transform
        transforms = self._camera_transforms(self._static_candidates.values())
        if transforms:
            self.link.send(Packet('static_tf', _to_dict(TFMessage(transforms=transforms))), persistent=True)

    def _on_tracking_status(self, message):
        state = self.link.state()
        if not state['connected'] or self._lab_input_session != state['session_id']:
            return
        now = time.monotonic()
        localization_age = (None if self._localization_received_at is None
                            else max(0.0, now - self._localization_received_at))
        payload = {'vo_state': int(message.vo_state), 'localized': self._localized,
                   'require_localized': self.cfg['require_localized'],
                   'localization_age_sec': localization_age}
        if hasattr(message, 'header'):
            payload['stamp'] = _to_dict(message.header.stamp)
        self._tracking_state = payload['vo_state']
        self._tracking_received_at = now
        self.link.send(Packet('tracking_status', payload))

    def _on_diagnostics(self, message):
        for status in message.status:
            if status.hardware_id != 'visual_slam' and status.name != 'Visual Slam Diagnostics':
                continue
            values = {entry.key: entry.value for entry in status.values}
            self._localized = values.get('localized_in_exist_map') == 'Yes'
            self._localization_received_at = time.monotonic()

    def _tracking_fields(self, now, connected):
        age = None if self._tracking_received_at is None else max(0.0, now - self._tracking_received_at)
        localization_age = (None if self._localization_received_at is None
                            else max(0.0, now - self._localization_received_at))
        localized = bool(self._localized and localization_age is not None
                         and localization_age <= self.cfg['localization_timeout_sec'])
        return {
            'tracking_ok': bool(connected and self._tracking_state == 1 and age is not None
                                and age <= self.cfg['tracking_timeout_sec']
                                and (not self._requires_localization or localized)),
            'tracking_age_sec': age, 'vo_state': self._tracking_state,
            'localized': localized, 'require_localized': self._requires_localization,
            'localization_age_sec': localization_age,
        }

    def _on_odometry(self, kind, message):
        state = self.link.state()
        if (not state['connected'] or self._lab_input_session != state['session_id']
                or self._lab_first_image_stamp is None
                or _stamp_ns(message.header.stamp) < self._lab_first_image_stamp):
            return
        self.link.send(Packet(kind, _to_dict(message)))

    def _publish_received(self, packet):
        if packet.kind == 'stereo':
            messages = _decode_stereo(packet, self._synchronizer.slop_ns)
            # Use acquisition time as well as local queue time: TCP may have
            # buffered stale bytes before they reached this process. When /clock
            # is bridged, its subscriber callback has not necessarily run yet.
            now_ns = (self._forwarded_clock_ns if self.cfg['forward_clock']
                      else self.get_clock().now().nanoseconds)
            if now_ns is None or now_ns <= 0:
                raise ProtocolError('camera timestamp gate is waiting for a valid /clock')
            for message in messages[:2]:
                stamp_ns = _stamp_ns(message.header.stamp)
                age = (now_ns - stamp_ns) / 1_000_000_000
                if stamp_ns <= 0 or not -self.cfg['max_future_image_sec'] <= age <= self.cfg['max_image_age_sec']:
                    raise ProtocolError('stale/future camera timestamp; synchronize UPC/LAB clocks and check TCP delay')
            if self._lab_input_session != packet.session_id:
                self._lab_input_session = packet.session_id
                self._lab_first_image_stamp = min(_stamp_ns(msg.header.stamp) for msg in messages[:2])
            # Publish calibration first. Neither camera's timestamp is modified.
            for name, message in zip(('left_info', 'right_info', 'left', 'right'),
                                     (messages[2], messages[3], messages[0], messages[1])):
                self._publishers[name].publish(message)
        elif packet.kind == 'static_tf':
            if set(packet.payload) != {'transforms'} or not isinstance(packet.payload['transforms'], list):
                raise ProtocolError('invalid static TF fields')
            if not 1 <= len(packet.payload['transforms']) <= 64:
                raise ProtocolError('invalid camera TF count')
            for transform in packet.payload['transforms']:
                _check_dictionary(transform, _to_dict(TransformStamped()))
            message = _from_dict(TFMessage, packet.payload)
            transforms = self._camera_transforms(message.transforms)
            if len(transforms) != len(message.transforms):
                raise ProtocolError('TF outside camera subtree')
            self._publishers['static_tf'].publish(message)
        elif packet.kind == 'tracking_status':
            fields = {'vo_state', 'localized', 'require_localized', 'localization_age_sec'}
            if set(packet.payload) not in (fields, fields | {'stamp'}):
                raise ProtocolError('invalid tracking status fields')
            state = packet.payload['vo_state']
            if type(state) is not int or state not in (0, 1, 2):
                raise ProtocolError('unknown visual odometry state')
            if any(type(packet.payload[key]) is not bool for key in ('localized', 'require_localized')):
                raise ProtocolError('invalid localization status flags')
            localization_age = packet.payload['localization_age_sec']
            if localization_age is not None and (type(localization_age) not in (int, float)
                                                  or not math.isfinite(localization_age) or localization_age < 0):
                raise ProtocolError('invalid localization status age')
            if 'stamp' in packet.payload:
                from builtin_interfaces.msg import Time
                _from_dict(Time, packet.payload['stamp'])
            self._tracking_state = state
            self._tracking_received_at = time.monotonic()
            self._localized = packet.payload['localized']
            self._requires_localization = self.cfg['require_localized'] or packet.payload['require_localized']
            self._localization_received_at = (None if localization_age is None
                                              else self._tracking_received_at - localization_age)
        else:
            msg_type = {'imu': Imu, 'clock': ClockMessage,
                        'tracking_odom': Odometry, 'slam_odom': Odometry}[packet.kind]
            if packet.kind == 'clock' and not self.cfg['forward_clock']:
                raise ProtocolError('clock forwarding is disabled')
            if packet.kind == 'imu' and not self.cfg['enable_imu']:
                raise ProtocolError('IMU forwarding is disabled')
            message = _from_dict(msg_type, packet.payload)
            if packet.kind == 'clock':
                if not 0 <= message.clock.nanosec < 1_000_000_000:
                    raise ProtocolError('invalid forwarded clock timestamp')
                self._forwarded_clock_ns = _stamp_ns(message.clock)
            self._publishers[packet.kind].publish(message)

    def _tick(self):
        state = self.link.state()
        changed = (state['connected'], state['session_id']) != self._previous_state
        if changed:
            self._synchronizer.clear()
            self._last_stereo_stamp = None
            self._tracking_state = 0
            self._tracking_received_at = None
            if self.role == 'upc':
                self._localized = False
                self._localization_received_at = None
                self._requires_localization = self.cfg['require_localized']
            self._lab_input_session = ''
            self._lab_first_image_stamp = None
            self._forwarded_clock_ns = None
            self._session = state['session_id']
            self._previous_state = (state['connected'], state['session_id'])
            self.get_logger().info('TCP bridge: ' + state['reason'])
        now = time.monotonic()
        if changed or now - self._last_status_at >= 0.1:
            state.update(self._tracking_fields(now, state['connected']))
            state['sync_drops'] = self._synchronizer.dropped
            self._status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
            self._last_status_at = now
        for packet in self.link.receive():
            # Invalidation can occur while the worker is blocked in an I/O call.
            current = self.link.state()
            if not current['connected'] or packet.session_id != current['session_id']:
                break
            try:
                self._publish_received(packet)
            except (ProtocolError, KeyError, TypeError, ValueError, AttributeError, AssertionError) as exc:
                self.link.invalidate('invalid received message: ' + str(exc))
                self.get_logger().error('Rejected TCP message: ' + str(exc))
                break

    def destroy_node(self):
        self.link.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
