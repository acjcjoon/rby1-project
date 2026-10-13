"""Convert local camera poses to base poses with acquisition-time head TF."""
import copy
import json
import math
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .geometry import compose, offset_covariance, xyz, xyzw
from .pose_queue import PoseQueue


class PoseAdapter(Node):
    def __init__(self):
        super().__init__('pose_adapter')
        defaults = {
            'base_frame': 'base', 'camera_frame': 'd435_link',
            'input_odom_topic': '/rby1/vslam/camera_odometry',
            'input_slam_odom_topic': '/rby1/vslam/camera_slam_odometry',
            'output_odom_topic': '/rby1/vslam/odom',
            'output_slam_odom_topic': '/rby1/vslam/slam_odom',
            'publish_odom_tf': False,
            'max_input_age_sec': 0.5, 'tf_wait_sec': 0.15,
            'pose_queue_size': 32,
            'tracking_status_topic': '/rby1/vslam/tracking_status',
            'tracking_timeout_sec': 0.75,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if not self.p['base_frame'] or not self.p['camera_frame']:
            raise ValueError('base_frame and camera_frame must be nonempty')
        for key in ('max_input_age_sec', 'tf_wait_sec', 'tracking_timeout_sec'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(key + ' must be positive and finite')
        if self.p['pose_queue_size'] < 1:
            raise ValueError('pose_queue_size must be positive')
        self.buffer = Buffer(cache_time=Duration(seconds=10.))
        self.listener = TransformListener(self.buffer, self)
        self.odom_broadcaster = (TransformBroadcaster(self)
                                 if self.p['publish_odom_tf'] else None)
        self.pending = PoseQueue(('odom', 'slam_odom'), self.p['pose_queue_size'])
        self.session = ''
        self.tracking_ready = False
        self.tracking_received = -math.inf
        self.last_published = {kind: 0 for kind in self.pending.kinds}
        self.last_warning = 0.
        self.publishers_by_kind = {}
        pose_qos = QoSProfile(
            depth=self.p['pose_queue_size'], reliability=ReliabilityPolicy.RELIABLE)
        for kind in ('odom', 'slam_odom'):
            self.publishers_by_kind[kind] = self.create_publisher(
                Odometry, self.p['output_' + kind + '_topic'], pose_qos)
            self.create_subscription(
                Odometry, self.p['input_' + kind + '_topic'],
                lambda msg, k=kind: self.on_pose(k, msg),
                pose_qos)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.retry_timer = self.create_timer(0.01, self.drain, clock=self.steady)
        self.retry_timer.cancel()
        self._retry_active = False
        self.create_subscription(String, self.p['tracking_status_topic'], self.on_tracking, 5)

    def on_tracking(self, message):
        try:
            state = json.loads(message.data)
            session = state.get('session_id', '')
            if not isinstance(session, str):
                raise ValueError('invalid session ID')
            ready = bool(session and state.get('ready') is True and state.get('tracking_ok') is True)
        except (ValueError, TypeError, AttributeError):
            session, ready = '', False
        if session != self.session or not ready:
            self.pending.clear()
            self.last_published = {kind: 0 for kind in self.pending.kinds}
            self._set_retry(False)
        self.session, self.tracking_ready = session, ready
        self.tracking_received = time.monotonic()

    def on_pose(self, kind, message):
        received = time.monotonic()
        if not self.tracking_ready or received - self.tracking_received > self.p['tracking_timeout_sec']:
            return
        dropped = self.pending.put(kind, (message, received))
        if dropped is not None:
            self.warn(f'{kind} pose queue full; dropped oldest pose')
        self.drain(kind)

    def _set_retry(self, active):
        if active and not self._retry_active:
            self.retry_timer.reset()
            self._retry_active = True
        elif not active and self._retry_active:
            self.retry_timer.cancel()
            self._retry_active = False

    def _drop(self, kind, detail):
        self.pending.pop(kind)
        self.warn(detail)

    def _drain_kind(self, kind):
        while True:
            item = self.pending.peek(kind)
            if item is None:
                return
            msg, received = item
            try:
                stamp = Time.from_msg(msg.header.stamp)
                age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
                if stamp.nanoseconds <= 0:
                    self._drop(kind, 'camera pose has no valid timestamp')
                    continue
                if stamp.nanoseconds <= self.last_published[kind]:
                    self._drop(kind, 'duplicate or out-of-order camera pose')
                    continue
                if age < -0.05:
                    self._drop(kind, 'future camera pose; check sensor clock')
                    continue
                if age > self.p['max_input_age_sec']:
                    self._drop(kind, 'stale camera pose; check Jetson processing load')
                    continue
                if msg.child_frame_id != self.p['camera_frame'] or not msg.header.frame_id:
                    self._drop(kind, 'unexpected camera pose frame')
                    continue
                # T_world_base(t) = T_world_camera(t) * T_camera_base(t).
                transform = self.buffer.lookup_transform(
                    self.p['camera_frame'], self.p['base_frame'], stamp,
                    timeout=Duration(seconds=0.)).transform
                pos, rot = compose(
                    xyz(msg.pose.pose.position), xyzw(msg.pose.pose.orientation),
                    xyz(transform.translation), xyzw(transform.rotation))
                out = Odometry()
                out.header = copy.deepcopy(msg.header)
                out.child_frame_id = self.p['base_frame']
                out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = pos
                (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
                 out.pose.pose.orientation.z, out.pose.pose.orientation.w) = rot
                out.pose.covariance = offset_covariance(
                    msg.pose.covariance, xyzw(msg.pose.pose.orientation), xyz(transform.translation))
                # Camera velocity cannot be relabelled as moving-head base velocity.
                # This topic is pose-only; mark the unused twist highly uncertain.
                out.twist.covariance = [1e6 if i % 7 == 0 else 0. for i in range(36)]
                self.publishers_by_kind[kind].publish(out)
                self.last_published[kind] = stamp.nanoseconds
                if kind == 'odom' and self.odom_broadcaster is not None:
                    odom_tf = TransformStamped()
                    odom_tf.header = copy.deepcopy(out.header)
                    odom_tf.child_frame_id = out.child_frame_id
                    odom_tf.transform.translation.x = out.pose.pose.position.x
                    odom_tf.transform.translation.y = out.pose.pose.position.y
                    odom_tf.transform.translation.z = out.pose.pose.position.z
                    odom_tf.transform.rotation = copy.deepcopy(out.pose.pose.orientation)
                    self.odom_broadcaster.sendTransform(odom_tf)
                self.pending.pop(kind)
            except TransformException as exc:
                if time.monotonic() - received > self.p['tf_wait_sec']:
                    self._drop(kind, f'Acquisition-time camera/base TF timed out: {exc}')
                    continue
                self.warn(f'Waiting for acquisition-time camera/base TF: {exc}')
                return
            except (ValueError, TypeError, OverflowError) as exc:
                self._drop(kind, str(exc))

    def drain(self, kind=None):
        if not self.tracking_ready or time.monotonic() - self.tracking_received > self.p['tracking_timeout_sec']:
            self.pending.clear()
            self._set_retry(False)
            return
        kinds = (kind,) if kind is not None else self.pending.kinds
        for selected in kinds:
            self._drain_kind(selected)
        self._set_retry(bool(self.pending))

    def warn(self, message):
        if time.monotonic() - self.last_warning > 2.:
            self.get_logger().warning(message)
            self.last_warning = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = PoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
