"""Convert returned rig poses to base poses with TF at the acquisition time."""
import copy
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import compose, offset_covariance


def xyz(v):
    return (v.x, v.y, v.z)


def xyzw(q):
    return (q.x, q.y, q.z, q.w)


class PoseAdapter(Node):
    def __init__(self):
        super().__init__('pose_adapter')
        defaults = {
            'base_frame': 'base', 'camera_frame': 'd435_link',
            'input_odom_topic': '/rby1/vslam/camera_odometry',
            'input_slam_odom_topic': '/rby1/vslam/camera_slam_odometry',
            'output_odom_topic': '/rby1/vslam/odom',
            'output_slam_odom_topic': '/rby1/vslam/slam_odom',
            'max_input_age_sec': 0.5, 'tf_wait_sec': 0.15,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if not self.p['base_frame'] or not self.p['camera_frame']:
            raise ValueError('base_frame and camera_frame must be nonempty')
        for key in ('max_input_age_sec', 'tf_wait_sec'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(key + ' must be positive and finite')
        self.buffer = Buffer(cache_time=Duration(seconds=10.))
        self.listener = TransformListener(self.buffer, self)
        self.pending = {}
        self.last_warning = 0.
        self.publishers_by_kind = {}
        for kind in ('odom', 'slam_odom'):
            self.publishers_by_kind[kind] = self.create_publisher(
                Odometry, self.p['output_' + kind + '_topic'], 5)
            self.create_subscription(
                Odometry, self.p['input_' + kind + '_topic'],
                lambda msg, k=kind: self.pending.update({k: (msg, time.monotonic())}),
                qos_profile_sensor_data)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.01, self.drain, clock=self.steady)

    def drain(self):
        for kind, (msg, received) in list(self.pending.items()):
            try:
                stamp = Time.from_msg(msg.header.stamp)
                age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
                if stamp.nanoseconds <= 0 or not -0.05 <= age <= self.p['max_input_age_sec']:
                    raise ValueError('stale/future camera pose; check clocks and bridge delay')
                if msg.child_frame_id != self.p['camera_frame'] or not msg.header.frame_id:
                    raise ValueError('unexpected camera pose frame')
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
                self.pending.pop(kind, None)
            except TransformException as exc:
                self.warn(f'Waiting for acquisition-time camera/base TF: {exc}')
                if time.monotonic() - received > self.p['tf_wait_sec']:
                    self.pending.pop(kind, None)
            except (ValueError, TypeError, OverflowError) as exc:
                self.pending.pop(kind, None)
                self.warn(str(exc))

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
