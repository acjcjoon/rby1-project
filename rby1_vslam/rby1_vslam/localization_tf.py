"""VSLAM global correction above the existing wheel-odometry TF tree.

The driver remains the sole odom->base publisher. No AMCL/slam_toolbox should
publish another global transform while this node is the localization source.
"""
import json
import math
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .geometry import planar_correction, wrap, yaw
from .pose_adapter import xyz, xyzw


class LocalizationTF(Node):
    def __init__(self):
        super().__init__('localization_tf')
        defaults = {
            'slam_odom_topic': '/rby1/vslam/slam_odom',
            'bridge_status_topic': '/rby1/vslam/bridge_status',
            'status_topic': '/rby1/vslam/localization_status',
            'map_frame': 'vslam_map', 'odom_frame': 'odom', 'base_frame': 'base',
            'pose_timeout_sec': 0.5, 'bridge_timeout_sec': 0.75,
            'max_correction_jump_m': 0.5, 'max_correction_jump_rad': 0.8,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if len({self.p['map_frame'], self.p['odom_frame'], self.p['base_frame']}) != 3:
            raise ValueError('map, odom and base frame must be distinct')
        for key in ('pose_timeout_sec', 'bridge_timeout_sec', 'max_correction_jump_m', 'max_correction_jump_rad'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(key + ' must be positive and finite')
        self.buffer = Buffer(cache_time=Duration(seconds=10.))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 5)
        self.bridge_ok = False
        self.bridge_received = -math.inf
        self.session = ''
        self.pending = None
        self.correction = None
        self.last_pose_stamp = 0
        self.pose_received = -math.inf
        self.jump_until = 0.
        self.last_status = 0.
        self.detail = 'waiting for VSLAM and timestamped wheel TF'
        self.create_subscription(Odometry, self.p['slam_odom_topic'], self.on_pose, qos_profile_sensor_data)
        self.create_subscription(String, self.p['bridge_status_topic'], self.on_bridge, 5)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.tick, clock=self.steady)

    def on_bridge(self, msg):
        try:
            state = json.loads(msg.data)
            session = str(state.get('session_id', ''))
            ok = state.get('connected') is True and state.get('tracking_ok') is True and bool(session)
        except (ValueError, TypeError, AttributeError):
            session, ok = '', False
        if session != self.session or not ok:
            self.pending = None
            self.correction = None
            self.last_pose_stamp = 0
        self.session = session
        self.bridge_ok = ok
        self.bridge_received = time.monotonic()

    def on_pose(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp <= self.last_pose_stamp:
            if stamp < self.last_pose_stamp:
                self.correction = None
                self.last_pose_stamp = 0
                self.detail = 'pose time reset'
            return
        self.pending = (msg, time.monotonic())

    def tick(self):
        now = time.monotonic()
        ros_now = self.get_clock().now()
        healthy = False
        if not self.bridge_ok or now-self.bridge_received > self.p['bridge_timeout_sec']:
            self.correction = None
            self.pending = None
            self.detail = 'bridge or tracking unavailable'
        elif self.pending is not None:
            msg, received = self.pending
            stamp = Time.from_msg(msg.header.stamp)
            age = (ros_now.nanoseconds-stamp.nanoseconds)/1e9
            try:
                if stamp.nanoseconds <= 0 or not -0.05 <= age <= self.p['pose_timeout_sec']:
                    raise ValueError('VSLAM pose timestamp stale/future')
                if msg.header.frame_id != self.p['map_frame'] or msg.child_frame_id != self.p['base_frame']:
                    raise ValueError('SLAM pose frame mismatch')
                # Query wheel TF at the SAME time as the visual measurement.
                wheel = self.buffer.lookup_transform(
                    self.p['odom_frame'], self.p['base_frame'], stamp,
                    timeout=Duration(seconds=0.)).transform
                correction = planar_correction(xyz(msg.pose.pose.position), xyzw(msg.pose.pose.orientation),
                                               xyz(wheel.translation), xyzw(wheel.rotation))
                if self.correction is not None:
                    old_p, old_q = self.correction
                    p, q = correction
                    if (math.hypot(p[0]-old_p[0], p[1]-old_p[1]) > self.p['max_correction_jump_m']
                            or abs(wrap(yaw(q)-yaw(old_q))) > self.p['max_correction_jump_rad']):
                        self.jump_until = now + 1.0
                self.correction = correction
                self.pose_received = received
                self.last_pose_stamp = stamp.nanoseconds
                self.pending = None
                self.detail = 'global correction ready'
            except TransformException:
                self.detail = 'waiting for acquisition-time odom->base TF'
                if now-received > 0.2:
                    self.pending = None
            except (ValueError, TypeError, OverflowError) as exc:
                self.correction = None
                self.pending = None
                self.detail = str(exc)
        if self.correction is not None:
            age = (ros_now.nanoseconds-self.last_pose_stamp)/1e9
            healthy = (now-self.pose_received <= self.p['pose_timeout_sec']
                       and -0.05 <= age <= self.p['pose_timeout_sec']
                       and now >= self.jump_until)
            if healthy:
                out = TransformStamped()
                out.header.stamp = ros_now.to_msg()
                out.header.frame_id = self.p['map_frame']
                out.child_frame_id = self.p['odom_frame']
                p, q = self.correction
                out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = p
                (out.transform.rotation.x, out.transform.rotation.y,
                 out.transform.rotation.z, out.transform.rotation.w) = q
                self.broadcaster.sendTransform(out)
            elif now < self.jump_until:
                self.detail = 'global correction jumped; motion must be rearmed'
            else:
                self.detail = 'global pose expired'
        if now-self.last_status >= 0.1:
            status = String()
            status.data = json.dumps({'healthy': healthy, 'session_id': self.session,
                                      'detail': self.detail,
                                      'map_frame': self.p['map_frame'], 'odom_frame': self.p['odom_frame']})
            self.status_pub.publish(status)
            self.last_status = now


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
