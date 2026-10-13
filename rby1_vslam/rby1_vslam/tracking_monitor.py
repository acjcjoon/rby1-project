"""Expose local Isaac ROS tracking health to pose conversion, Nav2 and UI."""

import json
import time

from diagnostic_msgs.msg import DiagnosticArray
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from .geometry import finite, quaternion, xyzw, xyz
from .tracking_state import TrackingState


def _stamp(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid timestamp')
    return stamp.sec + stamp.nanosec * 1e-9


class TrackingMonitor(Node):
    def __init__(self):
        super().__init__('tracking_monitor')
        defaults = {
            'tracking_timeout_sec': 0.5, 'require_localized': False,
            'isaac_status_topic': '/visual_slam/status',
            'camera_odom_topic': '/rby1/vslam/camera_odometry',
            'status_topic': '/rby1/vslam/tracking_status',
            'isaac_logger': 'visual_slam', 'camera_frame': 'd435_link',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.core = TrackingState(self.p['tracking_timeout_sec'], self.p['require_localized'])
        self.started_ns = time.time_ns()
        self.last_ros_now = None
        self.pub = self.create_publisher(String, self.p['status_topic'], 5)
        self.create_subscription(VisualSlamStatus, self.p['isaac_status_topic'],
                                 self.on_status, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['camera_odom_topic'],
                                 self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/diagnostics',
                                 self.on_diagnostics, qos_profile_sensor_data)
        logs_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Log, '/rosout', self.on_log, logs_qos)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.tick, clock=self.steady)

    def on_status(self, msg):
        try:
            self.core.observe_status(msg.vo_state, _stamp(msg.header.stamp), time.monotonic())
        except (ValueError, TypeError, OverflowError):
            self.core.status = None

    def on_odometry(self, msg):
        try:
            if msg.header.frame_id != 'vslam_odom' or msg.child_frame_id != self.p['camera_frame']:
                raise ValueError('camera odometry frame mismatch')
            finite(xyz(msg.pose.pose.position))
            quaternion(xyzw(msg.pose.pose.orientation))
            self.core.observe_odometry(_stamp(msg.header.stamp), time.monotonic())
        except (ValueError, TypeError, OverflowError):
            self.core.odom = None

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.hardware_id != 'visual_slam' or status.name != 'Visual Slam Diagnostics':
                continue
            values = {item.key: item.value for item in status.values}
            try:
                self.core.observe_localization(values.get('localized_in_exist_map') == 'Yes',
                                               _stamp(msg.header.stamp), time.monotonic())
            except (ValueError, TypeError, OverflowError):
                self.core.diagnostics = None

    def on_log(self, msg):
        stamp_ns = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        if msg.name != self.p['isaac_logger'] or stamp_ns < self.started_ns:
            return
        # The 3.2 reset service and reinitialization can change the map origin
        # without changing topic names or advancing a public tracker ID.
        if msg.msg in ('cuvslam: Reset', 'Initializing cuVSLAM.',
                       'cuVSLAM tracker was destroyed') or msg.msg.startswith(
                           'Trying to localize in map '):
            self.core.new_session()
            self.tick()

    def tick(self):
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_ros_now is not None and ros_now < self.last_ros_now:
            self.core.new_session()
        self.last_ros_now = ros_now
        state = self.core.report(time.monotonic(), ros_now)
        self.pub.publish(String(data=json.dumps(state, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackingMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
