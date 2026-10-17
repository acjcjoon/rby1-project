"""Align a new cuVSLAM map to the wheel-odometry ground frame.

cuVSLAM tracks d435_link because the camera is mounted on a moving head.  Its
default map origin is therefore the initial camera pose.  At the start of each
mapping session, set that camera pose to the current odom->d435_link transform
so the resulting map has the robot base on z=0 while retaining the real camera
height and orientation.
"""

import json
import math
import time

from isaac_ros_visual_slam_interfaces.srv import SetSlamPose
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import quaternion, xyzw, xyz


class MapOriginInitializer(Node):
    def __init__(self):
        super().__init__('map_origin_initializer')
        defaults = {
            'enabled': True,
            'odom_frame': 'odom',
            'camera_frame': 'd435_link',
            'tracking_status_topic': '/rby1/vslam/tracking_status',
            'origin_status_topic': '/rby1/vslam/map_origin_status',
            'set_pose_service': '/visual_slam/set_slam_pose',
            'minimum_camera_height_m': 0.2,
            'maximum_camera_height_m': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if not 0 < self.p['minimum_camera_height_m'] < self.p['maximum_camera_height_m']:
            raise ValueError('camera height limits must be positive and ordered')
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.client = self.create_client(SetSlamPose, self.p['set_pose_service'])
        self.publisher = self.create_publisher(String, self.p['origin_status_topic'], 5)
        self.create_subscription(String, self.p['tracking_status_topic'], self.on_tracking, 5)
        self.session = ''
        self.tracking_ok = False
        self.aligned_session = ''
        self.request_session = ''
        self.future = None
        self.detail = ('waiting for mapping mode' if not self.p['enabled']
                       else 'waiting for visual tracking')
        self.last_warning = 0.0
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.tick, clock=self.steady)

    def on_tracking(self, message):
        try:
            state = json.loads(message.data)
            session = state.get('session_id', '')
            tracking_ok = bool(session and state.get('ready') is True
                               and state.get('tracking_ok') is True)
            if not isinstance(session, str):
                raise ValueError('invalid tracking session')
        except (ValueError, TypeError, AttributeError):
            session, tracking_ok = '', False
        if session != self.session:
            self.aligned_session = ''
            self.future = None
            self.request_session = ''
            self.detail = 'waiting for visual tracking'
        self.session, self.tracking_ok = session, tracking_ok

    def _finish_request(self):
        if self.future is None or not self.future.done():
            return
        session, future = self.request_session, self.future
        self.future = None
        self.request_session = ''
        try:
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError('cuVSLAM rejected set_slam_pose')
            if session == self.session and self.tracking_ok:
                self.aligned_session = session
                self.detail = 'map origin aligned to odom ground'
                self.get_logger().info(self.detail)
        except Exception as exc:  # rclpy futures can surface middleware exceptions here.
            self.detail = 'map-origin request failed: ' + str(exc)
            self.warn(self.detail)

    def _request_alignment(self):
        transform = self.buffer.lookup_transform(
            self.p['odom_frame'], self.p['camera_frame'], Time(),
            timeout=Duration(seconds=0.0)).transform
        position = xyz(transform.translation)
        rotation = quaternion(xyzw(transform.rotation))
        if not all(math.isfinite(value) for value in position):
            raise ValueError('odom-to-camera TF is not finite')
        if not (self.p['minimum_camera_height_m'] <= position[2]
                <= self.p['maximum_camera_height_m']):
            raise ValueError('camera height is outside the configured safety range: '
                             + str(position[2]))
        request = SetSlamPose.Request()
        request.pose.position.x, request.pose.position.y, request.pose.position.z = position
        (request.pose.orientation.x, request.pose.orientation.y,
         request.pose.orientation.z, request.pose.orientation.w) = rotation
        self.request_session = self.session
        self.future = self.client.call_async(request)
        self.detail = 'setting cuVSLAM map origin from odom-to-camera TF'

    def tick(self):
        self._finish_request()
        if (self.p['enabled'] and self.tracking_ok and self.session
                and self.aligned_session != self.session and self.future is None):
            try:
                if not self.client.service_is_ready():
                    self.detail = 'waiting for cuVSLAM set_slam_pose service'
                else:
                    self._request_alignment()
            except (TransformException, ValueError, TypeError, OverflowError) as exc:
                self.detail = 'waiting for valid odom-to-camera TF: ' + str(exc)
                self.warn(self.detail)
        status = {
            'aligned': bool(not self.p['enabled'] or (
                self.session and self.aligned_session == self.session)),
            'session_id': self.session,
            'detail': self.detail,
        }
        self.publisher.publish(String(data=json.dumps(status, allow_nan=False)))

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warning > 2.0:
            self.get_logger().warning(message)
            self.last_warning = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapOriginInitializer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
