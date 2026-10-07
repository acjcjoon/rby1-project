"""Restamp rosbag sensors and optionally freeze stereo after a phase marker."""

import copy
import json
from collections import deque

from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .sensor_replay import rebase_stamp_ns


INPUT_PREFIX = '/rby1/vslam/replay_input'
MARKER_TOPIC = '/rby1/vslam/replay_marker'
TOPICS = {
    'left': ('/d435/d435/infra1/image_rect_raw', Image),
    'right': ('/d435/d435/infra2/image_rect_raw', Image),
    'left_info': ('/d435/d435/infra1/camera_info', CameraInfo),
    'right_info': ('/d435/d435/infra2/camera_info', CameraInfo),
    'imu': ('/d435/d435/imu', Imu),
}


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def set_stamp_ns(message, value):
    message.header.stamp.sec = value // 1_000_000_000
    message.header.stamp.nanosec = value % 1_000_000_000


class SensorReplayRelay(Node):
    def __init__(self):
        super().__init__('sensor_replay_relay')
        self.declare_parameter('stereo_mode', 'replay')
        self.declare_parameter('imu_mode', 'replay')
        self.declare_parameter('stereo_rate_hz', 30.0)
        self.declare_parameter('stereo_slop_ms', 5.0)
        self.declare_parameter('freeze_marker', 'returned_stationary')
        self.stereo_mode = str(self.get_parameter('stereo_mode').value)
        self.imu_mode = str(self.get_parameter('imu_mode').value)
        self.stereo_rate = float(self.get_parameter('stereo_rate_hz').value)
        self.stereo_slop_ns = int(float(
            self.get_parameter('stereo_slop_ms').value) * 1_000_000)
        self.freeze_marker = str(self.get_parameter('freeze_marker').value)
        if self.stereo_mode not in ('replay', 'freeze_after_marker'):
            raise ValueError('stereo_mode must be replay or freeze_after_marker')
        if self.imu_mode not in ('replay', 'none'):
            raise ValueError('imu_mode must be replay or none')
        if self.stereo_rate <= 0:
            raise ValueError('stereo replay rate must be positive')
        if self.stereo_slop_ns < 0 or not self.freeze_marker:
            raise ValueError('invalid stereo freeze settings')

        sensor_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publishers_by_name = {
            name: self.create_publisher(message_type, topic, sensor_qos)
            for name, (topic, message_type) in TOPICS.items()
            if name != 'imu' or self.imu_mode != 'none'
        }
        self.subscriptions_by_name = {}
        for name, (topic, message_type) in TOPICS.items():
            self.subscriptions_by_name[name] = self.create_subscription(
                message_type, INPUT_PREFIX + topic,
                lambda message, key=name: self._on_input(key, message), sensor_qos)
        self.marker_subscription = self.create_subscription(
            String, INPUT_PREFIX + MARKER_TOPIC, self._on_marker, 10)

        self.status_publisher = self.create_publisher(
            String, '/rby1/vslam/sensor_replay_status', 10)
        self.create_service(Trigger, '/rby1/vslam/sensor_replay/arm', self._arm)
        self.create_service(Trigger, '/rby1/vslam/sensor_replay/stop', self._stop)
        self.stereo_timer = self.create_timer(1.0 / self.stereo_rate, self._publish_fixed_stereo)

        self._stereo_candidates = {
            name: deque(maxlen=30) for name in ('left', 'right', 'left_info', 'right_info')
        }
        self._fixed_stereo = None
        self._freeze_marker_seen = False
        self._freeze_pending = False
        self._armed = False
        self._started = False
        self._first_source_ns = None
        self._first_output_ns = None
        self._counts = {name: 0 for name in TOPICS}
        self._publish_status('idle')

    def _publish_status(self, state, **extra):
        value = {
            'state': state,
            'stereo_mode': self.stereo_mode,
            'imu_mode': self.imu_mode,
            'freeze_marker': self.freeze_marker,
            'freeze_marker_seen': self._freeze_marker_seen,
            'freeze_pending': self._freeze_pending,
            'fixed_stereo_ready': self._fixed_stereo is not None,
            'counts': self._counts,
        }
        value.update(extra)
        self.status_publisher.publish(String(data=json.dumps(value, separators=(',', ':'))))

    def _cache_post_marker_stereo(self, name, message):
        self._stereo_candidates[name].append((stamp_ns(message), copy.deepcopy(message)))
        if self._fixed_stereo is not None:
            return
        names = ('left', 'right', 'left_info', 'right_info')
        for anchor_stamp, _ in self._stereo_candidates['left']:
            matches = {}
            for key in names:
                candidates = self._stereo_candidates[key]
                if not candidates:
                    break
                candidate = min(candidates, key=lambda item: abs(item[0] - anchor_stamp))
                if abs(candidate[0] - anchor_stamp) > self.stereo_slop_ns:
                    break
                matches[key] = candidate[1]
            if len(matches) == len(names):
                self._fixed_stereo = matches
                self._freeze_pending = False
                self._publish_status('stereo_frozen')
                self.get_logger().info(
                    'Captured and froze the first coherent stereo tuple after '
                    f'{self.freeze_marker!r}')
                break

    def _on_marker(self, message):
        if (not self._armed or self.stereo_mode != 'freeze_after_marker' or
                message.data != self.freeze_marker):
            return
        self._freeze_marker_seen = True
        self._freeze_pending = True
        self._fixed_stereo = None
        for candidates in self._stereo_candidates.values():
            candidates.clear()
        self._publish_status('freeze_marker_seen')
        self.get_logger().info(
            f'Received {self.freeze_marker!r}; waiting for the next coherent stereo tuple')

    def _begin_if_needed(self, source_ns):
        if self._started:
            return
        self._first_source_ns = source_ns
        self._first_output_ns = self.get_clock().now().nanoseconds
        self._started = True
        self._publish_status('running')
        self.get_logger().info('Replay epoch established; publishing rebased sensor data')

    def _rebased(self, message):
        output = copy.deepcopy(message)
        target = rebase_stamp_ns(
            stamp_ns(message), self._first_source_ns, self._first_output_ns)
        set_stamp_ns(output, target)
        return output

    def _on_input(self, name, message):
        if not self._armed:
            return
        self._begin_if_needed(stamp_ns(message))
        if name == 'imu':
            if self.imu_mode == 'replay':
                self.publishers_by_name['imu'].publish(self._rebased(message))
                self._counts['imu'] += 1
            return

        if self.stereo_mode == 'replay':
            publish_recorded = True
        else:
            if self._freeze_pending:
                self._cache_post_marker_stereo(name, message)
            publish_recorded = self._fixed_stereo is None
        if publish_recorded:
            self.publishers_by_name[name].publish(self._rebased(message))
            self._counts[name] += 1

    def _arm(self, _request, response):
        if self._armed:
            response.success = False
            response.message = 'already armed'
            return response
        self._armed = True
        self._started = False
        self._first_source_ns = None
        self._first_output_ns = None
        self._freeze_marker_seen = False
        self._freeze_pending = False
        self._fixed_stereo = None
        for candidates in self._stereo_candidates.values():
            candidates.clear()
        self._counts = {name: 0 for name in TOPICS}
        self._publish_status('armed')
        response.success = True
        response.message = 'armed; start the full bag playback now'
        return response

    def _stop(self, _request, response):
        freeze_complete = (
            self.stereo_mode != 'freeze_after_marker' or
            (self._freeze_marker_seen and self._fixed_stereo is not None))
        self._armed = False
        self._started = False
        self._publish_status('stopped', freeze_complete=freeze_complete)
        response.success = freeze_complete
        if freeze_complete:
            response.message = 'sensor output stopped; requested stereo freeze completed'
        else:
            response.message = (
                'sensor output stopped, but returned_stationary marker or coherent '
                'post-marker stereo tuple was missing')
        return response

    def _publish_fixed_stereo(self):
        if (not self._armed or not self._started or
                self.stereo_mode != 'freeze_after_marker' or
                self._fixed_stereo is None):
            return
        current = self.get_clock().now().nanoseconds
        for name in ('left_info', 'right_info', 'left', 'right'):
            message = copy.deepcopy(self._fixed_stereo[name])
            set_stamp_ns(message, current)
            self.publishers_by_name[name].publish(message)
            self._counts[name] += 1


def main(args=None):
    rclpy.init(args=args)
    node = SensorReplayRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
