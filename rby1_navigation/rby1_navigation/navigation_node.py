#!/usr/bin/env python3
"""Odometry closed-loop server producing raw mobile-base velocity."""

from collections import OrderedDict
import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from rby1_navigation.navigation_protocol import (
    COMMAND_ID_PATTERN,
    ProtocolError,
    encode_state,
    parse_command,
)
from rby1_navigation.se2_utils import (
    body_frame_error,
    compose,
    normalize_angle,
    pose2d_from_pose,
    proportional_velocity,
)


class NavigationNode(Node):
    """Convert relative planner goals into ``cmd_raw`` commands."""

    TERMINAL_HISTORY_SIZE = 20

    def __init__(self):
        super().__init__('rby1_navigation_node')
        self._declare_parameters()
        self._read_parameters()

        self.current_odom = None
        self.odom_received_at = None

        self.active_command_id = None
        self.active_target = None
        self.active_deadline = 0.0
        self.initial_position_error = 0.0
        self.initial_yaw_error = 0.0
        self.position_error = 0.0
        self.yaw_error = 0.0
        self.progress = 0.0
        self.phase = 'IDLE'
        self.pending_terminal = None
        self.finalization_started_at = 0.0

        self.last_state_publish_at = 0.0
        self.active_last_state = ('accepted', 0.0, '')
        self.terminal_history = OrderedDict()

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            qos,
        )
        self.cmd_raw_pub = self.create_publisher(
            Twist,
            self.cmd_raw_topic,
            qos,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            qos,
        )
        self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )
        self.get_logger().info(
            'Odometry navigation server ready: '
            f'command={self.command_topic}, state={self.state_topic}, '
            f'odom={self.odom_topic}, '
            f'cmd_raw={self.cmd_raw_topic}'
        )

    def _declare_parameters(self):
        self.declare_parameter('command_topic', '/rby1/navigation/command')
        self.declare_parameter('state_topic', '/rby1/navigation/state')
        self.declare_parameter('odom_topic', '/rby1/odom')
        self.declare_parameter('cmd_raw_topic', '/rby1/cmd_raw')
        self.declare_parameter('max_linear_velocity', 0.05)
        self.declare_parameter('max_angular_velocity', 0.35)
        self.declare_parameter('linear_kp', 1.0)
        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('position_tolerance', 0.007)
        self.declare_parameter('yaw_tolerance', math.radians(1.0))
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('state_publish_rate_hz', 5.0)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('default_command_timeout_sec', 30.0)
        self.declare_parameter('max_command_timeout_sec', 120.0)

    def _read_parameters(self):
        self.command_topic = self._string_parameter('command_topic')
        self.state_topic = self._string_parameter('state_topic')
        self.odom_topic = self._string_parameter('odom_topic')
        self.cmd_raw_topic = self._string_parameter('cmd_raw_topic')
        self.max_linear = self._positive_parameter('max_linear_velocity')
        self.max_angular = self._positive_parameter('max_angular_velocity')
        self.kp_linear = self._positive_parameter('linear_kp')
        self.kp_angular = self._positive_parameter('angular_kp')
        self.position_tolerance = self._positive_parameter(
            'position_tolerance'
        )
        self.yaw_tolerance = self._positive_parameter('yaw_tolerance')
        self.control_rate_hz = self._positive_parameter('control_rate_hz')
        self.state_publish_period = 1.0 / self._positive_parameter(
            'state_publish_rate_hz'
        )
        self.odom_timeout = self._positive_parameter('odom_timeout_sec')
        self.default_command_timeout = self._positive_parameter(
            'default_command_timeout_sec'
        )
        self.max_command_timeout = self._positive_parameter(
            'max_command_timeout_sec'
        )
        if self.default_command_timeout > self.max_command_timeout:
            raise ValueError(
                'default_command_timeout_sec cannot exceed '
                'max_command_timeout_sec'
            )

    def _string_parameter(self, name):
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be positive and finite')
        return value

    def odom_callback(self, message):
        self.current_odom = pose2d_from_pose(message.pose.pose)
        self.odom_received_at = time.monotonic()

    def command_callback(self, message):
        try:
            command = parse_command(
                message.data,
                default_timeout_sec=self.default_command_timeout,
                max_timeout_sec=self.max_command_timeout,
            )
        except ProtocolError as exc:
            command_id = self._best_effort_command_id(message.data)
            self.get_logger().warning(f'Rejected navigation command: {exc}')
            if command_id:
                if command_id == self.active_command_id:
                    self._publish_current_state(force=True)
                else:
                    self._publish_state(
                        'rejected',
                        0.0,
                        str(exc),
                        command_id=command_id,
                        force=True,
                        remember_terminal=True,
                    )
            return

        if command.action == 'cancel':
            self._handle_cancel(command.command_id)
            return

        previous = self.terminal_history.get(command.command_id)
        if previous is not None:
            self._publish_serialized(previous)
            return
        if self.active_command_id is not None:
            if command.command_id == self.active_command_id:
                self._publish_current_state(force=True)
            else:
                self._publish_state(
                    'rejected',
                    0.0,
                    f'navigation is busy with {self.active_command_id}',
                    command_id=command.command_id,
                    force=True,
                    remember_terminal=True,
                )
            return
        if not self._odom_is_fresh(time.monotonic()):
            self._publish_state(
                'rejected',
                0.0,
                'fresh odometry is unavailable',
                command_id=command.command_id,
                force=True,
                remember_terminal=True,
            )
            return

        relative_target = (command.x, command.y, command.yaw)
        self.active_command_id = command.command_id
        self.active_target = compose(self.current_odom, relative_target)
        self.active_deadline = time.monotonic() + command.timeout_sec
        self.initial_position_error = math.hypot(command.x, command.y)
        self.initial_yaw_error = abs(normalize_angle(command.yaw))
        self.position_error = self.initial_position_error
        self.yaw_error = normalize_angle(command.yaw)
        self.progress = 0.0
        self.pending_terminal = None
        self.phase = 'RUNNING'

        self._publish_state(
            'accepted',
            0.0,
            'relative odometry goal accepted',
            force=True,
        )
        self.get_logger().info(
            f'Accepted move_to {command.command_id}: '
            f'x={command.x:.3f} m, y={command.y:.3f} m, '
            f'yaw={command.yaw:.3f} rad'
        )

    @staticmethod
    def _best_effort_command_id(data):
        try:
            payload = json.loads(data)
        except (TypeError, json.JSONDecodeError):
            return ''
        if not isinstance(payload, dict):
            return ''
        value = str(payload.get('command_id', '')).strip()
        return value if COMMAND_ID_PATTERN.fullmatch(value) else ''

    def _handle_cancel(self, command_id):
        previous = self.terminal_history.get(command_id)
        if previous is not None:
            self._publish_serialized(previous)
            return
        if command_id != self.active_command_id:
            self._publish_state(
                'canceled',
                0.0,
                'command was not active',
                command_id=command_id,
                force=True,
                remember_terminal=True,
            )
            return
        self._begin_finalization('canceled', 'canceled by planner')

    def control_loop(self):
        now = time.monotonic()
        if self.active_command_id is None:
            if now - self.last_state_publish_at >= self.state_publish_period:
                self._publish_state(
                    'idle',
                    0.0,
                    'ready',
                    command_id='',
                    force=True,
                )
            return

        if self.phase == 'FINALIZING':
            self.publish_stop()
            self._finish_terminal()
            return

        if not self._odom_is_fresh(now):
            self._begin_finalization('failed', 'odometry became stale')
            return
        if now >= self.active_deadline:
            self._begin_finalization('failed', 'navigation command timed out')
            return

        reached = self._drive_to_target()
        self.progress = self._calculate_progress()
        if reached:
            self.progress = 1.0
            self._begin_finalization('succeeded', 'goal reached')
            return
        if now - self.last_state_publish_at >= self.state_publish_period:
            self._publish_state(
                'running',
                self.progress,
                'driving to odometry target',
                force=True,
            )

    def _drive_to_target(self):
        dx_body, dy_body, self.yaw_error = body_frame_error(
            self.current_odom,
            self.active_target,
        )
        self.position_error = math.hypot(dx_body, dy_body)
        if (
            self.position_error <= self.position_tolerance
            and abs(self.yaw_error) <= self.yaw_tolerance
        ):
            self.publish_stop()
            return True

        vx, vy, wz = proportional_velocity(
            self.current_odom,
            self.active_target,
            linear_kp=self.kp_linear,
            angular_kp=self.kp_angular,
            max_linear=self.max_linear,
            max_angular=self.max_angular,
        )

        self.publish_velocity(vx, vy, wz)
        return False

    def _calculate_progress(self):
        remaining_ratios = []
        if self.initial_position_error > self.position_tolerance:
            remaining_ratios.append(
                self.position_error / self.initial_position_error
            )
        if self.initial_yaw_error > self.yaw_tolerance:
            remaining_ratios.append(
                abs(self.yaw_error) / self.initial_yaw_error
            )
        if not remaining_ratios:
            return 1.0
        return max(0.0, min(1.0, 1.0 - max(remaining_ratios)))

    def _odom_is_fresh(self, now):
        return (
            self.current_odom is not None
            and self.odom_received_at is not None
            and 0.0 <= now - self.odom_received_at <= self.odom_timeout
        )

    def _begin_finalization(self, state, message):
        if self.phase == 'FINALIZING':
            return
        self.publish_stop()
        self.pending_terminal = (state, message)
        self.phase = 'FINALIZING'
        self.finalization_started_at = time.monotonic()

    def _finish_terminal(self):
        if self.active_command_id is None or self.pending_terminal is None:
            return
        state, message = self.pending_terminal
        progress = 1.0 if state == 'succeeded' else self.progress
        self._publish_state(
            state,
            progress,
            message,
            force=True,
            remember_terminal=True,
        )
        self.get_logger().info(
            f'Navigation {state}: {self.active_command_id} ({message})'
        )
        self.active_command_id = None
        self.active_target = None
        self.active_deadline = 0.0
        self.phase = 'IDLE'
        self.pending_terminal = None
        self.progress = 0.0
        self.active_last_state = ('accepted', 0.0, '')

    def publish_stop(self):
        self.publish_velocity(0.0, 0.0, 0.0)

    def publish_velocity(self, vx, vy, wz):
        """Publish raw velocity; rby1_control owns final cmd_vel output."""
        message = Twist()
        message.linear.x = float(vx)
        message.linear.y = float(vy)
        message.angular.z = float(wz)
        self.cmd_raw_pub.publish(message)

    def _publish_current_state(self, *, force=False):
        state, progress, message = self.active_last_state
        self._publish_state(state, progress, message, force=force)

    def _publish_state(
        self,
        state,
        progress,
        message,
        *,
        command_id=None,
        force=False,
        remember_terminal=False,
    ):
        now = time.monotonic()
        if (
            not force
            and now - self.last_state_publish_at < self.state_publish_period
        ):
            return
        command_id = (
            self.active_command_id
            if command_id is None
            else command_id
        )
        serialized = encode_state(
            command_id=command_id or '',
            state=state,
            progress=progress,
            message=message,
            stamp_ns=self.get_clock().now().nanoseconds,
            position_error=(
                self.position_error
                if command_id == self.active_command_id
                else None
            ),
            yaw_error=(
                abs(self.yaw_error)
                if command_id == self.active_command_id
                else None
            ),
        )
        if command_id and command_id == self.active_command_id:
            self.active_last_state = (state, progress, message)
        self.last_state_publish_at = now
        if remember_terminal and command_id:
            self.terminal_history[command_id] = serialized
            self.terminal_history.move_to_end(command_id)
            while len(self.terminal_history) > self.TERMINAL_HISTORY_SIZE:
                self.terminal_history.popitem(last=False)
        self._publish_serialized(serialized)

    def _publish_serialized(self, serialized):
        message = String()
        message.data = serialized
        self.state_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
