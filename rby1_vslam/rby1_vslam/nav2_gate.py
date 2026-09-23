"""UPC Nav2 command gate: watchdogs and cmd_raw, never navigation planning."""

import json
import math
import time

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .geometry import finite, yaw
from .nav2_gate_core import CancelEndpoint, GateLimits, VelocityGate


def _stamp(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid timestamp nanoseconds')
    return stamp.sec + stamp.nanosec * 1e-9


def _validate_odometry(msg):
    pose, twist = msg.pose.pose, msg.twist.twist
    finite((pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
            *msg.pose.covariance, *msg.twist.covariance))
    if not msg.header.frame_id or not msg.child_frame_id:
        raise ValueError('odometry frames are empty')


class Nav2Gate(Node):
    def __init__(self):
        super().__init__('nav2_gate')
        defaults = {
            'enabled': False, 'control_rate_hz': 20.0,
            'nav2_cmd_topic': '/rby1/vslam/nav2_cmd_vel',
            'cmd_raw_topic': '/rby1/cmd_raw',
            'wheel_odom_topic': '/rby1/odom',
            'slam_odom_topic': '/rby1/vslam/slam_odom',
            'bridge_status_topic': '/rby1/vslam/bridge_status',
            'localization_status_topic': '/rby1/vslam/localization_status',
            'status_topic': '/rby1/vslam/navigation_status',
            'base_frame': 'base', 'slam_frame': 'vslam_map',
            'navigation_actions': ['/rby1/vslam/nav2/navigate_to_pose',
                                   '/rby1/vslam/nav2/navigate_through_poses'],
            'cancel_response_timeout_sec': 3.0, 'cancel_retry_sec': 1.0,
        }
        defaults.update(vars(GateLimits()))
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for name in ('control_rate_hz', 'cancel_response_timeout_sec', 'cancel_retry_sec'):
            if not math.isfinite(float(self.p[name])) or self.p[name] <= 0:
                raise ValueError(name + ' must be positive and finite')
        actions = self.p['navigation_actions']
        if (not actions or len(set(actions)) != len(actions) or
                any(not name.startswith('/') or name.endswith('/') for name in actions)):
            raise ValueError('navigation_actions must be unique absolute action names')
        self.core = VelocityGate(GateLimits(**{key: self.p[key] for key in vars(GateLimits())}))
        self.arm_on_ready = bool(self.p['enabled'])
        self.velocity_pub = self.create_publisher(Twist, self.p['cmd_raw_topic'], 1)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 5)
        self.create_subscription(Twist, self.p['nav2_cmd_topic'], self.on_command, 1)
        self.create_subscription(Odometry, self.p['wheel_odom_topic'], self.on_wheel,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['slam_odom_topic'], self.on_slam,
                                 qos_profile_sensor_data)
        self.create_subscription(String, self.p['bridge_status_topic'], self.on_bridge, 5)
        self.create_subscription(String, self.p['localization_status_topic'],
                                 self.on_localization, 5)
        self.create_service(SetBool, '/rby1/vslam/enable', self.on_enable)
        self.create_service(Trigger, '/rby1/vslam/cancel', self.on_cancel)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cancel_clients = {}
        self.cancel_endpoints = {}
        self.action_statuses = {}
        self.cancel_futures = {}
        self.cancel_next_attempt = {}
        self.cancel_serial = None
        for name in actions:
            self.cancel_clients[name] = self.create_client(CancelGoal, name + '/_action/cancel_goal')
            self.cancel_endpoints[name] = CancelEndpoint()
            self.action_statuses[name] = {}
            self.cancel_next_attempt[name] = 0.0
            self.create_subscription(
                GoalStatusArray, name + '/_action/status',
                lambda msg, target=name: self.on_action_status(target, msg), status_qos)
        self.last_status_at = 0.0
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.p['control_rate_hz'], self.tick, clock=self.watchdog_clock)
        self.publish_stop()
        self.get_logger().info(
            'Nav2 gate starts stopped; verifying sensor health and canceling old Nav2 goals. '
            'This node must be the only cmd_raw publisher.')

    def ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_stop(self):
        self.velocity_pub.publish(Twist())

    def _stop_if_latched(self, was_enabled):
        if was_enabled and not self.core.enabled:
            self.arm_on_ready = False
            self.publish_stop()
            self.get_logger().warning(self.core.detail)

    def on_command(self, msg):
        was_enabled = self.core.enabled
        try:
            values = finite((msg.linear.x, msg.linear.y, msg.linear.z,
                             msg.angular.x, msg.angular.y, msg.angular.z))
            if any(abs(values[index]) > 1e-9 for index in (2, 3, 4)):
                raise ValueError('Nav2 command contains unsupported nonplanar velocity')
            self.core.update_command((values[0], values[1], values[5]), time.monotonic())
        except (ValueError, TypeError, OverflowError) as exc:
            self.core.fault(str(exc))
        self._stop_if_latched(was_enabled)

    def on_wheel(self, msg):
        was_enabled = self.core.enabled
        try:
            _validate_odometry(msg)
            # The wheel driver's child can be base or base_footprint. Nav2's
            # TF path is validated by localization_tf; this gate needs freshness.
            self.core.update_wheel(_stamp(msg.header.stamp), time.monotonic(), self.ros_now())
        except (ValueError, TypeError, OverflowError) as exc:
            self.core.invalidate_wheel('invalid wheel odometry: ' + str(exc))
        self._stop_if_latched(was_enabled)

    def on_slam(self, msg):
        was_enabled = self.core.enabled
        try:
            _validate_odometry(msg)
            if (msg.header.frame_id != self.p['slam_frame'] or
                    msg.child_frame_id != self.p['base_frame']):
                raise ValueError('SLAM odometry frames differ from slam_frame/base_frame')
            pose = msg.pose.pose
            q = pose.orientation
            planar = (pose.position.x, pose.position.y, yaw((q.x, q.y, q.z, q.w)))
            self.core.update_slam(planar, msg.header.frame_id, _stamp(msg.header.stamp),
                                  time.monotonic(), self.ros_now())
        except (ValueError, TypeError, OverflowError) as exc:
            self.core.invalidate_slam('invalid base SLAM pose: ' + str(exc))
        self._stop_if_latched(was_enabled)

    def on_bridge(self, msg):
        was_enabled = self.core.enabled
        try:
            state = json.loads(msg.data)
            session = state.get('session_id', '')
            if not isinstance(session, str):
                raise ValueError('session_id must be a string')
            self.core.update_bridge(state.get('connected') is True,
                                    state.get('tracking_ok') is True, session, time.monotonic())
        except (ValueError, TypeError, AttributeError):
            self.core.update_bridge(False, False, '', time.monotonic())
        self._stop_if_latched(was_enabled)

    def on_localization(self, msg):
        was_enabled = self.core.enabled
        try:
            state = json.loads(msg.data)
            session = state.get('session_id', '')
            if not isinstance(session, str):
                raise ValueError('session_id must be a string')
            healthy = state.get('healthy') is True and state.get('map_frame') == self.p['slam_frame']
            self.core.update_localization(healthy, session, time.monotonic())
        except (ValueError, TypeError, AttributeError):
            self.core.update_localization(False, '', time.monotonic())
        self._stop_if_latched(was_enabled)

    def on_enable(self, request, response):
        self.arm_on_ready = False
        self.core.set_competing_publisher(self.count_publishers(self.p['cmd_raw_topic']) > 1)
        if request.data:
            response.success, response.message = self.core.enable(time.monotonic(), self.ros_now())
        else:
            self.core.stop('disabled by operator; canceling Nav2 goals')
            response.success = True
            response.message = 'output stopped; Nav2 cancellation is asynchronous (see navigation_status)'
        self.publish_stop()  # rearming clears cached commands
        return response

    def on_cancel(self, request, response):
        del request
        self.arm_on_ready = False
        self.core.stop('operator canceled; explicit re-enable and a new Nav2 goal required')
        self.publish_stop()
        response.success = True
        response.message = 'output stopped; canceling both Nav2 actions (see navigation_status)'
        return response

    def on_action_status(self, name, message):
        statuses = {bytes(item.goal_info.goal_id.uuid): int(item.status)
                    for item in message.status_list}
        self.action_statuses[name] = statuses
        endpoint = self.cancel_endpoints[name]
        endpoint.status(statuses)
        if not self.core.enabled and endpoint.active:
            if not self.core.cancel_pending:
                self.core.stop('Nav2 goal received while gate disabled; canceling it')
                self.publish_stop()
            elif any(value in (0, 1, 2, 3) and key not in endpoint.requested_ids
                     for key, value in statuses.items()):
                # A new goal arrived after a no-goal cancel reply. Cancel it too.
                endpoint.acknowledged = False

    def _pump_cancellation(self, now):
        if self.cancel_serial != self.core.cancel_serial:
            for name, (future, _) in self.cancel_futures.items():
                self.cancel_clients[name].remove_pending_request(future)
                future.cancel()
            self.cancel_futures.clear()
            self.cancel_serial = self.core.cancel_serial
            for name in self.cancel_endpoints:
                endpoint = CancelEndpoint()
                endpoint.status(self.action_statuses[name])
                self.cancel_endpoints[name] = endpoint
                self.cancel_next_attempt[name] = now
        if not self.core.cancel_pending:
            return
        for name, endpoint in self.cancel_endpoints.items():
            client = self.cancel_clients[name]
            if name in self.cancel_futures:
                future, sent_at = self.cancel_futures[name]
                if future.done():
                    self.cancel_futures.pop(name)
                    try:
                        response = future.result()
                        endpoint.response(response.return_code,
                                          [bytes(item.goal_id.uuid) for item in response.goals_canceling])
                    except Exception as exc:  # rclpy transport failures stay latched
                        endpoint.acknowledged = False
                        endpoint.detail = 'cancel service failed: ' + str(exc)
                    self.cancel_next_attempt[name] = now + self.p['cancel_retry_sec']
                elif now - sent_at > self.p['cancel_response_timeout_sec']:
                    client.remove_pending_request(future)
                    future.cancel()
                    self.cancel_futures.pop(name)
                    endpoint.detail = 'cancel service timed out; waiting to retry'
                    self.cancel_next_attempt[name] = now + self.p['cancel_retry_sec']
                continue
            if endpoint.acknowledged:
                # Never mistake accepting cancellation for its completion.
                # If terminal status is lost, stay stopped until repaired.
                continue
            if now < self.cancel_next_attempt[name]:
                continue
            if not client.service_is_ready():
                endpoint.detail = 'cancel service unavailable; gate remains stopped'
                self.cancel_next_attempt[name] = now + self.p['cancel_retry_sec']
                continue
            try:
                # All-zero UUID and timestamp means cancel every goal (ROS 2
                # action_msgs/srv/CancelGoal policy, Humble and Jazzy). Humble
                # rcl_action/action_server.c:716-747 returns code 0 with an empty
                # goal list when idle; rclcpp_action/server.cpp:502-523 keeps it.
                future = client.call_async(CancelGoal.Request())
                self.cancel_futures[name] = future, now
                endpoint.detail = 'cancel-all request pending'
            except Exception as exc:
                endpoint.detail = 'could not send cancel request: ' + str(exc)
                self.cancel_next_attempt[name] = now + self.p['cancel_retry_sec']
        if all(endpoint.complete for endpoint in self.cancel_endpoints.values()):
            self.core.cancellation_complete(self.cancel_serial)

    def tick(self):
        now, ros_now = time.monotonic(), self.ros_now()
        was_enabled = self.core.enabled
        self.core.set_competing_publisher(self.count_publishers(self.p['cmd_raw_topic']) > 1)
        velocity = self.core.tick(now, ros_now)
        self._stop_if_latched(was_enabled)
        self._pump_cancellation(now)
        if self.arm_on_ready:
            success, _ = self.core.enable(now, ros_now)
            if success:
                self.arm_on_ready = False  # startup opt-in is consumed exactly once
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = velocity
        self.velocity_pub.publish(out)
        if now - self.last_status_at >= 0.2:
            self.report(now, ros_now)
            self.last_status_at = now

    def report(self, now, ros_now):
        status = {
            'state': self.core.state, 'detail': self.core.detail,
            'enabled': self.core.enabled, 'session_id': self.core.session,
            'input_guard': self.core.guard(now, ros_now),
            'cancel_pending': self.core.cancel_pending,
            'cancellation': {name: {'complete': endpoint.complete, 'detail': endpoint.detail}
                             for name, endpoint in self.cancel_endpoints.items()},
            'command_age_sec': (None if self.core.command_received is None else
                                now - self.core.command_received),
        }
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Nav2Gate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
