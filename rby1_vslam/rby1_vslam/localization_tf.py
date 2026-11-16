"""Wheel state estimation with explicit, converged corrections at stations.

The driver exclusively owns odom->base. During transit map->odom is frozen;
visual loss or a tracker restart never deletes the established wheel estimate.
"""

import copy
import json
import math
import time
import uuid

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .geometry import compose, finite, inverse, planar_correction, wrap, xyzw, xyz, yaw
from .station_estimator import ConvergenceWindow


class LocalizationTF(Node):
    def __init__(self):
        super().__init__('localization_tf')
        defaults = {
            'slam_odom_topic': '/rby1/vslam/slam_odom',
            'wheel_odom_topic': '/rby1/odom',
            'localization_odom_topic': '/rby1/vslam/localization_odom',
            'tracking_status_topic': '/rby1/vslam/tracking_status',
            'origin_status_topic': '/rby1/vslam/map_origin_status',
            'status_topic': '/rby1/vslam/localization_status',
            'navigation_status_topic': '/rby1/vslam/navigation_status',
            'station_mode_service': '/rby1/vslam/set_station_mode',
            'reset_visual_reference_service': '/rby1/vslam/reset_visual_reference',
            'map_frame': 'vslam_map', 'odom_frame': 'odom', 'base_frame': 'base',
            'wheel_base_frame': 'base_footprint',
            'require_origin_alignment': False, 'require_navigation_idle': True,
            'pose_timeout_sec': 0.5, 'tracking_timeout_sec': 0.75,
            'wheel_timeout_sec': 0.5, 'navigation_timeout_sec': 0.75,
            'stationary_hold_sec': 0.75,
            'stationary_linear_mps': 0.01, 'stationary_angular_rps': 0.02,
            'stationary_position_tolerance_m': 0.015,
            'stationary_yaw_tolerance_rad': 0.02,
            'convergence_window_sec': 1.5, 'convergence_min_samples': 15,
            'convergence_position_tolerance_m': 0.015,
            'convergence_yaw_tolerance_rad': 0.015,
            'max_correction_jump_m': 0.5, 'max_correction_jump_rad': 0.8,
            'max_wheel_linear_mps': 1.5, 'max_wheel_angular_rps': 3.0,
            'wheel_jump_tolerance_m': 0.05, 'wheel_jump_tolerance_rad': 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        frames = (self.p['map_frame'], self.p['odom_frame'], self.p['base_frame'])
        if not all(frames) or len(set(frames)) != 3:
            raise ValueError('map, odom and base frames must be nonempty and distinct')
        for name, default in defaults.items():
            if isinstance(default, (float, int)) and not isinstance(default, bool):
                if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                    raise ValueError(name + ' must be positive and finite')
        if self.p['convergence_min_samples'] < 2:
            raise ValueError('convergence_min_samples must be at least two')
        self.window = ConvergenceWindow(
            self.p['convergence_window_sec'], self.p['convergence_min_samples'],
            self.p['convergence_position_tolerance_m'],
            self.p['convergence_yaw_tolerance_rad'], self.p['pose_timeout_sec'])
        self.buffer = Buffer(cache_time=Duration(seconds=10.))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 5)
        self.odom_pub = self.create_publisher(Odometry, self.p['localization_odom_topic'], 5)
        self.session = uuid.uuid4().hex
        self.visual_session = ''
        self.reference_session = ''
        self.visual_alignment = ((0., 0., 0.), (0., 0., 0., 1.))
        self.tracking_ok = False
        self.tracking_received = -math.inf
        self.origin_session = ''
        self.origin_aligned = not self.p['require_origin_alignment']
        self.navigation = {}
        self.navigation_received = -math.inf
        self.station_mode = False
        self.station_converged = False
        self.correction_pending = False
        self.reset_reference_requested = False
        self.correction = None
        self.correction_count = 0
        self.pending = None
        self.last_pose_stamp = 0
        self.pose_after_ns = 0
        self.wheel = None
        self.wheel_received = -math.inf
        self.stopped_since = None
        self.stopped_pose = None
        self.last_ros_ns = 0
        self.last_status = 0.
        self.detail = 'request station correction to initialize localization'
        self.create_subscription(Odometry, self.p['wheel_odom_topic'], self.on_wheel,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['slam_odom_topic'], self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(String, self.p['tracking_status_topic'], self.on_tracking, 5)
        self.create_subscription(String, self.p['navigation_status_topic'], self.on_navigation, 5)
        if self.p['require_origin_alignment']:
            self.create_subscription(String, self.p['origin_status_topic'], self.on_origin, 5)
        self.create_service(SetBool, self.p['station_mode_service'], self.set_station_mode)
        self.create_service(Trigger, self.p['reset_visual_reference_service'], self.reset_visual_reference)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.tick, clock=self.steady)

    def clear_window(self):
        self.pending = None
        self.window.clear()

    def invalidate_wheel_reference(self, detail):
        """A changed wheel origin invalidates the established map transform."""
        self.correction = None
        self.wheel = None
        self.wheel_received = -math.inf
        self.stopped_since = None
        self.station_mode = False
        self.station_converged = False
        self.correction_pending = False
        self.reset_reference_requested = False
        # Keep a valid visual-to-map reference: a wheel reset changes odom,
        # not the established map coordinate system or saved station poses.
        self.session = uuid.uuid4().hex
        self.last_pose_stamp = 0
        self.pose_after_ns = self.get_clock().now().nanoseconds
        self.clear_window()
        self.detail = detail + '; request a new station correction'

    def set_station_mode(self, request, response):
        self.station_mode = bool(request.data)
        self.station_converged = False
        self.correction_pending = self.station_mode
        self.reset_reference_requested = False
        self.clear_window()
        self.pose_after_ns = self.get_clock().now().nanoseconds
        self.detail = ('station hold requested; waiting for stop and VO convergence'
                       if self.station_mode else 'wheel odometry transit; visual correction frozen')
        response.success = True
        response.message = self.detail
        # Tell the command gate to stop/cancel immediately, before sampling.
        self.report(time.monotonic(), self.get_clock().now())
        return response

    def reset_visual_reference(self, _request, response):
        now, ros_now = time.monotonic(), self.get_clock().now()
        if (not self.station_mode or self.correction is None
                or not self.is_stationary(now, ros_now) or not self.navigation_idle(now)):
            response.success = False
            response.message = 'requires initialized localization, stationary station hold and idle navigation'
        elif not self.tracking_available(now) or not self.origin_ready():
            response.success = False
            response.message = 'wait for visual tracking and map origin alignment'
        else:
            self.reset_reference_requested = True
            self.station_converged = False
            self.correction_pending = True
            self.clear_window()
            self.pose_after_ns = ros_now.nanoseconds
            self.detail = 'new visual reference requested; waiting for convergence'
            response.success = True
            response.message = self.detail + '; current wheel/map position will be preserved'
        self.report(now, ros_now)
        return response

    def on_tracking(self, msg):
        try:
            state = json.loads(msg.data)
            session = state.get('session_id', '')
            if not isinstance(session, str) or not session:
                raise ValueError('invalid visual session')
            ok = state.get('ready') is True and state.get('tracking_ok') is True
        except (ValueError, TypeError, AttributeError):
            session, ok = self.visual_session, False
        if session != self.visual_session:
            self.clear_window()
            self.last_pose_stamp = 0
            self.pose_after_ns = self.get_clock().now().nanoseconds
            self.reset_reference_requested = False
            self.station_converged = False
            self.origin_aligned = not self.p['require_origin_alignment']
            self.origin_session = ''
        if not ok:
            self.clear_window()
        self.visual_session = session
        self.tracking_ok = ok
        self.tracking_received = time.monotonic()

    def on_origin(self, msg):
        try:
            state = json.loads(msg.data)
            session = state.get('session_id', '')
            aligned = isinstance(session, str) and bool(session) and state.get('aligned') is True
        except (ValueError, TypeError, AttributeError):
            session, aligned = '', False
        aligned = aligned and session == self.visual_session
        if aligned != self.origin_aligned or session != self.origin_session:
            self.clear_window()
            self.pose_after_ns = self.get_clock().now().nanoseconds
        self.origin_session, self.origin_aligned = session, aligned

    def on_navigation(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                raise ValueError('invalid navigation status')
        except (ValueError, TypeError):
            state = {}
        self.navigation = state
        self.navigation_received = time.monotonic()

    def on_wheel(self, msg):
        now, ros_now = time.monotonic(), self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        try:
            if (msg.header.frame_id != self.p['odom_frame']
                    or msg.child_frame_id not in (self.p['base_frame'], self.p['wheel_base_frame'])):
                raise ValueError('wheel odometry frame mismatch')
            previous = self.wheel
            previous_stamp = Time.from_msg(previous.header.stamp).nanoseconds if previous else 0
            if previous_stamp and stamp < previous_stamp:
                self.invalidate_wheel_reference('wheel timestamp moved backwards')
                previous = None
            elif previous_stamp and stamp == previous_stamp:
                return  # Duplicates must not refresh wheel health or standstill.
            if stamp <= 0 or not -0.05 <= (ros_now.nanoseconds - stamp) / 1e9 <= self.p['wheel_timeout_sec']:
                raise ValueError('wheel odometry timestamp stale/future')
            if msg.child_frame_id != self.p['base_frame']:
                # This robot publishes wheel odometry at base_footprint. The
                # launch provides an identity base_footprint->base transform;
                # validate it before relabelling pose, twist and covariance.
                offset = self.buffer.lookup_transform(
                    self.p['base_frame'], msg.child_frame_id, Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.)).transform
                if (max(abs(v) for v in finite(xyz(offset.translation))) > 1e-6
                        or max(abs(v) for v in finite(xyzw(offset.rotation))[:3]) > 1e-6
                        or abs(abs(offset.rotation.w) - 1.) > 1e-6):
                    raise ValueError('wheel base frame must have an identity transform to base')
                msg = copy.deepcopy(msg)
                msg.child_frame_id = self.p['base_frame']
            position = finite(xyz(msg.pose.pose.position))
            heading = yaw(xyzw(msg.pose.pose.orientation))
            vx, vy, _ = finite(xyz(msg.twist.twist.linear))
            _, _, wz = finite(xyz(msg.twist.twist.angular))
            finite(msg.pose.covariance)
            finite(msg.twist.covariance)
            if previous is not None:
                old_position = xyz(previous.pose.pose.position)
                dt = (stamp - previous_stamp) / 1e9
                if (math.hypot(position[0] - old_position[0], position[1] - old_position[1])
                        > self.p['max_wheel_linear_mps'] * dt + self.p['wheel_jump_tolerance_m']
                        or abs(wrap(heading - yaw(xyzw(previous.pose.pose.orientation))))
                        > self.p['max_wheel_angular_rps'] * dt + self.p['wheel_jump_tolerance_rad']):
                    self.invalidate_wheel_reference('wheel odometry pose discontinuity')
        except (TransformException, ValueError, TypeError, OverflowError) as exc:
            self.wheel_received = -math.inf
            self.stopped_since = None
            self.clear_window()
            self.detail = str(exc)
            return
        if now - self.wheel_received > self.p['wheel_timeout_sec']:
            self.stopped_since = None
        stopped = (math.hypot(vx, vy) <= self.p['stationary_linear_mps']
                   and abs(wz) <= self.p['stationary_angular_rps'])
        if stopped and self.stopped_since is not None:
            old_x, old_y, old_yaw = self.stopped_pose
            stopped = (math.hypot(position[0] - old_x, position[1] - old_y)
                       <= self.p['stationary_position_tolerance_m']
                       and abs(wrap(heading - old_yaw)) <= self.p['stationary_yaw_tolerance_rad'])
        if not stopped:
            self.stopped_since = None
            self.station_converged = False
            self.clear_window()
        elif self.stopped_since is None:
            self.stopped_since = now
            self.stopped_pose = (position[0], position[1], heading)
            self.pose_after_ns = max(self.pose_after_ns, stamp)
        self.wheel, self.wheel_received = msg, now
        if self.correction is not None:
            self.publish_odometry()

    def on_pose(self, msg):
        if not self.station_mode or not self.correction_pending:
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp <= max(self.last_pose_stamp, self.pose_after_ns):
            return
        if self.pending is not None and stamp <= Time.from_msg(self.pending[0].header.stamp).nanoseconds:
            return
        self.pending = (msg, time.monotonic())

    def wheel_fresh(self, now, ros_now):
        return bool(self.wheel is not None
                    and now - self.wheel_received <= self.p['wheel_timeout_sec']
                    and -0.05 <= (ros_now.nanoseconds - Time.from_msg(self.wheel.header.stamp).nanoseconds)
                    / 1e9 <= self.p['wheel_timeout_sec'])

    def is_stationary(self, now, ros_now):
        return (self.wheel_fresh(now, ros_now) and self.stopped_since is not None
                and now - self.stopped_since >= self.p['stationary_hold_sec'])

    def navigation_idle(self, now):
        return (not self.p['require_navigation_idle'] or (
            now - self.navigation_received <= self.p['navigation_timeout_sec']
            and self.navigation.get('enabled') is False
            and self.navigation.get('cancel_pending') is False
            and self.navigation.get('active_goal_count') == 0))

    def tracking_available(self, now):
        return (self.tracking_ok and bool(self.visual_session)
                and now - self.tracking_received <= self.p['tracking_timeout_sec'])

    def origin_ready(self):
        return (not self.p['require_origin_alignment'] or (
            self.origin_aligned and self.origin_session == self.visual_session))

    def station_guard(self, now, ros_now):
        if not self.navigation_idle(now):
            return 'waiting for navigation disabled, cancellation complete and no active goals'
        if not self.is_stationary(now, ros_now):
            return 'waiting for fresh wheel odometry and stationary hold'
        if not self.tracking_available(now):
            return 'waiting for visual tracking; established wheel estimate preserved'
        if not self.origin_ready():
            return 'waiting for cuVSLAM ground-aligned map origin'
        if (self.reference_session and self.reference_session != self.visual_session
                and not self.reset_reference_requested):
            return 'visual session changed; explicit reset_visual_reference required'
        return ''

    def consume_pose(self, now, ros_now):
        if self.pending is None:
            return
        msg, received = self.pending
        stamp = Time.from_msg(msg.header.stamp)
        try:
            age = (ros_now.nanoseconds - stamp.nanoseconds) / 1e9
            if not -0.05 <= age <= self.p['pose_timeout_sec']:
                raise ValueError('visual pose timestamp stale/future')
            if msg.header.frame_id != self.p['map_frame'] or msg.child_frame_id != self.p['base_frame']:
                raise ValueError('SLAM pose frame mismatch')
            # Never substitute latest TF: head/VO and wheel data must agree in time.
            wheel = self.buffer.lookup_transform(
                self.p['odom_frame'], self.p['base_frame'], stamp,
                timeout=Duration(seconds=0.)).transform
            raw = planar_correction(xyz(msg.pose.pose.position), xyzw(msg.pose.pose.orientation),
                                    xyz(wheel.translation), xyzw(wheel.rotation))
            # Evaluate convergence at the stopped robot, not at the odom
            # origin: small yaw noise must not grow with distance from origin.
            anchor_x, anchor_y, anchor_yaw = self.stopped_pose
            anchor = ((anchor_x, anchor_y, 0.),
                      (0., 0., math.sin(anchor_yaw / 2.), math.cos(anchor_yaw / 2.)))
            sample = compose(*raw, *anchor)
            candidate = self.window.add(stamp.nanoseconds / 1e9, *sample)
            self.last_pose_stamp = stamp.nanoseconds
            self.pending = None
            self.detail = 'station hold; collecting converged visual pose'
            if candidate is None:
                return
            candidate = compose(*candidate, *inverse(*anchor))
            if self.reset_reference_requested:
                self.visual_alignment = compose(*self.correction, *inverse(*candidate))
                corrected = self.correction
            else:
                corrected = compose(*self.visual_alignment, *candidate)
                if self.correction is not None:
                    old_pose = compose(*self.correction, xyz(wheel.translation), xyzw(wheel.rotation))
                    new_pose = compose(*corrected, xyz(wheel.translation), xyzw(wheel.rotation))
                    if (math.hypot(new_pose[0][0] - old_pose[0][0], new_pose[0][1] - old_pose[0][1])
                            > self.p['max_correction_jump_m']
                            or abs(wrap(yaw(new_pose[1]) - yaw(old_pose[1])))
                            > self.p['max_correction_jump_rad']):
                        raise ValueError('station correction exceeds jump limit; wheel estimate preserved')
            reanchored = self.reset_reference_requested
            self.correction = corrected
            self.reference_session = self.visual_session
            self.reset_reference_requested = False
            self.station_converged = True
            self.correction_pending = False
            self.correction_count += 1
            self.window.clear()
            self.detail = ('visual reference reanchored; wheel/map position preserved'
                           if reanchored else 'station correction applied; station hold remains active')
            self.publish_odometry()
            self.get_logger().info(self.detail)
        except TransformException:
            self.detail = 'waiting for acquisition-time odom->base TF'
            if now - received > min(0.2, self.p['pose_timeout_sec']):
                self.clear_window()
        except (ValueError, TypeError, OverflowError) as exc:
            self.clear_window()
            self.detail = str(exc)

    def publish_odometry(self):
        if self.wheel is None or self.correction is None:
            return
        out = copy.deepcopy(self.wheel)
        out.header.frame_id = self.p['map_frame']
        position, rotation = compose(*self.correction, xyz(self.wheel.pose.pose.position),
                                     xyzw(self.wheel.pose.pose.orientation))
        out.pose.pose.position.x, out.pose.pose.position.y = position[:2]
        out.pose.pose.position.z = 0.
        heading = yaw(rotation)
        out.pose.pose.orientation.x = out.pose.pose.orientation.y = 0.
        out.pose.pose.orientation.z = math.sin(heading / 2.)
        out.pose.pose.orientation.w = math.cos(heading / 2.)
        # Rotate wheel uncertainty into map XY; retain conservative uncertainty
        # for constrained axes and a floor for the sampled station correction.
        angle = yaw(self.correction[1])
        c, s = math.cos(angle), math.sin(angle)
        jac = [[float(i == j) for j in range(6)] for i in range(6)]
        jac[0][0], jac[0][1], jac[1][0], jac[1][1] = c, -s, s, c
        cov = self.wheel.pose.covariance
        tmp = [[sum(jac[i][k] * cov[k * 6 + j] for k in range(6))
                for j in range(6)] for i in range(6)]
        out.pose.covariance = [sum(tmp[i][k] * jac[j][k] for k in range(6))
                               for i in range(6) for j in range(6)]
        for index in (0, 7, 35):
            out.pose.covariance[index] += 0.02 ** 2
        for index in (14, 21, 28):
            out.pose.covariance[index] = 1e6
        self.odom_pub.publish(out)

    def tick(self):
        now, ros_now = time.monotonic(), self.get_clock().now()
        if self.last_ros_ns and ros_now.nanoseconds < self.last_ros_ns:
            # A ROS clock reset invalidates wheel history, unlike a visual outage.
            self.invalidate_wheel_reference('ROS clock reset')
        self.last_ros_ns = ros_now.nanoseconds
        if self.station_mode and self.correction_pending:
            guard = self.station_guard(now, ros_now)
            if guard:
                self.clear_window()
                self.detail = guard
            else:
                self.consume_pose(now, ros_now)
        elif self.station_mode and not self.station_converged:
            self.detail = 'station correction no longer current; request a new station correction'
        if self.correction is not None and self.wheel_fresh(now, ros_now):
            out = TransformStamped()
            out.header.stamp = ros_now.to_msg()
            out.header.frame_id = self.p['map_frame']
            out.child_frame_id = self.p['odom_frame']
            position, rotation = self.correction
            out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = position
            (out.transform.rotation.x, out.transform.rotation.y,
             out.transform.rotation.z, out.transform.rotation.w) = rotation
            self.broadcaster.sendTransform(out)
        if now - self.last_status >= 0.1:
            self.report(now, ros_now)

    def report(self, now, ros_now):
        healthy = self.correction is not None and self.wheel_fresh(now, ros_now)
        mode = ('station_corrected' if self.station_converged else 'station_waiting') if self.station_mode else 'wheel_odometry'
        detail = self.detail
        if self.correction is None and not self.station_mode:
            mode = 'uninitialized'
            detail = 'localization not initialized; stop and request station correction before saving or navigation'
        elif self.correction is not None and not healthy:
            detail = 'wheel odometry unavailable/stale'
        status = {
            'healthy': healthy, 'initialized': self.correction is not None,
            'navigation_ready': healthy and not self.station_mode,
            'station_mode': self.station_mode, 'station_converged': self.station_converged,
            'correction_pending': self.correction_pending,
            'stationary_ready': self.is_stationary(now, ros_now),
            'navigation_idle': self.navigation_idle(now),
            'mode': mode, 'session_id': self.session,
            'visual_session_id': self.visual_session,
            'visual_reference_changed': bool(self.reference_session and self.reference_session != self.visual_session),
            'visual_tracking_ok': self.tracking_available(now),
            'correction_count': self.correction_count,
            'map_frame': self.p['map_frame'], 'odom_frame': self.p['odom_frame'],
            'detail': detail,
        }
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        self.last_status = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LocalizationTF()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
