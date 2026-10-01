"""UPC operator adapter using the existing VSLAM/Nav2 interfaces.

No driver, gate, SLAM or planner implementation is duplicated here. All
callbacks execute on the web node's ROS executor; HTTP only writes a mailbox.
"""
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from rby1_vslam.waypoint_store import (
    Waypoint, default_waypoints_path, load_waypoints, next_name, save_waypoints,
)


class VslamAdapter:
    def __init__(self, node):
        self.node, self.box = node, node.mailbox
        defaults = {
            'map_frame': 'vslam_map', 'base_frame': 'base',
            'waypoints_file': str(default_waypoints_path(
                get_package_share_directory('rby1_vslam'))),
            'navigate_action': '/rby1/vslam/nav2/navigate_to_pose',
            'enable_service': '/rby1/vslam/enable',
            'cancel_service': '/rby1/vslam/cancel',
            'bridge_status_topic': '/rby1/vslam/bridge_status',
            'navigation_status_topic': '/rby1/vslam/navigation_status',
            'localization_status_topic': '/rby1/vslam/localization_status',
            'map_topic': '/rby1/vslam/nav2/map',
            'path_topic': '/rby1/vslam/nav2/plan',
        }
        for key, value in defaults.items():
            node.declare_parameter(key, value)
        self.p = {key: node.get_parameter(key).value for key in defaults}
        self.file = Path(self.p['waypoints_file']).expanduser()
        if not self.file.is_absolute():
            raise ValueError('waypoints_file must be absolute')
        self.waypoints = []
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, node)
        self.action = ActionClient(node, NavigateToPose, self.p['navigate_action'])
        self.enable_client = node.create_client(SetBool, self.p['enable_service'])
        self.cancel_client = node.create_client(Trigger, self.p['cancel_service'])
        self.gate = {}
        self.gate_received = None
        self.last_cancel = -math.inf
        self.generation = 0
        self.active = False
        self.goal_handle = None
        self.deadline = None
        self.feedback_at = 0.0
        self.state = {'state': 'idle'}
        for key, field in (('bridge_status_topic', 'bridge'),
                           ('navigation_status_topic', 'gate'),
                           ('localization_status_topic', 'localization')):
            node.create_subscription(String, self.p[key],
                lambda msg, name=field: self.on_status(name, msg), 5)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, self.p['map_topic'], self.on_map, map_qos)
        node.create_subscription(NavPath, self.p['path_topic'], self.on_path, 1)
        node.create_timer(0.2, self.tick)
        try:
            self.reload()
        except (OSError, ValueError, TypeError) as exc:
            self.log('error', 'Waypoint load failed: ' + str(exc))

    def log(self, level, message):
        self.box.add_event({'level': level, 'message': message})

    def on_status(self, field, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('status is not an object')
            if field == 'gate':
                self.gate, self.gate_received = value, time.monotonic()
            self.box.update_operator(**{field: value})
        except (ValueError, TypeError):
            self.log('error', 'Invalid ' + field + ' status')

    def current_pose(self):
        transform = self.buffer.lookup_transform(
            self.p['map_frame'], self.p['base_frame'], Time(),
            timeout=Duration(seconds=0.0))
        stamp = transform.header.stamp
        age = self.node.get_clock().now().nanoseconds * 1e-9 - (
            stamp.sec + stamp.nanosec * 1e-9)
        if not -0.05 <= age <= 0.8:
            raise ValueError('map-to-base pose is stale')
        t = transform.transform
        q = t.rotation
        return {'x': t.translation.x, 'y': t.translation.y,
                'yaw': math.atan2(2 * (q.w*q.z + q.x*q.y),
                                  1 - 2 * (q.y*q.y + q.z*q.z))}

    def tick(self):
        try:
            pose = self.current_pose()
        except (TransformException, ValueError):
            pose = None
        gate_fresh = self.gate_received is not None and time.monotonic() - self.gate_received < 1.0
        self.box.update_operator(pose=pose, gate_fresh=gate_fresh,
                                 navigation=dict(self.state))
        if self.active and self.deadline is not None and time.monotonic() > self.deadline:
            self.log('error', 'Navigation gate/goal acknowledgement timed out')
            self.cancel()

    def publish_waypoints(self):
        self.box.update_operator(waypoints=[asdict(w) for w in self.waypoints],
                                 next_name=next_name(self.waypoints),
                                 map_frame=self.p['map_frame'])

    def reload(self):
        self.waypoints = load_waypoints(self.file, self.p['map_frame'])
        self.publish_waypoints()

    def waypoint(self, name):
        for item in self.waypoints:
            if item.name == name:
                return item
        raise ValueError('Waypoint does not exist: ' + name)

    def dispatch(self, operation, args):
        try:
            if operation == 'waypoint_reload':
                self.reload()
            elif operation == 'waypoint_save':
                name = args['name']
                if any(w.name == name for w in self.waypoints):
                    raise ValueError('Waypoint name already exists')
                pose = self.current_pose()
                changed = self.waypoints + [Waypoint(name, **pose)]
                save_waypoints(self.file, changed, self.p['map_frame'])
                self.waypoints = changed
                self.publish_waypoints()
                self.log('info', 'Saved current pose: ' + name)
            elif operation == 'waypoint_delete':
                target = self.waypoint(args['name'])
                changed = [w for w in self.waypoints if w != target]
                save_waypoints(self.file, changed, self.p['map_frame'])
                self.waypoints = changed
                self.publish_waypoints()
            elif operation == 'navigate':
                self.navigate(self.waypoint(args['name']))
            elif operation == 'nav_cancel':
                self.cancel()
        except Exception as exc:
            self.log('error', operation + ': ' + str(exc))
            if operation == 'navigate':
                self.cancel()

    def cancel(self):
        self.generation += 1
        self.active, self.deadline = False, None
        self.box.navigation_finished()
        self.state = {'state': 'canceled'}
        self.node.publish_command('stop', {})
        # Gate cancel disables forwarding immediately and cancels all Nav2
        # goals with its existing bounded, retried cancellation machinery.
        if self.cancel_client.service_is_ready():
            future = self.cancel_client.call_async(Trigger.Request())
            future.add_done_callback(self.on_cancel)
        elif self.enable_client.service_is_ready():
            self.enable_client.call_async(SetBool.Request(data=False))
            self.log('error', 'Nav2 cancel service unavailable; gate disable requested')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.last_cancel = time.monotonic()

    def on_cancel(self, future):
        try:
            result = future.result()
            self.log('info' if result.success else 'error', 'Nav cancel: ' + result.message)
        except Exception as exc:
            self.log('error', 'Nav cancel failed: ' + str(exc))

    def manual_ready(self):
        if self.active:
            self.cancel()
            return False
        if self.gate_received is None:
            # Manual mapping can run without Nav2 installed/running.
            return not self.enable_client.service_is_ready()
        if time.monotonic() - self.gate_received >= 1.0:
            return False
        if self.gate.get('enabled') is False and self.gate.get('cancel_pending') is False:
            return True
        if time.monotonic() - self.last_cancel > 1.0:
            self.cancel()
        return False

    def navigate(self, waypoint):
        if self.active:
            raise ValueError('Cancel the active goal first')
        if not self.enable_client.service_is_ready() or not self.action.server_is_ready():
            raise ValueError('Nav2 gate/action is not ready')
        self.current_pose()  # Never navigate against an expired global pose.
        self.generation += 1
        serial = self.generation
        self.active = True
        self.deadline = time.monotonic() + 5.0
        self.state = {'state': 'enabling', 'target': waypoint.name}
        future = self.enable_client.call_async(SetBool.Request(data=True))
        future.add_done_callback(lambda done: self.after_enable(done, waypoint, serial))

    def after_enable(self, future, waypoint, serial):
        if serial != self.generation or not self.active:
            # A late enable acknowledgement must not rearm a stopped session.
            if self.cancel_client.service_is_ready():
                self.cancel_client.call_async(Trigger.Request())
            return
        try:
            result = future.result()
            if not result.success:
                raise ValueError(result.message)
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = self.p['map_frame']
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = waypoint.x, waypoint.y
            goal.pose.pose.orientation.z = math.sin(waypoint.yaw / 2)
            goal.pose.pose.orientation.w = math.cos(waypoint.yaw / 2)
            self.state['state'] = 'sending'
            self.deadline = time.monotonic() + 5.0
            future = self.action.send_goal_async(goal,
                feedback_callback=lambda msg: self.on_feedback(msg, serial))
            future.add_done_callback(lambda done: self.on_goal(done, serial))
        except Exception as exc:
            self.log('error', 'Navigation enable failed: ' + str(exc))
            self.cancel()

    def on_goal(self, future, serial):
        try:
            handle = future.result()
            if serial != self.generation or not self.active:
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
                return
            if handle is None or not handle.accepted:
                raise ValueError('Nav2 rejected goal')
            self.goal_handle, self.deadline = handle, None
            self.state['state'] = 'navigating'
            handle.get_result_async().add_done_callback(lambda done: self.on_result(done, serial))
        except Exception as exc:
            if serial == self.generation:
                self.log('error', 'Goal response failed: ' + str(exc))
                self.cancel()

    def on_feedback(self, message, serial):
        if serial == self.generation and self.active:
            distance = float(message.feedback.distance_remaining)
            if math.isfinite(distance):
                self.state['distance_remaining'] = distance

    def on_result(self, future, serial):
        if serial != self.generation:
            return
        try:
            status = future.result().status
            self.log('info', 'Nav2 goal finished: status=' + str(status))
        except Exception as exc:
            status = None
            self.log('error', 'Nav2 result failed: ' + str(exc))
        self.goal_handle = None
        self.cancel()
        self.state = {'state': 'succeeded' if status == 4 else 'failed', 'status': status}

    def on_path(self, message):
        if message.header.frame_id != self.p['map_frame']:
            return
        stride = max(1, math.ceil(len(message.poses) / 1000))
        points = [[p.pose.position.x, p.pose.position.y] for p in message.poses[::stride]]
        if all(math.isfinite(v) for point in points for v in point):
            self.box.update_operator(path=points)

    def on_map(self, message):
        if message.header.frame_id != self.p['map_frame']:
            return
        info = message.info
        if info.width <= 0 or info.height <= 0 or not math.isfinite(info.resolution) or info.resolution <= 0:
            return
        if len(message.data) != info.width * info.height:
            return
        stride = max(1, math.ceil(max(info.width, info.height) / 300))
        width, height = math.ceil(info.width / stride), math.ceil(info.height / stride)
        cells = [int(message.data[y*info.width+x])
                 for y in range(0, info.height, stride) for x in range(0, info.width, stride)]
        q = info.origin.orientation
        origin = [info.origin.position.x, info.origin.position.y,
                  math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
        if not all(math.isfinite(v) for v in origin):
            return
        self.box.update_operator(map={'width': width, 'height': height,
            'resolution': info.resolution * stride, 'origin': origin, 'cells': cells})
