"""Station operator: wheel transit, stationary VO correction and named goals."""

import json
import math
from pathlib import Path
import sys
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from PyQt5.QtCore import QEvent, QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QListWidget, QMainWindow, QPushButton, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from .waypoint_store import (
    Waypoint, default_waypoints_path, load_waypoints, next_name, save_waypoints,
)


GOAL_STATUS_NAMES = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def _rpy(q):
    values = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid pose quaternion')
    x, y, z, w = (value / norm for value in values)
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


def _packet(message):
    packet = json.loads(message.data)
    if not isinstance(packet, dict):
        raise ValueError('status is not an object')
    return packet


class OperatorNode(Node):
    def __init__(self):
        super().__init__('station_operator')
        defaults = {
            'map_frame': 'vslam_map', 'base_frame': 'base',
            'waypoints_file': str(default_waypoints_path(
                get_package_share_directory('rby1_vslam'))),
            'navigate_action': '/rby1/vslam/nav2/navigate_to_pose',
            'enable_service': '/rby1/vslam/enable',
            'cancel_service': '/rby1/vslam/cancel',
            'station_service': '/rby1/vslam/set_station_mode',
            'reset_visual_service': '/rby1/vslam/reset_visual_reference',
            'control_command_topic': '/rby1/control/command',
            'control_state_topic': '/rby1/control/state',
            'control_event_topic': '/rby1/control/event',
            'vo_topic': '/rby1/vslam/camera_odometry',
            'base_vo_topic': '/rby1/vslam/odom',
            'localization_topic': '/rby1/vslam/localization_odom',
            'max_linear_speed': 0.2, 'max_angular_speed': 0.4,
            'status_timeout_sec': 1.0, 'operation_timeout_sec': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for name in ('max_linear_speed', 'max_angular_speed',
                     'status_timeout_sec', 'operation_timeout_sec'):
            value = float(self.p[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be positive and finite')
        self.waypoint_path = Path(str(self.p['waypoints_file'])).expanduser()
        if not self.waypoint_path.is_absolute():
            raise ValueError('waypoints_file must be absolute')
        self.map_frame = str(self.p['map_frame'])
        self.base_frame = str(self.p['base_frame'])
        self.navigate_client = ActionClient(self, NavigateToPose, self.p['navigate_action'])
        self.enable_client = self.create_client(SetBool, self.p['enable_service'])
        self.cancel_client = self.create_client(Trigger, self.p['cancel_service'])
        self.station_client = self.create_client(SetBool, self.p['station_service'])
        self.reset_client = self.create_client(Trigger, self.p['reset_visual_service'])
        self.control_pub = self.create_publisher(String, self.p['control_command_topic'], 20)
        self.statuses = {}
        self.poses = {}
        self.localization_pose_after_ns = 0
        self.log_sink = None
        self.operation_text = 'Waiting for localization / control backend'
        self.pending = None
        self.sequence = 0
        self.goal_handles = {}
        self.active_request = None
        self.startup_checked = False
        self.manual_active = False
        self.shutdown_future = None
        self.last_feedback = 0.0
        self.create_subscription(String, self.p['control_event_topic'], self._on_event, 20)
        for name, topic in (
            ('control', self.p['control_state_topic']),
            ('tracking', '/rby1/vslam/tracking_status'),
            ('adapter', '/rby1/vslam/pose_adapter_status'),
            ('localization', '/rby1/vslam/localization_status'),
            ('navigation', '/rby1/vslam/navigation_status'),
        ):
            self.create_subscription(
                String, topic, lambda msg, key=name: self._on_status(key, msg), 10)
        for name, topic in (
            ('Camera VO', self.p['vo_topic']), ('Base VO', self.p['base_vo_topic']),
            ('Localization', self.p['localization_topic']),
        ):
            self.create_subscription(
                Odometry, topic, lambda msg, key=name: self._on_pose(key, msg),
                qos_profile_sensor_data)

    def emit_log(self, message):
        self.get_logger().info(str(message))
        if self.log_sink:
            self.log_sink(str(message))

    def _on_event(self, message):
        try:
            packet = _packet(message)
            self.emit_log('CONTROL: ' + str(packet.get('message', message.data)))
        except (ValueError, TypeError):
            pass

    def _on_status(self, name, message):
        try:
            packet = _packet(message)
            if name == 'control' and (packet.get('schema_version') != 1 or
                                      packet.get('kind') != 'state'):
                return
            if name == 'localization':
                previous = self.status('localization')
                if (packet.get('session_id') != previous.get('session_id')
                        or packet.get('initialized') is not True
                        or previous.get('initialized') is not True):
                    # Odometry has no estimator session ID. Discard the last
                    # session's pose and wait for a sample acquired after this
                    # session has reported an initialized estimate.
                    self.poses.pop('Localization', None)
                    self.localization_pose_after_ns = self.get_clock().now().nanoseconds
            self.statuses[name] = (packet, time.monotonic())
        except (ValueError, TypeError):
            pass

    def _on_pose(self, name, message):
        try:
            if name == 'Localization':
                stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                if (not self.fresh('localization')
                        or self.status('localization').get('initialized') is not True
                        or stamp_ns <= self.localization_pose_after_ns):
                    return
            position = message.pose.pose.position
            values = (position.x, position.y, position.z, *_rpy(message.pose.pose.orientation))
            if not all(math.isfinite(value) for value in values):
                return
            self.poses[name] = (message, values, time.monotonic())
        except (ValueError, TypeError):
            pass

    def status(self, name):
        return self.statuses.get(name, ({}, -math.inf))[0]

    def fresh(self, name):
        return time.monotonic() - self.statuses.get(name, ({}, -math.inf))[1] < self.p['status_timeout_sec']

    def pose_fresh(self, name):
        item = self.poses.get(name)
        if item is None:
            return False
        msg, _, received = item
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return (time.monotonic() - received < self.p['status_timeout_sec']
                and -0.1 <= age < self.p['status_timeout_sec'])

    def current_pose(self):
        state = self.status('localization')
        if not self.fresh('localization'):
            raise ValueError('localization status is unavailable or stale')
        if state.get('initialized') is not True:
            raise ValueError('localization is not initialized; stop and request station correction')
        if state.get('healthy') is not True:
            raise ValueError('localization is unhealthy: ' + str(state.get('detail', 'estimate unavailable')))
        if (state.get('station_mode') is not True or state.get('station_converged') is not True
                or state.get('stationary_ready') is not True):
            raise ValueError('stop and complete station correction before saving a station')
        if not self.pose_fresh('Localization'):
            raise ValueError('localization pose is stale')
        msg, values, _ = self.poses['Localization']
        if msg.header.frame_id != self.map_frame or msg.child_frame_id != self.base_frame:
            raise ValueError('localization frames do not match waypoint frames')
        return values[0], values[1], values[5]

    def send_control(self, operation, arguments=None, announce=True):
        packet = {
            'schema_version': 1, 'kind': 'command', 'source': 'rby1_station_ui',
            'request_id': uuid.uuid4().hex, 'operation': operation,
            'arguments': dict(arguments or {}),
        }
        self.control_pub.publish(String(data=json.dumps(packet, allow_nan=False)))
        if announce:
            self.emit_log('Control request: ' + operation)

    def stop_manual(self):
        self.manual_active = False
        self.send_control('stop', {'publish_immediately': True}, announce=False)

    def _gate_present(self):
        return 'navigation' in self.statuses or self.enable_client.service_is_ready()

    def _localization_present(self):
        return 'localization' in self.statuses or self.station_client.service_is_ready()

    def _gate_idle(self):
        state = self.status('navigation')
        return (self.fresh('navigation') and state.get('enabled') is False
                and state.get('cancel_pending') is False
                and state.get('active_goal_count') == 0)

    def _disable_gate(self):
        if self.enable_client.service_is_ready():
            return self.enable_client.call_async(SetBool.Request(data=False))
        if self._gate_present():
            raise ValueError('navigation gate is unavailable; waiting for its recovery')
        return None

    def stop(self):
        self.startup_checked = True
        self.sequence += 1
        self.pending = None
        self.active_request = None
        self.stop_manual()
        for handle in list(self.goal_handles.values()):
            handle.cancel_goal_async()
        gate = None
        try:
            gate = self._disable_gate()
        except ValueError as exc:
            self.emit_log(str(exc))
        if gate is None and self.cancel_client.service_is_ready():
            self.cancel_client.call_async(Trigger.Request())
        self.operation_text = 'Stop requested; navigation cancellation in progress'
        return gate

    def _begin(self, kind, target=None):
        gate = self.stop()
        self.sequence += 1
        try:
            if self._gate_present() and gate is None:
                raise ValueError('Navigation gate is unavailable')
            self.pending = {
                'id': self.sequence, 'kind': kind, 'target': target, 'stage': 'gate',
                'future': gate, 'started': time.monotonic(),
            }
            self.operation_text = 'Waiting for navigation to stop'
        except ValueError as exc:
            self.fail(str(exc))

    def begin_manual(self, ready):
        if not self.fresh('control'):
            self.emit_log('Manual drive blocked: control backend is stale.')
            return
        self._begin('manual', ready)

    def enable_then_navigate(self, waypoint):
        state = self.status('localization')
        if (not self.fresh('localization') or state.get('initialized') is not True
                or state.get('healthy') is not True):
            self.emit_log('Goto blocked: first complete station VO correction.')
            return
        if state.get('station_mode') is True and (
                state.get('station_converged') is not True
                or state.get('stationary_ready') is not True):
            self.emit_log('Goto blocked: wait for stationary VO correction at this station.')
            return
        if not self._gate_present():
            self.emit_log('Goto blocked: navigation gate is unavailable.')
            return
        self._begin('navigate', waypoint)

    def request_station(self, label='Current station'):
        self._begin('station', label)

    def request_visual_reset(self):
        self._begin('reset', 'Visual reference recovery')

    def fail(self, message):
        self.stop()
        self.operation_text = message
        self.emit_log(message)

    def _response_ready(self, future):
        if future is None:
            return True
        if not future.done():
            return False
        response = future.result()
        if response is None or not response.success:
            raise ValueError(getattr(response, 'message', 'service returned no response'))
        return True

    def tick(self):
        if (not self.startup_checked and self.fresh('localization')
                and self.station_client.service_is_ready()):
            self.startup_checked = True
            if self.status('localization').get('initialized') is not True:
                self.request_station('Initial station')
        pending = self.pending
        if pending is None:
            return
        try:
            if (pending['stage'] != 'converge' and
                    time.monotonic() - pending['started'] > self.p['operation_timeout_sec']):
                raise ValueError('Operation timed out waiting for service/status; stopped')
            stage, kind = pending['stage'], pending['kind']
            if stage == 'gate':
                if not self._response_ready(pending['future']):
                    return
                if self._gate_present() and not self._gate_idle():
                    return
                station = kind in ('station', 'reset')
                if self.station_client.service_is_ready():
                    pending['stage'] = 'station_mode'
                    pending['mode_requested'] = time.monotonic()
                    pending['baseline'] = self.status('localization').get('correction_count', 0)
                    pending['future'] = self.station_client.call_async(SetBool.Request(data=station))
                    self.operation_text = ('Waiting for stationary VO correction' if station
                                           else 'Switching to wheel-only transit')
                elif self._localization_present() or kind != 'manual':
                    raise ValueError('Localization station service is unavailable')
                else:
                    self._manual_ready(pending)
            elif stage == 'station_mode':
                if not self._response_ready(pending['future']):
                    return
                state = self.status('localization')
                if (not self.fresh('localization') or self.statuses['localization'][1] <=
                        pending['mode_requested']):
                    return
                station = kind in ('station', 'reset')
                if state.get('station_mode') is not station:
                    return
                if station:
                    if kind == 'reset':
                        if state.get('stationary_ready') is not True:
                            self.operation_text = 'Waiting for the base to settle before VO recovery'
                            return
                        if not self.reset_client.service_is_ready():
                            raise ValueError('Visual reference recovery service is unavailable')
                        pending['future'] = self.reset_client.call_async(Trigger.Request())
                        pending['stage'] = 'reset'
                    else:
                        pending['stage'] = 'converge'
                elif kind == 'manual':
                    self._manual_ready(pending)
                elif state.get('navigation_ready') is True:
                    if not self.enable_client.service_is_ready():
                        raise ValueError('Navigation gate disappeared')
                    pending['future'] = self.enable_client.call_async(SetBool.Request(data=True))
                    pending['stage'] = 'enable'
                    pending['future'].add_done_callback(
                        lambda done, identifier=pending['id']:
                        self._after_enable_response(done, identifier))
            elif stage == 'reset':
                if self._response_ready(pending['future']):
                    # The reset service starts its own convergence window.
                    pending['stage'] = 'converge'
                    self.emit_log('VO recovery requested; waiting for a converged visual reference.')
            elif stage == 'enable':
                if self._response_ready(pending['future']):
                    self._send_goal(pending)
            elif stage == 'converge':
                state = self.status('localization')
                detail = str(state.get('detail', 'waiting for localization'))
                self.operation_text = str(pending['target']) + ': ' + detail
                if (self.fresh('localization') and state.get('healthy') is True
                        and state.get('station_mode') is True
                        and state.get('stationary_ready') is True
                        and state.get('station_converged') is True
                        and state.get('correction_count', 0) > pending['baseline']):
                    self.operation_text = str(pending['target']) + ': correction complete; station ready'
                    self.emit_log(self.operation_text)
                    self.pending = None
        except Exception as exc:
            self.fail('Blocked: ' + str(exc))

    def _manual_ready(self, pending):
        callback = pending['target']
        self.pending = None
        self.manual_active = True
        self.operation_text = 'Manual drive / wheel-only transit'
        callback()

    def _after_enable_response(self, future, request_id):
        del future
        # A timed-out or superseded enable must never arm navigation later.
        current = self.pending
        if not (current and current['id'] == request_id and current['stage'] == 'enable'):
            if self.active_request != request_id:
                self.stop()
                self.operation_text = 'Stopped after a delayed navigation enable response'

    def manual_allowed(self):
        if not self.fresh('control'):
            return False
        if self._gate_present() and not self._gate_idle():
            return False
        if self._localization_present():
            if not self.fresh('localization') or self.status('localization').get('station_mode') is not False:
                return False
        return self.manual_active

    def set_manual_velocity(self, vx, vy, wz):
        if self.manual_allowed():
            self.send_control('set_velocity', {'vx': vx, 'vy': vy, 'wz': wz}, announce=False)
            return True
        return False

    def _send_goal(self, pending):
        if not self.navigate_client.server_is_ready():
            raise ValueError('Nav2 NavigateToPose action is unavailable')
        waypoint, request_id = pending['target'], pending['id']
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        goal.pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        self.pending = None
        self.active_request = request_id
        self.operation_text = 'Travelling to ' + waypoint.name + ' / wheel-only localization'
        future = self.navigate_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda done: self._on_goal_response(done, request_id, waypoint.name))

    def _on_goal_response(self, future, request_id, station_name):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                raise ValueError('Nav2 rejected the goal')
            self.goal_handles[request_id] = handle
            if request_id != self.active_request:
                handle.cancel_goal_async()
            result = handle.get_result_async()
            result.add_done_callback(lambda done: self._on_goal_result(done, request_id, station_name))
        except Exception as exc:
            if request_id == self.active_request:
                self.stop()
                self.fail(str(exc))

    def _on_feedback(self, message):
        if self.active_request is None or time.monotonic() - self.last_feedback < 0.5:
            return
        self.last_feedback = time.monotonic()
        remaining = float(getattr(message.feedback, 'distance_remaining', float('nan')))
        if math.isfinite(remaining):
            self.emit_log(f'Distance remaining: {remaining:.2f} m')

    def _on_goal_result(self, future, request_id, station_name):
        self.goal_handles.pop(request_id, None)
        if request_id != self.active_request:
            return
        try:
            status = int(future.result().status)
            if status == 4:
                self.emit_log(station_name + ': Nav2 arrived; stopping and waiting for VO correction.')
                self.request_station(station_name)
            else:
                self.stop()
                self.operation_text = 'Navigation ' + GOAL_STATUS_NAMES.get(status, str(status))
                self.emit_log(self.operation_text)
        except Exception as exc:
            self.stop()
            self.fail('Nav2 result error: ' + str(exc))


class OperatorWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.waypoints = []
        self.manual_velocity = None
        self.manual_held = False
        self.setWindowTitle('RBY1 Station Operator')
        self.resize(1120, 860)
        self._build()
        self.node.log_sink = self.log
        self._load()
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)
        self.timer.start()

    def _build(self):
        root = QWidget(self)
        layout = QVBoxLayout(root)
        self.operation_label = QLabel(self.node.operation_text)
        self.operation_label.setWordWrap(True)
        self.operation_label.setStyleSheet('font-size: 16px; font-weight: bold; padding: 5px;')
        layout.addWidget(self.operation_label)
        self.control_label = QLabel('Control: waiting')
        self.tracking_label = QLabel('VO tracking: waiting')
        self.tracking_label.setWordWrap(True)
        self.adapter_label = QLabel('Base conversion: waiting for adapter status')
        self.adapter_label.setWordWrap(True)
        self.localization_label = QLabel('Localization: waiting')
        self.localization_label.setWordWrap(True)
        self.gate_label = QLabel('Navigation: waiting')
        self.gate_label.setWordWrap(True)
        for label in (self.control_label, self.tracking_label, self.adapter_label,
                      self.localization_label, self.gate_label):
            layout.addWidget(label)

        poses = QGroupBox('Live poses — XYZ [m], roll / pitch / yaw [deg]')
        pose_layout = QVBoxLayout(poses)
        self.pose_table = QTableWidget(3, 8)
        self.pose_table.setHorizontalHeaderLabels(['Source / frames', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw', 'Freshness'])
        self.pose_table.horizontalHeader().setStretchLastSection(True)
        self.pose_table.setColumnWidth(0, 300)
        # Keep all three live sources visible even when the window is compact.
        self.pose_table.setFixedHeight(140)
        self.pose_table.setEditTriggers(QTableWidget.NoEditTriggers)
        pose_layout.addWidget(self.pose_table)
        layout.addWidget(poses)

        middle = QHBoxLayout()
        manual_box = QGroupBox('Mobile base control — hold a direction to move')
        manual = QGridLayout(manual_box)
        controls = [
            ('Prepare power + servo', 'prepare_robot', {}),
            ('Control ON', 'request_control_manager', {'command': 'enable'}),
            ('Stream ON', 'request_stream', {'enabled': True}),
            ('Stream OFF', 'request_stream', {'enabled': False}),
            ('Control OFF', 'request_control_manager', {'command': 'disable'}),
        ]
        for index, (label, operation, arguments) in enumerate(controls):
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, op=operation, args=arguments: self._control(op, args))
            manual.addWidget(button, index // 3, index % 3)
        self.linear_speed = self._speed('max_linear_speed', 0.02)
        self.angular_speed = self._speed('max_angular_speed', 0.08)
        manual.addWidget(QLabel('Linear m/s | angular rad/s'), 2, 0)
        manual.addWidget(self.linear_speed, 2, 1)
        manual.addWidget(self.angular_speed, 2, 2)
        for label, row, column, direction in (
            ('Rotate left', 3, 0, (0, 0, 1)), ('Forward', 3, 1, (1, 0, 0)),
            ('Rotate right', 3, 2, (0, 0, -1)), ('Strafe left', 4, 0, (0, 1, 0)),
            ('Strafe right', 4, 2, (0, -1, 0)), ('Back', 5, 1, (-1, 0, 0)),
        ):
            button = QPushButton(label)
            button.setMinimumHeight(40)
            button.pressed.connect(lambda d=direction: self._start_manual(d))
            button.released.connect(self._stop_manual)
            manual.addWidget(button, row, column)
        stop = QPushButton('STOP / Space')
        stop.setMinimumHeight(40)
        stop.setStyleSheet('background: #b00020; color: white; font-weight: bold;')
        stop.clicked.connect(self._stop)
        manual.addWidget(stop, 4, 1)
        middle.addWidget(manual_box, 3)

        station_box = QGroupBox('Station correction')
        station_layout = QVBoxLayout(station_box)
        description = QLabel('Transit uses wheel odometry. At a station, stop and wait for a stable VO pose before the next task.')
        description.setWordWrap(True)
        station_layout.addWidget(description)
        correct = QPushButton('Stop + correct at this station')
        correct.clicked.connect(self._correct)
        station_layout.addWidget(correct)
        self.reset_button = QPushButton('Recover restarted VO reference')
        self.reset_button.clicked.connect(self._reset_visual)
        self.reset_button.setEnabled(False)
        station_layout.addWidget(self.reset_button)
        recovery = QLabel('Recovery anchors restarted VO to the current wheel estimate; it cannot recover wheel drift during the restart.')
        recovery.setWordWrap(True)
        station_layout.addWidget(recovery)
        station_layout.addStretch()
        middle.addWidget(station_box, 2)
        layout.addLayout(middle)

        box = QGroupBox('Stations / waypoints')
        waypoint_layout = QVBoxLayout(box)
        names = QHBoxLayout()
        self.name_edit = QLineEdit()
        names.addWidget(QLabel('Name'))
        names.addWidget(self.name_edit, 1)
        for text, callback in (('Save current localization', self._save_current),
                               ('Delete selected', self._delete_selected), ('Reload', self._load)):
            button = QPushButton(text)
            button.clicked.connect(callback)
            names.addWidget(button)
        waypoint_layout.addLayout(names)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(['Station', 'X [m]', 'Y [m]', 'Yaw [deg]'])
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setMaximumHeight(150)
        waypoint_layout.addWidget(self.table)
        go = QPushButton('Goto selected station → stop → VO correction')
        go.setMinimumHeight(38)
        go.clicked.connect(self._go_selected)
        waypoint_layout.addWidget(go)
        layout.addWidget(box)
        self.log_list = QListWidget()
        self.log_list.setMaximumHeight(95)
        layout.addWidget(self.log_list)
        self.setCentralWidget(root)

    def _speed(self, name, initial):
        spin = QDoubleSpinBox()
        spin.setRange(0.01, float(self.node.p[name]))
        spin.setSingleStep(0.01)
        spin.setValue(min(initial, float(self.node.p[name])))
        return spin

    def log(self, message):
        self.log_list.addItem(str(message))
        while self.log_list.count() > 100:
            self.log_list.takeItem(0)
        self.log_list.scrollToBottom()

    def tick(self):
        self.node.tick()
        if self.manual_velocity is not None:
            if not self.isActiveWindow() or not self.node.set_manual_velocity(*self.manual_velocity):
                self._stop_manual()
                self.node.operation_text = 'Manual drive stopped: control / navigation status unavailable'
        self.operation_label.setText(self.node.operation_text)
        state = self.node.status('control').get('snapshot', {})
        self.control_label.setText(
            f"Control: {'connected' if self.node.fresh('control') else 'STALE / unavailable'} | "
            f"power={state.get('power_enabled', '?')} servo={state.get('servo_enabled', '?')} "
            f"stream={state.get('stream_enabled', '?')}")
        state = self.node.status('tracking')
        tracking_connected = self.node.fresh('tracking')
        tracking_ok = tracking_connected and state.get('ready') is True and state.get('tracking_ok') is True
        self.tracking_label.setText(
            f"VO tracking: {'READY' if tracking_ok else 'BLOCKED'} | "
            f"status {'connected' if tracking_connected else 'STALE / unavailable'} | "
            f"ready={state.get('ready', '?')} tracking_ok={state.get('tracking_ok', '?')} | "
            f"{state.get('detail', 'waiting for tracking status')}"
            + ('; Base VO output is blocked by tracking status' if not tracking_ok else ''))
        adapter_streams = self.node.status('adapter').get('streams', {})
        if not isinstance(adapter_streams, dict):
            adapter_streams = {}
        conversion = []
        for kind, title in (('odom', 'VO'), ('slam_odom', 'SLAM')):
            stream = adapter_streams.get(kind, {})
            if not isinstance(stream, dict):
                stream = {}
            if not self.node.fresh('adapter'):
                conversion.append(title + ': status unavailable')
            elif stream.get('healthy') is True:
                conversion.append(title + ': LIVE')
            else:
                conversion.append(title + ': ' + str(stream.get('detail', 'waiting for base pose')))
        conversion_detail = 'Base conversion: ' + ' | '.join(conversion)
        self.adapter_label.setToolTip(conversion_detail)
        self.adapter_label.setText(conversion_detail if len(conversion_detail) <= 180
                                   else conversion_detail[:177] + '…')
        state = self.node.status('localization')
        if not self.node.fresh('localization'):
            health = 'UNAVAILABLE'
        elif state.get('initialized') is not True:
            health = 'NOT INITIALIZED'
        elif state.get('healthy') is not True:
            health = 'UNHEALTHY'
        elif not self.node.pose_fresh('Localization'):
            health = 'POSE STALE / waiting'
        else:
            health = 'HEALTHY'
        self.localization_label.setText(
            f"Localization: {health} | "
            f"status {'connected' if self.node.fresh('localization') else 'STALE / unavailable'} | "
            f"{state.get('mode', 'waiting')} | "
            f"corrections={state.get('correction_count', 0)} | {state.get('detail', '')}")
        self.reset_button.setEnabled(state.get('visual_reference_changed') is True)
        state = self.node.status('navigation')
        self.gate_label.setText(
            f"Navigation: {'ON' if state.get('enabled') else 'OFF'} | "
            f"{'fresh' if self.node.fresh('navigation') else 'unavailable / STALE'} | "
            f"{state.get('detail', '')}")
        for row, name in enumerate(('Camera VO', 'Base VO', 'Localization')):
            item = self.node.poses.get(name)
            if item is None:
                freshness = 'Waiting'
                if name == 'Localization' and self.node.fresh('localization'):
                    freshness = ('Waiting for current pose'
                                 if self.node.status('localization').get('initialized') is True
                                 else 'NOT INITIALIZED')
                elif name == 'Base VO' and not tracking_ok:
                    freshness = 'TRACKING BLOCKED'
                columns = [name, '—', '—', '—', '—', '—', '—', freshness]
            else:
                msg, values, received = item
                columns = [f'{name}: {msg.header.frame_id} → {msg.child_frame_id}']
                columns += [f'{value:.4f}' for value in values[:3]]
                columns += [f'{math.degrees(value):.2f}' for value in values[3:]]
                columns += [('LIVE' if self.node.pose_fresh(name) else 'STALE') +
                            f' ({time.monotonic() - received:.1f}s)']
            for column, value in enumerate(columns):
                cell = QTableWidgetItem(value)
                cell.setToolTip(value)
                self.pose_table.setItem(row, column, cell)

    def _load(self):
        try:
            self.waypoints = load_waypoints(self.node.waypoint_path, self.node.map_frame)
            self._refresh_table()
            self.log('Waypoints: ' + str(self.node.waypoint_path))
        except (OSError, ValueError, TypeError) as exc:
            self.log('Failed to read waypoints: ' + str(exc))

    def _persist(self, waypoints):
        save_waypoints(self.node.waypoint_path, waypoints, self.node.map_frame)
        self.waypoints = waypoints
        self._refresh_table()

    def _refresh_table(self):
        self.table.setRowCount(len(self.waypoints))
        for row, item in enumerate(self.waypoints):
            for column, value in enumerate((item.name, f'{item.x:.4f}', f'{item.y:.4f}', f'{math.degrees(item.yaw):.2f}')):
                cell = QTableWidgetItem(value)
                cell.setFlags(cell.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(row, column, cell)
        self.name_edit.setText(next_name(self.waypoints))

    def _save_current(self):
        name = self.name_edit.text().strip()
        if not name or any(item.name == name for item in self.waypoints):
            self.log('Save failed: use a nonempty, unique station name.')
            return
        try:
            x, y, yaw = self.node.current_pose()
            self._persist([*self.waypoints, Waypoint(name, x, y, yaw)])
            self.log(f'Saved {name}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°')
        except (OSError, ValueError, TypeError) as exc:
            self.log('Save failed: ' + str(exc))

    def _selected_index(self):
        rows = self.table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def _delete_selected(self):
        index = self._selected_index()
        if index is None:
            self.log('Select a station to delete.')
            return
        try:
            self._persist(self.waypoints[:index] + self.waypoints[index + 1:])
        except (OSError, ValueError, TypeError) as exc:
            self.log('Deletion failed: ' + str(exc))

    def _go_selected(self):
        index = self._selected_index()
        if index is None:
            self.log('Select a station for Goto.')
            return
        self._release_manual()
        self.node.enable_then_navigate(self.waypoints[index])

    def _control(self, operation, arguments):
        self._stop()
        if self.node.fresh('control'):
            self.node.send_control(operation, arguments)
        else:
            self.log('Control backend is unavailable.')

    def _start_manual(self, direction):
        self.manual_held = True
        velocity = (direction[0] * self.linear_speed.value(),
                    direction[1] * self.linear_speed.value(),
                    direction[2] * self.angular_speed.value())
        def ready():
            if self.manual_held and self.isActiveWindow():
                self.manual_velocity = velocity
            else:
                self.node.stop()
        self.node.begin_manual(ready)

    def _release_manual(self):
        self.manual_held = False
        self.manual_velocity = None

    def _stop_manual(self):
        if self.manual_held or self.manual_velocity is not None:
            self._stop()

    def _stop(self):
        self._release_manual()
        self.node.stop()

    def _correct(self):
        self._release_manual()
        self.node.request_station()

    def _reset_visual(self):
        self._release_manual()
        self.node.request_visual_reset()

    def event(self, event):
        if event.type() == QEvent.WindowDeactivate and hasattr(self, 'node'):
            self._stop_manual()
        return super().event(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self._stop()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self.timer.stop()
        self._release_manual()
        self.node.shutdown_future = self.node.stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    node = None
    try:
        node = OperatorNode()
        window = OperatorWindow(node)
        window.show()
        spin_timer = QTimer()
        spin_timer.setInterval(10)
        def spin_ros():
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except KeyboardInterrupt:
                app.quit()
        spin_timer.timeout.connect(spin_ros)
        spin_timer.start()
        return app.exec_()
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            if rclpy.ok():
                future = node.shutdown_future or node.stop()
                started = time.monotonic()
                # Keep the client alive long enough to deliver stop/cancel and
                # receive the gate's acknowledgement when the window closes.
                while rclpy.ok() and time.monotonic() - started < 1.0:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if time.monotonic() - started < 0.1:
                        continue
                    if future is None and not node._gate_present():
                        break
                    if future is not None and future.done() and node._gate_idle():
                        break
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
