"""Qt operator UI for manual mapping drives and named Nav2 poses."""

import json
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .waypoint_store import (
    Waypoint, default_waypoints_path, load_waypoints, next_name, save_waypoints,
)


def _yaw_from_quaternion(rotation):
    sin_yaw = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cos_yaw = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(sin_yaw, cos_yaw)


def _control_message(operation, arguments=None):
    return json.dumps({
        'schema_version': 1,
        'kind': 'command',
        'source': 'rby1_vslam_waypoint_ui',
        'request_id': '',
        'operation': operation,
        'arguments': dict(arguments or {}),
    }, ensure_ascii=False, separators=(',', ':'), allow_nan=False)


class OperatorNode(Node):
    def __init__(self):
        super().__init__('waypoint_ui')
        defaults = {
            'map_frame': 'vslam_map',
            'base_frame': 'base',
            'waypoints_file': str(default_waypoints_path(
                get_package_share_directory('rby1_vslam'))),
            'navigate_action': '/rby1/vslam/nav2/navigate_to_pose',
            'enable_service': '/rby1/vslam/enable',
            'cancel_service': '/rby1/vslam/cancel',
            'control_command_topic': '/rby1/control/command',
            'control_event_topic': '/rby1/control/event',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        waypoint_path = Path(str(self.p['waypoints_file'])).expanduser()
        if not waypoint_path.is_absolute():
            raise ValueError('waypoints_file must be an absolute path')
        self.waypoint_path = waypoint_path
        self.map_frame = str(self.p['map_frame'])
        self.base_frame = str(self.p['base_frame'])
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigate_client = ActionClient(
            self, NavigateToPose, str(self.p['navigate_action']))
        self.enable_client = self.create_client(SetBool, str(self.p['enable_service']))
        self.cancel_client = self.create_client(Trigger, str(self.p['cancel_service']))
        self.control_pub = self.create_publisher(
            String, str(self.p['control_command_topic']), 20)
        self.create_subscription(
            String, str(self.p['control_event_topic']), self._on_control_event, 50)
        self.create_subscription(
            String, '/rby1/vslam/bridge_status', self._on_bridge_status, 10)
        self.create_subscription(
            String, '/rby1/vslam/navigation_status', self._on_navigation_status, 10)
        self.log_sink = None
        self.bridge_sink = None
        self.navigation_sink = None
        self.goal_handle = None
        self.last_feedback_log = 0.0

    def emit_log(self, message):
        self.get_logger().info(str(message))
        if self.log_sink is not None:
            self.log_sink(str(message))

    def _on_control_event(self, message):
        try:
            packet = json.loads(message.data)
            text = str(packet.get('message', message.data))
            level = str(packet.get('level', 'info')).upper()
            self.emit_log(f'CONTROL {level}: {text}')
        except (TypeError, ValueError):
            self.emit_log('CONTROL: ' + message.data)

    def _on_bridge_status(self, message):
        if self.bridge_sink is None:
            return
        try:
            state = json.loads(message.data)
            connected = state.get('connected') is True
            tracking = state.get('tracking_ok') is True
            self.bridge_sink(
                f"VSLAM: {'connected' if connected else 'disconnected'} / "
                f"tracking {'OK' if tracking else 'waiting'}"
            )
        except (TypeError, ValueError, AttributeError):
            self.bridge_sink('VSLAM: status decode failed')

    def _on_navigation_status(self, message):
        if self.navigation_sink is None:
            return
        try:
            state = json.loads(message.data)
            enabled = state.get('enabled') is True
            detail = str(state.get('detail', ''))
            self.navigation_sink(
                f"Nav gate: {'ON' if enabled else 'OFF'} - {detail}"
            )
        except (TypeError, ValueError, AttributeError):
            self.navigation_sink('Nav gate: status decode failed')

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.2),
        ).transform
        return (
            float(transform.translation.x),
            float(transform.translation.y),
            _yaw_from_quaternion(transform.rotation),
        )

    def send_control(self, operation, arguments=None, announce=True):
        self.control_pub.publish(String(data=_control_message(operation, arguments)))
        if announce:
            self.emit_log(f'Control request: {operation}')

    def set_manual_velocity(self, vx, vy, wz):
        self.send_control(
            'set_velocity', {'vx': vx, 'vy': vy, 'wz': wz}, announce=False)

    def stop_manual(self):
        self.send_control('stop', {'publish_immediately': True}, announce=False)

    def disable_and_cancel_navigation(self):
        if self.enable_client.service_is_ready():
            self.enable_client.call_async(SetBool.Request(data=False))
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(Trigger.Request())

    def enable_then_navigate(self, waypoint):
        if not self.enable_client.service_is_ready():
            self.emit_log('Navigation failed: Nav gate enable service is not ready.')
            return
        request = SetBool.Request(data=True)
        future = self.enable_client.call_async(request)
        future.add_done_callback(
            lambda completed, target=waypoint: self._after_enable(completed, target))

    def _after_enable(self, future, waypoint):
        try:
            response = future.result()
        except Exception as exc:
            self.emit_log(f'Navigation failed: gate response error: {exc}')
            return
        if response is None or not response.success:
            detail = getattr(response, 'message', 'no response')
            self.emit_log(f'Navigation deferred: {detail}')
            return
        if not self.navigate_client.server_is_ready():
            self.emit_log('Navigation failed: Nav2 NavigateToPose action is not ready.')
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        goal.pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        self.emit_log(
            f'Navigate to {waypoint.name}: x={waypoint.x:.3f}, '
            f'y={waypoint.y:.3f}, yaw={math.degrees(waypoint.yaw):.1f} deg')
        future = self.navigate_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.emit_log(f'Failed to send Nav2 goal: {exc}')
            return
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.emit_log('Nav2 rejected the goal.')
            self.goal_handle = None
            return
        self.emit_log('Nav2 accepted the goal.')
        result = self.goal_handle.get_result_async()
        result.add_done_callback(self._on_goal_result)

    def _on_feedback(self, message):
        now = time.monotonic()
        if now - self.last_feedback_log < 0.5:
            return
        self.last_feedback_log = now
        feedback = message.feedback
        remaining = getattr(feedback, 'distance_remaining', float('nan'))
        if math.isfinite(float(remaining)):
            self.emit_log(f'Distance remaining: {float(remaining):.2f} m')

    def _on_goal_result(self, future):
        try:
            wrapped = future.result()
            self.emit_log(f'Nav2 goal finished (status={wrapped.status}).')
        except Exception as exc:
            self.emit_log(f'Nav2 result error: {exc}')
        self.goal_handle = None

    def emergency_stop(self):
        self.stop_manual()
        if not self.cancel_client.service_is_ready():
            self.emit_log('Stop request: cancel service is not ready.')
            return
        future = self.cancel_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_cancel_response)

    def _on_cancel_response(self, future):
        try:
            response = future.result()
            self.emit_log('Stop request: ' + str(response.message))
        except Exception as exc:
            self.emit_log(f'Stop service error: {exc}')


class OperatorWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.waypoints = []
        self.manual_velocity = None
        self.setWindowTitle('RBY1 VSLAM Mapping / Waypoints')
        self.resize(820, 720)
        self._build()
        self.node.log_sink = self.log
        self.node.bridge_sink = self.bridge_label.setText
        self.node.navigation_sink = self.gate_label.setText
        self._load()
        self.manual_timer = QTimer(self)
        self.manual_timer.setInterval(100)
        self.manual_timer.timeout.connect(self._publish_manual)

    def _build(self):
        root = QWidget(self)
        layout = QVBoxLayout(root)
        warning = QLabel(
            'PHYSICAL ROBOT / NO LIDAR: no obstacle detection or avoidance. '
            'Keep the hardware EMO within reach; the red UI button is software-only.')
        warning.setStyleSheet('color: #b00020; font-weight: bold; padding: 6px;')
        warning.setWordWrap(True)
        layout.addWidget(warning)
        self.bridge_label = QLabel('VSLAM: waiting')
        self.gate_label = QLabel('Nav gate: waiting')
        layout.addWidget(self.bridge_label)
        layout.addWidget(self.gate_label)

        prepare_box = QGroupBox('1. Physical Robot Control Setup')
        prepare_layout = QHBoxLayout(prepare_box)
        prepare = QPushButton('Prepare Power + Servo')
        control = QPushButton('Control Manager ON')
        stream = QPushButton('Stream ON')
        stream_off = QPushButton('Stream OFF')
        prepare.clicked.connect(lambda: self.node.send_control('prepare_robot'))
        control.clicked.connect(lambda: self.node.send_control(
            'request_control_manager', {'command': 'enable'}))
        stream.clicked.connect(lambda: self.node.send_control(
            'request_stream', {'enabled': True, 'value': 0.0}))
        stream_off.clicked.connect(lambda: self.node.send_control(
            'request_stream', {'enabled': False, 'value': 0.0}))
        for button in (prepare, control, stream, stream_off):
            prepare_layout.addWidget(button)
        layout.addWidget(prepare_box)

        manual_box = QGroupBox('2. Manual Mapping Drive (hold button to move)')
        manual_layout = QGridLayout(manual_box)
        self.linear_speed = QDoubleSpinBox()
        self.linear_speed.setRange(0.01, 0.08)
        self.linear_speed.setSingleStep(0.01)
        self.linear_speed.setValue(0.02)
        self.angular_speed = QDoubleSpinBox()
        self.angular_speed.setRange(0.03, 0.20)
        self.angular_speed.setSingleStep(0.01)
        self.angular_speed.setValue(0.08)
        manual_layout.addWidget(QLabel('Linear m/s'), 0, 0)
        manual_layout.addWidget(self.linear_speed, 0, 1)
        manual_layout.addWidget(QLabel('Angular rad/s'), 0, 2)
        manual_layout.addWidget(self.angular_speed, 0, 3)
        controls = [
            ('ROTATE LEFT', 1, 0, lambda: (0.0, 0.0, self.angular_speed.value())),
            ('FORWARD', 1, 1, lambda: (self.linear_speed.value(), 0.0, 0.0)),
            ('ROTATE RIGHT', 1, 2, lambda: (0.0, 0.0, -self.angular_speed.value())),
            ('STRAFE LEFT', 2, 0, lambda: (0.0, self.linear_speed.value(), 0.0)),
            ('STOP', 2, 1, lambda: (0.0, 0.0, 0.0)),
            ('STRAFE RIGHT', 2, 2, lambda: (0.0, -self.linear_speed.value(), 0.0)),
            ('BACK', 3, 1, lambda: (-self.linear_speed.value(), 0.0, 0.0)),
        ]
        for text, row, column, velocity in controls:
            button = QPushButton(text)
            button.setMinimumHeight(42)
            if text == 'STOP':
                button.clicked.connect(self._stop_manual)
            else:
                button.pressed.connect(
                    lambda selected=velocity: self._start_manual(selected()))
                button.released.connect(self._stop_manual)
            manual_layout.addWidget(button, row, column)
        layout.addWidget(manual_box)

        waypoint_box = QGroupBox('3. Save Current Pose / Navigate')
        waypoint_layout = QVBoxLayout(waypoint_box)
        name_row = QHBoxLayout()
        name_row.addWidget(QLabel('Name'))
        self.name_edit = QLineEdit()
        save_button = QPushButton('Save Current Pose')
        delete_button = QPushButton('Delete Selected')
        reload_button = QPushButton('Reload')
        name_row.addWidget(self.name_edit, 1)
        for button in (save_button, delete_button, reload_button):
            name_row.addWidget(button)
        waypoint_layout.addLayout(name_row)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(['Name', 'X [m]', 'Y [m]', 'Yaw [deg]'])
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.horizontalHeader().setStretchLastSection(True)
        waypoint_layout.addWidget(self.table)
        action_row = QHBoxLayout()
        go = QPushButton('Navigate to Selected Point')
        stop = QPushButton('SOFTWARE STOP + Cancel Nav2 Goal')
        go.setMinimumHeight(44)
        stop.setMinimumHeight(44)
        stop.setStyleSheet('background: #b00020; color: white; font-weight: bold;')
        action_row.addWidget(go)
        action_row.addWidget(stop)
        waypoint_layout.addLayout(action_row)
        save_button.clicked.connect(self._save_current)
        delete_button.clicked.connect(self._delete_selected)
        reload_button.clicked.connect(self._load)
        go.clicked.connect(self._go_selected)
        stop.clicked.connect(self._emergency_stop)
        layout.addWidget(waypoint_box)

        log_box = QGroupBox('Status / Actual Velocity Commands')
        log_layout = QVBoxLayout(log_box)
        self.log_list = QListWidget()
        self.log_list.setMaximumHeight(145)
        log_layout.addWidget(self.log_list)
        layout.addWidget(log_box)
        self.setCentralWidget(root)

    def log(self, message):
        self.log_list.addItem(str(message))
        while self.log_list.count() > 100:
            self.log_list.takeItem(0)
        self.log_list.scrollToBottom()

    def _load(self):
        try:
            self.waypoints = load_waypoints(self.node.waypoint_path, self.node.map_frame)
            self._refresh_table()
            self.log(f'Waypoint file: {self.node.waypoint_path}')
        except (OSError, ValueError, TypeError) as exc:
            self.log(f'Failed to read waypoint file: {exc}')

    def _persist(self):
        save_waypoints(self.node.waypoint_path, self.waypoints, self.node.map_frame)
        self._refresh_table()

    def _refresh_table(self):
        self.table.setRowCount(len(self.waypoints))
        for row, item in enumerate(self.waypoints):
            values = (item.name, f'{item.x:.4f}', f'{item.y:.4f}',
                      f'{math.degrees(item.yaw):.2f}')
            for column, value in enumerate(values):
                cell = QTableWidgetItem(value)
                cell.setFlags(cell.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(row, column, cell)
        self.name_edit.setText(next_name(self.waypoints))

    def _save_current(self):
        name = self.name_edit.text().strip()
        if not name:
            self.log('Save failed: waypoint name is empty.')
            return
        if any(item.name == name for item in self.waypoints):
            self.log(f'Save failed: waypoint {name!r} already exists.')
            return
        try:
            x, y, yaw = self.node.current_pose()
            self.waypoints.append(Waypoint(name, x, y, yaw))
            self._persist()
            self.log(f'Saved {name}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg')
        except (TransformException, OSError, ValueError, TypeError) as exc:
            self.log(f'Failed to save current pose: {exc}')

    def _selected_index(self):
        rows = self.table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def _delete_selected(self):
        index = self._selected_index()
        if index is None:
            self.log('Select a waypoint to delete.')
            return
        item = self.waypoints.pop(index)
        try:
            self._persist()
            self.log(f'{item.name} deleted.')
        except (OSError, ValueError) as exc:
            self.waypoints.insert(index, item)
            self.log(f'Failed to save deletion: {exc}')

    def _go_selected(self):
        index = self._selected_index()
        if index is None:
            self.log('Select a waypoint to navigate to.')
            return
        self._stop_manual()
        self.node.enable_then_navigate(self.waypoints[index])

    def _start_manual(self, velocity):
        self.node.disable_and_cancel_navigation()
        self.manual_velocity = tuple(float(value) for value in velocity)
        self._publish_manual()
        self.manual_timer.start()

    def _publish_manual(self):
        if self.manual_velocity is not None:
            self.node.set_manual_velocity(*self.manual_velocity)

    def _stop_manual(self):
        self.manual_timer.stop()
        self.manual_velocity = None
        self.node.stop_manual()

    def _emergency_stop(self):
        self.manual_timer.stop()
        self.manual_velocity = None
        self.node.emergency_stop()

    def closeEvent(self, event):
        self.manual_timer.stop()
        self.node.stop_manual()
        self.node.disable_and_cancel_navigation()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = None
    app = QApplication(sys.argv[:1])
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
                node.stop_manual()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
