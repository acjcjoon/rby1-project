"""Local mobile-only operator using the existing safe control topic protocol."""
import math
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtCore import QTimer, QEvent, Qt
from PyQt5.QtWidgets import QApplication, QDoubleSpinBox, QGridLayout, QLabel, QPushButton, QWidget
from rby1_control.topic_protocol import decode_message, encode_message


class MobileOperator(Node):
    def __init__(self):
        super().__init__('mobile_base_qt')
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 0.4)
        self.publisher = self.create_publisher(String, 'control/command', 10)
        self.last_state = 0.0
        self.status = 'Waiting for control backend'
        self.create_subscription(String, 'control/state', self.on_state, 10)
        self.create_subscription(String, 'control/event', self.on_event, 20)

    def on_state(self, message):
        try:
            decode_message(message.data, expected_kind='state')
        except ValueError:
            return
        self.last_state = time.monotonic()

    def on_event(self, message):
        try:
            self.status = str(decode_message(message.data).get('message', message.data))
        except ValueError:
            pass

    def fresh(self):
        return time.monotonic() - self.last_state < 1.0

    def send(self, operation, arguments=None):
        self.publisher.publish(String(data=encode_message('command', {
            'source': 'mobile_base_qt', 'request_id': uuid.uuid4().hex,
            'operation': operation, 'arguments': arguments or {},
        })))


class MobileWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.velocity = None
        self.setWindowTitle('RBY1 Mobile Base / UPC')
        layout = QGridLayout(self)
        layout.addWidget(QLabel('Hold a direction to move. STOP is a software stop.'), 0, 0, 1, 3)
        setup = [
            ('Power ON', 'request_power', {'enabled': True, 'target': 'all'}),
            ('Servo ON', 'request_servo', {'enabled': True, 'target': 'all'}),
            ('Control ON', 'request_control_manager', {'command': 'enable'}),
            ('Stream ON', 'request_stream', {'enabled': True}),
            ('Stream OFF', 'request_stream', {'enabled': False}),
            ('Control OFF', 'request_control_manager', {'command': 'disable'}),
        ]
        for index, (label, operation, arguments) in enumerate(setup):
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, op=operation, args=arguments: self.action(op, args))
            layout.addWidget(button, 1 + index // 3, index % 3)
        self.linear = self.speed('max_linear_speed', 0.02)
        self.angular = self.speed('max_angular_speed', 0.08)
        layout.addWidget(QLabel('Linear m/s / Angular rad/s'), 3, 0)
        layout.addWidget(self.linear, 3, 1)
        layout.addWidget(self.angular, 3, 2)
        directions = [
            ('Rotate left', 4, 0, (0, 0, 1)), ('Forward', 4, 1, (1, 0, 0)),
            ('Rotate right', 4, 2, (0, 0, -1)),
            ('Strafe left', 5, 0, (0, 1, 0)), ('Strafe right', 5, 2, (0, -1, 0)),
            ('Back', 6, 1, (-1, 0, 0)),
        ]
        for label, row, column, direction in directions:
            button = QPushButton(label)
            button.setMinimumHeight(45)
            button.pressed.connect(lambda d=direction: self.start(d))
            button.released.connect(self.stop)
            layout.addWidget(button, row, column)
        stop = QPushButton('STOP')
        stop.setStyleSheet('background: #b00020; color: white')
        stop.clicked.connect(self.stop)
        layout.addWidget(stop, 5, 1)
        self.status = QLabel(node.status)
        self.status.setWordWrap(True)
        layout.addWidget(self.status, 7, 0, 1, 3)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def speed(self, name, initial):
        limit = float(self.node.get_parameter(name).value)
        if not math.isfinite(limit) or limit <= 0:
            raise ValueError(f'{name} must be positive and finite')
        widget = QDoubleSpinBox()
        widget.setRange(0, limit)
        widget.setSingleStep(0.01)
        widget.setValue(min(initial, limit))
        return widget

    def action(self, operation, arguments):
        self.stop()
        if self.node.fresh():
            self.node.send(operation, arguments)

    def start(self, direction):
        if self.node.fresh() and self.isActiveWindow():
            self.velocity = dict(zip(('vx', 'vy', 'wz'), (
                direction[0] * self.linear.value(), direction[1] * self.linear.value(),
                direction[2] * self.angular.value(),
            )))

    def stop(self):
        self.velocity = None
        if rclpy.ok():
            self.node.send('stop')

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        connected = self.node.fresh()
        self.status.setText(('Connected: ' if connected else 'Disconnected: ') + self.node.status)
        if self.velocity is not None:
            if connected and self.isActiveWindow():
                self.node.send('set_velocity', self.velocity)
            else:
                self.stop()

    def event(self, event):
        if event.type() == QEvent.WindowDeactivate and hasattr(self, 'node'):
            self.stop()
        return super().event(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self.stop()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self.stop()
        self.timer.stop()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    node = MobileOperator()
    try:
        window = MobileWindow(node)
        window.show()
        app.exec_()
    finally:
        if rclpy.ok():
            node.send('stop')
            rclpy.spin_once(node, timeout_sec=0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
