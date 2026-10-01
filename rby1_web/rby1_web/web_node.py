"""ROS adapter for the mobile-base HTTP operator."""
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rby1_control.topic_protocol import decode_message, encode_message

from .web_server import CommandMailbox, create_server


class MobileBaseWeb(Node):
    def __init__(self):
        super().__init__('mobile_base_web')
        for key, default in (('host', '0.0.0.0'), ('port', 8080),
                             ('max_linear_speed', 0.2), ('max_angular_speed', 0.4),
                             ('operator_mode', 'mobile'),
                             ('control_command_topic', 'control/command'),
                             ('control_state_topic', 'control/state'),
                             ('control_event_topic', 'control/event'),
                             ('control_response_topic', 'control/response')):
            self.declare_parameter(key, default)
        mode = self.get_parameter('operator_mode').value
        if mode not in ('mobile', 'vslam'):
            raise ValueError('operator_mode must be mobile or vslam')
        self.mailbox = CommandMailbox(
            self.get_parameter('max_linear_speed').value,
            self.get_parameter('max_angular_speed').value,
            operator_enabled=mode == 'vslam',
        )
        topic = lambda key: self.get_parameter(key).value
        self.publisher = self.create_publisher(String, topic('control_command_topic'), 10)
        self.create_subscription(String, topic('control_state_topic'), self.state_callback, 10)
        self.create_subscription(String, topic('control_event_topic'), self.event_callback, 20)
        self.create_subscription(String, topic('control_response_topic'), self.response_callback, 20)
        self.operator = None
        if mode == 'vslam':
            from .vslam_adapter import VslamAdapter
            self.operator = VslamAdapter(self)
        self.command_timer = self.create_timer(0.04, self.publish_commands)
        self.server = create_server((self.get_parameter('host').value,
                                     self.get_parameter('port').value), self.mailbox)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'Mobile-base web UI: http://<UPC-IP>:{self.server.server_port} '
            '(trusted local network only; no automatic power/servo/stream startup)')

    def state_callback(self, message):
        try:
            packet = decode_message(message.data, expected_kind='state')
            self.mailbox.update_state(packet['snapshot'])
        except (ValueError, KeyError):
            self.get_logger().warning('Invalid backend state packet')

    def event_callback(self, message):
        try:
            self.mailbox.add_event(decode_message(message.data, expected_kind='event'))
        except ValueError:
            pass

    def response_callback(self, message):
        try:
            packet = decode_message(message.data, expected_kind='response')
            if packet.get('source') == 'mobile_base_web' and not packet.get('accepted'):
                self.mailbox.add_event({'level': 'error', 'message': packet.get('message')})
        except ValueError:
            pass

    def publish_command(self, operation, arguments):
        message = String()
        message.data = encode_message('command', {
            'source': 'mobile_base_web', 'request_id': uuid.uuid4().hex,
            'operation': operation, 'arguments': arguments,
        })
        self.publisher.publish(message)

    def publish_commands(self):
        for operation, arguments in self.mailbox.drain():
            if operation in ('waypoint_save', 'waypoint_delete', 'waypoint_reload',
                             'navigate', 'nav_cancel') and self.operator is not None:
                self.operator.dispatch(operation, arguments)
            elif operation == 'set_velocity' and self.operator is not None:
                if self.operator.manual_ready():
                    self.publish_command(operation, arguments)
                else:
                    self.publish_command('stop', {})
            else:
                self.publish_command(operation, arguments)

    def close(self):
        self.command_timer.cancel()
        if self.operator is not None and rclpy.ok():
            self.operator.cancel()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        if rclpy.ok():
            self.publish_command('stop', {})
            self.publish_command('request_stream', {'enabled': False})
            deadline = time.monotonic() + 0.2
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MobileBaseWeb()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
