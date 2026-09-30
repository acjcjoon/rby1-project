"""ROS transport used only by the scheduler Qt process."""
from __future__ import annotations
import json
import threading
from typing import Any
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

class SchedulerUiNode(Node):
    def __init__(self) -> None:
        super().__init__('rby1_scheduler_ui')
        self._lock = threading.Lock()
        self._state: dict[str, Any] | None = None
        self._state_version = 0
        self._seen_version = -1
        self._events: list[dict[str, Any]] = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'scheduler/state', self._state_callback, qos)
        self.create_subscription(String, 'scheduler/events', self._event_callback, 50)
        self.command_pub = self.create_publisher(String, 'scheduler/command', 20)

    def send_command(self, command: dict[str, Any]) -> None:
        self.command_pub.publish(String(data=json.dumps(command, ensure_ascii=False)))

    def consume_state(self) -> dict[str, Any] | None:
        with self._lock:
            if self._state_version == self._seen_version:
                return None
            self._seen_version = self._state_version
            return self._state

    def consume_events(self) -> list[dict[str, Any]]:
        with self._lock:
            events, self._events = self._events, []
            return events

    def _state_callback(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except (TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Invalid scheduler state: {exc}')
            return
        with self._lock:
            self._state = state
            self._state_version += 1

    def _event_callback(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        with self._lock:
            self._events.append(event)
            del self._events[:-100]
