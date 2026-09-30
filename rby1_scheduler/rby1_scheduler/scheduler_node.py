"""ROS node that owns runtime scheduling and publishes UI-ready JSON state."""
from __future__ import annotations
import json
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from .data_source import JsonDataSource
from .lab_config import load_lab_config
from .planner_client import MockPlannerClient
from .runtime_engine import RuntimeEngine
from .simulation import ScheduleSimulator

class SchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__('rby1_scheduler')
        share = Path(get_package_share_directory('rby1_scheduler'))
        mock = share / 'mock_server'
        self.declare_parameter('server_input_path', str(mock / 'server_input.json'))
        self.declare_parameter('scheduler_output_path', str(mock / 'scheduler_output.json'))
        self.declare_parameter('scheduler_events_path', str(mock / 'scheduler_events.jsonl'))
        self.declare_parameter('lab_config_path', str(share / 'config' / 'lab.yaml'))
        self.declare_parameter('mock_planner_duration_sec', 4.0)
        #서버 읽기 어댑터 -> HttpDataSource(server_url)로 교체 예정
        self.source = JsonDataSource(
            self.get_parameter('server_input_path').value,
            self.get_parameter('scheduler_output_path').value,
            self.get_parameter('scheduler_events_path').value,
        )
        self.engine = RuntimeEngine(MockPlannerClient(float(self.get_parameter('mock_planner_duration_sec').value)))
        self.lab_config_path = str(self.get_parameter('lab_config_path').value)
        self.lab = load_lab_config(self.lab_config_path)
        self.simulator = ScheduleSimulator(self.lab)
        self.simulation: dict = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, 'scheduler/state', qos)
        self.event_pub = self.create_publisher(String, 'scheduler/events', 50)
        self.command_sub = self.create_subscription(String, 'scheduler/command', self._command, 20)
        self._last_input_check = 0.0
        self._last_output = 0.0
        self._force_reload = True
        self.timer = self.create_timer(0.2, self._tick)
        self.get_logger().info('Scheduler ready with JSON mock server and mock planner.')

    def _tick(self) -> None:
        now = time.monotonic()
        if self._force_reload or now - self._last_input_check >= 1.0:
            self._last_input_check = now
            try:
                raw = self.lab.enrich_snapshot(self.source.read_snapshot())
                self.engine.sync_external(raw, force=self._force_reload)
                self._force_reload = False
            except Exception as exc:
                self.get_logger().error(f'External JSON read failed: {exc}')
        self.engine.tick()
        events = self.engine.drain_events()
        if events:
            self.source.append_events(events)
            for event in events:
                self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
        state = self.engine.snapshot()
        ui_state = {**state, 'simulation': self.simulation, 'environment': self.lab.summary(), 'planner_mode': 'mock'}
        self.state_pub.publish(String(data=json.dumps(ui_state, ensure_ascii=False)))
        if events or now - self._last_output >= 1.0:
            self._last_output = now
            try:
                self.source.write_scheduler_state(state)
            except Exception as exc:
                self.get_logger().error(f'Scheduler JSON write failed: {exc}')

    def _command(self, message: String) -> None:
        try:
            command = json.loads(message.data)
            if command.get('cmd') == 'simulate':
                self.simulation = self.simulator.run(self.source.read_snapshot())
                self.get_logger().info(f"Simulation {self.simulation['status']}: {self.simulation['end_min']:.1f} min")
                return
            if command.get('cmd') == 'reload':
                self.lab = load_lab_config(self.lab_config_path)
                self.simulator = ScheduleSimulator(self.lab)
                self.simulation = {}
                self._force_reload = True
                return
            self.engine.apply_command(command)
        except Exception as exc:
            self.get_logger().warning(f'Scheduler command rejected: {exc}')

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
