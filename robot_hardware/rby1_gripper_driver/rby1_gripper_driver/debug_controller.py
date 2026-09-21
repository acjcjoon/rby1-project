"""Interactive terminal publisher for lightweight gripper verification."""

from __future__ import annotations

import math
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from .debug_commands import DebugInstruction, parse_debug_command


HELP = """\
Commands (close ratio: 0.0=open, 1.0=closed):
  home                         home both grippers
  set <right> <left>           command both grippers explicitly
  right <ratio>                command only the right gripper
  left <ratio>                 command only the left gripper
  open [right|left|both]       open one or both grippers
  close [right|left|both]      close one or both grippers
  torque <on|off>              Dynamixel torque control
  state                        print the latest normalized state
  help                         show this help
  quit                         exit this debug controller
"""


class GripperDebugController(Node):
    """Publish commands and call driver services from a terminal."""

    def __init__(self) -> None:
        super().__init__('gripper_debug_controller')
        self._command_pub = self.create_publisher(
            Float64MultiArray, 'gripper/command', 10
        )
        self._state_sub = self.create_subscription(
            Float64MultiArray, 'gripper/state', self._on_state, 10
        )
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._ready_sub = self.create_subscription(
            Bool, 'gripper/ready', self._on_ready, ready_qos
        )
        self._home_client = self.create_client(Trigger, 'gripper/home')
        self._torque_client = self.create_client(
            SetBool, 'gripper/torque_enable'
        )
        self._lock = threading.Lock()
        self._latest_state: Optional[Tuple[float, float]] = None
        self._last_command: Optional[Tuple[float, float]] = None
        self._ready: Optional[bool] = None

    def _on_state(self, message: Float64MultiArray) -> None:
        if len(message.data) != 2:
            return
        values = (float(message.data[0]), float(message.data[1]))
        if not all(math.isfinite(value) for value in values):
            return
        with self._lock:
            self._latest_state = values

    def _on_ready(self, message: Bool) -> None:
        with self._lock:
            changed = self._ready is None or bool(message.data) != self._ready
            self._ready = bool(message.data)
        if changed:
            self.get_logger().info(f'driver ready={self._ready}')

    def execute(self, instruction: DebugInstruction) -> bool:
        """Execute one parsed instruction; return false when quitting."""

        if instruction.action == 'noop':
            return True
        if instruction.action == 'help':
            print(HELP, flush=True)
            return True
        if instruction.action == 'quit':
            return False
        if instruction.action == 'state':
            self._print_state()
            return True
        if instruction.action == 'home':
            self._call_home()
            return True
        if instruction.action == 'torque':
            assert instruction.enabled is not None
            self._call_torque(instruction.enabled)
            return True
        if instruction.action == 'set':
            self._publish_command(
                (instruction.values[0], instruction.values[1])
            )
            return True
        if instruction.action in ('set_side', 'preset'):
            self._execute_partial(instruction)
            return True
        raise ValueError(f'unsupported instruction: {instruction.action}')

    def _execute_partial(self, instruction: DebugInstruction) -> None:
        ratio = instruction.values[0]
        if instruction.side == 'both':
            self._publish_command((ratio, ratio))
            return

        with self._lock:
            base = self._latest_state or self._last_command
        if base is None:
            raise ValueError(
                'no state received yet; wait for gripper/state before '
                'sending a single-side command'
            )
        command = list(base)
        command[0 if instruction.side == 'right' else 1] = ratio
        self._publish_command((command[0], command[1]))

    def _publish_command(self, values: Tuple[float, float]) -> None:
        with self._lock:
            ready = self._ready
        if ready is None:
            raise ValueError(
                'no ready message received; check the /rby1 namespace and '
                'whether gripper_driver is running'
            )
        if ready is False:
            raise ValueError(
                'driver is not ready; inspect diagnostics and run home when '
                'calibration is required'
            )
        message = Float64MultiArray()
        message.data = [values[0], values[1]]
        self._command_pub.publish(message)
        with self._lock:
            self._last_command = values
        self.get_logger().info(
            'published close ratios '
            f'right={values[0]}, left={values[1]}'
        )

    def _call_home(self) -> None:
        if not self._home_client.service_is_ready():
            if not self._home_client.wait_for_service(timeout_sec=1.0):
                raise ValueError('gripper/home service is unavailable')
        future = self._home_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done: self._report_service_result('home', done)
        )
        self.get_logger().warning(
            'dual-gripper homing requested; keep both full strokes clear'
        )

    def _call_torque(self, enabled: bool) -> None:
        if not self._torque_client.service_is_ready():
            if not self._torque_client.wait_for_service(timeout_sec=1.0):
                raise ValueError(
                    'gripper/torque_enable service is unavailable'
                )
        request = SetBool.Request()
        request.data = enabled
        future = self._torque_client.call_async(request)
        future.add_done_callback(
            lambda done: self._report_service_result('torque', done)
        )

    def _report_service_result(self, name, future) -> None:
        try:
            response = future.result()
            log = self.get_logger().info if response.success else (
                self.get_logger().error
            )
            log(f'{name}: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'{name} service failed: {exc}')

    def _print_state(self) -> None:
        with self._lock:
            state = self._latest_state
            ready = self._ready
        ready_text = (
            'unknown (no ready message received)'
            if ready is None
            else str(ready)
        )
        if state is None:
            self.get_logger().info(
                f'driver ready={ready_text}; state unavailable'
            )
        else:
            self.get_logger().info(
                f'driver ready={ready_text}; measured close ratios '
                f'right={state[0]}, left={state[1]}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperDebugController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print(HELP, flush=True)
    try:
        while rclpy.ok():
            try:
                line = input('gripper> ')
            except EOFError:
                break
            try:
                if not node.execute(parse_debug_command(line)):
                    break
            except (ValueError, TypeError) as exc:
                node.get_logger().error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
