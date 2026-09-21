"""ROS 2 driver for real or Isaac-simulated RBY1 grippers."""

from __future__ import annotations

import math
import time
from typing import Optional, Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from .core import Calibration, GripperDriver, GripperError, GripperState
from .sdk_bus import (
    closed_at_minimum_for_transport,
    create_bus,
    motor_ids_for_transport,
)


def _two_values(values: Sequence, label: str):
    if len(values) != 2:
        raise ValueError(f'{label} must contain exactly 2 values')
    return values


class GripperDriverNode(Node):
    """Expose a selected gripper transport through one normalized interface."""

    def __init__(self) -> None:
        super().__init__('gripper_driver')

        # Transport selection. ``real`` remains the default so existing launch
        # commands preserve their behavior.
        self.declare_parameter('transport', 'real')
        self.declare_parameter('sim_host', '127.0.0.1')
        self.declare_parameter('sim_command_port', 5007)
        self.declare_parameter('sim_state_port', 5008)
        self.declare_parameter('sim_feedback_timeout_sec', 1.0)

        # Physical RBY1 Dynamixel gripper settings (ignored by sim as noted).
        self.declare_parameter('device_name', '')
        self.declare_parameter('baud_rate', 2_000_000)
        # The physical robot uses ID 0=right and ID 1=left. Isaac's UDP
        # bridge uses the opposite ID convention and is mapped separately
        # after transport selection below.
        self.declare_parameter('right_motor_id', 0)
        self.declare_parameter('left_motor_id', 1)
        self.declare_parameter('torque_constants', [1.0, 1.0])
        self.declare_parameter('position_torque_limit', 5.0)
        self.declare_parameter('closed_at_minimum', [False, False])
        self.declare_parameter('endpoint_margin_ratio', 0.0)
        self.declare_parameter('minimum_calibration_span_rad', 0.01)

        self.declare_parameter('use_saved_calibration', False)
        self.declare_parameter('calibration_min_rad', [0.0, 0.0])
        self.declare_parameter('calibration_max_rad', [0.0, 0.0])
        self.declare_parameter('auto_home', True)
        self.declare_parameter('auto_enable_torque', True)
        self.declare_parameter('disable_torque_on_shutdown', True)

        self.declare_parameter('homing_torque', 0.3)
        self.declare_parameter('homing_sample_period_sec', 0.05)
        self.declare_parameter('homing_stall_threshold_rad', 0.000001)
        self.declare_parameter('homing_stall_samples', 30)
        self.declare_parameter('homing_direction_timeout_sec', 15.0)
        self.declare_parameter('homing_max_read_failures', 3)

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('state_rate_hz', 10.0)

        configured_real_motor_ids = (
            int(self.get_parameter('right_motor_id').value),
            int(self.get_parameter('left_motor_id').value),
        )
        configured_real_closed_at_minimum = tuple(
            bool(value)
            for value in _two_values(
                self.get_parameter('closed_at_minimum').value,
                'closed_at_minimum',
            )
        )
        self._endpoint_margin_ratio = float(
            self.get_parameter('endpoint_margin_ratio').value
        )
        self._minimum_calibration_span_rad = float(
            self.get_parameter('minimum_calibration_span_rad').value
        )
        self._disable_torque_on_shutdown = bool(
            self.get_parameter('disable_torque_on_shutdown').value
        )
        self._last_warning_at = {}
        self._last_ready: Optional[bool] = None
        self._last_state: Optional[GripperState] = None
        self._closed = False
        self._baud_rate = int(self.get_parameter('baud_rate').value)
        state_rate = float(self.get_parameter('state_rate_hz').value)
        if not math.isfinite(state_rate) or state_rate <= 0.0:
            raise ValueError('state_rate_hz must be positive')
        control_rate = float(self.get_parameter('control_rate_hz').value)
        if not math.isfinite(control_rate) or control_rate <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        torque_constants = _two_values(
            self.get_parameter('torque_constants').value,
            'torque_constants',
        )
        selection = create_bus(
            str(self.get_parameter('transport').value),
            device_name=str(self.get_parameter('device_name').value),
            sim_host=str(self.get_parameter('sim_host').value),
            sim_command_port=int(
                self.get_parameter('sim_command_port').value
            ),
            sim_state_port=int(self.get_parameter('sim_state_port').value),
            sim_feedback_timeout_sec=float(
                self.get_parameter('sim_feedback_timeout_sec').value
            ),
            warning=self.get_logger().warning,
        )
        self._transport = selection.transport
        # Public/core order is always [right, left]. Only the transport's
        # numeric IDs differ: physical RBY1=(0, 1), Isaac UDP=(1, 0).
        self._motor_ids = motor_ids_for_transport(
            self._transport,
            configured_real_motor_ids,
        )
        self._closed_at_minimum = closed_at_minimum_for_transport(
            self._transport,
            configured_real_closed_at_minimum,
        )
        self._hardware_id = selection.hardware_id
        self._driver_name = selection.driver_name
        self._version_key = selection.version_key
        self._driver_version = selection.version
        self._bus = selection.bus
        self._driver = GripperDriver(
            self._bus,
            motor_ids=self._motor_ids,
            torque_constants=torque_constants,
            baud_rate=self._baud_rate,
            position_torque_limit=float(
                self.get_parameter('position_torque_limit').value
            ),
        )
        startup_error: Optional[Exception] = None
        try:
            self._driver.initialize()
            self._configure_startup()
        except Exception as exc:
            # Tool-flange power is commonly still OFF when the UI stack is
            # launched.  Keep the ROS interface alive so the operator can
            # apply the required 12 V and explicitly retry initialization by
            # calling gripper/home.
            startup_error = exc
            self._driver.shutdown(False)

        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._state_pub = self.create_publisher(
            Float64MultiArray, 'gripper/state', 10
        )
        self._motor_state_pub = self.create_publisher(
            JointState, 'gripper/motor_state', 10
        )
        self._ready_pub = self.create_publisher(
            Bool, 'gripper/ready', ready_qos
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, 'gripper/diagnostics', 10
        )
        self._command_sub = self.create_subscription(
            Float64MultiArray,
            'gripper/command',
            self._on_command,
            10,
        )
        self._home_service = self.create_service(
            Trigger, 'gripper/home', self._on_home
        )
        self._torque_service = self.create_service(
            SetBool, 'gripper/torque_enable', self._on_torque_enable
        )

        self._state_timer = self.create_timer(
            1.0 / state_rate, self._publish_state
        )

        self._control_timer = self.create_timer(
            1.0 / control_rate, self._refresh_target
        )

        self._publish_ready(force=True)
        self._publish_diagnostics()
        if startup_error is None:
            self.get_logger().info(
                'Gripper driver initialized '
                f'(transport={self._transport}, endpoint={self._hardware_id}, '
                f'baud={self._baud_rate}, active IDs='
                f'{self._driver.active_motor_ids}, '
                f'driver={self._driver_name}, '
                f'version={self._driver_version}). '
                'Command order is [right, left], where '
                '0.0=open and 1.0=closed.'
            )
        else:
            self.get_logger().warning(
                'Gripper hardware was unavailable during startup; the node '
                'will remain alive and NOT READY. Apply 12 V tool-flange '
                'power, clear both grippers, then call gripper/home to retry '
                f'initialization: {startup_error}'
            )
        if self._driver.calibration is None:
            self.get_logger().warning(
                'Gripper is not calibrated. Run the gripper/home service '
                'with a clear workspace, or configure saved calibration, '
                'before sending commands.'
            )

    def _configure_startup(self) -> None:
        use_saved_calibration = bool(
            self.get_parameter('use_saved_calibration').value
        )
        auto_home_requested = bool(self.get_parameter('auto_home').value)
        auto_home = auto_home_requested
        auto_enable_torque = bool(
            self.get_parameter('auto_enable_torque').value
        )
        if use_saved_calibration:
            calibration = self._calibration_from_parameters()
            self._driver.load_calibration(calibration)
            self.get_logger().info(
                'Loaded saved gripper calibration: '
                f'min={list(calibration.minimum_rad)}, '
                f'max={list(calibration.maximum_rad)}'
            )

        if (
            self._transport == 'sim'
            and (auto_home or (use_saved_calibration and auto_enable_torque))
            and not self._wait_for_sim_feedback()
        ):
            raise GripperError(
                'Isaac gripper feedback was not received before startup '
                'calibration/torque enable'
            )

        if auto_home:
            if self._transport == 'real':
                self.get_logger().warning(
                    'REAL AUTO-HOMING STARTING: both grippers will move '
                    'through their full stroke. Keep both sides clear.'
                )
            else:
                self.get_logger().info(
                    'auto_home is enabled; running virtual dual-gripper '
                    'endpoint calibration'
                )
            self._home_driver()
        elif (
            self._driver.calibration is not None
            and auto_enable_torque
        ):
            self._driver.set_torque_enabled(True)

    def _calibration_from_parameters(self) -> Calibration:
        minimum = _two_values(
            self.get_parameter('calibration_min_rad').value,
            'calibration_min_rad',
        )
        maximum = _two_values(
            self.get_parameter('calibration_max_rad').value,
            'calibration_max_rad',
        )
        return Calibration(
            minimum_rad=(float(minimum[0]), float(minimum[1])),
            maximum_rad=(float(maximum[0]), float(maximum[1])),
            closed_at_minimum=self._closed_at_minimum,
            endpoint_margin_ratio=self._endpoint_margin_ratio,
            minimum_span_rad=self._minimum_calibration_span_rad,
        )

    def _home_driver(self) -> Calibration:
        # The physical left motor was verified to close for positive torque
        # and open for negative torque. Sweep open first and close second so
        # a successful real homing run finishes at the closed endpoint.
        # Isaac's virtual motor keeps its native positive-then-negative sweep.
        direction_order = (
            (-1.0, 1.0)
            if self._transport == 'real'
            else (1.0, -1.0)
        )
        calibration = self._driver.home(
            homing_torque=float(self.get_parameter('homing_torque').value),
            direction_order=direction_order,
            sample_period_sec=float(
                self.get_parameter('homing_sample_period_sec').value
            ),
            stall_threshold_rad=float(
                self.get_parameter('homing_stall_threshold_rad').value
            ),
            stall_samples=int(
                self.get_parameter('homing_stall_samples').value
            ),
            direction_timeout_sec=float(
                self.get_parameter('homing_direction_timeout_sec').value
            ),
            max_read_failures=int(
                self.get_parameter('homing_max_read_failures').value
            ),
            closed_at_minimum=self._closed_at_minimum,
            endpoint_margin_ratio=self._endpoint_margin_ratio,
            minimum_span_rad=self._minimum_calibration_span_rad,
        )
        self.get_logger().info(
            f'{self._transport.capitalize()} gripper homing complete. '
            'Save these values to skip '
            f'future homing: min={list(calibration.minimum_rad)}, '
            f'max={list(calibration.maximum_rad)}'
        )
        return calibration

    def _on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) != 2:
            self._warn_throttled(
                'bad_command_length',
                'Rejected gripper command: data must be [right, left]',
            )
            return
        if not self._transport_feedback_ready():
            self._warn_throttled(
                'sim_feedback_missing',
                'Rejected gripper command: Isaac feedback is unavailable '
                'or stale',
            )
            self._publish_ready()
            return
        try:
            self._driver.command(message.data)
        except GripperError as exc:
            self._warn_throttled(
                'command_rejected', f'Rejected gripper command: {exc}'
            )
        self._publish_ready()

    def _on_home(self, _request, response):
        if not self._transport_feedback_ready():
            response.success = False
            response.message = (
                'Isaac gripper feedback is unavailable or stale'
            )
            self._publish_ready(force=True)
            self._publish_diagnostics()
            return response
        if self._transport == 'real':
            self.get_logger().warning(
                'Physical dual-gripper homing requested. Both grippers will '
                'move through their full stroke; keep both sides clear.'
            )
        else:
            self.get_logger().info(
                'Virtual dual-gripper homing requested'
            )
        not_ready = Bool()
        not_ready.data = False
        self._ready_pub.publish(not_ready)
        self._last_ready = False
        try:
            if not self._driver.initialized:
                self._driver.initialize()
            calibration = self._home_driver()
            response.success = True
            response.message = (
                f'min={list(calibration.minimum_rad)}, '
                f'max={list(calibration.maximum_rad)}'
            )
        except Exception as exc:
            # initialize() may have opened the serial port before discovering
            # that one or both motors are unpowered. Close it so the next HOME
            # request can make a clean retry after 12 V is restored.
            if not self._driver.initialized:
                self._driver.shutdown(False)
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Gripper initialization failed: {exc}')
        self._publish_ready(force=True)
        self._publish_diagnostics()
        return response

    def _on_torque_enable(self, request, response):
        try:
            self._driver.set_torque_enabled(bool(request.data))
            response.success = True
            response.message = (
                'gripper torque enabled'
                if request.data
                else 'gripper torque disabled'
            )
        except GripperError as exc:
            response.success = False
            response.message = str(exc)
        self._publish_ready(force=True)
        self._publish_diagnostics()
        return response

    def _refresh_target(self) -> None:
        try:
            self._driver.repeat_last_command()
        except GripperError as exc:
            self._warn_throttled(
                'target_refresh', f'Failed to refresh gripper target: {exc}'
            )

    def _publish_state(self) -> None:
        try:
            state = self._driver.read_state()
        except GripperError as exc:
            self._warn_throttled(
                'state_read', f'Failed to read gripper state: {exc}'
            )
            self._publish_ready()
            self._publish_diagnostics()
            return

        self._last_state = state
        # Before calibration the core deliberately reports NaN close ratios.
        # Do not put those on the normalized ROS contract; motor_state still
        # exposes raw encoder feedback for hardware diagnostics.
        if all(math.isfinite(value) for value in state.close_ratios):
            normalized = Float64MultiArray()
            normalized.data = list(state.close_ratios)
            self._state_pub.publish(normalized)

        motor_state = JointState()
        motor_state.header.stamp = self.get_clock().now().to_msg()
        motor_state.name = ['right_gripper_motor', 'left_gripper_motor']
        motor_state.position = list(state.positions_rad)
        motor_state.velocity = list(state.velocities_rad_s)
        self._motor_state_pub.publish(motor_state)

        self._publish_ready()
        self._publish_diagnostics(state)

    def _publish_ready(self, force: bool = False) -> None:
        ready = self._driver.ready and self._transport_feedback_ready()
        if force or ready != self._last_ready:
            message = Bool()
            message.data = ready
            self._ready_pub.publish(message)
            self._last_ready = ready

    def _publish_diagnostics(
        self, state: Optional[GripperState] = None
    ) -> None:
        state = state or self._last_state
        status = DiagnosticStatus()
        namespace = self.get_namespace().rstrip('/')
        status.name = (
            f'{namespace}/gripper_driver'
            if namespace
            else '/gripper_driver'
        )
        status.hardware_id = self._hardware_id
        communication_healthy = (
            self._driver.healthy and self._transport_feedback_ready()
        )
        ready = self._driver.ready and self._transport_feedback_ready()

        if not communication_healthy:
            status.level = DiagnosticStatus.ERROR
            status.message = (
                'Isaac gripper feedback is unavailable or stale'
                if self._transport == 'sim'
                else 'Dynamixel communication is unhealthy'
            )
        elif self._driver.busy:
            status.level = DiagnosticStatus.WARN
            status.message = 'Homing in progress'
        elif self._driver.calibration is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'Calibration required'
        elif not self._driver.enabled:
            status.level = DiagnosticStatus.WARN
            status.message = 'Torque disabled'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Ready'

        values = {
            'transport': self._transport,
            'driver': self._driver_name,
            'endpoint': self._hardware_id,
            self._version_key: self._driver_version,
            'ready': str(ready).lower(),
            'initialized': str(self._driver.initialized).lower(),
            'healthy': str(communication_healthy).lower(),
            'torque_enabled': str(self._driver.enabled).lower(),
            'motor_ids_right_left': str(list(self._motor_ids)),
            'active_motor_ids': str(list(self._driver.active_motor_ids)),
            'state_source': (
                'isaac_feedback'
                if self._transport == 'sim'
                else 'dynamixel_measurement'
            ),
        }
        if self._transport == 'real':
            # Keep the pre-simulation diagnostic key for real-mode consumers.
            values['device_name'] = self._hardware_id
            values['baud_rate'] = str(self._baud_rate)
        if self._transport == 'sim':
            feedback_age = self._bus.feedback_age_sec
            values['sim_feedback_received'] = str(
                self._bus.feedback_received
            ).lower()
            values['sim_feedback_age_sec'] = (
                'never' if feedback_age is None else f'{feedback_age:.3f}'
            )
        calibration = self._driver.calibration
        if calibration is not None:
            values['calibration_min_rad_right_left'] = str(
                list(calibration.minimum_rad)
            )
            values['calibration_max_rad_right_left'] = str(
                list(calibration.maximum_rad)
            )

        if self._driver.target_close_ratios is not None:
            values['target_close_ratio_right_left'] = str(
                list(self._driver.target_close_ratios)
            )
        if state is not None:
            values['close_ratio_right_left'] = str(
                list(state.close_ratios)
            )
            values['position_rad_right_left'] = str(
                list(state.positions_rad)
            )
            values['current_amp_right_left'] = str(
                list(state.currents_amp)
            )
            values['temperature_c_right_left'] = str(
                list(state.temperatures_c)
            )
        status.values = [
            KeyValue(key=key, value=value)
            for key, value in values.items()
        ]

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics_pub.publish(message)

    def _transport_feedback_ready(self) -> bool:
        return (
            self._transport != 'sim'
            or bool(self._bus.feedback_fresh)
        )

    def _wait_for_sim_feedback(self) -> bool:
        timeout = float(
            self.get_parameter('sim_feedback_timeout_sec').value
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._transport_feedback_ready():
                return True
            time.sleep(0.01)
        return self._transport_feedback_ready()

    def _warn_throttled(
        self, key: str, message: str, period_sec: float = 2.0
    ) -> None:
        now = time.monotonic()
        last = self._last_warning_at.get(key, -math.inf)
        if now - last >= period_sec:
            self.get_logger().warning(message)
            self._last_warning_at[key] = now

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._driver.shutdown(self._disable_torque_on_shutdown)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GripperDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
