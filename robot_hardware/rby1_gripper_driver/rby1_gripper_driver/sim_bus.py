"""Dynamixel-like UDP transport for the RBY1 Isaac Sim gripper bridge.

The packet layout and virtual calibration range match the Apache-2.0
``SimDynamixelBus`` shipped by Rainbow Robotics in ``rby1-sim-isaac``.  This
local adapter intentionally implements only the methods used by
``GripperDriver`` so the ROS package does not depend on Isaac Sim's Python
environment.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import socket
import struct
import threading
import time
from typing import Dict, Optional, Sequence


_COMMAND_MAGIC = 0x4743
_STATE_MAGIC = 0x4753
_COMMAND_FORMAT = '<HBBff'
_STATE_FORMAT = '<HHfff'

SIM_HOME_MIN_RAD = -0.785398
SIM_HOME_MAX_RAD = 0.785398
SIM_HOMING_STEP_RAD = 0.1

_CURRENT_CONTROL_MODE = 0
_CURRENT_BASED_POSITION_CONTROL_MODE = 5
_FAKE_MODEL_NUMBER = 1020
_MOTOR_IDS = (0, 1)


@dataclass
class _Motor:
    torque_enable: bool = False
    operating_mode: int = _CURRENT_BASED_POSITION_CONTROL_MODE
    goal_position_rad: float = SIM_HOME_MAX_RAD
    present_position_rad: float = SIM_HOME_MAX_RAD
    goal_torque: float = 0.0
    homing_direction: int = 0


@dataclass(frozen=True)
class SimMotorState:
    """Attribute-compatible subset of an SDK Dynamixel motor state."""

    torque_enable: bool
    position: float
    velocity: float = 0.0
    current: float = 0.0
    temperature: int = 25


def _validate_port(value: int, label: str, *, allow_zero: bool = False) -> int:
    port = int(value)
    minimum = 0 if allow_zero else 1
    if port < minimum or port > 65535:
        raise ValueError(f'{label} must be in [{minimum}, 65535]')
    return port


def _closeness_from_rad(position_rad: float) -> float:
    span = SIM_HOME_MAX_RAD - SIM_HOME_MIN_RAD
    ratio = (SIM_HOME_MAX_RAD - float(position_rad)) / span
    return max(0.0, min(1.0, ratio))


def _rad_from_closeness(close_ratio: float) -> float:
    ratio = max(0.0, min(1.0, float(close_ratio)))
    span = SIM_HOME_MAX_RAD - SIM_HOME_MIN_RAD
    return SIM_HOME_MAX_RAD - ratio * span


class SimDynamixelBus:
    """Subset of ``rby1_sdk.DynamixelBus`` backed by Isaac Sim UDP."""

    current_control_mode = _CURRENT_CONTROL_MODE
    current_based_position_control_mode = (
        _CURRENT_BASED_POSITION_CONTROL_MODE
    )

    def __init__(
        self,
        *,
        sim_host: str = '127.0.0.1',
        command_port: int = 5007,
        state_port: int = 5008,
        feedback_timeout_sec: float = 1.0,
    ) -> None:
        host = str(sim_host).strip()
        if not host:
            raise ValueError('sim_host must not be empty')
        timeout = float(feedback_timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('sim_feedback_timeout_sec must be positive')

        self._sim_address = (
            host,
            _validate_port(command_port, 'sim_command_port'),
        )
        # Port zero is useful for isolated unit tests. The ROS-facing factory
        # rejects it, so production always has a stable feedback endpoint.
        self._state_port = _validate_port(
            state_port, 'sim_state_port', allow_zero=True
        )
        self._feedback_timeout_sec = timeout
        self._motors: Dict[int, _Motor] = {
            motor_id: _Motor() for motor_id in _MOTOR_IDS
        }
        self._lock = threading.RLock()
        self._send_socket: Optional[socket.socket] = None
        self._receive_socket: Optional[socket.socket] = None
        self._receive_thread: Optional[threading.Thread] = None
        self._running = False
        self._last_feedback_at: Optional[float] = None
        self._torque_constants = [1.0, 1.0]

    @property
    def state_port(self) -> int:
        """Return the bound feedback port, resolving a requested port zero."""

        return self._state_port

    @property
    def feedback_received(self) -> bool:
        with self._lock:
            return self._last_feedback_at is not None

    @property
    def feedback_fresh(self) -> bool:
        """Return whether an Isaac state packet arrived within the timeout."""

        with self._lock:
            return self._feedback_is_fresh_locked()

    @property
    def feedback_age_sec(self) -> Optional[float]:
        with self._lock:
            if self._last_feedback_at is None:
                return None
            return max(0.0, time.monotonic() - self._last_feedback_at)

    def open_port(self) -> bool:
        if self._send_socket is not None:
            return True

        send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receive_socket.settimeout(0.2)
        try:
            receive_socket.bind(('0.0.0.0', self._state_port))
        except OSError:
            send_socket.close()
            receive_socket.close()
            return False

        self._state_port = int(receive_socket.getsockname()[1])
        self._send_socket = send_socket
        self._receive_socket = receive_socket
        with self._lock:
            self._last_feedback_at = None
        self._running = True
        self._receive_thread = threading.Thread(
            target=self._receive_loop,
            name='RBY1SimGripperState',
            daemon=True,
        )
        self._receive_thread.start()
        return True

    def close_port(self) -> None:
        self._running = False
        for transport_socket in (self._receive_socket, self._send_socket):
            if transport_socket is not None:
                try:
                    transport_socket.close()
                except OSError:
                    pass
        self._receive_socket = None
        self._send_socket = None
        if self._receive_thread is not None:
            self._receive_thread.join(timeout=0.5)
            self._receive_thread = None

    def set_baud_rate(self, _baud_rate: int) -> bool:
        return True

    def set_torque_constant(self, values: Sequence[float]) -> None:
        with self._lock:
            self._torque_constants = [float(value) for value in values]

    def ping(self, motor_id: int) -> Optional[int]:
        return (
            _FAKE_MODEL_NUMBER
            if int(motor_id) in self._motors
            else None
        )

    def group_sync_write_torque_enable(
        self, ids: Sequence[int], enabled: int
    ) -> None:
        with self._lock:
            for motor_id in ids:
                motor = self._motors.get(int(motor_id))
                if motor is not None:
                    motor.torque_enable = bool(enabled)
        self._send_command_packet()

    def group_sync_write_operating_mode(self, values) -> None:
        with self._lock:
            for motor_id, mode in values:
                motor = self._motors.get(int(motor_id))
                if motor is not None:
                    motor.operating_mode = int(mode)

    def group_sync_write_send_torque(self, values) -> None:
        with self._lock:
            for motor_id, torque in values:
                motor = self._motors.get(int(motor_id))
                if motor is None:
                    continue
                motor.goal_torque = float(torque)
                if motor.operating_mode == _CURRENT_CONTROL_MODE:
                    motor.homing_direction = (
                        1 if torque > 1e-9 else -1 if torque < -1e-9 else 0
                    )

    def group_sync_write_send_position(self, values) -> None:
        with self._lock:
            for motor_id, position in values:
                motor = self._motors.get(int(motor_id))
                if motor is not None:
                    motor.goal_position_rad = float(position)
        self._send_command_packet()

    def group_fast_sync_read_encoder(self, ids: Sequence[int]):
        result = []
        with self._lock:
            for requested_id in ids:
                motor_id = int(requested_id)
                motor = self._motors.get(motor_id)
                if motor is None:
                    continue
                if motor.operating_mode == _CURRENT_CONTROL_MODE:
                    self._advance_fake_homing_locked(motor)
                elif not self._feedback_is_fresh_locked():
                    return None
                result.append((motor_id, motor.present_position_rad))
        return result or None

    def get_motor_states(self, ids: Sequence[int]):
        result = []
        with self._lock:
            if not self._feedback_is_fresh_locked():
                return None
            for requested_id in ids:
                motor_id = int(requested_id)
                motor = self._motors.get(motor_id)
                if motor is None:
                    continue
                result.append((
                    motor_id,
                    SimMotorState(
                        torque_enable=motor.torque_enable,
                        position=motor.present_position_rad,
                        current=motor.goal_torque,
                    ),
                ))
        return result or None

    def _feedback_is_fresh_locked(self) -> bool:
        return (
            self._last_feedback_at is not None
            and time.monotonic() - self._last_feedback_at
            <= self._feedback_timeout_sec
        )

    @staticmethod
    def _advance_fake_homing_locked(motor: _Motor) -> None:
        if motor.homing_direction > 0:
            motor.present_position_rad = min(
                motor.present_position_rad + SIM_HOMING_STEP_RAD,
                SIM_HOME_MAX_RAD,
            )
        elif motor.homing_direction < 0:
            motor.present_position_rad = max(
                motor.present_position_rad - SIM_HOMING_STEP_RAD,
                SIM_HOME_MIN_RAD,
            )

    def _send_command_packet(self) -> None:
        send_socket = self._send_socket
        if send_socket is None:
            return

        with self._lock:
            left = self._motors[0]
            right = self._motors[1]
            # Isaac's server currently applies both packet values even when a
            # torque-enable flag is false. Preserve the measured position of
            # each disabled side so torque-disabled motors cannot move as a
            # side effect of a command for the other motor.
            left_position = (
                left.goal_position_rad
                if left.torque_enable
                else left.present_position_rad
            )
            right_position = (
                right.goal_position_rad
                if right.torque_enable
                else right.present_position_rad
            )
            flags = (
                0x1
                | (int(left.torque_enable) << 1)
                | (int(right.torque_enable) << 2)
            )
            packet = struct.pack(
                _COMMAND_FORMAT,
                _COMMAND_MAGIC,
                flags,
                0,
                _closeness_from_rad(left_position),
                _closeness_from_rad(right_position),
            )
        try:
            send_socket.sendto(packet, self._sim_address)
        except OSError:
            pass

    def _receive_loop(self) -> None:
        receive_socket = self._receive_socket
        if receive_socket is None:
            return
        packet_size = struct.calcsize(_STATE_FORMAT)
        while self._running:
            try:
                packet, _address = receive_socket.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(packet) < packet_size:
                continue
            try:
                magic, _reserved, left, right, _sim_time = struct.unpack_from(
                    _STATE_FORMAT, packet
                )
            except struct.error:
                continue
            if magic != _STATE_MAGIC:
                continue
            with self._lock:
                for motor_id, close_ratio in ((0, left), (1, right)):
                    motor = self._motors[motor_id]
                    if (
                        motor.operating_mode
                        == _CURRENT_BASED_POSITION_CONTROL_MODE
                    ):
                        motor.present_position_rad = _rad_from_closeness(
                            close_ratio
                        )
                self._last_feedback_at = time.monotonic()


__all__ = [
    'SIM_HOME_MAX_RAD',
    'SIM_HOME_MIN_RAD',
    'SimDynamixelBus',
    'SimMotorState',
]
