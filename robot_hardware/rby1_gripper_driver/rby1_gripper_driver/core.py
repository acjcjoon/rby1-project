"""Hardware-independent RB-Y1 gripper control logic.

The ROS node and the vendor SDK adapter deliberately live outside this module
so the calibration and command-safety rules can be unit tested without ROS or
a serial device.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Callable, Optional, Sequence, Tuple


Pair = Tuple[float, float]
BoolPair = Tuple[bool, bool]


class GripperError(RuntimeError):
    """Base error raised by the gripper driver core."""


class GripperConfigurationError(GripperError):
    """Raised for an invalid static configuration or calibration."""


class GripperHardwareError(GripperError):
    """Raised when the Dynamixel gripper does not return usable data."""


class GripperNotReadyError(GripperError):
    """Raised when a command arrives before the driver is ready."""


def _float_pair(values: Sequence[float], label: str) -> Pair:
    if len(values) != 2:
        raise GripperConfigurationError(f'{label} must contain 2 values')
    if any(isinstance(value, bool) for value in values):
        raise GripperConfigurationError(f'{label} must contain numbers')
    pair = (float(values[0]), float(values[1]))
    if not all(math.isfinite(value) for value in pair):
        raise GripperConfigurationError(f'{label} must contain finite values')
    return pair


@dataclass(frozen=True)
class Calibration:
    """Encoder endpoints and direction for the right and left grippers."""

    minimum_rad: Pair
    maximum_rad: Pair
    closed_at_minimum: BoolPair = (True, True)
    endpoint_margin_ratio: float = 0.0
    minimum_span_rad: float = 0.01

    def __post_init__(self) -> None:
        minimum = _float_pair(self.minimum_rad, 'minimum_rad')
        maximum = _float_pair(self.maximum_rad, 'maximum_rad')
        if len(self.closed_at_minimum) != 2:
            raise GripperConfigurationError(
                'closed_at_minimum must contain 2 values'
            )
        if not 0.0 <= float(self.endpoint_margin_ratio) < 0.5:
            raise GripperConfigurationError(
                'endpoint_margin_ratio must be in [0.0, 0.5)'
            )
        if not math.isfinite(float(self.minimum_span_rad)):
            raise GripperConfigurationError('minimum_span_rad must be finite')
        if float(self.minimum_span_rad) <= 0.0:
            raise GripperConfigurationError(
                'minimum_span_rad must be positive'
            )
        for index, (low, high) in enumerate(zip(minimum, maximum)):
            if high - low < float(self.minimum_span_rad):
                side = ('right', 'left')[index]
                raise GripperConfigurationError(
                    f'{side} calibration span is too small: '
                    f'{high - low:.6f} rad'
                )

    @property
    def usable_minimum_rad(self) -> Pair:
        margin = float(self.endpoint_margin_ratio)
        return tuple(
            low + ((high - low) * margin)
            for low, high in zip(self.minimum_rad, self.maximum_rad)
        )  # type: ignore[return-value]

    @property
    def usable_maximum_rad(self) -> Pair:
        margin = float(self.endpoint_margin_ratio)
        return tuple(
            high - ((high - low) * margin)
            for low, high in zip(self.minimum_rad, self.maximum_rad)
        )  # type: ignore[return-value]

    def positions_for(self, close_ratios: Sequence[float]) -> Pair:
        """Convert close ratios (0=open, 1=closed) to encoder radians."""

        ratios = _float_pair(close_ratios, 'close_ratios')
        if any(value < 0.0 or value > 1.0 for value in ratios):
            raise GripperConfigurationError(
                'close_ratios must be within [0.0, 1.0]'
            )
        result = []
        for ratio, low, high, closed_at_low in zip(
            ratios,
            self.usable_minimum_rad,
            self.usable_maximum_rad,
            self.closed_at_minimum,
        ):
            if closed_at_low:
                result.append(high - ratio * (high - low))
            else:
                result.append(low + ratio * (high - low))
        return (result[0], result[1])

    def close_ratios_for(self, positions_rad: Sequence[float]) -> Pair:
        """Convert encoder radians to clamped close ratios."""

        positions = _float_pair(positions_rad, 'positions_rad')
        result = []
        for position, low, high, closed_at_low in zip(
            positions,
            self.usable_minimum_rad,
            self.usable_maximum_rad,
            self.closed_at_minimum,
        ):
            if closed_at_low:
                ratio = (high - position) / (high - low)
            else:
                ratio = (position - low) / (high - low)
            result.append(max(0.0, min(1.0, ratio)))
        return (result[0], result[1])


@dataclass(frozen=True)
class GripperState:
    """One ordered state sample; every pair is ``(right, left)``."""

    positions_rad: Pair
    velocities_rad_s: Pair
    currents_amp: Pair
    temperatures_c: Pair
    torque_enabled: BoolPair
    close_ratios: Pair


class GripperDriver:
    """Own a bus and enforce calibration before position commands."""

    def __init__(
        self,
        bus: Any,
        *,
        motor_ids: Sequence[int] = (0, 1),
        torque_constants: Sequence[float] = (1.0, 1.0),
        baud_rate: int = 2_000_000,
        position_torque_limit: float = 5.0,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        if len(motor_ids) != 2:
            raise GripperConfigurationError('motor_ids must contain 2 values')
        ids = (int(motor_ids[0]), int(motor_ids[1]))
        if ids[0] == ids[1] or any(
            value < 0 or value >= 0x80 for value in ids
        ):
            raise GripperConfigurationError(
                'motor_ids must be distinct Dynamixel IDs in [0, 127]'
            )
        constants = _float_pair(torque_constants, 'torque_constants')
        if any(value <= 0.0 for value in constants):
            raise GripperConfigurationError(
                'torque_constants must contain positive values'
            )
        if int(baud_rate) <= 0:
            raise GripperConfigurationError('baud_rate must be positive')
        if not math.isfinite(float(position_torque_limit)):
            raise GripperConfigurationError(
                'position_torque_limit must be finite'
            )
        if float(position_torque_limit) <= 0.0:
            raise GripperConfigurationError(
                'position_torque_limit must be positive'
            )
        self._bus = bus
        self._motor_ids = ids
        self._torque_constants = constants

        # Semantic index 0 is the right gripper and index 1 is the left
        # gripper. With the physical default mapping those are Dynamixel IDs
        # 0 and 1 respectively. Every SDK operation below is derived from the
        # active semantic index.
        self._active_indices = (0, 1)

        self._baud_rate = int(baud_rate)
        self._position_torque_limit = float(position_torque_limit)
        self._clock = clock
        self._sleep = sleeper
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()

        self._initialized = False
        self._healthy = False
        self._enabled = False
        self._busy = False
        self._calibration: Optional[Calibration] = None
        self._target_close_ratios: Optional[Pair] = None
        self._target_positions_rad: Optional[Pair] = None

    @property
    def motor_ids(self) -> Tuple[int, int]:
        return self._motor_ids

    @property
    def active_motor_ids(self) -> Tuple[int, ...]:
        return tuple(self._motor_ids[index] for index in self._active_indices)

    @property
    def initialized(self) -> bool:
        with self._lock:
            return self._initialized

    @property
    def healthy(self) -> bool:
        with self._lock:
            return self._healthy

    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    @property
    def calibration(self) -> Optional[Calibration]:
        with self._lock:
            return self._calibration

    @property
    def target_close_ratios(self) -> Optional[Pair]:
        with self._lock:
            return self._target_close_ratios

    @property
    def ready(self) -> bool:
        with self._lock:
            return (
                self._initialized
                and self._healthy
                and self._enabled
                and not self._busy
                and self._calibration is not None
            )

    def initialize(self) -> None:
        """Open the serial bus, configure it, and verify active motors."""

        with self._lock:
            if self._initialized:
                return
            if not self._bus.open_port():
                raise GripperHardwareError('failed to open the gripper port')
            if not self._bus.set_baud_rate(self._baud_rate):
                raise GripperHardwareError(
                    f'failed to set gripper baud rate to {self._baud_rate}'
                )

            constants = [1.0] * (max(self.active_motor_ids) + 1)
            for index in self._active_indices:
                constants[self._motor_ids[index]] = self._torque_constants[index]
            self._bus.set_torque_constant(constants)

            missing = [
                motor_id
                for motor_id in self.active_motor_ids
                if not self._bus.ping(motor_id)
            ]
            if missing:
                raise GripperHardwareError(
                    f'gripper Dynamixel IDs did not respond: {missing}'
                )
            self._initialized = True
            self._healthy = True
            self._stop_requested.clear()

    def load_calibration(self, calibration: Calibration) -> None:
        """Load known endpoints without moving the hardware."""

        if not isinstance(calibration, Calibration):
            raise GripperConfigurationError('calibration has an invalid type')
        with self._lock:
            if self._busy:
                raise GripperNotReadyError('gripper is busy')
            self._calibration = calibration
            self._target_close_ratios = None
            self._target_positions_rad = None

    def home(
        self,
        *,
        homing_torque: float = 0.1,
        direction_order: Sequence[float] = (1.0, -1.0),
        sample_period_sec: float = 0.05,
        stall_threshold_rad: float = 0.002,
        stall_samples: int = 10,
        direction_timeout_sec: float = 10.0,
        max_read_failures: int = 3,
        closed_at_minimum: Sequence[bool] = (True, True),
        endpoint_margin_ratio: float = 0.0,
        minimum_span_rad: float = 0.01,
    ) -> Calibration:
        """Discover active encoder endpoints by gently driving to hard stops."""

        homing_torque = float(homing_torque)
        directions = _float_pair(direction_order, 'direction_order')
        sample_period_sec = float(sample_period_sec)
        stall_threshold_rad = float(stall_threshold_rad)
        direction_timeout_sec = float(direction_timeout_sec)
        if not math.isfinite(homing_torque) or homing_torque <= 0.0:
            raise GripperConfigurationError('homing_torque must be positive')
        if set(directions) != {-1.0, 1.0}:
            raise GripperConfigurationError(
                'direction_order must contain -1.0 and 1.0 exactly once'
            )
        if not math.isfinite(sample_period_sec) or sample_period_sec <= 0.0:
            raise GripperConfigurationError(
                'sample_period_sec must be positive'
            )
        if not math.isfinite(stall_threshold_rad) or stall_threshold_rad < 0.0:
            raise GripperConfigurationError(
                'stall_threshold_rad must be non-negative'
            )
        if int(stall_samples) <= 0:
            raise GripperConfigurationError('stall_samples must be positive')
        if (
            not math.isfinite(direction_timeout_sec)
            or direction_timeout_sec <= 0.0
        ):
            raise GripperConfigurationError(
                'direction_timeout_sec must be positive'
            )
        if int(max_read_failures) < 0:
            raise GripperConfigurationError(
                'max_read_failures must be non-negative'
            )
        if len(closed_at_minimum) != 2:
            raise GripperConfigurationError(
                'closed_at_minimum must contain 2 values'
            )

        with self._lock:
            self._require_initialized_locked()
            if self._busy:
                raise GripperNotReadyError('gripper is already busy')
            self._busy = True
            self._enabled = False
            self._target_close_ratios = None
            self._target_positions_rad = None

            try:
                self._set_mode_locked(self._bus.current_control_mode)
                current = self._read_encoders_locked()
                minimum = [current[0], current[1]]
                maximum = [current[0], current[1]]
                for index in set(range(2)) - set(self._active_indices):
                    # A valid placeholder keeps the public (right, left) pair
                    # shape intact. It is never used for an SDK operation.
                    minimum[index] = 0.0
                    maximum[index] = 1.0

                for direction in directions:
                    stable_count = 0
                    read_failures = 0
                    previous = current
                    deadline = self._clock() + direction_timeout_sec

                    while stable_count < int(stall_samples):
                        if self._stop_requested.is_set():
                            raise GripperHardwareError(
                                'homing was interrupted'
                            )
                        if self._clock() >= deadline:
                            raise GripperHardwareError(
                                'homing timed out before active motors stalled'
                            )

                        self._bus.group_sync_write_send_torque([
                            (
                                self._motor_ids[index],
                                direction * homing_torque,
                            )
                            for index in self._active_indices
                        ])
                        self._sleep(sample_period_sec)

                        try:
                            current = self._read_encoders_locked()
                        except GripperHardwareError:
                            read_failures += 1
                            if read_failures > int(max_read_failures):
                                raise
                            continue

                        read_failures = 0
                        for index in self._active_indices:
                            minimum[index] = min(
                                minimum[index], current[index]
                            )
                            maximum[index] = max(
                                maximum[index], current[index]
                            )

                        if all(
                            abs(current[index] - previous[index])
                            <= stall_threshold_rad
                            for index in self._active_indices
                        ):
                            stable_count += 1
                        else:
                            stable_count = 0
                        previous = current

                self._bus.group_sync_write_send_torque([
                    (motor_id, 0.0) for motor_id in self.active_motor_ids
                ])
                calibration_closed_at_minimum = [True, True]
                for index in self._active_indices:
                    calibration_closed_at_minimum[index] = bool(
                        closed_at_minimum[index]
                    )
                calibration = Calibration(
                    minimum_rad=(minimum[0], minimum[1]),
                    maximum_rad=(maximum[0], maximum[1]),
                    closed_at_minimum=(
                        calibration_closed_at_minimum[0],
                        calibration_closed_at_minimum[1],
                    ),
                    endpoint_margin_ratio=float(endpoint_margin_ratio),
                    minimum_span_rad=float(minimum_span_rad),
                )
                self._calibration = calibration
                self._activate_holding_locked(current)
                self._target_close_ratios = calibration.close_ratios_for(
                    current
                )
                self._target_positions_rad = current
                self._healthy = True
                return calibration
            except Exception:
                self._safe_disable_locked()
                self._healthy = False
                raise
            finally:
                self._busy = False

    def set_torque_enabled(self, enabled: bool) -> None:
        """Enable safely at the current position, or disable active motors."""

        with self._lock:
            self._require_initialized_locked()
            if self._busy:
                raise GripperNotReadyError('gripper is busy')
            if not enabled:
                self._safe_disable_locked()
                self._target_close_ratios = None
                self._target_positions_rad = None
                return
            if self._calibration is None:
                raise GripperNotReadyError(
                    'gripper must be homed or given saved calibration first'
                )
            positions = self._read_encoders_locked()
            self._activate_holding_locked(positions)
            self._target_positions_rad = positions
            self._target_close_ratios = (
                self._calibration.close_ratios_for(positions)
            )

    def command(self, close_ratios: Sequence[float]) -> Pair:
        """Send one normalized active-gripper command and remember the target."""

        requested_ratios = _float_pair(close_ratios, 'close_ratios')
        if any(value < 0.0 or value > 1.0 for value in requested_ratios):
            raise GripperConfigurationError(
                'close_ratios must be within [0.0, 1.0]'
            )
        ratio_values = list(requested_ratios)
        for index in set(range(2)) - set(self._active_indices):
            ratio_values[index] = 1.0
        ratios = (ratio_values[0], ratio_values[1])
        with self._lock:
            self._require_ready_locked()
            assert self._calibration is not None
            positions = self._calibration.positions_for(ratios)
            self._write_positions_locked(positions)
            self._target_close_ratios = ratios
            self._target_positions_rad = positions
            return positions

    def repeat_last_command(self) -> None:
        """Refresh the current target, matching the vendor 10 Hz example."""

        with self._lock:
            if not self.ready or self._target_positions_rad is None:
                return
            self._write_positions_locked(self._target_positions_rad)

    def read_state(self) -> GripperState:
        """Read active motor states and convert positions to close ratios."""

        with self._lock:
            self._require_initialized_locked()
            raw = self._bus.get_motor_states(list(self.active_motor_ids))
            if raw is None:
                self._healthy = False
                raise GripperHardwareError(
                    'failed to read gripper motor states'
                )
            states = {int(motor_id): state for motor_id, state in raw}
            if any(motor_id not in states for motor_id in self.active_motor_ids):
                self._healthy = False
                raise GripperHardwareError(
                    'motor-state response did not contain every active gripper'
                )

            ordered = {
                index: states[self._motor_ids[index]]
                for index in self._active_indices
            }
            positions = self._state_pair_values(ordered, 'position')
            velocities = self._state_pair_values(ordered, 'velocity')
            currents = self._state_pair_values(ordered, 'current')
            temperatures = self._state_pair_values(ordered, 'temperature')
            torque_enabled_values = [False, False]
            for index in self._active_indices:
                torque_enabled_values[index] = bool(
                    getattr(ordered[index], 'torque_enable', False)
                )
            torque_enabled = (
                torque_enabled_values[0],
                torque_enabled_values[1],
            )
            if self._enabled and not all(
                torque_enabled[index] for index in self._active_indices
            ):
                self._enabled = False
                self._target_close_ratios = None
                self._target_positions_rad = None
            if self._calibration is None:
                close_ratios = (math.nan, math.nan)
            else:
                close_ratios = self._calibration.close_ratios_for(positions)
            self._healthy = True
            return GripperState(
                positions_rad=positions,
                velocities_rad_s=velocities,
                currents_amp=currents,
                temperatures_c=temperatures,
                torque_enabled=torque_enabled,
                close_ratios=close_ratios,
            )

    def shutdown(self, disable_torque: bool = True) -> None:
        """Stop homing, optionally release torque, and close when supported."""

        self._stop_requested.set()
        with self._lock:
            if self._initialized and disable_torque:
                self._safe_disable_locked()
            close_port = getattr(self._bus, 'close_port', None)
            if callable(close_port):
                try:
                    close_port()
                except Exception:
                    pass
            self._initialized = False
            self._healthy = False

    def _require_initialized_locked(self) -> None:
        if not self._initialized:
            raise GripperNotReadyError('gripper bus is not initialized')

    def _require_ready_locked(self) -> None:
        if self._busy:
            raise GripperNotReadyError('gripper is busy')
        if self._calibration is None:
            raise GripperNotReadyError(
                'gripper must be homed or given saved calibration first'
            )
        if not self._enabled:
            raise GripperNotReadyError('gripper torque is disabled')
        if not self._healthy:
            raise GripperNotReadyError('gripper communication is unhealthy')
        self._require_initialized_locked()

    def _set_mode_locked(self, mode: int) -> None:
        ids = list(self.active_motor_ids)
        self._bus.group_sync_write_torque_enable(ids, 0)
        self._bus.group_sync_write_operating_mode([
            (motor_id, int(mode)) for motor_id in self.active_motor_ids
        ])
        self._bus.group_sync_write_send_torque([
            (motor_id, 0.0) for motor_id in self.active_motor_ids
        ])
        self._bus.group_sync_write_torque_enable(ids, 1)

    def _activate_holding_locked(self, positions: Pair) -> None:
        ids = list(self.active_motor_ids)
        self._bus.group_sync_write_torque_enable(ids, 0)
        self._bus.group_sync_write_operating_mode([
            (motor_id, self._bus.current_based_position_control_mode)
            for motor_id in self.active_motor_ids
        ])
        self._bus.group_sync_write_send_torque([
            (motor_id, self._position_torque_limit)
            for motor_id in self.active_motor_ids
        ])
        self._write_positions_locked(positions)
        self._bus.group_sync_write_torque_enable(ids, 1)
        self._enabled = True
        self._healthy = True

    def _safe_disable_locked(self) -> None:
        try:
            self._bus.group_sync_write_send_torque([
                (motor_id, 0.0) for motor_id in self.active_motor_ids
            ])
        except Exception:
            pass
        try:
            self._bus.group_sync_write_torque_enable(
                list(self.active_motor_ids), 0
            )
        except Exception:
            pass
        self._enabled = False

    def _write_positions_locked(self, positions: Pair) -> None:
        self._bus.group_sync_write_send_position([
            (self._motor_ids[index], positions[index])
            for index in self._active_indices
        ])

    def _read_encoders_locked(self) -> Pair:
        raw = self._bus.group_fast_sync_read_encoder(
            list(self.active_motor_ids)
        )
        if raw is None:
            self._healthy = False
            raise GripperHardwareError('failed to read gripper encoders')
        values = {int(motor_id): float(value) for motor_id, value in raw}
        if any(motor_id not in values for motor_id in self.active_motor_ids):
            self._healthy = False
            raise GripperHardwareError(
                'encoder response did not contain every active gripper'
            )
        pair_values = [0.0, 0.0]
        for index in self._active_indices:
            pair_values[index] = values[self._motor_ids[index]]
        if not all(
            math.isfinite(pair_values[index])
            for index in self._active_indices
        ):
            self._healthy = False
            raise GripperHardwareError('encoder response was not finite')
        self._healthy = True
        return pair_values[0], pair_values[1]

    def _state_pair_values(self, states: dict[int, Any], field: str) -> Pair:
        values = [0.0, 0.0]
        for index in self._active_indices:
            try:
                value = float(getattr(states[index], field))
            except (AttributeError, TypeError, ValueError):
                value = math.nan
            values[index] = value if math.isfinite(value) else math.nan
        return (values[0], values[1])
