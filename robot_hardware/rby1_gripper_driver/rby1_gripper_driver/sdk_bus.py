"""Select the real SDK or Isaac Sim transport for the gripper core."""

from __future__ import annotations

from importlib import metadata
from dataclasses import dataclass
from typing import Any, Callable, Optional, Tuple

from .sim_bus import SimDynamixelBus


SIM_MOTOR_IDS_RIGHT_LEFT = (1, 0)
SIM_CLOSED_AT_MINIMUM_RIGHT_LEFT = (True, True)


class SdkImportError(RuntimeError):
    """Raised when the vendor Python SDK is unavailable or incomplete."""


class SdkDynamixelBus:
    """Expose the subset of the SDK bus used by :class:`GripperDriver`."""

    def __init__(self, sdk_module, device_name: str) -> None:
        bus_type = getattr(sdk_module, 'DynamixelBus', None)
        if bus_type is None:
            raise SdkImportError(
                'rby1_sdk does not expose DynamixelBus; install a current SDK'
            )
        self._bus = bus_type(device_name)
        self.current_control_mode = int(bus_type.CurrentControlMode)
        self.current_based_position_control_mode = int(
            bus_type.CurrentBasedPositionControlMode
        )

    def open_port(self):
        return self._bus.open_port()

    def close_port(self) -> None:
        close = getattr(self._bus, 'close_port', None)
        if callable(close):
            close()

    def set_baud_rate(self, baud_rate):
        return self._bus.set_baud_rate(baud_rate)

    def set_torque_constant(self, values) -> None:
        self._bus.set_torque_constant(values)

    def ping(self, motor_id):
        return self._bus.ping(motor_id)

    def group_sync_write_torque_enable(self, ids, enabled) -> None:
        try:
            self._bus.group_sync_write_torque_enable(ids, enabled)
        except TypeError:
            self._bus.group_sync_write_torque_enable([
                (motor_id, enabled) for motor_id in ids
            ])

    def group_sync_write_operating_mode(self, values) -> None:
        self._bus.group_sync_write_operating_mode(values)

    def group_sync_write_send_torque(self, values) -> None:
        self._bus.group_sync_write_send_torque(values)

    def group_sync_write_send_position(self, values) -> None:
        self._bus.group_sync_write_send_position(values)

    def group_fast_sync_read_encoder(self, ids):
        return self._bus.group_fast_sync_read_encoder(ids)

    def get_motor_states(self, ids):
        return self._bus.get_motor_states(ids)


def create_sdk_bus(
    device_name: str = '',
    warning: Optional[Callable[[str], None]] = None,
) -> Tuple[SdkDynamixelBus, str, str]:
    """Return an SDK bus with its resolved path and package version."""

    try:
        import rby1_sdk as rby
    except ImportError as exc:
        raise SdkImportError(
            'rby1_sdk is not installed; install rby1-sdk in the ROS Python '
            'environment'
        ) from exc

    upc = getattr(rby, 'upc', None)
    if not device_name:
        device_name = str(
            getattr(upc, 'GripperDeviceName', '/dev/rby1_gripper')
        )

    initialize_device = getattr(upc, 'initialize_device', None)
    if callable(initialize_device):
        try:
            initialize_device(device_name)
        except Exception as exc:
            # The port open below is the authoritative availability test.
            if warning is not None:
                warning(
                    f'could not tune latency for {device_name}: {exc}; '
                    'continuing with the serial open check'
                )

    try:
        sdk_version = metadata.version('rby1-sdk')
    except metadata.PackageNotFoundError:
        sdk_version = 'unknown'
    return SdkDynamixelBus(rby, device_name), device_name, sdk_version


@dataclass(frozen=True)
class BusSelection:
    """One resolved transport and its diagnostic metadata."""

    bus: Any
    transport: str
    hardware_id: str
    driver_name: str
    version_key: str
    version: str


def motor_ids_for_transport(
    transport: str,
    real_motor_ids: Tuple[int, int],
) -> Tuple[int, int]:
    """Resolve the transport's numeric IDs in semantic [right, left] order."""

    normalized = str(transport).strip().lower()
    if normalized == 'real':
        return int(real_motor_ids[0]), int(real_motor_ids[1])
    if normalized == 'sim':
        return SIM_MOTOR_IDS_RIGHT_LEFT
    raise ValueError("transport must be either 'real' or 'sim'")


def closed_at_minimum_for_transport(
    transport: str,
    real_closed_at_minimum: Tuple[bool, bool],
) -> Tuple[bool, bool]:
    """Keep physical direction settings from reversing Isaac commands."""

    normalized = str(transport).strip().lower()
    if normalized == 'real':
        return (
            bool(real_closed_at_minimum[0]),
            bool(real_closed_at_minimum[1]),
        )
    if normalized == 'sim':
        return SIM_CLOSED_AT_MINIMUM_RIGHT_LEFT
    raise ValueError("transport must be either 'real' or 'sim'")


def create_bus(
    transport: str,
    *,
    device_name: str = '',
    sim_host: str = '127.0.0.1',
    sim_command_port: int = 5007,
    sim_state_port: int = 5008,
    sim_feedback_timeout_sec: float = 1.0,
    warning: Optional[Callable[[str], None]] = None,
) -> BusSelection:
    """Create a real or simulated bus without changing the driver core."""

    normalized = str(transport).strip().lower()
    if normalized == 'real':
        bus, resolved_device, sdk_version = create_sdk_bus(
            device_name,
            warning=warning,
        )
        return BusSelection(
            bus=bus,
            transport='real',
            hardware_id=resolved_device,
            driver_name='rby1_sdk.DynamixelBus',
            version_key='rby1_sdk_version',
            version=sdk_version,
        )
    if normalized == 'sim':
        command_port = _production_port(
            sim_command_port, 'sim_command_port'
        )
        state_port = _production_port(sim_state_port, 'sim_state_port')
        bus = SimDynamixelBus(
            sim_host=sim_host,
            command_port=command_port,
            state_port=state_port,
            feedback_timeout_sec=sim_feedback_timeout_sec,
        )
        return BusSelection(
            bus=bus,
            transport='sim',
            hardware_id=(
                f'udp://{str(sim_host).strip()}:{command_port}'
                f'?state_port={state_port}'
            ),
            driver_name='IsaacSimGripperUDP',
            version_key='sim_protocol',
            version='rby1-gripper-udp-v1',
        )
    raise ValueError("transport must be either 'real' or 'sim'")


def _production_port(value: int, label: str) -> int:
    port = int(value)
    if port < 1 or port > 65535:
        raise ValueError(f'{label} must be in [1, 65535]')
    return port
