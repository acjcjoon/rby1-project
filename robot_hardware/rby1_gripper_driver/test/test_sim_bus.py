import socket
import struct
import time

import pytest

from rby1_gripper_driver.sdk_bus import (
    closed_at_minimum_for_transport,
    create_bus,
    motor_ids_for_transport,
)
from rby1_gripper_driver.sim_bus import (
    SIM_HOME_MAX_RAD,
    SIM_HOME_MIN_RAD,
    SimDynamixelBus,
)


COMMAND_FORMAT = '<HBBff'
STATE_FORMAT = '<HHfff'


def _receive_latest(receiver, count):
    packet = None
    for _index in range(count):
        packet, _address = receiver.recvfrom(64)
    assert packet is not None
    return struct.unpack(COMMAND_FORMAT, packet)


def test_sim_bus_emits_isaac_packet_and_preserves_torque_disabled_side():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.settimeout(1.0)
    command_port = receiver.getsockname()[1]
    bus = SimDynamixelBus(
        command_port=command_port,
        state_port=0,
    )
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        assert bus.open_port()
        feedback = struct.pack(STATE_FORMAT, 0x4753, 0, 0.0, 0.75, 1.0)
        sender.sendto(feedback, ('127.0.0.1', bus.state_port))
        deadline = time.monotonic() + 1.0
        while not bus.feedback_received and time.monotonic() < deadline:
            time.sleep(0.005)

        bus.group_sync_write_send_position([(0, SIM_HOME_MIN_RAD)])
        bus.group_sync_write_torque_enable([0], 1)

        magic, flags, reserved, left, right = _receive_latest(receiver, 2)
        assert magic == 0x4743
        assert reserved == 0
        assert flags & 0x1
        assert flags & 0x2
        assert not flags & 0x4
        assert left == pytest.approx(1.0)
        # ID 1 is disabled, so its latest measured position is retained.
        assert right == pytest.approx(0.75)
    finally:
        sender.close()
        bus.close_port()
        receiver.close()


def test_sim_bus_decodes_left_and_right_feedback_in_sdk_id_order():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    command_port = receiver.getsockname()[1]
    bus = SimDynamixelBus(
        command_port=command_port,
        state_port=0,
    )
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        assert bus.open_port()
        packet = struct.pack(STATE_FORMAT, 0x4753, 0, 0.25, 0.75, 1.0)
        sender.sendto(packet, ('127.0.0.1', bus.state_port))

        deadline = time.monotonic() + 1.0
        while not bus.feedback_received and time.monotonic() < deadline:
            time.sleep(0.005)
        states = bus.get_motor_states([0, 1])

        assert states is not None
        by_id = dict(states)
        span = SIM_HOME_MAX_RAD - SIM_HOME_MIN_RAD
        assert by_id[0].position == pytest.approx(
            SIM_HOME_MAX_RAD - 0.25 * span
        )
        assert by_id[1].position == pytest.approx(
            SIM_HOME_MAX_RAD - 0.75 * span
        )
    finally:
        sender.close()
        bus.close_port()
        receiver.close()


def test_sim_bus_rejects_stale_feedback():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    command_port = receiver.getsockname()[1]
    bus = SimDynamixelBus(
        command_port=command_port,
        state_port=0,
        feedback_timeout_sec=0.02,
    )
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        assert bus.open_port()
        packet = struct.pack(STATE_FORMAT, 0x4753, 0, 0.0, 0.0, 1.0)
        sender.sendto(packet, ('127.0.0.1', bus.state_port))

        deadline = time.monotonic() + 1.0
        while not bus.feedback_received and time.monotonic() < deadline:
            time.sleep(0.005)
        assert bus.feedback_fresh
        assert bus.get_motor_states([0]) is not None

        time.sleep(0.03)
        assert not bus.feedback_fresh
        assert bus.get_motor_states([0]) is None
    finally:
        sender.close()
        bus.close_port()
        receiver.close()


def test_sim_transport_factory_does_not_require_vendor_sdk():
    selection = create_bus(
        'SIM',
        sim_host='127.0.0.1',
        sim_command_port=5007,
        sim_state_port=5008,
    )

    assert selection.transport == 'sim'
    assert isinstance(selection.bus, SimDynamixelBus)
    assert selection.driver_name == 'IsaacSimGripperUDP'
    assert selection.hardware_id == 'udp://127.0.0.1:5007?state_port=5008'


def test_transport_factory_rejects_unknown_transport():
    with pytest.raises(ValueError, match="'real' or 'sim'"):
        create_bus('mock')


def test_transport_motor_ids_keep_physical_and_isaac_conventions_separate():
    assert motor_ids_for_transport('real', (0, 1)) == (0, 1)
    assert motor_ids_for_transport('sim', (0, 1)) == (1, 0)


def test_physical_closed_direction_does_not_reverse_isaac_commands():
    assert closed_at_minimum_for_transport(
        'real', (True, False)
    ) == (True, False)
    assert closed_at_minimum_for_transport(
        'sim', (True, False)
    ) == (True, True)
