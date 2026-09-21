import threading
import time
from types import SimpleNamespace

from rby1_control.manipulator_controller import ManipulatorController
from rby1_control.power_servo_state_adaptor import PowerServoStateAdaptor
from rby1_control.state_manager import StateManager


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


def test_state_manager_combines_driver_and_sdk_feedback():
    clock = Clock()
    manager = StateManager(clock=clock)
    manager.update_driver_state(
        control_state=2,
        stream_enabled=True,
        emo_active=False,
        collision_active=False,
    )
    manager.update_power_servo(True, True)

    status = manager.snapshot()
    assert status.control_state == 2
    assert status.stream_enabled is True
    assert status.power_enabled is True
    assert status.servo_enabled is True
    assert manager.safety_reason(maximum_age_sec=1.0) is None

    clock.now += 1.1
    assert manager.safety_reason(maximum_age_sec=1.0) == (
        'robot state is unavailable or stale'
    )


def test_manipulator_controller_orders_joint_feedback_by_name():
    controller = ManipulatorController(clock=lambda: 5.0)
    message = SimpleNamespace(
        name=['right_arm_1', 'right_arm_0'] + [
            f'right_arm_{index}' for index in range(2, 7)
        ],
        position=[0.2, 0.1, 0.3, 0.4, 0.5, 0.6, 0.7],
    )

    assert controller.update_joint_state('right_arm', message)
    assert controller.joint_order_verified['right_arm'] is True
    assert controller.joint_groups_deg['right_arm'][0] < (
        controller.joint_groups_deg['right_arm'][1]
    )
    assert controller.joint_updated_at['right_arm'] == 5.0


def test_power_servo_adaptor_never_blocks_polling_thread():
    release = threading.Event()
    received = []
    errors = []

    class Robot:
        def connect(self):
            release.wait(timeout=1.0)
            return True

        def is_power_on(self, _pattern):
            return True

        def is_servo_on(self, _pattern):
            return False

    adaptor = PowerServoStateAdaptor(
        address='robot:50051',
        model='m',
        poll_period_sec=0.01,
        on_state=lambda power, servo: received.append((power, servo)),
        on_error=errors.append,
        robot_factory=lambda _address, _model: Robot(),
    )
    started = time.monotonic()
    adaptor.poll()
    assert time.monotonic() - started < 0.05

    release.set()
    deadline = time.monotonic() + 1.0
    while not received and time.monotonic() < deadline:
        adaptor.poll()
        time.sleep(0.001)
    adaptor.close()

    assert received == [(True, False)]
    assert errors == []
