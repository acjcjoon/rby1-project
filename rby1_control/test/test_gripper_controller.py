import pytest

from rby1_control.gripper_controller import (
    GripperCommandError,
    GripperController,
)


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


def ready_controller(clock):
    controller = GripperController(clock=clock)
    controller.update_ready(True)
    controller.update_state((0.25, 0.75))
    return controller


def test_requires_ready_driver_and_fresh_feedback():
    clock = Clock()
    controller = GripperController(clock=clock, state_timeout_sec=1.0)

    with pytest.raises(GripperCommandError, match='not ready'):
        controller.command('open')

    controller.update_ready(True)
    controller.update_state((0.0, 0.0))
    clock.now += 1.1
    with pytest.raises(GripperCommandError, match='stale'):
        controller.command('close')


def test_left_side_presets_use_normalized_open_close_ratios():
    controller = ready_controller(Clock())
    assert controller.command('open', 'left') == (0.25, 0.0)
    assert controller.command('close', 'left') == (0.25, 1.0)


def test_right_and_both_presets_use_normalized_open_close_ratios():
    controller = ready_controller(Clock())
    assert controller.command('close', 'right') == (1.0, 0.75)
    assert controller.command('open', 'both') == (0.0, 0.0)


def test_feedback_marks_command_complete_within_tolerance():
    controller = ready_controller(Clock())
    controller.command('close')
    assert controller.status().motion_active is True
    assert controller.update_state((0.0, 0.97)) is False
    assert controller.update_state((0.97, 0.97)) is True
    assert controller.status().motion_active is False


def test_arbitrary_ratio_command_preserves_other_side():
    controller = ready_controller(Clock())
    assert controller.command_ratio(0.6, "left") == (0.25, 0.6)
    assert controller.command_ratio(0.4, "right") == (0.4, 0.6)
    assert controller.command_ratio(0.2, "both") == (0.2, 0.2)


def test_settle_completion_requires_new_fresh_feedback_and_keeps_target():
    clock = Clock()
    controller = ready_controller(clock)
    marker = controller.status().feedback_sequence
    target = controller.command("close", "left")

    controller.update_state((0.25, 0.55))
    controller.complete_after_settle(marker)

    status = controller.status()
    assert status.motion_active is False
    assert status.target == target
    assert status.error is None


def test_settle_completion_rejects_missing_post_command_feedback():
    clock = Clock()
    controller = ready_controller(clock)
    marker = controller.status().feedback_sequence
    controller.command("close", "left")

    with pytest.raises(GripperCommandError, match="no new"):
        controller.complete_after_settle(marker)


def test_cancel_holds_the_latest_fresh_position():
    clock = Clock()
    controller = ready_controller(clock)
    controller.command("close", "left")
    controller.update_state((0.25, 0.55))

    assert controller.cancel_and_hold() == (0.25, 0.55)
    status = controller.status()
    assert status.target == (0.25, 0.55)
    assert status.motion_active is False


def test_timeout_is_reported_once():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=2.0,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))
    controller.command('close')

    clock.now += 2.1
    assert controller.tick() == (
        'gripper command timed out before reaching its target'
    )
    assert controller.tick() is None
    assert controller.status().motion_active is False


def test_stale_feedback_stops_active_tracking():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=1.0,
        command_timeout_sec=5.0,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))
    controller.command('close')

    clock.now += 1.1
    assert controller.tick() == 'gripper state became stale during a command'
    assert controller.status().motion_active is False


@pytest.mark.parametrize('side', ['port', '', 'BOTH_HANDS'])
def test_rejects_unknown_side(side):
    controller = ready_controller(Clock())
    with pytest.raises(GripperCommandError, match='side'):
        controller.command('open', side)
