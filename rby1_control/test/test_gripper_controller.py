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
    open_controller = ready_controller(Clock())
    close_controller = ready_controller(Clock())
    assert open_controller.command('open', 'left') == (0.25, 0.0)
    assert close_controller.command('close', 'left') == (0.25, 1.0)


def test_right_and_both_presets_use_normalized_open_close_ratios():
    right_controller = ready_controller(Clock())
    both_controller = ready_controller(Clock())
    assert right_controller.command('close', 'right') == (1.0, 0.75)
    assert both_controller.command('open', 'both') == (0.0, 0.0)


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

    both_controller = ready_controller(Clock())
    assert both_controller.command_ratio(0.2, "both") == (0.2, 0.2)


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


def test_trapezoidal_profile_accelerates_cruises_and_decelerates():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=10.0,
        default_speed_ratio_per_sec=0.5,
        max_speed_ratio_per_sec=1.0,
        acceleration_ratio_per_sec2=1.0,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))

    assert controller.command('close', 'right') == (1.0, 0.0)
    assert controller.take_command_target() == (0.0, 0.0)
    status = controller.status()
    assert status.profile_active is True
    assert status.reference_velocity == (0.0, 0.0)
    assert status.planned_duration_sec == pytest.approx(2.5)

    clock.now += 0.5
    assert controller.tick() is None
    assert controller.take_command_target() == pytest.approx((0.125, 0.0))
    assert controller.status().reference_velocity == pytest.approx((0.5, 0.0))

    clock.now += 1.0
    controller.tick()
    assert controller.take_command_target() == pytest.approx((0.625, 0.0))
    assert controller.status().reference_velocity == pytest.approx((0.5, 0.0))

    clock.now += 1.0
    controller.tick()
    assert controller.take_command_target() == (1.0, 0.0)
    status = controller.status()
    assert status.profile_active is False
    assert status.reference_velocity == (0.0, 0.0)
    assert status.profile_completed_at == pytest.approx(clock.now)


def test_short_move_uses_triangular_profile_and_does_not_overshoot():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=10.0,
        default_speed_ratio_per_sec=1.0,
        max_speed_ratio_per_sec=1.0,
        acceleration_ratio_per_sec2=1.0,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))

    controller.command_ratio(0.25, 'right')
    assert controller.status().planned_duration_sec == pytest.approx(1.0)
    controller.take_command_target()

    clock.now += 0.5
    controller.tick()
    assert controller.take_command_target() == pytest.approx((0.125, 0.0))
    assert controller.status().reference_velocity == pytest.approx((0.5, 0.0))

    # A delayed callback samples only "now" and clamps exactly to the goal.
    clock.now += 2.0
    controller.tick()
    assert controller.take_command_target() == (0.25, 0.0)
    assert controller.take_command_target() is None
    assert controller.status().profile_active is False


def test_each_side_keeps_an_independent_profile():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=10.0,
        max_speed_ratio_per_sec=1.0,
        acceleration_ratio_per_sec2=1.0,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))

    controller.command('close', 'left', 0.5)
    controller.take_command_target()
    clock.now += 0.5
    controller.tick()
    assert controller.take_command_target() == pytest.approx((0.0, 0.125))

    assert controller.command_ratio(0.5, 'right', 1.0) == (0.5, 1.0)
    # Starting right does not reset the in-flight left reference or velocity.
    assert controller.take_command_target() == pytest.approx((0.0, 0.125))
    assert controller.status().reference_velocity == pytest.approx((0.0, 0.5))

    clock.now += 0.25
    controller.tick()
    assert controller.take_command_target() == pytest.approx((0.03125, 0.25))
    assert controller.status().reference_velocity == pytest.approx((0.25, 0.5))


def test_rejects_recommanding_an_active_selected_side_after_advancing():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=10.0,
        default_speed_ratio_per_sec=0.5,
        acceleration_ratio_per_sec2=1.0,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))
    controller.command('close', 'left')
    controller.take_command_target()

    clock.now += 0.5
    with pytest.raises(GripperCommandError, match='active trajectory: left'):
        controller.command('open', 'left')

    # The rejected request still advances the existing profile to "now".
    assert controller.take_command_target() == pytest.approx((0.0, 0.125))
    assert controller.status().target == (0.0, 1.0)


def test_active_side_does_not_block_a_command_for_the_opposite_idle_side():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=10.0,
    )
    controller.update_ready(True)
    controller.update_state((0.1, 0.2))
    controller.command('close', 'left')

    assert controller.command_ratio(0.6, 'right') == (0.6, 1.0)
    assert controller.status().profile_active is True


def test_recommand_is_accepted_when_profile_ends_exactly_now():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=10.0,
        default_speed_ratio_per_sec=1.0,
        max_speed_ratio_per_sec=1.0,
        acceleration_ratio_per_sec2=1.0,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))
    controller.command_ratio(0.25, 'right')

    clock.now += 1.0
    assert controller.command_ratio(0.5, 'right') == (0.5, 0.0)
    assert controller.status().profile_active is True


def test_tick_retains_only_the_latest_sample_without_backlog():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=10.0,
        default_speed_ratio_per_sec=0.5,
        acceleration_ratio_per_sec2=1.0,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))
    controller.command('close', 'right')
    controller.take_command_target()

    clock.now += 0.5
    controller.tick()
    clock.now += 1.0
    controller.tick()

    assert controller.take_command_target() == pytest.approx((0.625, 0.0))
    assert controller.take_command_target() is None


def test_cancel_clears_profiles_and_publishes_fresh_measured_hold():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=10.0,
    )
    controller.update_ready(True)
    controller.update_state((0.2, 0.3))
    controller.command('close')
    controller.take_command_target()
    clock.now += 0.25
    controller.tick()
    controller.update_state((0.24, 0.36))

    assert controller.cancel_and_hold() == (0.24, 0.36)
    assert controller.take_command_target() == (0.24, 0.36)
    status = controller.status()
    assert status.target == (0.24, 0.36)
    assert status.command_reference == (0.24, 0.36)
    assert status.reference_velocity == (0.0, 0.0)
    assert status.profile_active is False
    assert status.motion_active is False


def test_stale_cancel_clears_targets_and_rebases_recovered_single_side():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=1.0,
        command_timeout_sec=10.0,
    )
    controller.update_ready(True)
    controller.update_state((0.2, 0.3))
    controller.command('close')
    controller.take_command_target()

    clock.now += 1.1
    assert controller.cancel_and_hold() is None
    status = controller.status()
    assert status.target is None
    assert status.command_reference is None
    assert status.sample_pending is False
    assert controller.take_command_target() is None

    controller.update_state((0.4, 0.5))
    assert controller.command_ratio(0.8, 'left') == (0.4, 0.8)
    assert controller.take_command_target() == (0.4, 0.5)


def test_stale_failure_clears_targets_and_rebases_recovered_single_side():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=1.0,
        command_timeout_sec=10.0,
    )
    controller.update_ready(True)
    controller.update_state((0.2, 0.3))
    controller.command('close')
    controller.take_command_target()

    clock.now += 1.1
    assert controller.tick() == 'gripper state became stale during a command'
    status = controller.status()
    assert status.target is None
    assert status.command_reference is None
    assert status.sample_pending is False
    assert controller.take_command_target() is None

    controller.update_state((0.4, 0.5))
    assert controller.command_ratio(0.8, 'left') == (0.4, 0.8)
    assert controller.take_command_target() == (0.4, 0.5)


def test_failure_with_fresh_feedback_publishes_one_measured_hold_sample():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=10.0,
        command_timeout_sec=1.0,
    )
    controller.update_ready(True)
    controller.update_state((0.2, 0.3))
    controller.command('close')
    controller.take_command_target()

    clock.now += 1.1
    assert controller.tick() == (
        'gripper command timed out before reaching its target'
    )
    status = controller.status()
    assert status.target == (0.2, 0.3)
    assert status.command_reference == (0.2, 0.3)
    assert status.sample_pending is True
    assert controller.take_command_target() == (0.2, 0.3)
    assert controller.take_command_target() is None


@pytest.mark.parametrize(
    'speed',
    [0.0, -0.1, float('nan'), float('inf'), True, 1.01, 'fast'],
)
def test_rejects_invalid_command_speed(speed):
    controller = ready_controller(Clock())
    with pytest.raises(GripperCommandError, match='speed'):
        controller.command('close', speed=speed)


def test_constructor_validates_speed_and_acceleration_limits():
    with pytest.raises(ValueError, match='default_speed'):
        GripperController(
            default_speed_ratio_per_sec=1.1,
            max_speed_ratio_per_sec=1.0,
        )
    with pytest.raises(ValueError, match='acceleration'):
        GripperController(acceleration_ratio_per_sec2=0.0)


def test_dynamic_deadline_includes_profile_duration_and_margin():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=20.0,
        command_timeout_sec=5.0,
        default_speed_ratio_per_sec=0.1,
        max_speed_ratio_per_sec=1.0,
        acceleration_ratio_per_sec2=1.0,
        tracking_timeout_margin_sec=2.0,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))
    controller.command('close', 'right')

    status = controller.status()
    assert status.planned_duration_sec == pytest.approx(10.1)
    assert status.command_deadline == pytest.approx(clock.now + 12.1)


def test_begin_settle_extends_deadline_but_keeps_stale_checks_active():
    clock = Clock()
    controller = GripperController(
        clock=clock,
        state_timeout_sec=2.0,
        command_timeout_sec=1.0,
        default_speed_ratio_per_sec=1.0,
        max_speed_ratio_per_sec=1.0,
        acceleration_ratio_per_sec2=1.0,
        tracking_timeout_margin_sec=0.5,
    )
    controller.update_ready(True)
    controller.update_state((0.0, 0.0))
    controller.command_ratio(0.01, 'right')
    controller.take_command_target()

    clock.now += 0.2
    controller.tick()
    assert controller.status().profile_active is False
    deadline = controller.begin_settle(3.0)
    assert deadline == pytest.approx(clock.now + 3.5)
    assert controller.status().motion_active is True

    # Extending the position deadline does not suppress stale-state safety.
    clock.now += 2.1
    assert controller.tick() == 'gripper state became stale during a command'


def test_begin_settle_rejects_an_active_profile():
    controller = ready_controller(Clock())
    controller.command('close', 'right')

    with pytest.raises(GripperCommandError, match='endpoint'):
        controller.begin_settle(1.0)
