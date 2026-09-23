"""Failure-path coverage for the UPC Nav2 gate, runnable without ROS."""

import math

import pytest

from rby1_vslam.nav2_gate_core import CancelEndpoint, GateLimits, VelocityGate


def healthy(gate, now=10.0, ros_now=1000.0, session='session-a', pose=(0.0, 0.0, 0.0)):
    gate.update_bridge(True, True, session, now)
    gate.update_localization(True, session, now)
    gate.update_wheel(ros_now, now, ros_now)
    gate.update_slam(pose, 'vslam_map', ros_now, now, ros_now)


def armed():
    gate = VelocityGate()
    healthy(gate)
    gate.cancellation_complete(gate.cancel_serial)
    assert gate.enable(10.0, 1000.0)[0]
    return gate


def test_startup_waits_for_cancel_barrier_and_fresh_command():
    gate = VelocityGate()
    healthy(gate)
    assert not gate.update_command((0.1, 0.0, 0.0), 9.99)
    assert not gate.enable(10.0, 1000.0)[0]
    gate.cancellation_complete(gate.cancel_serial)
    assert gate.enable(10.0, 1000.0)[0]
    assert gate.tick(10.01, 1000.01) == (0.0, 0.0, 0.0)
    assert not gate.update_command((0.1, 0.0, 0.0), 10.0)
    assert gate.update_command((0.1, 0.0, 0.0), 10.01)
    assert gate.tick(10.02, 1000.02) == (0.1, 0.0, 0.0)


def test_limits_cap_vector_norm_and_rotation():
    gate = armed()
    gate.update_command((1.0, 1.0, -2.0), 10.01)
    vx, vy, wz = gate.tick(10.02, 1000.02)
    assert math.hypot(vx, vy) == pytest.approx(0.15)
    assert vx == pytest.approx(vy)
    assert wz == -0.35


def test_command_timeout_stops_without_disarming_normal_nav2_idle():
    gate = armed()
    gate.update_command((0.1, 0.0, 0.0), 10.01)
    assert gate.tick(10.27, 1000.27) == (0.0, 0.0, 0.0)
    assert gate.enabled
    gate.update_command((0.05, 0.0, 0.0), 10.28)
    assert gate.tick(10.29, 1000.29) == (0.05, 0.0, 0.0)


@pytest.mark.parametrize('fault', ['tracking', 'disconnect', 'session', 'correction', 'publisher'])
def test_faults_latch_and_recovery_cannot_auto_resume(fault):
    gate = armed()
    gate.update_command((0.1, 0.0, 0.0), 10.01)
    if fault == 'tracking':
        gate.update_bridge(True, False, 'session-a', 10.02)
    elif fault == 'disconnect':
        gate.update_bridge(False, False, 'session-a', 10.02)
    elif fault == 'session':
        gate.update_bridge(True, True, 'session-b', 10.02)
    elif fault == 'correction':
        gate.update_localization(False, 'session-a', 10.02)
    else:
        gate.set_competing_publisher(True)
    assert not gate.enabled
    assert gate.cancel_pending
    gate.set_competing_publisher(False)
    healthy(gate, 10.1, 1000.1, session='session-b')
    assert gate.tick(10.11, 1000.11) == (0.0, 0.0, 0.0)
    assert not gate.enable(10.11, 1000.11)[0]
    gate.cancellation_complete(gate.cancel_serial)
    assert gate.enable(10.12, 1000.12)[0]
    assert gate.tick(10.13, 1000.13) == (0.0, 0.0, 0.0)


def test_localization_status_must_match_transport_session():
    gate = VelocityGate()
    healthy(gate)
    gate.cancellation_complete(gate.cancel_serial)
    gate.update_localization(True, 'previous-session', 10.01)
    assert not gate.enable(10.02, 1000.02)[0]


@pytest.mark.parametrize('source', ['wheel', 'slam'])
def test_fresh_receipt_does_not_hide_old_or_future_source_stamp(source):
    gate = armed()
    if source == 'wheel':
        gate.update_wheel(999.0, 10.01, 1000.01)
    else:
        gate.update_slam((0, 0, 0), 'vslam_map', 1001.0, 10.01, 1000.01)
    assert not gate.enabled
    assert gate.cancel_pending


@pytest.mark.parametrize('source', ['wheel', 'slam'])
def test_replayed_samples_do_not_refresh_watchdog(source):
    gate = armed()
    if source == 'wheel':
        assert not gate.update_wheel(1000.0, 10.4, 1000.4)
        assert gate.wheel[1] == 10.0
    else:
        assert not gate.update_slam((0, 0, 0), 'vslam_map', 1000.0, 10.4, 1000.4)
        assert gate.slam[3] == 10.0


def test_monotonic_watchdog_stops_when_simulation_clock_pauses():
    gate = armed()
    gate.update_command((0.1, 0, 0), 10.01)
    assert gate.tick(10.51, 1000.0) == (0.0, 0.0, 0.0)
    assert not gate.enabled
    assert gate.cancel_pending


def test_ros_clock_rewind_latches_and_invalidates_cached_poses():
    gate = armed()
    gate.tick(10.01, 1000.01)
    gate.tick(10.02, 999.0)
    assert not gate.enabled
    assert gate.wheel is None and gate.slam is None


@pytest.mark.parametrize('pose,frame', [((0.6, 0, 0), 'vslam_map'),
                                       ((0, 0, 0.9), 'vslam_map'),
                                       ((0, 0, 0), 'another-map')])
def test_map_jump_latches_even_with_fresh_data(pose, frame):
    gate = armed()
    gate.update_slam(pose, frame, 1000.01, 10.01, 1000.01)
    assert not gate.enabled
    assert gate.cancel_pending


def test_yaw_wrap_is_not_a_pose_jump():
    gate = VelocityGate()
    healthy(gate, pose=(0, 0, math.pi - 0.01))
    gate.cancellation_complete(gate.cancel_serial)
    assert gate.enable(10, 1000)[0]
    gate.update_slam((0, 0, -math.pi + 0.01), 'vslam_map', 1000.01, 10.01, 1000.01)
    assert gate.enabled


def test_invalid_command_latches_and_late_cancellation_cannot_unlock_new_fault():
    gate = armed()
    serial = gate.cancel_serial
    gate.update_command((math.nan, 0, 0), 10.01)
    assert not gate.enabled
    gate.cancellation_complete(serial)
    assert gate.cancel_pending


def test_cancel_acceptance_waits_for_terminal_action_status():
    endpoint = CancelEndpoint()
    goal = b'goal-id'
    endpoint.status({goal: 2})
    assert endpoint.response(0, [goal])
    assert not endpoint.complete
    endpoint.status({goal: 3})
    assert not endpoint.complete
    endpoint.status({goal: 5})
    assert endpoint.complete


def test_missing_status_rejected_cancel_or_another_active_goal_block_rearm():
    endpoint = CancelEndpoint()
    assert not endpoint.response(1, [])
    assert not endpoint.complete
    endpoint.response(0, [b'goal-1'])
    assert not endpoint.complete
    endpoint.status({b'goal-1': 5, b'goal-2': 1})
    assert not endpoint.complete


def test_no_goal_cancel_reply_completes_without_inventing_an_active_goal():
    endpoint = CancelEndpoint()
    endpoint.response(0, [])
    assert endpoint.complete


def test_cancel_terminal_evidence_survives_goal_expiry_but_not_unknown_disappearance():
    endpoint = CancelEndpoint()
    endpoint.response(0, [b'goal'])
    endpoint.status({b'goal': 3})
    endpoint.status({})
    assert not endpoint.complete  # disappearance is not proof of cancellation
    endpoint.status({b'goal': 5})
    endpoint.status({})
    assert endpoint.complete


def test_duplicate_enable_cannot_bypass_a_new_sensor_fault():
    gate = armed()
    assert not gate.enable(11.0, 1001.0)[0]
    assert not gate.enabled
    assert gate.cancel_pending


@pytest.mark.parametrize('value', [0, -1, math.inf, math.nan])
def test_invalid_limit_rejected(value):
    with pytest.raises(ValueError):
        GateLimits(cmd_timeout_sec=value)
