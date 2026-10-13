"""Regression scenarios for local tracking health and Nav2 stop/rearm rules."""

import unittest

from rby1_vslam.nav2_gate_core import CancelEndpoint, VelocityGate
from rby1_vslam.tracking_state import TrackingState


class TrackingTests(unittest.TestCase):
    def healthy_tracker(self, require_localized=False):
        tracker = TrackingState(require_localized=require_localized)
        tracker.observe_status(1, 100.0, 10.0)
        tracker.observe_odometry(100.0, 10.0)
        return tracker

    def test_status_alone_never_releases_navigation(self):
        tracker = TrackingState()
        tracker.observe_status(1, 100.0, 10.0)
        self.assertFalse(tracker.report(10.1, 100.1)['tracking_ok'])
        tracker.observe_odometry(100.0, 10.0)
        self.assertTrue(tracker.report(10.1, 100.1)['tracking_ok'])

    def test_loss_and_sensor_silence_stop_tracking(self):
        tracker = self.healthy_tracker()
        self.assertFalse(tracker.report(10.6, 100.6)['tracking_ok'])
        tracker.observe_status(2, 100.1, 10.1)
        self.assertFalse(tracker.report(10.1, 100.1)['tracking_ok'])

    def test_future_source_clock_is_rejected(self):
        tracker = self.healthy_tracker()
        self.assertFalse(tracker.report(10.0, 99.0)['tracking_ok'])

    def test_duplicate_samples_cannot_refresh_watchdog(self):
        tracker = self.healthy_tracker()
        self.assertFalse(tracker.observe_status(1, 100.0, 10.4))
        self.assertFalse(tracker.observe_odometry(100.0, 10.4))
        self.assertFalse(tracker.report(10.6, 100.6)['tracking_ok'])

    def test_localization_must_be_confirmed_and_fresh(self):
        tracker = self.healthy_tracker(require_localized=True)
        self.assertFalse(tracker.report(10.0, 100.0)['tracking_ok'])
        tracker.observe_localization(True, 100.0, 10.0)
        self.assertTrue(tracker.report(10.0, 100.0)['tracking_ok'])
        tracker.observe_status(1, 100.6, 10.6)
        tracker.observe_odometry(100.6, 10.6)
        self.assertFalse(tracker.report(10.6, 100.6)['tracking_ok'])

    def test_restarting_tracker_invalidates_old_pose_and_localization(self):
        tracker = self.healthy_tracker(require_localized=True)
        tracker.observe_localization(True, 100.0, 10.0)
        old = tracker.session
        tracker.new_session()
        self.assertNotEqual(tracker.session, old)
        self.assertFalse(tracker.report(10.0, 100.0)['tracking_ok'])

    def test_backwards_sensor_time_creates_a_new_epoch(self):
        tracker = self.healthy_tracker()
        old = tracker.session
        tracker.observe_status(1, 99.0, 10.1)
        self.assertNotEqual(tracker.session, old)
        self.assertIsNone(tracker.odom)
        self.assertFalse(tracker.report(10.1, 99.0)['tracking_ok'])


class GateTests(unittest.TestCase):
    def healthy_gate(self):
        gate = VelocityGate()
        gate.cancellation_complete(gate.cancel_serial)
        gate.update_tracking(True, True, 'local-1', 10.0)
        gate.update_localization(True, 'local-1', 10.0)
        gate.update_wheel(100.0, 10.0, 100.0)
        gate.update_slam((0.0, 0.0, 0.0), 'vslam_map', 100.0, 10.0, 100.0)
        return gate

    def armed_gate(self):
        gate = self.healthy_gate()
        self.assertTrue(gate.enable(10.0, 100.0)[0])
        gate.update_command((0.1, 0.0, 0.1), 10.01)
        self.assertNotEqual(gate.tick(10.02, 100.02), (0.0, 0.0, 0.0))
        return gate

    def test_fault_does_not_auto_resume_after_recovery(self):
        gate = self.armed_gate()
        gate.update_tracking(True, False, 'local-1', 10.03)
        self.assertEqual(gate.tick(10.03, 100.03), (0.0, 0.0, 0.0))
        gate.update_tracking(True, True, 'local-1', 10.04)
        self.assertFalse(gate.enabled)
        self.assertTrue(gate.cancel_pending)
        self.assertFalse(gate.enable(10.04, 100.04)[0])

    def test_component_restart_clears_cached_pose_and_command(self):
        gate = self.armed_gate()
        gate.update_tracking(True, True, 'local-2', 10.03)
        self.assertIsNone(gate.slam)
        self.assertIsNone(gate.command)
        self.assertFalse(gate.localization_ok)
        self.assertFalse(gate.enabled)

    def test_watchdog_stops_on_wheel_or_tracking_expiry(self):
        gate = self.armed_gate()
        self.assertEqual(gate.tick(10.8, 100.8), (0.0, 0.0, 0.0))
        self.assertFalse(gate.enabled)

    def test_cancel_acknowledgement_waits_for_terminal_goal(self):
        endpoint = CancelEndpoint()
        endpoint.status({'goal': 2})
        endpoint.response(0, ['goal'])
        self.assertFalse(endpoint.complete)
        endpoint.status({'goal': 3})
        self.assertFalse(endpoint.complete)
        endpoint.status({'goal': 5})
        self.assertTrue(endpoint.complete)

    def test_old_commands_stay_discarded_after_enable(self):
        gate = self.healthy_gate()
        gate.update_command((0.1, 0.0, 0.0), 9.9)
        self.assertTrue(gate.enable(10.0, 100.0)[0])
        self.assertEqual(gate.tick(10.01, 100.01), (0.0, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
