"""Exercise real adapter callbacks with ROS interface doubles, no hardware."""
import importlib.util
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


def load_adapter():
    symbols = {
        'ament_index_python.packages': ['get_package_share_directory'],
        'geometry_msgs.msg': ['PoseStamped'], 'nav2_msgs.action': ['NavigateToPose'],
        'nav_msgs.msg': ['OccupancyGrid', 'Path'], 'rclpy.action': ['ActionClient'],
        'rclpy.duration': ['Duration'],
        'rclpy.qos': ['DurabilityPolicy', 'QoSProfile', 'ReliabilityPolicy'],
        'rclpy.time': ['Time'], 'std_msgs.msg': ['String'],
        'std_srvs.srv': ['SetBool', 'Trigger'],
        'tf2_ros': ['Buffer', 'TransformException', 'TransformListener'],
        'rby1_vslam.waypoint_store': ['Waypoint', 'default_waypoints_path',
                                     'load_waypoints', 'next_name', 'save_waypoints'],
    }
    modules = {}
    for name, attrs in symbols.items():
        parts = name.split('.')
        for i in range(1, len(parts)+1):
            key = '.'.join(parts[:i])
            modules.setdefault(key, ModuleType(key))
        for attr in attrs:
            setattr(modules[name], attr, Mock())
    modules['tf2_ros'].TransformException = RuntimeError
    path = Path(__file__).resolve().parents[1] / 'rby1_web/vslam_adapter.py'
    spec = importlib.util.spec_from_file_location('adapter_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.VslamAdapter


Adapter = load_adapter()


class NavigationLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.adapter = Adapter.__new__(Adapter)
        self.adapter.node = Mock()
        self.adapter.box = Mock()
        self.adapter.cancel_client = Mock()
        self.adapter.cancel_client.service_is_ready.return_value = True
        self.adapter.enable_client = Mock()
        self.adapter.enable_client.service_is_ready.return_value = False
        self.adapter.action = Mock()
        self.adapter.goal_handle = None
        self.adapter.active = False
        self.adapter.generation = 3
        self.adapter.gate_received = None
        self.adapter.gate = {}
        self.adapter.last_cancel = -float('inf')

    def test_manual_without_nav2_needs_no_tcp(self):
        self.assertTrue(self.adapter.manual_ready())
        self.adapter.enable_client.service_is_ready.assert_called_once()

    def test_manual_waits_for_gate_to_relinquish_velocity(self):
        self.adapter.gate_received = time.monotonic()
        self.adapter.gate = {'enabled': True, 'cancel_pending': False}
        self.assertFalse(self.adapter.manual_ready())
        self.adapter.cancel_client.call_async.assert_called_once()
        self.adapter.gate = {'enabled': False, 'cancel_pending': True}
        self.assertFalse(self.adapter.manual_ready())
        self.adapter.gate['cancel_pending'] = False
        self.assertTrue(self.adapter.manual_ready())

    def test_unknown_stale_gate_does_not_race_manual_commands(self):
        self.adapter.gate_received = time.monotonic() - 2
        self.adapter.gate = {'enabled': False, 'cancel_pending': False}
        self.assertFalse(self.adapter.manual_ready())

    def test_stop_invalidates_pending_enable_and_late_goal_acceptance(self):
        self.adapter.active = True
        self.adapter.cancel()
        self.assertEqual(self.adapter.generation, 4)
        enabled = Mock()
        self.adapter.after_enable(enabled, Mock(), 3)
        enabled.result.assert_not_called()
        self.adapter.action.send_goal_async.assert_not_called()
        accepted = Mock()
        accepted.result.return_value.accepted = True
        self.adapter.on_goal(accepted, 3)
        accepted.result.return_value.cancel_goal_async.assert_called_once()

    def test_old_results_do_not_finish_a_new_goal(self):
        self.adapter.active = True
        old = Mock()
        self.adapter.on_result(old, 2)
        old.result.assert_not_called()
        self.assertTrue(self.adapter.active)

    def test_rejected_enable_never_sends_a_goal(self):
        self.adapter.active = True
        rejected = Mock()
        rejected.result.return_value = SimpleNamespace(success=False, message='tracking unavailable')
        self.adapter.after_enable(rejected, Mock(), 3)
        self.adapter.action.send_goal_async.assert_not_called()
        self.assertFalse(self.adapter.active)

    def test_successful_enable_sends_and_accepts_the_existing_nav2_goal(self):
        self.adapter.active = True
        self.adapter.p = {'map_frame': 'vslam_map'}
        self.adapter.state = {'state': 'enabling'}
        enabled = Mock()
        enabled.result.return_value = SimpleNamespace(success=True)
        target = SimpleNamespace(name='Point 1', x=1.5, y=-0.5, yaw=0.2)
        self.adapter.after_enable(enabled, target, 3)
        self.adapter.action.send_goal_async.assert_called_once()
        goal = self.adapter.action.send_goal_async.call_args.args[0]
        self.assertEqual(goal.pose.header.frame_id, 'vslam_map')
        self.assertEqual(goal.pose.pose.position.x, 1.5)
        accepted = Mock()
        accepted.result.return_value.accepted = True
        self.adapter.on_goal(accepted, 3)
        self.assertEqual(self.adapter.state['state'], 'navigating')
        accepted.result.return_value.get_result_async.assert_called_once()


if __name__ == '__main__':
    unittest.main()
