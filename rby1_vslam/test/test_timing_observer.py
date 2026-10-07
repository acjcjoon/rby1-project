"""ROS-free checks for the passive capture's payload and timestamp handling."""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location(
    'timing_observer', Path(__file__).resolve().parents[1] / 'scripts/timing_observer.py')
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


class TimingMetadataTest(unittest.TestCase):
    def test_image_keeps_exact_stamp_but_not_pixels(self):
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1_790_000_000, nanosec=123456789),
                                   frame_id='d435_infra1_optical_frame'),
            width=640, height=480, step=640, encoding='mono8', data=b'private-pixels')
        result = observer.message_metadata(message, 'sensor_msgs/msg/Image')
        self.assertEqual(result['source_stamp_ns'], 1_790_000_000_123456789)
        self.assertEqual(result['payload_bytes'], 14)
        self.assertNotIn('data', result)
        self.assertNotIn('private-pixels', json.dumps(result))

    def test_imu_keeps_motion_values_for_stationary_jitter_analysis(self):
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=7, nanosec=8),
                                   frame_id='d435_gyro_optical_frame'),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.1, w=0.99),
            angular_velocity=SimpleNamespace(x=0.01, y=-0.02, z=0.03),
            linear_acceleration=SimpleNamespace(x=0.1, y=9.7, z=-0.2),
        )
        result = observer.message_metadata(message, 'sensor_msgs/msg/Imu')
        self.assertEqual(result['source_stamp_ns'], 7_000_000_008)
        self.assertEqual(result['angular_velocity'], [0.01, -0.02, 0.03])
        self.assertEqual(result['linear_acceleration'], [0.1, 9.7, -0.2])

    def test_status_keeps_session_and_fault_reason(self):
        result = observer.message_metadata(SimpleNamespace(data=json.dumps({
            'session_id': 'session-a', 'enabled': False,
            'detail': 'base SLAM pose expired', 'dropped_stereo': 7,
        })), 'std_msgs/msg/String')
        self.assertEqual(result['state']['session_id'], 'session-a')
        self.assertEqual(result['state']['dropped_stereo'], 7)
        self.assertEqual(result['state']['detail'], 'base SLAM pose expired')

    def test_internal_timing_string_promotes_exact_source_stamp(self):
        result = observer.message_metadata(SimpleNamespace(data=json.dumps({
            'stage': 'upc_stereo_enqueued', 'source_stamp_ns': 123456789,
            'session_id': 'session-a', 'wall_ns': 20, 'monotonic_ns': 10,
        })), 'std_msgs/msg/String')
        self.assertEqual(result['source_stamp_ns'], 123456789)
        self.assertEqual(result['timing_stage'], 'upc_stereo_enqueued')

    def test_malformed_status_is_retained_for_diagnosis(self):
        result = observer.message_metadata(SimpleNamespace(data='broken{'), 'std_msgs/msg/String')
        self.assertEqual(result, {'text': 'broken{'})

    def test_nonfinite_fault_telemetry_remains_valid_json(self):
        result = observer.safe_json({'pose': [float('nan'), float('inf'), -float('inf')]})
        encoded = json.dumps(result, allow_nan=False)
        self.assertEqual(json.loads(encoded)['pose'], ['nan', 'inf', '-inf'])

    def test_rosout_keeps_failure_text(self):
        message = SimpleNamespace(
            level=40, name='visual_slam', msg='Visual tracking is lost',
            file='visual_slam_impl.cpp', function='UpdatePose', line=123)
        result = observer.message_metadata(message, 'rcl_interfaces/msg/Log')
        self.assertEqual(result['log_name'], 'visual_slam')
        self.assertEqual(result['log_message'], 'Visual tracking is lost')

    def test_jazzy_diagnostic_uint8_bytes_is_normalized(self):
        status = SimpleNamespace(
            level=b'\x02', name='visual_slam', hardware_id='gpu',
            message='tracking degraded', values=[])
        result = observer.message_metadata(
            SimpleNamespace(status=[status]), 'diagnostic_msgs/msg/DiagnosticArray')
        self.assertEqual(result['diagnostics'][0]['level'], 2)
        json.dumps(result, allow_nan=False)

    def test_action_status_keeps_status_five_and_goal_id(self):
        goal_info = SimpleNamespace(
            goal_id=SimpleNamespace(uuid=[1, 2, 255]),
            stamp=SimpleNamespace(sec=10, nanosec=20))
        message = SimpleNamespace(status_list=[SimpleNamespace(
            goal_info=goal_info, status=5)])
        result = observer.message_metadata(message, 'action_msgs/msg/GoalStatusArray')
        self.assertEqual(result['goal_statuses'][0], {
            'goal_id': '0102ff', 'status': 5, 'goal_stamp_ns': 10_000_000_020})

    def test_parameter_event_preserves_typed_changes(self):
        def value(kind, **fields):
            defaults = {
                'bool_value': False, 'integer_value': 0, 'double_value': 0.0,
                'string_value': '', 'byte_array_value': [], 'bool_array_value': [],
                'integer_array_value': [], 'double_array_value': [],
                'string_array_value': [],
            }
            defaults.update(fields)
            return SimpleNamespace(type=kind, **defaults)

        message = SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=34),
            node='/rby1/vslam/nav2/controller_server',
            new_parameters=[SimpleNamespace(
                name='goal_checker.stateful', value=value(1, bool_value=False))],
            changed_parameters=[
                SimpleNamespace(name='goal_checker.xy_goal_tolerance',
                                value=value(3, double_value=0.01)),
                SimpleNamespace(name='controller_plugins',
                                value=value(9, string_array_value=['FollowPath'])),
            ],
            deleted_parameters=[SimpleNamespace(name='old_parameter')],
        )
        result = observer.message_metadata(message, 'rcl_interfaces/msg/ParameterEvent')
        self.assertEqual(result['source_stamp_ns'], 12_000_000_034)
        self.assertEqual(result['parameter_node'], message.node)
        self.assertEqual(result['new_parameters'][0]['value'], False)
        self.assertEqual(result['changed_parameters'][0]['value'], 0.01)
        self.assertEqual(result['changed_parameters'][1]['value'], ['FollowPath'])
        self.assertEqual(result['deleted_parameters'], ['old_parameter'])

    def test_tf_metadata_keeps_only_localization_chain(self):
        def transform(parent, child, x, yaw):
            return SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=4, nanosec=5),
                                       frame_id=parent),
                child_frame_id=child,
                transform=SimpleNamespace(
                    translation=SimpleNamespace(x=x, y=0.25, z=0.0),
                    rotation=SimpleNamespace(x=0.0, y=0.0,
                                             z=math.sin(yaw / 2.0),
                                             w=math.cos(yaw / 2.0))))

        message = SimpleNamespace(transforms=[
            transform('vslam_map', 'odom', 1.5, 0.2),
            transform('base', 'link_head_0', 9.0, 1.0),
        ])
        result = observer.message_metadata(message, 'tf2_msgs/msg/TFMessage')
        self.assertEqual(len(result['transforms']), 1)
        self.assertEqual(result['transforms'][0]['parent_frame'], 'vslam_map')
        self.assertEqual(result['transforms'][0]['child_frame'], 'odom')
        self.assertAlmostEqual(result['transforms'][0]['yaw'], 0.2)

    def test_path_metadata_exposes_short_path_endpoint(self):
        def pose(x, y, yaw=0.0):
            return SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(x=0.0, y=0.0,
                                            z=math.sin(yaw / 2.0),
                                            w=math.cos(yaw / 2.0))))

        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2),
                                   frame_id='vslam_map'),
            poses=[pose(0.0, 0.0), pose(0.03, 0.04, 0.5)])
        result = observer.message_metadata(message, 'nav_msgs/msg/Path')
        self.assertEqual(result['pose_count'], 2)
        self.assertAlmostEqual(result['path_length_m'], 0.05)
        self.assertEqual(result['start_pose'][:2], [0.0, 0.0])
        self.assertEqual(result['end_pose'][:2], [0.03, 0.04])
        self.assertAlmostEqual(result['end_pose'][2], 0.5)

    def test_follow_path_feedback_keeps_controller_progress(self):
        pose = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=4),
                                   frame_id='vslam_map'),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.2, y=-0.1),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        message = SimpleNamespace(
            goal_id=SimpleNamespace(uuid=[10, 11]),
            feedback=SimpleNamespace(
                current_pose=pose, distance_to_goal=0.025, speed=0.04))
        result = observer.message_metadata(
            message, 'nav2_msgs/action/FollowPath_FeedbackMessage')
        self.assertEqual(result['goal_id'], '0a0b')
        self.assertEqual(result['current_pose'][:2], [0.2, -0.1])
        self.assertEqual(result['distance_to_goal_m'], 0.025)
        self.assertEqual(result['speed_mps'], 0.04)


if __name__ == '__main__':
    unittest.main()
