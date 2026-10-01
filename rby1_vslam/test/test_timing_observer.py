"""ROS-free checks for the passive capture's payload and timestamp handling."""
import importlib.util
import json
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

    def test_status_keeps_session_and_fault_reason(self):
        result = observer.message_metadata(SimpleNamespace(data=json.dumps({
            'session_id': 'session-a', 'enabled': False,
            'detail': 'base SLAM pose expired', 'dropped_stereo': 7,
        })), 'std_msgs/msg/String')
        self.assertEqual(result['state']['session_id'], 'session-a')
        self.assertEqual(result['state']['dropped_stereo'], 7)
        self.assertEqual(result['state']['detail'], 'base SLAM pose expired')

    def test_malformed_status_is_retained_for_diagnosis(self):
        result = observer.message_metadata(SimpleNamespace(data='broken{'), 'std_msgs/msg/String')
        self.assertEqual(result, {'text': 'broken{'})

    def test_nonfinite_fault_telemetry_remains_valid_json(self):
        result = observer.safe_json({'pose': [float('nan'), float('inf'), -float('inf')]})
        encoded = json.dumps(result, allow_nan=False)
        self.assertEqual(json.loads(encoded)['pose'], ['nan', 'inf', '-inf'])


if __name__ == '__main__':
    unittest.main()
