import math
from pathlib import Path
import tempfile
import unittest

from rby1_vslam.geometry import compose, inverse, planar_correction, yaw
from rby1_vslam.map_tool import has_map_database, pose_quaternion, validate_path
from rby1_vslam.modes import depth_module_profile, isaac_parameters, parse_tracking_mode


class ModeTests(unittest.TestCase):
    def test_humble_api_and_tf_ownership(self):
        for mode, imu in ((0, False), (1, True)):
            parameters = isaac_parameters(mode)
            self.assertEqual(parameters['enable_imu_fusion'], imu)
            self.assertEqual(parameters['base_frame'], 'd435_link')
            self.assertNotIn('tracking_mode', parameters)
            self.assertFalse(parameters['publish_odom_to_base_tf'])
            self.assertFalse(parameters['publish_map_to_odom_tf'])

    def test_rgbd_acquisition_is_distinct_from_supported_vslam_inputs(self):
        self.assertEqual(parse_tracking_mode('2'), 2)
        with self.assertRaisesRegex(ValueError, 'only supported for camera recording'):
            isaac_parameters(2)

    def test_invalid_modes_are_rejected(self):
        for value in (True, 3, '1.1', 'foo'):
            with self.assertRaises(ValueError):
                parse_tracking_mode(value)

    def test_realsense_legacy_shared_profile(self):
        self.assertEqual(depth_module_profile(0, False, '640x360x30', '640,480,30'), '640,360,30')
        self.assertEqual(depth_module_profile(2, False, '640,360,30', '640,480,30'), '640,480,30')
        with self.assertRaisesRegex(ValueError, 'identical'):
            depth_module_profile(0, True, '640,360,30', '640,480,30')


class MapTests(unittest.TestCase):
    def test_missing_or_non_database_directory_cannot_localize(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                validate_path(tmp, 'localize')
            (Path(tmp) / 'map.mdb').write_bytes(b'fixture')
            self.assertTrue(has_map_database(tmp))
            self.assertEqual(validate_path(tmp, 'localize'), str(Path(tmp)))

    def test_saving_over_an_existing_map_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / 'map.mdb').write_bytes(b'fixture')
            with self.assertRaisesRegex(ValueError, 'new or empty'):
                validate_path(tmp, 'save')

    def test_save_needs_an_existing_parent_and_absolute_path(self):
        with self.assertRaises(ValueError):
            validate_path('relative-map', 'save')
        with tempfile.TemporaryDirectory() as tmp:
            destination = Path(tmp) / 'new-map'
            self.assertEqual(validate_path(destination, 'save'), str(destination))
            with self.assertRaises(ValueError):
                validate_path(destination / 'missing-parent-map', 'save')


class GeometryTests(unittest.TestCase):
    def test_head_motion_compensation_preserves_stationary_base(self):
        base_position, base_rotation = (2.0, 3.0, 0.0), pose_quaternion(0, 0, 0.4)
        for head_pitch in (0.0, 0.2, -0.3):
            camera_position, camera_rotation = compose(
                base_position, base_rotation, (0.1, 0.0, 1.5), pose_quaternion(0, head_pitch, 0))
            camera_base = inverse((0.1, 0.0, 1.5), pose_quaternion(0, head_pitch, 0))
            compensated_position, compensated_rotation = compose(camera_position, camera_rotation, *camera_base)
            for actual, expected in zip(compensated_position, base_position):
                self.assertAlmostEqual(actual, expected)
            self.assertAlmostEqual(yaw(compensated_rotation), 0.4)

    def test_global_correction_matches_wheel_pose_in_map(self):
        correction = planar_correction((3, 4, 0), pose_quaternion(0, 0, math.pi / 2),
                                       (1, 2, 0), pose_quaternion(0, 0, 0))
        position, rotation = compose(*correction, (1, 2, 0), pose_quaternion(0, 0, 0))
        self.assertAlmostEqual(position[0], 3)
        self.assertAlmostEqual(position[1], 4)
        self.assertAlmostEqual(yaw(rotation), math.pi / 2)


if __name__ == '__main__':
    unittest.main()
