import math

import pytest

from rby1_vslam.geometry import compose, inverse, offset_covariance, planar_correction, quaternion, yaw


def test_camera_rig_offset_and_rotation_are_composed_not_relabelled():
    q90 = (0., 0., math.sin(math.pi/4), math.cos(math.pi/4))
    position, rotation = compose((2., 1., 0.), q90, (-1., 0., 0.), (0., 0., 0., 1.))
    assert position == pytest.approx((2., 0., 0.))
    assert yaw(rotation) == pytest.approx(math.pi/2)


def test_dynamic_camera_mount_motion_is_removed_from_base_pose():
    angle = 0.4
    rig = ((1., 2., 0.8), (0., 0., math.sin(angle/2), math.cos(angle/2)))
    camera_base = inverse(*rig)
    position, rotation = compose(*rig, *camera_base)
    assert position == pytest.approx((0., 0., 0.))
    assert yaw(rotation) == pytest.approx(0.)


def test_yaw_uncertainty_propagates_through_camera_lever_arm():
    cov = [0.] * 36
    cov[35] = 0.04
    out = offset_covariance(cov, (0., 0., 0., 1.), (1., 0., 0.))
    assert out[7] == pytest.approx(0.04)
    assert out[11] == pytest.approx(0.04)
    assert out[31] == pytest.approx(0.04)


@pytest.mark.parametrize('q', [(0., 0., 0., 0.), (0., 0., 0., math.nan), (0., 0., 0., 2.)])
def test_invalid_quaternion_rejected(q):
    with pytest.raises(ValueError):
        quaternion(q)


def test_global_correction_composed_with_wheel_pose_matches_visual_base():
    visual = ((2., 3., 0.1), (0., 0., math.sin(0.4), math.cos(0.4)))
    wheel = ((1., -1., 0.), (0., 0., math.sin(0.1), math.cos(0.1)))
    correction = planar_correction(*visual, *wheel)
    position, rotation = compose(*correction, *wheel)
    assert position[:2] == pytest.approx(visual[0][:2])
    assert position[2] == 0.  # Nav2 uses a planar global correction.
    assert yaw(rotation) == pytest.approx(yaw(visual[1]))
