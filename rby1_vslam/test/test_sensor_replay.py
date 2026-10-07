import pytest

from rby1_vslam.sensor_replay import (
    CONDITION_PROFILES, condition_profile, rebase_stamp_ns)


def test_condition_matrix_is_complete():
    assert CONDITION_PROFILES == {
        'baseline': ('replay', 'replay'),
        'no_imu': ('replay', 'none'),
        'fixed_stereo': ('freeze_after_marker', 'replay'),
        'fixed_stereo_no_imu': ('freeze_after_marker', 'none'),
    }
    for removed_or_unknown in ('fixed_imu', 'fixed_both', 'unknown'):
        with pytest.raises(ValueError):
            condition_profile(removed_or_unknown)


def test_rebase_preserves_recorded_delta():
    assert rebase_stamp_ns(1_250, 1_000, 50_000) == 50_250
    with pytest.raises(ValueError):
        rebase_stamp_ns(-1, 0, 0)
