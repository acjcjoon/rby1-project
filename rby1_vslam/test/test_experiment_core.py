import math

import pytest

from rby1_vslam.experiment_core import (
    build_route, format_summary, measured_error, pose_error, success, summarize,
)


def test_route_starts_and_returns_to_first_station():
    legs = build_route(['A', 'B', 'C'], 2)
    assert [(l.start, l.target, l.round) for l in legs] == [
        ('A', 'B', 1), ('B', 'C', 1), ('C', 'A', 1),
        ('A', 'B', 2), ('B', 'C', 2), ('C', 'A', 2)]
    assert [l.index for l in legs] == list(range(1, 7))


@pytest.mark.parametrize('stations,rounds', [(['A'], 1), (['A', 'A'], 1), (['A', 'B'], 0)])
def test_route_rejects_bad_input(stations, rounds):
    with pytest.raises(ValueError):
        build_route(stations, rounds)


def test_measured_error_translation_only():
    ex, ey, err, yaw = measured_error(6.0, 8.0, 6.0, 8.0, 500.0)
    assert (ex, ey, err, yaw) == pytest.approx((6.0, 8.0, 10.0, 0.0))


def test_measured_error_ccw_rotation_is_positive():
    # CCW rotation about the front centre: left corner moves back, right corner forward.
    theta = math.radians(1.0)
    half = 250.0
    _, _, err, yaw = measured_error(-half * math.sin(theta), 0.0, half * math.sin(theta), 0.0, 500.0)
    assert err == pytest.approx(0.0, abs=1e-9)
    assert yaw == pytest.approx(1.0, rel=1e-3)


def test_pose_error_in_station_frame():
    target = (1.0, 2.0, math.pi / 2)          # station faces +y
    pose = (1.0, 2.01, math.pi / 2 + math.radians(1))  # 10 mm ahead along its heading
    ex, ey, err, yaw = pose_error(pose, target)
    assert ex == pytest.approx(10.0)
    assert ey == pytest.approx(0.0, abs=1e-9)
    assert yaw == pytest.approx(1.0)


def test_success_needs_both_tolerances():
    assert success(19.9, 1.9, 20, 2)
    assert not success(20.1, 0.0, 20, 2)
    assert not success(5.0, -2.1, 20, 2)


def test_summary_counts_only_measured_rows():
    rows = [
        {'round': 1, 'gt_err_mm': 10.0, 'gt_yaw_deg': 1.0, 'success': True, 'nav_s': 10.0,
         'correction_wait_s': 2.0},
        {'round': 2, 'gt_err_mm': 30.0, 'gt_yaw_deg': -3.0, 'success': False, 'nav_s': 12.0,
         'correction_wait_s': ''},
        {'round': 2, 'gt_err_mm': '', 'gt_yaw_deg': '', 'success': '', 'nav_s': 11.0,
         'correction_wait_s': ''},
    ]
    s = summarize(rows)
    assert s['legs'] == 3 and s['measured'] == 2
    assert s['success_rate'] == pytest.approx(50.0)
    assert s['pos_err_mm'][0] == pytest.approx(20.0)
    assert s['yaw_err_deg'][0] == pytest.approx(2.0)
    assert s['first_round'][1][0] == pytest.approx(10.0)
    assert s['last_round'][1][0] == pytest.approx(30.0)
    assert '성공률 50%' in format_summary(s)


def test_ground_truth_prefers_tag_over_ruler():
    from rby1_vslam.experiment_core import set_ground_truth
    row = {'pos_tol_mm': 20, 'yaw_tol_deg': 2}
    tag = {'x_mm': 3.0, 'y_mm': 4.0, 'err_mm': 5.0, 'yaw_deg': 0.5}
    assert set_ground_truth(row, tag=tag, ruler=(30.0, 0.0, 30.0, 0.0)) is True
    assert row['gt_source'] == 'tag' and row['gt_err_mm'] == 5.0
    row2 = {'pos_tol_mm': 20, 'yaw_tol_deg': 2}
    assert set_ground_truth(row2, ruler=(30.0, 0.0, 30.0, 0.0)) is False
    assert row2['gt_source'] == 'ruler'
    assert set_ground_truth({'pos_tol_mm': 20, 'yaw_tol_deg': 2}) is None
