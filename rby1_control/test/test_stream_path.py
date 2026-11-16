import math

import pytest

from rby1_control.stream_path import (
    build_stream_path_plan,
    quaternion_distance_rad,
    rpy_deg_to_quaternion,
    slerp,
)


START = (0.4, -0.2, 0.9, 0.0, 0.0, -90.0)


def test_quaternion_matches_manipulator_controller_convention():
    from rby1_control.manipulator_controller import ManipulatorController

    for rpy in ((0.0, 0.0, -90.0), (12.5, -40.0, 170.0), (-170.0, 85.0, 3.0)):
        expected = ManipulatorController.rpy_deg_to_quaternion(*rpy)
        assert rpy_deg_to_quaternion(*rpy) == pytest.approx(expected)


def test_slerp_endpoints_and_midpoint():
    a = rpy_deg_to_quaternion(0.0, 0.0, 0.0)
    b = rpy_deg_to_quaternion(0.0, 0.0, 90.0)
    assert slerp(a, b, 0.0) == pytest.approx(a)
    assert slerp(a, b, 1.0) == pytest.approx(b)
    mid = slerp(a, b, 0.5)
    assert quaternion_distance_rad(a, mid) == pytest.approx(math.radians(45.0))
    assert quaternion_distance_rad(mid, b) == pytest.approx(math.radians(45.0))


def test_plan_allocates_time_by_required_time_and_interpolates():
    waypoints = [
        (0.4, -0.1, 0.9, 0.0, 0.0, -90.0),   # 0.10 m, no rotation
        (0.4, -0.1, 0.9, 0.0, 0.0, -60.0),   # no translation, 30 deg
    ]
    plan = build_stream_path_plan(
        START, waypoints, 10.0, 0.05, math.radians(30.0),
    )
    # 0.10 m / 0.05 m/s = 2 s; 30 deg / 30 deg/s = 1 s -> 2:1 allocation.
    assert plan.required_time_sec == pytest.approx(3.0)
    assert plan.segments[0].end_time_sec == pytest.approx(10.0 * 2.0 / 3.0)
    assert plan.segments[1].end_time_sec == pytest.approx(10.0)
    assert plan.linear_distance_m == pytest.approx(0.10)
    assert plan.angular_distance_rad == pytest.approx(math.radians(30.0))

    x, y, z, *q = plan.target_at(0.0)
    assert (x, y, z) == pytest.approx(START[:3])
    assert tuple(q) == pytest.approx(rpy_deg_to_quaternion(0.0, 0.0, -90.0))

    x, y, z, *q = plan.target_at(10.0 / 3.0)
    assert (x, y, z) == pytest.approx((0.4, -0.15, 0.9))

    x, y, z, *q = plan.target_at(10.0)
    assert (x, y, z) == pytest.approx((0.4, -0.1, 0.9))
    assert tuple(q) == pytest.approx(rpy_deg_to_quaternion(0.0, 0.0, -60.0))
    # Clamped beyond the end and before the start.
    assert plan.target_at(99.0) == pytest.approx(plan.target_at(10.0))
    assert plan.target_at(-1.0) == pytest.approx(plan.target_at(0.0))


def test_plan_rejects_duration_shorter_than_velocity_limits_allow():
    waypoints = [(0.4, 0.3, 0.9, 0.0, 0.0, -90.0)]  # 0.5 m at 0.05 m/s = 10 s
    with pytest.raises(ValueError, match='needs at least 10.00 s'):
        build_stream_path_plan(START, waypoints, 5.0, 0.05, 0.5)
    build_stream_path_plan(START, waypoints, 10.0, 0.05, 0.5)


def test_plan_rejects_limits_above_backend_caps():
    waypoints = [(0.41, -0.2, 0.9, 0.0, 0.0, -90.0)]
    with pytest.raises(ValueError, match='exceeds the backend limit'):
        build_stream_path_plan(
            START, waypoints, 5.0, 0.2, 0.5,
            backend_linear_velocity_limit=0.1,
        )
    with pytest.raises(ValueError, match='exceeds the backend limit'):
        build_stream_path_plan(
            START, waypoints, 5.0, 0.05, 1.0,
            backend_angular_velocity_limit=0.5,
        )


def test_plan_rejects_malformed_inputs():
    with pytest.raises(ValueError, match='at least one waypoint'):
        build_stream_path_plan(START, [], 5.0, 0.05, 0.5)
    with pytest.raises(ValueError, match='6 values'):
        build_stream_path_plan(START, [(0.0, 0.0, 0.0)], 5.0, 0.05, 0.5)
    with pytest.raises(ValueError, match='positive'):
        build_stream_path_plan(START, [START], 0.0, 0.05, 0.5)
    with pytest.raises(ValueError, match='non-finite'):
        build_stream_path_plan(
            START, [(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0)], 5.0, 0.05, 0.5,
        )


def test_zero_length_path_holds_the_pose():
    plan = build_stream_path_plan(START, [START, START], 2.0, 0.05, 0.5)
    assert plan.required_time_sec == 0.0
    assert plan.segments[0].end_time_sec == pytest.approx(1.0)
    assert plan.target_at(1.3)[:3] == pytest.approx(START[:3])


def test_arc_waypoints_keep_orientation_fixed():
    radius = 0.23
    waypoints = []
    for k in range(1, 31):
        theta = math.radians(180.0 - 2.0 * k)
        dx = radius * math.cos(theta) + radius
        dy = radius * math.sin(theta)
        waypoints.append((START[0] + dx, START[1] + dy, START[2], 0.0, 0.0, -90.0))
    plan = build_stream_path_plan(START, waypoints, 12.0, 0.05, 0.3)
    assert plan.angular_distance_rad == pytest.approx(0.0, abs=1e-9)
    assert plan.linear_distance_m == pytest.approx(radius * math.radians(60.0), rel=1e-3)
    fixed = rpy_deg_to_quaternion(0.0, 0.0, -90.0)
    for t in (0.0, 3.3, 7.7, 12.0):
        assert tuple(plan.target_at(t)[3:]) == pytest.approx(fixed)
