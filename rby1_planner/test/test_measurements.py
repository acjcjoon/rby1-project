"""Measurements tab math: tag-to-tag relative pose and odometry delta."""
import math

import pytest

from rby1_planner.measurements import (
    OdometryTracker,
    average_relative_pose,
    parse_tag_name,
)
from rby1_planner.observation import ObjectObservation, rpy_deg_to_quaternion
from test_vision_manager import _manager


def _obs(tag, position, rpy_deg, stamp=1):
    return ObjectObservation(
        object_id=tag,
        frame_id='base',
        stamp_ns=stamp,
        received_at_ns=stamp,
        position=position,
        orientation_xyzw=rpy_deg_to_quaternion(rpy_deg),
        confidence=1.0,
    )


@pytest.mark.parametrize('text, expected', [
    ('tag5', 'tag_5'), ('Tag_12', 'tag_12'), (' tag0 ', 'tag_0'),
])
def test_parse_tag_name(text, expected):
    assert parse_tag_name(text) == expected


@pytest.mark.parametrize('text', ['5', 'tagx', 'tag', 'tag-1', ''])
def test_parse_tag_name_rejects(text):
    with pytest.raises(ValueError):
        parse_tag_name(text)


def test_relative_pose_is_expressed_in_tag1_frame():
    # Tag1 is yawed +90 deg, so tag1 +y points along base -x. Tag2 sits
    # 0.3 m along base -x from tag1 (= +y in tag1) and is yawed 10 deg more.
    first = _obs('tag_5', (1.0, 0.2, 0.5), (0.0, 0.0, 90.0))
    second = _obs('tag_6', (0.7, 0.2, 0.5), (0.0, 0.0, 100.0))
    result = average_relative_pose([(first, second)], camera='d435')
    assert result.xyz == pytest.approx((0.0, 0.3, 0.0), abs=1e-9)
    assert result.rpy_deg == pytest.approx((0.0, 0.0, 10.0), abs=1e-6)
    assert result.distance == pytest.approx(0.3)
    assert result.base_delta_xyz == pytest.approx((-0.3, 0.0, 0.0))


def test_relative_pose_average_trims_outliers():
    pairs = []
    for index in range(10):
        noise = 0.001 * (index % 3 - 1)
        pairs.append((
            _obs('tag_1', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), index),
            _obs('tag_2', (0.2 + noise, 0.0, 0.0), (0.0, 0.0, 0.0), index),
        ))
    pairs.append((
        _obs('tag_1', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 99),
        _obs('tag_2', (0.9, 0.0, 0.0), (0.0, 0.0, 0.0), 99),
    ))
    result = average_relative_pose(pairs, camera='d405_right', outlier_count=2)
    assert result.xyz[0] == pytest.approx(0.2, abs=1e-3)
    assert result.sample_count == 11


def test_odometry_tracker_composes_moves_in_start_frame():
    tracker = OdometryTracker()
    tracker.update((5.0, 5.0, math.pi / 2), 0.0)
    tracker.start()
    t = 0.0
    # Drive 1 m along odom +y (= start-frame forward), then turn and drive
    # 0.5 m along odom -x (= start-frame left), in small increments.
    for i in range(1, 51):
        t += 0.02
        tracker.update((5.0, 5.0 + 0.02 * i, math.pi / 2), t)
    for i in range(1, 11):   # turn in place by +90 deg
        t += 0.02
        tracker.update((5.0, 6.0, math.pi / 2 + math.pi / 20 * i), t)
    for i in range(1, 26):
        t += 0.02
        tracker.update((5.0 - 0.02 * i, 6.0, math.pi), t)
    delta = tracker.finish()
    assert delta.x == pytest.approx(1.0, abs=1e-9)
    assert delta.y == pytest.approx(0.5, abs=1e-9)
    assert delta.yaw == pytest.approx(math.pi / 2, abs=1e-9)
    assert delta.odom_dx == pytest.approx(-0.5, abs=1e-9)
    assert delta.odom_dy == pytest.approx(1.0, abs=1e-9)
    assert delta.path_length == pytest.approx(1.5, abs=1e-6)
    assert delta.duration_sec == pytest.approx(1.7)
    assert delta.skipped_jumps == 0


def test_odometry_tracker_requires_odometry_before_start():
    with pytest.raises(RuntimeError):
        OdometryTracker().start()


def test_odometry_tracker_skips_large_jump():
    tracker = OdometryTracker()
    tracker.update((3.0, 0.0, 0.0), 0.0)
    tracker.start()
    tracker.update((3.1, 0.0, 0.0), 0.1)
    tracker.update((0.0, 0.0, 0.0), 0.2)   # reset: 3.1 m in one sample
    tracker.update((0.1, 0.0, 0.0), 0.3)
    delta = tracker.finish()
    assert delta.skipped_jumps == 1
    assert delta.x == pytest.approx(0.2)


def test_measurement_hold_survives_task_release():
    manager, _node, _clock, spawned = _manager()
    manager.hold('measurements', 'd435')
    manager.require('d435')
    manager.release_all()          # Task finished
    assert manager.running('d435')
    manager.release_owner('measurements')
    assert not manager.running('d435')
    assert len(spawned) == 1


def test_measurement_hold_blocks_off_mode():
    manager, *_ = _manager()
    manager.hold('measurements', 'd405_left')
    with pytest.raises(RuntimeError):
        manager.set_mode('off')
    manager.release_owner('measurements')
    manager.set_mode('off')
    with pytest.raises(RuntimeError):
        manager.hold('measurements', 'd405_left')
