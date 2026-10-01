from pathlib import Path

import pytest
import yaml

from rby1_vslam.waypoint_store import (
    Waypoint, default_waypoints_path, load_waypoints, next_name, save_waypoints,
)


def test_default_waypoints_are_saved_inside_installed_package(tmp_path):
    share = tmp_path / 'install' / 'share' / 'rby1_vslam'
    path = default_waypoints_path(share)
    save_waypoints(path, [Waypoint('Dock', 1.0, 2.0, 0.5)])
    assert path == share / 'data' / 'rby1_vslam_waypoints.yaml'
    assert load_waypoints(path) == [Waypoint('Dock', 1.0, 2.0, 0.5)]


def test_default_waypoints_follow_symlink_install_to_source(tmp_path):
    source = tmp_path / 'src' / 'rby1_vslam'
    (source / 'config').mkdir(parents=True)
    (source / 'package.xml').write_text('<package/>', encoding='utf-8')
    config = source / 'config' / 'isaac_vslam.yaml'
    config.write_text('{}', encoding='utf-8')
    share = tmp_path / 'install' / 'share' / 'rby1_vslam'
    (share / 'config').mkdir(parents=True)
    try:
        (share / 'config' / config.name).symlink_to(config)
    except OSError:
        pytest.skip('symlink creation is unavailable on this host')
    path = default_waypoints_path(share)
    save_waypoints(path, [Waypoint('Dock', 1.0, 2.0, 0.5)])
    assert path == source / 'data' / 'rby1_vslam_waypoints.yaml'
    assert load_waypoints(path) == [Waypoint('Dock', 1.0, 2.0, 0.5)]


def test_waypoint_round_trip_and_next_name(tmp_path):
    path = tmp_path / 'nested' / 'points.yaml'
    points = [Waypoint('Point 1', 1.0, -2.0, 0.25), Waypoint('Dock', 0.0, 0.0, -1.0)]
    save_waypoints(path, points)
    assert load_waypoints(path) == points
    assert next_name(points) == 'Point 2'
    assert not Path(str(path) + '.tmp').exists()


def test_waypoint_file_rejects_wrong_frame(tmp_path):
    path = tmp_path / 'points.yaml'
    save_waypoints(path, [Waypoint('Point 1', 0.0, 0.0, 0.0)])
    with pytest.raises(ValueError, match='does not match'):
        load_waypoints(path, 'some_other_map')


def test_waypoint_file_rejects_duplicate_names(tmp_path):
    path = tmp_path / 'points.yaml'
    path.write_text(yaml.safe_dump({
        'schema_version': 1,
        'frame_id': 'vslam_map',
        'waypoints': [
            {'name': 'Point 1', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            {'name': 'Point 1', 'x': 1.0, 'y': 0.0, 'yaw': 0.0},
        ],
    }), encoding='utf-8')
    with pytest.raises(ValueError, match='duplicate'):
        load_waypoints(path)
