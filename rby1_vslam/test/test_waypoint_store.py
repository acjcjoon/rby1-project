from pathlib import Path

import pytest
import yaml

from rby1_vslam.waypoint_store import Waypoint, load_waypoints, next_name, save_waypoints


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
