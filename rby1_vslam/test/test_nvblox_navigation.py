"""Offline regressions for optional obstacle mapping and command watchdogs."""

from copy import deepcopy
from pathlib import Path
import sys

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from rby1_vslam.navigation_config import configure_navigation
from rby1_vslam.nav2_gate_core import VelocityGate


def _settings():
    with open(PACKAGE / 'config/navigation.yaml', encoding='utf-8') as stream:
        base = yaml.safe_load(stream)
    with open(PACKAGE / 'config/navigation_nvblox.yaml', encoding='utf-8') as stream:
        nvblox = yaml.safe_load(stream)
    return base, nvblox


@pytest.mark.parametrize('scan', [False, True])
@pytest.mark.parametrize('saved_map', ['', '/tmp/test-map.yaml'])
def test_nvblox_keeps_obstacles_unknown_and_static_map(scan, saved_map):
    base, overlay = _settings()
    original = deepcopy(base)
    params = configure_navigation(base, use_scan=scan, use_nvblox=True,
                                  occupancy_map=saved_map, nvblox_settings=overlay)
    assert base == original
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        assert costmap['track_unknown_space'] is True
        assert ('obstacle_layer' in costmap['plugins']) == scan
        assert costmap['plugins'].index('nvblox_layer') < costmap['plugins'].index('inflation_layer')
        assert costmap['nvblox_layer']['nvblox_map_slice_topic'] == '/nvblox_node/static_map_slice'
        assert costmap['nvblox_layer']['nav2_costmap_global_frame'] == costmap['global_frame']
    global_map = params['global_costmap']['global_costmap']['ros__parameters']
    assert ('static_layer' in global_map['plugins']) == bool(saved_map)
    assert global_map['rolling_window'] == (not saved_map)
    controller = params['controller_server']['ros__parameters']['FollowPath']
    assert 'RegulatedPurePursuitController' in controller['plugin']
    assert controller['allow_reversing'] is False
    assert controller['use_rotate_to_heading'] is True
    assert 'max_y_vel' not in controller
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['min_velocity'][0] == 0.0
    assert smoother['min_velocity'][1] == smoother['max_velocity'][1] == 0.0
    assert params['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False


def test_legacy_navigation_and_custom_plugins_are_preserved():
    base, _ = _settings()
    assert configure_navigation(base, use_scan=True, use_nvblox=False) == base
    result = configure_navigation(base, use_scan=False, use_nvblox=False)
    assert result['controller_server'] == base['controller_server']
    assert result['velocity_smoother'] == base['velocity_smoother']
    for name in ('local_costmap', 'global_costmap'):
        assert result[name][name]['ros__parameters']['track_unknown_space'] is False
    base['global_costmap']['global_costmap']['ros__parameters']['plugins'].append('custom_layer')
    result = configure_navigation(base, use_scan=False, use_nvblox=False,
                                  occupancy_map='/tmp/test-map.yaml')
    assert 'custom_layer' in result['global_costmap']['global_costmap']['ros__parameters']['plugins']


def _localization(gate, now=10.0, ros_now=100.0):
    gate.update_localization(True, True, True, False, 'wheel-session', now)
    gate.update_wheel(ros_now, now, ros_now)
    gate.update_estimate((0.0, 0.0, 0.0), 'vslam_map', ros_now, now, ros_now)


def _observations(gate, now=10.0, ros_now=100.0):
    for source in gate.observation_sources:
        gate.update_observation(source, ros_now, now, ros_now)


def _ready(*, mapping=True):
    gate = VelocityGate(require_nvblox=mapping, forward_only=mapping)
    gate.cancellation_complete(gate.cancel_serial)
    _localization(gate)
    if mapping:
        _observations(gate)
    assert gate.enable(10.0, 100.0)[0]
    gate.set_goal_active(True, 10.01)
    return gate


def test_enable_requires_camera_slice_and_integration_but_legacy_does_not():
    gate = VelocityGate(require_nvblox=True)
    gate.cancellation_complete(gate.cancel_serial)
    _localization(gate)
    assert 'depth' in gate.enable(10.0, 100.0)[1]
    gate.update_observation('depth', 100.0, 10.0, 100.0)
    assert 'slice' in gate.enable(10.0, 100.0)[1]
    gate.update_observation('slice', 100.0, 10.0, 100.0)
    assert 'integrated_depth' in gate.enable(10.0, 100.0)[1]
    gate.update_observation('integrated_depth', 100.0, 10.0, 100.0)
    assert gate.enable(10.0, 100.0)[0]
    assert _ready(mapping=False).enabled


@pytest.mark.parametrize('source, elapsed', [
    ('depth', 0.51), ('slice', 1.01), ('integrated_depth', 1.01)])
def test_mapping_loss_stops_cancels_and_requires_explicit_reenable(source, elapsed):
    gate = _ready()
    now, ros_now = 10.0 + elapsed, 100.0 + elapsed
    _localization(gate, now, ros_now)
    for other in gate.observation_sources:
        if other != source:
            gate.update_observation(other, ros_now, now, ros_now)
    assert gate.update_command((0.1, 0.0, 0.1), now)
    assert gate.tick(now, ros_now) == (0.0, 0.0, 0.0)
    assert not gate.enabled and gate.cancel_pending
    assert source in gate.detail
    _observations(gate, now + 0.01, ros_now + 0.01)
    assert gate.tick(now + 0.01, ros_now + 0.01) == (0.0, 0.0, 0.0)
    assert not gate.enable(now + 0.01, ros_now + 0.01)[0]  # cancellation barrier persists
    gate.cancellation_complete(gate.cancel_serial)
    assert gate.enable(now + 0.01, ros_now + 0.01)[0]
    assert gate.tick(now + 0.01, ros_now + 0.01) == (0.0, 0.0, 0.0)  # no old command


@pytest.mark.parametrize('source', ['depth', 'slice', 'integrated_depth'])
def test_replayed_stamps_cannot_refresh_mapping_watchdog(source):
    gate = _ready()
    assert not gate.update_observation(source, 100.0, 10.1, 100.1)
    assert getattr(gate, source) == (100.0, 10.0)
    assert not gate.update_observation(source, 99.99, 10.1, 100.1)
    assert getattr(gate, source) is None
    assert not gate.enabled


@pytest.mark.parametrize('offset', [-2.0, 2.0])
def test_future_and_stale_depth_are_rejected(offset):
    gate = _ready()
    assert not gate.update_observation('depth', 100.1 + offset, 10.1, 100.1)
    assert not gate.enabled


def test_live_camera_and_republished_map_cannot_hide_stalled_integration():
    gate = _ready()
    # Camera and ESDF timers keep running after the camera TF disappears.
    # Repeating the last back-projected cloud cannot renew successful integration.
    for step in range(1, 12):
        now, ros_now = 10.0 + step * 0.1, 100.0 + step * 0.1
        _localization(gate, now, ros_now)
        gate.update_observation('depth', ros_now, now, ros_now)
        gate.update_observation('slice', ros_now, now, ros_now)
        assert not gate.update_observation('integrated_depth', 100.0, now, ros_now)
        gate.update_command((0.1, 0.0, 0.0), now)
        velocity = gate.tick(now, ros_now)
    assert velocity == (0.0, 0.0, 0.0)
    assert not gate.enabled and gate.cancel_pending
    assert 'integrated_depth' in gate.detail


@pytest.mark.parametrize('command', [(-0.1, 0.0, 0.0), (0.1, 0.02, 0.0)])
def test_forward_only_rejects_reverse_or_lateral_commands(command):
    gate = _ready()
    assert not gate.update_command(command, 10.02)
    assert not gate.enabled and gate.cancel_pending
    assert gate.tick(10.02, 100.02) == (0.0, 0.0, 0.0)


def test_forward_rotation_is_bounded_and_legacy_omni_is_unchanged():
    gate = _ready()
    assert gate.update_command((0.5, 1e-8, 0.5), 10.02)
    assert gate.tick(10.02, 100.02) == (0.15, 0.0, 0.25)
    gate = _ready(mapping=False)
    assert gate.update_command((-0.1, 0.05, 0.0), 10.02)
    assert gate.tick(10.02, 100.02) == (-0.1, 0.05, 0.0)


def test_nvblox_does_not_relax_station_hold_or_localization_faults():
    gate = _ready()
    gate.update_localization(True, True, False, True, 'wheel-session', 10.1)
    assert not gate.enabled
    assert 'station hold' in gate.detail
    gate = _ready()
    gate.update_localization(True, True, True, False, 'different-session', 10.1)
    assert not gate.enabled
    assert 'session changed' in gate.detail
