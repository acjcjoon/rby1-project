"""ROS-free checks for the physical mecanum Nav2 configuration."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


CONFIG = Path(__file__).resolve().parents[1] / 'config/navigation.yaml'
PACKAGE = Path(__file__).resolve().parents[1] / 'package.xml'


def load():
    return yaml.safe_load(CONFIG.read_text(encoding='utf-8'))


def test_rby1_rectangular_footprint_is_used_by_both_costmaps():
    data = load()
    expected = [[0.3475, 0.3], [0.3475, -0.3], [-0.3475, -0.3], [-0.3475, 0.3]]
    resolutions = []
    for name in ('local_costmap', 'global_costmap'):
        params = data[name][name]['ros__parameters']
        assert 'robot_radius' not in params
        assert yaml.safe_load(params['footprint']) == expected
        assert params['footprint_padding'] == 0.05
        resolutions.append(params['resolution'])
    assert resolutions[0] == resolutions[1]
    assert resolutions[0] <= 0.05


def test_mppi_uses_humble_omni_model_and_lateral_sampling():
    controller = load()['controller_server']['ros__parameters']
    follow = controller['FollowPath']

    assert follow['plugin'] == 'nav2_mppi_controller::MPPIController'
    # Humble accepts these exact case-sensitive model names, not the newer
    # motion-model plugin namespace used by later Nav2 releases.
    assert follow['motion_model'] == 'Omni'
    assert follow['vx_min'] < 0.0 < follow['vx_max']
    assert follow['vy_max'] > 0.0
    assert follow['vy_std'] > 0.0
    assert follow['wz_max'] > 0.0

    # A 20 Hz controller with model_dt 0.05 enables Humble's control-sequence
    # shift. A model step shorter than the controller period throws at startup.
    assert math.isclose(
        follow['model_dt'], 1.0 / controller['controller_frequency'], abs_tol=1e-9)


def test_mppi_limits_fit_smoother_and_prediction_fits_local_costmap():
    data = load()
    controller = data['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    smoother = data['velocity_smoother']['ros__parameters']
    local = data['local_costmap']['local_costmap']['ros__parameters']

    assert follow['vx_max'] <= smoother['max_velocity'][0]
    assert follow['vx_min'] >= smoother['min_velocity'][0]
    assert follow['vy_max'] <= min(
        smoother['max_velocity'][1], abs(smoother['min_velocity'][1]))
    assert follow['wz_max'] <= min(
        smoother['max_velocity'][2], abs(smoother['min_velocity'][2]))

    max_planar_speed = math.hypot(
        max(abs(follow['vx_min']), follow['vx_max']), follow['vy_max'])
    assert max_planar_speed * follow['model_dt'] <= local['resolution']

    footprint = yaml.safe_load(local['footprint'])
    padded_radius = max(math.hypot(x, y) for x, y in footprint) + local['footprint_padding']
    usable_radius = min(local['width'], local['height']) / 2.0 - padded_radius
    predicted_distance = max_planar_speed * follow['time_steps'] * follow['model_dt']
    assert predicted_distance <= usable_radius


def test_mppi_uses_full_footprint_and_allows_same_yaw_shuttle_reverse():
    follow = load()['controller_server']['ros__parameters']['FollowPath']
    critics = follow['critics']

    assert 'CostCritic' in critics
    assert follow['CostCritic']['enabled'] is True
    assert follow['CostCritic']['consider_footprint'] is True
    # With one inflation layer, Humble's empty/default selector reliably uses
    # the last layer without depending on its fully namespaced runtime name.
    assert not follow['CostCritic'].get('inflation_layer_name', '')
    assert 'GoalAngleCritic' in critics
    assert 'PathAngleCritic' in critics
    assert follow['PathAngleCritic']['forward_preference'] is False
    assert 'PreferForwardCritic' not in critics
    assert 'TwirlingCritic' in critics
    assert follow['visualize'] is False

    twirling = follow['TwirlingCritic']
    assert {'cost_power', 'cost_weight'} <= twirling.keys()
    assert {'twirling_cost_power', 'twirling_cost_weight'}.isdisjoint(twirling)


def test_humble_mppi_config_has_no_dwb_or_modern_acceleration_keys():
    follow = load()['controller_server']['ros__parameters']['FollowPath']
    incompatible = {
        'debug_trajectory_details', 'min_vel_x', 'min_vel_y', 'max_vel_x',
        'max_vel_y', 'max_vel_theta', 'vx_samples', 'vy_samples',
        'vtheta_samples', 'sim_time', 'linear_granularity',
        'angular_granularity', 'acc_lim_x', 'acc_lim_y', 'acc_lim_theta',
        'decel_lim_x', 'decel_lim_y', 'decel_lim_theta',
        # Acceleration constraints were added after Humble; the downstream
        # velocity_smoother owns these limits in this stack.
        'ax_max', 'ax_min', 'ay_max', 'ay_min', 'az_max',
    }
    assert incompatible.isdisjoint(follow)
    assert 'omni' not in follow


def test_mppi_runtime_dependency_is_declared():
    dependencies = [node.text for node in ET.parse(PACKAGE).getroot().findall('exec_depend')]
    assert 'nav2_mppi_controller' in dependencies


def test_theta_and_strict_goal_settings_are_explicit():
    data = load()
    controller = data['controller_server']['ros__parameters']
    planner = data['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_theta_star_planner/ThetaStarPlanner'
    assert controller['goal_checker']['plugin'] == 'nav2_controller::StoppedGoalChecker'
    assert controller['goal_checker']['xy_goal_tolerance'] == 0.01
    assert math.isclose(
        controller['goal_checker']['yaw_goal_tolerance'], math.radians(1), abs_tol=1e-7)
    assert controller['goal_checker']['stateful'] is False
    assert controller['goal_checker']['trans_stopped_velocity'] == 0.02
    assert controller['goal_checker']['rot_stopped_velocity'] == 0.02
