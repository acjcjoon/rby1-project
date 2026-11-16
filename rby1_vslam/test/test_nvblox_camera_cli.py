"""Exercise the camera command without opening hardware or requiring ROS."""

import json
import glob
import os
from pathlib import Path
import subprocess

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'run_d435i_host.sh'


@pytest.fixture
def camera_command(tmp_path):
    setup = tmp_path / 'setup.bash'
    setup.write_text('# Camera CLI test: no ROS environment needed.\n')
    ros2 = tmp_path / 'ros2'
    ros2.write_text(
        '#!/usr/bin/python3\n'
        'import json, os, sys\n'
        'if sys.argv[1:3] == ["pkg", "prefix"]:\n'
        '    sys.exit(0)\n'
        'print("CAMERA_TEST_ARGUMENTS=" + json.dumps(sys.argv[1:]))\n'
        'print("CAMERA_TEST_PRELOAD=" + os.environ.get("LD_PRELOAD", ""))\n'
    )
    ros2.chmod(0o755)
    env = dict(os.environ)
    for name in ('RBY1_D435_HOST_CONFIG', 'RBY1_D435_SERIAL'):
        env.pop(name, None)
    env.update(RBY1_ROS_SETUP=str(setup), PATH=f'{tmp_path}:{env["PATH"]}')
    # A real harmless library lets the dynamic loader exercise LD_PRELOAD without
    # requiring RealSense on CI. The mock ros2 command never loads a camera.
    mock_library = glob.glob('/lib/*-linux-gnu/libm.so.6')[0]
    env['RBY1_REALSENSE_LIBRARY'] = mock_library
    env.pop('LD_PRELOAD', None)

    def run(*arguments, extra_env=None):
        result = subprocess.run(
            ['bash', str(SCRIPT), *arguments], env={**env, **(extra_env or {})},
            text=True, capture_output=True, check=False,
        )
        command = None
        for line in result.stdout.splitlines():
            if line.startswith('CAMERA_TEST_ARGUMENTS='):
                command = json.loads(line.split('=', 1)[1])
        return result, command

    return run


@pytest.mark.parametrize('mode', ['0', '1'])
def test_existing_vo_camera_command_keeps_projector_off(camera_command, mode):
    result, command = camera_command(mode)
    assert result.returncode == 0, result.stderr
    assert command[:3] == ['launch', 'realsense2_camera', 'rs_launch.py']
    parameters = dict(value.split(':=', 1) for value in command[3:])
    assert parameters['enable_depth'] == 'false'
    assert parameters['enable_color'] == 'false'
    assert parameters['depth_module.infra_profile'] == '640,480,30'
    assert parameters['enable_gyro'] == ('true' if mode == '1' else 'false')
    config = yaml.safe_load(Path(parameters['config_file']).read_text())
    assert config['depth_module.emitter_enabled'] == 0
    assert config['depth_module.emitter_on_off'] is False
    assert 'CAMERA_TEST_PRELOAD=\n' in result.stdout


@pytest.mark.parametrize('fps,extra', [
    (90, []), (60, ['--camera-fps', '60']), (60, ['--camera-fps=60']),
])
def test_nvblox_acquires_matching_native_depth_and_stereo(camera_command, fps, extra):
    result, command = camera_command('0', '--nvblox', *extra)
    assert result.returncode == 0, result.stderr
    parameters = dict(value.split(':=', 1) for value in command[3:])
    assert parameters['enable_depth'] == 'true'
    assert parameters['enable_color'] == 'false'
    assert parameters['enable_infra1'] == parameters['enable_infra2'] == 'true'
    assert parameters['enable_sync'] == parameters['align_depth.enable'] == 'false'
    assert parameters['depth_module.infra_profile'] == f'640,480,{fps}'
    assert parameters['depth_module.depth_profile'] == f'640,480,{fps}'
    config = yaml.safe_load(Path(parameters['config_file']).read_text())
    assert config['depth_module.emitter_enabled'] == 1
    assert config['depth_module.emitter_on_off'] is True
    assert config['depth_module.laser_power'] > 0
    assert f'split outputs={fps // 2} Hz each' in result.stdout
    assert 'CAMERA_TEST_PRELOAD=/lib/' in result.stdout


@pytest.mark.parametrize('fps', [60, 90])
def test_optional_color_preserves_native_depth_and_vo_streams(camera_command, fps):
    result, command = camera_command('0', '--nvblox', '--color', '--camera-fps', str(fps))
    assert result.returncode == 0, result.stderr
    parameters = dict(value.split(':=', 1) for value in command[3:])
    assert parameters['enable_color'] == 'true'
    assert parameters['rgb_camera.color_profile'] == '640,480,30'
    assert parameters['rgb_camera.color_format'] == 'RGB8'
    assert parameters['enable_depth'] == 'true'
    assert parameters['enable_infra1'] == parameters['enable_infra2'] == 'true'
    assert parameters['depth_module.depth_profile'] == f'640,480,{fps}'
    assert parameters['depth_module.infra_profile'] == f'640,480,{fps}'
    assert parameters['align_depth.enable'] == parameters['enable_sync'] == 'false'
    config = yaml.safe_load(Path(parameters['config_file']).read_text())
    assert config['color_qos'] == config['color_info_qos'] == 'SENSOR_DATA'


def test_passthrough_overrides_survive_nvblox_options(camera_command):
    result, command = camera_command(
        '--nvblox', 'depth_module.exposure:=8000', '--camera-fps', '60',
        'depth_module.enable_auto_exposure:=false',
    )
    assert result.returncode == 0, result.stderr
    assert command[-2:] == [
        'depth_module.exposure:=8000', 'depth_module.enable_auto_exposure:=false',
    ]


@pytest.mark.parametrize('arguments', [
    ['0', '--nvblox', '--camera-fps', '45'],
    ['0', '--nvblox', '--camera-fps'],
    ['0', '--nvblox', '--misspelled-flag'],
    ['2', '--nvblox'],
    ['0', '--nvblox', '--rsusb-library'],
])
def test_invalid_mode_or_rate_never_launches_camera(camera_command, arguments):
    result, command = camera_command(*arguments)
    assert result.returncode == 2
    assert command is None


def test_missing_rsusb_library_prevents_nonfunctional_splitter_mode(camera_command, tmp_path):
    result, command = camera_command('--nvblox', '--rsusb-library', str(tmp_path / 'missing.so'))
    assert result.returncode == 1
    assert command is None
    assert 'Missing RSUSB library:' in result.stderr


def test_library_flag_overrides_environment_and_preserves_existing_preload(camera_command):
    library = glob.glob('/lib/*-linux-gnu/libm.so.6')[0]
    result, command = camera_command('--nvblox', '--rsusb-library', library, extra_env={
        'RBY1_REALSENSE_LIBRARY': '/missing/from/environment.so', 'LD_PRELOAD': library,
    })
    assert result.returncode == 0, result.stderr
    assert command is not None
    assert f'CAMERA_TEST_PRELOAD={library}:{library}' in result.stdout
