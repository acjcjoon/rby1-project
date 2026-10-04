"""No-hardware checks: launcher routing and local UI release/timeout stops."""
import ast
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
BASH = os.environ.get('RBY1_TEST_BASH') or shutil.which('bash')


@unittest.skipUnless(BASH, 'Bash required')
class Launchers(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='.launcher-test-', dir=ROOT)
        self.addCleanup(self.temp.cleanup)
        setup = Path(self.temp.name) / 'setup.bash'
        self.setup = setup
        def shell_path(path):
            value = Path(path).as_posix()
            return '/' + value[0].lower() + value[2:] if len(value) > 1 and value[1] == ':' else value
        self.shell_path = shell_path
        ros2 = Path(self.temp.name) / 'ros2'
        self.ros2 = ros2
        ros2.write_text(f'#!{shell_path(BASH)}\nprintf "MOCK_ROS2 %s\\n" "$*"\n'
                        'printf "MOCK_LD_LIBRARY_PATH=%s\\n" "${LD_LIBRARY_PATH:-}"\n',
                        encoding='utf-8')
        ros2.chmod(0o755)
        setup.write_text(f'export PATH="{shell_path(self.temp.name)}:$PATH"\n', encoding='utf-8')
        rsusb_lib = Path(self.temp.name) / 'rsusb' / 'Release'
        rsusb_lib.mkdir(parents=True)
        (rsusb_lib / 'librealsense2.so.2.58').touch()
        self.rsusb_lib = rsusb_lib
        self.env = dict(os.environ, RBY1_ROS_SETUP=setup.as_posix(),
                        RBY1_WORKSPACE_SETUP=setup.as_posix(),
                        RBY1_RSUSB_LIB_DIR=rsusb_lib.as_posix(), DISPLAY=':0', ROS_DOMAIN_ID='17')

    def run_script(self, script, *args):
        return subprocess.run([BASH, str(ROOT / script), *args], env=self.env,
                              cwd=ROOT, capture_output=True, text=True,
                              encoding='utf-8', errors='replace', timeout=10)

    def test_four_routes(self):
        for mode in ('mobile_base', 'vslam'):
            for ui in ('web', 'upc'):
                with self.subTest(mode=mode, ui=ui):
                    args = ['lab_host:=192.0.2.1'] if mode == 'vslam' else []
                    result = self.run_script(f'run_{mode}_{ui}.sh', *args)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    target = 'physical.launch.py' if mode == 'vslam' else 'mobile_base_web.launch.py'
                    self.assertIn(target, result.stdout)
                    self.assertIn('ui_backend:=' + ('qt' if ui == 'upc' else 'web'), result.stdout)
                    self.assertIn('ROS_DOMAIN_ID=17', result.stdout)
                    if mode == 'vslam':
                        self.assertIn('enable_imu:=true', result.stdout)
                        self.assertIn('RealSense backend=' + self.rsusb_lib.as_posix(), result.stdout)
                        self.assertIn('MOCK_LD_LIBRARY_PATH=' + self.rsusb_lib.as_posix(), result.stdout)
                    else:
                        self.assertNotIn('enable_imu', result.stdout)

    def test_vslam_rejects_missing_rsusb_library(self):
        self.env['RBY1_RSUSB_LIB_DIR'] = str(Path(self.temp.name) / 'missing')
        result = self.run_script('run_vslam_upc.sh', 'lab_host:=192.0.2.1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Missing RSUSB librealsense', result.stderr)

    def test_imu_opt_out_is_forwarded_after_default(self):
        result = self.run_script('run_vslam_web.sh', 'lab_host:=192.0.2.1', 'enable_imu:=false')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertLess(result.stdout.index('enable_imu:=true'), result.stdout.index('enable_imu:=false'))

    def test_missing_host_and_ui_override_rejected(self):
        self.assertNotEqual(self.run_script('run_vslam_web.sh').returncode, 0)
        self.assertNotEqual(self.run_script('run_mobile_base_web.sh', 'ui_backend:=qt').returncode, 0)
        self.assertNotEqual(self.run_script('run_mobile_base_web.sh', 'lab_host:=192.0.2.1').returncode, 0)

    def test_help_needs_no_ros(self):
        self.env['RBY1_ROS_SETUP'] = '/nonexistent'
        for script in ('run_mobile_base_web.sh', 'run_mobile_base_upc.sh',
                       'run_vslam_web.sh', 'run_vslam_upc.sh', 'record_trial.sh'):
            self.assertEqual(self.run_script(script, '--help').returncode, 0)

    def test_qt_requires_desktop(self):
        self.env.pop('DISPLAY', None)
        self.env.pop('WAYLAND_DISPLAY', None)
        self.assertNotEqual(self.run_script('run_mobile_base_upc.sh').returncode, 0)

    def test_recording_finalizes_and_copies_waypoints(self):
        # Mock recorder finishes normally; no ROS nodes or hardware are used.
        self.ros2.write_text(self.ros2.read_text().splitlines()[0] + '''
case "$1 $2" in
  'bag record')
    while (($#)); do
      if [[ "$1" == -o ]]; then mkdir -p "$2"; break; fi
      shift
    done
    printf 'Subscribed to /rby1/odom\\n'
    ;;
  'node list') printf '/rby1/test_node\\n' ;;
  'topic list') printf '/rby1/odom [nav_msgs/msg/Odometry]\\n' ;;
  'param dump') printf 'test_parameters: {}\\n' ;;
  'bag info') printf 'Mock bag finalized\\n' ;;
esac
''', encoding='utf-8')
        with self.setup.open('a', encoding='utf-8') as handle:
            handle.write('''
python3() { if (($# > 3)); then shift 2; "$@"; fi; }
timeout() { shift; "$@"; }
''')
        points = Path(self.temp.name) / 'points.yaml'
        points.write_text('points: []\n')
        output = Path(self.temp.name) / 'trials'
        result = self.run_script('record_trial.sh', '--role', 'upc', '--run-id', 'trial01',
                                 '--output', output.relative_to(ROOT).as_posix(),
                                 '--waypoints', points.relative_to(ROOT).as_posix())
        self.assertEqual(result.returncode, 0, result.stderr)
        trial, = output.iterdir()
        self.assertTrue((trial / 'bag').is_dir())
        self.assertIn('finalized', (trial / 'bag_info.txt').read_text())
        self.assertEqual((trial / 'waypoints_end.yaml').read_text(), points.read_text())
        self.assertIn('ROS_DOMAIN_ID=17', (trial / 'environment.txt').read_text())
        self.assertTrue((trial / 'params_end' / '_rby1_test_node.yaml').is_file())

    def test_shell_syntax(self):
        for script in ('scripts/launch_operator.sh', 'run_mobile_base_web.sh',
                       'run_mobile_base_upc.sh', 'run_vslam_web.sh', 'run_vslam_upc.sh', 'record_trial.sh'):
            result = subprocess.run([BASH, '-n', str(ROOT / script)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


class MobileStop(unittest.TestCase):
    def setUp(self):
        # Exercise window logic without importing ROS/Qt or connecting hardware.
        tree = ast.parse((ROOT / 'rby1_web/rby1_web/mobile_base_qt.py').read_text())
        window = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'MobileWindow')
        scope = {'QWidget': object, 'rclpy': types.SimpleNamespace(ok=lambda: True, spin_once=lambda *a, **k: None)}
        exec(compile(ast.Module(body=[window], type_ignores=[]), '<window>', 'exec'), scope)
        self.window = scope['MobileWindow'].__new__(scope['MobileWindow'])
        self.sent = []
        self.fresh = True
        self.active = True
        self.window.node = types.SimpleNamespace(fresh=lambda: self.fresh, status='ready',
                                                send=lambda *args: self.sent.append(args))
        self.window.status = types.SimpleNamespace(setText=lambda text: None)
        self.window.isActiveWindow = lambda: self.active
        self.window.linear = types.SimpleNamespace(value=lambda: 0.02)
        self.window.angular = types.SimpleNamespace(value=lambda: 0.08)
        self.window.velocity = None

    def test_hold_release(self):
        self.window.start((1, 0, 0))
        self.window.tick()
        self.assertEqual(self.sent[-1], ('set_velocity', {'vx': 0.02, 'vy': 0.0, 'wz': 0.0}))
        self.window.stop()
        self.window.tick()
        self.assertEqual(self.sent[-1], ('stop',))
        self.assertIsNone(self.window.velocity)

    def test_connection_loss_and_focus_loss_stop_without_resuming(self):
        for cause in ('fresh', 'active'):
            self.fresh = self.active = True
            self.window.start((1, 0, 0))
            setattr(self, cause, False)
            self.window.tick()
            self.assertEqual(self.sent[-1], ('stop',))
            self.fresh = self.active = True
            self.window.tick()
            self.assertIsNone(self.window.velocity)


if __name__ == '__main__':
    unittest.main()
