"""Exercise launch wiring without ROS; GPU/component loading still needs Jetson."""

import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


PACKAGE = Path(__file__).resolve().parents[1]


class Record:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return str(context[self.name])


def module(name, **members):
    result = types.ModuleType(name)
    result.__dict__.update(members)
    return result


class LaunchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.shares = {'rby1_vslam': PACKAGE}
        for name, version in (('isaac_ros_visual_slam', '3.2.6'), ('realsense2_camera', '4.51.1')):
            folder = Path(self.temp.name) / name
            folder.mkdir()
            (folder / 'package.xml').write_text(f'<package><version>{version}</version></package>')
            self.shares[name] = folder
        stubs = {
            'ament_index_python': module('ament_index_python'),
            'ament_index_python.packages': module('ament_index_python.packages',
                get_package_share_directory=lambda name: str(self.shares[name])),
            'launch': module('launch', LaunchDescription=list),
            'launch.actions': module('launch.actions', DeclareLaunchArgument=Record,
                                     OpaqueFunction=Record, IncludeLaunchDescription=Record,
                                     RegisterEventHandler=Record),
            'launch.conditions': module('launch.conditions', IfCondition=Record),
            'launch.substitutions': module('launch.substitutions', LaunchConfiguration=Configuration),
            'launch.launch_description_sources': module('launch.launch_description_sources',
                                                         PythonLaunchDescriptionSource=Record),
            'launch.event_handlers': module('launch.event_handlers', OnShutdown=Record),
            'launch_ros': module('launch_ros'),
            'launch_ros.actions': module('launch_ros.actions', Node=Record, ComposableNodeContainer=Record),
            'launch_ros.descriptions': module('launch_ros.descriptions', ComposableNode=Record,
                                              ParameterFile=Record),
            'launch_ros.parameter_descriptions': module('launch_ros.parameter_descriptions', ParameterValue=Record),
            'nav2_common': module('nav2_common'),
            'nav2_common.launch': module('nav2_common.launch', RewrittenYaml=Record),
            'yaml': module('yaml'),
        }
        self.patcher = patch.dict(sys.modules, stubs)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.distro = patch.dict(os.environ, {'ROS_DISTRO': 'humble'})
        self.distro.start()
        self.addCleanup(self.distro.stop)

    def load(self, name):
        spec = importlib.util.spec_from_file_location('launch_under_test', PACKAGE / 'launch' / name)
        result = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(result)
        return result

    def defaults(self, launch):
        return {item.args[0]: item.kwargs['default_value'] for item in launch.generate_launch_description()
                if item.args and 'default_value' in item.kwargs}

    def test_isaac_component_uses_direct_camera_topics_and_supported_api(self):
        launch = self.load('isaac.launch.py')
        context = self.defaults(launch)
        context['tracking_mode'] = '1'
        container, monitor = launch._launch(context)[:2]
        component = container.kwargs['composable_node_descriptions'][0]
        parameters = component.kwargs['parameters'][-1]
        remaps = dict(component.kwargs['remappings'])
        self.assertTrue(parameters['enable_imu_fusion'])
        self.assertFalse(parameters['publish_odom_to_base_tf'])
        self.assertNotIn('tracking_mode', parameters)
        self.assertEqual(remaps['visual_slam/imu'], '/d435/d435/imu')
        self.assertEqual(remaps['visual_slam/image_0'], '/d435/d435/infra1/image_rect_raw')
        self.assertEqual(monitor.kwargs['executable'], 'tracking_monitor')
        self.assertFalse({'lab_host', 'port', 'start_bridge'} & set(context))

    def test_rgbd_is_rejected_before_component_creation(self):
        launch = self.load('isaac.launch.py')
        context = self.defaults(launch)
        context['tracking_mode'] = '2'
        with self.assertRaises(ValueError):
            launch._launch(context)

    def test_wrong_isaac_release_is_rejected(self):
        launch = self.load('isaac.launch.py')
        (self.shares['isaac_ros_visual_slam'] / 'package.xml').write_text(
            '<package><version>4.5.0</version></package>')
        with self.assertRaisesRegex(ValueError, 'release-3.2'):
            launch._launch(self.defaults(launch))

    def test_saved_map_mode_requires_valid_database_and_localization_monitor(self):
        launch = self.load('isaac.launch.py')
        context = self.defaults(launch)
        context.update(mode='localization', map_path=self.temp.name)
        with self.assertRaises(ValueError):
            launch._launch(context)
        (Path(self.temp.name) / 'map.mdb').write_bytes(b'fixture')
        _, monitor = launch._launch(context)[:2]
        self.assertTrue(monitor.kwargs['parameters'][0]['require_localized'])

    def test_realsense_legacy_driver_produces_preserved_topic_namespace(self):
        launch = self.load('d435i.launch.py')
        camera = launch._camera(self.defaults(launch))[0]
        self.assertEqual(camera.kwargs['namespace'], 'd435/d435')
        parameters = camera.kwargs['parameters'][-1]
        self.assertEqual(parameters['depth_module.profile'], '640,360,30')
        self.assertFalse(parameters['enable_depth'])
        self.assertNotIn('depth_module.infra_profile', parameters)

    def test_jetson_passes_only_declared_arguments_to_its_children(self):
        launch = self.load('jetson.launch.py')
        child_by_file = {'isaac.launch.py': self.load('isaac.launch.py'),
                         'd435i.launch.py': self.load('d435i.launch.py')}
        for action in launch.generate_launch_description():
            if 'launch_arguments' not in action.kwargs:
                continue
            child_name = Path(action.args[0].args[0]).name
            passed = dict(action.kwargs['launch_arguments'])
            self.assertTrue(set(passed) <= set(self.defaults(child_by_file[child_name])))

    def test_physical_stack_reuses_robot_nodes_and_passes_declared_arguments(self):
        launch = self.load('physical.launch.py')
        context = self.defaults(launch)
        context.update(start_driver='false', start_description='false', start_control='false',
                       start_ui='false')
        children = {'jetson.launch.py': self.load('jetson.launch.py'),
                    'navigation.launch.py': self.load('navigation.launch.py')}
        actions = launch._launch(context)
        self.assertEqual(len(actions), 2)
        self.assertFalse({'lab_host', 'port', 'bridge_params_file'} & set(context))
        for action in actions:
            child_name = Path(action.args[0].args[0]).name
            self.assertTrue(set(dict(action.kwargs['launch_arguments'])) <= set(self.defaults(children[child_name])))

    def test_physical_rgbd_fails_before_accessing_hardware_packages(self):
        launch = self.load('physical.launch.py')
        context = self.defaults(launch)
        context['tracking_mode'] = '2'
        with self.assertRaisesRegex(ValueError, 'camera recording'):
            launch._launch(context)

    def test_nav2_publishes_through_gate_and_never_owns_final_cmd_vel(self):
        launch = self.load('navigation.launch.py')
        actions = launch._launch(self.defaults(launch))
        smoother = next(action for action in actions if action.kwargs.get('executable') == 'velocity_smoother')
        self.assertEqual(dict(smoother.kwargs['remappings'])['cmd_vel_smoothed'], '/rby1/vslam/nav2_cmd_vel')
        gate = next(action for action in actions if action.kwargs.get('executable') == 'nav2_gate')
        self.assertEqual(gate.kwargs['parameters'][0]['cmd_raw_topic'], '/rby1/cmd_raw')
        for action in actions:
            self.assertNotIn('/rby1/cmd_vel', dict(action.kwargs.get('remappings', [])).values())


if __name__ == '__main__':
    unittest.main()
