"""Save, load or export the static nvblox 3.2 map from its container.

The .nvblx layer cake, a PLY mesh and a cuVSLAM database are different artifacts.
The existing map_tool owns the cuVSLAM database. This tool never changes TF or
localization, and only reloads an odom map in the localization session that saved
it. A new wheel odometry origin requires rebuilding the nvblox map.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

from .map_tool import _call, _spin_until, positive_float


def build_parser():
    parser = argparse.ArgumentParser(
        description='Save/load the static nvblox map or export a PLY mesh (Isaac ROS 3.2).',
        epilog='Run inside the nvblox container with its ROS_DOMAIN_ID. Stop the base, '
        'disable navigation, and wait for cancellation before running. '
        'Persistent maps belong under /workspaces/isaac_ros-dev/maps. '
        'Do not run concurrent map operations.',
    )
    commands = parser.add_subparsers(dest='command', required=True)
    for command in ('save', 'load', 'mesh'):
        sub = commands.add_parser(command)
        sub.add_argument('--path', required=True,
                         help='Absolute container file path (.nvblx for save/load; .ply for mesh)')
        sub.add_argument('--service-prefix', default='/nvblox_node')
        sub.add_argument('--localization-status-topic',
                         default='/rby1/vslam/localization_status')
        sub.add_argument('--service-timeout', type=positive_float, default=15.0)
        sub.add_argument('--timeout', type=positive_float, default=120.0,
                         help='Stop-state and service response timeout in seconds')
    return parser


def metadata_path(path):
    return Path(str(path) + '.json')


def validate_path(value, command):
    path = Path(value)
    if not path.is_absolute():
        raise ValueError('--path must be absolute inside the nvblox container')
    suffix = '.ply' if command == 'mesh' else '.nvblx'
    if not path.suffix:
        path = Path(str(path) + suffix)
    if path.suffix.lower() != suffix:
        raise ValueError('{} requires a {} file path'.format(command, suffix))
    if command == 'load':
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError('load needs an existing, nonempty .nvblx file')
        if not metadata_path(path).is_file():
            raise ValueError('load needs the .nvblx.json session metadata created by this tool')
    else:
        if not path.parent.is_dir():
            raise ValueError('create the persistent parent directory first: ' + str(path.parent))
        if path.exists() or path.is_symlink():
            raise ValueError('output already exists; choose a new map name: ' + str(path))
        if command == 'save' and metadata_path(path).exists():
            raise ValueError('map metadata already exists; choose a new map name')
    return path


class StopState:
    """Reuse the existing localization node's stationary/idle decisions."""

    def __init__(self):
        self.state = {}
        self.received = -math.inf

    def update(self, data, now):
        try:
            state = json.loads(data)
        except (TypeError, ValueError):
            state = None
        self.state = state if isinstance(state, dict) else {}
        self.received = now

    def ready(self, now):
        return (0.0 <= now - self.received <= 0.75
                and self.state.get('stationary_ready') is True
                and self.state.get('navigation_idle') is True
                and self.state.get('correction_pending') is False
                and isinstance(self.state.get('session_id'), str)
                and bool(self.state['session_id'])
                and self.state.get('odom_frame') == 'odom')


def validate_metadata(metadata, session_id, parameters):
    if not isinstance(metadata, dict) or metadata.get('schema_version') != 1:
        raise ValueError('unsupported nvblox map metadata')
    if metadata.get('localization_session_id') != session_id:
        raise ValueError('saved map belongs to a different localization/odom session; '
                         'rebuild the nvblox map after a wheel origin or localization restart')
    if metadata.get('global_frame') != parameters['global_frame']:
        raise ValueError('saved map frame differs from the running nvblox frame')
    saved_voxel_size = metadata.get('voxel_size')
    if (not isinstance(saved_voxel_size, (int, float))
            or not math.isfinite(saved_voxel_size)
            or not math.isclose(saved_voxel_size, parameters['voxel_size'], rel_tol=1e-6)):
        raise ValueError('saved voxel size differs from the running nvblox voxel size')
    if metadata.get('mapping_type') != parameters['mapping_type']:
        raise ValueError('saved mapping type differs from the running nvblox mapping type')


def read_parameters(rclpy, node, args):
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters

    request = GetParameters.Request()
    request.names = ['global_frame', 'voxel_size', 'mapping_type']
    response = _call(rclpy, node, GetParameters, args.service_prefix + '/get_parameters',
                     request, args.service_timeout, args.timeout)
    values = response.values
    if (len(values) != 3 or values[0].type != ParameterType.PARAMETER_STRING
            or values[1].type != ParameterType.PARAMETER_DOUBLE
            or values[2].type != ParameterType.PARAMETER_STRING):
        raise RuntimeError('nvblox global_frame, voxel_size and mapping_type parameters unavailable')
    parameters = dict(global_frame=values[0].string_value,
                      voxel_size=values[1].double_value, mapping_type=values[2].string_value)
    if parameters['global_frame'] != 'odom':
        raise ValueError('this experiment map tool requires nvblox global_frame=odom')
    if not math.isfinite(parameters['voxel_size']) or parameters['voxel_size'] <= 0:
        raise ValueError('nvblox voxel_size must be finite and positive')
    return parameters


def perform_operation(rclpy, node, args, stopped, parameters, service_type):
    suffix = {'save': '/save_map', 'load': '/load_map', 'mesh': '/save_ply'}[args.command]
    name = args.service_prefix + suffix
    client = node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=args.service_timeout):
            raise TimeoutError('Service not available: ' + name)
        print('Waiting for stationary base, navigation disabled and cancellation complete...',
              flush=True)
        _spin_until(rclpy, node, lambda: stopped.ready(time.monotonic()), args.timeout,
                    'No fresh stationary/idle localization status; stop the base, disable '
                    'navigation, and finish station correction')
        session_id = stopped.state['session_id']
        # Validate immediately before the request, after service discovery and stop checks.
        args.path = validate_path(str(args.path), args.command)
        if args.command == 'load':
            with metadata_path(args.path).open() as stream:
                validate_metadata(json.load(stream), session_id, parameters)
        request = service_type.Request()
        request.file_path = str(args.path)
        future = client.call_async(request)
        _spin_until(rclpy, node, future.done, args.timeout,
                    'No response from {}; operation may still be running. '
                    'Do not retry until its server logs confirm completion.'.format(name))
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('nvblox rejected {}; inspect its logs'.format(args.command))
        if args.command != 'load':
            if not args.path.is_file() or args.path.stat().st_size == 0:
                raise RuntimeError('service succeeded but output is absent/empty; run the tool '
                                   'in the same container/filesystem as nvblox')
        if stopped.state.get('session_id') != session_id:
            raise RuntimeError('operation completed during a localization session change; '
                               'the map must not be used for navigation')
        if args.command == 'save':
            metadata = dict(schema_version=1, saved_at_utc=datetime.now(timezone.utc).isoformat(),
                            localization_session_id=session_id, artifact='static_mapper_layer_cake',
                            **parameters)
            # Exclusive creation protects an earlier map's coordinate metadata.
            with metadata_path(args.path).open('x') as stream:
                json.dump(metadata, stream, indent=2, allow_nan=False)
                stream.write('\n')
            print('Static nvblox map saved: ' + str(args.path))
            print('Session metadata: ' + str(metadata_path(args.path)))
            print('Save cuVSLAM separately with map_tool; dynamic layers are not in this file.')
        elif args.command == 'mesh':
            print('PLY mesh exported: ' + str(args.path))
            print('PLY is for inspection; load requires the .nvblx map and its metadata.')
        else:
            print('Static nvblox map loaded in the same odom session: ' + str(args.path))
            print('Verify ESDF and costmaps in RViz before enabling navigation.')
        return 0
    finally:
        node.destroy_client(client)


def run(args, ros_args=None):
    import rclpy
    from nvblox_msgs.srv import FilePath
    from std_msgs.msg import String

    rclpy.init(args=ros_args or [])
    node = rclpy.create_node('rby1_nvblox_map_tool')
    stopped = StopState()
    try:
        node.create_subscription(String, args.localization_status_topic,
                                 lambda msg: stopped.update(msg.data, time.monotonic()), 5)
        parameters = read_parameters(rclpy, node, args)
        return perform_operation(rclpy, node, args, stopped, parameters, FilePath)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if '--ros-args' in argv:
        index = argv.index('--ros-args')
        argv, ros_args = argv[:index], argv[index:]
    args = build_parser().parse_args(argv)
    try:
        args.service_prefix = args.service_prefix.rstrip('/')
        if (not args.service_prefix.startswith('/')
                or not args.localization_status_topic.startswith('/')):
            raise ValueError('service prefix and localization status topic must be absolute ROS names')
        args.path = validate_path(args.path, args.command)
        return run(args, ros_args)
    except KeyboardInterrupt:
        print('Interrupted; the server-side map operation may still be running.', file=sys.stderr)
        return 130
    except ImportError as exc:
        print('Run nvblox_map_tool in the Isaac ROS container after sourcing its workspace: '
              + str(exc), file=sys.stderr)
        return 2
    except (ValueError, RuntimeError, TimeoutError, OSError) as exc:
        print('nvblox_map_tool: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
