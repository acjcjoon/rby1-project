"""Exercise bridge payload handling without installing ROS on the test host.

Small dataclass stand-ins model standard ROS message fields only. These tests
verify our validation/conversion boundary and raw-byte preservation; they do not
claim to validate generated ROS Python setters, DDS, QoS, or Isaac ROS runtime.
"""

from dataclasses import asdict, dataclass, field, is_dataclass
import copy
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rby1_vslam.wire import Packet, ProtocolError, decode_body, decode_header, encode_packet, HEADER


@dataclass
class Stamp:
    sec: int = 0
    nanosec: int = 0


@dataclass
class Header:
    stamp: Stamp = field(default_factory=Stamp)
    frame_id: str = ''


@dataclass
class Vector:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0


@dataclass
class Transform:
    translation: Vector = field(default_factory=Vector)
    rotation: Quaternion = field(default_factory=Quaternion)


@dataclass
class TransformStamped:
    header: Header = field(default_factory=Header)
    child_frame_id: str = ''
    transform: Transform = field(default_factory=Transform)


@dataclass
class ROI:
    x_offset: int = 0
    y_offset: int = 0
    height: int = 0
    width: int = 0
    do_rectify: bool = False


@dataclass
class CameraInfo:
    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    distortion_model: str = ''
    d: list = field(default_factory=list)
    k: list = field(default_factory=lambda: [0.0] * 9)
    r: list = field(default_factory=lambda: [0.0] * 9)
    p: list = field(default_factory=lambda: [0.0] * 12)
    binning_x: int = 0
    binning_y: int = 0
    roi: ROI = field(default_factory=ROI)


@dataclass
class Image:
    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    encoding: str = ''
    is_bigendian: int = 0
    step: int = 0
    data: list = field(default_factory=list)


@dataclass
class Imu:
    header: Header = field(default_factory=Header)
    orientation: Quaternion = field(default_factory=Quaternion)
    orientation_covariance: list = field(default_factory=lambda: [0.0] * 9)
    angular_velocity: Vector = field(default_factory=Vector)
    angular_velocity_covariance: list = field(default_factory=lambda: [0.0] * 9)
    linear_acceleration: Vector = field(default_factory=Vector)
    linear_acceleration_covariance: list = field(default_factory=lambda: [0.0] * 9)


@dataclass
class TFMessage:
    transforms: list = field(default_factory=list)


@dataclass
class ClockMessage:
    clock: Stamp = field(default_factory=Stamp)


def assign_fields(message, values):
    for key, value in values.items():
        current = getattr(message, key)
        if is_dataclass(current):
            assign_fields(current, value)
        elif isinstance(message, TFMessage) and key == 'transforms':
            result = []
            for item in value:
                transform = TransformStamped()
                assign_fields(transform, item)
                result.append(transform)
            message.transforms = result
        else:
            setattr(message, key, copy.deepcopy(value))


@pytest.fixture
def bridge(monkeypatch):
    def install(name, **attributes):
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    install('rclpy')
    install('rclpy.clock', Clock=object, ClockType=SimpleNamespace(STEADY_TIME=1))
    install('rclpy.node', Node=object)
    install('rclpy.qos', DurabilityPolicy=object, QoSProfile=object, ReliabilityPolicy=object)
    install('rosidl_runtime_py')
    install('rosidl_runtime_py.convert', message_to_ordereddict=asdict)
    install('rosidl_runtime_py.set_message', set_message_fields=assign_fields)
    for package, attributes in {
        'nav_msgs': {'Odometry': object},
        'geometry_msgs': {'TransformStamped': TransformStamped},
        'rosgraph_msgs': {'Clock': ClockMessage},
        'sensor_msgs': {'CameraInfo': CameraInfo, 'Image': Image, 'Imu': Imu},
        'std_msgs': {'String': object},
        'tf2_msgs': {'TFMessage': TFMessage},
        'builtin_interfaces': {'Time': Stamp},
    }.items():
        install(package)
        install(package + '.msg', **attributes)
    path = Path(__file__).resolve().parents[1] / 'rby1_vslam' / 'bridge_node.py'
    spec = importlib.util.spec_from_file_location('rby1_vslam._bridge_test_only', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stereo_messages():
    result = []
    for side, nanosec in (('infra1', 123456789), ('infra2', 125456789)):
        header = Header(Stamp(123, nanosec), 'd435_' + side + '_optical_frame')
        image = Image(copy.deepcopy(header), height=2, width=3, encoding='mono8',
                      step=4, data=bytes([0, 127, 255, 17, 5, 0, 128, 19]))
        info = CameraInfo(copy.deepcopy(header), height=2, width=3, distortion_model='plumb_bob',
                          d=[0.1, -0.2, 0.0, 0.0, 0.0],
                          k=[100.0, 0.0, 1.5, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0],
                          p=[100.0, 0.0, 1.5, -5.0, 0.0, 100.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                          roi=ROI(width=3, height=2))
        result.append((image, info))
    return result[0][0], result[1][0], result[0][1], result[1][1]


def test_stereo_roundtrip_preserves_pixels_padding_stamps_and_calibration(bridge):
    originals = stereo_messages()
    outbound = bridge._stereo_packet(*originals)
    encoded = encode_packet(Packet(outbound.kind, outbound.payload, outbound.blob, 'test-session'))
    metadata_size, blob_size = decode_header(encoded[:HEADER.size])
    incoming = decode_body(encoded[HEADER.size:HEADER.size + metadata_size], encoded[-blob_size:])
    decoded = bridge._decode_stereo(incoming, slop_ns=5_000_000)
    for before, after in zip(originals, decoded):
        before_fields, after_fields = asdict(before), asdict(after)
        if isinstance(before, Image):
            assert bytes(before_fields.pop('data')) == bytes(after_fields.pop('data'))
        assert before_fields == after_fields
    assert 'data' not in incoming.payload['left']['message']
    assert decoded[0].header.stamp.nanosec != decoded[1].header.stamp.nanosec


@pytest.mark.parametrize('mutation', [
    lambda value: value.pop('k'),
    lambda value: value['header'].update(unexpected=True),
    lambda value: value.update(k=[1.0] * 8),
    lambda value: value['roi'].update(do_rectify=1),
    lambda value: value['header']['stamp'].update(nanosec=0.5),
    lambda value: value.update(distortion_model={'unexpected': 'object'}),
])
def test_typed_camera_metadata_rejects_malformed_fields(bridge, mutation):
    value = asdict(stereo_messages()[2])
    mutation(value)
    with pytest.raises(ProtocolError):
        bridge._from_dict(CameraInfo, value)


def test_imu_signed_unknown_covariance_and_fractional_measurement_roundtrip(bridge):
    message = Imu(Header(Stamp(2, 987654321), 'd435_imu_optical_frame'),
                  orientation_covariance=[-1.0] + [0.0] * 8,
                  angular_velocity=Vector(-0.1, 0.2, -0.3),
                  linear_acceleration=Vector(0.001, -0.002, 9.81))
    assert bridge._from_dict(Imu, bridge._to_dict(message)) == message


@pytest.mark.parametrize('change', ['wrong_stamp', 'wrong_frame', 'short_data', 'uncalibrated', 'oversize_slop'])
def test_stereo_rejects_inconsistent_input(bridge, change):
    left, right, left_info, right_info = stereo_messages()
    if change == 'wrong_stamp':
        left_info.header.stamp.nanosec += 1
    elif change == 'wrong_frame':
        left_info.header.frame_id = 'unrelated_camera'
    elif change == 'short_data':
        left.data = left.data[:-1]
    elif change == 'uncalibrated':
        left_info.k[0] = 0.0
    elif change == 'oversize_slop':
        right.header.stamp.nanosec += 10_000_000
        right_info.header.stamp.nanosec += 10_000_000
    with pytest.raises(ProtocolError):
        packet = bridge._stereo_packet(left, right, left_info, right_info)
        bridge._decode_stereo(packet, slop_ns=5_000_000)


def tf(parent, child):
    return TransformStamped(Header(frame_id=parent), child)


def test_tf_filter_follows_camera_root_and_excludes_mount_robot_and_cycles(bridge):
    node = SimpleNamespace(cfg={'camera_root_frame': 'd435_link', 'camera_frame_prefix': 'd435_'})
    selected = bridge.BridgeNode._camera_transforms(node, [
        tf('link_head_2', 'd435_camera_center'), tf('d435_camera_center', 'd435_link'),
        tf('d435_infra1_frame', 'd435_infra1_optical_frame'),
        tf('d435_link', 'd435_infra1_frame'), tf('odom', 'base'),
        tf('d435_cycle_a', 'd435_cycle_b'), tf('d435_cycle_b', 'd435_cycle_a'),
        tf('d435_link', 'd405_link'),
    ])
    assert [transform.child_frame_id for transform in selected] == [
        'd435_infra1_frame', 'd435_infra1_optical_frame']


def fake_receiver(bridge, now_ns, forward_clock=False):
    published = []
    node = SimpleNamespace(
        cfg=dict(bridge.DEFAULTS, forward_clock=forward_clock),
        _synchronizer=SimpleNamespace(slop_ns=5_000_000),
        _forwarded_clock_ns=None, _lab_input_session='', _lab_first_image_stamp=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now_ns)),
        _publishers={name: SimpleNamespace(publish=published.append)
                     for name in ('left', 'right', 'left_info', 'right_info', 'clock')},
    )
    return node, published


def test_lab_rejects_stale_acquisition_time_before_any_image_is_published(bridge):
    outbound = bridge._stereo_packet(*stereo_messages())
    packet = Packet(outbound.kind, outbound.payload, outbound.blob, 'test-session')
    node, published = fake_receiver(bridge, now_ns=124_000_000_000)
    with pytest.raises(ProtocolError, match='synchronize UPC/LAB clocks'):
        bridge.BridgeNode._publish_received(node, packet)
    assert published == []


def test_lab_forwarded_clock_is_used_before_ros_clock_callback(bridge):
    outbound = bridge._stereo_packet(*stereo_messages())
    packet = Packet(outbound.kind, outbound.payload, outbound.blob, 'test-session')
    node, published = fake_receiver(bridge, now_ns=0, forward_clock=True)
    clock_packet = Packet('clock', {'clock': {'sec': 123, 'nanosec': 140000000}}, session_id='test-session')
    bridge.BridgeNode._publish_received(node, clock_packet)
    bridge.BridgeNode._publish_received(node, packet)
    assert len(published) == 5
    assert node._lab_input_session == 'test-session'
    assert node._lab_first_image_stamp == 123_123456789


def tracking_receiver(bridge, require_localized=False):
    return SimpleNamespace(
        cfg=dict(bridge.DEFAULTS, require_localized=require_localized),
        _tracking_state=1, _tracking_received_at=10.0,
        _localized=False, _localization_received_at=None,
        _requires_localization=require_localized,
    )


def test_saved_map_tracking_requires_fresh_confirmed_localization(bridge):
    node = tracking_receiver(bridge, require_localized=True)
    assert not bridge.BridgeNode._tracking_fields(node, 10.1, True)['tracking_ok']
    node._localized = True
    node._localization_received_at = 10.0
    assert bridge.BridgeNode._tracking_fields(node, 10.1, True)['tracking_ok']
    node._tracking_received_at = 11.1
    assert not bridge.BridgeNode._tracking_fields(node, 11.1, True)['tracking_ok']
    node._localization_received_at = 11.1
    assert bridge.BridgeNode._tracking_fields(node, 11.1, True)['tracking_ok']
    assert not bridge.BridgeNode._tracking_fields(node, 11.1, False)['tracking_ok']


def test_mapping_tracking_does_not_require_existing_map_flag(bridge):
    node = tracking_receiver(bridge)
    fields = bridge.BridgeNode._tracking_fields(node, 10.1, True)
    assert fields['tracking_ok']
    assert not fields['localized']
    assert not bridge.BridgeNode._tracking_fields(node, 10.6, True)['tracking_ok']


def test_upc_obeys_lab_localization_requirement_and_preserves_diagnostic_age(bridge):
    node = tracking_receiver(bridge)
    bridge.BridgeNode._publish_received(node, Packet('tracking_status', {
        'vo_state': 1, 'localized': True, 'require_localized': True, 'localization_age_sec': 2.0,
    }))
    fields = bridge.BridgeNode._tracking_fields(node, node._tracking_received_at, True)
    assert fields['require_localized']
    assert fields['localization_age_sec'] == pytest.approx(2.0)
    assert not fields['tracking_ok']


@pytest.mark.parametrize('mutation', [
    lambda value: value.update(vo_state=True),
    lambda value: value.update(localized='Yes'),
    lambda value: value.update(require_localized=1),
    lambda value: value.update(localization_age_sec=-1),
    lambda value: value.pop('require_localized'),
])
def test_upc_rejects_malformed_tracking_flags(bridge, mutation):
    payload = {'vo_state': 1, 'localized': True, 'require_localized': True, 'localization_age_sec': 0.1}
    mutation(payload)
    with pytest.raises(ProtocolError):
        bridge.BridgeNode._publish_received(tracking_receiver(bridge), Packet('tracking_status', payload))


def test_lab_diagnostic_localization_uses_only_visual_slam_source(bridge):
    node = tracking_receiver(bridge, require_localized=True)
    other = SimpleNamespace(hardware_id='other', name='other', values=[
        SimpleNamespace(key='localized_in_exist_map', value='Yes')])
    bridge.BridgeNode._on_diagnostics(node, SimpleNamespace(status=[other]))
    assert not node._localized
    actual = SimpleNamespace(hardware_id='visual_slam', name='Visual Slam Diagnostics', values=[
        SimpleNamespace(key='localized_in_exist_map', value='Yes')])
    bridge.BridgeNode._on_diagnostics(node, SimpleNamespace(status=[actual]))
    assert node._localized and node._localization_received_at is not None
    actual.values[0].value = 'No'
    bridge.BridgeNode._on_diagnostics(node, SimpleNamespace(status=[actual]))
    assert not node._localized
