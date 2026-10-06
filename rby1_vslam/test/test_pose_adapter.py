"""ROS-stubbed checks for PoseAdapter FIFO, retry and drop behavior."""

from dataclasses import dataclass, field
import copy
import importlib.util
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rby1_vslam.pose_queue import PoseQueue


@dataclass
class Stamp:
    sec: int = 0
    nanosec: int = 0


@dataclass
class Header:
    stamp: Stamp = field(default_factory=Stamp)
    frame_id: str = 'vslam_odom'


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
    w: float = 1.0


class Odometry:
    def __init__(self):
        self.header = Header()
        self.child_frame_id = 'd435_link'
        self.pose = SimpleNamespace(pose=SimpleNamespace(
            position=Vector(), orientation=Quaternion()), covariance=[0.0] * 36)
        self.twist = SimpleNamespace(covariance=[0.0] * 36)


class TransformStamped:
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ''
        self.transform = SimpleNamespace(
            translation=Vector(), rotation=Quaternion())


class FakeTime:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, stamp):
        return cls(stamp.sec * 1_000_000_000 + stamp.nanosec)


class TransformException(Exception):
    pass


@pytest.fixture
def pose_adapter_module(monkeypatch):
    def install(name, **attributes):
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    install('geometry_msgs')
    install('geometry_msgs.msg', TransformStamped=TransformStamped)
    install('nav_msgs')
    install('nav_msgs.msg', Odometry=Odometry)
    install('std_msgs')
    install('std_msgs.msg', String=SimpleNamespace)
    install('rclpy', init=lambda: None, spin=lambda node: None,
            ok=lambda: False, shutdown=lambda: None)
    install('rclpy.clock', Clock=object, ClockType=SimpleNamespace(STEADY_TIME=1))
    install('rclpy.duration', Duration=lambda **kwargs: kwargs)
    install('rclpy.node', Node=object)
    install('rclpy.time', Time=FakeTime)
    install('rclpy.qos', QoSProfile=object,
            ReliabilityPolicy=SimpleNamespace(RELIABLE=1))
    install('tf2_ros', Buffer=object, TransformBroadcaster=object,
            TransformException=TransformException, TransformListener=object)
    path = Path(__file__).resolve().parents[1] / 'rby1_vslam' / 'pose_adapter.py'
    spec = importlib.util.spec_from_file_location(
        'rby1_vslam._pose_adapter_test_only', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeTimer:
    def __init__(self):
        self.reset_count = 0
        self.cancel_count = 0

    def reset(self):
        self.reset_count += 1

    def cancel(self):
        self.cancel_count += 1


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class FakeBuffer:
    def __init__(self):
        self.fail_stamps = set()

    def lookup_transform(self, target, source, stamp, timeout):
        if stamp.nanoseconds in self.fail_stamps:
            raise TransformException('missing transform')
        return SimpleNamespace(transform=SimpleNamespace(
            translation=Vector(), rotation=Quaternion()))


def message(stamp_ns):
    result = Odometry()
    result.header.stamp = Stamp(stamp_ns // 1_000_000_000,
                                stamp_ns % 1_000_000_000)
    return result


def adapter(module, now_ns=2_000_000_000, capacity=3):
    result = module.PoseAdapter.__new__(module.PoseAdapter)
    result.p = {
        'camera_frame': 'd435_link', 'base_frame': 'base',
        'max_input_age_sec': 0.5, 'tf_wait_sec': 0.15,
    }
    result.pending = PoseQueue(('odom', 'slam_odom'), capacity)
    result.retry_timer = FakeTimer()
    result._retry_active = False
    result.buffer = FakeBuffer()
    result.publishers_by_kind = {'odom': FakePublisher(), 'slam_odom': FakePublisher()}
    result.odom_broadcaster = None
    result.get_clock = lambda: SimpleNamespace(now=lambda: FakeTime(now_ns))
    result.events = []
    result.warnings = []
    result._timing = lambda stage, kind, msg, **fields: result.events.append(
        (stage, kind, result._stamp_ns(msg), fields))
    result.warn = result.warnings.append
    return result


def test_pose_arrival_publishes_immediately_in_fifo_order(pose_adapter_module):
    node = adapter(pose_adapter_module)
    for stamp in (1_900_000_000, 1_910_000_000, 1_920_000_000):
        node.on_pose('odom', message(stamp))
    assert [node._stamp_ns(msg) for msg in node.publishers_by_kind['odom'].messages] == [
        1_900_000_000, 1_910_000_000, 1_920_000_000]
    assert not node.pending
    assert node.retry_timer.reset_count == 0


def test_missing_tf_queues_then_drains_all_and_cancels_retry(pose_adapter_module):
    node = adapter(pose_adapter_module)
    stamps = (1_900_000_000, 1_910_000_000, 1_920_000_000)
    node.buffer.fail_stamps.update(stamps)
    for stamp in stamps:
        node.on_pose('odom', message(stamp))
    assert node.pending.depth('odom') == 3
    assert node.retry_timer.reset_count == 1
    node.buffer.fail_stamps.clear()
    node.drain()
    assert [node._stamp_ns(msg) for msg in node.publishers_by_kind['odom'].messages] == list(stamps)
    assert not node.pending
    assert node.retry_timer.cancel_count == 1


def test_tf_timeout_drops_head_then_publishes_following_pose(pose_adapter_module):
    node = adapter(pose_adapter_module)
    old_stamp, good_stamp = 1_900_000_000, 1_920_000_000
    node.buffer.fail_stamps.add(old_stamp)
    node.pending.put('odom', (message(old_stamp), time.monotonic() - 1.0))
    node.pending.put('odom', (message(good_stamp), time.monotonic()))
    node.drain()
    assert [event[3].get('reason') for event in node.events
            if event[0] == 'upc_pose_adapter_dropped'] == ['tf_timeout']
    assert [node._stamp_ns(msg) for msg in node.publishers_by_kind['odom'].messages] == [
        good_stamp]


def test_full_waiting_queue_drops_oldest_and_other_kind_is_independent(
        pose_adapter_module):
    node = adapter(pose_adapter_module, capacity=2)
    odom_stamps = (1_900_000_000, 1_910_000_000, 1_920_000_000)
    node.buffer.fail_stamps.update(odom_stamps)
    for stamp in odom_stamps:
        node.on_pose('odom', message(stamp))
    node.on_pose('slam_odom', message(1_930_000_000))
    drops = [event for event in node.events
             if event[0] == 'upc_pose_adapter_dropped']
    assert [(event[2], event[3]['reason']) for event in drops] == [
        (1_900_000_000, 'queue_overflow')]
    assert node.pending.depth('odom') == 2
    assert [node._stamp_ns(msg) for msg in node.publishers_by_kind['slam_odom'].messages] == [
        1_930_000_000]
