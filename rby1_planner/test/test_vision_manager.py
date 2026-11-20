"""On-demand tag detection: manager policy and Task runner gating."""
import pytest

from rby1_planner.task_commands import Task
from rby1_planner.task_lifecycle import TaskRunStatus
from rby1_planner.task_runner import PlannerTaskRunner
from rby1_planner.vision_manager import VisionManager, launch_command
from test_task_runner import (
    FakeBackend,
    FakeCamera,
    FakeNode,
    _camera_observation,
)


class FakeProcess:
    def __init__(self):
        self.returncode = None
        self.pid = -1

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.returncode = 0
        return 0


class GraphNode:
    def __init__(self):
        self.nodes = []

    def get_node_names_and_namespaces(self):
        return list(self.nodes)


def _manager(mode='auto'):
    node = GraphNode()
    clock = {'now': 10.0}
    spawned = []

    def spawn(camera):
        process = FakeProcess()
        spawned.append((camera, process))
        return process

    manager = VisionManager(
        node,
        {'d405_left': '/d405_left/detections', 'd435': '/d435/detections'},
        mode=mode,
        clock=lambda: clock['now'],
        spawn=spawn,
        subscribe=False,
    )
    # Stopping a fake process must not signal a real process group.
    manager._signal = lambda process, _signum: process.wait()
    return manager, node, clock, spawned


def test_launch_command_starts_only_one_detection_pair():
    command = launch_command('d435')
    assert 'mount_tf:=false' in command
    assert 'detection_cameras:=d435' in command


def test_auto_mode_starts_on_require_and_stops_on_release():
    manager, node, clock, spawned = _manager()
    assert spawned == [] and not manager.running('d435')

    manager.require('d435')
    manager.require('d435')
    assert [camera for camera, _ in spawned] == ['d435']
    assert manager.ready('d435')[0] is False

    clock['now'] += 1.0
    manager._on_detections('d435')
    assert manager.ready('d435') == (False, 'waiting for the d435 tag TF node')
    node.nodes = [('apriltag_node', '/d435'), ('depth_tag_tf_node', '/d435')]
    assert manager.ready('d435') == (True, '')
    clock['now'] += 2.0  # heartbeat went stale
    assert manager.ready('d435')[0] is False

    manager.require('d405_left')
    manager.release_except({'d435'})
    assert manager.running('d435') and not manager.running('d405_left')
    manager.release_all()
    assert not manager.running('d435')


def test_modes_on_and_off():
    manager, _node, _clock, spawned = _manager('on')
    assert sorted(camera for camera, _ in spawned) == ['d405_left', 'd435']
    manager.require('d435')
    manager.release_all()
    assert manager.running('d435')  # "on" keeps every camera running

    manager.set_mode('off')
    assert not manager.running('d435') and not manager.running('d405_left')
    with pytest.raises(RuntimeError, match='switched off'):
        manager.require('d435')

    manager.set_mode('auto')
    manager.require('d435')
    with pytest.raises(RuntimeError, match='running Task'):
        manager.set_mode('off')
    manager.set_mode('on')
    manager.set_mode('auto')
    assert manager.running('d435') and not manager.running('d405_left')
    with pytest.raises(ValueError):
        manager.set_mode('sometimes')
    with pytest.raises(RuntimeError):
        manager.require('webcam')


def test_external_pair_is_used_and_never_stopped():
    manager, node, clock, spawned = _manager()
    node.nodes = [('apriltag_node', '/d435'), ('depth_tag_tf_node', '/d435')]
    manager.require('d435')
    assert spawned == []
    clock['now'] += 0.1
    manager._on_detections('d435')
    assert manager.ready('d435') == (True, '')
    assert 'd435: external' in manager.status_text()
    manager.release_all()


def test_exited_process_is_reported():
    manager, _node, _clock, spawned = _manager()
    manager.require('d435')
    spawned[0][1].returncode = 2
    with pytest.raises(RuntimeError, match='exited'):
        manager.ready('d435')


class FakeVision:
    def __init__(self):
        self.calls = []
        self.is_ready = False
        self.held = set()

    def require(self, camera):
        if camera not in self.held:
            self.calls.append(('require', camera))
        self.held.add(camera)

    def ready(self, camera):
        return self.is_ready, '' if self.is_ready else 'waiting'

    def release_except(self, keep):
        for camera in sorted(self.held - set(keep)):
            self.held.discard(camera)
            self.calls.append(('release', camera))

    def release_all(self):
        self.release_except(())


def _camera_task():
    task = Task('look')
    task.joint_relative('right_arm', [1, 0, 0, 0, 0, 0, 0], [2.0, 0.2, 0.2])
    task.camera_linear_absolute(
        'right_arm', 'tag_0', [3.0, 0.1, 0.1, 0.5],
        camera_source='d405_right', detection_timeout_sec=1.0,
    )
    task.joint_relative('right_arm', [1, 0, 0, 0, 0, 0, 0], [2.0, 0.2, 0.2])
    return task.build()


def _runner(vision):
    node, backend, camera = FakeNode(), FakeBackend(), FakeCamera()
    results = []
    runner = PlannerTaskRunner(
        node, backend, camera=camera, vision=vision,
        on_finished=results.append, clock=lambda: backend.now,
    )
    return runner, backend, camera, results


def test_runner_starts_detection_for_camera_step_and_releases_after():
    vision = FakeVision()
    runner, backend, camera, results = _runner(vision)
    runner.start(_camera_task())
    assert vision.calls == []  # the first step needs no camera

    for _ in range(6):
        runner.tick()
        backend.now += 0.05
    assert vision.calls == [('require', 'd405_right')]
    assert camera.marker_calls == 0  # nothing is measured before it is ready

    vision.is_ready = True
    camera.observation = _camera_observation(stamp_ns=camera.marker + 5)
    for _ in range(40):
        runner.tick()
        backend.now += 0.05
        if not runner.active:
            break
    assert results[-1].status is TaskRunStatus.SUCCEEDED
    assert camera.marker_calls >= 1
    assert vision.calls == [
        ('require', 'd405_right'), ('release', 'd405_right'),
    ]


def test_runner_fails_when_detection_does_not_start_and_releases():
    vision = FakeVision()
    runner, backend, _camera, results = _runner(vision)
    runner.start(_camera_task())
    for _ in range(8):
        runner.tick()
        backend.now += 0.05
    assert vision.calls == [('require', 'd405_right')]
    backend.now += PlannerTaskRunner.VISION_START_TIMEOUT_SEC + 1.0
    runner.tick()
    assert results[-1].status is TaskRunStatus.FAILED
    assert 'tag detection for d405_right' in results[-1].message
    assert vision.held == set()


def test_cancel_releases_detection():
    vision = FakeVision()
    runner, backend, _camera, results = _runner(vision)
    runner.start(_camera_task())
    for _ in range(6):
        runner.tick()
        backend.now += 0.05
    assert vision.held == {'d405_right'}
    runner.stop('cancel')
    assert results[-1].status is TaskRunStatus.CANCELED
    assert vision.held == set()
