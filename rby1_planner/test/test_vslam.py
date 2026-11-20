"""VSLAM Task steps and client helpers; no robot, container or ROS graph."""
import pytest

from rby1_planner.task import build_tasks, offline_map_creation
from rby1_planner.task_commands import DynamicTaskDefinition, Task, VslamStep
from rby1_planner.task_lifecycle import TaskRunStatus
from rby1_planner.task_runner import PlannerTaskRunner
from rby1_planner.vslam_client import (
    VslamJobState,
    correction_complete,
    list_maps,
    station_pose_blocker,
    validate_map_name,
)
from test_task_runner import FakeBackend, FakeNode


class FakeVslam:
    def __init__(self):
        self.state = 'stopped'
        self.mode = ''
        self.started = []
        self.tracking = (False, 'waiting for tracking status')
        self.correction_requests = []
        self.correction = VslamJobState()
        self.saved = []
        self.job = VslamJobState()

    def process_state(self):
        return self.state

    def start(self, mode, map_name=''):
        self.started.append((mode, map_name))
        self.state = 'running'
        self.mode = mode

    def tracking_ready(self):
        return self.tracking

    def request_station_correction(self, timeout_sec=None):
        self.correction_requests.append(timeout_sec)
        self.correction = VslamJobState('running', 'station correction', 'hold')

    def correction_state(self):
        return self.correction

    def save_map(self, name):
        self.saved.append(name)
        self.job = VslamJobState('running', f'save map {name}', 'running')
        return name

    def job_state(self):
        return self.job


def _runner(vslam):
    node = FakeNode()
    backend = FakeBackend()
    messages, results, operator = [], [], []
    runner = PlannerTaskRunner(
        node,
        backend,
        vslam=vslam,
        on_status=messages.append,
        on_finished=results.append,
        clock=lambda: backend.now,
    )
    runner.on_operator_control = operator.append
    return runner, backend, messages, results, operator


def _mapping_task(name=''):
    task = Task('mapping')
    task.vslam_start('mapping', timeout_sec=10.0)
    task.vslam_station_correct()
    task.vslam_operator_mapping()
    task.vslam_save_map(name)
    return task.build()


def test_vslam_step_validation():
    assert isinstance(_mapping_task(), DynamicTaskDefinition)
    with pytest.raises(ValueError):
        VslamStep('fly')
    with pytest.raises(ValueError):
        VslamStep('start', mode='rgbd')
    with pytest.raises(ValueError):
        VslamStep('start', mode='localization')
    with pytest.raises(ValueError):
        VslamStep('save_map', map_name='../escape')
    with pytest.raises(ValueError):
        VslamStep('start', timeout_sec=0.0)


def test_mapping_task_runs_start_correct_operator_save():
    vslam = FakeVslam()
    runner, backend, messages, results, operator = _runner(vslam)
    runner.start(_mapping_task('lab_01'))

    runner.tick()
    assert vslam.started == [('mapping', '')]
    runner.tick()
    assert runner.active and vslam.correction_requests == []
    vslam.tracking = (True, 'local visual tracking ready')
    runner.tick()

    runner.tick()
    assert vslam.correction_requests == [30.0]
    assert backend.stop_calls == 1
    runner.tick()
    assert not runner.operator_step_active
    vslam.correction = VslamJobState('succeeded', 'station correction', 'ok')
    runner.tick()

    runner.tick()
    assert runner.operator_step_active and operator == [True]
    assert backend.stream_requests == []  # Stream is left to the operator
    for _ in range(5):
        runner.tick()
    assert runner.active and vslam.saved == []
    assert runner.finish_operator_step()
    runner.tick()
    assert operator == [True, False] and not runner.operator_step_active

    runner.tick()
    assert vslam.saved == ['lab_01']
    runner.tick()
    assert runner.active
    vslam.job = VslamJobState('succeeded', 'save map lab_01', 'Map saved')
    runner.tick()
    runner.tick()
    assert not runner.active
    assert results[-1].status is TaskRunStatus.SUCCEEDED
    assert vslam.state == 'running'  # the stack is left running


def test_save_map_defaults_to_timestamp_name():
    vslam = FakeVslam()
    vslam.state, vslam.mode = 'running', 'mapping'
    runner, *_ = _runner(vslam)
    runner.start(Task('save').vslam_save_map().build())
    runner.tick()
    assert len(vslam.saved) == 1 and vslam.saved[0].startswith('map_')
    validate_map_name(vslam.saved[0])


def test_start_reuses_same_mode_and_rejects_other_mode():
    vslam = FakeVslam()
    vslam.state, vslam.mode = 'running', 'mapping'
    vslam.tracking = (True, 'ready')
    runner, _backend, _messages, results, _operator = _runner(vslam)
    runner.start(Task('reuse').vslam_start('mapping').build())
    runner.tick()
    runner.tick()
    runner.tick()
    assert vslam.started == [] and results[-1].status is TaskRunStatus.SUCCEEDED

    vslam.mode = 'localization'
    runner.start(Task('clash').vslam_start('mapping').build())
    runner.tick()
    assert results[-1].status is TaskRunStatus.FAILED
    assert 'localization mode' in results[-1].message


def test_tracking_timeout_and_stack_exit_fail_the_task():
    vslam = FakeVslam()
    runner, backend, _messages, results, _operator = _runner(vslam)
    runner.start(Task('timeout').vslam_start('mapping', timeout_sec=5.0).build())
    runner.tick()
    backend.now += 6.0
    runner.tick()
    assert results[-1].status is TaskRunStatus.FAILED
    assert 'tracking' in results[-1].message

    vslam.state = 'stopped'
    runner.start(Task('exit').vslam_start('mapping').build())
    runner.tick()
    vslam.state = 'exited'
    runner.tick()
    assert results[-1].status is TaskRunStatus.FAILED


def test_failures_and_cancel_release_operator_control():
    vslam = FakeVslam()
    vslam.state, vslam.mode = 'running', 'mapping'
    runner, backend, _messages, results, operator = _runner(vslam)
    runner.start(Task('correct').vslam_station_correct().build())
    runner.tick()
    vslam.correction = VslamJobState('failed', 'station correction', 'timed out')
    runner.tick()
    assert results[-1].status is TaskRunStatus.FAILED

    runner.start(Task('drive').vslam_operator_mapping().build())
    runner.tick()
    assert operator == [True]
    stops = backend.stop_calls
    runner.stop('operator cancel')
    assert operator == [True, False]
    assert backend.stop_calls > stops
    assert results[-1].status is TaskRunStatus.CANCELED
    assert not runner.finish_operator_step()

    runner.start(Task('lost').vslam_operator_mapping().build())
    runner.tick()
    vslam.state = 'exited'
    runner.tick()
    assert results[-1].status is TaskRunStatus.FAILED
    assert operator[-1] is False


def test_vslam_step_without_client_fails():
    runner, _backend, _messages, results, _operator = _runner(None)
    runner.start(Task('none').vslam_start('mapping').build())
    runner.tick()
    assert results[-1].status is TaskRunStatus.FAILED


def test_offline_map_creation_is_registered():
    definition = build_tasks()['offline_map_creation']
    actions = [
        step.action for step in definition.commands
        if isinstance(step, VslamStep)
    ]
    assert actions == ['start', 'correct', 'operator', 'save_map']
    assert not isinstance(definition.commands[0], VslamStep)  # head pitch
    assert offline_map_creation('lab').commands[-1].map_name == 'lab'


def test_map_name_and_map_listing(tmp_path):
    assert validate_map_name(' lab_01 ') == 'lab_01'
    for bad in ('', '..', 'a/b', '.hidden', 'x' * 65, 'sp ace'):
        with pytest.raises(ValueError):
            validate_map_name(bad)
    (tmp_path / 'b_map').mkdir()
    (tmp_path / 'b_map' / 'data.mdb').write_bytes(b'x')
    (tmp_path / 'a_map').mkdir()
    (tmp_path / 'a_map' / 'data.mdb').write_bytes(b'x')
    (tmp_path / 'empty').mkdir()
    (tmp_path / 'file.mdb').write_bytes(b'x')
    assert list_maps(tmp_path) == ['a_map', 'b_map']
    assert list_maps(tmp_path / 'missing') == []


def test_station_pose_guards():
    good = {
        'initialized': True, 'healthy': True, 'station_mode': True,
        'station_converged': True, 'stationary_ready': True,
        'correction_count': 3,
    }
    assert station_pose_blocker(good, status_fresh=True) == ''
    assert 'stale' in station_pose_blocker(good, status_fresh=False)
    assert 'not initialized' in station_pose_blocker(
        {**good, 'initialized': False}, status_fresh=True)
    assert 'unhealthy' in station_pose_blocker(
        {**good, 'healthy': False}, status_fresh=True)
    assert 'station correction' in station_pose_blocker(
        {**good, 'station_converged': False}, status_fresh=True)
    assert correction_complete(good, baseline_count=2)
    assert not correction_complete(good, baseline_count=3)
    assert not correction_complete(
        {**good, 'stationary_ready': False}, baseline_count=2)


# ---------------------------------------------------------------------------
# Nav2 Goto
# ---------------------------------------------------------------------------
from types import SimpleNamespace  # noqa: E402

from rby1_planner.vslam_client import VslamClient, gate_idle, goto_blocker  # noqa: E402


def test_goto_guards():
    ready = {'initialized': True, 'healthy': True, 'station_mode': True,
             'station_converged': True, 'stationary_ready': True}
    assert goto_blocker(ready, status_fresh=True) == ''
    assert goto_blocker({**ready, 'station_mode': False}, status_fresh=True) == ''
    assert goto_blocker(ready, status_fresh=False)
    assert goto_blocker({**ready, 'healthy': False}, status_fresh=True)
    assert goto_blocker({**ready, 'station_converged': False}, status_fresh=True)
    idle = {'enabled': False, 'cancel_pending': False, 'active_goal_count': 0}
    assert gate_idle(idle, status_fresh=True)
    assert not gate_idle(idle, status_fresh=False)
    assert not gate_idle({**idle, 'enabled': True}, status_fresh=True)
    assert not gate_idle({**idle, 'active_goal_count': 1}, status_fresh=True)


def test_task_start_is_refused_during_goto():
    vslam = FakeVslam()
    vslam.state, vslam.mode = 'running', 'mapping'
    vslam.navigation_busy = lambda: True
    runner, *_ = _runner(vslam)
    with pytest.raises(RuntimeError, match='Goto is running'):
        runner.start(Task('save').vslam_save_map().build())
    vslam.navigation_busy = lambda: False
    runner.start(Task('save').vslam_save_map().build())
    assert runner.active


class DoneFuture:
    def __init__(self, success=True, message=''):
        self._response = SimpleNamespace(success=success, message=message)

    def done(self):
        return True

    def exception(self):
        return None

    def result(self):
        return self._response


class FakeService:
    def __init__(self, on_call=None, success=True):
        self.calls = []
        self.on_call = on_call
        self.success = success

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls.append(getattr(request, 'data', None))
        if self.on_call is not None:
            self.on_call(request)
        return DoneFuture(self.success, 'rejected' if not self.success else '')


class FakeGoalHandle:
    accepted = True

    def __init__(self):
        self.canceled = False
        self.result_callbacks = []

    def cancel_goal_async(self):
        self.canceled = True

    def get_result_async(self):
        handle = self

        class Result:
            def add_done_callback(self, callback):
                handle.result_callbacks.append(callback)
        return Result()


class FakeActionClient:
    def __init__(self):
        self.goals = []
        self.handle = FakeGoalHandle()

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        handle = self.handle

        class Response:
            def add_done_callback(self, callback):
                callback(SimpleNamespace(result=lambda: handle))
        return Response()


@pytest.fixture
def goto_client():
    """Real VslamClient on a ROS node with every external endpoint faked."""

    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('nav2_msgs')
    rclpy.init()
    node = rclpy.create_node('vslam_goto_test')
    clock = {'now': 100.0}
    client = VslamClient(
        node, launch_script='/nonexistent.sh', clock=lambda: clock['now'],
    )
    client._goto_timer.cancel()
    state = {'station_mode': True, 'count': 3}

    def localization():
        client._statuses['localization'] = ({
            'initialized': True, 'healthy': True,
            'station_mode': state['station_mode'],
            'station_converged': state['station_mode'],
            'stationary_ready': True,
            'navigation_ready': not state['station_mode'],
            'correction_count': state['count'],
        }, clock['now'])

    def gate(enabled=False, goals=0):
        client._statuses['navigation'] = ({
            'enabled': enabled, 'cancel_pending': False,
            'active_goal_count': goals, 'detail': '',
        }, clock['now'])

    def on_station(request):
        state['station_mode'] = bool(request.data)
        if request.data:
            state['count'] += 1

    client._station_client = FakeService(on_station)
    client._gate_enable_client = FakeService()
    client._gate_cancel_client = FakeService()
    client._navigate_client = FakeActionClient()
    client.navigation_process_state = lambda: 'running'

    def step(enabled=False):
        clock['now'] += 0.1
        localization()
        gate(enabled)
        client._tick_goto()

    localization()
    gate()
    yield SimpleNamespace(
        client=client, clock=clock, step=step, state=state,
        point=SimpleNamespace(name='LH1', x=1.0, y=2.0, yaw=0.5),
    )
    node.destroy_node()
    rclpy.shutdown()


def test_goto_runs_gate_transit_goal_and_correction(goto_client):
    c, client = goto_client, goto_client.client
    client.goto(c.point)
    assert client.navigation_busy()
    with pytest.raises(RuntimeError, match='already running'):
        client.goto(c.point)
    with pytest.raises(RuntimeError, match='Goto is running'):
        client.request_station_correction()

    c.step()   # gate idle -> leave station mode
    assert c.state['station_mode'] is False
    c.step()   # transit confirmed -> enable gate
    assert client._gate_enable_client.calls[-1] is True
    c.step()   # goal sent
    goal = client._navigate_client.goals[0]
    assert goal.pose.header.frame_id == 'vslam_map'
    assert goal.pose.pose.position.x == 1.0 and goal.pose.pose.position.y == 2.0
    for _ in range(200):   # travelling has no stage timeout
        c.step(enabled=True)
    assert client.goto_state().running

    handle = client._navigate_client.handle
    handle.result_callbacks[0](SimpleNamespace(
        result=lambda: SimpleNamespace(status=4)))
    c.step(enabled=True)   # arrived -> disable gate
    assert client._gate_enable_client.calls[-1] is False
    c.step()   # gate idle -> station correction requested
    assert c.state['station_mode'] is True
    c.step()
    c.step()
    assert not client.navigation_busy()
    assert client.goto_state().status == 'succeeded'


def test_goto_stop_and_failures_disarm_the_gate(goto_client):
    c, client = goto_client, goto_client.client
    client.goto(c.point)
    for _ in range(3):
        c.step()
    client.stop_goto()
    assert client._navigate_client.handle.canceled
    assert client._gate_enable_client.calls[-1] is False
    assert client.goto_state().status == 'failed'
    assert not client.navigation_busy()

    client._navigate_client = FakeActionClient()
    client.goto(c.point)
    for _ in range(3):
        c.step()
    client._navigate_client.handle.result_callbacks[0](SimpleNamespace(
        result=lambda: SimpleNamespace(status=6)))
    c.step(enabled=True)
    assert 'ABORTED' in client.goto_state().message
    assert client._gate_enable_client.calls[-1] is False

    # A gate that never becomes idle times out instead of hanging.
    client.goto(c.point)
    for _ in range(120):
        c.step(enabled=True)
    assert 'timed out' in client.goto_state().message

    c.state['station_mode'] = True
    c.step()
    client._statuses['localization'][0]['healthy'] = False
    with pytest.raises(RuntimeError, match='station correction'):
        client.goto(c.point)
