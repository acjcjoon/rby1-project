from pathlib import Path
from rby1_scheduler.data_source import JsonDataSource
from rby1_scheduler.lab_config import load_lab_config
from rby1_scheduler.models import TaskStatus
from rby1_scheduler.planner_client import MockPlannerClient
from rby1_scheduler.runtime_engine import RuntimeEngine

class Clock:
    def __init__(self):
        self.value = 0.0
    def __call__(self):
        return self.value


def test_live_tie_break_prefers_batch_order_before_workflow_order(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(
        root / 'mock_server' / 'server_input.json',
        tmp_path / 'out.json', tmp_path / 'events.jsonl',
    )
    config = load_lab_config(root / 'config' / 'lab.yaml')
    engine = RuntimeEngine(MockPlannerClient(), mock_process_duration_sec=1.0)
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    for task in engine.tasks.values():
        task.status = TaskStatus.DONE

    earlier_batch_later_step = engine.tasks['B001.T08']
    later_batch_earlier_step = engine.tasks['B002.T06']
    for task in (earlier_batch_later_step, later_batch_earlier_step):
        task.status = TaskStatus.PENDING
        task.prerequisites = []
    engine.capacities['LH2'] = 100

    assert engine.choose().task_id == 'B001.T08'

def test_operator_task_button_starts_selected_and_updates_plate_after_success(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    clock = Clock()
    engine = RuntimeEngine(
        MockPlannerClient(1.0, clock=clock), clock=clock,
        mock_process_duration_sec=1.0,
    )
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    engine.apply_command({'cmd': 'toggle_task', 'task_id': 'B001.T02'})
    assert engine.current_task_id == 'B001.S1'
    assert engine.selected_target_id == 'B001.T02'
    plate = engine.plates['B001-A']
    assert plate.location == 'PREP'
    clock.value = 2.0
    engine.tick()
    assert plate.location == 'LH1'
    assert engine.current_task_id == 'B001.T02'
    clock.value = 4.0
    engine.tick()
    assert plate.location == 'ZS'
    assert 'S1' in plate.completed_tasks
    assert 'T02' in plate.completed_tasks
    assert engine.selected_target_id == ''

def test_manual_transfer_reaches_planner_and_records_result(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    clock = Clock()
    engine = RuntimeEngine(MockPlannerClient(1.0, clock=clock))
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    engine.apply_command({'cmd': 'manual_transfer', 'source': 'LH1', 'destination': 'ZS'})
    assert engine.current_task_id == 'MANUAL.T001'
    assert engine.tasks['MANUAL.T001'].priority.name == 'P1'
    assert engine.robot.status == 'running'
    assert engine.paused is True
    clock.value = 2.0
    engine.tick()
    assert engine.current_task_id == ''
    assert engine.last_planner_result['status'] == 'succeeded'
    assert engine.last_planner_result['task_id'] == 'MANUAL.T001'

def test_manual_transfer_pause_button_cancel_path(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    engine = RuntimeEngine(MockPlannerClient())
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    engine.apply_command({'cmd': 'manual_transfer', 'source': 'MPR', 'destination': 'LH1'})
    engine.apply_command({'cmd': 'pause'})
    engine.tick()
    assert engine.paused is True
    assert engine.current_task_id == ''
    assert engine.last_planner_result['status'] == 'canceled'

def test_live_pause_cancels_active_planner_task(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    engine = RuntimeEngine(MockPlannerClient(), mock_process_duration_sec=4.0)
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    engine.apply_command({'cmd': 'toggle_task', 'task_id': 'B001.T02'})
    assert engine.current_task_id == 'B001.S1'

    engine.apply_command({'cmd': 'pause'})
    engine.tick()

    assert engine.paused is True
    assert engine.current_task_id == ''
    assert engine.last_planner_result['status'] == 'canceled'

def test_live_timeline_grows_while_running_and_records_completion(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    data = config.enrich_snapshot(source.read_snapshot())
    clock = Clock()
    engine = RuntimeEngine(
        MockPlannerClient(120.0, clock=clock),
        clock=clock,
        mock_process_duration_sec=60.0,
    )
    engine.sync_external(data)
    engine.apply_command({'cmd': 'toggle_task', 'task_id': 'B001.T02'})

    clock.value = 30.0
    running = engine.snapshot()['live_timeline']
    assert running['elapsed_min'] == 0.5
    assert running['timeline'][0]['status'] == 'running'
    assert running['timeline'][0]['duration_min'] == 0.5

    clock.value = 61.0
    engine.tick()
    assert engine.current_task_id == 'B001.T02'
    assert engine.snapshot()['live_timeline']['task_count'] == 2

    clock.value = 182.0
    engine.tick()
    completed = engine.snapshot()['live_timeline']
    assert completed['task_count'] == 2
    assert [row['status'] for row in completed['timeline']] == [
        'succeeded', 'succeeded',
    ]
    assert completed['timeline'][0]['kind'] == 'process'
    assert completed['timeline'][1]['kind'] == 'transfer'

def test_pressing_the_running_task_button_again_requests_cancel(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    engine = RuntimeEngine(MockPlannerClient(), mock_process_duration_sec=4.0)
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))

    engine.apply_command({'cmd': 'toggle_task', 'task_id': 'B001.T02'})
    engine.apply_command({'cmd': 'toggle_task', 'task_id': 'B001.T02'})

    assert engine.current_task_id == ''
    assert engine.last_planner_result['status'] == 'canceled'
    assert engine.selected_target_id == ''

def test_operator_selection_runs_prerequisites_when_plate_is_not_at_source(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    engine = RuntimeEngine(MockPlannerClient(), mock_process_duration_sec=4.0)
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))

    engine.apply_command({'cmd': 'toggle_task', 'task_id': 'B001.T02'})

    assert engine.current_task_id == 'B001.S1'
    assert engine.snapshot()['selected_target_id'] == 'B001.T02'


def test_operator_can_queue_batch_target_while_live_play_is_busy(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(
        root / 'mock_server' / 'server_input.json',
        tmp_path / 'out.json', tmp_path / 'events.jsonl',
    )
    config = load_lab_config(root / 'config' / 'lab.yaml')
    clock = Clock()
    engine = RuntimeEngine(
        MockPlannerClient(1.0, clock=clock), clock=clock,
        mock_process_duration_sec=1.0,
    )
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    engine.apply_command({'cmd': 'play'})
    engine.tick()
    assert engine.current_task_id == 'B001.S1'

    engine.apply_command({'cmd': 'toggle_task', 'task_id': 'B002.T02'})

    assert engine.current_task_id == 'B001.S1'
    assert engine.selected_target_id == 'B002.T02'
    clock.value = 2.0
    engine.tick()
    assert engine.current_task_id == 'B002.S1'


def test_canceling_queued_target_does_not_cancel_unrelated_live_task(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(
        root / 'mock_server' / 'server_input.json',
        tmp_path / 'out.json', tmp_path / 'events.jsonl',
    )
    config = load_lab_config(root / 'config' / 'lab.yaml')
    engine = RuntimeEngine(
        MockPlannerClient(), mock_process_duration_sec=4.0,
    )
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    engine.apply_command({'cmd': 'play'})
    engine.tick()
    assert engine.current_task_id == 'B001.S1'

    engine.apply_command({'cmd': 'toggle_task', 'task_id': 'B002.T02'})
    engine.apply_command({'cmd': 'toggle_task', 'task_id': 'B002.T02'})

    assert engine.current_task_id == 'B001.S1'
    assert engine.selected_target_id == ''
    assert engine.paused is False


def test_mock_play_starts_authored_process_and_live_timeline(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    engine = RuntimeEngine(
        MockPlannerClient(),
        mock_process_duration_sec=4.0,
    )
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))

    engine.apply_command({'cmd': 'play'})
    engine.tick()

    assert engine.current_task_id == 'B001.S1'
    live = engine.snapshot()['live_timeline']
    assert live['task_count'] == 1
    assert live['timeline'][0]['kind'] == 'process'
    assert live['timeline'][0]['route'] == 'LH1'


def test_mock_play_completes_all_batches_without_filling_lh2_deadlock(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(
        root / 'mock_server' / 'server_input.json',
        tmp_path / 'out.json', tmp_path / 'events.jsonl',
    )
    config = load_lab_config(root / 'config' / 'lab.yaml')
    clock = Clock()
    engine = RuntimeEngine(
        MockPlannerClient(1.0, clock=clock), clock=clock,
        mock_process_duration_sec=1.0,
    )
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))

    initial = engine.snapshot()
    initial_transfers = [
        task for task in initial['queue']
        if task['batch_id'] == 'B001' and task['kind'] == 'transfer'
    ]
    assert all(task['operator_startable'] for task in initial_transfers)

    engine.apply_command({'cmd': 'play'})
    for _ in range(250):
        engine.tick()
        clock.value += 2.0
        if not engine.current_task_id and engine.choose() is None:
            break

    assert all(task.status.value == 'done' for task in engine.tasks.values())
    assert {plate.location for plate in engine.plates.values()} == {'WASTE'}
