from pathlib import Path
from rby1_scheduler.data_source import JsonDataSource
from rby1_scheduler.lab_config import load_lab_config
from rby1_scheduler.planner_client import MockPlannerClient
from rby1_scheduler.runtime_engine import RuntimeEngine

class Clock:
    def __init__(self):
        self.value = 0.0
    def __call__(self):
        return self.value

def test_priority_dispatch_updates_plate_only_after_planner_success(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    clock = Clock()
    engine = RuntimeEngine(MockPlannerClient(1.0, clock=clock))
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    engine.apply_command({'cmd': 'play'})
    engine.tick()
    assert engine.current_task_id == 'B001.T01'
    plate = engine.plates['B001-C']
    assert plate.location == 'INBOX'
    clock.value = 2.0
    engine.tick()
    assert plate.location == 'LH1'
    assert 'T01' in plate.completed_tasks

def test_manual_transfer_reaches_planner_and_records_result(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    config = load_lab_config(root / 'config' / 'lab.yaml')
    clock = Clock()
    engine = RuntimeEngine(MockPlannerClient(1.0, clock=clock))
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    engine.apply_command({'cmd': 'manual_transfer', 'source': 'LH1', 'destination': 'ZS'})
    assert engine.current_task_id == 'MANUAL.T001'
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
    engine = RuntimeEngine(MockPlannerClient())
    engine.sync_external(config.enrich_snapshot(source.read_snapshot()))
    engine.apply_command({'cmd': 'play'})
    engine.tick()
    assert engine.current_task_id == 'B001.T01'

    engine.apply_command({'cmd': 'pause'})
    engine.tick()

    assert engine.paused is True
    assert engine.current_task_id == ''
    assert engine.last_planner_result['status'] == 'canceled'
