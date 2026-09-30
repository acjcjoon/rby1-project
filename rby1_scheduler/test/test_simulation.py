import copy
from pathlib import Path
from rby1_scheduler.data_source import JsonDataSource
from rby1_scheduler.lab_config import load_lab_config
from rby1_scheduler.simulation import ScheduleSimulator

def test_lab_config_supplies_static_queue_fields(tmp_path):
    root = Path(__file__).parents[1]
    config = load_lab_config(root / 'config' / 'lab.yaml')
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    data = config.enrich_snapshot(source.read_snapshot())
    task = next(item for item in data['work_queue'] if item['task_id'] == 'B002.T01')
    assert task['source'] == 'INBOX'
    assert task['destination'] == 'LH1'
    assert task['priority'] == 'P5'
    assert data['capacities']['LH1'] == 3

def test_simulation_completes_without_mutating_server_snapshot(tmp_path):
    root = Path(__file__).parents[1]
    config = load_lab_config(root / 'config' / 'lab.yaml')
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    data = source.read_snapshot()
    before = copy.deepcopy(data)
    result = ScheduleSimulator(config).run(data)
    assert result['status'] == 'completed'
    assert result['end_min'] > 0
    assert result['simulated_count'] > 0
    assert result['pending'] == []
    s2 = next(row for row in result['timeline'] if row['task_id'] == 'B001.S2')
    transfer = next(row for row in result['timeline'] if row['kind'] == 'transfer')
    assert s2['duration_min'] == 360
    assert transfer['duration_min'] == 5
    assert data == before
