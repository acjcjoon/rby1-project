from rby1_scheduler.ui.batch_queue_tab import build_batch_groups


def test_batch_queue_groups_tasks_and_uses_workflow_order():
    state = {
        'batches': [
            {
                'batch_id': 'B001',
                'molar_ratio': {'display': '1:1.2'},
                'status': 'waiting',
            },
            {
                'batch_id': 'B002',
                'molar_ratio': {'display': '1:1.4'},
                'status': 'waiting',
            },
        ],
        'queue': [
            {'task_id': 'B002.T05', 'batch_id': 'B002', 'order': 3},
            {'task_id': 'B001.T02', 'batch_id': 'B001', 'order': 2},
            {'task_id': 'B001.T01', 'batch_id': 'B001', 'order': 1},
            {'task_id': 'MANUAL.T001', 'batch_id': 'MANUAL', 'order': -1},
        ],
    }

    groups = build_batch_groups(state)

    assert [group['batch_id'] for group in groups] == [
        'B001', 'B002', 'MANUAL',
    ]
    assert [task['task_id'] for task in groups[0]['tasks']] == [
        'B001.T01', 'B001.T02',
    ]
    assert groups[0]['molar_ratio']['display'] == '1:1.2'


def test_lab_config_adds_every_transfer_to_each_batch(tmp_path):
    from pathlib import Path

    from rby1_scheduler.data_source import JsonDataSource
    from rby1_scheduler.lab_config import load_lab_config

    root = Path(__file__).parents[1]
    config = load_lab_config(root / 'config' / 'lab.yaml')
    source = JsonDataSource(
        root / 'mock_server' / 'server_input.json',
        tmp_path / 'out.json',
        tmp_path / 'events.jsonl',
    )
    data = config.enrich_snapshot(source.read_snapshot())
    groups = build_batch_groups({'batches': data['batches'], 'queue': data['work_queue']})

    expected = [
        step.step_id for step in config.workflow.values()
        if step.kind == 'transfer'
    ]
    assert len(groups) == 10
    assert [task['task_id'].rsplit('.', 1)[-1] for task in groups[0]['tasks']] == expected
