from rby1_scheduler.ui.gantt_chart import build_gantt_rows


def test_gantt_rows_use_resource_order_and_reuse_available_lanes():
    timeline = [
        {
            'task_id': 'B001.S1',
            'kind': 'process',
            'resource': 'INC',
            'route': 'INC',
            'start_min': 0,
            'end_min': 10,
            'duration_min': 10,
        },
        {
            'task_id': 'B002.S1',
            'kind': 'process',
            'resource': 'INC',
            'route': 'INC',
            'start_min': 5,
            'end_min': 8,
            'duration_min': 3,
        },
        {
            'task_id': 'B003.T01',
            'kind': 'transfer',
            'resource': 'ROBOT',
            'route': 'INBOX -> LH1',
            'start_min': 0,
            'end_min': 5,
            'duration_min': 5,
        },
        {
            'task_id': 'B001.S2',
            'kind': 'process',
            'resource': 'INC',
            'route': 'INC',
            'start_min': 10,
            'end_min': 12,
            'duration_min': 2,
        },
    ]

    rows = build_gantt_rows(timeline)

    assert [row.resource for row in rows] == ['ROBOT', 'INC']
    incubator = rows[1]
    assert incubator.lane_count == 2
    assert [(bar.task_id, bar.lane) for bar in incubator.bars] == [
        ('B001.S1', 0),
        ('B002.S1', 1),
        ('B001.S2', 0),
    ]


def test_gantt_rows_ignore_invalid_intervals():
    rows = build_gantt_rows([
        {
            'task_id': 'B001.BAD',
            'resource': 'LH1',
            'start_min': 10,
            'end_min': 5,
        },
        {
            'task_id': 'B001.S1',
            'resource': 'LH2',
            'start_min': 0,
            'end_min': 20,
        },
    ])

    assert len(rows) == 1
    assert rows[0].resource == 'LH2'
