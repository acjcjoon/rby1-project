from pathlib import Path
from rby1_scheduler.data_source import JsonDataSource

def test_mock_server_contains_ten_abcd_batches(tmp_path):
    root = Path(__file__).parents[1]
    source = JsonDataSource(root / 'mock_server' / 'server_input.json', tmp_path / 'out.json', tmp_path / 'events.jsonl')
    data = source.read_snapshot()
    plates = [plate for batch in data['batches'] for plate in batch['plates'].values()]
    assert len(data['batches']) == 10
    assert len(plates) == 40
    assert len({plate['barcode'] for plate in plates}) == 40
    for batch in data['batches']:
        assert set(batch['plates']) == {'A', 'B', 'C', 'D'}
        assert batch['molar_ratio']['display']
