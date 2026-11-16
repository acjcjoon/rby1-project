"""File/service failure paths and odom session isolation for nvblox map operations."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from rby1_vslam import nvblox_map_tool as tool


PARAMETERS = {'global_frame': 'odom', 'voxel_size': 0.05, 'mapping_type': 'static_tsdf'}
STATUS = {'stationary_ready': True, 'navigation_idle': True,
          'correction_pending': False, 'session_id': 'session-a', 'odom_frame': 'odom'}


def test_paths_match_server_extensions_and_preserve_existing_maps(tmp_path):
    path = tool.validate_path(str(tmp_path / 'map'), 'save')
    assert path == tmp_path / 'map.nvblx'
    path.write_bytes(b'old map')
    with pytest.raises(ValueError, match='already exists'):
        tool.validate_path(str(path), 'save')
    assert path.read_bytes() == b'old map'
    with pytest.raises(ValueError, match='metadata'):
        tool.validate_path(str(path), 'load')
    with pytest.raises(ValueError, match='absolute'):
        tool.validate_path('map.nvblx', 'save')
    with pytest.raises(ValueError, match='.ply'):
        tool.validate_path(str(tmp_path / 'map.nvblx'), 'mesh')


@pytest.mark.parametrize('field,value', [
    ('stationary_ready', False), ('stationary_ready', 1),
    ('navigation_idle', False), ('correction_pending', True),
    ('session_id', ''), ('odom_frame', 'new_odom'),
])
def test_stop_guard_rejects_moving_busy_or_invalid_state(field, value):
    stopped = tool.StopState()
    stopped.update(json.dumps({**STATUS, field: value}), 10.)
    assert not stopped.ready(10.1)


def test_stop_guard_rejects_stale_and_invalid_json():
    stopped = tool.StopState()
    stopped.update(json.dumps(STATUS), 10.)
    assert stopped.ready(10.2)
    assert not stopped.ready(11.)
    assert not stopped.ready(9.)
    stopped.update('[]', 11.)
    assert not stopped.ready(11.)
    stopped.update('{invalid}', 12.)
    assert not stopped.ready(12.)


def test_load_requires_same_coordinates_and_voxel_geometry():
    metadata = dict(schema_version=1, localization_session_id='session-a', **PARAMETERS)
    tool.validate_metadata(metadata, 'session-a', PARAMETERS)
    with pytest.raises(ValueError, match='different localization/odom session'):
        tool.validate_metadata(metadata, 'session-b', PARAMETERS)
    with pytest.raises(ValueError, match='voxel size'):
        tool.validate_metadata({**metadata, 'voxel_size': 0.1}, 'session-a', PARAMETERS)
    with pytest.raises(ValueError, match='frame'):
        tool.validate_metadata({**metadata, 'global_frame': 'vslam_map'}, 'session-a', PARAMETERS)
    with pytest.raises(ValueError, match='mapping type'):
        tool.validate_metadata({**metadata, 'mapping_type': 'dynamic'}, 'session-a', PARAMETERS)


class Service:
    Request = SimpleNamespace


class FakeRos:
    @staticmethod
    def ok():
        return True

    @staticmethod
    def spin_once(node, timeout_sec):
        node.clock += timeout_sec


class FakeNode:
    def __init__(self, *, success=True, create_output=True, pending=False):
        self.success = success
        self.create_output = create_output
        self.pending = pending
        self.clock = 10.
        self.calls = []
        self.destroyed = False

    def create_client(self, service_type, name):
        assert service_type is Service
        self.name = name
        return self

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.calls.append(request)
        if self.create_output and not self.name.endswith('/load_map'):
            Path(request.file_path).write_bytes(b'saved data')
        return SimpleNamespace(done=lambda: not self.pending, exception=lambda: None,
                               result=lambda: SimpleNamespace(success=self.success))

    def destroy_client(self, client):
        assert client is self
        self.destroyed = True


def operation(tmp_path, monkeypatch, node, command='save'):
    monkeypatch.setattr(tool.time, 'monotonic', lambda: node.clock)
    stopped = tool.StopState()
    stopped.update(json.dumps(STATUS), node.clock)
    filename = 'map.ply' if command == 'mesh' else 'map.nvblx'
    args = SimpleNamespace(command=command, service_prefix='/nvblox_node',
                           path=tmp_path / filename, service_timeout=0.2, timeout=0.3)
    return tool.perform_operation(FakeRos, node, args, stopped, PARAMETERS, Service)


def test_successful_save_checks_file_then_writes_session_metadata(tmp_path, monkeypatch):
    node = FakeNode()
    assert operation(tmp_path, monkeypatch, node) == 0
    assert node.name == '/nvblox_node/save_map'
    metadata = json.loads((tmp_path / 'map.nvblx.json').read_text())
    assert metadata['localization_session_id'] == 'session-a'
    assert metadata['artifact'] == 'static_mapper_layer_cake'
    assert node.destroyed


@pytest.mark.parametrize('options,error', [
    ({'success': False}, 'rejected'),
    ({'create_output': False}, 'absent/empty'),
    ({'pending': True}, 'operation may still be running'),
])
def test_failed_or_timed_out_save_never_creates_usable_metadata(
        tmp_path, monkeypatch, options, error):
    node = FakeNode(**options)
    with pytest.raises((RuntimeError, TimeoutError), match=error):
        operation(tmp_path, monkeypatch, node)
    assert not (tmp_path / 'map.nvblx.json').exists()
    assert node.destroyed


def test_load_different_session_is_rejected_before_service_call(tmp_path, monkeypatch):
    (tmp_path / 'map.nvblx').write_bytes(b'saved data')
    metadata = dict(schema_version=1, localization_session_id='other-session', **PARAMETERS)
    (tmp_path / 'map.nvblx.json').write_text(json.dumps(metadata))
    node = FakeNode()
    with pytest.raises(ValueError, match='different localization/odom session'):
        operation(tmp_path, monkeypatch, node, 'load')
    assert node.calls == []
    assert node.destroyed


def test_load_same_session_uses_load_service_without_changing_file(tmp_path, monkeypatch):
    path = tmp_path / 'map.nvblx'
    path.write_bytes(b'original map')
    metadata = dict(schema_version=1, localization_session_id='session-a', **PARAMETERS)
    (tmp_path / 'map.nvblx.json').write_text(json.dumps(metadata))
    node = FakeNode()
    assert operation(tmp_path, monkeypatch, node, 'load') == 0
    assert node.name == '/nvblox_node/load_map'
    assert path.read_bytes() == b'original map'


def test_mesh_exports_explicit_ply_file_without_load_metadata(tmp_path, monkeypatch):
    node = FakeNode()
    assert operation(tmp_path, monkeypatch, node, 'mesh') == 0
    assert node.name == '/nvblox_node/save_ply'
    assert node.calls[0].file_path.endswith('.ply')
    assert (tmp_path / 'map.ply').is_file()
    assert not (tmp_path / 'map.ply.json').exists()
