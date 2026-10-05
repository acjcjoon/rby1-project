"""Offline plotting smoke test with no ROS dependencies."""

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


pytest.importorskip('matplotlib')


def stat(mean, p95=None):
    return {'count': 2, 'min': mean, 'mean': mean, 'p50': mean,
            'p95': mean if p95 is None else p95, 'p99': mean, 'max': mean}


def test_report_generates_three_png_files(tmp_path):
    sample = {
        'session_id': 'abcdef0123456789', 'source_stamp_ns': 1,
        'upc_start_wall_ns': 1_000_000_000, 'total_ms': 80.0,
        'returned_pose_ms': 70.0, 'transport_queue_residual_ms': 45.0,
        'upc_tx_queue_socket_ms': 8.0, 'network_roundtrip_residual_ms': 20.0,
        'lab_input_delivery_ms': 5.0, 'lab_tx_queue_socket_ms': 4.0,
        'upc_return_delivery_ms': 15.0,
        'lab_bridge_publish_ms': 2.0, 'lab_vslam_ms': 20.0,
        'upc_bridge_publish_ms': 2.0, 'upc_dds_delivery_ms': 3.0,
        'pose_adapter_ms': 8.0, 'forward_one_way_wall_ms': 20.0,
        'return_one_way_wall_ms': 25.0, 'over_threshold': False,
    }
    session = {
        'session_id': sample['session_id'], 'started': 2, 'complete': 2,
        'completion_fraction': 1.0, 'samples_over_threshold': 0,
        'total_ms': stat(80.0, 95.0),
    }
    for field in (
        'returned_pose_ms', 'transport_queue_residual_ms',
        'upc_tx_queue_socket_ms', 'network_roundtrip_residual_ms',
        'lab_input_delivery_ms', 'lab_tx_queue_socket_ms',
        'upc_return_delivery_ms',
        'lab_bridge_publish_ms', 'lab_vslam_ms', 'upc_bridge_publish_ms',
        'upc_dds_delivery_ms', 'pose_adapter_ms',
        'forward_one_way_wall_ms', 'return_one_way_wall_ms',
    ):
        session[field] = stat(sample[field])
    report = tmp_path / 'timing_report.json'
    report.write_text(json.dumps({
        'stale_threshold_ms': 500.0,
        'end_to_end': {'samples': [sample, dict(sample, source_stamp_ns=2,
                                                upc_start_wall_ns=1_033_000_000)],
                       'sessions': [session]},
    }), encoding='utf-8')
    output = tmp_path / 'plots'
    script = Path(__file__).resolve().parents[1] / 'scripts/plot_vslam_timing.py'
    env = dict(os.environ, MPLCONFIGDIR=str(tmp_path / 'matplotlib'))
    result = subprocess.run(
        [sys.executable, str(script), '--report', str(report),
         '--output-dir', str(output), '--threshold-ms', '500'],
        capture_output=True, text=True, env=env, timeout=30)
    assert result.returncode == 0, result.stderr
    assert {path.name for path in output.glob('*.png')} == {
        'total_latency_by_session.png', 'session_stage_breakdown.png',
        'one_way_wall_clock_diagnostic.png',
    }
    assert all(path.stat().st_size > 1000 for path in output.glob('*.png'))
