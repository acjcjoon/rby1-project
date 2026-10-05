"""ROS-free checks for capture post-processing and stage diagnosis."""

import importlib.util
import json
from pathlib import Path


spec = importlib.util.spec_from_file_location(
    'analyze_vslam_timing',
    Path(__file__).resolve().parents[1] / 'scripts/analyze_vslam_timing.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def message(role, topic, index, stamp_ns, mono_ns, wall_offset_ns=0, **fields):
    result = {
        'event': 'message', 'role': role, 'topic': topic,
        'observer_index': index, 'source_stamp_ns': stamp_ns,
        'observed_monotonic_ns': mono_ns,
        'observed_wall_ns': mono_ns + wall_offset_ns,
        'observed_ros_ns': mono_ns + wall_offset_ns,
    }
    result.update(fields)
    return result


def capture(role, topics):
    return {
        'path': f'/tmp/{role}', 'events_path': f'/tmp/{role}/events.jsonl',
        'role': role, 'messages': topics, 'malformed_lines': 0, 'start': {},
        'subscription_unavailable': [], 'last_observer_health': None,
    }


def timing_message(role, stage, kind, session, stamp, mono_ms, wall_ms=None):
    wall_ms = mono_ms if wall_ms is None else wall_ms
    state = {
        'schema_version': 1, 'stage': stage, 'role': role,
        'kind': kind, 'session_id': session, 'source_stamp_ns': stamp,
        'monotonic_ns': int(mono_ms * 1_000_000),
        'wall_ns': int(wall_ms * 1_000_000),
    }
    return {
        'event': 'message', 'role': role, 'topic': '/rby1/vslam/timing',
        'state': state, 'source_stamp_ns': stamp,
        'observed_monotonic_ns': state['monotonic_ns'] + 1000,
        'observed_wall_ns': state['wall_ns'] + 1000,
    }


def test_topic_summary_reports_rate_and_stale_gap():
    records = [
        message('lab', '/pose', 1, 1_000_000_000, 1_010_000_000),
        message('lab', '/pose', 2, 1_033_000_000, 1_043_000_000),
        message('lab', '/pose', 3, 1_700_000_000, 1_710_000_000),
    ]
    result = analysis.topic_summary(records, stale_ns=500_000_000)
    assert result['gaps_over_stale'] == 1
    assert result['observed_gap_ms']['max'] == 667.0
    assert result['source_age_ms']['max'] == 10.0


def test_exact_stamp_matching_does_not_invent_nearest_latency():
    left = [
        message('lab', '/image', 1, 100, 1_000),
        message('lab', '/image', 2, 200, 2_000),
    ]
    right = [
        message('lab', '/pose', 1, 100, 1_400),
        message('lab', '/pose', 2, 201, 2_400),
    ]
    result = analysis.match_latency(left, right, 'observed_monotonic_ns', 500.0)
    assert result['matched'] == 1
    assert result['left_match_fraction'] == 0.5


def test_report_flags_cuvslam_when_images_continue_but_pose_stalls():
    image_topic = '/d435/d435/infra1/camera_info'
    pose_topic = '/rby1/vslam/camera_odometry'
    images = []
    for index in range(31):
        stamp = 1_000_000_000 + index * 33_000_000
        images.append(message('lab', image_topic, index + 1, stamp, stamp + 2_000_000))
    poses = [
        message('lab', pose_topic, 1, images[0]['source_stamp_ns'], 1_005_000_000),
        message('lab', pose_topic, 2, images[-1]['source_stamp_ns'], 1_995_000_000),
    ]
    report = analysis.build_report([capture('lab', {
        image_topic: images, pose_topic: poses,
    })], stale_ms=500.0)
    assert any('cuVSLAM tracking or LAB compute' in item for item in report['findings'])


def test_internal_events_form_clock_independent_stacked_sample():
    stamp = 9_000_000_000
    session = 'session-a'
    upc = [
        timing_message('upc', 'upc_stereo_enqueued', 'stereo', session, stamp, 100, 1000),
        timing_message('upc', 'upc_stereo_socket_sent', 'stereo', session, stamp, 110, 1010),
        timing_message('upc', 'upc_pose_socket_received', 'tracking_odom', session, stamp, 165, 1068),
        timing_message('upc', 'upc_pose_dequeued', 'tracking_odom', session, stamp, 170, 1070),
        timing_message('upc', 'upc_pose_published', 'tracking_odom', session, stamp, 172, 1072),
        timing_message('upc', 'upc_pose_adapter_received', 'odom', '', stamp, 175, 1075),
        timing_message('upc', 'upc_pose_adapter_published', 'odom', '', stamp, 180, 1080),
    ]
    # LAB wall time has a +40 ms clock offset. The one-way fields inherit it,
    # while the total and residual remain based on same-host durations.
    lab = [
        timing_message('lab', 'lab_stereo_socket_received', 'stereo', session, stamp, 195, 1050),
        timing_message('lab', 'lab_stereo_dequeued', 'stereo', session, stamp, 200, 1050),
        timing_message('lab', 'lab_stereo_published', 'stereo', session, stamp, 202, 1052),
        timing_message('lab', 'lab_pose_enqueued', 'tracking_odom', session, stamp, 222, 1072),
        timing_message('lab', 'lab_pose_socket_sent', 'tracking_odom', session, stamp, 230, 1080),
    ]
    result = analysis.build_end_to_end([
        capture('upc', {'/rby1/vslam/timing': upc}),
        capture('lab', {'/rby1/vslam/timing': lab}),
    ])
    assert result['complete'] == 1
    sample = result['samples'][0]
    assert sample['total_ms'] == 80.0
    assert sample['lab_bridge_publish_ms'] == 2.0
    assert sample['lab_vslam_ms'] == 20.0
    assert sample['upc_bridge_publish_ms'] == 2.0
    assert sample['upc_dds_delivery_ms'] == 3.0
    assert sample['pose_adapter_ms'] == 5.0
    assert sample['transport_queue_residual_ms'] == 48.0
    assert sample['upc_tx_queue_socket_ms'] == 10.0
    assert sample['network_roundtrip_residual_ms'] == 20.0
    assert sample['lab_input_delivery_ms'] == 7.0
    assert sample['lab_tx_queue_socket_ms'] == 8.0
    assert sample['upc_return_delivery_ms'] == 10.0
    assert sample['forward_one_way_wall_ms'] == 50.0
    assert sample['return_one_way_wall_ms'] == -2.0
    assert sample['forward_socket_wall_ms'] == 40.0
    assert sample['return_socket_wall_ms'] == -12.0


def test_cli_writes_human_and_json_reports(tmp_path):
    run_dir = tmp_path / 'trial_upc'
    run_dir.mkdir()
    records = [
        {'event': 'start', 'role': 'upc', 'host': 'upc'},
        message('upc', '/rby1/vslam/camera_odometry', 1,
                1_000_000_000, 1_010_000_000),
    ]
    (run_dir / 'events.jsonl').write_text(
        ''.join(json.dumps(record) + '\n' for record in records), encoding='utf-8')
    assert analysis.main([str(run_dir)]) == 0
    assert (run_dir / 'timing_report.txt').is_file()
    parsed = json.loads((run_dir / 'timing_report.json').read_text(encoding='utf-8'))
    assert parsed['captures'][0]['role'] == 'upc'
