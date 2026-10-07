"""ROS-free checks for capture post-processing and stage diagnosis."""

import importlib.util
import json
import math
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


def timing_message(role, stage, kind, session, stamp, mono_ms, wall_ms=None, **fields):
    wall_ms = mono_ms if wall_ms is None else wall_ms
    state = {
        'schema_version': 1, 'stage': stage, 'role': role,
        'kind': kind, 'session_id': session, 'source_stamp_ns': stamp,
        'monotonic_ns': int(mono_ms * 1_000_000),
        'wall_ns': int(wall_ms * 1_000_000),
    }
    state.update(fields)
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


def test_bridge_summary_preserves_bounded_pose_fifo_drop_counter():
    result = analysis.bridge_summary([{
        'state': {
            'connected': True, 'session_id': 'session-a', 'reason': 'connected',
            'tx_mailbox': {'dropped_oldest': {'tracking_odom': 2},
                           'discarded_on_clear': {'slam_odom': 1}},
            'rx_mailbox': {'dropped_oldest': {'slam_odom': 3}},
        },
    }])
    assert result['mailboxes']['tx_mailbox']['dropped_oldest'] == {
        'tracking_odom': 2}
    assert result['mailboxes']['rx_mailbox']['dropped_oldest'] == {
        'slam_odom': 3}
    assert result['mailboxes']['tx_mailbox']['discarded_on_clear'] == {
        'slam_odom': 1}


def test_report_counts_explicit_pose_adapter_drop_reasons():
    timing = [
        timing_message('upc', 'upc_pose_adapter_dropped', 'odom', '', 100, 1,
                       reason='stale'),
        timing_message('upc', 'upc_pose_adapter_dropped', 'odom', '', 200, 2,
                       reason='tf_timeout'),
        timing_message('upc', 'upc_pose_adapter_dropped', 'odom', '', 300, 3,
                       reason='stale'),
    ]
    report = analysis.build_report([
        capture('upc', {'/rby1/vslam/timing': timing})], stale_ms=500.0)
    assert report['pose_adapter_drops']['upc'] == {
        'samples': 3, 'reasons': {'stale': 2, 'tf_timeout': 1}}
    assert any('PoseAdapter explicitly dropped 3 poses' in item
               for item in report['findings'])


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
    assert result['raw_return_samples'][0]['latency_ms'] == 72.0
    assert result['sessions'][0]['raw_returned'] == 1
    assert result['sessions'][0]['funnel'][-1]['count'] == 1
    assert result['delivery_bins'][0]['final_fraction'] == 1.0
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


def test_goal_status_six_is_reported_as_aborted_once():
    records = [
        {'observed_wall_ns': 1, 'observed_monotonic_ns': 1,
         'goal_statuses': [{'goal_id': 'abc', 'status': 1}]},
        {'observed_wall_ns': 2, 'observed_monotonic_ns': 2,
         'goal_statuses': [{'goal_id': 'abc', 'status': 2}]},
        {'observed_wall_ns': 3, 'observed_monotonic_ns': 3,
         'goal_statuses': [{'goal_id': 'abc', 'status': 6}]},
        {'observed_wall_ns': 4, 'observed_monotonic_ns': 4,
         'goal_statuses': [{'goal_id': 'abc', 'status': 6}]},
    ]
    result = analysis.goal_status_summary(records)
    assert result['status_6_aborted_goals'] == 1
    assert result['status_5_canceled_goals'] == 0
    assert result['final_status_names'] == {'ABORTED': 1}
    assert [item['status_name'] for item in result['transitions']] == [
        'ACCEPTED', 'EXECUTING', 'ABORTED']


def test_localization_correction_wraps_yaw_and_segments_sessions():
    def tf_record(index, session, x, yaw):
        return {
            'observed_monotonic_ns': index * 10_000_000,
            'observed_wall_ns': index * 10_000_000,
            'bridge_session_id': session,
            'transforms': [{
                'parent_frame': 'vslam_map', 'child_frame': 'odom',
                'translation': [x, 0.0, 0.0], 'yaw': yaw,
            }],
        }

    records = [
        tf_record(1, 'a', 0.0, math.radians(179.0)),
        tf_record(2, 'a', 0.01, math.radians(-179.0)),
        tf_record(3, 'a', 0.08, math.radians(-179.0)),
        tf_record(4, 'b', 2.0, 1.0),  # New session is not a correction jump.
    ]
    odom_records = [
        {'observed_monotonic_ns': index * 10_000_000,
         'velocity': [0.0, 0.0, 0.0]}
        for index in range(1, 5)
    ]
    result = analysis.localization_correction_summary(
        records, odom_records, jump_m=0.05, jump_rad=math.radians(5.0))
    assert result['segments'] == 2
    assert result['jump_count'] == 1
    assert result['goal_tolerance_exceedance_count'] == 2
    assert result['stationary_matched_steps'] == 2
    assert result['stationary_goal_tolerance_exceedance_count'] == 2
    assert math.isclose(result['update_gap_ms']['max'], 10.0)
    assert math.isclose(result['yaw_step_rad']['max'], math.radians(2.0), abs_tol=1e-9)
    assert math.isclose(result['jumps'][0]['translation_m'], 0.07, abs_tol=1e-9)


def test_marked_mapping_loop_reports_stationary_jitter_and_return_error():
    def odom(index, mono_ns, x, yaw=0.0):
        return message(
            'upc', '/rby1/vslam/slam_odom', index, mono_ns, mono_ns,
            position=[x, 0.0, 0.0],
            orientation=[0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)],
            velocity=[0.0, 0.0, 0.0], frame_id='vslam_map',
            child_frame_id='base')

    records = [
        odom(1, 1_100_000_000, 0.000),
        odom(2, 2_100_000_000, 0.002),
        odom(3, 4_100_000_000, 0.500),
        odom(4, 6_100_000_000, 0.030, math.radians(2.0)),
        odom(5, 7_100_000_000, 0.032, math.radians(2.2)),
    ]
    imu = [
        message('upc', '/d435/d435/imu', index, mono_ns, mono_ns,
                angular_velocity=[0.0, 0.0, 0.01 * index],
                linear_acceleration=[0.01 * index, 0.0, 9.8])
        for index, mono_ns in enumerate(
            (1_200_000_000, 2_200_000_000, 6_200_000_000, 7_200_000_000), 1)
    ]
    data = capture('upc', {
        '/rby1/vslam/slam_odom': records,
        '/d435/d435/imu': imu,
    })
    data['markers'] = [
        {'phase': 'initial_stationary', 'monotonic_ns': 1_000_000_000},
        {'phase': 'mapping_motion', 'monotonic_ns': 4_000_000_000},
        {'phase': 'returned_stationary', 'monotonic_ns': 6_000_000_000},
        {'phase': 'capture_end', 'monotonic_ns': 8_000_000_000},
    ]
    result = analysis.motion_debug_summary(data)
    pose = result['pose_topics']['/rby1/vslam/slam_odom']
    assert result['complete'] is True
    assert pose['phases']['initial_stationary']['samples'] == 2
    assert math.isclose(
        pose['phases']['initial_stationary']['translation_jitter_m']['max'], 0.001)
    assert math.isclose(pose['return_error']['translation_m'], 0.03)
    assert math.isclose(pose['return_error']['yaw_rad'], math.radians(2.1))
    assert result['imu']['phases']['returned_stationary']['samples'] == 2


def test_parameter_events_keep_false_and_deletion():
    records = [{
        'observed_monotonic_ns': 1, 'observed_wall_ns': 2,
        'parameter_node': '/rby1/vslam/nav2/controller_server',
        'new_parameters': [{'name': 'debug', 'value': False}],
        'changed_parameters': [{'name': 'goal_checker.xy_goal_tolerance',
                                'value': 0.01}],
        'deleted_parameters': ['old'],
    }]
    result = analysis.parameter_event_summary(records)
    assert result['messages'] == 1
    assert result['event_count'] == 3
    assert result['events'][0]['value'] is False
    assert result['events'][-1]['operation'] == 'deleted'


def test_parameter_event_summary_keeps_runtime_tail_after_declarations():
    records = []
    for index in range(501):
        records.append({
            'observed_monotonic_ns': index,
            'parameter_node': '/rby1/vslam/nav2/controller_server',
            'new_parameters': [{'name': f'initial_{index}', 'value': index}],
            'changed_parameters': [], 'deleted_parameters': [],
        })
    records.append({
        'observed_monotonic_ns': 1000,
        'parameter_node': '/rby1/vslam/nav2/controller_server',
        'new_parameters': [],
        'changed_parameters': [{'name': 'goal_checker.xy_goal_tolerance',
                                'value': 0.02}],
        'deleted_parameters': [],
    })
    result = analysis.parameter_event_summary(records)
    assert result['truncated'] is True
    assert result['discarded_initial_events'] == 2
    assert result['events'][-1]['operation'] == 'changed'
    assert result['events'][-1]['value'] == 0.02


def test_path_summary_compares_plan_endpoint_to_structured_goal():
    requested = {
        'observed_monotonic_ns': 1, 'state': {
            'event': 'requested', 'request_id': 'nav-1',
            'goal': {'name': 'Point 2', 'x': 0.1, 'y': 0.0, 'yaw': 0.0},
        },
    }
    accepted = {
        'observed_monotonic_ns': 2, 'state': {
            'event': 'accepted', 'request_id': 'nav-1', 'goal_id': 'action-1',
            'goal': {'name': 'Point 2', 'x': 0.1, 'y': 0.0, 'yaw': 0.0},
        },
    }
    plan = {
        'observed_monotonic_ns': 3, 'observed_wall_ns': 3,
        'pose_count': 2, 'path_length_m': 0.05, 'direct_distance_m': 0.05,
        'max_step_m': 0.05, 'end_pose': [0.05, 0.0, 0.0],
    }
    result = analysis.path_quality_summary([plan], [], [requested, accepted])
    assert result['plans'] == 1
    assert result['requested_goals_seen'] == 1
    assert result['accepted_goals_seen'] == 1
    assert result['matched_plans'] == 1
    assert math.isclose(result['endpoint_error_m']['max'], 0.05)
    assert result['samples'][0]['request_id'] == 'nav-1'


def test_path_summary_does_not_match_rejected_finished_or_nonfinite_endpoint():
    goal = {'name': 'close', 'x': 0.02, 'y': 0.0, 'yaw': 0.0}
    events = [
        {'observed_monotonic_ns': 1, 'state': {
            'event': 'requested', 'request_id': 'rejected', 'goal': goal}},
        {'observed_monotonic_ns': 2, 'state': {
            'event': 'rejected', 'request_id': 'rejected', 'goal': goal}},
        {'observed_monotonic_ns': 3, 'state': {
            'event': 'requested', 'request_id': 'accepted', 'goal': goal}},
        {'observed_monotonic_ns': 4, 'state': {
            'event': 'accepted', 'request_id': 'accepted', 'goal_id': 'id', 'goal': goal}},
        {'observed_monotonic_ns': 6, 'state': {
            'event': 'finished', 'request_id': 'accepted', 'goal_id': 'id',
            'status': 6, 'goal': goal}},
    ]
    base = {'observed_wall_ns': 10, 'pose_count': 1, 'path_length_m': 0.0,
            'direct_distance_m': 0.0, 'max_step_m': 0.0}
    plans = [
        dict(base, observed_monotonic_ns=2, end_pose=[0.0, 0.0, 0.0]),
        dict(base, observed_monotonic_ns=5, end_pose=['nan', 0.0, 0.0]),
        dict(base, observed_monotonic_ns=7, end_pose=[0.0, 0.0, 0.0]),
    ]
    result = analysis.path_quality_summary(plans, [], events)
    assert result['requested_goals_seen'] == 2
    assert result['accepted_goals_seen'] == 1
    assert result['matched_plans'] == 0
    assert result['endpoint_error_m'] is None
