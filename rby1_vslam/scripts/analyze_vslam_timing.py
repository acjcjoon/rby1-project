#!/usr/bin/env python3
"""Summarize one or two rby1_vslam timing captures without ROS dependencies."""

import argparse
from collections import Counter, defaultdict, deque
import csv
import json
import math
from pathlib import Path
import statistics
import sys


SELECTED_TOPICS = (
    '/d435/d435/infra1/camera_info',
    '/d435/d435/infra2/camera_info',
    '/d435/d435/imu',
    '/visual_slam/status',
    '/rby1/vslam/camera_odometry',
    '/rby1/vslam/camera_slam_odometry',
    '/rby1/vslam/odom',
    '/rby1/vslam/slam_odom',
    '/rby1/vslam/bridge_status',
    '/rby1/vslam/timing',
    '/rby1/vslam/localization_status',
    '/rby1/vslam/navigation_status',
)

RELEVANT_LOG_TERMS = (
    'tracking is lost', 'failed to track', 'failed to get slam pose',
    'stale', 'future clock', 'queue overflow', 'heartbeat timed out',
    'rejected tcp', 'tcp bridge', 'cancel', 'failed to make progress',
    'progress checker', 'controller patience exceeded', 'no valid trajectories',
    'optimizer fail to compute path', 'control loop missed its desired rate',
    'no valid path', 'failed to create plan', 'failed to transform',
    'could not transform', 'transform timeout', 'transform data too old',
    'outside map', 'occupied', 'abort',
)

GOAL_STATUS_NAMES = {
    0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
}

EXPECTED_TOPICS = {
    'upc': (
        '/d435/d435/infra1/camera_info',
        '/d435/d435/infra2/camera_info',
        '/rby1/vslam/bridge_status',
        '/rby1/vslam/camera_odometry',
        '/rby1/vslam/camera_slam_odometry',
        '/rby1/vslam/odom',
        '/rby1/vslam/slam_odom',
    ),
    'lab': (
        '/d435/d435/infra1/camera_info',
        '/d435/d435/infra2/camera_info',
        '/rby1/vslam/bridge_status',
        '/rby1/vslam/camera_odometry',
        '/rby1/vslam/camera_slam_odometry',
        '/visual_slam/status',
    ),
}


def percentile(values, quantile):
    """Linear percentile for a non-empty numeric sequence."""
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def numeric_stats(values):
    if not values:
        return None
    return {
        'count': len(values),
        'min': min(values),
        'mean': statistics.fmean(values),
        'p50': percentile(values, 0.50),
        'p95': percentile(values, 0.95),
        'p99': percentile(values, 0.99),
        'max': max(values),
    }


def ns_stats_ms(values):
    return numeric_stats([value / 1_000_000.0 for value in values])


def positive_deltas(records, key):
    values = []
    nonpositive = 0
    previous = None
    for record in records:
        value = record.get(key)
        if not isinstance(value, int):
            continue
        if previous is not None:
            delta = value - previous
            if delta > 0:
                values.append(delta)
            else:
                nonpositive += 1
        previous = value
    return values, nonpositive


def sustained_rate_hz(records, key):
    values = [record.get(key) for record in records if isinstance(record.get(key), int)]
    if len(values) < 2 or values[-1] <= values[0]:
        return None
    return (len(values) - 1) * 1_000_000_000.0 / (values[-1] - values[0])


def topic_summary(records, stale_ns):
    records = sorted(records, key=lambda item: item.get('observed_monotonic_ns', 0))
    observed_gaps, observed_nonpositive = positive_deltas(records, 'observed_monotonic_ns')
    source_records = [item for item in records
                      if isinstance(item.get('source_stamp_ns'), int)
                      and item['source_stamp_ns'] > 0]
    source_gaps, source_nonpositive = positive_deltas(source_records, 'source_stamp_ns')
    source_ages = [item['observed_ros_ns'] - item['source_stamp_ns']
                   for item in source_records
                   if isinstance(item.get('observed_ros_ns'), int)]
    max_gap_at_wall_ns = None
    if len(records) > 1:
        gap_pairs = [(records[index]['observed_monotonic_ns'] -
                      records[index - 1]['observed_monotonic_ns'],
                      records[index].get('observed_wall_ns'))
                     for index in range(1, len(records))
                     if isinstance(records[index].get('observed_monotonic_ns'), int)
                     and isinstance(records[index - 1].get('observed_monotonic_ns'), int)]
        positive_pairs = [item for item in gap_pairs if item[0] > 0]
        if positive_pairs:
            max_gap_at_wall_ns = max(positive_pairs)[1]
    return {
        'messages': len(records),
        'observed_rate_hz': sustained_rate_hz(records, 'observed_monotonic_ns'),
        'source_rate_hz': sustained_rate_hz(source_records, 'source_stamp_ns'),
        'observed_gap_ms': ns_stats_ms(observed_gaps),
        'source_gap_ms': ns_stats_ms(source_gaps),
        'source_age_ms': ns_stats_ms(source_ages),
        'gaps_over_stale': sum(delta > stale_ns for delta in observed_gaps),
        'max_gap_at_wall_ns': max_gap_at_wall_ns,
        'nonpositive_observed_deltas': observed_nonpositive,
        'nonpositive_source_deltas': source_nonpositive,
    }


def load_capture(path):
    path = Path(path).expanduser().resolve()
    events_path = path if path.is_file() else path / 'events.jsonl'
    if not events_path.is_file():
        raise ValueError(f'events.jsonl not found: {path}')
    messages = defaultdict(list)
    starts = []
    subscription_unavailable = []
    last_health = None
    malformed = 0
    with events_path.open(encoding='utf-8') as stream:
        for line in stream:
            try:
                record = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                malformed += 1
                continue
            if record.get('event') == 'start':
                starts.append(record)
            elif record.get('event') == 'subscription_unavailable':
                subscription_unavailable.append(record)
            elif record.get('event') == 'observer_health':
                last_health = record
            if record.get('event') == 'message' and isinstance(record.get('topic'), str):
                messages[record['topic']].append(record)
    role_candidates = [item.get('role') for item in starts if item.get('role')]
    if not role_candidates:
        role_candidates = [item.get('role') for values in messages.values()
                           for item in values if item.get('role')]
    role = role_candidates[0] if role_candidates else 'unknown'
    return {
        'path': str(events_path.parent),
        'events_path': str(events_path),
        'role': role,
        'messages': dict(messages),
        'malformed_lines': malformed,
        'start': starts[0] if starts else {},
        'subscription_unavailable': subscription_unavailable,
        'last_observer_health': last_health,
    }


def match_latency(left_records, right_records, clock_field, stale_ms):
    right_by_stamp = defaultdict(deque)
    valid_right = 0
    for record in sorted(right_records, key=lambda item: item.get(clock_field, 0)):
        stamp = record.get('source_stamp_ns')
        timestamp = record.get(clock_field)
        if isinstance(stamp, int) and stamp > 0 and isinstance(timestamp, int):
            right_by_stamp[stamp].append(timestamp)
            valid_right += 1
    deltas_ms = []
    valid_left = 0
    for record in sorted(left_records, key=lambda item: item.get(clock_field, 0)):
        stamp = record.get('source_stamp_ns')
        timestamp = record.get(clock_field)
        if not (isinstance(stamp, int) and stamp > 0 and isinstance(timestamp, int)):
            continue
        valid_left += 1
        if right_by_stamp[stamp]:
            deltas_ms.append((right_by_stamp[stamp].popleft() - timestamp) / 1_000_000.0)
    stats = numeric_stats(deltas_ms)
    return {
        'left_messages_with_stamp': valid_left,
        'right_messages_with_stamp': valid_right,
        'matched': len(deltas_ms),
        'left_match_fraction': len(deltas_ms) / valid_left if valid_left else None,
        'right_match_fraction': len(deltas_ms) / valid_right if valid_right else None,
        'latency_ms': stats,
        'negative_samples': sum(value < 0 for value in deltas_ms),
        'samples_over_stale': sum(value > stale_ms for value in deltas_ms),
    }


def timing_events(capture):
    """Return validated internal boundary events from one passive capture."""
    result = []
    for record in capture.get('messages', {}).get('/rby1/vslam/timing', []):
        state = record.get('state')
        if not isinstance(state, dict):
            continue
        if not isinstance(state.get('stage'), str):
            continue
        if not all(isinstance(state.get(name), int)
                   for name in ('source_stamp_ns', 'wall_ns', 'monotonic_ns')):
            continue
        if state['source_stamp_ns'] <= 0:
            continue
        event = dict(state)
        event['observer_wall_ns'] = record.get('observed_wall_ns')
        event['observer_monotonic_ns'] = record.get('observed_monotonic_ns')
        result.append(event)
    return sorted(result, key=lambda event: event['monotonic_ns'])


def _first_after(events, monotonic_ns=None):
    if not events:
        return None
    if monotonic_ns is None:
        return events[0]
    return next((event for event in events
                 if event['monotonic_ns'] >= monotonic_ns), None)


def build_end_to_end(captures, stale_ms=500.0):
    """Match recorder-only stage events into clock-safe per-frame samples.

    The total is measured exclusively on UPC monotonic time. Each explicit
    component is also a duration on one host. Subtracting those components
    from the total produces a clock-independent transport/queue residual.
    One-way fields are diagnostic only because they use cross-host wall time.
    """
    by_role = {capture['role']: capture for capture in captures}
    if 'upc' not in by_role or 'lab' not in by_role:
        return {'started': 0, 'complete': 0, 'completion_fraction': None,
                'incomplete_reasons': {}, 'sessions': [], 'samples': []}
    upc_events = timing_events(by_role['upc'])
    lab_events = timing_events(by_role['lab'])

    def session_index(events):
        index = defaultdict(list)
        for event in events:
            session = event.get('session_id')
            stamp = event.get('source_stamp_ns')
            if session and isinstance(stamp, int):
                index[(event.get('stage'), event.get('kind'), session, stamp)].append(event)
        return index

    def stamp_index(events):
        index = defaultdict(list)
        for event in events:
            index[(event.get('stage'), event.get('kind'),
                   event.get('source_stamp_ns'))].append(event)
        return index

    upc_session = session_index(upc_events)
    lab_session = session_index(lab_events)
    upc_stamp = stamp_index(upc_events)
    starts = [event for event in upc_events
              if event.get('stage') == 'upc_stereo_enqueued'
              and event.get('kind') == 'stereo' and event.get('session_id')]
    samples = []
    incomplete = Counter()
    started_by_session = Counter(str(event['session_id']) for event in starts)

    def need(index, stage, kind, session, stamp, after=None):
        return _first_after(index.get((stage, kind, session, stamp), []), after)

    def need_stamp(stage, kind, stamp, after=None):
        return _first_after(upc_stamp.get((stage, kind, stamp), []), after)

    # Keep every raw tracking pose that made it back to UPC, including poses
    # that PoseAdapter later rejects as stale.  The original end-to-end sample
    # set intentionally contains only successful final base poses, which can
    # otherwise make a threshold plot look healthy by hiding rejected tails.
    raw_return_samples = []
    raw_return_by_key = {}
    for start in starts:
        session = str(start['session_id'])
        stamp = start['source_stamp_ns']
        returned = need(
            upc_session, 'upc_pose_published', 'tracking_odom',
            session, stamp, start['monotonic_ns'])
        if returned is None:
            continue
        latency_ms = (returned['monotonic_ns'] - start['monotonic_ns']) / 1_000_000.0
        sample = {
            'session_id': session, 'source_stamp_ns': stamp,
            'upc_start_wall_ns': start['wall_ns'],
            'latency_ms': latency_ms,
            'over_threshold': latency_ms > stale_ms,
        }
        raw_return_samples.append(sample)
        raw_return_by_key[(session, stamp)] = sample

    for start in starts:
        session = str(start['session_id'])
        stamp = start['source_stamp_ns']
        key_args = (session, stamp)
        boundaries = {
            'lab_dequeued': need(lab_session, 'lab_stereo_dequeued', 'stereo', *key_args),
            'lab_published': need(lab_session, 'lab_stereo_published', 'stereo', *key_args),
            'lab_pose': need(lab_session, 'lab_pose_enqueued', 'tracking_odom', *key_args),
            'upc_dequeued': need(upc_session, 'upc_pose_dequeued', 'tracking_odom', *key_args),
            'upc_published': need(upc_session, 'upc_pose_published', 'tracking_odom', *key_args),
        }
        missing = [name for name, event in boundaries.items() if event is None]
        if missing:
            incomplete['+'.join(missing)] += 1
            continue
        adapter_received = need_stamp(
            'upc_pose_adapter_received', 'odom', stamp,
            boundaries['upc_dequeued']['monotonic_ns'])
        adapter_published = need_stamp(
            'upc_pose_adapter_published', 'odom', stamp,
            adapter_received['monotonic_ns'] if adapter_received else None)
        if adapter_received is None or adapter_published is None:
            incomplete['pose_adapter'] += 1
            continue

        def duration_ms(right, left, clock='monotonic_ns'):
            return (right[clock] - left[clock]) / 1_000_000.0

        lab_bridge_ms = duration_ms(boundaries['lab_published'], boundaries['lab_dequeued'])
        vslam_ms = duration_ms(boundaries['lab_pose'], boundaries['lab_published'])
        upc_bridge_ms = duration_ms(boundaries['upc_published'], boundaries['upc_dequeued'])
        upc_dds_ms = duration_ms(adapter_received, boundaries['upc_published'])
        adapter_ms = duration_ms(adapter_published, adapter_received)
        total_ms = duration_ms(adapter_published, start)
        local_components = lab_bridge_ms + vslam_ms + upc_bridge_ms + upc_dds_ms + adapter_ms
        socket_boundaries = {
            'upc_sent': need(upc_session, 'upc_stereo_socket_sent', 'stereo', *key_args),
            'lab_received': need(
                lab_session, 'lab_stereo_socket_received', 'stereo', *key_args),
            'lab_sent': need(
                lab_session, 'lab_pose_socket_sent', 'tracking_odom', *key_args),
            'upc_received': need(
                upc_session, 'upc_pose_socket_received', 'tracking_odom', *key_args),
        }
        socket_complete = all(socket_boundaries.values())
        if socket_complete:
            upc_tx_ms = duration_ms(socket_boundaries['upc_sent'], start)
            lab_input_ms = duration_ms(
                boundaries['lab_published'], socket_boundaries['lab_received'])
            lab_tx_ms = duration_ms(socket_boundaries['lab_sent'], boundaries['lab_pose'])
            upc_return_ms = duration_ms(
                adapter_received, socket_boundaries['upc_received'])
            network_ms = total_ms - (
                upc_tx_ms + lab_input_ms + vslam_ms + lab_tx_ms
                + upc_return_ms + adapter_ms)
            forward_socket_ms = duration_ms(
                socket_boundaries['lab_received'], socket_boundaries['upc_sent'], 'wall_ns')
            return_socket_ms = duration_ms(
                socket_boundaries['upc_received'], socket_boundaries['lab_sent'], 'wall_ns')
        else:
            # Compatibility for captures made after boundary events were added
            # but before socket-worker tracing existed.
            upc_tx_ms = lab_tx_ms = 0.0
            lab_input_ms = lab_bridge_ms
            upc_return_ms = upc_bridge_ms + upc_dds_ms
            network_ms = total_ms - (
                lab_input_ms + vslam_ms + upc_return_ms + adapter_ms)
            forward_socket_ms = return_socket_ms = None
        samples.append({
            'session_id': session, 'source_stamp_ns': stamp,
            'upc_start_wall_ns': start['wall_ns'],
            'total_ms': total_ms,
            'returned_pose_ms': duration_ms(boundaries['upc_published'], start),
            'transport_queue_residual_ms': total_ms - local_components,
            'upc_tx_queue_socket_ms': upc_tx_ms,
            'network_roundtrip_residual_ms': network_ms,
            'lab_input_delivery_ms': lab_input_ms,
            'lab_bridge_publish_ms': lab_bridge_ms,
            'lab_vslam_ms': vslam_ms,
            'lab_tx_queue_socket_ms': lab_tx_ms,
            'upc_return_delivery_ms': upc_return_ms,
            'upc_bridge_publish_ms': upc_bridge_ms,
            'upc_dds_delivery_ms': upc_dds_ms,
            'pose_adapter_ms': adapter_ms,
            'forward_one_way_wall_ms': duration_ms(
                boundaries['lab_dequeued'], start, 'wall_ns'),
            'return_one_way_wall_ms': duration_ms(
                boundaries['upc_dequeued'], boundaries['lab_pose'], 'wall_ns'),
            'forward_socket_wall_ms': forward_socket_ms,
            'return_socket_wall_ms': return_socket_ms,
            'socket_boundaries_complete': socket_complete,
            'over_threshold': total_ms > stale_ms,
        })

    sessions = []
    fields = (
        'total_ms', 'returned_pose_ms', 'transport_queue_residual_ms',
        'upc_tx_queue_socket_ms', 'network_roundtrip_residual_ms',
        'lab_input_delivery_ms', 'lab_bridge_publish_ms', 'lab_vslam_ms',
        'lab_tx_queue_socket_ms', 'upc_return_delivery_ms',
        'upc_bridge_publish_ms', 'upc_dds_delivery_ms', 'pose_adapter_ms',
        'forward_one_way_wall_ms', 'return_one_way_wall_ms',
        'forward_socket_wall_ms', 'return_socket_wall_ms',
    )
    by_session = defaultdict(list)
    for sample in samples:
        by_session[sample['session_id']].append(sample)
    starts_by_session = defaultdict(list)
    for start in starts:
        starts_by_session[str(start['session_id'])].append(start)
    raw_by_session = defaultdict(list)
    for sample in raw_return_samples:
        raw_by_session[sample['session_id']].append(sample)

    funnel_definitions = (
        ('camera_enqueued', 'UPC camera frames enqueued'),
        ('upc_socket_sent', 'UPC stereo socket sent'),
        ('lab_published', 'LAB stereo delivered'),
        ('lab_pose', 'cuVSLAM tracking pose'),
        ('upc_socket_received', 'UPC pose socket received'),
        ('upc_published', 'UPC pose DDS published'),
        ('base_pose', 'Final base pose published'),
    )

    for session, session_starts in sorted(
            starts_by_session.items(),
            key=lambda item: min(v['wall_ns'] for v in item[1])):
        values = by_session.get(session, [])
        raw_values = raw_by_session.get(session, [])
        start_count = len(session_starts)
        stage_counts = Counter()
        for start in session_starts:
            stamp = start['source_stamp_ns']
            after = start['monotonic_ns']
            stage_counts['camera_enqueued'] += 1
            stage_counts['upc_socket_sent'] += need(
                upc_session, 'upc_stereo_socket_sent', 'stereo',
                session, stamp, after) is not None
            stage_counts['lab_published'] += need(
                lab_session, 'lab_stereo_published', 'stereo',
                session, stamp) is not None
            stage_counts['lab_pose'] += need(
                lab_session, 'lab_pose_enqueued', 'tracking_odom',
                session, stamp) is not None
            stage_counts['upc_socket_received'] += need(
                upc_session, 'upc_pose_socket_received', 'tracking_odom',
                session, stamp, after) is not None
            stage_counts['upc_published'] += (session, stamp) in raw_return_by_key
            stage_counts['base_pose'] += need_stamp(
                'upc_pose_adapter_published', 'odom', stamp, after) is not None
        funnel = []
        previous = None
        for key, label in funnel_definitions:
            count = stage_counts[key]
            funnel.append({
                'key': key, 'label': label, 'count': count,
                'fraction_of_started': count / start_count if start_count else None,
                'fraction_of_previous': count / previous if previous else None,
                'dropped_from_previous': (previous - count) if previous is not None else 0,
            })
            previous = count
        summary = {
            'session_id': session,
            'started': start_count, 'complete': len(values),
            'completion_fraction': len(values) / start_count if start_count else None,
            'samples_over_threshold': sum(value['over_threshold'] for value in values),
            'raw_returned': len(raw_values),
            'raw_return_samples_over_threshold': sum(
                value['over_threshold'] for value in raw_values),
            'raw_return_pose_ms': numeric_stats([
                value['latency_ms'] for value in raw_values]),
            'funnel': funnel,
        }
        summary.update({field: numeric_stats([
            value[field] for value in values
            if isinstance(value.get(field), (int, float))])
                        for field in fields})
        summary['socket_boundary_samples'] = sum(
            value['socket_boundaries_complete'] for value in values)
        # Individually positive one-way measurements are a necessary but not
        # sufficient NTP health check. Keep this explicit in the report.
        summary['one_way_wall_samples_nonnegative'] = sum(
            value['forward_one_way_wall_ms'] >= 0
            and value['return_one_way_wall_ms'] >= 0 for value in values)
        sessions.append(summary)

    # Ten-second delivery bins expose outages and rejection bursts that a
    # successful-samples-only scatter plot cannot show.
    delivery_bins = []
    if starts:
        first_wall_ns = min(start['wall_ns'] for start in starts)
        bin_width_ns = 10_000_000_000
        bins = defaultdict(lambda: {
            'started': 0, 'raw_returned': 0, 'raw_over_threshold': 0,
            'final': 0,
        })
        final_keys = {(sample['session_id'], sample['source_stamp_ns'])
                      for sample in samples}
        for start in starts:
            key = (str(start['session_id']), start['source_stamp_ns'])
            index = (start['wall_ns'] - first_wall_ns) // bin_width_ns
            bucket = bins[int(index)]
            bucket['started'] += 1
            raw = raw_return_by_key.get(key)
            if raw is not None:
                bucket['raw_returned'] += 1
                bucket['raw_over_threshold'] += raw['over_threshold']
            bucket['final'] += key in final_keys
        for index, bucket in sorted(bins.items()):
            started_count = bucket['started']
            bucket.update({
                'elapsed_s': index * 10.0,
                'raw_return_fraction': (bucket['raw_returned'] / started_count
                                        if started_count else None),
                'final_fraction': (bucket['final'] / started_count
                                   if started_count else None),
                'raw_over_threshold_fraction': (
                    bucket['raw_over_threshold'] / bucket['raw_returned']
                    if bucket['raw_returned'] else None),
            })
            delivery_bins.append(bucket)
    return {
        'started': len(starts), 'complete': len(samples),
        'completion_fraction': len(samples) / len(starts) if starts else None,
        'incomplete_reasons': dict(incomplete), 'sessions': sessions,
        'samples': samples, 'raw_return_samples': raw_return_samples,
        'delivery_bins': delivery_bins,
    }


def bridge_summary(records):
    states = [record.get('state') for record in records if isinstance(record.get('state'), dict)]
    if not states:
        return None
    reasons = Counter(str(state.get('reason', 'unknown')) for state in states)
    sessions = {state.get('session_id') for state in states if state.get('session_id')}
    transitions = []
    previous = None
    for state in states:
        current = (state.get('connected'), state.get('session_id'), state.get('tracking_ok'),
                   state.get('vo_state'), state.get('reason'))
        if current != previous:
            transitions.append({
                'connected': current[0], 'session_id': current[1],
                'tracking_ok': current[2], 'vo_state': current[3], 'reason': current[4],
            })
            previous = current

    def maximum_number(name):
        values = [state.get(name) for state in states
                  if isinstance(state.get(name), (int, float))]
        return max(values) if values else None

    mailboxes = {}
    for direction in ('tx_mailbox', 'rx_mailbox'):
        result = {}
        for metric in ('enqueued', 'drained', 'replaced', 'expired', 'dropped_oldest',
                       'discarded_on_clear'):
            by_kind = defaultdict(list)
            for state in states:
                values = state.get(direction, {}).get(metric, {})
                if isinstance(values, dict):
                    for kind, value in values.items():
                        if isinstance(value, int):
                            by_kind[kind].append(value)
            result[metric] = {kind: max(values) for kind, values in by_kind.items()}
        mailboxes[direction] = result
    tracking_ages = [state.get('tracking_age_sec') for state in states
                     if isinstance(state.get('tracking_age_sec'), (int, float))]
    return {
        'samples': len(states),
        'connected_false_samples': sum(state.get('connected') is False for state in states),
        'tracking_not_ok_samples': sum(state.get('tracking_ok') is False for state in states),
        'reasons': dict(reasons),
        'sessions': sorted(sessions),
        'connection_count': maximum_number('connection_count'),
        'dropped_stereo': maximum_number('dropped_stereo'),
        'sync_drops': maximum_number('sync_drops'),
        'imu_overflows': maximum_number('imu_overflows'),
        'max_tracking_age_sec': max(tracking_ages) if tracking_ages else None,
        'mailboxes': mailboxes,
        'transitions': transitions,
    }


def visual_status_summary(records):
    states = Counter(record.get('vo_state') for record in records
                     if isinstance(record.get('vo_state'), int))
    return {'samples': sum(states.values()),
            'vo_state_counts': {str(key): value for key, value in sorted(states.items())}}


def goal_status_summary(records):
    last = {}
    transitions = []
    for record in records:
        for status in record.get('goal_statuses', []):
            goal = status.get('goal_id')
            value = status.get('status')
            if goal and isinstance(value, int) and last.get(goal) != value:
                transitions.append({'observed_wall_ns': record.get('observed_wall_ns'),
                                    'observed_monotonic_ns': record.get('observed_monotonic_ns'),
                                    'goal_id': goal, 'status': value,
                                    'status_name': GOAL_STATUS_NAMES.get(value, str(value))})
                last[goal] = value
    final_names = Counter(GOAL_STATUS_NAMES.get(value, str(value)) for value in last.values())
    return {
        'goals': len(last),
        'final_status_counts': dict(Counter(str(value) for value in last.values())),
        'final_status_names': dict(final_names),
        'status_5_canceled_goals': sum(value == 5 for value in last.values()),
        'status_6_aborted_goals': sum(value == 6 for value in last.values()),
        'transitions': transitions,
    }


def relevant_logs(records):
    matched = []
    counts = Counter()
    for record in records:
        message = str(record.get('log_message', ''))
        lowered = message.lower()
        terms = [term for term in RELEVANT_LOG_TERMS if term in lowered]
        name = str(record.get('log_name', '')).lower()
        nav_warning = (int(record.get('log_level', 0)) >= 30 and any(
            term in name for term in (
                'nav2', 'controller_server', 'planner_server', 'bt_navigator',
                'costmap', 'localization_tf', 'nav2_gate', 'waypoint_ui')))
        if not terms and not nav_warning:
            continue
        if nav_warning and not terms:
            terms = ['nav2 warning/error']
        for term in terms:
            counts[term] += 1
        if len(matched) < 100:
            matched.append({
                'observed_wall_ns': record.get('observed_wall_ns'),
                'observed_monotonic_ns': record.get('observed_monotonic_ns'),
                'name': record.get('log_name'), 'level': record.get('log_level'),
                'message': message,
            })
    return {'counts': dict(counts), 'samples': matched, 'truncated': sum(counts.values()) > 100}


def _angle_delta(right, left):
    return math.atan2(math.sin(right - left), math.cos(right - left))


def localization_correction_summary(records, odom_records=(),
                                    jump_m=0.05, jump_rad=math.radians(5.0),
                                    goal_tolerance_m=0.01,
                                    goal_tolerance_rad=math.radians(1.0),
                                    stopped_linear_mps=0.02,
                                    stopped_angular_rps=0.02,
                                    velocity_match_sec=0.25):
    """Measure vslam_map->odom correction changes, not normal robot motion."""
    broadcasts = []
    for record in sorted(records, key=lambda item: item.get('observed_monotonic_ns', 0)):
        for transform in record.get('transforms', []):
            if (transform.get('parent_frame'), transform.get('child_frame')) != (
                    'vslam_map', 'odom'):
                continue
            translation = transform.get('translation')
            yaw_value = transform.get('yaw')
            observed = record.get('observed_monotonic_ns')
            if (not isinstance(translation, list) or len(translation) < 2
                    or not isinstance(yaw_value, (int, float))
                    or not isinstance(observed, int)
                    or not all(math.isfinite(float(value))
                               for value in (translation[0], translation[1], yaw_value))):
                continue
            broadcasts.append({
                'observed_monotonic_ns': observed,
                'observed_wall_ns': record.get('observed_wall_ns'),
                'session_id': str(record.get('bridge_session_id') or ''),
                'x': float(translation[0]), 'y': float(translation[1]),
                'yaw': float(yaw_value),
            })
    distinct = []
    for sample in broadcasts:
        if (distinct and sample['session_id'] == distinct[-1]['session_id']
                and all(abs(sample[name] - distinct[-1][name]) <= 1e-12
                        for name in ('x', 'y', 'yaw'))):
            continue
        distinct.append(sample)
    odom_samples = []
    for record in sorted(odom_records,
                         key=lambda item: item.get('observed_monotonic_ns', 0)):
        observed = record.get('observed_monotonic_ns')
        velocity = record.get('velocity')
        if (not isinstance(observed, int) or not isinstance(velocity, list)
                or len(velocity) < 3
                or not all(isinstance(value, (int, float)) and math.isfinite(value)
                           for value in velocity[:3])):
            continue
        odom_samples.append((observed, [float(value) for value in velocity[:3]]))
    odom_index = 0
    match_ns = int(velocity_match_sec * 1e9)
    for sample in distinct:
        while (odom_index + 1 < len(odom_samples)
               and odom_samples[odom_index + 1][0] <= sample['observed_monotonic_ns']):
            odom_index += 1
        candidates = odom_samples[odom_index:odom_index + 2] if odom_samples else []
        nearest = (min(candidates, key=lambda item: abs(
            item[0] - sample['observed_monotonic_ns'])) if candidates else None)
        sample['stationary'] = None
        if nearest is not None and abs(nearest[0] - sample['observed_monotonic_ns']) <= match_ns:
            vx, vy, wz = nearest[1]
            sample['stationary'] = (
                math.hypot(vx, vy) <= stopped_linear_mps
                and abs(wz) <= stopped_angular_rps)
            sample['wheel_velocity'] = nearest[1]
            sample['wheel_velocity_age_ms'] = (
                sample['observed_monotonic_ns'] - nearest[0]) / 1e6
    translation_steps = []
    yaw_steps = []
    stationary_translation_steps = []
    stationary_yaw_steps = []
    stationary_tolerance_exceedances = []
    jumps = []
    tolerance_exceedances = []
    segments = 0
    previous = None
    for sample in distinct:
        if previous is None or sample['session_id'] != previous['session_id']:
            segments += 1
            previous = sample
            continue
        translation = math.hypot(sample['x'] - previous['x'], sample['y'] - previous['y'])
        yaw_change = abs(_angle_delta(sample['yaw'], previous['yaw']))
        translation_steps.append(translation)
        yaw_steps.append(yaw_change)
        stationary_step = sample.get('stationary') is True and previous.get('stationary') is True
        if stationary_step:
            stationary_translation_steps.append(translation)
            stationary_yaw_steps.append(yaw_change)
            if translation > goal_tolerance_m or yaw_change > goal_tolerance_rad:
                stationary_tolerance_exceedances.append({
                    'observed_wall_ns': sample['observed_wall_ns'],
                    'session_id': sample['session_id'],
                    'translation_m': translation,
                    'yaw_rad': yaw_change,
                    'yaw_deg': math.degrees(yaw_change),
                    'wheel_velocity': sample.get('wheel_velocity'),
                    'wheel_velocity_age_ms': sample.get('wheel_velocity_age_ms'),
                })
        if translation > goal_tolerance_m or yaw_change > goal_tolerance_rad:
            tolerance_exceedances.append({
                'observed_wall_ns': sample['observed_wall_ns'],
                'session_id': sample['session_id'],
                'translation_m': translation,
                'yaw_rad': yaw_change,
                'yaw_deg': math.degrees(yaw_change),
            })
        if translation > jump_m or yaw_change > jump_rad:
            jumps.append({
                'observed_wall_ns': sample['observed_wall_ns'],
                'session_id': sample['session_id'],
                'translation_m': translation,
                'yaw_rad': yaw_change,
                'yaw_deg': math.degrees(yaw_change),
                'from': [previous['x'], previous['y'], previous['yaw']],
                'to': [sample['x'], sample['y'], sample['yaw']],
            })
        previous = sample
    def session_gaps(samples):
        return [right['observed_monotonic_ns'] - left['observed_monotonic_ns']
                for left, right in zip(samples, samples[1:])
                if right['session_id'] == left['session_id']
                and right['observed_monotonic_ns'] > left['observed_monotonic_ns']]

    broadcast_gaps = session_gaps(broadcasts)
    update_gaps = session_gaps(distinct)
    return {
        'broadcasts': len(broadcasts), 'distinct_updates': len(distinct),
        'segments': segments, 'jump_threshold_m': jump_m,
        'jump_threshold_rad': jump_rad,
        'goal_tolerance_m': goal_tolerance_m,
        'goal_tolerance_rad': goal_tolerance_rad,
        'stopped_linear_mps': stopped_linear_mps,
        'stopped_angular_rps': stopped_angular_rps,
        'velocity_match_sec': velocity_match_sec,
        'translation_step_m': numeric_stats(translation_steps),
        'yaw_step_rad': numeric_stats(yaw_steps),
        'stationary_translation_step_m': numeric_stats(stationary_translation_steps),
        'stationary_yaw_step_rad': numeric_stats(stationary_yaw_steps),
        'stationary_matched_steps': len(stationary_translation_steps),
        'stationary_goal_tolerance_exceedance_count': len(
            stationary_tolerance_exceedances),
        'stationary_goal_tolerance_exceedances': stationary_tolerance_exceedances[:100],
        'broadcast_gap_ms': ns_stats_ms(broadcast_gaps),
        'update_gap_ms': ns_stats_ms(update_gaps),
        'goal_tolerance_exceedance_count': len(tolerance_exceedances),
        'goal_tolerance_exceedances': tolerance_exceedances[:100],
        'jumps': jumps[:100], 'jump_count': len(jumps),
    }


def parameter_event_summary(records):
    events = []
    state = {}
    relevant_prefixes = ('/rby1/vslam/nav2/', '/rby1/vslam/localization_tf',
                         '/rby1/vslam/nav2_gate')
    for record in sorted(records, key=lambda item: item.get('observed_monotonic_ns', 0)):
        node = str(record.get('parameter_node') or '')
        if not node.startswith(relevant_prefixes):
            continue
        for operation, field in (('new', 'new_parameters'), ('changed', 'changed_parameters')):
            for parameter in record.get(field, []):
                name = str(parameter.get('name') or '')
                value = parameter.get('value')
                state[(node, name)] = value
                events.append({'observed_wall_ns': record.get('observed_wall_ns'),
                               'node': node, 'name': name,
                               'operation': operation, 'value': value})
        for name in record.get('deleted_parameters', []):
            state.pop((node, str(name)), None)
            events.append({'observed_wall_ns': record.get('observed_wall_ns'),
                           'node': node, 'name': str(name),
                           'operation': 'deleted', 'value': None})
    return {
        # Declarations normally arrive first. Keep the tail so an initialization
        # burst cannot hide the runtime changed/deleted events under diagnosis.
        'messages': len(records), 'events': events[-500:],
        'event_count': len(events), 'tracked_parameters': len(state),
        'operation_counts': dict(Counter(item['operation'] for item in events)),
        'truncated': len(events) > 500,
        'discarded_initial_events': max(0, len(events) - 500),
    }


def path_quality_summary(records, rosout_records, navigation_event_records=()):
    import re
    goal_pattern = re.compile(
        r'Navigate to (?P<name>.*?): x=(?P<x>[-+0-9.eE]+), '
        r'y=(?P<y>[-+0-9.eE]+), yaw=(?P<yaw>[-+0-9.eE]+) deg')
    goals_by_request = {}
    goal_order = []

    def finite_goal(state):
        goal = state.get('goal') if isinstance(state, dict) else None
        if not isinstance(goal, dict):
            return None
        values = (goal.get('x'), goal.get('y'), goal.get('yaw'))
        if not all(isinstance(value, (int, float)) and math.isfinite(value)
                   for value in values):
            return None
        return {
            'name': str(goal.get('name') or ''),
            'x': float(values[0]), 'y': float(values[1]), 'yaw': float(values[2]),
        }

    for record in sorted(navigation_event_records,
                         key=lambda item: item.get('observed_monotonic_ns', 0)):
        state = record.get('state')
        observed = record.get('observed_monotonic_ns')
        if not isinstance(state, dict) or not isinstance(observed, int):
            continue
        event = str(state.get('event') or '')
        request_id = str(state.get('request_id') or '')
        goal_values = finite_goal(state)
        if event == 'requested' and goal_values is not None:
            # New captures carry a request ID. A unique legacy ID lets us retain
            # the event for reporting without treating it as accepted.
            request_id = request_id or f'legacy-{observed}-{len(goal_order)}'
            entry = {
                **goal_values, 'request_id': request_id,
                'requested_ns': observed, 'accepted_ns': None,
                'terminal_ns': None, 'terminal_event': None,
                'goal_id': None, 'source': 'navigation_event',
            }
            goals_by_request[request_id] = entry
            goal_order.append(entry)
            continue
        entry = goals_by_request.get(request_id) if request_id else None
        if entry is None and not request_id:
            # Compatibility for the first schema revision, which emitted the
            # same goal but no request ID on accepted/terminal events.
            candidates = [item for item in goal_order
                          if item['terminal_ns'] is None and
                          (goal_values is None or (
                              item['name'] == goal_values['name']
                              and item['x'] == goal_values['x']
                              and item['y'] == goal_values['y']
                              and item['yaw'] == goal_values['yaw']))]
            entry = candidates[-1] if candidates else None
        if entry is None:
            continue
        if event == 'accepted':
            entry['accepted_ns'] = observed
            entry['goal_id'] = str(state.get('goal_id') or '') or None
        elif event in ('finished', 'rejected', 'send_error', 'result_error'):
            entry['terminal_ns'] = observed
            entry['terminal_event'] = event

    structured_goals = [item for item in goal_order
                        if isinstance(item.get('accepted_ns'), int)
                        and item.get('terminal_event') not in ('rejected', 'send_error')]
    fallback_goals = []
    for record in sorted(rosout_records, key=lambda item: item.get('observed_monotonic_ns', 0)):
        match = goal_pattern.search(str(record.get('log_message', '')))
        observed = record.get('observed_monotonic_ns')
        # Structured navigation_event is exact and preferred. The regex keeps
        # older captures analyzable, but do not duplicate the same request.
        if match and isinstance(observed, int) and not any(
                abs(item['requested_ns'] - observed) < 100_000_000
                for item in goal_order):
            fallback_goals.append({
                'requested_ns': observed, 'accepted_ns': observed,
                'terminal_ns': None, 'terminal_event': None,
                'request_id': None, 'goal_id': None, 'source': 'rosout',
                'name': match.group('name'), 'x': float(match.group('x')),
                'y': float(match.group('y')),
                'yaw': math.radians(float(match.group('yaw'))),
            })
    fallback_goals.sort(key=lambda item: item['accepted_ns'])
    for left, right in zip(fallback_goals, fallback_goals[1:]):
        left['terminal_ns'] = right['accepted_ns']

    samples = []
    sorted_records = sorted(records, key=lambda item: item.get('observed_monotonic_ns', 0))
    for record in sorted_records:
        observed = record.get('observed_monotonic_ns')
        if not isinstance(observed, int):
            continue
        sample = {
            'observed_wall_ns': record.get('observed_wall_ns'),
            'pose_count': record.get('pose_count'),
            'path_length_m': record.get('path_length_m'),
            'direct_distance_m': record.get('direct_distance_m'),
            'max_step_m': record.get('max_step_m'),
            'end_pose': record.get('end_pose'),
        }
        candidates = [goal for goal in structured_goals
                      if goal['accepted_ns'] <= observed
                      and (goal['terminal_ns'] is None or observed <= goal['terminal_ns'])]
        if not candidates:
            candidates = [goal for goal in fallback_goals
                          if goal['accepted_ns'] <= observed
                          and (goal['terminal_ns'] is None or observed <= goal['terminal_ns'])]
        goal = max(candidates, key=lambda item: item['accepted_ns']) if candidates else None
        end = record.get('end_pose')
        valid_end = (isinstance(end, list) and len(end) >= 3
                     and all(isinstance(value, (int, float)) and math.isfinite(value)
                             for value in end[:3]))
        if goal is not None and valid_end:
            sample.update(
                request_id=goal['request_id'], goal_id=goal['goal_id'],
                goal_source=goal['source'],
                goal_name=goal['name'], requested_goal=[goal['x'], goal['y'], goal['yaw']],
                endpoint_error_m=math.hypot(end[0] - goal['x'], end[1] - goal['y']),
                endpoint_yaw_error_rad=abs(_angle_delta(end[2], goal['yaw'])),
            )
        samples.append(sample)
    lengths = [item['path_length_m'] for item in samples
               if isinstance(item.get('path_length_m'), (int, float))]
    endpoint_errors = [item['endpoint_error_m'] for item in samples
                       if isinstance(item.get('endpoint_error_m'), (int, float))]
    endpoint_yaw_errors = [item['endpoint_yaw_error_rad'] for item in samples
                           if isinstance(item.get('endpoint_yaw_error_rad'), (int, float))]
    return {
        'plans': len(samples), 'requested_goals_seen': len(goal_order) + len(fallback_goals),
        'accepted_goals_seen': len(structured_goals) + len(fallback_goals),
        'matched_plans': len(endpoint_errors),
        'path_length_m': numeric_stats(lengths),
        'endpoint_error_m': numeric_stats(endpoint_errors),
        'endpoint_yaw_error_rad': numeric_stats(endpoint_yaw_errors),
        'empty_paths': sum(item.get('pose_count') == 0 for item in samples),
        'single_pose_paths': sum(item.get('pose_count') == 1 for item in samples),
        'samples': samples[:200], 'truncated': len(samples) > 200,
    }


def state_transition_summary(records):
    transitions = []
    previous = object()
    unhealthy = 0
    for record in sorted(records, key=lambda item: item.get('observed_monotonic_ns', 0)):
        state = record.get('state')
        if not isinstance(state, dict):
            continue
        if state.get('healthy') is False or state.get('state') == 'fault':
            unhealthy += 1
        signature = (
            state.get('healthy'), state.get('enabled'), state.get('state'),
            state.get('detail'), state.get('session_id'), state.get('tracking_ok'),
        )
        if signature == previous:
            continue
        transitions.append({
            'observed_wall_ns': record.get('observed_wall_ns'),
            'observed_monotonic_ns': record.get('observed_monotonic_ns'),
            'state': state,
        })
        previous = signature
    return {'samples': len(records), 'unhealthy_samples': unhealthy,
            'transitions': transitions[:200], 'truncated': len(transitions) > 200}


def abort_evidence(goal_summary, messages, child_summaries=None):
    result = []
    child_summaries = child_summaries or {}
    status_topics = (
        '/rby1/vslam/localization_status', '/rby1/vslam/navigation_status',
        '/rby1/vslam/bridge_status',
    )
    rosout = messages.get('/rosout', [])
    for transition in goal_summary.get('transitions', []):
        if transition.get('status') != 6:
            continue
        when = transition.get('observed_monotonic_ns')
        if not isinstance(when, int):
            continue
        same_goal_times = [item.get('observed_monotonic_ns')
                           for item in goal_summary.get('transitions', [])
                           if item.get('goal_id') == transition.get('goal_id')
                           and isinstance(item.get('observed_monotonic_ns'), int)
                           and item['observed_monotonic_ns'] <= when]
        window_start = min(same_goal_times) if same_goal_times else when - 5_000_000_000
        warning_logs = []
        diagnostic_logs = []
        for record in rosout:
            observed = record.get('observed_monotonic_ns')
            if (not isinstance(observed, int)
                    or not window_start <= observed <= when + 2_000_000_000):
                continue
            item = {
                'observed_monotonic_ns': observed,
                'offset_sec': (observed - when) / 1e9,
                'name': record.get('log_name'), 'level': record.get('log_level'),
                'message': record.get('log_message'),
            }
            if int(record.get('log_level', 0)) >= 30:
                warning_logs.append(item)
            elif any(term in str(record.get('log_message', '')).lower()
                     for term in RELEVANT_LOG_TERMS):
                diagnostic_logs.append(item)
        selected_logs = warning_logs[-40:] + diagnostic_logs[-15:]
        selected_logs.sort(key=lambda item: item['observed_monotonic_ns'])
        for item in selected_logs:
            item.pop('observed_monotonic_ns', None)
        child_terminals = {}
        for action, summary in child_summaries.items():
            selected = [item for item in summary.get('transitions', [])
                        if item.get('status') in (4, 5, 6)
                        and isinstance(item.get('observed_monotonic_ns'), int)
                        and window_start <= item['observed_monotonic_ns']
                        <= when + 2_000_000_000]
            if selected:
                child_terminals[action] = selected[-20:]
        latest_states = {}
        for topic in status_topics:
            candidates = [record for record in messages.get(topic, [])
                          if isinstance(record.get('observed_monotonic_ns'), int)
                          and record['observed_monotonic_ns'] <= when
                          and isinstance(record.get('state'), dict)]
            if candidates:
                record = max(candidates, key=lambda item: item['observed_monotonic_ns'])
                latest_states[topic] = {
                    'age_sec': (when - record['observed_monotonic_ns']) / 1e9,
                    'state': record['state'],
                }
        result.append({
            'goal_id': transition.get('goal_id'),
            'observed_wall_ns': transition.get('observed_wall_ns'),
            'observed_monotonic_ns': when,
            'goal_window_sec': (when - window_start) / 1e9,
            'nearby_logs': selected_logs, 'latest_states': latest_states,
            'warning_error_log_count': len(warning_logs),
            'diagnostic_log_count': len(diagnostic_logs),
            'logs_truncated': (len(warning_logs) > 40 or len(diagnostic_logs) > 15),
            'child_terminal_transitions': child_terminals,
        })
    return result


def pose_adapter_drop_summary(records):
    reasons = Counter()
    for record in records:
        state = record.get('state')
        if not isinstance(state, dict) or state.get('stage') != 'upc_pose_adapter_dropped':
            continue
        reasons[str(state.get('reason') or 'unknown')] += 1
    return {'samples': sum(reasons.values()), 'reasons': dict(reasons)}


def degraded(upstream, downstream, stale_ms):
    if not upstream or not downstream:
        return False
    up_rate, down_rate = upstream.get('observed_rate_hz'), downstream.get('observed_rate_hz')
    rate_loss = (isinstance(up_rate, (int, float)) and isinstance(down_rate, (int, float))
                 and up_rate > 0 and down_rate < up_rate * 0.8)
    up_gap = (upstream.get('observed_gap_ms') or {}).get('max', 0)
    down_gap = (downstream.get('observed_gap_ms') or {}).get('max', 0)
    new_gap = down_gap > stale_ms and down_gap > up_gap + max(50.0, up_gap * 0.5)
    return rate_loss or new_gap


def maximum_counter(bridge, direction, metric, kind):
    if not bridge:
        return 0
    return bridge.get('mailboxes', {}).get(direction, {}).get(metric, {}).get(kind, 0)


def build_report(captures, stale_ms):
    stale_ns = int(stale_ms * 1_000_000)
    by_role = {capture['role']: capture for capture in captures}
    topic_stats = {}
    bridge = {}
    visual_status = {}
    goals = {}
    child_goals = {}
    logs = {}
    adapter_drops = {}
    corrections = {}
    parameter_events = {}
    path_quality = {}
    status_timelines = {}
    aborts = {}
    for capture in captures:
        role = capture['role']
        topic_stats[role] = {
            topic: topic_summary(records, stale_ns)
            for topic, records in capture['messages'].items()
        }
        bridge[role] = bridge_summary(
            capture['messages'].get('/rby1/vslam/bridge_status', []))
        visual_status[role] = visual_status_summary(
            capture['messages'].get('/visual_slam/status', []))
        goals[role] = goal_status_summary(capture['messages'].get(
            '/rby1/vslam/nav2/navigate_to_pose/_action/status', []))
        child_goals[role] = {
            'compute_path_to_pose': goal_status_summary(capture['messages'].get(
                '/rby1/vslam/nav2/compute_path_to_pose/_action/status', [])),
            'follow_path': goal_status_summary(capture['messages'].get(
                '/rby1/vslam/nav2/follow_path/_action/status', [])),
        }
        logs[role] = relevant_logs(capture['messages'].get('/rosout', []))
        adapter_drops[role] = pose_adapter_drop_summary(
            capture['messages'].get('/rby1/vslam/timing', []))
        corrections[role] = localization_correction_summary(
            capture['messages'].get('/tf', []),
            capture['messages'].get('/rby1/odom', []))
        parameter_events[role] = parameter_event_summary(
            capture['messages'].get('/parameter_events', []))
        path_quality[role] = path_quality_summary(
            capture['messages'].get('/rby1/vslam/nav2/plan', []),
            capture['messages'].get('/rosout', []),
            capture['messages'].get('/rby1/vslam/navigation_event', []))
        status_timelines[role] = {
            name: state_transition_summary(capture['messages'].get(topic, []))
            for name, topic in (
                ('localization', '/rby1/vslam/localization_status'),
                ('navigation', '/rby1/vslam/navigation_status'),
                ('bridge', '/rby1/vslam/bridge_status'),
            )
        }
        aborts[role] = abort_evidence(
            goals[role], capture['messages'], child_goals[role])

    end_to_end = build_end_to_end(captures, stale_ms)

    stages = []

    def add_stage(label, left_role, left_topic, right_role, right_topic, cross_host=False):
        left = by_role.get(left_role)
        right = by_role.get(right_role)
        if not left or not right:
            return
        clock = 'observed_wall_ns' if cross_host else 'observed_monotonic_ns'
        result = match_latency(left['messages'].get(left_topic, []),
                               right['messages'].get(right_topic, []), clock, stale_ms)
        result.update(label=label, left_role=left_role, left_topic=left_topic,
                      right_role=right_role, right_topic=right_topic,
                      clock_basis='cross-host wall clock' if cross_host else 'same-host monotonic')
        stages.append(result)

    add_stage('UPC frame metadata -> LAB frame metadata', 'upc',
              '/d435/d435/infra1/camera_info', 'lab',
              '/d435/d435/infra1/camera_info', True)
    add_stage('LAB frame metadata -> cuVSLAM tracking pose', 'lab',
              '/d435/d435/infra1/camera_info', 'lab',
              '/rby1/vslam/camera_odometry')
    add_stage('LAB frame metadata -> cuVSLAM map pose', 'lab',
              '/d435/d435/infra1/camera_info', 'lab',
              '/rby1/vslam/camera_slam_odometry')
    add_stage('LAB tracking pose -> UPC returned pose', 'lab',
              '/rby1/vslam/camera_odometry', 'upc',
              '/rby1/vslam/camera_odometry', True)
    add_stage('LAB map pose -> UPC returned map pose', 'lab',
              '/rby1/vslam/camera_slam_odometry', 'upc',
              '/rby1/vslam/camera_slam_odometry', True)
    add_stage('UPC returned tracking pose -> PoseAdapter odom', 'upc',
              '/rby1/vslam/camera_odometry', 'upc', '/rby1/vslam/odom')
    add_stage('UPC returned map pose -> PoseAdapter slam_odom', 'upc',
              '/rby1/vslam/camera_slam_odometry', 'upc', '/rby1/vslam/slam_odom')
    add_stage('UPC frame metadata -> UPC returned tracking pose', 'upc',
              '/d435/d435/infra1/camera_info', 'upc',
              '/rby1/vslam/camera_odometry')

    findings = []

    for capture in captures:
        role = capture['role']
        missing = [topic for topic in EXPECTED_TOPICS.get(role, ())
                   if not capture['messages'].get(topic)]
        if missing:
            findings.append(f'{role.upper()} recorder saw no messages on: {", ".join(missing)}.')
        if capture.get('subscription_unavailable'):
            unavailable = ', '.join(item.get('topic', '?')
                                    for item in capture['subscription_unavailable'])
            findings.append(f'{role.upper()} could not create observers for: {unavailable}.')

    def stats(role, topic):
        return topic_stats.get(role, {}).get(topic)

    upc_image = stats('upc', '/d435/d435/infra1/camera_info')
    lab_image = stats('lab', '/d435/d435/infra1/camera_info')
    lab_pose = stats('lab', '/rby1/vslam/camera_odometry')
    upc_pose = stats('upc', '/rby1/vslam/camera_odometry')
    upc_adapted = stats('upc', '/rby1/vslam/odom')
    upc_map_pose = stats('upc', '/rby1/vslam/camera_slam_odometry')
    upc_adapted_map = stats('upc', '/rby1/vslam/slam_odom')

    if upc_image and upc_image['gaps_over_stale']:
        findings.append('UPC camera/driver input itself has gaps above the stale threshold.')
    if degraded(upc_image, lab_image, stale_ms):
        findings.append('UPC input is healthier than LAB image input: inspect stereo sync, UPC TX queue, TCP, and LAB receive scheduling.')
    if degraded(lab_image, lab_pose, stale_ms):
        findings.append('LAB images are healthier than tracking pose output: cuVSLAM tracking or LAB compute is the leading suspect.')
    if degraded(lab_pose, upc_pose, stale_ms):
        findings.append('LAB tracking pose is healthier than UPC returned pose: LAB TX queue, TCP return path, or UPC receive scheduling is suspect.')
    if degraded(upc_pose, upc_adapted, stale_ms) or degraded(upc_map_pose, upc_adapted_map, stale_ms):
        findings.append('Returned camera pose is healthier than PoseAdapter output: inspect UPC TF availability and PoseAdapter scheduling.')

    for role, state in bridge.items():
        if not state:
            continue
        if (state.get('sync_drops') or 0) > 0:
            findings.append(f'{role.upper()} stereo synchronizer dropped {state["sync_drops"]} queued samples.')
        if (state.get('imu_overflows') or 0) > 0:
            findings.append(f'{role.upper()} bridge had {state["imu_overflows"]} IMU queue overflows and session resets.')
        if (state.get('connection_count') or 0) > 1 or len(state.get('sessions', [])) > 1:
            findings.append(f'{role.upper()} bridge reconnected; analyze each session separately.')
        stereo_replaced = maximum_counter(state, 'tx_mailbox', 'replaced', 'stereo')
        if stereo_replaced:
            findings.append(f'{role.upper()} TX latest-only queue replaced {stereo_replaced} stereo packets before send.')
        for direction, label in (('tx_mailbox', 'TX'), ('rx_mailbox', 'RX')):
            pose_replaced = sum(maximum_counter(state, direction, 'replaced', kind)
                                for kind in ('tracking_odom', 'slam_odom'))
            if pose_replaced:
                findings.append(
                    f'{role.upper()} {label} latest-only queue replaced '
                    f'{pose_replaced} pose packets.')
            pose_dropped = sum(maximum_counter(state, direction, 'dropped_oldest', kind)
                               for kind in ('tracking_odom', 'slam_odom'))
            if pose_dropped:
                findings.append(
                    f'{role.upper()} {label} bounded pose FIFO dropped '
                    f'{pose_dropped} oldest packets at capacity.')
            pose_cleared = sum(maximum_counter(state, direction, 'discarded_on_clear', kind)
                               for kind in ('tracking_odom', 'slam_odom'))
            if pose_cleared:
                findings.append(
                    f'{role.upper()} {label} discarded {pose_cleared} queued pose '
                    'packets during session reset/shutdown.')

    lab_vo = visual_status.get('lab', {}).get('vo_state_counts', {})
    unhealthy_vo = sum(count for value, count in lab_vo.items() if value != '1')
    if unhealthy_vo:
        findings.append(f'LAB cuVSLAM reported non-tracking vo_state for {unhealthy_vo} status samples.')
    for role, log_summary in logs.items():
        failures = sum(count for term, count in log_summary['counts'].items()
                       if term in ('tracking is lost', 'failed to track',
                                   'failed to get slam pose'))
        if failures:
            findings.append(f'{role.upper()} ROS logs contain {failures} cuVSLAM tracking/SLAM failures.')
    canceled = goals.get('upc', {}).get('status_5_canceled_goals', 0)
    if canceled:
        findings.append(f'UPC recorded {canceled} NavigateToPose goals ending in status 5 (canceled).')
    aborted = goals.get('upc', {}).get('status_6_aborted_goals', 0)
    if aborted:
        findings.append(
            f'UPC recorded {aborted} NavigateToPose goals ending in status 6 (ABORTED); '
            'inspect the correlated Nav2 logs and child action states below.')
    upc_correction = corrections.get('upc', {})
    if upc_correction.get('goal_tolerance_exceedance_count'):
        findings.append(
            f'UPC vslam_map->odom changed by more than the 1 cm / 1 deg goal tolerance '
            f'{upc_correction["goal_tolerance_exceedance_count"]} times; inspect whether '
            'these updates occurred while the base was stationary or converging on the goal.')
    if upc_correction.get('stationary_goal_tolerance_exceedance_count'):
        findings.append(
            f'UPC vslam_map->odom exceeded 1 cm / 1 deg while wheel odometry classified '
            f'the base as stationary '
            f'{upc_correction["stationary_goal_tolerance_exceedance_count"]} times; '
            'this is directly large enough to disturb the requested goal tolerance.')
    if upc_correction.get('jump_count'):
        largest = (upc_correction.get('translation_step_m') or {}).get('max')
        largest_yaw = (upc_correction.get('yaw_step_rad') or {}).get('max')
        findings.append(
            f'UPC vslam_map->odom correction exceeded the diagnostic jump threshold '
            f'{upc_correction["jump_count"]} times '
            f'(max {largest:.4f} m, {math.degrees(largest_yaw):.2f} deg).')
    upc_paths = path_quality.get('upc', {})
    endpoint_max = (upc_paths.get('endpoint_error_m') or {}).get('max')
    if isinstance(endpoint_max, (int, float)) and endpoint_max > 0.01:
        findings.append(
            f'UPC global plan endpoint differs from the requested waypoint by as much as '
            f'{endpoint_max:.4f} m, above the 0.01 m goal tolerance.')
    endpoint_yaw_max = (upc_paths.get('endpoint_yaw_error_rad') or {}).get('max')
    if (isinstance(endpoint_yaw_max, (int, float))
            and endpoint_yaw_max > math.radians(1.0)):
        findings.append(
            f'UPC global plan endpoint yaw differs from the requested waypoint by as much as '
            f'{math.degrees(endpoint_yaw_max):.2f} deg, above the 1 deg goal tolerance.')
    if upc_paths.get('single_pose_paths'):
        findings.append(
            f'UPC planner emitted {upc_paths["single_pose_paths"]} single-pose global paths; '
            'check whether close start/goal poses quantized to the same costmap cell.')
    upc_adapter_drops = adapter_drops.get('upc', {})
    if upc_adapter_drops.get('samples'):
        findings.append(
            f'UPC PoseAdapter explicitly dropped {upc_adapter_drops["samples"]} poses: '
            f'{upc_adapter_drops["reasons"]}.')
    component_labels = {
        'upc_tx_queue_socket_ms': 'UPC TX queue/socket',
        'network_roundtrip_residual_ms': 'TCP roundtrip residual',
        'lab_input_delivery_ms': 'LAB receive/publish',
        'lab_vslam_ms': 'cuVSLAM',
        'lab_tx_queue_socket_ms': 'LAB TX queue/socket',
        'upc_return_delivery_ms': 'UPC receive/DDS',
        'pose_adapter_ms': 'PoseAdapter',
    }
    for session in end_to_end.get('sessions', []):
        completion = session.get('completion_fraction')
        if isinstance(completion, (int, float)) and completion < 0.9:
            findings.append(
                f'Session {session["session_id"][:12]} published final base poses for only '
                f'{session["complete"]}/{session["started"]} camera frames '
                f'({100 * completion:.1f}%); inspect the delivery funnel and queue replacements.')
        raw_over = session.get('raw_return_samples_over_threshold', 0)
        if raw_over:
            findings.append(
                f'Session {session["session_id"][:12]} returned {raw_over}/'
                f'{session.get("raw_returned", 0)} raw camera poses over {stale_ms:g} ms; '
                'PoseAdapter rejection hides these from successful end-to-end samples.')
        if not session['samples_over_threshold']:
            continue
        means = {label: (session.get(field) or {}).get('mean')
                 for field, label in component_labels.items()}
        means = {label: value for label, value in means.items()
                 if isinstance(value, (int, float))}
        leading = max(means, key=means.get) if means else 'unknown stage'
        findings.append(
            f'Session {session["session_id"][:12]} has '
            f'{session["samples_over_threshold"]}/{session["complete"]} complete samples '
            f'over {stale_ms:g} ms; largest mean component is {leading}.')
    if not findings:
        findings.append('No stage-specific failure was proven from this capture; check missing-topic warnings and capture both PCs together.')

    warnings = []
    if len(captures) == 2:
        warnings.append('Cross-host wall-clock latency includes UPC/LAB clock offset; verify environment.txt NTP data before treating it as one-way network latency.')
    for capture in captures:
        if capture['malformed_lines']:
            warnings.append(f'{capture["role"]}: ignored {capture["malformed_lines"]} malformed JSONL lines.')
        if capture['role'] == 'upc' and not parameter_events.get('upc', {}).get('messages'):
            warnings.append(
                'UPC did not observe /parameter_events; do not infer that Nav2 parameters '
                'were unchanged. Compare params_start/params_end snapshots.')
        if capture['role'] == 'upc' and not corrections.get('upc', {}).get('broadcasts'):
            warnings.append(
                'UPC did not observe vslam_map->odom on /tf; localization correction '
                'jump analysis is unavailable.')
        if capture['role'] == 'upc' and capture.get('start', {}).get('nav2_debug'):
            quality = path_quality.get('upc', {})
            if not quality.get('plans'):
                warnings.append(
                    'UPC Nav2 debug capture saw no /plan messages; either planning never '
                    'succeeded or the plan publisher/subscription was unavailable.')
            elif quality.get('accepted_goals_seen') and not quality.get('matched_plans'):
                warnings.append(
                    'UPC saw accepted navigation requests and plans but could not correlate '
                    'their accepted-to-terminal lifetimes; inspect navigation_event delivery.')
    if len(captures) == 2 and not end_to_end['samples']:
        warnings.append(
            'No complete internal timing samples were matched. Rebuild both bridge nodes, '
            'record /rby1/vslam/timing, and start the recorder before the trial.')
    for session in end_to_end.get('sessions', []):
        if session.get('socket_boundary_samples', 0) < session['complete']:
            warnings.append(
                f'Session {session["session_id"][:12]} is missing socket-worker boundaries '
                'for some samples; its TCP residual also includes unseparated queues.')
        network = session.get('network_roundtrip_residual_ms') or {}
        if isinstance(network.get('min'), (int, float)) and network['min'] < -1.0:
            warnings.append(
                f'Session {session["session_id"][:12]} has negative TCP residual samples; '
                'inspect exact-stamp pairing and event ordering before interpreting the stack.')
    return {
        'schema_version': 2,
        'stale_threshold_ms': stale_ms,
        'captures': [{key: value for key, value in capture.items()
                      if key not in ('messages',)} for capture in captures],
        'topic_stats': topic_stats,
        'pipeline_stages': stages,
        'end_to_end': end_to_end,
        'bridge': bridge,
        'visual_slam_status': visual_status,
        'navigate_to_pose': goals,
        'child_actions': child_goals,
        'relevant_logs': logs,
        'pose_adapter_drops': adapter_drops,
        'localization_correction': corrections,
        'parameter_events': parameter_events,
        'path_quality': path_quality,
        'status_timelines': status_timelines,
        'aborted_goal_evidence': aborts,
        'findings': findings,
        'warnings': warnings,
    }


def format_number(value, digits=2):
    return '-' if value is None else f'{value:.{digits}f}'


def render_text(report):
    lines = [
        'rby1_vslam timing diagnosis',
        f'stale threshold: {report["stale_threshold_ms"]:.1f} ms',
        '',
        'Likely bottlenecks / events',
    ]
    lines.extend(f'- {finding}' for finding in report['findings'])
    if report['warnings']:
        lines.extend(['', 'Warnings'])
        lines.extend(f'- {warning}' for warning in report['warnings'])
    lines.extend(['', 'Topic health',
                  'role  topic                                           count  obs_Hz  src_Hz  max_gap_ms  gaps>limit  age_p95_ms  age_max_ms'])
    for role, topics in sorted(report['topic_stats'].items()):
        for topic in SELECTED_TOPICS:
            values = topics.get(topic)
            if not values:
                continue
            gap = values.get('observed_gap_ms') or {}
            age = values.get('source_age_ms') or {}
            lines.append(
                f'{role:<5} {topic:<47} {values["messages"]:>6}  '
                f'{format_number(values.get("observed_rate_hz")):>6}  '
                f'{format_number(values.get("source_rate_hz")):>6}  '
                f'{format_number(gap.get("max")):>10}  '
                f'{values["gaps_over_stale"]:>10}  '
                f'{format_number(age.get("p95")):>10}  {format_number(age.get("max")):>10}')
    lines.extend(['', 'Matched pipeline stages',
                  'stage                                             matched  left%  right%  p50_ms  p95_ms  max_ms  >limit  clock'])
    for stage in report['pipeline_stages']:
        latency = stage.get('latency_ms') or {}
        left_fraction = stage.get('left_match_fraction')
        right_fraction = stage.get('right_match_fraction')
        lines.append(
            f'{stage["label"]:<49} {stage["matched"]:>7}  '
            f'{format_number(None if left_fraction is None else 100 * left_fraction, 1):>5}  '
            f'{format_number(None if right_fraction is None else 100 * right_fraction, 1):>6}  '
            f'{format_number(latency.get("p50")):>6}  {format_number(latency.get("p95")):>6}  '
            f'{format_number(latency.get("max")):>6}  {stage["samples_over_stale"]:>6}  '
            f'{stage["clock_basis"]}')
    end_to_end = report.get('end_to_end', {})
    lines.extend([
        '', 'Per-session internal timing',
        f'session       complete/started  raw_p95  raw_max  raw>{report["stale_threshold_ms"]:g}  '
        'total_p50  total_p95  total_max  '
        f'final>{report["stale_threshold_ms"]:g}  '
        'tcp_resid_mean  vslam_mean  adapter_mean',
    ])
    for session in end_to_end.get('sessions', []):
        total = session.get('total_ms') or {}
        raw = session.get('raw_return_pose_ms') or {}
        transport = session.get('network_roundtrip_residual_ms') or {}
        vslam = session.get('lab_vslam_ms') or {}
        adapter = session.get('pose_adapter_ms') or {}
        lines.append(
            f'{session["session_id"][:12]:<12}  '
            f'{session["complete"]:>7}/{session["started"]:<7}  '
            f'{format_number(raw.get("p95")):>7}  '
            f'{format_number(raw.get("max")):>7}  '
            f'{session.get("raw_return_samples_over_threshold", 0):>7}  '
            f'{format_number(total.get("p50")):>9}  '
            f'{format_number(total.get("p95")):>9}  '
            f'{format_number(total.get("max")):>9}  '
            f'{session["samples_over_threshold"]:>5}  '
            f'{format_number(transport.get("mean")):>14}  '
            f'{format_number(vslam.get("mean")):>10}  '
            f'{format_number(adapter.get("mean")):>12}')
    if not end_to_end.get('sessions'):
        lines.append('(no complete UPC+LAB internal timing samples)')
    for session in end_to_end.get('sessions', []):
        lines.extend(['', f'Delivery funnel {session["session_id"][:12]}'])
        for stage in session.get('funnel', []):
            fraction = stage.get('fraction_of_started')
            lines.append(
                f'- {stage["label"]:<31} {stage["count"]:>7}  '
                f'{format_number(None if fraction is None else 100 * fraction, 1):>5}% of input  '
                f'dropped since previous={stage["dropped_from_previous"]}')
    lines.extend(['', 'Bridge summaries'])
    for role, state in sorted(report['bridge'].items()):
        if not state:
            lines.append(f'- {role}: no bridge_status samples')
            continue
        lines.append(
            f'- {role}: samples={state["samples"]}, connected_false={state["connected_false_samples"]}, '
            f'tracking_not_ok={state["tracking_not_ok_samples"]}, sessions={len(state["sessions"])}, '
            f'dropped_stereo={state.get("dropped_stereo")}, sync_drops={state.get("sync_drops")}, '
            f'imu_overflows={state.get("imu_overflows")}, max_tracking_age_sec={state.get("max_tracking_age_sec")}')
        for direction, metrics in state.get('mailboxes', {}).items():
            lines.append(
                f'  {direction}: replaced={metrics.get("replaced", {})}, '
                f'dropped_oldest={metrics.get("dropped_oldest", {})}, '
                f'discarded_on_clear={metrics.get("discarded_on_clear", {})}, '
                f'expired={metrics.get("expired", {})}')
    lines.extend(['', 'Status'])
    for role, state in sorted(report['visual_slam_status'].items()):
        if state['samples']:
            lines.append(f'- {role} cuVSLAM vo_state counts: {state["vo_state_counts"]}')
    upc_goals = report['navigate_to_pose'].get('upc')
    if upc_goals:
        lines.append(f'- UPC NavigateToPose final statuses: {upc_goals["final_status_names"]}')
    for action, summary in report.get('child_actions', {}).get('upc', {}).items():
        if summary.get('goals'):
            lines.append(f'- UPC {action} final statuses: {summary["final_status_names"]}')
    correction = report.get('localization_correction', {}).get('upc', {})
    if correction.get('broadcasts'):
        translation = correction.get('translation_step_m') or {}
        yaw_steps = correction.get('yaw_step_rad') or {}
        stationary_translation = correction.get('stationary_translation_step_m') or {}
        stationary_yaw = correction.get('stationary_yaw_step_rad') or {}
        update_gaps = correction.get('update_gap_ms') or {}
        lines.extend([
            '', 'Localization correction (vslam_map -> odom)',
            f'- broadcasts={correction["broadcasts"]}, distinct_updates={correction["distinct_updates"]}, '
            f'segments={correction["segments"]}, '
            f'over_1cm_or_1deg={correction["goal_tolerance_exceedance_count"]}, '
            f'over_5cm_or_5deg={correction["jump_count"]}',
            f'- distinct update gap: p50={format_number(update_gaps.get("p50"), 2)} ms, '
            f'p95={format_number(update_gaps.get("p95"), 2)} ms, '
            f'p99={format_number(update_gaps.get("p99"), 2)} ms, '
            f'max={format_number(update_gaps.get("max"), 2)} ms',
            f'- translation step: p50={format_number(translation.get("p50"), 4)} m, '
            f'p95={format_number(translation.get("p95"), 4)} m, '
            f'p99={format_number(translation.get("p99"), 4)} m, '
            f'max={format_number(translation.get("max"), 4)} m',
            f'- yaw step: p50={format_number(None if yaw_steps.get("p50") is None else math.degrees(yaw_steps["p50"]), 3)} deg, '
            f'p95={format_number(None if yaw_steps.get("p95") is None else math.degrees(yaw_steps["p95"]), 3)} deg, '
            f'p99={format_number(None if yaw_steps.get("p99") is None else math.degrees(yaw_steps["p99"]), 3)} deg, '
            f'max={format_number(None if yaw_steps.get("max") is None else math.degrees(yaw_steps["max"]), 3)} deg',
            f'- stationary-matched steps={correction.get("stationary_matched_steps", 0)}, '
            f'over_1cm_or_1deg={correction.get("stationary_goal_tolerance_exceedance_count", 0)} '
            f'(stopped <= {correction.get("stopped_linear_mps", 0):.3f} m/s, '
            f'{correction.get("stopped_angular_rps", 0):.3f} rad/s)',
            f'- stationary translation step: '
            f'p95={format_number(stationary_translation.get("p95"), 4)} m, '
            f'p99={format_number(stationary_translation.get("p99"), 4)} m, '
            f'max={format_number(stationary_translation.get("max"), 4)} m',
            f'- stationary yaw step: '
            f'p95={format_number(None if stationary_yaw.get("p95") is None else math.degrees(stationary_yaw["p95"]), 3)} deg, '
            f'p99={format_number(None if stationary_yaw.get("p99") is None else math.degrees(stationary_yaw["p99"]), 3)} deg, '
            f'max={format_number(None if stationary_yaw.get("max") is None else math.degrees(stationary_yaw["max"]), 3)} deg',
        ])
        for step in correction.get('stationary_goal_tolerance_exceedances', [])[:10]:
            lines.append(
                f'  stationary over tolerance wall_ns={step.get("observed_wall_ns")}: '
                f'{step["translation_m"]:.4f} m, {step["yaw_deg"]:.2f} deg, '
                f'wheel={step.get("wheel_velocity")}')
        for step in correction.get('goal_tolerance_exceedances', [])[:10]:
            lines.append(
                f'  over tolerance wall_ns={step.get("observed_wall_ns")}: '
                f'{step["translation_m"]:.4f} m, {step["yaw_deg"]:.2f} deg')
        for jump in correction.get('jumps', [])[:10]:
            lines.append(
                f'  jump wall_ns={jump.get("observed_wall_ns")}: '
                f'{jump["translation_m"]:.4f} m, {jump["yaw_deg"]:.2f} deg')
    paths = report.get('path_quality', {}).get('upc', {})
    if paths.get('plans'):
        endpoint = paths.get('endpoint_error_m') or {}
        endpoint_yaw = paths.get('endpoint_yaw_error_rad') or {}
        lengths = paths.get('path_length_m') or {}
        lines.extend([
            '', 'Global path quality',
            f'- plans={paths["plans"]}, requested_goals_seen={paths["requested_goals_seen"]}, '
            f'accepted_goals_seen={paths.get("accepted_goals_seen", 0)}, '
            f'matched_plans={paths.get("matched_plans", 0)}, '
            f'empty={paths["empty_paths"]}, single_pose={paths["single_pose_paths"]}',
            f'- path length: p50={format_number(lengths.get("p50"), 3)} m, '
            f'max={format_number(lengths.get("max"), 3)} m',
            f'- requested goal -> plan endpoint error: '
            f'p50={format_number(endpoint.get("p50"), 4)} m, '
            f'p95={format_number(endpoint.get("p95"), 4)} m, '
            f'max={format_number(endpoint.get("max"), 4)} m',
            f'- requested yaw -> plan endpoint yaw error: '
            f'p50={format_number(None if endpoint_yaw.get("p50") is None else math.degrees(endpoint_yaw["p50"]), 3)} deg, '
            f'p95={format_number(None if endpoint_yaw.get("p95") is None else math.degrees(endpoint_yaw["p95"]), 3)} deg, '
            f'max={format_number(None if endpoint_yaw.get("max") is None else math.degrees(endpoint_yaw["max"]), 3)} deg',
        ])
    params = report.get('parameter_events', {}).get('upc', {})
    lines.extend([
        '', 'Nav2/localization parameter events',
        f'- parameter-event messages={params.get("messages", 0)}, '
        f'relevant events={params.get("event_count", 0)}, '
        f'tracked parameters={params.get("tracked_parameters", 0)}, '
        f'operations={params.get("operation_counts", {})}',
    ])
    for event in params.get('events', [])[-30:]:
        lines.append(
            f'  {event["operation"]}: {event["node"]}.{event["name"]}={event["value"]!r}')
    aborts = report.get('aborted_goal_evidence', {}).get('upc', [])
    if aborts:
        lines.extend(['', 'ABORTED (status 6) evidence windows'])
        for abort in aborts:
            lines.append(
                f'- goal={str(abort.get("goal_id") or "")[:12]} '
                f'wall_ns={abort.get("observed_wall_ns")} '
                f'window={abort.get("goal_window_sec", 0):.2f}s '
                f'warn/error={abort.get("warning_error_log_count", 0)} '
                f'diagnostic={abort.get("diagnostic_log_count", 0)}')
            for topic, state in abort.get('latest_states', {}).items():
                payload = state.get('state', {})
                lines.append(
                    f'  latest {topic} ({state.get("age_sec", 0):.3f}s old): '
                    f'{payload.get("state", payload.get("healthy"))} / '
                    f'{payload.get("detail", payload.get("reason", ""))}')
            for action, transitions in abort.get('child_terminal_transitions', {}).items():
                for transition in transitions:
                    child_when = transition.get('observed_monotonic_ns')
                    parent_when = abort.get('observed_monotonic_ns')
                    offset = ((child_when - parent_when) / 1e9
                              if isinstance(child_when, int) and isinstance(parent_when, int)
                              else None)
                    offset_text = '' if offset is None else f' ({offset:+.3f}s)'
                    lines.append(
                        f'  child {action}: goal={str(transition.get("goal_id") or "")[:12]} '
                        f'{transition.get("status_name", transition.get("status"))}'
                        f'{offset_text}')
            for log in abort.get('nearby_logs', []):
                lines.append(
                    f'  rosout {log["offset_sec"]:+.3f}s [{log.get("name")}] '
                    f'{log.get("message")}')
    for role, log_summary in sorted(report['relevant_logs'].items()):
        if log_summary['counts']:
            lines.append(f'- {role} relevant log counts: {log_summary["counts"]}')
    for role, summary in sorted(report.get('pose_adapter_drops', {}).items()):
        if summary.get('samples'):
            lines.append(f'- {role} PoseAdapter drops: {summary["reasons"]}')
    lines.extend([
        '',
        'Interpretation order',
        '1. UPC frame-metadata gap -> camera/driver.',
        '2. UPC frame metadata healthy but LAB bad -> stereo sync/TCP/input bridge.',
        '3. LAB frame metadata healthy but LAB pose bad -> cuVSLAM/GPU/compute/tracking.',
        '4. LAB pose healthy but UPC returned pose bad -> return bridge/TCP.',
        '5. UPC returned pose healthy but adapted pose bad -> TF/PoseAdapter.',
    ])
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='+', help='One capture dir, or UPC and LAB capture dirs')
    parser.add_argument('--output-dir', type=Path,
                        help='Destination (default: the single capture dir or current directory)')
    parser.add_argument('--stale-ms', type=float, default=500.0,
                        help='Gap threshold used in the report (default: 500)')
    args = parser.parse_args(argv)
    if not 1 <= len(args.captures) <= 2:
        parser.error('provide one capture directory, or one UPC and one LAB directory')
    if not math.isfinite(args.stale_ms) or args.stale_ms <= 0:
        parser.error('--stale-ms must be positive and finite')
    try:
        captures = [load_capture(path) for path in args.captures]
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    roles = [capture['role'] for capture in captures]
    if len(captures) == 2 and (set(roles) != {'upc', 'lab'}):
        parser.error(f'two captures must contain one upc and one lab role; got {roles}')
    report = build_report(captures, args.stale_ms)
    if args.output_dir:
        output_dir = args.output_dir.expanduser().resolve()
    elif len(captures) == 1:
        output_dir = Path(captures[0]['path'])
    else:
        output_dir = Path.cwd()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / 'timing_report.json'
    text_path = output_dir / 'timing_report.txt'
    samples_path = output_dir / 'timing_samples.csv'
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                         encoding='utf-8')
    text_path.write_text(render_text(report), encoding='utf-8')
    samples = report.get('end_to_end', {}).get('samples', [])
    if samples:
        with samples_path.open('w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
    else:
        samples_path.write_text('', encoding='utf-8')
    print(text_path)
    print(json_path)
    print(samples_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
