#!/usr/bin/env python3
"""Summarize one or two rby1_vslam timing captures without ROS dependencies."""

import argparse
from collections import Counter, defaultdict, deque
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
    '/rby1/vslam/localization_status',
    '/rby1/vslam/navigation_status',
)

RELEVANT_LOG_TERMS = (
    'tracking is lost', 'failed to track', 'failed to get slam pose',
    'stale', 'future clock', 'queue overflow', 'heartbeat timed out',
    'rejected tcp', 'tcp bridge', 'cancel',
)

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
        for metric in ('enqueued', 'drained', 'replaced', 'expired'):
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
                                    'goal_id': goal, 'status': value})
                last[goal] = value
    return {
        'goals': len(last),
        'final_status_counts': dict(Counter(str(value) for value in last.values())),
        'status_5_canceled_goals': sum(value == 5 for value in last.values()),
        'transitions': transitions,
    }


def relevant_logs(records):
    matched = []
    counts = Counter()
    for record in records:
        message = str(record.get('log_message', ''))
        lowered = message.lower()
        terms = [term for term in RELEVANT_LOG_TERMS if term in lowered]
        if not terms:
            continue
        for term in terms:
            counts[term] += 1
        if len(matched) < 100:
            matched.append({
                'observed_wall_ns': record.get('observed_wall_ns'),
                'name': record.get('log_name'), 'level': record.get('log_level'),
                'message': message,
            })
    return {'counts': dict(counts), 'samples': matched, 'truncated': sum(counts.values()) > 100}


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
    logs = {}
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
        logs[role] = relevant_logs(capture['messages'].get('/rosout', []))

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
        pose_replaced = sum(maximum_counter(state, 'tx_mailbox', 'replaced', kind)
                            for kind in ('tracking_odom', 'slam_odom'))
        if pose_replaced:
            findings.append(f'{role.upper()} TX latest-only queue replaced {pose_replaced} pose packets before send.')

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
    if not findings:
        findings.append('No stage-specific failure was proven from this capture; check missing-topic warnings and capture both PCs together.')

    warnings = []
    if len(captures) == 2:
        warnings.append('Cross-host wall-clock latency includes UPC/LAB clock offset; verify environment.txt NTP data before treating it as one-way network latency.')
    for capture in captures:
        if capture['malformed_lines']:
            warnings.append(f'{capture["role"]}: ignored {capture["malformed_lines"]} malformed JSONL lines.')
    return {
        'schema_version': 1,
        'stale_threshold_ms': stale_ms,
        'captures': [{key: value for key, value in capture.items()
                      if key not in ('messages',)} for capture in captures],
        'topic_stats': topic_stats,
        'pipeline_stages': stages,
        'bridge': bridge,
        'visual_slam_status': visual_status,
        'navigate_to_pose': goals,
        'relevant_logs': logs,
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
            lines.append(f'  {direction}: replaced={metrics.get("replaced", {})}, expired={metrics.get("expired", {})}')
    lines.extend(['', 'Status'])
    for role, state in sorted(report['visual_slam_status'].items()):
        if state['samples']:
            lines.append(f'- {role} cuVSLAM vo_state counts: {state["vo_state_counts"]}')
    upc_goals = report['navigate_to_pose'].get('upc')
    if upc_goals:
        lines.append(f'- UPC NavigateToPose final statuses: {upc_goals["final_status_counts"]}')
    for role, log_summary in sorted(report['relevant_logs'].items()):
        if log_summary['counts']:
            lines.append(f'- {role} relevant log counts: {log_summary["counts"]}')
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
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                         encoding='utf-8')
    text_path.write_text(render_text(report), encoding='utf-8')
    print(text_path)
    print(json_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
