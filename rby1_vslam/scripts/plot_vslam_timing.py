#!/usr/bin/env python3
"""Create offline per-session VSLAM latency plots from timing captures."""

import argparse
import json
import math
from pathlib import Path
import sys

from analyze_vslam_timing import build_report, load_capture


SEGMENTS = (
    ('upc_tx_queue_socket_ms', 'UPC TX queue/socket', '#4c78a8'),
    ('network_roundtrip_residual_ms', 'TCP roundtrip (residual)', '#9ecae9'),
    ('lab_input_delivery_ms', 'LAB receive + publish', '#72b7b2'),
    ('lab_vslam_ms', 'cuVSLAM', '#f58518'),
    ('lab_tx_queue_socket_ms', 'LAB TX queue/socket', '#ffbf79'),
    ('upc_return_delivery_ms', 'UPC receive + DDS', '#54a24b'),
    ('pose_adapter_ms', 'PoseAdapter', '#e45756'),
)


def _pyplot():
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            'matplotlib is required only for offline plotting; install it with '
            '"python -m pip install matplotlib"') from exc
    return plt


def load_report(args, parser):
    if args.report:
        if args.captures:
            parser.error('use either --report or two capture directories, not both')
        try:
            return json.loads(args.report.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as exc:
            parser.error(str(exc))
    if not 1 <= len(args.captures) <= 2:
        parser.error('provide one/two capture directories, or --report timing_report.json')
    try:
        captures = [load_capture(path) for path in args.captures]
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    roles = {capture['role'] for capture in captures}
    if len(captures) == 2 and roles != {'upc', 'lab'}:
        parser.error(f'captures must contain one UPC and one LAB role; got {sorted(roles)}')
    return build_report(captures, args.threshold_ms)


def plot_total_timeline(
        plt, samples, raw_samples, delivery_bins, sessions, threshold_ms, output):
    if not raw_samples:
        raw_samples = [{
            'session_id': sample['session_id'],
            'upc_start_wall_ns': sample['upc_start_wall_ns'],
            'latency_ms': sample.get('returned_pose_ms', sample['total_ms']),
            'over_threshold': sample.get('returned_pose_ms', sample['total_ms']) > threshold_ms,
        } for sample in samples]
    first_wall = min(sample['upc_start_wall_ns']
                     for sample in raw_samples + samples)
    fig, (ax, health) = plt.subplots(
        2, 1, figsize=(13, 8.5), sharex=True,
        gridspec_kw={'height_ratios': [2.15, 1.0]})
    normal_raw = [sample for sample in raw_samples if not sample['over_threshold']]
    stale_raw = [sample for sample in raw_samples if sample['over_threshold']]

    def elapsed(values):
        return [(sample['upc_start_wall_ns'] - first_wall) / 1e9 for sample in values]

    ax.scatter(elapsed(normal_raw), [sample['latency_ms'] for sample in normal_raw],
               s=8, alpha=0.28, color='#7f7f7f', label='Raw returned camera pose')
    if stale_raw:
        ax.scatter(elapsed(stale_raw), [sample['latency_ms'] for sample in stale_raw],
                   s=18, alpha=0.8, color='#d62728', marker='x', linewidths=0.8,
                   label=f'Raw returned pose > {threshold_ms:g} ms (n={len(stale_raw)})')
    ax.scatter(elapsed(samples), [sample['total_ms'] for sample in samples],
               s=8, alpha=0.42, color='#1f77b4', label='Final base pose (accepted)')
    ax.axhline(threshold_ms, color='red', linestyle='--', linewidth=1.4,
               label=f'{threshold_ms:g} ms threshold')
    maximum = max([sample['latency_ms'] for sample in raw_samples]
                  + [sample['total_ms'] for sample in samples])
    if maximum > threshold_ms * 2:
        ax.set_yscale('log')
        ax.set_ylabel('Latency (ms, log scale)')
    else:
        ax.set_ylabel('Latency (ms)')
    completion = sum(session.get('complete', 0) for session in sessions)
    started = sum(session.get('started', 0) for session in sessions)
    ax.set_title(
        'Raw return latency and accepted final poses '
        f'(final delivery {completion}/{started} = '
        f'{100 * completion / started:.1f}%)' if started else
        'Raw return latency and accepted final poses')
    ax.grid(True, alpha=0.25, which='both')
    ax.legend(loc='best', fontsize=8, ncols=2)

    if delivery_bins:
        x = [item['elapsed_s'] + 5.0 for item in delivery_bins]
        final = [100.0 * item['final_fraction'] for item in delivery_bins]
        returned = [100.0 * item['raw_return_fraction'] for item in delivery_bins]
        stale = [100.0 * (item['raw_over_threshold_fraction'] or 0.0)
                 for item in delivery_bins]
        health.bar(x, final, width=8.5, color='#4c78a8', alpha=0.55,
                   label='Final base poses / camera frames')
        health.plot(x, returned, color='#f58518', marker='o', markersize=3,
                    linewidth=1.3, label='Raw returned poses / camera frames')
        health.plot(x, stale, color='#d62728', marker='x', markersize=4,
                    linewidth=1.0, label='Raw returns over threshold / raw returns')
        health.set_ylim(0, 105)
        health.set_ylabel('Delivery / stale (%)')
        health.legend(loc='best', fontsize=8, ncols=3)
    else:
        health.text(0.5, 0.5, 'Delivery bins unavailable; regenerate timing_report.json',
                    transform=health.transAxes, ha='center', va='center')
        health.set_ylabel('Delivery (%)')
    health.set_xlabel('Elapsed UPC wall time (s)')
    health.set_title('10-second delivery health (drops and stale rejection are visible here)')
    health.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_session_breakdown(plt, sessions, threshold_ms, output):
    labels = [f'S{index + 1} {session["session_id"][:8]}\n'
              f'final {100 * (session.get("completion_fraction") or 0):.1f}%'
              for index, session in enumerate(sessions)]
    fig_height = max(4.5, 0.58 * len(labels) + 2.2)
    fig, ax = plt.subplots(figsize=(13, fig_height))
    left = [0.0] * len(sessions)
    totals = [((session.get('total_ms') or {}).get('mean') or 0.0)
              for session in sessions]
    for field, label, color in SEGMENTS:
        widths = [((session.get(field) or {}).get('mean') or 0.0)
                  for session in sessions]
        bars = ax.barh(labels, widths, left=left, label=label, color=color,
                       edgecolor='white', linewidth=0.35)
        for bar, width, total in zip(bars, widths, totals):
            if width > 0 and total > 0 and width / total >= 0.08:
                ax.text(bar.get_x() + bar.get_width() / 2,
                        bar.get_y() + bar.get_height() / 2,
                        f'{100 * width / total:.0f}%', ha='center', va='center',
                        fontsize=7, color='black')
        left = [position + width for position, width in zip(left, widths)]
    p95 = [((session.get('total_ms') or {}).get('p95')) for session in sessions]
    valid = [(value, labels[index]) for index, value in enumerate(p95)
             if isinstance(value, (int, float)) and math.isfinite(value)]
    if valid:
        ax.scatter([value for value, _ in valid], [label for _, label in valid],
                   marker='D', s=28, color='black', label='Total p95', zorder=5)
    ax.axvline(threshold_ms, color='red', linestyle='--', linewidth=1.5,
               label=f'{threshold_ms:g} ms threshold')
    ax.set(title='Successful final poses only: mean latency composition per TCP session',
           xlabel='Latency (ms)', ylabel='Session')
    ax.grid(True, axis='x', alpha=0.25)
    ax.legend(loc='best', fontsize=8, ncols=2)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_delivery_funnel(plt, sessions, output):
    rows = []
    for index, session in enumerate(sessions):
        for stage in session.get('funnel', []):
            rows.append((index, session, stage))
    if not rows:
        return False
    labels = [f'S{index + 1}  {stage["label"]}'
              for index, _, stage in rows]
    fractions = [100.0 * (stage.get('fraction_of_started') or 0.0)
                 for _, _, stage in rows]
    fig, ax = plt.subplots(figsize=(13, max(5.0, 0.48 * len(rows) + 1.8)))
    colors = [plt.get_cmap('RdYlGn')(fraction / 100.0) for fraction in fractions]
    bars = ax.barh(labels, fractions, color=colors, edgecolor='white')
    for bar, (_, _, stage), fraction in zip(bars, rows, fractions):
        dropped = stage.get('dropped_from_previous', 0)
        suffix = '' if not dropped else f', -{dropped:,} from previous'
        ax.text(min(fraction + 1.0, 98.0), bar.get_y() + bar.get_height() / 2,
                f'{stage["count"]:,} ({fraction:.1f}%{suffix})',
                va='center', fontsize=8)
    ax.set_xlim(0, 108)
    ax.invert_yaxis()
    ax.set(title='Pipeline delivery funnel (percentage of camera frames enqueued)',
           xlabel='Frames reaching stage (%)', ylabel='Stage')
    ax.grid(True, axis='x', alpha=0.25)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return True


def plot_latency_tails(plt, sessions, threshold_ms, output):
    fields = (
        ('raw_return_pose_ms', 'Raw returned camera pose'),
        ('total_ms', 'Final base pose (accepted)'),
        ('upc_tx_queue_socket_ms', 'UPC TX queue/socket'),
        ('network_roundtrip_residual_ms', 'TCP roundtrip residual'),
        ('lab_input_delivery_ms', 'LAB receive + publish'),
        ('lab_vslam_ms', 'cuVSLAM'),
        ('lab_tx_queue_socket_ms', 'LAB TX queue/socket'),
        ('upc_return_delivery_ms', 'UPC receive + DDS'),
        ('pose_adapter_ms', 'PoseAdapter'),
    )
    rows = []
    for index, session in enumerate(sessions):
        for field, label in fields:
            stats = session.get(field)
            if stats and all(isinstance(stats.get(key), (int, float))
                             for key in ('p50', 'p95', 'p99', 'max')):
                rows.append((index, label, stats))
    if not rows:
        return False
    y = list(range(len(rows)))
    labels = [f'S{index + 1}  {label}' for index, label, _ in rows]
    fig, ax = plt.subplots(figsize=(13, max(6.0, 0.48 * len(rows) + 2.0)))
    for position, (_, _, stats) in zip(y, rows):
        ax.hlines(position, stats['p50'], stats['p99'], color='#9e9e9e', linewidth=2)
    ax.scatter([stats['p50'] for _, _, stats in rows], y, marker='o', s=34,
               color='#4c78a8', label='p50', zorder=3)
    ax.scatter([stats['p95'] for _, _, stats in rows], y, marker='D', s=34,
               color='#f58518', label='p95', zorder=3)
    ax.scatter([stats['p99'] for _, _, stats in rows], y, marker='^', s=38,
               color='#e45756', label='p99', zorder=3)
    ax.scatter([stats['max'] for _, _, stats in rows], y, marker='x', s=42,
               color='black', label='max', zorder=3)
    ax.axvline(threshold_ms, color='red', linestyle='--', linewidth=1.4,
               label=f'{threshold_ms:g} ms threshold')
    ax.set_xscale('log')
    ax.set_yticks(y, labels)
    ax.invert_yaxis()
    ax.set(title='Latency tails by stage (raw return includes poses later rejected)',
           xlabel='Latency (ms, log scale)', ylabel='Stage')
    ax.grid(True, axis='x', alpha=0.25, which='both')
    ax.legend(loc='best', fontsize=8, ncols=5)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return True


def plot_one_way_diagnostic(plt, samples, sessions, output):
    first_wall = min(sample['upc_start_wall_ns'] for sample in samples)
    colors = plt.get_cmap('tab10')
    fig, axes = plt.subplots(2, 1, figsize=(13, 7.5), sharex=True)
    for index, session in enumerate(sessions):
        values = [sample for sample in samples
                  if sample['session_id'] == session['session_id']]
        x = [(sample['upc_start_wall_ns'] - first_wall) / 1e9 for sample in values]
        forward = [(sample.get('forward_socket_wall_ms')
                    if isinstance(sample.get('forward_socket_wall_ms'), (int, float))
                    else sample['forward_one_way_wall_ms']) for sample in values]
        returned = [(sample.get('return_socket_wall_ms')
                     if isinstance(sample.get('return_socket_wall_ms'), (int, float))
                     else sample['return_one_way_wall_ms']) for sample in values]
        axes[0].scatter(x, forward,
                        s=9, alpha=0.6, color=colors(index % 10))
        axes[1].scatter(x, returned,
                        s=9, alpha=0.6, color=colors(index % 10))
    for ax, title in zip(axes, ('UPC to LAB wall-clock estimate',
                                'LAB to UPC wall-clock estimate')):
        ax.axhline(0, color='red', linestyle='--', linewidth=1.0)
        ax.set_ylabel('Latency (ms)')
        ax.set_title(title)
        ax.grid(True, alpha=0.25)
    axes[1].set_xlabel('Elapsed UPC wall time (s)')
    fig.suptitle('One-way diagnostic (valid only after verifying UPC/LAB clock sync)')
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def _marked_motion(report):
    motion = report.get('motion_analysis') or {}
    if motion.get('upc', {}).get('available'):
        return 'upc', motion['upc']
    for role, result in motion.items():
        if result.get('available'):
            return role, result
    return None, None


def plot_pose_trajectory(plt, role, motion, output):
    preferred = (
        ('/rby1/vslam/odom', 'VSLAM tracking (base)'),
        ('/rby1/vslam/slam_odom', 'SLAM-corrected (base)'),
        ('/rby1/odom', 'Wheel odometry'),
    )
    available = [(topic, label, motion.get('pose_topics', {}).get(topic))
                 for topic, label in preferred
                 if motion.get('pose_topics', {}).get(topic, {}).get('trace')]
    if not available:
        return False
    colors = {
        'initial_stationary': '#4c78a8',
        'mapping_motion': '#f58518',
        'returned_stationary': '#54a24b',
    }
    fig, axes = plt.subplots(1, len(available),
                             figsize=(5.2 * len(available), 5.2), squeeze=False)
    for ax, (topic, label, result) in zip(axes[0], available):
        initial = result.get('phases', {}).get('initial_stationary')
        center = initial.get('center') if initial else None
        if center is None:
            first = result['trace'][0]
            center = [first['x'], first['y'], first['z'], first['yaw']]
        for phase, color in colors.items():
            trace = [sample for sample in result['trace'] if sample.get('phase') == phase]
            if not trace:
                continue
            ax.plot([sample['x'] - center[0] for sample in trace],
                    [sample['y'] - center[1] for sample in trace],
                    color=color, linewidth=1.2, alpha=0.85,
                    label=phase.replace('_', ' '))
        closure = result.get('return_error') or {}
        closure_text = ''
        if closure:
            closure_text = (f'\nreturn {closure["translation_m"] * 100:.1f} cm, '
                            f'{math.degrees(closure["yaw_rad"]):.2f} deg')
        ax.scatter([0], [0], marker='*', s=100, color='black', label='initial center')
        ax.set(title=label + closure_text, xlabel='x from initial center (m)',
               ylabel='y from initial center (m)')
        ax.axis('equal')
        ax.grid(True, alpha=0.25)
        ax.legend(loc='best', fontsize=7)
    fig.suptitle(f'{role.upper()} marked mapping-loop trajectories')
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return True


def plot_stationary_jitter(plt, role, motion, output):
    preferred = (
        ('/rby1/vslam/odom', 'VSLAM tracking'),
        ('/rby1/vslam/slam_odom', 'SLAM-corrected'),
        ('/rby1/odom', 'Wheel odometry'),
    )
    available = [(topic, label, motion.get('pose_topics', {}).get(topic))
                 for topic, label in preferred
                 if motion.get('pose_topics', {}).get(topic, {}).get('trace')]
    imu = motion.get('imu') or {}
    if not available and not imu.get('trace'):
        return False
    fig, axes = plt.subplots(3, 1, figsize=(13, 9.5), sharex=True)
    colors = plt.get_cmap('tab10')
    stationary_phases = ('initial_stationary', 'returned_stationary')
    for index, (_, label, result) in enumerate(available):
        x_values, radial_values, yaw_values = [], [], []
        for sample in result['trace']:
            phase = sample.get('phase')
            summary = result.get('phases', {}).get(phase, {})
            center = summary.get('center')
            if phase not in stationary_phases or not center:
                continue
            x_values.append(sample['elapsed_sec'])
            radial_values.append(100.0 * math.hypot(
                sample['x'] - center[0], sample['y'] - center[1]))
            delta = math.atan2(math.sin(sample['yaw'] - center[3]),
                               math.cos(sample['yaw'] - center[3]))
            yaw_values.append(math.degrees(delta))
        if x_values:
            axes[0].plot(x_values, radial_values, linewidth=0.8, alpha=0.8,
                         color=colors(index), label=label)
            axes[1].plot(x_values, yaw_values, linewidth=0.8, alpha=0.8,
                         color=colors(index), label=label)
    imu_x, gyro, accel = [], [], []
    for sample in imu.get('trace', []):
        phase = sample.get('phase')
        summary = imu.get('phases', {}).get(phase, {})
        center = summary.get('acceleration_center')
        if phase not in stationary_phases or not center:
            continue
        imu_x.append(sample['elapsed_sec'])
        gyro.append(sample['gyro_norm_rps'])
        accel.append(math.sqrt(sum(
            (sample['linear_acceleration'][axis] - center[axis]) ** 2
            for axis in range(3))))
    if imu_x:
        axes[2].plot(imu_x, gyro, linewidth=0.7, color='#e45756',
                     label='gyro norm (rad/s)')
        axes[2].plot(imu_x, accel, linewidth=0.7, color='#72b7b2',
                     label='accel residual (m/s²)')
    origin = motion.get('intervals', [{}])[0].get('start_monotonic_ns', 0)
    for axis in axes:
        for interval in motion.get('intervals', []):
            if interval['phase'] not in stationary_phases:
                continue
            start = (interval['start_monotonic_ns'] - origin) / 1e9
            end = (interval['end_monotonic_ns'] - origin) / 1e9
            axis.axvspan(start, end, color=('#4c78a8' if
                         interval['phase'] == 'initial_stationary' else '#54a24b'),
                         alpha=0.07)
        axis.grid(True, alpha=0.25)
        if axis.lines:
            axis.legend(loc='best', fontsize=8, ncols=3)
    axes[0].axhline(1.0, color='red', linestyle='--', linewidth=1.0,
                    label='1 cm reference')
    axes[1].axhline(1.0, color='red', linestyle='--', linewidth=1.0)
    axes[1].axhline(-1.0, color='red', linestyle='--', linewidth=1.0)
    axes[0].set_ylabel('Position jitter (cm)')
    axes[1].set_ylabel('Yaw from phase center (deg)')
    axes[2].set_ylabel('IMU magnitude')
    axes[2].set_xlabel('Elapsed time from initial-stationary marker (s)')
    fig.suptitle(f'{role.upper()} stationary pose/IMU jitter (mapping motion omitted)')
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='*')
    parser.add_argument('--report', type=Path,
                        help='Previously generated timing_report.json')
    parser.add_argument('--output-dir', type=Path, default=Path('timing_plots'))
    parser.add_argument('--threshold-ms', type=float, default=500.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.threshold_ms) or args.threshold_ms <= 0:
        parser.error('--threshold-ms must be positive and finite')
    report = load_report(args, parser)
    end_to_end = report.get('end_to_end') or {}
    samples = end_to_end.get('samples') or []
    raw_samples = end_to_end.get('raw_return_samples') or []
    delivery_bins = end_to_end.get('delivery_bins') or []
    sessions = end_to_end.get('sessions') or []
    motion_role, motion = _marked_motion(report)
    if (not samples or not sessions) and motion is None:
        parser.error('report has neither complete internal timing nor marked motion samples')
    plt = _pyplot()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = []
    if samples and sessions:
        timing_outputs = (
            output_dir / 'total_latency_by_session.png',
            output_dir / 'session_stage_breakdown.png',
            output_dir / 'pipeline_delivery_funnel.png',
            output_dir / 'latency_tail_by_stage.png',
            output_dir / 'one_way_wall_clock_diagnostic.png',
        )
        plot_total_timeline(
            plt, samples, raw_samples, delivery_bins, sessions,
            args.threshold_ms, timing_outputs[0])
        plot_session_breakdown(plt, sessions, args.threshold_ms, timing_outputs[1])
        plot_delivery_funnel(plt, sessions, timing_outputs[2])
        plot_latency_tails(plt, sessions, args.threshold_ms, timing_outputs[3])
        plot_one_way_diagnostic(plt, samples, sessions, timing_outputs[4])
        outputs.extend(timing_outputs)
    if motion is not None:
        trajectory = output_dir / 'mapping_pose_trajectory.png'
        jitter = output_dir / 'stationary_pose_imu_jitter.png'
        if plot_pose_trajectory(plt, motion_role, motion, trajectory):
            outputs.append(trajectory)
        if plot_stationary_jitter(plt, motion_role, motion, jitter):
            outputs.append(jitter)
    for path in outputs:
        print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
