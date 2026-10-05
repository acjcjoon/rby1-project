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
    if len(args.captures) != 2:
        parser.error('provide UPC and LAB capture directories, or --report timing_report.json')
    try:
        captures = [load_capture(path) for path in args.captures]
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    roles = {capture['role'] for capture in captures}
    if roles != {'upc', 'lab'}:
        parser.error(f'captures must contain one UPC and one LAB role; got {sorted(roles)}')
    return build_report(captures, args.threshold_ms)


def plot_total_timeline(plt, samples, sessions, threshold_ms, output):
    first_wall = min(sample['upc_start_wall_ns'] for sample in samples)
    colors = plt.get_cmap('tab10')
    fig, ax = plt.subplots(figsize=(13, 6.5))
    for index, session in enumerate(sessions):
        session_id = session['session_id']
        values = [sample for sample in samples if sample['session_id'] == session_id]
        x = [(sample['upc_start_wall_ns'] - first_wall) / 1e9 for sample in values]
        y = [sample['total_ms'] for sample in values]
        ax.scatter(x, y, s=10, alpha=0.65, color=colors(index % 10),
                   label=f'S{index + 1} {session_id[:8]} (n={len(values)})')
    ax.axhline(threshold_ms, color='red', linestyle='--', linewidth=1.5,
               label=f'{threshold_ms:g} ms threshold')
    ax.set(title='UPC camera enqueue to base pose, by TCP session',
           xlabel='Elapsed UPC wall time (s)', ylabel='End-to-end latency (ms)')
    ax.grid(True, alpha=0.25)
    ax.legend(loc='best', fontsize=8, ncols=2)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_session_breakdown(plt, sessions, threshold_ms, output):
    labels = [f'S{index + 1}\n{session["session_id"][:8]}'
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
    ax.set(title='Mean end-to-end latency composition per TCP session',
           xlabel='Latency (ms)', ylabel='Session')
    ax.grid(True, axis='x', alpha=0.25)
    ax.legend(loc='best', fontsize=8, ncols=2)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


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
    sessions = end_to_end.get('sessions') or []
    if not samples or not sessions:
        parser.error('report has no complete internal timing samples')
    plt = _pyplot()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = (
        output_dir / 'total_latency_by_session.png',
        output_dir / 'session_stage_breakdown.png',
        output_dir / 'one_way_wall_clock_diagnostic.png',
    )
    plot_total_timeline(plt, samples, sessions, args.threshold_ms, outputs[0])
    plot_session_breakdown(plt, sessions, args.threshold_ms, outputs[1])
    plot_one_way_diagnostic(plt, samples, sessions, outputs[2])
    for path in outputs:
        print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
