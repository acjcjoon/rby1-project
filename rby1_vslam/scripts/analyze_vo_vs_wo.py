#!/usr/bin/env python3
"""Compare head-camera VO base pose with wheel odometry from one rosbag.

Run with the system numpy/matplotlib (user-site numpy 2.x breaks matplotlib):
  source /opt/ros/humble/setup.bash
  PYTHONNOUSERSITE=1 python3 rby1_vslam/scripts/analyze_vo_vs_wo.py BAG \
      --labels "직진,우90턴,우횡이동,좌90턴" [--out DIR]

Both trajectories are expressed relative to their own first pose, so the
comparison is of motion, not of absolute frames. Motion/stop segments come
from the wheel odometry twist.
"""

import argparse
import csv
import json
import math
import os

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

VO_TOPIC = '/rby1/vslam/odom'
WO_TOPIC = '/rby1/odom'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[VO_TOPIC, WO_TOPIC]))
    data = {VO_TOPIC: [], WO_TOPIC: []}
    while reader.has_next():
        topic, raw, recv_ns = reader.read_next()
        m = deserialize_message(raw, Odometry)
        p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        data[topic].append((stamp, recv_ns * 1e-9, p.x, p.y, yaw_of(q),
                            math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z)))
    return {k: np.array(sorted(v)) for k, v in data.items()}


def relative(arr):
    """Pose relative to the first sample: rotate into the start heading."""
    x0, y0, th0 = arr[0, 2], arr[0, 3], arr[0, 4]
    c, s = math.cos(-th0), math.sin(-th0)
    dx, dy = arr[:, 2] - x0, arr[:, 3] - y0
    yaw = np.unwrap(arr[:, 4]) - th0
    return np.column_stack([c * dx - s * dy, s * dx + c * dy, yaw])


def segments(t, moving, merge_gap=0.6, min_len=0.4):
    segs, start = [], None
    for i, m in enumerate(moving):
        if m and start is None:
            start = i
        elif not m and start is not None:
            segs.append([t[start], t[i - 1]])
            start = None
    if start is not None:
        segs.append([t[start], t[-1]])
    merged = []
    for s in segs:
        if merged and s[0] - merged[-1][1] < merge_gap:
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    return [s for s in merged if s[1] - s[0] >= min_len]


def pose_at(t, tq, rel):
    return np.column_stack([np.interp(tq, t, rel[:, k]) for k in range(3)])


def stop_metrics(t, rel, t0, t1, final_win=10.0, tol_mm=2.0, tol_deg=0.1):
    m = (t >= t0) & (t <= t1)
    if m.sum() < 5:
        return None
    tt, pp = t[m] - t0, rel[m]
    win = min(final_win, (t1 - t0) / 2.0)
    fin = tt >= tt[-1] - win
    ref = pp[fin].mean(axis=0)
    dpos = np.hypot(pp[:, 0] - ref[0], pp[:, 1] - ref[1]) * 1000.0
    dyaw = np.degrees(np.abs(pp[:, 2] - ref[2]))
    outside = np.where((dpos > tol_mm) | (dyaw > tol_deg))[0]
    conv = 0.0 if len(outside) == 0 else float(tt[min(outside[-1] + 1, len(tt) - 1)])
    resid = np.hypot(pp[fin, 0] - pp[fin, 0].mean(), pp[fin, 1] - pp[fin, 1].mean()) * 1000.0
    return {
        'max_dev_mm': float(dpos.max()), 'max_dev_yaw_deg': float(dyaw.max()),
        'p2p_mm': float(max(np.ptp(pp[:, 0]), np.ptp(pp[:, 1])) * 1000.0),
        'converge_s': conv, 'converged': conv < tt[-1] - 1e-6 or len(outside) == 0,
        'rms_final_mm': float(np.sqrt(np.mean(resid ** 2))),
        'rms_final_yaw_deg': float(np.degrees(np.std(pp[fin, 2]))),
        'final_window_s': float(win),
    }


def jump_metrics(t, rel, t0, t1):
    """Max frame-to-frame discontinuity: |Δp_k − Δp_{k−1}| (velocity-independent)."""
    m = (t >= t0) & (t <= t1)
    pp = rel[m]
    if len(pp) < 4:
        return None
    d = np.diff(pp, axis=0)
    dd = np.diff(d, axis=0)
    step = np.hypot(d[:, 0], d[:, 1]) * 1000.0
    return {'max_step_mm': float(step.max()),
            'max_jump_mm': float(np.hypot(dd[:, 0], dd[:, 1]).max() * 1000.0),
            'max_jump_yaw_deg': float(np.degrees(np.abs(dd[:, 2]).max()))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--labels', default='', help='comma separated motion names in order')
    ap.add_argument('--out', default=None)
    ap.add_argument('--lin', type=float, default=0.005, help='moving if |v| > lin (m/s)')
    ap.add_argument('--ang', type=float, default=0.01, help='moving if |w| > ang (rad/s)')
    a = ap.parse_args()
    out = a.out or os.path.join(a.bag.rstrip('/') + '_analysis')
    os.makedirs(out, exist_ok=True)

    d = read_bag(a.bag)
    vo, wo = d[VO_TOPIC], d[WO_TOPIC]
    t_start = max(vo[0, 0], wo[0, 0])
    vo = vo[vo[:, 0] >= t_start]
    wo = wo[wo[:, 0] >= t_start]
    # Start both at the same instant: VO sample after the common start.
    wo_rel_full = relative(wo)
    vo_rel = relative(vo)
    # Re-anchor wheel to the VO start time so both share t=0 origin.
    w0 = pose_at(wo[:, 0], np.array([vo[0, 0]]), wo_rel_full)[0]
    c, s = math.cos(-w0[2]), math.sin(-w0[2])
    dx, dy = wo_rel_full[:, 0] - w0[0], wo_rel_full[:, 1] - w0[1]
    wo_rel = np.column_stack([c * dx - s * dy, s * dx + c * dy, wo_rel_full[:, 2] - w0[2]])
    T0 = vo[0, 0]
    tv, tw = vo[:, 0] - T0, wo[:, 0] - T0

    stamp_skew = {k: float(np.median(v[:, 1] - v[:, 0])) for k, v in (('vo', vo), ('wo', wo))}
    moving = (wo[:, 5] > a.lin) | (wo[:, 6] > a.ang)
    motions = segments(tw, moving)
    labels = [x.strip() for x in a.labels.split(',') if x.strip()]
    names = [labels[i] if i < len(labels) else f'motion{i + 1}' for i in range(len(motions))]

    # Stops: before first motion, between motions, after the last.
    bounds = [0.0] + [v for s in motions for v in s] + [min(tv[-1], tw[-1])]
    stops = [(bounds[2 * i], bounds[2 * i + 1]) for i in range(len(motions) + 1)]

    rows = []
    for i, (s0, s1) in enumerate(motions):
        pv0, pv1 = pose_at(tv, np.array([s0, s1]), vo_rel)
        pw0, pw1 = pose_at(tw, np.array([s0, s1]), wo_rel)

        def disp(p0, p1):
            c0, s0_ = math.cos(-p0[2]), math.sin(-p0[2])
            dx_, dy_ = p1[0] - p0[0], p1[1] - p0[1]
            return (c0 * dx_ - s0_ * dy_, s0_ * dx_ + c0 * dy_, p1[2] - p0[2])
        dv, dw = disp(pv0, pv1), disp(pw0, pw1)
        jv, jw = jump_metrics(tv, vo_rel, s0, s1), jump_metrics(tw, wo_rel, s0, s1)
        rows.append({
            'kind': 'motion', 'name': names[i], 't0': s0, 't1': s1, 'dur_s': s1 - s0,
            'vo_dx_mm': dv[0] * 1e3, 'vo_dy_mm': dv[1] * 1e3, 'vo_dyaw_deg': math.degrees(dv[2]),
            'wo_dx_mm': dw[0] * 1e3, 'wo_dy_mm': dw[1] * 1e3, 'wo_dyaw_deg': math.degrees(dw[2]),
            'diff_pos_mm': math.hypot(dv[0] - dw[0], dv[1] - dw[1]) * 1e3,
            'diff_yaw_deg': math.degrees(dv[2] - dw[2]),
            **{'vo_' + k: v for k, v in (jv or {}).items()},
            **{'wo_' + k: v for k, v in (jw or {}).items()},
        })
    for i, (s0, s1) in enumerate(stops):
        if s1 - s0 < 2.0:
            continue
        name = ('정지(시작 전)' if i == 0 else f'정지(after {names[i - 1]})')
        sv, sw = stop_metrics(tv, vo_rel, s0, s1), stop_metrics(tw, wo_rel, s0, s1)
        rows.append({'kind': 'stop', 'name': name, 't0': s0, 't1': s1, 'dur_s': s1 - s0,
                     **{'vo_' + k: v for k, v in (sv or {}).items()},
                     **{'wo_' + k: v for k, v in (sw or {}).items()}})

    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(os.path.join(out, 'segments.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    with open(os.path.join(out, 'timeseries.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'vo_x', 'vo_y', 'vo_yaw', 'wo_x', 'wo_y', 'wo_yaw'])
        wi = pose_at(tw, tv, wo_rel)
        for k in range(len(tv)):
            w.writerow([f'{tv[k]:.4f}', *[f'{v:.5f}' for v in vo_rel[k]], *[f'{v:.5f}' for v in wi[k]]])
    summary = {'bag': a.bag, 'duration_s': float(tv[-1]), 'vo_hz': len(tv) / tv[-1],
               'wo_hz': len(tw) / tw[-1], 'recv_minus_stamp_s': stamp_skew,
               'motions': [[n, s[0], s[1]] for n, s in zip(names, motions)]}
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    plot(out, tv, vo_rel, tw, wo_rel, motions, names, stops)
    plot_sections(out, tv, vo_rel, tw, wo_rel, motions, names)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    for r in rows:
        print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})


def plot(out, tv, vo, tw, wo, motions, names, stops):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    cjk = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    if os.path.exists(cjk):
        font_manager.fontManager.addfont(cjk)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=cjk).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    cv, cw = '#d1495b', '#2e6f95'

    def shade(ax):
        for (s0, s1), n in zip(motions, names):
            ax.axvspan(s0, s1, color='0.88', zorder=0)

    # Fig 5-3: x, y, yaw vs time
    fig, axs = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True)
    for ax, k, lab, sc in zip(axs, range(3), ['x (m)', 'y (m)', 'yaw (°)'], [1, 1, 180 / math.pi]):
        shade(ax)
        ax.plot(tw, wo[:, k] * sc, color=cw, lw=1.6, label='휠 오도메트리 (WO)')
        ax.plot(tv, vo[:, k] * sc, color=cv, lw=0.9, label='시각 주행거리계 (VO)')
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
    for (s0, s1), n in zip(motions, names):
        axs[0].text((s0 + s1) / 2, 1.02, n, transform=axs[0].get_xaxis_transform(),
                    ha='center', va='bottom', fontsize=9)
    axs[0].legend(loc='upper left', fontsize=9)
    axs[-1].set_xlabel('시간 (s)  — 회색: 주행 구간, 흰색: 정지 구간')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_xy_yaw_time.png'), dpi=160)
    plt.close(fig)

    # XY path
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.plot(wo[:, 0], wo[:, 1], color=cw, lw=2, label='WO')
    ax.plot(vo[:, 0], vo[:, 1], color=cv, lw=1, label='VO')
    ax.plot(0, 0, 'ko', ms=5)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x (m, 시작 자세 기준)')
    ax.set_ylabel('y (m)')
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_xy_path.png'), dpi=160)
    plt.close(fig)

    # VO - WO difference
    wi = pose_at(tw, tv, wo)
    fig, axs = plt.subplots(2, 1, figsize=(11, 5), sharex=True)
    for ax in axs:
        shade(ax)
        ax.grid(alpha=0.3)
    axs[0].plot(tv, np.hypot(vo[:, 0] - wi[:, 0], vo[:, 1] - wi[:, 1]) * 1000, color='k', lw=0.8)
    axs[0].set_ylabel('|VO−WO| 위치 (mm)')
    axs[1].plot(tv, np.degrees(vo[:, 2] - wi[:, 2]), color='k', lw=0.8)
    axs[1].set_ylabel('VO−WO yaw (°)')
    axs[1].set_xlabel('시간 (s)')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_vo_minus_wo.png'), dpi=160)
    plt.close(fig)

    # Stop zoom: deviation from each stop's final mean
    valid = [(i, s) for i, s in enumerate(stops) if s[1] - s[0] >= 2.0]
    if valid:
        fig, axs = plt.subplots(1, len(valid), figsize=(3.4 * len(valid), 3.4), sharey=True,
                                squeeze=False)
        for ax, (i, (s0, s1)) in zip(axs[0], valid):
            for t, p, col, lab in ((tw, wo, cw, 'WO'), (tv, vo, cv, 'VO')):
                m = (t >= s0) & (t <= s1)
                tt, pp = t[m] - s0, p[m]
                win = min(10.0, (s1 - s0) / 2)
                ref = pp[tt >= tt[-1] - win].mean(axis=0)
                ax.plot(tt, np.hypot(pp[:, 0] - ref[0], pp[:, 1] - ref[1]) * 1000, color=col,
                        lw=1, label=lab)
            ax.axhline(2.0, color='0.5', ls='--', lw=0.8)
            ax.set_title('시작 전' if i == 0 else f'{names[i - 1]} 후', fontsize=10)
            ax.set_xlabel('정지 후 시간 (s)')
            ax.grid(alpha=0.3)
        axs[0][0].set_ylabel('최종 평균 대비 편차 (mm)')
        axs[0][0].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, 'fig_stop_settling.png'), dpi=160)
        plt.close(fig)


def local_track(t, rel, t_ref, t0, t1):
    """Pose in the body frame at t_ref, for samples in [t0, t1]."""
    p0 = pose_at(t, np.array([t_ref]), rel)[0]
    m = (t >= t0) & (t <= t1)
    c, s = math.cos(-p0[2]), math.sin(-p0[2])
    dx, dy = rel[m, 0] - p0[0], rel[m, 1] - p0[1]
    return t[m] - t_ref, np.column_stack([c * dx - s * dy, s * dx + c * dy, rel[m, 2] - p0[2]])


def motion_kind(t, rel, s0, s1):
    tt, p = local_track(t, rel, s0, s1, s1)
    d = p[-1]
    if abs(math.degrees(d[2])) > 20.0:
        return 'yaw'
    return 'x' if abs(d[0]) >= abs(d[1]) else 'y'


def plot_sections(out, tv, vo, tw, wo, motions, names, pre=2.0, post_max=25.0):
    """Report figure split by motion type: x translation, yaw, y translation."""
    import matplotlib.pyplot as plt
    cv, cw = '#d1495b', '#2e6f95'
    groups = {'x': [], 'yaw': [], 'y': []}
    for i, (s0, s1) in enumerate(motions):
        end = motions[i + 1][0] if i + 1 < len(motions) else min(tv[-1], tw[-1])
        groups[motion_kind(tw, wo, s0, s1)].append((names[i], s0, s1, min(end, s1 + post_max)))
    titles = {'x': '전후 이동 (x)', 'yaw': '제자리 회전 (yaw)', 'y': '좌우 이동 (y)'}
    order = [k for k in ('x', 'yaw', 'y') if groups[k]]
    styles = ['-', '--', ':', '-.']

    def draw(axcol, kind):
        a0, a1, a2 = axcol
        for j, (name, s0, s1, e) in enumerate(groups[kind]):
            ls = styles[j % len(styles)]
            tag = f' {name}' if len(groups[kind]) > 1 else ''
            t_v, pv = local_track(tv, vo, s0, s0 - pre, e)
            t_w, pw = local_track(tw, wo, s0, s0 - pre, e)
            pwi = np.column_stack([np.interp(t_v, t_w, pw[:, k]) for k in range(3)])
            if kind == 'yaw':
                sign = 1.0 if pw[-1, 2] >= 0 else -1.0
                main_v, main_w = np.degrees(sign * pv[:, 2]), np.degrees(sign * pw[:, 2])
                err = np.degrees(sign * (pv[:, 2] - pwi[:, 2]))
                leak_v = np.hypot(pv[:, 0], pv[:, 1]) * 1000
                leak_w = np.hypot(pw[:, 0], pw[:, 1]) * 1000
                ylab = ('회전각 |Δyaw| (°)', 'VO−WO yaw (°)', '회전 중 위치 이동 (mm)')
            else:
                k, o = (0, 1) if kind == 'x' else (1, 0)
                sign = 1.0 if pw[-1, k] >= 0 else -1.0
                main_v, main_w = sign * pv[:, k] * 1000, sign * pw[:, k] * 1000
                err = sign * (pv[:, k] - pwi[:, k]) * 1000
                leak_v, leak_w = pv[:, o] * 1000, pw[:, o] * 1000
                axis = 'x' if kind == 'x' else 'y'
                other = 'y' if kind == 'x' else 'x'
                ylab = (f'이동량 |Δ{axis}| (mm)', f'VO−WO Δ{axis} (mm)', f'수직 방향 Δ{other} (mm)')
            a0.plot(t_w, main_w, color=cw, lw=1.8, ls=ls, label='WO' + tag)
            a0.plot(t_v, main_v, color=cv, lw=1.0, ls=ls, label='VO' + tag)
            a1.plot(t_v, err, color='k', lw=0.9, ls=ls, label=name)
            a2.plot(t_w, leak_w, color=cw, lw=1.8, ls=ls)
            a2.plot(t_v, leak_v, color=cv, lw=1.0, ls=ls)
            for ax in axcol:
                ax.axvspan(0, s1 - s0, color='0.9', zorder=0)
        for ax, lab in zip(axcol, ylab):
            ax.set_ylabel(lab)
            ax.grid(alpha=0.3)
            ax.axhline(0, color='0.6', lw=0.6)
        a0.legend(fontsize=8, loc='lower right')
        if len(groups[kind]) > 1:
            a1.legend(fontsize=8)
        a2.set_xlabel('동작 시작 후 시간 (s)  — 회색: 주행')

    fig, axs = plt.subplots(3, len(order), figsize=(4.6 * len(order), 8.4), squeeze=False)
    for c, kind in enumerate(order):
        draw(axs[:, c], kind)
        axs[0, c].set_title(f'({chr(97 + c)}) {titles[kind]}', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_sections.png'), dpi=180)
    plt.close(fig)
    for kind in order:
        fig, axs = plt.subplots(3, 1, figsize=(5.2, 8.0), sharex=True)
        draw(axs, kind)
        axs[0].set_title(titles[kind], fontsize=11)
        fig.tight_layout()
        fig.savefig(os.path.join(out, f'fig_section_{kind}.png'), dpi=180)
        plt.close(fig)


if __name__ == '__main__':
    main()
