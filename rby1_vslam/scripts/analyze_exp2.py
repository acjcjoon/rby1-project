#!/usr/bin/env python3
"""Experiment 2: VO settling after a stop, two walls alternated, plus IR scene analysis.

  source /opt/ros/humble/setup.bash
  PYTHONNOUSERSITE=1 python3 rby1_vslam/scripts/analyze_exp2.py BAG IR_DIR [--out DIR]
      [--window 11.5] [--fps-label 30]

Trials = stops (>= window) that follow a straight translation. Each trial is
assigned to a wall by base heading (two clusters). The wall with fewer IR
corners is labelled "poor" unless --poor-heading-deg is given.
All metrics use the same window after the stop so trials are comparable:
  drift_mm      |final ref − pose at stop| (final ref = mean of last --final s)
  max_dev_mm    max distance from the final ref inside the window
  settle_s      time after which the pose stays within ±tol mm of the final ref
  path_mm       path length travelled by the estimate inside the window
  yaw_drift_deg |final yaw − yaw at stop|
"""

import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_vo_vs_wo import (VO_TOPIC, WO_TOPIC, pose_at, read_bag, relative,  # noqa: E402
                              segments)

METRICS = [('drift_mm', '정지 후 표류량 (mm)'), ('max_dev_mm', '최대 변동 폭 (mm)'),
           ('settle_s', '수렴 시간 (s)'), ('path_mm', '정지 중 누적 이동 경로 (mm)'),
           ('yaw_drift_deg', 'yaw 표류 (°)')]


def stop_window_metrics(t, rel, t0, win, final, tol):
    m = (t >= t0) & (t <= t0 + win)
    tt, p = t[m] - t0, rel[m]
    p_stop = pose_at(t, np.array([t0]), rel)[0]
    ref = p[tt >= win - final].mean(axis=0)
    dev = np.hypot(p[:, 0] - ref[0], p[:, 1] - ref[1]) * 1000
    out = np.where(dev > tol)[0]
    settle = 0.0 if len(out) == 0 else float(tt[min(out[-1] + 1, len(tt) - 1)])
    moved = np.where(np.any(np.diff(p, axis=0) != 0, axis=1))[0]
    freeze = float(tt[moved[-1] + 1]) if len(moved) else 0.0
    return {
        'freeze_s': freeze,
        'drift_mm': float(math.hypot(ref[0] - p_stop[0], ref[1] - p_stop[1]) * 1000),
        'max_dev_mm': float(dev.max()),
        'settle_s': settle,
        # Resample to a fixed 10 Hz so path length does not grow with the sensor rate.
        'path_mm': float(np.sum(np.hypot(*np.diff(pose_at(
            t, t0 + np.arange(0.0, win + 1e-9, 0.1), rel)[:, :2], axis=0).T)) * 1000),
        'yaw_drift_deg': float(abs(math.degrees(ref[2] - p_stop[2]))),
    }, tt, dev


def ir_frames(ir_dir):
    files = sorted(glob.glob(os.path.join(ir_dir, '*.png')))
    return np.array([int(os.path.basename(f)[:-4]) * 1e-9 for f in files]), files


def ir_stats(path, grid=(8, 6)):
    import cv2
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    corners = cv2.goodFeaturesToTrack(img, maxCorners=1000, qualityLevel=0.01, minDistance=8)
    pts = np.zeros((0, 2)) if corners is None else corners.reshape(-1, 2)
    h, w = img.shape
    cells = set((int(x * grid[0] / w), int(y * grid[1] / h)) for x, y in pts)
    return {
        'mean': float(img.mean()), 'std': float(img.std()),
        'sat_pct': float((img >= 250).mean() * 100), 'dark_pct': float((img <= 20).mean() * 100),
        'corners': int(len(pts)), 'coverage_pct': 100.0 * len(cells) / (grid[0] * grid[1]),
        'lap_var': float(cv2.Laplacian(img, cv2.CV_64F).var()),
    }, img, pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('ir_dir')
    ap.add_argument('--out', default=None)
    ap.add_argument('--window', type=float, default=11.5)
    ap.add_argument('--final', type=float, default=3.0)
    ap.add_argument('--tol', type=float, default=2.0)
    ap.add_argument('--fps-label', default='')
    ap.add_argument('--include-backward', action='store_true',
                    help='also count stops after backward moves (protocol: forward only)')
    ap.add_argument('--poor-heading-deg', type=float, default=None,
                    help='relative heading (deg, WO frame) of the poor wall; default: fewer corners')
    a = ap.parse_args()
    out = a.out or a.bag.rstrip('/') + '_analysis'
    os.makedirs(out, exist_ok=True)

    d = read_bag(a.bag)
    vo, wo = d[VO_TOPIC], d[WO_TOPIC]
    T0 = max(vo[0, 0], wo[0, 0])
    vo, wo = vo[vo[:, 0] >= T0], wo[wo[:, 0] >= T0]
    vr, wr = relative(vo), relative(wo)
    tv, tw = vo[:, 0] - T0, wo[:, 0] - T0
    motions = segments(tw, (wo[:, 5] > 0.005) | (wo[:, 6] > 0.01))
    end = min(tv[-1], tw[-1])

    trials = []
    for i, (s0, s1) in enumerate(motions):
        nxt = motions[i + 1][0] if i + 1 < len(motions) else end
        w0, w1 = pose_at(tw, np.array([s0, s1]), wr)
        dyaw = math.degrees(w1[2] - w0[2])
        dist = math.hypot(w1[0] - w0[0], w1[1] - w0[1])
        if abs(dyaw) > 10 or dist < 0.1 or nxt - s1 < a.window:
            continue
        c, s = math.cos(-w0[2]), math.sin(-w0[2])
        fwd = c * (w1[0] - w0[0]) - s * (w1[1] - w0[1])
        v0, v1 = pose_at(tv, np.array([s0, s1]), vr)
        cv_, sv_ = math.cos(-v0[2]), math.sin(-v0[2])
        dv = (cv_ * (v1[0] - v0[0]) - sv_ * (v1[1] - v0[1]),
              sv_ * (v1[0] - v0[0]) + cv_ * (v1[1] - v0[1]))
        dw = (fwd, s * (w1[0] - w0[0]) + c * (w1[1] - w0[1]))
        err = math.hypot(dv[0] - dw[0], dv[1] - dw[1])
        trials.append({'motion': i + 1, 't_stop': s1, 'stop_len_s': nxt - s1,
                       'move_mm': fwd * 1000, 'heading_deg': math.degrees(w1[2]),
                       'move_err_mm': err * 1000, 'move_err_pct': err / max(abs(fwd), 1e-6) * 100,
                       'move_lat_err_mm': abs(dv[1] - dw[1]) * 1000,
                       'included': fwd > 0 or a.include_backward})

    # Two heading clusters.
    hd = np.array([t['heading_deg'] for t in trials])
    centers = [hd.min(), hd.max()]
    for _ in range(10):
        lab = np.array([0 if abs(h - centers[0]) <= abs(h - centers[1]) else 1 for h in hd])
        centers = [hd[lab == k].mean() for k in (0, 1)]
    for t, k in zip(trials, lab):
        t['heading_group'] = int(k)

    ir_t, ir_files = ir_frames(a.ir_dir)
    ir_t = ir_t - T0
    ir_cache = {}

    def stats_at(idx):
        if idx not in ir_cache:
            ir_cache[idx] = ir_stats(ir_files[idx])
        return ir_cache[idx]

    for t in trials:
        vm, t['_tt_vo'], t['_dev_vo'] = stop_window_metrics(tv, vr, t['t_stop'], a.window,
                                                            a.final, a.tol)
        wm, _, _ = stop_window_metrics(tw, wr, t['t_stop'], a.window, a.final, a.tol)
        t.update({'vo_' + k: v for k, v in vm.items()})
        t.update({'wo_' + k: v for k, v in wm.items()})
        idx = np.where((ir_t >= t['t_stop']) & (ir_t <= t['t_stop'] + a.window))[0]
        st = [stats_at(j)[0] for j in idx]
        t['ir_n'] = len(st)
        for k in ('mean', 'sat_pct', 'dark_pct', 'corners', 'coverage_pct', 'lap_var'):
            t['ir_' + k] = float(np.mean([x[k] for x in st])) if st else float('nan')
        t['ir_mean_cv_pct'] = (float(np.std([x['mean'] for x in st]) / np.mean([x['mean'] for x in st]) * 100)
                               if st else float('nan'))
        t['_ir_idx'] = idx

    if a.poor_heading_deg is None:
        corners = [np.mean([t['ir_corners'] for t in trials
                            if t['heading_group'] == k and t['included']]) for k in (0, 1)]
        poor = int(np.argmin(corners))
    else:
        poor = int(np.argmin([abs(c - a.poor_heading_deg) for c in centers]))
    for t in trials:
        t['wall'] = 'poor' if t['heading_group'] == poor else 'rich'
    extra = [t for t in trials if not t['included']]
    trials = [t for t in trials if t['included']]

    wall_name = {'rich': '특징점 충분·조명 안정', 'poor': '특징점 부족·조명 불안정'}
    summary = {'bag': a.bag, 'ir_dir': a.ir_dir, 'fps': a.fps_label, 'window_s': a.window,
               'final_ref_s': a.final, 'tol_mm': a.tol,
               'heading_centers_deg': {('poor' if k == poor else 'rich'): centers[k] for k in (0, 1)},
               'walls': {}}
    for wall in ('rich', 'poor'):
        ts = [t for t in trials if t['wall'] == wall and t['included']]
        row = {'n': len(ts)}
        for k, _ in METRICS:
            for src in ('vo', 'wo'):
                v = np.array([t[f'{src}_{k}'] for t in ts])
                row[f'{src}_{k}'] = [float(v.mean()), float(v.std(ddof=1)) if len(v) > 1 else 0.0]
        for k in ('move_mm', 'move_err_mm', 'move_err_pct', 'move_lat_err_mm'):
            v = np.array([t[k] for t in ts])
            row[k] = [float(v.mean()), float(v.std(ddof=1)) if len(v) > 1 else 0.0]
        for k in ('ir_mean', 'ir_sat_pct', 'ir_dark_pct', 'ir_corners', 'ir_coverage_pct',
                  'ir_lap_var', 'ir_mean_cv_pct'):
            v = np.array([t[k] for t in ts])
            row[k] = [float(np.nanmean(v)), float(np.nanstd(v, ddof=1)) if len(v) > 1 else 0.0]
        summary['walls'][wall] = row

    summary['excluded_stops'] = [{k: v for k, v in t.items() if not k.startswith('_')}
                                 for t in extra]
    with open(os.path.join(out, 'trials.csv'), 'w', newline='') as f:
        keys = [k for k in trials[0] if not k.startswith('_')]
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for t in trials + extra:
            w.writerow({k: (round(t[k], 3) if isinstance(t[k], float) else t[k]) for k in keys})
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    plots(out, trials, summary, wall_name, a, ir_t, ir_files, stats_at, tv, vr, tw, wr, motions)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    for t in trials:
        print({k: (round(v, 1) if isinstance(v, float) else v) for k, v in t.items()
               if not k.startswith('_') and not k.startswith('wo_')})


def plots(out, trials, summary, wall_name, a, ir_t, ir_files, stats_at, tv, vr, tw, wr, motions):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    cjk = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    if os.path.exists(cjk):
        font_manager.fontManager.addfont(cjk)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=cjk).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    col = {'rich': '#2e6f95', 'poor': '#d1495b'}
    fps = f' ({a.fps_label} fps)' if a.fps_label else ''

    # 1. Settling traces per wall
    fig, axs = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, wall in zip(axs, ('rich', 'poor')):
        for n, t in enumerate([t for t in trials if t['wall'] == wall]):
            ax.plot(t['_tt_vo'], t['_dev_vo'], color=col[wall], lw=1.1, alpha=0.55 + 0.09 * n,
                    label=f"시행 {n + 1}")
        ax.axhline(a.tol, color='0.5', ls='--', lw=0.8)
        ax.set_title(wall_name[wall] + fps, fontsize=11)
        ax.set_xlabel('정지 후 시간 (s)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axs[0].set_ylabel(f'VO 최종값 대비 편차 (mm)\n최종값 = 마지막 {a.final:g}초 평균')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_exp2_settling.png'), dpi=170)
    plt.close(fig)

    # 2. Metrics: dot + mean ± sd per wall (VO), WO as reference
    panels = [('move_err_pct', '주행 중 VO 오차\n(이동거리 대비 %)', None)] + \
        [('vo_' + k, lab, 'wo_' + k) for k, lab in METRICS if k != 'yaw_drift_deg']
    fig, axs = plt.subplots(1, len(panels), figsize=(2.75 * len(panels), 3.7))
    for ax, (k, lab, wk) in zip(axs, panels):
        for x, wall in enumerate(('rich', 'poor')):
            v = np.array([t[k] for t in trials if t['wall'] == wall])
            ax.scatter(np.full(len(v), x) + np.linspace(-0.08, 0.08, len(v)), v, s=16,
                       color=col[wall], alpha=0.7, zorder=3)
            ax.errorbar(x + 0.25, v.mean(), yerr=v.std(ddof=1) if len(v) > 1 else 0, fmt='o',
                        color='k', ms=5, capsize=4, zorder=4)
            if wk:
                wv = np.array([t[wk] for t in trials if t['wall'] == wall])
                ax.scatter([x + 0.4], [wv.mean()], marker='x', color='0.4', s=22, zorder=4)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['충분', '부족'], fontsize=9)
        ax.set_xlim(-0.4, 1.6)
        ax.set_title(lab, fontsize=9.5)
        ax.grid(alpha=0.3, axis='y')
        ax.set_ylim(bottom=0)
    fig.suptitle(f'벽별 VO 지표{fps} · 정지 지표는 정지 후 {a.window:g}초 창 · 점: 시행, '
                 '검은 점·막대: 평균 ± 표준편차, ×: WO 평균', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_exp2_metrics.png'), dpi=170)
    plt.close(fig)

    # 3. IR scene comparison: representative frame per wall (median corners) + histogram
    fig = plt.figure(figsize=(11, 7.4))
    gs = fig.add_gridspec(2, 2, height_ratios=[3, 1.4])
    for c, wall in enumerate(('rich', 'poor')):
        idx = np.concatenate([t['_ir_idx'] for t in trials if t['wall'] == wall])
        if not len(idx):
            continue
        cs = [stats_at(j)[0]['corners'] for j in idx]
        j = idx[np.argsort(cs)[len(cs) // 2]]
        st, img, pts = stats_at(j)
        ax = fig.add_subplot(gs[0, c])
        ax.imshow(img, cmap='gray', vmin=0, vmax=255)
        ax.scatter(pts[:, 0], pts[:, 1], s=7, c='#ffcc00', edgecolors='none')
        h, w = img.shape
        for gx in range(1, 8):
            ax.axvline(gx * w / 8, color='w', lw=0.4, alpha=0.4)
        for gy in range(1, 6):
            ax.axhline(gy * h / 6, color='w', lw=0.4, alpha=0.4)
        sat = np.ma.masked_where(img < 250, img)
        ax.imshow(sat, cmap='autumn', alpha=0.6, vmin=0, vmax=255)
        s = summary['walls'][wall]
        ax.set_title(f"({'ab'[c]}) {wall_name[wall]}{fps}\n"
                     f"코너 {s['ir_corners'][0]:.0f}개, 격자 점유 {s['ir_coverage_pct'][0]:.0f}%, "
                     f"포화 {s['ir_sat_pct'][0]:.1f}%, 평균 밝기 {s['ir_mean'][0]:.0f}", fontsize=10)
        ax.axis('off')
        axh = fig.add_subplot(gs[1, c])
        axh.hist(img.ravel(), bins=64, range=(0, 255), color=col[wall], alpha=0.85)
        axh.set_yscale('log')
        axh.set_xlabel('IR 밝기 (0–255)')
        axh.set_ylabel('픽셀 수 (log)')
        axh.grid(alpha=0.3)
    fig.text(0.5, 0.005, '노란 점: 검출 코너(goodFeaturesToTrack) · 주황: 포화 픽셀(≥250) · 흰 격자: 8×6 분포 판정 칸',
             ha='center', fontsize=9)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(os.path.join(out, 'fig_exp2_ir_scene.png'), dpi=170)
    plt.close(fig)

    # 4. IR over time: brightness & corners, stops shaded by wall
    sel = np.arange(0, len(ir_files), max(1, len(ir_files) // 400))
    st = [stats_at(j)[0] for j in sel]
    fig, axs = plt.subplots(3, 1, figsize=(11, 6.6), sharex=True)
    for ax in axs:
        for s0, s1 in motions:
            ax.axvspan(s0, s1, color='0.9', zorder=0)
        for t in trials:
            ax.axvspan(t['t_stop'], t['t_stop'] + a.window, color=col[t['wall']], alpha=0.12, zorder=0)
        ax.grid(alpha=0.3)
    axs[0].plot(ir_t[sel], [x['mean'] for x in st], color='k', lw=0.9)
    axs[0].set_ylabel('IR 평균 밝기')
    axs[1].plot(ir_t[sel], [x['corners'] for x in st], color='k', lw=0.9)
    axs[1].set_ylabel('코너 수')
    wi = pose_at(tw, tv, wr)
    axs[2].plot(tv, np.hypot(vr[:, 0] - wi[:, 0], vr[:, 1] - wi[:, 1]) * 1000, color='k', lw=0.8)
    axs[2].set_ylabel('|VO−WO| (mm)')
    axs[2].set_xlabel('시간 (s)  — 회색: 주행, 파랑: 충분·안정 벽 정지, 빨강: 부족·불안정 벽 정지')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_exp2_ir_timeline.png'), dpi=170)
    plt.close(fig)


if __name__ == '__main__':
    main()
