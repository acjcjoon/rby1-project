#!/usr/bin/env python3
"""Station round-trip experiment (table 6): compare conditions from the experiment CSVs.

  PYTHONNOUSERSITE=1 python3 rby1_vslam/scripts/analyze_exp3.py RUN.csv [RUN.csv ...] --out DIR

All vectors are in the target-station frame (x + ahead, y + left, yaw + CCW):
  real   = gt_*            true base pose relative to the registered station (AprilTag)
  est    = est_err_*       the robot's own estimate at measurement time (after correction)
  est0   = loc_arrived     the estimate at Nav2 arrival (before any correction)
  loc_err      = |est − real|   how wrong the robot's belief is when it starts the next leg
  loc_err_pre  = |est0 − real|  belief error before the station correction (VSLAM run)
"""

import argparse
import csv
import json
import math
import os

import numpy as np

COND_LABEL = {'wheel': '휠 오도메트리 단독', 'vslam': '휠 + VSLAM 보정 (방안 2)'}
COND_COLOR = {'wheel': '#2e6f95', 'vslam': '#d1495b'}


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def station_frame(px, py, pyaw, tx, ty, tyaw):
    dx, dy = px - tx, py - ty
    c, s = math.cos(-tyaw), math.sin(-tyaw)
    return (c * dx - s * dy) * 1000, (s * dx + c * dy) * 1000, \
        (math.degrees(pyaw - tyaw) + 180) % 360 - 180


def load(path):
    rows = list(csv.DictReader(open(path)))
    out = []
    for r in rows:
        tx, ty, tyaw = f(r['target_x']), f(r['target_y']), math.radians(f(r['target_yaw_deg']))
        d = {'leg': int(r['leg']), 'round': int(r['round']), 'to': r['to'], 'cond': r['condition'],
             'real': np.array([f(r['gt_x_mm']), f(r['gt_y_mm']), f(r['gt_yaw_deg'])]),
             'est': np.array([f(r['est_err_x_mm']), f(r['est_err_y_mm']), f(r['est_err_yaw_deg'])]),
             'nav_s': f(r['nav_s']), 'wait_s': f(r['correction_wait_s']),
             'corr_mm': f(r['correction_mm']), 'residual_px': f(r['tag_residual_px']),
             'success': r['success'] == 'True', 'gt_source': r['gt_source']}
        if r['loc_arrived_x']:
            d['est0'] = np.array(station_frame(f(r['loc_arrived_x']), f(r['loc_arrived_y']),
                                               math.radians(f(r['loc_arrived_yaw_deg'])), tx, ty, tyaw))
        else:
            d['est0'] = np.full(3, np.nan)
        d['real_mm'] = math.hypot(*d['real'][:2])
        d['est_mm'] = math.hypot(*d['est'][:2])
        d['loc_err_mm'] = math.hypot(*(d['est'][:2] - d['real'][:2]))
        d['loc_err_deg'] = d['est'][2] - d['real'][2]
        d['loc_err_pre_mm'] = math.hypot(*(d['est0'][:2] - d['real'][:2]))
        d['corr_vec'] = d['est'][:2] - d['est0'][:2]
        d['needed_vec'] = d['real'][:2] - d['est0'][:2]     # ideal correction
        out.append(d)
    return out


def ms(v):
    v = np.asarray([x for x in v if np.isfinite(x)])
    if not len(v):
        return float('nan'), float('nan')
    return float(v.mean()), float(v.std(ddof=1)) if len(v) > 1 else 0.0


def summarize(rows, pos_tol=20.0, yaw_tol=2.0):
    first, last = min(r['round'] for r in rows), max(r['round'] for r in rows)
    s = {
        'legs': len(rows),
        'success_pct': 100.0 * np.mean([r['real_mm'] <= pos_tol and abs(r['real'][2]) <= yaw_tol
                                        for r in rows]),
        'real_mm': ms([r['real_mm'] for r in rows]),
        'real_yaw_deg': ms([abs(r['real'][2]) for r in rows]),
        'real_mm_round1': ms([r['real_mm'] for r in rows if r['round'] == first]),
        'real_mm_round_last': ms([r['real_mm'] for r in rows if r['round'] == last]),
        'est_mm': ms([r['est_mm'] for r in rows]),
        'loc_err_mm': ms([r['loc_err_mm'] for r in rows]),
        'loc_err_yaw_deg': ms([abs(r['loc_err_deg']) for r in rows]),
        'loc_err_pre_mm': ms([r['loc_err_pre_mm'] for r in rows]),
        'corr_mm': ms([r['corr_mm'] for r in rows]),
        'wait_s': ms([r['wait_s'] for r in rows]),
        'nav_s': ms([r['nav_s'] for r in rows]),
        'tag_residual_px_max': float(np.nanmax([r['residual_px'] for r in rows])),
        'last_round': last,
    }
    return s


def plots(out, runs):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    cjk = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    if os.path.exists(cjk):
        font_manager.fontManager.addfont(cjk)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=cjk).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    marker = {}

    def mk(name):
        return marker.setdefault(name, 'os^D'[len(marker) % 4])

    # 1. Real arrival error + yaw per leg
    fig, axs = plt.subplots(2, 1, figsize=(10, 6.4), sharex=True)
    for cond, rows in runs.items():
        legs = [r['leg'] for r in rows]
        axs[0].plot(legs, [r['real_mm'] for r in rows], '-', color=COND_COLOR[cond], lw=1.2,
                    label=COND_LABEL[cond])
        axs[1].plot(legs, [r['real'][2] for r in rows], '-', color=COND_COLOR[cond], lw=1.2)
        for r in rows:
            axs[0].plot(r['leg'], r['real_mm'], mk(r['to']), color=COND_COLOR[cond], ms=5)
            axs[1].plot(r['leg'], r['real'][2], mk(r['to']), color=COND_COLOR[cond], ms=5)
    axs[0].axhline(20, color='0.4', ls='--', lw=0.9)
    axs[0].text(0.6, 22, '허용 2 cm', fontsize=8, color='0.3')
    for v in (2, -2):
        axs[1].axhline(v, color='0.4', ls='--', lw=0.9)
    axs[0].set_ylabel('실제 도착 위치 오차 (mm)')
    axs[1].set_ylabel('실제 도착 방향 오차 (°)')
    axs[1].set_xlabel('구간 (1회 왕복 = 2구간) · 마커: 도착 스테이션 ' +
                      ', '.join(f'{k}={v}' for k, v in marker.items()))
    axs[0].legend(fontsize=9)
    for ax in axs:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_exp3_real_error.png'), dpi=170)
    plt.close(fig)

    # 2. Belief vs truth: estimated error, real error, localization error
    fig, axs = plt.subplots(1, len(runs), figsize=(5.4 * len(runs), 4.2), sharey=True, squeeze=False)
    for ax, (cond, rows) in zip(axs[0], runs.items()):
        legs = [r['leg'] for r in rows]
        ax.plot(legs, [r['real_mm'] for r in rows], 'o-', color='k', ms=4, lw=1, label='실제 오차 (태그)')
        ax.plot(legs, [r['est_mm'] for r in rows], 's-', color=COND_COLOR[cond], ms=4, lw=1,
                label='추정 오차 (로봇의 믿음)')
        ax.plot(legs, [r['loc_err_mm'] for r in rows], '^--', color='#8a5cc2', ms=4, lw=1,
                label='위치추정 오차 |추정−실제|')
        if cond == 'vslam':
            ax.plot(legs, [r['loc_err_pre_mm'] for r in rows], 'v:', color='0.5', ms=4, lw=1,
                    label='보정 전 위치추정 오차')
        ax.axhline(20, color='0.4', ls='--', lw=0.8)
        ax.set_title(COND_LABEL[cond], fontsize=11)
        ax.set_xlabel('구간')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper left')
    axs[0][0].set_ylabel('mm')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_exp3_belief_vs_truth.png'), dpi=170)
    plt.close(fig)

    # 3. Arrival scatter in the station frame
    fig, axs = plt.subplots(1, len(runs), figsize=(5.0 * len(runs), 4.8), squeeze=False)
    for ax, (cond, rows) in zip(axs[0], runs.items()):
        for st in sorted({r['to'] for r in rows}):
            pts = np.array([r['real'][:2] for r in rows if r['to'] == st])
            ax.plot(pts[:, 1], pts[:, 0], mk(st), ls='', color=COND_COLOR[cond], alpha=0.75,
                    label=f'{st} 실제')
            ests = np.array([r['est'][:2] for r in rows if r['to'] == st])
            ax.plot(ests[:, 1], ests[:, 0], mk(st), ls='', mfc='none', color='0.45',
                    label=f'{st} 추정')
        ax.add_patch(matplotlib.patches.Circle((0, 0), 20, fill=False, ls='--', color='0.3'))
        ax.axhline(0, color='0.7', lw=0.6)
        ax.axvline(0, color='0.7', lw=0.6)
        ax.set_aspect('equal', adjustable='datalim')
        ax.invert_xaxis()            # +y (left) drawn to the left
        ax.set_xlabel('좌우 오차 y (mm, 왼쪽 +)')
        ax.set_ylabel('전후 오차 x (mm, 앞 +)')
        ax.set_title(COND_LABEL[cond] + '\n점선 원: 2 cm', fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_exp3_scatter.png'), dpi=170)
    plt.close(fig)

    # 4. Correction quality (VSLAM run): applied correction vs ideal correction
    if 'vslam' in runs:
        rows = runs['vslam']
        fig, axs = plt.subplots(1, 2, figsize=(10, 4.4))
        need = np.array([r['needed_vec'] for r in rows])
        corr = np.array([r['corr_vec'] for r in rows])
        for k, (ax, name) in enumerate(zip(axs, ('x (전후)', 'y (좌우)'))):
            ax.plot(need[:, k], corr[:, k], 'o', color=COND_COLOR['vslam'])
            lim = np.nanmax(np.abs(np.r_[need[:, k], corr[:, k]])) * 1.15
            ax.plot([-lim, lim], [-lim, lim], '--', color='0.4', lw=0.9, label='이상적 보정 (y = x)')
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
            ax.set_aspect('equal')
            ax.set_xlabel(f'필요한 보정 {name} (mm) = 실제 − 보정 전 추정')
            ax.set_ylabel(f'적용된 보정 {name} (mm)')
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
        fig.suptitle('VSLAM 스테이션 보정의 정확도 (점이 대각선에 가까울수록 정확)', fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(out, 'fig_exp3_correction.png'), dpi=170)
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    runs = {}
    for path in a.csv:
        rows = load(path)
        runs[rows[0]['cond']] = rows
    summary = {c: summarize(r) for c, r in runs.items()}
    with open(os.path.join(a.out, 'summary.json'), 'w') as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=1)
    lines = ['| 지표 | ' + ' | '.join(COND_LABEL[c] for c in runs) + ' |',
             '|---|' + '---|' * len(runs)]

    def row(label, key, unit, digits=1):
        cells = []
        for c in runs:
            v = summary[c][key]
            cells.append('—' if isinstance(v, tuple) and not np.isfinite(v[0]) else
                         (f'{v[0]:.{digits}f} ± {v[1]:.{digits}f} {unit}' if isinstance(v, tuple)
                          else f'{v:.{digits}f} {unit}'))
        lines.append(f'| {label} | ' + ' | '.join(cells) + ' |')
    row('구간 수', 'legs', '', 0)
    row('도착 성공률 (2 cm·2° 동시)', 'success_pct', '%', 0)
    row('실제 도착 위치 오차', 'real_mm', 'mm')
    row('실제 도착 방향 오차 |yaw|', 'real_yaw_deg', '°', 2)
    row('1회차 실제 위치 오차', 'real_mm_round1', 'mm')
    row('마지막 회차 실제 위치 오차', 'real_mm_round_last', 'mm')
    row('추정 도착 오차 (로봇의 믿음)', 'est_mm', 'mm')
    row('위치추정 오차 |추정−실제|', 'loc_err_mm', 'mm')
    row('위치추정 방향 오차', 'loc_err_yaw_deg', '°', 2)
    row('보정 전 위치추정 오차', 'loc_err_pre_mm', 'mm')
    row('보정량', 'corr_mm', 'mm')
    row('보정 대기 시간', 'wait_s', 's')
    row('스테이션 간 이동 시간', 'nav_s', 's')
    row('태그 잔차 최대', 'tag_residual_px_max', 'px', 2)
    table = '\n'.join(lines)
    open(os.path.join(a.out, 'table6.md'), 'w').write(table + '\n')
    print(table)
    plots(a.out, runs)


if __name__ == '__main__':
    main()
