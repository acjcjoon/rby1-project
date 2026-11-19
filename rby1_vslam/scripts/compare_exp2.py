#!/usr/bin/env python3
"""Compare experiment-2 analyses across frame rates (same walls, same motions).

  PYTHONNOUSERSITE=1 python3 rby1_vslam/scripts/compare_exp2.py \
      30=/path/exp2_alt_30_..._analysis 90=/path/exp2_alt_90_..._analysis --out DIR

Reads each analysis' trials.csv (included trials only) and writes
compare_table.csv/md and fig_exp2_fps_compare.png. p = two-sided Mann-Whitney U.
"""

import argparse
import csv
import os

import numpy as np
from scipy.stats import mannwhitneyu

ROWS = [('move_err_pct', '주행 중 VO 오차', '% 이동거리'),
        ('move_lat_err_mm', '주행 중 VO 횡방향 오차', 'mm'),
        ('vo_drift_mm', '정지 후 표류량', 'mm'),
        ('vo_max_dev_mm', '최대 변동 폭', 'mm'),
        ('vo_settle_s', '수렴 시간 (±2 mm)', 's'),
        ('vo_path_mm', '정지 중 누적 이동 경로 (10 Hz)', 'mm'),
        ('vo_yaw_drift_deg', 'yaw 표류', '°')]
WALL = {'rich': '특징점 충분 벽', 'poor': '특징점 부족 벽'}


def load(d):
    with open(os.path.join(d, 'trials.csv')) as f:
        rows = [r for r in csv.DictReader(f) if r['included'] == 'True']
    for r in rows:
        for k, v in r.items():
            try:
                r[k] = float(v)
            except ValueError:
                pass
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs=2, help='LABEL=analysis_dir')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    runs = [(s.split('=', 1)[0], load(s.split('=', 1)[1])) for s in a.runs]
    (la, ra), (lb, rb) = runs

    table = []
    for wall in ('rich', 'poor'):
        for k, name, unit in ROWS:
            va = np.array([r[k] for r in ra if r['wall'] == wall])
            vb = np.array([r[k] for r in rb if r['wall'] == wall])
            p = mannwhitneyu(va, vb, alternative='two-sided').pvalue
            table.append({'wall': WALL[wall], 'metric': name, 'unit': unit,
                          f'{la}_n': len(va), f'{la}_mean': va.mean(), f'{la}_sd': va.std(ddof=1),
                          f'{lb}_n': len(vb), f'{lb}_mean': vb.mean(), f'{lb}_sd': vb.std(ddof=1),
                          'p_mannwhitney': p})
    with open(os.path.join(a.out, 'compare_table.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(table[0]))
        w.writeheader()
        for r in table:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    with open(os.path.join(a.out, 'compare_table.md'), 'w') as f:
        f.write(f'| 벽 | 지표 | 단위 | {la} fps | {lb} fps | p |\n|---|---|---|---|---|---|\n')
        for r in table:
            f.write(f"| {r['wall']} | {r['metric']} | {r['unit']} | "
                    f"{r[la + '_mean']:.2f} ± {r[la + '_sd']:.2f} | "
                    f"{r[lb + '_mean']:.2f} ± {r[lb + '_sd']:.2f} | {r['p_mannwhitney']:.2f} |\n")
    print(open(os.path.join(a.out, 'compare_table.md')).read())

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    cjk = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    if os.path.exists(cjk):
        font_manager.fontManager.addfont(cjk)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=cjk).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    col = {la: '#8f8f8f', lb: '#d17a22'}
    shown = [r for r in ROWS if r[0] != 'vo_yaw_drift_deg']
    fig, axs = plt.subplots(1, len(shown), figsize=(2.6 * len(shown), 3.9))
    for ax, (k, name, unit) in zip(axs, shown):
        for wi, wall in enumerate(('rich', 'poor')):
            for j, (lab, rows) in enumerate(runs):
                v = np.array([r[k] for r in rows if r['wall'] == wall])
                x = wi * 1.0 + (j - 0.5) * 0.36
                ax.scatter(np.full(len(v), x) + np.linspace(-0.05, 0.05, len(v)), v, s=14,
                           color=col[lab], alpha=0.65, zorder=3,
                           label=f'{lab} fps' if wi == 0 else None)
                ax.errorbar(x + 0.1, v.mean(), yerr=v.std(ddof=1), fmt='o', color='k', ms=4,
                            capsize=3, zorder=4)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['충분 벽', '부족 벽'], fontsize=9)
        ax.set_xlim(-0.5, 1.5)
        ax.set_ylim(bottom=0)
        ax.set_title(f'{name}\n({unit})', fontsize=9.5)
        ax.grid(alpha=0.3, axis='y')
    axs[0].legend(fontsize=8, loc='lower left')
    fig.suptitle(f'프레임레이트 비교 ({la} vs {lb} fps) · 벽·동작 동일, 벽당 n=5 · '
                 '점: 시행, 검은 점·막대: 평균 ± 표준편차', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'fig_exp2_fps_compare.png'), dpi=170)
    plt.close(fig)


if __name__ == '__main__':
    main()
