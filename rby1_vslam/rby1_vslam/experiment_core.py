"""Station round-trip experiment: route, error arithmetic and CSV records.

Measurement convention (operator, at the two floor markers placed under the
base's front-left and front-right corners when the station was saved):
  dx = forward offset of the corner from its marker [mm], + = robot is ahead
  dy = lateral offset of the corner from its marker [mm], + = robot is to the left
Position error = |mean of the two corner offsets|; yaw error = atan2(dxR − dxL, L)
(+ = counter-clockwise), L = distance between the two corners.

AprilTag ground truth (station_tag) reports the base origin in the registered
station frame with the same signs (x + ahead, y + left, yaw + CCW). The ground
truth (gt_*) of a leg is the tag result when available, else the ruler.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence

CONDITIONS = {
    'wheel': '휠 오도메트리 단독 (보정 없음)',
    'vslam': '휠 오도메트리 + VSLAM 전역 보정 (방안 2)',
}

COLUMNS = [
    'run_id', 'condition', 'leg', 'round', 'from', 'to',
    'start_wall', 'arrived_wall', 'ready_wall', 'recorded_wall',
    'nav_s', 'correction_wait_s', 'measure_s',
    'target_x', 'target_y', 'target_yaw_deg',
    'loc_arrived_x', 'loc_arrived_y', 'loc_arrived_yaw_deg',
    'loc_ready_x', 'loc_ready_y', 'loc_ready_yaw_deg',
    'loc_record_x', 'loc_record_y', 'loc_record_yaw_deg',
    'slam_record_x', 'slam_record_y', 'slam_record_yaw_deg',
    'correction_mm', 'correction_deg',
    'est_err_x_mm', 'est_err_y_mm', 'est_err_mm', 'est_err_yaw_deg',
    'slam_err_x_mm', 'slam_err_y_mm', 'slam_err_mm', 'slam_err_yaw_deg',
    'dx_left_mm', 'dy_left_mm', 'dx_right_mm', 'dy_right_mm', 'corner_span_mm',
    'meas_err_x_mm', 'meas_err_y_mm', 'meas_err_mm', 'meas_err_yaw_deg',
    'tag_x_mm', 'tag_y_mm', 'tag_err_mm', 'tag_yaw_deg', 'tag_residual_px', 'tag_ids',
    'tag_frames', 'tag_corner_sd_px',
    'gt_source', 'gt_x_mm', 'gt_y_mm', 'gt_err_mm', 'gt_yaw_deg',
    'success', 'pos_tol_mm', 'yaw_tol_deg', 'note',
]

MEASURE_MODES = {
    'tag': 'AprilTag 자동 (기록·진행 자동)',
    'tag+ruler': 'AprilTag + 줄자 (둘 다 기록, 줄자 입력 대기)',
    'ruler': '줄자 수동',
}


@dataclass(frozen=True)
class Leg:
    index: int      # 1-based over the whole run
    round: int      # 1-based
    start: str
    target: str


def build_route(stations: Sequence[str], rounds: int) -> List[Leg]:
    """Start at stations[0]; each round visits the others and returns to it."""
    names = [s.strip() for s in stations if s.strip()]
    if len(names) < 2:
        raise ValueError('at least two stations are required')
    if len(set(names)) != len(names):
        raise ValueError('station names must be unique')
    if rounds < 1:
        raise ValueError('rounds must be at least 1')
    order = names[1:] + names[:1]
    legs, previous = [], names[0]
    for r in range(1, rounds + 1):
        for target in order:
            legs.append(Leg(len(legs) + 1, r, previous, target))
            previous = target
    return legs


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def pose_error(pose, target):
    """Pose minus target in the target (station) frame: (x mm, y mm, |xy| mm, yaw deg)."""
    if pose is None or target is None:
        return None
    dx, dy = pose[0] - target[0], pose[1] - target[1]
    c, s = math.cos(-target[2]), math.sin(-target[2])
    ex, ey = (c * dx - s * dy) * 1000.0, (s * dx + c * dy) * 1000.0
    return ex, ey, math.hypot(ex, ey), wrap_deg(math.degrees(pose[2] - target[2]))


def measured_error(dx_left, dy_left, dx_right, dy_right, span_mm):
    """Ruler offsets of the two front corners -> (x mm, y mm, |xy| mm, yaw deg)."""
    values = (dx_left, dy_left, dx_right, dy_right, span_mm)
    if any(not math.isfinite(v) for v in values):
        raise ValueError('measurements must be finite numbers')
    if span_mm <= 0:
        raise ValueError('corner span must be positive')
    ex, ey = (dx_left + dx_right) / 2.0, (dy_left + dy_right) / 2.0
    yaw = math.degrees(math.atan2(dx_right - dx_left, span_mm))
    return ex, ey, math.hypot(ex, ey), yaw


def success(err_mm, yaw_deg, pos_tol_mm, yaw_tol_deg):
    return err_mm <= pos_tol_mm and abs(yaw_deg) <= yaw_tol_deg


def set_ground_truth(row, tag=None, ruler=None):
    """Fill gt_* from the tag result (preferred) or the ruler tuple (x, y, |xy|, yaw)."""
    if tag is not None:
        row.update({'gt_source': 'tag', 'gt_x_mm': tag['x_mm'], 'gt_y_mm': tag['y_mm'],
                    'gt_err_mm': tag['err_mm'], 'gt_yaw_deg': tag['yaw_deg']})
    elif ruler is not None:
        row.update({'gt_source': 'ruler', 'gt_x_mm': ruler[0], 'gt_y_mm': ruler[1],
                    'gt_err_mm': ruler[2], 'gt_yaw_deg': ruler[3]})
    else:
        return None
    row['success'] = success(row['gt_err_mm'], row['gt_yaw_deg'], row['pos_tol_mm'],
                             row['yaw_tol_deg'])
    return row['success']


def summarize(rows: Sequence[Dict]) -> Dict:
    """Aggregate recorded legs that carry a ruler measurement."""
    measured = [r for r in rows if isinstance(r.get('gt_err_mm'), (int, float))
                and math.isfinite(r['gt_err_mm'])]
    out = {'legs': len(rows), 'measured': len(measured)}
    if not measured:
        return out

    def stats(values):
        values = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
        if not values:
            return None
        mean = sum(values) / len(values)
        sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1)) if len(values) > 1 else 0.0
        return mean, sd

    out['success_rate'] = 100.0 * sum(1 for r in measured if r['success']) / len(measured)
    out['pos_err_mm'] = stats([r['gt_err_mm'] for r in measured])
    out['yaw_err_deg'] = stats([abs(r['gt_yaw_deg']) for r in measured])
    out['nav_s'] = stats([r.get('nav_s') for r in rows])
    out['correction_wait_s'] = stats([r.get('correction_wait_s') for r in rows])
    first = min(r['round'] for r in measured)
    last = max(r['round'] for r in measured)
    out['first_round'] = (first, stats([r['gt_err_mm'] for r in measured if r['round'] == first]))
    out['last_round'] = (last, stats([r['gt_err_mm'] for r in measured if r['round'] == last]))
    return out


def format_summary(summary: Dict) -> str:
    if not summary.get('measured'):
        return f"기록 {summary.get('legs', 0)}건 · 실측 없음"

    def ms(value, unit, digits=1):
        return '—' if value is None else f'{value[0]:.{digits}f} ± {value[1]:.{digits}f} {unit}'
    first, last = summary['first_round'], summary['last_round']
    parts = [
        f"실측 {summary['measured']}/{summary['legs']}건",
        f"성공률 {summary['success_rate']:.0f}%",
        f"위치 오차 {ms(summary['pos_err_mm'], 'mm')}",
        f"방향 오차 {ms(summary['yaw_err_deg'], '°', 2)}",
        f"{first[0]}회차 {ms(first[1], 'mm')} → {last[0]}회차 {ms(last[1], 'mm')}",
        f"이동 {ms(summary['nav_s'], 's')}",
        f"보정 대기 {ms(summary['correction_wait_s'], 's')}",
    ]
    return ' · '.join(parts)


def blank_row() -> Dict:
    return {name: '' for name in COLUMNS}


def fill_pose(row: Dict, prefix: str, pose: Optional[Sequence[float]]):
    if pose is None:
        return
    row[prefix + '_x'] = pose[0]
    row[prefix + '_y'] = pose[1]
    row[prefix + '_yaw_deg'] = math.degrees(pose[2])
