"""Station round-trip experiment window (table 6): drive A→B→C→A rounds, measure ground truth
(AprilTag PnP from the head IR camera and/or a ruler) and log every leg."""

import csv
import json
import math
from pathlib import Path
import time

import numpy as np
from PyQt5.QtGui import QBrush, QColor
from PyQt5.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QListWidget, QPushButton, QSpinBox, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)
from std_msgs.msg import String

from . import station_tag as st
from .experiment_core import (
    COLUMNS, CONDITIONS, MEASURE_MODES, blank_row, build_route, fill_pose, format_summary,
    measured_error, pose_error, set_ground_truth, success, summarize,
)

INTERRUPT_GRACE_SEC = 2.0
START_RADIUS_M = 0.15
MIN_TAGS = 2
MIN_DETECTION_RATIO = 0.5

TABLE_COLUMNS = [
    ('leg', '구간'), ('round', '회차'), ('to', '도착'), ('nav_s', '이동 s'),
    ('correction_wait_s', '보정 대기 s'), ('correction_mm', '보정량 mm'),
    ('est_err_mm', '추정 오차 mm'), ('gt_source', '정답'), ('gt_err_mm', '위치 오차 mm'),
    ('gt_yaw_deg', '방향 오차 °'), ('tag_residual_px', '태그 잔차 px'), ('success', '성공'),
    ('note', '메모'),
]


def _fmt(value, digits=1):
    if isinstance(value, bool):
        return 'O' if value else 'X'
    if isinstance(value, float):
        return '' if not math.isfinite(value) else f'{value:.{digits}f}'
    return str(value)


class ExperimentWindow(QWidget):
    def __init__(self, node, waypoints_getter, probe=None):
        super().__init__()
        self.node = node
        self.waypoints = waypoints_getter
        self._probe = probe
        self.tag_due = None
        self.tag_refs_path = node.waypoint_path.with_name(node.waypoint_path.stem + '_tags.json')
        self.tag_refs = self._load_refs()
        self.state = 'idle'
        self.legs = []
        self.leg_index = 0
        self.rows = []
        self.current = None
        self.csv_path = None
        self.run_id = None
        self.pause_requested = False
        self.idle_since = None
        self.setWindowTitle('스테이션 왕복 실험 (표 6)')
        self.resize(1000, 820)
        self._build()
        self.node.observer = self

    # ---------- layout ----------
    def _build(self):
        layout = QVBoxLayout(self)

        setup = QGroupBox('1. 실험 설정')
        form = QFormLayout(setup)
        self.route_edit = QLineEdit('A,B,C')
        form.addRow('경로 (첫 스테이션에서 출발·복귀)', self.route_edit)
        self.rounds_spin = QSpinBox()
        self.rounds_spin.setRange(1, 50)
        self.rounds_spin.setValue(10)
        form.addRow('반복 횟수 (1회 = A→B→C→A)', self.rounds_spin)
        self.condition_combo = QComboBox()
        for key, label in CONDITIONS.items():
            self.condition_combo.addItem(label, key)
        form.addRow('조건', self.condition_combo)
        self.span_spin = self._spin(1, 2000, 500, ' mm', 0)
        form.addRow('앞 모서리 좌·우 표식 간 거리 L', self.span_spin)
        tol = QHBoxLayout()
        self.pos_tol_spin = self._spin(1, 200, 20, ' mm', 0)
        self.yaw_tol_spin = self._spin(0.1, 20, 2, ' °', 1)
        tol.addWidget(QLabel('위치'))
        tol.addWidget(self.pos_tol_spin)
        tol.addWidget(QLabel('방향'))
        tol.addWidget(self.yaw_tol_spin)
        form.addRow('성공 기준 (동시 만족)', tol)
        self.mode_combo = QComboBox()
        for key, label in MEASURE_MODES.items():
            self.mode_combo.addItem(label, key)
        form.addRow('정답 측정 방식', self.mode_combo)
        self.settle_spin = self._spin(0.0, 10.0, 1.0, ' s', 1)
        form.addRow('정지 후 태그 측정 전 대기', self.settle_spin)
        self.operator_edit = QLineEdit()
        self.operator_edit.setPlaceholderText('예: 조명 ON, 블라인드 닫음, 측정자 이름')
        form.addRow('실험 메모', self.operator_edit)
        form.addRow('저장 폴더', QLabel(str(self.node.p['experiment_dir'])))
        layout.addWidget(setup)

        tags = QGroupBox('AprilTag 기준 — 스테이션 저장 직후, 움직이지 않은 상태에서 등록 '
                         '(헤드·팔 자세는 실험 내내 고정)')
        tag_row = QGridLayout(tags)
        self.tag_station_combo = QComboBox()
        self.tag_station_combo.setEditable(True)
        self.tag_size_spin = self._spin(0.02, 1.0, 0.220, ' m', 3)
        register = QPushButton('선택 스테이션 태그 기준 등록')
        register.clicked.connect(self.register_tags)
        check = QPushButton('태그 확인 (현재 오차)')
        check.clicked.connect(self.check_tags)
        tag_row.addWidget(QLabel('스테이션'), 0, 0)
        tag_row.addWidget(self.tag_station_combo, 0, 1)
        tag_row.addWidget(QLabel('검은 사각형 한 변'), 0, 2)
        tag_row.addWidget(self.tag_size_spin, 0, 3)
        tag_row.addWidget(register, 0, 4)
        tag_row.addWidget(check, 0, 5)
        self.tag_label = QLabel()
        self.tag_label.setWordWrap(True)
        tag_row.addWidget(self.tag_label, 1, 0, 1, 6)
        self.tag_buttons = (register, check)
        layout.addWidget(tags)
        self._refresh_tag_label()

        buttons = QHBoxLayout()
        self.start_button = QPushButton('실험 시작')
        self.start_button.clicked.connect(self.start)
        self.pause_button = QPushButton('현재 구간 후 일시정지')
        self.pause_button.clicked.connect(self.request_pause)
        self.resume_button = QPushButton('다음 구간 진행')
        self.resume_button.clicked.connect(self.resume)
        self.retry_button = QPushButton('현재 구간 재시도')
        self.retry_button.clicked.connect(self.retry)
        self.abort_button = QPushButton('실험 종료')
        self.abort_button.setStyleSheet('background: #b00020; color: white; font-weight: bold;')
        self.abort_button.clicked.connect(self.abort)
        for b in (self.start_button, self.pause_button, self.resume_button, self.retry_button,
                  self.abort_button):
            b.setMinimumHeight(34)
            buttons.addWidget(b)
        layout.addLayout(buttons)

        self.status_label = QLabel('대기 중: 로봇을 첫 스테이션에 두고 보정을 마친 뒤 시작하세요.')
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet('font-size: 17px; font-weight: bold; padding: 6px;'
                                        'background: #f2f2f2;')
        layout.addWidget(self.status_label)

        measure = QGroupBox('2. 실측 입력 — 도착 후 로봇이 멈추면 앞 모서리 두 곳을 표식 기준으로 잰다')
        grid = QGridLayout(measure)
        guide = QLabel('Δx: 표식보다 로봇이 앞(전진 방향)이면 +, 뒤면 −   ·   '
                       'Δy: 표식보다 로봇이 왼쪽이면 +, 오른쪽이면 −   (로봇 기준, mm)')
        guide.setWordWrap(True)
        grid.addWidget(guide, 0, 0, 1, 6)
        self.inputs = {}
        for col, (key, label) in enumerate((('dx_left_mm', '왼쪽 Δx'), ('dy_left_mm', '왼쪽 Δy'),
                                            ('dx_right_mm', '오른쪽 Δx'), ('dy_right_mm', '오른쪽 Δy'))):
            grid.addWidget(QLabel(label), 1, col)
            edit = QLineEdit()
            edit.setPlaceholderText('mm')
            edit.textChanged.connect(self._preview)
            edit.returnPressed.connect(self.record)
            grid.addWidget(edit, 2, col)
            self.inputs[key] = edit
        self.note_edit = QLineEdit()
        self.note_edit.setPlaceholderText('메모 (선택)')
        grid.addWidget(self.note_edit, 2, 4)
        self.preview_label = QLabel('—')
        grid.addWidget(self.preview_label, 3, 0, 1, 4)
        self.record_button = QPushButton('기록 후 다음 이동 (Enter)')
        self.record_button.setMinimumHeight(38)
        self.record_button.setStyleSheet('font-weight: bold;')
        self.record_button.clicked.connect(self.record)
        grid.addWidget(self.record_button, 3, 4)
        self.skip_button = QPushButton('줄자 없이 기록')
        self.skip_button.clicked.connect(lambda: self.record(skip=True))
        grid.addWidget(self.skip_button, 3, 5)
        self.retag_button = QPushButton('태그 재측정')
        self.retag_button.clicked.connect(self.retag)
        grid.addWidget(self.retag_button, 2, 5)
        layout.addWidget(measure)

        self.table = QTableWidget(0, len(TABLE_COLUMNS))
        self.table.setHorizontalHeaderLabels([t for _, t in TABLE_COLUMNS])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.table, 1)
        self.summary_label = QLabel('—')
        self.summary_label.setWordWrap(True)
        self.summary_label.setStyleSheet('padding: 4px;')
        layout.addWidget(self.summary_label)
        self.log_list = QListWidget()
        self.log_list.setMaximumHeight(90)
        layout.addWidget(self.log_list)
        self._update_buttons()

    def _spin(self, low, high, value, suffix, decimals):
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(decimals)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    # ---------- helpers ----------
    def log(self, text):
        self.log_list.addItem(time.strftime('%H:%M:%S ') + text)
        self.log_list.scrollToBottom()
        self.node.emit_log('[실험] ' + text)

    def _publish(self, event, **data):
        packet = {'event': event, 'run_id': self.run_id, 'wall': time.time(), **data}
        self.node.experiment_pub.publish(String(data=json.dumps(packet, ensure_ascii=False,
                                                                default=str)))

    def _pose(self, name, require_fresh=True):
        item = self.node.poses.get(name)
        if item is None or (require_fresh and not self.node.pose_fresh(name)):
            return None
        values = item[1]
        return values[0], values[1], values[5]

    def _waypoint(self, name):
        for item in self.waypoints():
            if item.name == name:
                return item
        return None

    def _set_status(self, text):
        self.status_label.setText(text)

    def _update_buttons(self):
        running = self.state not in ('idle', 'done')
        self.start_button.setEnabled(not running)
        self.pause_button.setEnabled(self.state in ('travel', 'settle', 'tagging', 'measure'))
        self.resume_button.setEnabled(self.state == 'paused')
        self.retry_button.setEnabled(self.state == 'interrupted')
        self.abort_button.setEnabled(running)
        measuring = self.state == 'measure'
        for widget in (*self.inputs.values(), self.note_edit, self.record_button, self.skip_button,
                       self.retag_button):
            widget.setEnabled(measuring)
        for widget in (self.route_edit, self.rounds_spin, self.condition_combo, self.span_spin,
                       self.pos_tol_spin, self.yaw_tol_spin, self.mode_combo, self.tag_size_spin,
                       *self.tag_buttons):
            widget.setEnabled(not running)

    def _leg(self):
        return self.legs[self.leg_index] if 0 <= self.leg_index < len(self.legs) else None

    def _leg_text(self, leg):
        return (f'구간 {leg.index}/{len(self.legs)} · {leg.round}회차 · '
                f'{leg.start} → {leg.target}')

    # ---------- run control ----------
    def start(self):
        try:
            route = [s.strip() for s in self.route_edit.text().split(',') if s.strip()]
            legs = build_route(route, self.rounds_spin.value())
            missing = [name for name in route if self._waypoint(name) is None]
            if missing:
                raise ValueError('저장되지 않은 스테이션: ' + ', '.join(missing))
            if self._mode() != 'ruler':
                no_ref = [n for n in route if len(self.tag_refs.get(n, {}).get('ids', [])) < MIN_TAGS]
                if no_ref:
                    raise ValueError('AprilTag 기준이 없는 스테이션: ' + ', '.join(no_ref)
                                     + ' (태그 2개 이상으로 등록 필요)')
            state = self.node.status('localization')
            if not self.node.fresh('localization') or state.get('initialized') is not True:
                raise ValueError('위치 추정이 초기화되지 않음: 첫 스테이션에서 보정을 먼저 완료')
            here = self._pose('Localization')
            first = self._waypoint(route[0])
            if here is None:
                raise ValueError('현재 위치 추정값이 없음')
            distance = math.hypot(here[0] - first.x, here[1] - first.y)
            if distance > START_RADIUS_M:
                raise ValueError(f'로봇이 {route[0]}에서 {distance:.2f} m 떨어져 있음: '
                                 f'먼저 {route[0]}로 Goto 후 시작')
        except ValueError as exc:
            self.log('시작 불가: ' + str(exc))
            self._set_status('시작 불가: ' + str(exc))
            return
        condition = self.condition_combo.currentData()
        self.legs, self.leg_index, self.rows = legs, 0, []
        self.run_id = time.strftime('%Y%m%d_%H%M%S') + '_' + condition
        folder = Path(str(self.node.p['experiment_dir'])).expanduser()
        folder.mkdir(parents=True, exist_ok=True)
        self.csv_path = folder / (self.run_id + '.csv')
        with open(self.csv_path, 'w', newline='') as f:
            csv.DictWriter(f, fieldnames=COLUMNS).writeheader()
        meta = {
            'run_id': self.run_id, 'condition': condition, 'route': route,
            'rounds': self.rounds_spin.value(), 'corner_span_mm': self.span_spin.value(),
            'pos_tol_mm': self.pos_tol_spin.value(), 'yaw_tol_deg': self.yaw_tol_spin.value(),
            'note': self.operator_edit.text(), 'waypoints_file': str(self.node.waypoint_path),
            'measure_mode': self._mode(), 'tag_refs_file': str(self.tag_refs_path),
            'tag_refs': {n: {k: v for k, v in self.tag_refs[n].items() if k != 'ref_corners_base'}
                         for n in route if n in self.tag_refs},
            'stations': {n: vars(self._waypoint(n)) for n in route},
            'started_wall': time.time(),
        }
        with open(folder / (self.run_id + '_meta.json'), 'w') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        self.table.setRowCount(0)
        self.node.correct_on_arrival = condition == 'vslam'
        self.pause_requested = False
        self.log(f'실험 시작 {self.run_id}: {CONDITIONS[condition]}, 구간 {len(legs)}개 → {self.csv_path}')
        self._publish('start', **meta)
        self._go()

    def _go(self):
        leg = self._leg()
        if leg is None:
            self._finish()
            return
        target = self._waypoint(leg.target)
        if target is None:
            self._interrupt(f'스테이션 {leg.target}이 목록에서 사라짐')
            return
        self.current = {'leg': leg, 'start_wall': time.time(), 'start_mono': time.monotonic(),
                        'target': (target.x, target.y, target.yaw)}
        if not self.node.enable_then_navigate(target):
            self._interrupt('Goto가 시작되지 않음 (로그 확인)')
            return
        self.state = 'travel'
        self.idle_since = None
        self._set_status(self._leg_text(leg) + ' · 이동 중')
        self._publish('leg_start', leg=leg.index, round=leg.round, start=leg.start, target=leg.target)
        self._update_buttons()

    # Observer hooks from OperatorNode
    def on_arrived(self, station):
        if self.state != 'travel' or self.current is None:
            return
        self.current['arrived_wall'] = time.time()
        self.current['arrived_mono'] = time.monotonic()
        self.current['loc_arrived'] = self._pose('Localization', require_fresh=False)
        self.state = 'settle'
        self.idle_since = None
        leg = self.current['leg']
        self._set_status(self._leg_text(leg) + (' · 도착, VSLAM 보정 대기' if self.node.correct_on_arrival
                                                else ' · 도착'))
        self._publish('arrived', leg=leg.index, target=station)

    def on_ready(self, station):
        if self.state != 'settle' or self.current is None:
            return
        self.current['ready_wall'] = time.time()
        self.current['ready_mono'] = time.monotonic()
        self.current['loc_ready'] = self._pose('Localization', require_fresh=False)
        leg = self.current['leg']
        for edit in self.inputs.values():
            edit.clear()
        self.note_edit.clear()
        self._publish('ready', leg=leg.index, target=station)
        if self._mode() == 'ruler':
            self._enter_measure(' · 정지 완료 → 줄자 실측을 입력하세요')
        else:
            self._start_tagging(self.settle_spin.value())

    def _enter_measure(self, text):
        self.state = 'measure'
        self._set_status(self._leg_text(self.current['leg']) + text)
        self._update_buttons()
        self._preview()
        self.inputs['dx_left_mm'].setFocus()

    def _start_tagging(self, delay):
        self.state = 'tagging'
        self.tag_due = time.monotonic() + delay
        self.current.pop('tag', None)
        self.current.pop('tag_error', None)
        self._set_status(self._leg_text(self.current['leg']) + ' · 정지 완료 → AprilTag 측정 중')
        self._update_buttons()

    def _on_leg_tags(self, result, error):
        if self.state != 'tagging' or self.current is None:
            return
        leg = self.current['leg']
        try:
            if error:
                raise ValueError(error)
            self.current['tag'] = self._solve(leg.target, result)
        except ValueError as exc:
            self.current['tag_error'] = str(exc)
            self.log(f'{leg.target} 태그 측정 실패: {exc}')
            self._enter_measure(' · 태그 측정 실패 → "태그 재측정" 또는 줄자 입력 / 건너뛰기')
            return
        tag = self.current['tag']
        self.log(f"{leg.target} 태그: {tag['err_mm']:.1f} mm (x {tag['x_mm']:+.1f}, y {tag['y_mm']:+.1f}), "
                 f"{tag['yaw_deg']:+.2f}°, 잔차 {tag['residual_px']:.2f} px, ID {tag['ids']}")
        if self._mode() == 'tag':
            self.record(skip=True)
        else:
            self._enter_measure(' · 태그 측정 완료 → 줄자 실측을 입력하세요')

    def retag(self):
        if self.state == 'measure' and self.current is not None:
            self._start_tagging(0.0)

    def on_failed(self, message):
        if self.state in ('travel', 'settle'):
            self._interrupt(message)

    def _interrupt(self, message):
        self.state = 'interrupted'
        leg = self._leg()
        text = (self._leg_text(leg) + ' · ' if leg else '') + '중단: ' + str(message)
        self._set_status(text + ' → "현재 구간 재시도" 또는 "실험 종료"')
        self.log(text)
        self._publish('interrupted', leg=leg.index if leg else None, message=str(message))
        self._update_buttons()

    def tick(self):
        """Drive tag captures; detect an operator STOP or a silent cancel while moving."""
        if self._probe is not None:
            self._probe.tick()
        if self.state == 'tagging' and self.tag_due is not None and time.monotonic() >= self.tag_due:
            self.tag_due = None
            self._capture(self._on_leg_tags)
            return
        if self.state not in ('travel', 'settle'):
            return
        busy = self.node.pending is not None or self.node.active_request is not None
        if busy:
            self.idle_since = None
            return
        now = time.monotonic()
        if self.idle_since is None:
            self.idle_since = now
        elif now - self.idle_since > INTERRUPT_GRACE_SEC:
            self._interrupt('이동/보정이 외부에서 정지됨 (STOP 또는 취소)')

    def _preview(self):
        tag = (self.current or {}).get('tag')
        prefix = (f"태그: {tag['err_mm']:.1f} mm / {tag['yaw_deg']:+.2f}°   ·   " if tag else
                  ('태그 실패   ·   ' if (self.current or {}).get('tag_error') else ''))
        try:
            values = [float(self.inputs[k].text()) for k in
                      ('dx_left_mm', 'dy_left_mm', 'dx_right_mm', 'dy_right_mm')]
            ex, ey, err, yaw = measured_error(*values, self.span_spin.value())
            ok = success(err, yaw, self.pos_tol_spin.value(), self.yaw_tol_spin.value())
            self.preview_label.setText(
                prefix + f'줄자: 위치 오차 {err:.1f} mm (x {ex:+.1f}, y {ey:+.1f}) · '
                f'방향 오차 {yaw:+.2f}° · ' + ('성공' if ok else '실패'))
            self.preview_label.setStyleSheet('color: #1b6e2d;' if ok else 'color: #b00020;')
        except ValueError:
            self.preview_label.setText(prefix + '줄자: 네 값을 모두 입력하면 오차를 계산합니다')
            self.preview_label.setStyleSheet('')

    def record(self, skip=False):
        if self.state not in ('measure', 'tagging') or self.current is None:
            return
        row = blank_row()
        leg = self.current['leg']
        measured = None
        if not skip and self._mode() == 'tag' and self.current.get('tag'):
            skip = True
        if not skip:
            try:
                values = {k: float(e.text()) for k, e in self.inputs.items()}
                measured = measured_error(values['dx_left_mm'], values['dy_left_mm'],
                                          values['dx_right_mm'], values['dy_right_mm'],
                                          self.span_spin.value())
            except ValueError:
                self.log('줄자 실측값 4개를 숫자로 입력하세요 (mm). 측정을 못 했다면 "줄자 없이 기록".')
                return
            row.update(values)
        now_wall, now_mono = time.time(), time.monotonic()
        target = self.current['target']
        loc_record = self._pose('Localization', require_fresh=False)
        slam_record = self._pose('SLAM')
        row.update({
            'run_id': self.run_id, 'condition': self.condition_combo.currentData(),
            'leg': leg.index, 'round': leg.round, 'from': leg.start, 'to': leg.target,
            'start_wall': self.current['start_wall'],
            'arrived_wall': self.current.get('arrived_wall', ''),
            'ready_wall': self.current.get('ready_wall', ''), 'recorded_wall': now_wall,
            'nav_s': self.current['arrived_mono'] - self.current['start_mono'],
            'measure_s': now_mono - self.current['ready_mono'],
            'target_x': target[0], 'target_y': target[1],
            'target_yaw_deg': math.degrees(target[2]),
            'corner_span_mm': self.span_spin.value(), 'pos_tol_mm': self.pos_tol_spin.value(),
            'yaw_tol_deg': self.yaw_tol_spin.value(), 'note': self.note_edit.text(),
        })
        if self.node.correct_on_arrival:
            row['correction_wait_s'] = self.current['ready_mono'] - self.current['arrived_mono']
        fill_pose(row, 'loc_arrived', self.current.get('loc_arrived'))
        fill_pose(row, 'loc_ready', self.current.get('loc_ready'))
        fill_pose(row, 'loc_record', loc_record)
        fill_pose(row, 'slam_record', slam_record)
        a, b = self.current.get('loc_arrived'), self.current.get('loc_ready')
        if self.node.correct_on_arrival and a and b:
            row['correction_mm'] = math.hypot(b[0] - a[0], b[1] - a[1]) * 1000
            row['correction_deg'] = math.degrees(math.atan2(math.sin(b[2] - a[2]),
                                                            math.cos(b[2] - a[2])))
        for prefix, pose in (('est_err', loc_record), ('slam_err', slam_record)):
            err = pose_error(pose, target)
            if err:
                row[prefix + '_x_mm'], row[prefix + '_y_mm'], row[prefix + '_mm'], \
                    row[prefix + '_yaw_deg'] = err
        if measured:
            row['meas_err_x_mm'], row['meas_err_y_mm'], row['meas_err_mm'], \
                row['meas_err_yaw_deg'] = measured
        tag = self.current.get('tag')
        if tag:
            row.update({'tag_x_mm': tag['x_mm'], 'tag_y_mm': tag['y_mm'], 'tag_err_mm': tag['err_mm'],
                        'tag_yaw_deg': tag['yaw_deg'], 'tag_residual_px': tag['residual_px'],
                        'tag_ids': ' '.join(str(i) for i in tag['ids']),
                        'tag_frames': tag['frames'], 'tag_corner_sd_px': tag['corner_sd_px']})
        elif self.current.get('tag_error'):
            row['note'] = (row['note'] + ' ' if row['note'] else '') + '[태그 실패] ' + self.current['tag_error']
        set_ground_truth(row, tag=tag, ruler=measured)
        with open(self.csv_path, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=COLUMNS).writerow(
                {k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()})
        self.rows.append(row)
        self._append_table(row)
        self.summary_label.setText(format_summary(summarize(self.rows)))
        self._publish('recorded', **{k: row[k] for k in (
            'leg', 'round', 'to', 'gt_source', 'gt_err_mm', 'gt_yaw_deg', 'success', 'est_err_mm',
            'correction_mm', 'tag_residual_px')})
        self.log(f"{leg.target} 기록: 정답({row['gt_source'] or '없음'}) "
                 + (f"{row['gt_err_mm']:.1f} mm / {row['gt_yaw_deg']:+.2f}°"
                    if row['gt_source'] else ''))
        self.leg_index += 1
        self.current = None
        if self.leg_index >= len(self.legs):
            self._finish()
        elif self.pause_requested:
            self.pause_requested = False
            self.state = 'paused'
            self._set_status('일시정지: "다음 구간 진행"으로 계속 — 다음: ' + self._leg_text(self._leg()))
            self._update_buttons()
        else:
            self._go()

    def _append_table(self, row):
        r = self.table.rowCount()
        self.table.insertRow(r)
        for c, (key, _) in enumerate(TABLE_COLUMNS):
            item = QTableWidgetItem(_fmt(row.get(key, '')))
            if key == 'success' and isinstance(row.get(key), bool):
                item.setForeground(QBrush(QColor('#1b6e2d' if row[key] else '#b00020')))
            self.table.setItem(r, c, item)
        self.table.scrollToBottom()

    def request_pause(self):
        self.pause_requested = True
        self.log('현재 구간 기록 후 일시정지합니다.')

    def resume(self):
        if self.state == 'paused':
            self._go()

    def retry(self):
        """Re-drive the current leg (after an interruption or a bad arrival)."""
        if self.state != 'interrupted':
            return
        leg = self._leg()
        if leg is None:
            return
        self.log(f'구간 {leg.index} 재시도: {leg.target}로 다시 이동 '
                 '(현재 위치에서 출발; 필요하면 먼저 수동으로 출발 스테이션 근처로 이동)')
        self._publish('retry', leg=leg.index)
        self._go()

    def _finish(self):
        self.state = 'done'
        self.node.correct_on_arrival = True
        text = '실험 완료: ' + format_summary(summarize(self.rows))
        self._set_status(text)
        self.log(text + f' → {self.csv_path}')
        self._publish('finish', rows=len(self.rows))
        self._update_buttons()

    def abort(self):
        if self.state in ('travel', 'settle'):
            self.node.stop()
        self.node.correct_on_arrival = True
        self.state = 'done'
        self._set_status('실험 종료 (기록 ' + str(len(self.rows)) + '건 저장됨: '
                         + str(self.csv_path) + ')')
        self.log('실험 종료')
        self._publish('abort', rows=len(self.rows))
        self._update_buttons()

    # ---------- AprilTag ----------
    def _mode(self):
        return self.mode_combo.currentData()

    def probe(self):
        if self._probe is None:
            from .tag_probe import TagProbe
            self._probe = TagProbe(self.node, str(self.node.p['tag_image_topic']),
                                   str(self.node.p['tag_info_topic']), self.node.base_frame)
        return self._probe

    def _capture(self, callback):
        try:
            self.probe().capture(callback, duration=2.0, max_frames=40)
        except Exception as exc:
            callback(None, f'tag capture could not start: {exc}')

    def _load_refs(self):
        try:
            with open(self.tag_refs_path) as f:
                data = json.load(f)
            for ref in data.values():
                ref['ref_corners_base'] = {int(k): np.asarray(v) for k, v in
                                           ref['ref_corners_base'].items()}
            return data
        except FileNotFoundError:
            return {}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.node.emit_log(f'[실험] 태그 기준 파일을 읽지 못함: {exc}')
            return {}

    def _save_refs(self):
        data = {}
        for name, ref in self.tag_refs.items():
            item = dict(ref)
            item['ref_corners_base'] = {str(k): np.asarray(v).tolist()
                                        for k, v in ref['ref_corners_base'].items()}
            data[name] = item
        tmp = self.tag_refs_path.with_suffix('.tmp')
        with open(tmp, 'w') as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
        tmp.replace(self.tag_refs_path)

    def _refresh_tag_label(self):
        names = [w.name for w in self.waypoints()]
        current = self.tag_station_combo.currentText()
        self.tag_station_combo.clear()
        self.tag_station_combo.addItems(names)
        if current:
            self.tag_station_combo.setCurrentText(current)
        lines = []
        for name in names:
            ref = self.tag_refs.get(name)
            lines.append(f"{name}: " + (f"ID {ref['ids']} · {ref['size_m'] * 1000:.0f} mm · "
                                        f"{time.strftime('%m-%d %H:%M', time.localtime(ref['captured_wall']))}"
                                        if ref else '미등록'))
        self.tag_label.setText('   |   '.join(lines) + f'   (파일: {self.tag_refs_path.name})')

    def _good_tags(self, result):
        frames = max(1, result['frames'])
        return sorted(i for i, n in result['detections'].items() if n / frames >= MIN_DETECTION_RATIO)

    def _solve(self, station, result):
        ref = self.tag_refs.get(station)
        if not ref:
            raise ValueError(f'{station} 태그 기준 없음')
        ids = [i for i in self._good_tags(result) if i in ref['ref_corners_base']]
        if len(ids) < MIN_TAGS:
            raise ValueError(f"기준 태그 {ref['ids']} 중 {ids}만 보임 ({MIN_TAGS}개 이상 필요)")
        out = st.solve_planar({i: ref['ref_corners_base'][i] for i in ids},
                              {i: result['corners'][i] for i in ids},
                              result['K'], result['D'], result['T_base_cam'])
        out['frames'] = result['frames']
        out['corner_sd_px'] = max(result['corner_sd_px'][i] for i in ids)
        return out

    def register_tags(self):
        station = self.tag_station_combo.currentText().strip()
        waypoint = self._waypoint(station)
        if waypoint is None:
            self.log('태그 기준 등록: 먼저 스테이션을 저장하세요.')
            return
        here = self._pose('Localization')
        if here is not None:
            gap = math.hypot(here[0] - waypoint.x, here[1] - waypoint.y) * 1000
            if gap > 10:
                self.log(f'주의: 현재 위치 추정이 {station}에서 {gap:.0f} mm 떨어져 있음. 태그 기준은 '
                         '스테이션을 저장한 직후(움직이지 않은 상태)에 등록해야 정답이 맞습니다.')
        size = self.tag_size_spin.value()
        self._set_status(f'{station} 태그 기준 등록: IR 영상 2초 수집 중…')

        def done(result, error):
            try:
                if error:
                    raise ValueError(error)
                ids = self._good_tags(result)
                if len(ids) < MIN_TAGS:
                    raise ValueError(f'안정적으로 보이는 태그 {ids} ({MIN_TAGS}개 이상 필요, 3개 권장)')
                corners = {i: result['corners'][i] for i in ids}
                ref_base = st.reference_corners(corners, result['K'], result['D'], size,
                                                result['T_base_cam'])
                self.tag_refs[station] = {
                    'ids': ids, 'size_m': size, 'frame': result['frame'],
                    'captured_wall': time.time(), 'frames': result['frames'],
                    'corner_sd_px': {str(i): result['corner_sd_px'][i] for i in ids},
                    'T_base_cam': st.to_list(result['T_base_cam']),
                    'distance_m': {str(i): float(np.linalg.norm(
                        np.asarray(ref_base[i]).mean(axis=0)[:2])) for i in ids},
                    'ref_corners_base': ref_base,
                }
                self._save_refs()
                self._refresh_tag_label()
                dist = ', '.join(f"ID{i} {self.tag_refs[station]['distance_m'][str(i)]:.2f} m" for i in ids)
                text = f'{station} 태그 기준 등록 완료: {dist}' + (
                    '' if len(ids) >= 3 else ' · 태그 3개(가까운 2 + 먼 1) 권장')
            except Exception as exc:
                text = f'{station} 태그 기준 등록 실패: {exc}'
            self._set_status(text)
            self.log(text)
        self._capture(done)

    def check_tags(self):
        station = self.tag_station_combo.currentText().strip()
        self._set_status(f'태그 확인: IR 영상 2초 수집 중… (기준: {station or "없음"})')

        def done(result, error):
            try:
                if error:
                    raise ValueError(error)
                seen = self._good_tags(result)
                text = f"보이는 태그 {seen} ({result['frames']} 프레임)"
                if station in self.tag_refs:
                    r = self._solve(station, result)
                    text += (f" · {station} 기준 오차 {r['err_mm']:.1f} mm (x {r['x_mm']:+.1f}, "
                             f"y {r['y_mm']:+.1f}), {r['yaw_deg']:+.2f}°, 잔차 {r['residual_px']:.2f} px")
            except Exception as exc:
                text = f'태그 확인 실패: {exc}'
            self._set_status(text)
            self.log(text)
        self._capture(done)

    def showEvent(self, event):
        self._refresh_tag_label()
        self.node.observer = self
        super().showEvent(event)

    def closeEvent(self, event):
        # Closing must not leave the robot running unobserved: end the run first.
        if self.state not in ('idle', 'done'):
            self.log('실험 진행 중에는 창을 닫을 수 없습니다. 먼저 "실험 종료"를 누르세요.')
            event.ignore()
            return
        event.accept()
