"""Measurements tab: tag-to-tag relative pose and base odometry delta.

Nothing here commands the robot. The tag measurement only keeps the chosen
camera's AprilTag detection running while it samples; the odometry
measurement only reads ``/rby1/odom``.

Results are appended to CSV files under ``~/rby1_measurements/``.
"""
from __future__ import annotations

import csv
from datetime import datetime
import math
from pathlib import Path
import time
from typing import Callable, List, Optional, Tuple

from .camera import ObservationUnavailable
from .measurements import (
    OdometryDelta,
    TagRelativeResult,
    average_relative_pose,
    format_odometry_delta,
    format_tag_result,
    parse_tag_name,
)
from .observation import ObjectObservation
from .qt_compat import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QTimer,
    QVBoxLayout,
    QWidget,
)
from .scenario_ui import ScenarioPanel


CAMERA_CHOICES = (
    ('D405 (right hand)', 'd405_right'),
    ('D405 (left hand)', 'd405_left'),
    ('D435 (head)', 'd435'),
)
VISION_OWNER = 'measurements'
RESULT_DIR = Path.home() / 'rby1_measurements'
TAG_CSV = 'tag_relative.csv'
ODOM_CSV = 'base_odometry.csv'

TAG_SAMPLE_COUNT = 20
TAG_OUTLIER_COUNT = 2
TAG_MIN_SAMPLES = 5
TAG_SAMPLING_TIMEOUT_SEC = 6.0
# Both tags must come from (nearly) the same camera frame to be paired.
TAG_PAIR_MAX_SKEW_NS = 60_000_000
VISION_START_TIMEOUT_SEC = 15.0
ODOM_STALE_SEC = 1.0


def _append_csv(name: str, header: List[str], row: List[object]) -> Path:
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    path = RESULT_DIR / name
    new_file = not path.exists()
    with path.open('a', newline='') as stream:
        writer = csv.writer(stream)
        if new_file:
            writer.writerow(header)
        writer.writerow(row)
    return path


class MeasurementsPanel(QWidget):
    TICK_MS = 50

    def __init__(
        self,
        backend,
        *,
        on_log: Callable[[str, str], None],
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.on_log = on_log
        self.cameras = getattr(backend, 'cameras', {}) or {}
        self.vision = getattr(backend, 'vision', None)
        self.odometry = getattr(backend, 'base_odometry', None)

        # Tag measurement state machine: idle -> starting -> sampling.
        self._tag_state = 'idle'
        self._tag_request: Optional[Tuple[str, str, str]] = None
        self._tag_deadline = 0.0
        self._tag_marker = 0
        self._tag_pairs: List[Tuple[ObjectObservation, ObjectObservation]] = []
        self._tag_seen: set = set()

        self._build_ui()
        self.timer = QTimer(self)
        self.timer.setInterval(self.TICK_MS)
        self.timer.timeout.connect(self._tick)
        self.timer.start()
        self._odom_refresh_counter = 0
        self._refresh_odometry()

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)
        top = QHBoxLayout()
        top.setSpacing(8)
        root.addLayout(top, 3)

        # --- 1. Tag-to-tag relative pose ----------------------------------
        tag_group = QGroupBox('Tag-to-Tag Relative Pose')
        tag_layout = QVBoxLayout(tag_group)
        grid = QGridLayout()
        self.camera_combo = QComboBox()
        for label, source in CAMERA_CHOICES:
            if not self.cameras or source in self.cameras:
                self.camera_combo.addItem(label, source)
        ScenarioPanel._style_combo_popup(self.camera_combo)
        self.tag1_edit = QLineEdit()
        self.tag1_edit.setPlaceholderText('tag5')
        self.tag2_edit = QLineEdit()
        self.tag2_edit.setPlaceholderText('tag6')
        grid.addWidget(QLabel('Camera'), 0, 0)
        grid.addWidget(self.camera_combo, 0, 1)
        grid.addWidget(QLabel('Tag 1 (reference)'), 1, 0)
        grid.addWidget(self.tag1_edit, 1, 1)
        grid.addWidget(QLabel('Tag 2'), 2, 0)
        grid.addWidget(self.tag2_edit, 2, 1)
        tag_layout.addLayout(grid)

        buttons = QHBoxLayout()
        self.tag_measure_button = QPushButton('Measure')
        self.tag_measure_button.clicked.connect(self._start_tag_measurement)
        self.tag_cancel_button = QPushButton('Cancel')
        self.tag_cancel_button.clicked.connect(
            lambda: self._finish_tag('Tag measurement cancelled', 'warning')
        )
        self.tag_cancel_button.setEnabled(False)
        self.tag_copy_button = QPushButton('Copy')
        self.tag_copy_button.clicked.connect(
            lambda: self._copy(self.tag_result_view)
        )
        buttons.addWidget(self.tag_measure_button, 2)
        buttons.addWidget(self.tag_cancel_button, 1)
        buttons.addWidget(self.tag_copy_button, 1)
        tag_layout.addLayout(buttons)

        self.tag_status_label = QLabel(
            'Both tags must be visible to the selected camera. '
            'Keep the robot still while sampling.'
        )
        self.tag_status_label.setObjectName('secondaryNote')
        self.tag_status_label.setWordWrap(True)
        tag_layout.addWidget(self.tag_status_label)
        self.tag_result_view = QPlainTextEdit()
        self.tag_result_view.setReadOnly(True)
        tag_layout.addWidget(self.tag_result_view, 1)
        top.addWidget(tag_group, 1)

        # --- 2. Base odometry delta ---------------------------------------
        odom_group = QGroupBox('Base Odometry Delta')
        odom_layout = QVBoxLayout(odom_group)
        note_row = QHBoxLayout()
        self.odom_note_edit = QLineEdit()
        self.odom_note_edit.setPlaceholderText(
            'note (optional), e.g. station A align'
        )
        note_row.addWidget(QLabel('Note'))
        note_row.addWidget(self.odom_note_edit, 1)
        odom_layout.addLayout(note_row)
        odom_buttons = QHBoxLayout()
        self.odom_start_button = QPushButton('Start Measurement')
        self.odom_start_button.clicked.connect(self._start_odometry)
        self.odom_finish_button = QPushButton('Finish Measurement')
        self.odom_finish_button.clicked.connect(self._finish_odometry)
        self.odom_cancel_button = QPushButton('Cancel')
        self.odom_cancel_button.clicked.connect(self._cancel_odometry)
        self.odom_copy_button = QPushButton('Copy')
        self.odom_copy_button.clicked.connect(
            lambda: self._copy(self.odom_result_view)
        )
        odom_buttons.addWidget(self.odom_start_button, 2)
        odom_buttons.addWidget(self.odom_finish_button, 2)
        odom_buttons.addWidget(self.odom_cancel_button, 1)
        odom_buttons.addWidget(self.odom_copy_button, 1)
        odom_layout.addLayout(odom_buttons)
        self.odom_status_label = QLabel('')
        self.odom_status_label.setObjectName('secondaryNote')
        self.odom_status_label.setWordWrap(True)
        odom_layout.addWidget(self.odom_status_label)
        self.odom_live_label = QLabel('')
        self.odom_live_label.setWordWrap(True)
        odom_layout.addWidget(self.odom_live_label)
        self.odom_result_view = QPlainTextEdit()
        self.odom_result_view.setReadOnly(True)
        odom_layout.addWidget(self.odom_result_view, 1)
        top.addWidget(odom_group, 1)

        # --- History ------------------------------------------------------
        history_group = QGroupBox(f'History (saved to {RESULT_DIR}/)')
        history_layout = QVBoxLayout(history_group)
        self.history_view = QPlainTextEdit()
        self.history_view.setReadOnly(True)
        self.history_view.setMaximumBlockCount(500)
        history_layout.addWidget(self.history_view)
        root.addWidget(history_group, 2)
        self._update_odom_buttons()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _log(self, level: str, message: str) -> None:
        self.on_log(level, f'[Measurements] {message}')

    def _history(self, text: str) -> None:
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.history_view.appendPlainText(f'[{stamp}] {text}')

    @staticmethod
    def _copy(view: QPlainTextEdit) -> None:
        clipboard = QApplication.clipboard()
        if clipboard is not None:
            clipboard.setText(view.toPlainText())

    def _tick(self) -> None:
        if self._tag_state != 'idle':
            try:
                self._tag_step()
            except Exception as exc:
                self._finish_tag(f'Tag measurement failed: {exc}', 'error')
        self._odom_refresh_counter += 1
        if self._odom_refresh_counter >= 4:
            self._odom_refresh_counter = 0
            self._refresh_odometry()

    # ------------------------------------------------------------------
    # Tag-to-tag relative pose
    # ------------------------------------------------------------------
    def _start_tag_measurement(self) -> None:
        if self._tag_state != 'idle':
            return
        camera = str(self.camera_combo.currentData() or '')
        if camera not in self.cameras:
            self._log('error', f'camera {camera!r} is not available')
            return
        try:
            tag1 = parse_tag_name(self.tag1_edit.text())
            tag2 = parse_tag_name(self.tag2_edit.text())
        except ValueError as exc:
            self.tag_status_label.setText(str(exc))
            self._log('warning', str(exc))
            return
        if tag1 == tag2:
            self.tag_status_label.setText('Tag 1 and Tag 2 must differ.')
            return
        if self.vision is not None:
            try:
                self.vision.hold(VISION_OWNER, camera)
            except Exception as exc:
                self.tag_status_label.setText(str(exc))
                self._log('error', f'tag detection not started: {exc}')
                return
        self._tag_request = (camera, tag1, tag2)
        self._tag_state = 'starting'
        self._tag_deadline = time.monotonic() + VISION_START_TIMEOUT_SEC
        self._tag_pairs = []
        self._tag_seen = set()
        self.tag_measure_button.setEnabled(False)
        self.tag_cancel_button.setEnabled(True)
        self.tag_status_label.setText(f'Starting tag detection on {camera}...')
        self._log('info', f'measuring {tag2} relative to {tag1} on {camera}')

    def _tag_step(self) -> None:
        camera, tag1, tag2 = self._tag_request
        client = self.cameras[camera]
        now = time.monotonic()
        if self._tag_state == 'starting':
            ready, detail = (
                self.vision.ready(camera) if self.vision is not None
                else (True, '')
            )
            if not ready:
                if now > self._tag_deadline:
                    raise RuntimeError(
                        f'tag detection did not start: {detail}'
                    )
                self.tag_status_label.setText(detail)
                return
            self._tag_state = 'sampling'
            self._tag_marker = client.capture_marker()
            self._tag_deadline = now + TAG_SAMPLING_TIMEOUT_SEC
            self.tag_status_label.setText('Sampling...')
            return

        first = client.get_observation(tag1, newer_than_ns=self._tag_marker)
        second = client.get_observation(tag2, newer_than_ns=self._tag_marker)
        if first is not None and second is not None:
            key = (first.stamp_ns, second.stamp_ns)
            if (
                key not in self._tag_seen
                and abs(first.stamp_ns - second.stamp_ns)
                <= TAG_PAIR_MAX_SKEW_NS
            ):
                self._tag_seen.add(key)
                self._tag_pairs.append((first, second))
        count = len(self._tag_pairs)
        self.tag_status_label.setText(
            f'Sampling... {count}/{TAG_SAMPLE_COUNT}'
        )
        if count >= TAG_SAMPLE_COUNT:
            self._complete_tag()
        elif now > self._tag_deadline:
            if count >= TAG_MIN_SAMPLES:
                self._complete_tag()
                return
            missing = [
                tag for tag, obs in ((tag1, first), (tag2, second))
                if obs is None
            ]
            reasons = '; '.join(
                f'{tag}: {client.unavailable_reason(tag) or "not detected"}'
                for tag in missing
            )
            raise ObservationUnavailable(
                f'only {count} paired samples in '
                f'{TAG_SAMPLING_TIMEOUT_SEC:.0f} s'
                + (f' ({reasons})' if reasons else '')
            )

    def _complete_tag(self) -> None:
        camera, _tag1, _tag2 = self._tag_request
        result = average_relative_pose(
            self._tag_pairs,
            camera=camera,
            outlier_count=TAG_OUTLIER_COUNT,
        )
        text = format_tag_result(result)
        self.tag_result_view.setPlainText(text)
        self._history(text)
        self._save_tag(result)
        self._finish_tag(
            f'Done: |d| = {result.distance:.4f} m '
            f'({result.sample_count} samples)',
            'info',
        )
        self._log('info', text.replace('\n', ' |'))

    def _finish_tag(self, message: str, level: str) -> None:
        if self.vision is not None:
            try:
                self.vision.release_owner(VISION_OWNER)
            except Exception as exc:
                self._log('warning', f'tag detection release: {exc}')
        self._tag_state = 'idle'
        self._tag_request = None
        self.tag_measure_button.setEnabled(True)
        self.tag_cancel_button.setEnabled(False)
        self.tag_status_label.setText(message)
        if level != 'info':
            self._log(level, message)

    def _save_tag(self, result: TagRelativeResult) -> None:
        try:
            path = _append_csv(
                TAG_CSV,
                ['time', 'camera', 'tag1', 'tag2', 'x_m', 'y_m', 'z_m',
                 'roll_deg', 'pitch_deg', 'yaw_deg', 'distance_m',
                 'std_x_m', 'std_y_m', 'std_z_m', 'samples'],
                [datetime.now().isoformat(timespec='seconds'),
                 result.camera, result.tag1, result.tag2,
                 *(f'{v:.5f}' for v in result.xyz),
                 *(f'{v:.3f}' for v in result.rpy_deg),
                 f'{result.distance:.5f}',
                 *(f'{v:.5f}' for v in result.xyz_std),
                 result.sample_count],
            )
        except OSError as exc:
            self._log('warning', f'CSV not written: {exc}')
            return
        self._log('info', f'saved to {path}')

    # ------------------------------------------------------------------
    # Base odometry delta
    # ------------------------------------------------------------------
    def _odom_fresh(self) -> bool:
        received = getattr(self.odometry, 'latest_received', None)
        return (
            received is not None
            and time.monotonic() - received <= ODOM_STALE_SEC
        )

    def _update_odom_buttons(self) -> None:
        active = bool(self.odometry is not None and self.odometry.active)
        self.odom_start_button.setEnabled(
            self.odometry is not None and not active
        )
        self.odom_finish_button.setEnabled(active)
        self.odom_cancel_button.setEnabled(active)

    def _refresh_odometry(self) -> None:
        if self.odometry is None:
            self.odom_status_label.setText('Base odometry is unavailable.')
            return
        pose = self.odometry.latest_pose
        if pose is None:
            self.odom_status_label.setText('Waiting for /rby1/odom ...')
        else:
            state = 'live' if self._odom_fresh() else 'STALE'
            self.odom_status_label.setText(
                f'/rby1/odom {state}: x={pose[0]:+.3f} m, '
                f'y={pose[1]:+.3f} m, yaw={math.degrees(pose[2]):+.1f} deg'
            )
        if self.odometry.active:
            delta = self.odometry.current()
            self.odom_live_label.setText(
                f'Measuring: xy = [{delta.x:+.4f}, {delta.y:+.4f}] m'
            )
        self._update_odom_buttons()

    def _start_odometry(self) -> None:
        if self.odometry is None:
            return
        if not self._odom_fresh():
            self._log('error', 'odometry is stale; measurement not started')
            self.odom_live_label.setText(
                'Cannot start: /rby1/odom is not being received.'
            )
            return
        try:
            self.odometry.start()
        except RuntimeError as exc:
            self._log('error', str(exc))
            return
        self.odom_result_view.clear()
        self.odom_live_label.setText('Measuring...')
        self._log('info', 'base odometry measurement started')
        self._update_odom_buttons()

    def _finish_odometry(self) -> None:
        if self.odometry is None or not self.odometry.active:
            return
        stale = not self._odom_fresh()
        delta = self.odometry.finish()
        text = format_odometry_delta(delta)
        if stale:
            text += '\nWARNING: odometry was stale at Finish.'
        note = self.odom_note_edit.text().strip()
        title = 'Base odometry delta' + (f' [{note}]' if note else '')
        self.odom_result_view.setPlainText(text)
        self.odom_live_label.setText('Finished.')
        self._history(f'{title}\n{text}')
        self._log('info', f'{title}: ' + text.replace('\n', ' | '))
        self._save_odometry(delta, note)
        self._update_odom_buttons()

    def _cancel_odometry(self) -> None:
        if self.odometry is None:
            return
        self.odometry.cancel()
        self.odom_live_label.setText('Cancelled.')
        self._log('info', 'base odometry measurement cancelled')
        self._update_odom_buttons()

    def _save_odometry(self, delta: OdometryDelta, note: str) -> None:
        start = delta.start_pose or (math.nan,) * 3
        end = delta.end_pose or (math.nan,) * 3
        try:
            path = _append_csv(
                ODOM_CSV,
                ['time', 'note', 'dx_start_m', 'dy_start_m', 'dyaw_deg',
                 'dx_odom_m', 'dy_odom_m', 'planar_distance_m',
                 'path_length_m', 'duration_s',
                 'start_x', 'start_y', 'start_yaw_deg',
                 'end_x', 'end_y', 'end_yaw_deg',
                 'samples', 'skipped_jumps'],
                [datetime.now().isoformat(timespec='seconds'), note,
                 f'{delta.x:.5f}', f'{delta.y:.5f}',
                 f'{math.degrees(delta.yaw):.3f}',
                 f'{delta.odom_dx:.5f}', f'{delta.odom_dy:.5f}',
                 f'{delta.planar_distance:.5f}',
                 f'{delta.path_length:.4f}', f'{delta.duration_sec:.2f}',
                 f'{start[0]:.5f}', f'{start[1]:.5f}',
                 f'{math.degrees(start[2]):.3f}',
                 f'{end[0]:.5f}', f'{end[1]:.5f}',
                 f'{math.degrees(end[2]):.3f}',
                 delta.sample_count, delta.skipped_jumps],
            )
        except OSError as exc:
            self._log('warning', f'CSV not written: {exc}')
            return
        self._log('info', f'saved to {path}')

    def close_panel(self) -> None:
        self.timer.stop()
        if self._tag_state != 'idle':
            self._finish_tag('closed', 'info')


__all__ = ['MeasurementsPanel']
