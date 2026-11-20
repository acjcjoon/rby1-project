"""VSLAM tab: cuVSLAM stack control, localization, map and point storage.

Base driving stays on the Base tab; this panel never publishes velocity. It
only stops the base before a station correction.
"""
from __future__ import annotations

import math
from typing import Callable, Optional

from .qt_compat import (
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QPlainTextEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTimer,
    QVBoxLayout,
    QWidget,
    Qt,
    enum_value,
)
from .scenario_ui import ScenarioPanel


class VslamPanel(QWidget):
    REFRESH_MS = 200

    def __init__(
        self,
        vslam,
        *,
        on_log: Callable[[str, str], None],
        stop_base: Callable[[], None],
        runner=None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.vslam = vslam
        self.on_log = on_log
        self.stop_base = stop_base
        self.runner = runner
        self._waypoints: list = []
        self._last_job = None
        self._build_ui()
        if self.vslam is None:
            self._set_unavailable()
            return
        self._reload_maps()
        self._reload_points()
        self.timer = QTimer(self)
        self.timer.setInterval(self.REFRESH_MS)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)
        left = QVBoxLayout()
        right = QVBoxLayout()
        root.addLayout(left, 3)
        root.addLayout(right, 2)

        stack = QGroupBox('cuVSLAM Stack (Isaac container)')
        stack_layout = QGridLayout(stack)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem('Mapping (new map)', 'mapping')
        self.mode_combo.addItem('Localization (saved map)', 'localization')
        self.mode_combo.currentIndexChanged.connect(self._mode_changed)
        self.map_combo = QComboBox()
        self.map_reload_button = QPushButton('Reload Maps')
        self.map_reload_button.clicked.connect(self._reload_maps)
        self.start_button = QPushButton('Start VSLAM')
        self.start_button.clicked.connect(self._start)
        self.stop_button = QPushButton('Stop VSLAM')
        self.stop_button.clicked.connect(self._stop)
        self.process_label = QLabel('Stack: stopped')
        stack_layout.addWidget(QLabel('Mode'), 0, 0)
        stack_layout.addWidget(self.mode_combo, 0, 1, 1, 2)
        stack_layout.addWidget(QLabel('Saved map'), 1, 0)
        stack_layout.addWidget(self.map_combo, 1, 1)
        stack_layout.addWidget(self.map_reload_button, 1, 2)
        stack_layout.addWidget(self.start_button, 2, 1)
        stack_layout.addWidget(self.stop_button, 2, 2)
        stack_layout.addWidget(self.process_label, 3, 0, 1, 3)
        left.addWidget(stack)

        status = QGroupBox('Status')
        status_layout = QVBoxLayout(status)
        self.tracking_label = QLabel('VO tracking: waiting')
        self.adapter_label = QLabel('Base conversion: waiting')
        self.localization_label = QLabel('Localization: waiting')
        self.pose_label = QLabel('Pose (map -> base): -')
        for label in (
            self.tracking_label,
            self.adapter_label,
            self.localization_label,
            self.pose_label,
        ):
            label.setWordWrap(True)
            status_layout.addWidget(label)
        left.addWidget(status)

        poses = QGroupBox('Live poses - XYZ [m], roll / pitch / yaw [deg]')
        poses_layout = QVBoxLayout(poses)
        self.pose_table = QTableWidget(3, 8)
        self.pose_table.setHorizontalHeaderLabels([
            'Source / frames', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw',
            'Freshness',
        ])
        self.pose_table.horizontalHeader().setStretchLastSection(True)
        self.pose_table.verticalHeader().setVisible(False)
        self.pose_table.setColumnWidth(0, 215)
        for column in range(1, 7):
            self.pose_table.setColumnWidth(column, 60)
        # Three fixed rows: no scroll bars (they would stay light-coloured
        # and cover the last row).
        self.pose_table.setVerticalScrollBarPolicy(
            enum_value(Qt, 'ScrollBarPolicy', 'ScrollBarAlwaysOff'))
        self.pose_table.setHorizontalScrollBarPolicy(
            enum_value(Qt, 'ScrollBarPolicy', 'ScrollBarAlwaysOff'))
        self.pose_table.verticalHeader().setDefaultSectionSize(28)
        self.pose_table.setFixedHeight(118)
        # Same dark palette as the list and log widgets of the main window.
        self.pose_table.setStyleSheet(
            """
            QTableWidget {
                background: #171a1f;
                alternate-background-color: #171a1f;
                color: #e8edf2;
                border: 1px solid #4b5563;
                border-radius: 4px;
                gridline-color: #374151;
            }
            QTableWidget::item { background: #171a1f; color: #e8edf2; }
            QTableWidget::item:selected {
                background: #2c6e9b;
                color: #ffffff;
            }
            QHeaderView::section {
                background: #232830;
                color: #e8edf2;
                border: 0px;
                border-right: 1px solid #374151;
                border-bottom: 1px solid #4b5563;
                padding: 3px;
            }
            QTableCornerButton::section { background: #232830; border: 0px; }
            """
        )
        poses_layout.addWidget(self.pose_table)
        left.addWidget(poses)

        station = QGroupBox('Localization')
        station_layout = QGridLayout(station)
        self.correct_button = QPushButton('Stop + correct at this point')
        self.correct_button.setToolTip(
            'Stops the base, then waits for a converged visual correction. '
            'Keep the base and head still.'
        )
        self.correct_button.clicked.connect(self._correct)
        self.reset_button = QPushButton('Recover restarted VO reference')
        self.reset_button.clicked.connect(self._reset_visual)
        self.reset_button.setEnabled(False)
        self.correction_label = QLabel('Correction: -')
        self.correction_label.setWordWrap(True)
        station_layout.addWidget(self.correct_button, 0, 0)
        station_layout.addWidget(self.reset_button, 0, 1)
        station_layout.addWidget(self.correction_label, 1, 0, 1, 2)

        self.hint_spins = {}
        hint_row = QHBoxLayout()
        for name, suffix, value, low, high in (
            ('x', ' m', 0.0, -100.0, 100.0),
            ('y', ' m', 0.0, -100.0, 100.0),
            ('z', ' m', 1.5, -5.0, 5.0),
            ('yaw', ' deg', 0.0, -180.0, 180.0),
        ):
            spin = QDoubleSpinBox()
            spin.setRange(low, high)
            spin.setDecimals(3 if name != 'yaw' else 1)
            spin.setSingleStep(0.05 if name != 'yaw' else 5.0)
            spin.setValue(value)
            spin.setPrefix(name + ' ')
            spin.setSuffix(suffix)
            self.hint_spins[name] = spin
            hint_row.addWidget(spin)
        self.hint_from_point_button = QPushButton('Hint <- selected point')
        self.hint_from_point_button.setToolTip(
            'Copies x / y / yaw of the selected point. The hint is the pose '
            'of d435_link in the map; z is the camera height.'
        )
        self.hint_from_point_button.clicked.connect(self._hint_from_point)
        self.localize_button = QPushButton('Localize in saved map')
        self.localize_button.clicked.connect(self._localize)
        station_layout.addWidget(
            QLabel('Pose hint (d435_link in map):'), 2, 0, 1, 2
        )
        station_layout.addLayout(hint_row, 3, 0, 1, 2)
        station_layout.addWidget(self.hint_from_point_button, 4, 0)
        station_layout.addWidget(self.localize_button, 4, 1)
        left.addWidget(station)

        map_group = QGroupBox('Map')
        map_layout = QHBoxLayout(map_group)
        self.map_name_edit = QLineEdit()
        self.map_name_edit.setPlaceholderText('new map name (e.g. lab_01)')
        self.save_map_button = QPushButton('Save Map')
        self.save_map_button.clicked.connect(self._save_map)
        self.job_label = QLabel('-')
        map_layout.addWidget(self.map_name_edit, 2)
        map_layout.addWidget(self.save_map_button)
        map_layout.addWidget(self.job_label, 3)
        left.addWidget(map_group)
        left.addStretch(1)

        points = QGroupBox('Points (map frame)')
        points_layout = QVBoxLayout(points)
        name_row = QHBoxLayout()
        self.point_name_edit = QLineEdit()
        self.save_point_button = QPushButton('Save Point')
        self.save_point_button.setToolTip(
            'Saves the current corrected pose. Requires a completed '
            '"Stop + correct" at this position.'
        )
        self.save_point_button.clicked.connect(self._save_point)
        name_row.addWidget(QLabel('Name'))
        name_row.addWidget(self.point_name_edit, 1)
        name_row.addWidget(self.save_point_button)
        points_layout.addLayout(name_row)
        self.point_list = QListWidget()
        points_layout.addWidget(self.point_list, 1)
        point_buttons = QHBoxLayout()
        self.delete_point_button = QPushButton('Delete Selected')
        self.delete_point_button.clicked.connect(self._delete_point)
        self.reload_points_button = QPushButton('Reload')
        self.reload_points_button.clicked.connect(self._reload_points)
        point_buttons.addWidget(self.delete_point_button)
        point_buttons.addWidget(self.reload_points_button)
        points_layout.addLayout(point_buttons)
        self.points_file_label = QLabel('')
        self.points_file_label.setObjectName('secondaryNote')
        self.points_file_label.setWordWrap(True)
        points_layout.addWidget(self.points_file_label)
        right.addWidget(points, 3)

        navigation = QGroupBox('Navigation (Nav2, no obstacle sensing)')
        navigation_layout = QGridLayout(navigation)
        self.nav_start_button = QPushButton('Start Nav2')
        self.nav_start_button.clicked.connect(self._start_navigation)
        self.nav_stop_button = QPushButton('Stop Nav2')
        self.nav_stop_button.clicked.connect(self._stop_navigation)
        self.goto_button = QPushButton(
            'Goto selected point -> stop -> VO correction'
        )
        self.goto_button.setMinimumHeight(34)
        self.goto_button.setToolTip(
            'Nav2 drives to the selected point with wheel-odometry '
            'localization, then stops and requests a station correction. '
            'There is no obstacle sensing: watch the robot.'
        )
        self.goto_button.clicked.connect(self._goto)
        self.goto_stop_button = QPushButton('STOP Goto')
        self.goto_stop_button.setMinimumHeight(34)
        self.goto_stop_button.setStyleSheet(
            'background: #b00020; color: white; font-weight: bold;'
        )
        self.goto_stop_button.clicked.connect(self._stop_goto)
        self.nav_label = QLabel('Nav2: stopped')
        self.nav_label.setWordWrap(True)
        self.goto_label = QLabel('Goto: -')
        self.goto_label.setWordWrap(True)
        navigation_layout.addWidget(self.nav_start_button, 0, 0)
        navigation_layout.addWidget(self.nav_stop_button, 0, 1)
        navigation_layout.addWidget(self.goto_button, 1, 0)
        navigation_layout.addWidget(self.goto_stop_button, 1, 1)
        navigation_layout.addWidget(self.nav_label, 2, 0, 1, 2)
        navigation_layout.addWidget(self.goto_label, 3, 0, 1, 2)
        right.addWidget(navigation)

        self.finish_button = QPushButton('Finish mapping (continue Task)')
        self.finish_button.setMinimumHeight(36)
        self.finish_button.setToolTip(
            'Ends the operator step of the offline_map_creation Task. '
            'The Task then saves the map.'
        )
        self.finish_button.setEnabled(False)
        self.finish_button.clicked.connect(self._finish_operator_step)
        right.addWidget(self.finish_button)

        log_group = QGroupBox('VSLAM Log')
        log_layout = QVBoxLayout(log_group)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(600)
        log_layout.addWidget(self.log_view)
        right.addWidget(log_group, 2)

        # Same readable dark-theme popup as the Scenario combos.
        for combo in (self.mode_combo, self.map_combo):
            ScenarioPanel._style_combo_popup(combo)
        self._mode_changed()

    def _set_unavailable(self) -> None:
        for child in self.findChildren(QPushButton):
            child.setEnabled(False)
        self.process_label.setText(
            'VSLAM client is unavailable (rby1_vslam not built or '
            'vslam_enabled is false).'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _log(self, level: str, message: str) -> None:
        prefix = {'error': '[ERROR] ', 'warning': '[WARN] '}.get(level, '')
        self.log_view.appendPlainText(prefix + message)
        self.on_log(level, 'VSLAM: ' + message)

    def _guard(self, label: str, action: Callable[[], object]):
        try:
            return action()
        except Exception as exc:
            self._log('error', f'{label} failed: {exc}')
            return None

    def _mode_changed(self, *_args) -> None:
        localization = self.mode_combo.currentData() == 'localization'
        self.map_combo.setEnabled(localization)

    def _selected_map(self) -> str:
        return str(self.map_combo.currentText()).strip()

    def _selected_point(self):
        row = self.point_list.currentRow()
        if 0 <= row < len(self._waypoints):
            return self._waypoints[row]
        return None

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------
    def _reload_maps(self) -> None:
        selected = self._selected_map()
        names = self._guard('Map list', self.vslam.list_maps) or []
        self.map_combo.clear()
        for name in names:
            self.map_combo.addItem(name)
        if selected in names:
            self.map_combo.setCurrentIndex(names.index(selected))

    def _start(self) -> None:
        mode = str(self.mode_combo.currentData())
        name = self._selected_map() if mode == 'localization' else ''
        self._guard('Start', lambda: self.vslam.start(mode, name))

    def _stop(self) -> None:
        if self.runner is not None and self.runner.active:
            self._log('warning', 'Stop the running Task before stopping VSLAM.')
            return
        self._guard('Stop', self.vslam.stop)

    def _correct(self) -> None:
        self._guard('Base stop', self.stop_base)
        self._guard('Station correction', self.vslam.request_station_correction)

    def _reset_visual(self) -> None:
        self._guard('VO recovery', self.vslam.reset_visual_reference)

    def _hint_from_point(self) -> None:
        point = self._selected_point()
        if point is None:
            self._log('warning', 'Select a point first.')
            return
        self.hint_spins['x'].setValue(point.x)
        self.hint_spins['y'].setValue(point.y)
        self.hint_spins['yaw'].setValue(math.degrees(point.yaw))

    def _localize(self) -> None:
        name = self._selected_map()
        if not name:
            self._log('warning', 'Select a saved map first.')
            return
        self._guard('Localize', lambda: self.vslam.localize(
            name,
            x=self.hint_spins['x'].value(),
            y=self.hint_spins['y'].value(),
            z=self.hint_spins['z'].value(),
            yaw=math.radians(self.hint_spins['yaw'].value()),
        ))

    def _save_map(self) -> None:
        name = self.map_name_edit.text().strip()
        if not name:
            self._log('warning', 'Enter a new map name.')
            return
        self._guard('Base stop', self.stop_base)
        self._guard('Save map', lambda: self.vslam.save_map(name))

    def _reload_points(self) -> None:
        try:
            self._waypoints = list(self.vslam.load_waypoints())
            path = self.vslam.waypoints_path()
            self.points_file_label.setText(str(path))
            self.point_name_edit.setText(self.vslam.next_point_name())
        except Exception as exc:
            self._waypoints = []
            self._log('error', f'Point file read failed: {exc}')
        self.point_list.clear()
        for item in self._waypoints:
            self.point_list.addItem(
                f'{item.name}:  x={item.x:.3f} m, y={item.y:.3f} m, '
                f'yaw={math.degrees(item.yaw):.1f} deg'
            )

    def _save_point(self) -> None:
        name = self.point_name_edit.text().strip()
        point = self._guard(
            'Save point', lambda: self.vslam.save_current_point(name)
        )
        if point is not None:
            self._log(
                'info',
                f'Saved point {point.name}: x={point.x:.3f}, y={point.y:.3f}, '
                f'yaw={math.degrees(point.yaw):.1f} deg',
            )
            self._reload_points()

    def _delete_point(self) -> None:
        point = self._selected_point()
        if point is None:
            self._log('warning', 'Select a point to delete.')
            return
        self._guard('Delete point', lambda: self.vslam.delete_point(point.name))
        self._reload_points()

    def _start_navigation(self) -> None:
        self._guard('Start Nav2', self.vslam.start_navigation)

    def _stop_navigation(self) -> None:
        self._guard('Base stop', self.stop_base)
        self._guard('Stop Nav2', self.vslam.stop_navigation)

    def _goto(self) -> None:
        if self.runner is not None and self.runner.active:
            self._log('warning', 'Goto is blocked while a Task is running.')
            return
        point = self._selected_point()
        if point is None:
            self._log('warning', 'Select a point for Goto.')
            return
        # Release any Base tab jog before Nav2 takes the base.
        self._guard('Base stop', self.stop_base)
        self._guard('Goto', lambda: self.vslam.goto(point))

    def _stop_goto(self) -> None:
        self._guard('Goto stop', self.vslam.stop_goto)
        self._guard('Base stop', self.stop_base)

    def _refresh_poses(self, rows) -> None:
        for row, pose in enumerate(rows):
            if not pose.values:
                columns = [pose.name] + ['-'] * 6 + ['Waiting']
            else:
                columns = [
                    f'{pose.name}: {pose.frame_id} -> {pose.child_frame_id}'
                ]
                columns += [f'{value:.4f}' for value in pose.values[:3]]
                columns += [
                    f'{math.degrees(value):.2f}' for value in pose.values[3:]
                ]
                columns.append(
                    ('LIVE' if pose.fresh else 'STALE')
                    + f' ({pose.age_sec:.1f}s)'
                )
            for column, text in enumerate(columns):
                item = self.pose_table.item(row, column)
                if item is None:
                    item = QTableWidgetItem(text)
                    self.pose_table.setItem(row, column, item)
                elif item.text() != text:
                    item.setText(text)

    def _finish_operator_step(self) -> None:
        if self.runner is None or not self.runner.finish_operator_step():
            self._log('warning', 'No operator mapping step is running.')
            return
        self._guard('Base stop', self.stop_base)
        self._log('info', 'Operator mapping finished; Task continues.')

    # ------------------------------------------------------------------
    # Periodic refresh
    # ------------------------------------------------------------------
    def refresh(self) -> None:
        for line in self.vslam.drain_log():
            self.log_view.appendPlainText(line)
        snapshot = self.vslam.snapshot()
        running = snapshot.process == 'running'
        task_active = self.runner is not None and self.runner.active
        operator_step = (
            self.runner is not None and self.runner.operator_step_active
        )

        detail = snapshot.process
        if snapshot.process != 'stopped' and snapshot.mode:
            detail += f' ({snapshot.mode}'
            detail += f', map {snapshot.map_name})' if snapshot.map_name else ')'
        self.process_label.setText('Stack: ' + detail)
        self.start_button.setEnabled(
            snapshot.process in ('stopped', 'exited') and not task_active
        )
        self.stop_button.setEnabled(running and not task_active)

        self.tracking_label.setText(
            'VO tracking: '
            + ('READY' if snapshot.tracking_ready else 'BLOCKED')
            + ' | ' + (snapshot.tracking_detail or '-')
        )
        self.adapter_label.setText('Base conversion: ' + snapshot.adapter_detail)
        state: dict = snapshot.localization
        if not snapshot.localization_fresh:
            health = 'UNAVAILABLE'
        elif state.get('initialized') is not True:
            health = 'NOT INITIALIZED'
        elif state.get('healthy') is not True:
            health = 'UNHEALTHY'
        else:
            health = 'HEALTHY'
        self.localization_label.setText(
            f"Localization: {health} | {state.get('mode', 'waiting')} | "
            f"corrections={state.get('correction_count', 0)} | "
            f"{state.get('detail', '')}"
        )
        if snapshot.pose is None:
            self.pose_label.setText('Pose (map -> base): -')
        else:
            x, y, yaw = snapshot.pose
            self.pose_label.setText(
                f'Pose (map -> base): x={x:.3f} m, y={y:.3f} m, '
                f'yaw={math.degrees(yaw):.1f} deg '
                + ('[LIVE]' if snapshot.pose_fresh else '[STALE]')
            )

        self.correct_button.setEnabled(
            running and (not task_active or operator_step)
            and not snapshot.correction.running
        )
        self.reset_button.setEnabled(
            running and state.get('visual_reference_changed') is True
        )
        self.correction_label.setText(
            f'Correction: {snapshot.correction.status}'
            + (f' | {snapshot.correction.message}'
               if snapshot.correction.message else '')
        )
        job_idle = not snapshot.job.running
        self.save_map_button.setEnabled(running and job_idle and not task_active)
        self.localize_button.setEnabled(running and job_idle and not task_active)
        self.job_label.setText(
            '-' if snapshot.job.status == 'idle'
            else f'{snapshot.job.label}: {snapshot.job.status}'
        )
        self.save_point_button.setEnabled(running)
        self.finish_button.setEnabled(operator_step)

        self._refresh_poses(snapshot.poses)
        nav_running = snapshot.navigation_process == 'running'
        goto_running = snapshot.goto.running
        gate: dict = snapshot.navigation or {}
        self.nav_start_button.setEnabled(
            snapshot.navigation_process in ('stopped', 'exited')
        )
        self.nav_stop_button.setEnabled(nav_running)
        self.goto_button.setEnabled(
            running and nav_running and not goto_running and not task_active
        )
        self.goto_stop_button.setEnabled(goto_running)
        self.nav_label.setText(
            f'Nav2: {snapshot.navigation_process} | gate '
            + ('ON' if gate.get('enabled') else 'OFF') + ' | '
            + ('fresh' if snapshot.navigation_fresh else 'no status') + ' | '
            + str(gate.get('detail', ''))
        )
        self.goto_label.setText(
            'Goto: -' if snapshot.goto.status == 'idle'
            else f'{snapshot.goto.label}: {snapshot.goto.status} | '
                 f'{snapshot.goto.message}'
        )
        # Corrections and manual saves must not race a Goto.
        if goto_running:
            self.correct_button.setEnabled(False)

        if snapshot.job != self._last_job:
            previous: Optional[object] = self._last_job
            self._last_job = snapshot.job
            if previous is not None and snapshot.job.status in (
                'succeeded', 'failed'
            ):
                self.on_log(
                    'info' if snapshot.job.status == 'succeeded' else 'error',
                    f'VSLAM: {snapshot.job.label}: {snapshot.job.message}',
                )
                self._reload_maps()

    def close(self) -> bool:
        timer = getattr(self, 'timer', None)
        if timer is not None:
            timer.stop()
        return super().close()


__all__ = ['VslamPanel']
