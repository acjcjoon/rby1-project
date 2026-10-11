"""Live scheduler controls and live/simulation Gantt timelines."""
from __future__ import annotations
from typing import Any, Callable
from .gantt_chart import GanttChart
from .qt import (
    QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QTabWidget,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

class SchedulingTab(QWidget):
    def __init__(self, send_command: Callable[[dict[str, Any]], None]) -> None:
        super().__init__()
        self.send_command = send_command
        self.status = QLabel('Scheduler \uc5f0\uacb0 \ub300\uae30 \uc911')
        self.robot = QLabel('Robot: UNKNOWN')
        self.sim_button = QPushButton('Sim')
        self.play_button = QPushButton('Play')
        self.pause_button = QPushButton('Pause / Cancel (Space)')
        self.reload_button = QPushButton('JSON / lab.yaml \ub2e4\uc2dc \uc77d\uae30')
        self.sim_button.setToolTip('\uc2e4\uc81c Planner\uc640 \uc11c\ubc84 \uc0c1\ud0dc\ub97c \ubc14\uafb8\uc9c0 \uc54a\uace0 \uac00\uc0c1 \uc2dc\uac04\uc73c\ub85c \uacc4\uc0b0\ud569\ub2c8\ub2e4.')
        self.play_button.setToolTip('\ub77c\uc774\ube0c \uc791\uc5c5 \uc2e4\ud589\uc744 \ud5c8\uc6a9\ud569\ub2c8\ub2e4.')
        self.pause_button.setToolTip(
            '\uc0c8 \uc791\uc5c5 \uc120\ud0dd\uc744 \uba48\ucd94\uace0 \ud604\uc7ac \uc2e4\ud589 \uc911\uc778 Planner \uc791\uc5c5\uc744 cancel\ud569\ub2c8\ub2e4.'
        )
        self.devices = QTableWidget(0, 3)
        self.devices.setHorizontalHeaderLabels(['\uc7a5\ube44/\uc704\uce58', '\uc0ac\uc6a9', '\uc6a9\ub7c9'])
        self.batches = QTableWidget(0, 4)
        self.batches.setHorizontalHeaderLabels(['Batch', '\ubab0\ube44', 'Plate \uc218', '\uc0c1\ud0dc'])
        self.timeline_tabs = QTabWidget()
        self.live_status = QLabel('Live: \uc2e4\ud589 \uae30\ub85d \uc5c6\uc74c')
        self.live_gantt = GanttChart(
            empty_message='Play\ub97c \uc2e4\ud589\ud558\uba74 \uc2e4\uc81c \uc791\uc5c5 \ud0c0\uc784\ub77c\uc778\uc774 \ud45c\uc2dc\ub429\ub2c8\ub2e4.',
            minimum_span_min=1.0,
            accessible_name='\ub77c\uc774\ube0c \uc791\uc5c5 \uac04\ud2b8 \ucc28\ud2b8',
        )
        live_page = QWidget()
        live_layout = QVBoxLayout(live_page)
        live_layout.addWidget(self.live_status)
        live_layout.addWidget(self.live_gantt)
        self.simulation_status = QLabel('Simulation: \uc2e4\ud589 \uc804')
        self.simulation_gantt = GanttChart(
            accessible_name='\uc2dc\ubbac\ub808\uc774\uc158 \uc7a5\ube44 \uc810\uc720 \uac04\ud2b8 \ucc28\ud2b8',
        )
        simulation_page = QWidget()
        simulation_layout = QVBoxLayout(simulation_page)
        simulation_layout.addWidget(self.simulation_status)
        simulation_layout.addWidget(self.simulation_gantt)
        self.timeline_tabs.addTab(live_page, '\ub77c\uc774\ube0c \ud0c0\uc784\ub77c\uc778')
        self.timeline_tabs.addTab(simulation_page, '\uc2dc\ubbac\ub808\uc774\uc158 \uc608\uc0c1')
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        buttons = QHBoxLayout()
        for button in (self.sim_button, self.play_button, self.pause_button, self.reload_button):
            buttons.addWidget(button)
        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.robot)
        layout.addLayout(buttons)
        layout.addWidget(self.timeline_tabs, 3)
        lower = QHBoxLayout()
        lower.addWidget(self.devices)
        lower.addWidget(self.batches)
        layout.addLayout(lower, 1)
        layout.addWidget(QLabel('\ucd5c\uadfc \uc774\ubca4\ud2b8'))
        layout.addWidget(self.log, 1)
        self.sim_button.clicked.connect(self._simulate)
        self.play_button.clicked.connect(self._play)
        self.pause_button.clicked.connect(lambda: self.send_command({'cmd': 'pause'}))
        self.reload_button.clicked.connect(lambda: self.send_command({'cmd': 'reload'}))

    def update_state(self, state: dict[str, Any]) -> None:
        scheduler = state.get('scheduler', {})
        robot = state.get('robot', {})
        mode = str(state.get('planner_mode', 'unknown')).upper()
        self.status.setText(
            f"Scheduler: {str(scheduler.get('status', 'unknown')).upper()} | "
            f"Planner: {mode} | revision: {state.get('input_revision', '-')}"
        )
        self.robot.setText(
            f"Robot: {str(robot.get('status', 'unknown')).upper()} | "
            f"Task: {robot.get('task_id') or '-'} | Phase: {robot.get('phase') or '-'} | "
            f"Progress: {float(robot.get('progress', 0.0)) * 100:.0f}%"
        )
        devices = state.get('devices', [])
        self.devices.setRowCount(len(devices))
        for row, device in enumerate(devices):
            for column, value in enumerate((device.get('id'), device.get('used'), device.get('capacity'))):
                self.devices.setItem(row, column, QTableWidgetItem(str(value)))
        batches = state.get('batches', [])
        plates = state.get('plates', [])
        self.batches.setRowCount(len(batches))
        for row, batch in enumerate(batches):
            batch_plates = [plate for plate in plates if plate['batch_id'] == batch['batch_id']]
            ratio = batch.get('molar_ratio', {}).get('display', '')
            values = (batch['batch_id'], ratio, len(batch_plates), batch.get('status', ''))
            for column, value in enumerate(values):
                self.batches.setItem(row, column, QTableWidgetItem(str(value)))
        live_resources = ['ROBOT'] + [
            str(device.get('id', '')) for device in devices
            if str(device.get('id', ''))
        ]
        self._update_live_timeline(
            state.get('live_timeline', {}),
            live_resources,
        )
        self._update_simulation(state.get('simulation', {}))
        for table in (self.devices, self.batches):
            table.resizeColumnsToContents()

    def _update_live_timeline(
        self,
        live: dict[str, Any],
        resources: list[str],
    ) -> None:
        timeline = list(live.get('timeline', []))
        elapsed = float(live.get('elapsed_min', 0.0))
        status = str(live.get('status', 'idle')).upper()
        running = sum(
            str(item.get('status', '')).lower() == 'running'
            for item in timeline
        )
        self.live_status.setText(
            f'Live: {status} | \uacbd\uacfc {self._format_elapsed(elapsed)} | '
            f'\uc791\uc5c5 {len(timeline)}\uac74 | \uc2e4\ud589 \uc911 {running}\uac74'
        )
        self.live_gantt.set_timeline(
            timeline,
            end_min=elapsed,
            resources=resources,
        )

    def _update_simulation(self, simulation: dict[str, Any]) -> None:
        if not simulation:
            self.simulation_status.setText('Simulation: \uc2e4\ud589 \uc804')
            self.simulation_gantt.set_timeline([])
            return
        status = str(simulation.get('status', 'unknown')).upper()
        end_min = float(simulation.get('end_min', 0.0))
        pending = len(simulation.get('pending', []))
        self.simulation_status.setText(
            f"Simulation: {status} | \uc608\uc0c1 \uc885\ub8cc {end_min:.0f} min ({end_min / 60:.1f} h) | "
            f"\uac00\uc0c1 \uc2e4\ud589 {simulation.get('simulated_count', 0)}\uac74 | \ubbf8\uc644\ub8cc {pending}\uac74"
        )
        self.simulation_gantt.set_timeline(
            list(simulation.get('timeline', [])),
            end_min=end_min,
        )

    def append_events(self, events: list[dict[str, Any]]) -> None:
        for event in events:
            detail = event.get('task_id') or event.get('message') or ''
            self.log.appendPlainText(f"[{event.get('timestamp', '')}] {event.get('type', '')} {detail}".rstrip())

    def _play(self) -> None:
        self.timeline_tabs.setCurrentIndex(0)
        self.send_command({'cmd': 'play'})

    def _simulate(self) -> None:
        self.timeline_tabs.setCurrentIndex(1)
        self.send_command({'cmd': 'simulate'})

    @staticmethod
    def _format_elapsed(value_min: float) -> str:
        total_seconds = max(0, round(value_min * 60.0))
        minutes, seconds = divmod(total_seconds, 60)
        hours, minutes = divmod(minutes, 60)
        return f'{hours:02d}:{minutes:02d}:{seconds:02d}'
