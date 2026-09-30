"""Live scheduling controls and side-effect-free simulation preview."""
from __future__ import annotations
from typing import Any, Callable
from .gantt_chart import GanttChart
from .qt import (
    QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

class SchedulingTab(QWidget):
    def __init__(self, send_command: Callable[[dict[str, Any]], None]) -> None:
        super().__init__()
        self.send_command = send_command
        self.status = QLabel('Scheduler \uc5f0\uacb0 \ub300\uae30 \uc911')
        self.robot = QLabel('Robot: UNKNOWN')
        self.sim_button = QPushButton('Sim')
        self.play_button = QPushButton('Play')
        self.pause_button = QPushButton('Pause')
        self.promote_button = QPushButton('\uc120\ud0dd \uc791\uc5c5 P0 \uc2b9\uaca9')
        self.retry_button = QPushButton('\uc120\ud0dd \uc791\uc5c5 \uc7ac\uc2dc\ub3c4')
        self.reload_button = QPushButton('JSON / lab.yaml \ub2e4\uc2dc \uc77d\uae30')
        self.sim_button.setToolTip('\uc2e4\uc81c Planner\uc640 \uc11c\ubc84 \uc0c1\ud0dc\ub97c \ubc14\uafb8\uc9c0 \uc54a\uace0 \uac00\uc0c1 \uc2dc\uac04\uc73c\ub85c \uacc4\uc0b0\ud569\ub2c8\ub2e4.')
        self.play_button.setToolTip('\ub77c\uc774\ube0c \uc791\uc5c5 \uc2e4\ud589\uc744 \ud5c8\uc6a9\ud569\ub2c8\ub2e4.')
        self.queue = QTableWidget(0, 6)
        self.queue.setHorizontalHeaderLabels(['\uc6b0\uc120\uc21c\uc704', 'Task', 'Plate', '\uacbd\ub85c', '\uc0c1\ud0dc', '\uc624\ub958'])
        self.devices = QTableWidget(0, 3)
        self.devices.setHorizontalHeaderLabels(['\uc7a5\ube44/\uc704\uce58', '\uc0ac\uc6a9', '\uc6a9\ub7c9'])
        self.batches = QTableWidget(0, 4)
        self.batches.setHorizontalHeaderLabels(['Batch', '\ubab0\ube44', 'Plate \uc218', '\uc0c1\ud0dc'])
        self.simulation_status = QLabel('Simulation: \uc2e4\ud589 \uc804')
        self.simulation_gantt = GanttChart()
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        buttons = QHBoxLayout()
        for button in (self.sim_button, self.play_button, self.pause_button, self.promote_button, self.retry_button, self.reload_button):
            buttons.addWidget(button)
        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.robot)
        layout.addLayout(buttons)
        layout.addWidget(QLabel('\ub77c\uc774\ube0c \uc791\uc5c5 \ud050'))
        layout.addWidget(self.queue, 2)
        lower = QHBoxLayout()
        lower.addWidget(self.devices)
        lower.addWidget(self.batches)
        layout.addLayout(lower, 1)
        layout.addWidget(self.simulation_status)
        layout.addWidget(self.simulation_gantt, 2)
        layout.addWidget(QLabel('\ucd5c\uadfc \uc774\ubca4\ud2b8'))
        layout.addWidget(self.log, 1)
        self.sim_button.clicked.connect(lambda: self.send_command({'cmd': 'simulate'}))
        self.play_button.clicked.connect(lambda: self.send_command({'cmd': 'play'}))
        self.pause_button.clicked.connect(lambda: self.send_command({'cmd': 'pause'}))
        self.reload_button.clicked.connect(lambda: self.send_command({'cmd': 'reload'}))
        self.promote_button.clicked.connect(lambda: self._selected_command('promote'))
        self.retry_button.clicked.connect(lambda: self._selected_command('retry'))

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
        plate_by_key = {(plate['batch_id'], plate['role']): plate['plate_id'] for plate in state.get('plates', [])}
        rows = state.get('queue', [])
        self.queue.setRowCount(len(rows))
        for row, task in enumerate(rows):
            values = [
                task.get('priority', ''), task.get('task_id', ''),
                plate_by_key.get((task.get('batch_id'), task.get('role')), ''),
                f"{task.get('source', '')} -> {task.get('destination', '')}",
                task.get('status', ''), task.get('error', ''),
            ]
            for column, value in enumerate(values):
                self.queue.setItem(row, column, QTableWidgetItem(str(value)))
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
        self._update_simulation(state.get('simulation', {}))
        for table in (self.queue, self.devices, self.batches):
            table.resizeColumnsToContents()

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

    def _selected_command(self, command: str) -> None:
        row = self.queue.currentRow()
        item = self.queue.item(row, 1) if row >= 0 else None
        if item is not None:
            self.send_command({'cmd': command, 'task_id': item.text(), 'reason': 'PyQt operator'})
