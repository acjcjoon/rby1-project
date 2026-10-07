"""Temporary manual pick/place panel for Planner communication testing."""
from __future__ import annotations
from typing import Any, Callable
from .qt import (
    QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPlainTextEdit,
    QProgressBar, QPushButton, QVBoxLayout, QWidget,
)

LOCATIONS = [
    ('INBOX', 'INBOX'), ('STORE', 'STORE'),
    ('LH1', 'LH1'), ('LH2', 'LH2'),
]

class CommunicationTestTab(QWidget):
    def __init__(self, send_command: Callable[[dict[str, Any]], None]) -> None:
        super().__init__()
        self.send_command = send_command
        self.pick = QComboBox()
        self.place = QComboBox()
        for label, value in LOCATIONS:
            self.pick.addItem(label, value)
            self.place.addItem(label, value)
        self.play = QPushButton('Play')
        self.pause = QPushButton('Pause / Cancel (Space)')
        self.pause.setToolTip(
            '\uc2a4\ucf00\uc904\ub9c1\uc744 \uc77c\uc2dc\uc815\uc9c0\ud558\uace0 \ud604\uc7ac \ud14c\uc2a4\ud2b8 \uc791\uc5c5\uc5d0 cancel \uc694\uccad\uc744 \ubcf4\ub0c5\ub2c8\ub2e4.'
        )
        selectors = QGridLayout()
        selectors.addWidget(QLabel('Pick'), 0, 0)
        selectors.addWidget(QLabel('Place'), 0, 1)
        selectors.addWidget(self.pick, 1, 0)
        selectors.addWidget(self.place, 1, 1)
        controls = QHBoxLayout()
        controls.addWidget(self.play)
        controls.addWidget(self.pause)
        left_box = QGroupBox('Planner \ud1b5\uc2e0 \ud14c\uc2a4\ud2b8')
        left = QVBoxLayout(left_box)
        left.addLayout(selectors)
        left.addLayout(controls)
        left.addWidget(QLabel('Play \uc2dc \uc790\ub3d9 \uc2a4\ucf00\uc904\ub9c1\uc740 \uc77c\uc2dc\uc815\uc9c0\ub429\ub2c8\ub2e4.'))
        left.addStretch(1)
        self.robot_status = QLabel('UNKNOWN')
        self.mission_id = QLabel('-')
        self.task_id = QLabel('-')
        self.phase = QLabel('-')
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.message = QLabel('-')
        self.last_result = QPlainTextEdit()
        self.last_result.setReadOnly(True)
        state_grid = QGridLayout()
        labels = [
            ('Robot state', self.robot_status), ('Mission ID', self.mission_id),
            ('Task', self.task_id), ('Phase', self.phase), ('Message', self.message),
        ]
        for row, (name, widget) in enumerate(labels):
            state_grid.addWidget(QLabel(name), row, 0)
            state_grid.addWidget(widget, row, 1)
        state_grid.addWidget(QLabel('Progress'), len(labels), 0)
        state_grid.addWidget(self.progress, len(labels), 1)
        right_box = QGroupBox('Robot / Planner State')
        right = QVBoxLayout(right_box)
        right.addLayout(state_grid)
        right.addWidget(QLabel('\ub9c8\uc9c0\ub9c9 \uacb0\uacfc'))
        right.addWidget(self.last_result)
        layout = QHBoxLayout(self)
        layout.addWidget(left_box, 1)
        layout.addWidget(right_box, 2)
        self.play.clicked.connect(self._play)
        self.pause.clicked.connect(lambda: self.send_command({'cmd': 'pause'}))

    def update_state(self, state: dict[str, Any]) -> None:
        robot = state.get('robot', {})
        self.robot_status.setText(str(robot.get('status', 'unknown')).upper())
        self.mission_id.setText(str(robot.get('mission_id') or '-'))
        self.task_id.setText(str(robot.get('task_id') or '-'))
        self.phase.setText(str(robot.get('phase') or '-'))
        self.message.setText(str(robot.get('message') or '-'))
        self.progress.setValue(round(float(robot.get('progress', 0.0)) * 100))
        result = state.get('last_planner_result', {})
        if result:
            self.last_result.setPlainText(
                f"Status: {result.get('status', '')}\n"
                f"Mission: {result.get('mission_id', '')}\n"
                f"Task: {result.get('task_id', '')}\n"
                f"Message: {result.get('message', '')}\n"
                f"Time: {result.get('timestamp', '')}"
            )

    def _play(self) -> None:
        self.send_command({
            'cmd': 'manual_transfer',
            'source': str(self.pick.currentData()),
            'destination': str(self.place.currentData()),
        })
