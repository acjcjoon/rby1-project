"""Top-level tabbed scheduler window."""
from __future__ import annotations
from typing import Any, Callable
from .communication_tab import CommunicationTestTab
from .plate_tab import PlateTab
from .qt import QKeySequence, QMainWindow, QShortcut, QTabWidget
from .scheduling_tab import SchedulingTab

class SchedulerMainWindow(QMainWindow):
    def __init__(self, send_command: Callable[[dict[str, Any]], None]) -> None:
        super().__init__()
        self._send_command = send_command
        self.setWindowTitle('RB-Y1 Scheduler')
        self.resize(1280, 820)
        self.tabs = QTabWidget()
        self.scheduling_tab = SchedulingTab(send_command)
        self.plate_tab = PlateTab()
        self.communication_tab = CommunicationTestTab(send_command)
        self.tabs.addTab(self.scheduling_tab, '\uc2a4\ucf00\uc904\ub9c1')
        self.tabs.addTab(self.plate_tab, '\ud50c\ub808\uc774\ud2b8 \uc815\ubcf4')
        self.tabs.addTab(self.communication_tab, '\ud1b5\uc2e0 \ud14c\uc2a4\ud2b8')
        self.setCentralWidget(self.tabs)
        self.pause_shortcut = QShortcut(QKeySequence('Space'), self)
        self.pause_shortcut.setAutoRepeat(False)
        self.pause_shortcut.activated.connect(self._pause_immediately)
        self.statusBar().showMessage(
            'Space: Pause scheduling and cancel the active task'
        )

    def update_state(self, state: dict[str, Any]) -> None:
        self.scheduling_tab.update_state(state)
        self.plate_tab.update_state(state)
        self.communication_tab.update_state(state)

    def append_events(self, events: list[dict[str, Any]]) -> None:
        self.scheduling_tab.append_events(events)

    def _pause_immediately(self) -> None:
        self._send_command({'cmd': 'pause'})
