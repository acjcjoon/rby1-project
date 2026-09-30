"""Planner boundary with a deterministic mock used until ROS Action is connected."""
from __future__ import annotations
import time
import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional
from .models import WorkItem

@dataclass(frozen=True)
class PlannerReport:
    status: str = 'idle'
    mission_id: str = ''
    task_id: str = ''
    phase: str = ''
    progress: float = 0.0
    message: str = ''

class PlannerClient(ABC):
    @abstractmethod
    def start(self, task: WorkItem) -> str:
        raise NotImplementedError

    @abstractmethod
    def poll(self) -> PlannerReport:
        raise NotImplementedError

    @abstractmethod
    def cancel(self) -> None:
        raise NotImplementedError

class MockPlannerClient(PlannerClient):
    def __init__(self, duration_sec: float = 4.0, clock=time.monotonic) -> None:
        self.duration_sec = max(0.1, float(duration_sec))
        self.clock = clock
        self._task: Optional[WorkItem] = None
        self._mission_id = ''
        self._started_at = 0.0
        self._terminal: Optional[PlannerReport] = None

    def start(self, task: WorkItem) -> str:
        if self._task is not None:
            raise RuntimeError('planner is already running')
        self._task = task
        self._mission_id = f'mission-{uuid.uuid4().hex[:12]}'
        self._started_at = self.clock()
        self._terminal = None
        return self._mission_id

    def poll(self) -> PlannerReport:
        if self._terminal is not None:
            result, self._terminal = self._terminal, None
            return result
        if self._task is None:
            return PlannerReport()
        elapsed = self.clock() - self._started_at
        progress = min(1.0, elapsed / self.duration_sec)
        if progress < 1.0:
            phase = 'picking' if progress < 0.4 else 'moving' if progress < 0.8 else 'placing'
            return PlannerReport('running', self._mission_id, self._task.task_id, phase, progress)
        task, mission_id = self._task, self._mission_id
        self._task = None
        self._mission_id = ''
        return PlannerReport('succeeded', mission_id, task.task_id, 'completed', 1.0, 'mock transfer completed')

    def cancel(self) -> None:
        if self._task is None:
            return
        self._terminal = PlannerReport('canceled', self._mission_id, self._task.task_id, 'canceled', 0.0, 'canceled by operator')
        self._task = None
        self._mission_id = ''
