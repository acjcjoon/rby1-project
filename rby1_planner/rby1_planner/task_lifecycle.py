"""Structured lifecycle events emitted by the planner Task runner."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class TaskRunStatus(str, Enum):
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    CANCELED = 'canceled'


@dataclass(frozen=True)
class TaskProgress:
    task_name: str
    phase: str
    completed_steps: int
    total_steps: int
    progress: float
    message: str = ''


@dataclass(frozen=True)
class TaskRunResult:
    task_name: str
    status: TaskRunStatus
    message: str = ''


__all__ = ['TaskProgress', 'TaskRunResult', 'TaskRunStatus']
