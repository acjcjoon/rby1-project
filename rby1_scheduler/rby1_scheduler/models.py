"""Domain records exchanged by the scheduler, mock server and UI."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Any

class Priority(IntEnum):
    P0 = 0
    P1 = 1
    P2 = 2
    P3 = 3
    P4 = 4
    P5 = 5

class TaskStatus(str, Enum):
    PENDING = 'pending'
    RUNNING = 'running'
    DONE = 'done'
    FAILED = 'failed'
    BLOCKED = 'blocked'
    CANCELED = 'canceled'
    UNKNOWN = 'unknown'

@dataclass
class PlateRecord:
    plate_id: str
    batch_id: str
    role: str
    barcode: str
    molar_ratio: dict[str, Any]
    location: str
    current_task: str = ''
    task_status: str = TaskStatus.PENDING.value
    completed_tasks: list[str] = field(default_factory=list)
    updated_at: str = ''

    @classmethod
    def from_dict(cls, batch_id: str, ratio: dict[str, Any], role: str, raw: dict[str, Any]) -> 'PlateRecord':
        return cls(
            plate_id=str(raw.get('plate_id') or f'{batch_id}-{role}'), batch_id=batch_id,
            role=role, barcode=str(raw['barcode']), molar_ratio=dict(ratio),
            location=str(raw.get('location', 'UNKNOWN')),
            current_task=str(raw.get('current_task', '')),
            task_status=str(raw.get('task_status', TaskStatus.PENDING.value)),
            completed_tasks=[str(value) for value in raw.get('completed_tasks', [])],
            updated_at=str(raw.get('updated_at', '')),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            'plate_id': self.plate_id, 'batch_id': self.batch_id, 'role': self.role,
            'barcode': self.barcode, 'molar_ratio': self.molar_ratio,
            'location': self.location, 'current_task': self.current_task,
            'task_status': self.task_status, 'completed_tasks': list(self.completed_tasks),
            'updated_at': self.updated_at,
        }

@dataclass
class WorkItem:
    task_id: str
    batch_id: str
    role: str
    source: str
    destination: str
    priority: Priority = Priority.P5
    status: TaskStatus = TaskStatus.PENDING
    prerequisites: list[str] = field(default_factory=list)
    order: int = 0
    error: str = ''

    @classmethod
    def from_dict(cls, raw: dict[str, Any], order: int) -> 'WorkItem':
        priority = raw.get('priority', 'P5')
        if isinstance(priority, str):
            priority = Priority[priority.upper()]
        return cls(
            task_id=str(raw['task_id']), batch_id=str(raw['batch_id']), role=str(raw['role']),
            source=str(raw['source']), destination=str(raw['destination']),
            priority=Priority(int(priority)), status=TaskStatus(str(raw.get('status', 'pending')).lower()),
            prerequisites=[str(value) for value in raw.get('prerequisites', [])], order=order,
            error=str(raw.get('error', '')),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            'task_id': self.task_id, 'batch_id': self.batch_id, 'role': self.role,
            'source': self.source, 'destination': self.destination,
            'priority': self.priority.name, 'status': self.status.value,
            'prerequisites': list(self.prerequisites), 'error': self.error,
        }
