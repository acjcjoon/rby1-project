"""Event-driven runtime scheduler independent from ROS and storage transport."""
from __future__ import annotations
from datetime import datetime, timezone
from typing import Any
from .models import PlateRecord, Priority, TaskStatus, WorkItem
from .planner_client import PlannerClient, PlannerReport

def _now() -> str:
    return datetime.now(timezone.utc).isoformat()

class RuntimeEngine:
    def __init__(self, planner: PlannerClient) -> None:
        self.planner = planner
        self.input_revision = -1
        self.paused = True
        self.plates: dict[str, PlateRecord] = {}
        self.tasks: dict[str, WorkItem] = {}
        self.batches: dict[str, dict[str, Any]] = {}
        self.capacities: dict[str, int] = {}
        self.current_task_id = ''
        self.current_mission_id = ''
        self.robot = PlannerReport()
        self.last_planner_result: dict[str, Any] = {}
        self._events: list[dict[str, Any]] = []
        self._pending_events: list[dict[str, Any]] = []
        self._sequence = 0

    def sync_external(self, raw: dict[str, Any], force: bool = False) -> bool:
        revision = int(raw.get('revision', 0))
        if not force and revision == self.input_revision:
            return False
        if self.current_task_id:
            self._event('external_sync_deferred', message='planner task is running')
            return False
        plates: dict[str, PlateRecord] = {}
        batches: dict[str, dict[str, Any]] = {}
        for batch in raw.get('batches', []):
            batch_id = str(batch['batch_id'])
            ratio = dict(batch.get('molar_ratio', {}))
            batches[batch_id] = {'batch_id': batch_id, 'molar_ratio': ratio, 'status': str(batch.get('status', 'waiting'))}
            for role, plate_raw in batch.get('plates', {}).items():
                plate = PlateRecord.from_dict(batch_id, ratio, str(role), plate_raw)
                if plate.plate_id in plates:
                    raise ValueError(f'duplicate plate_id: {plate.plate_id}')
                plates[plate.plate_id] = plate
        tasks = {task.task_id: task for index, item in enumerate(raw.get('work_queue', [])) if (task := WorkItem.from_dict(item, index))}
        self.plates, self.tasks, self.batches = plates, tasks, batches
        self.capacities = {str(key): int(value) for key, value in raw.get('capacities', {}).items()}
        self.input_revision = revision
        self._event('external_state_loaded', revision=revision, plates=len(plates), tasks=len(tasks))
        return True

    def tick(self) -> None:
        self.robot = self.planner.poll()
        if self.current_task_id and self.robot.status in {'succeeded', 'failed', 'canceled'}:
            self._finish_current(self.robot)
        if not self.paused and not self.current_task_id and self.robot.status == 'idle':
            task = self.choose()
            if task is not None:
                self._start(task)

    def choose(self) -> WorkItem | None:
        candidates = [task for task in self.tasks.values() if self._startable(task)]
        return min(candidates, key=lambda task: (int(task.priority), task.order, task.task_id)) if candidates else None

    def apply_command(self, command: dict[str, Any]) -> None:
        kind = str(command.get('cmd', '')).lower()
        if kind == 'pause':
            self.paused = True
            if self.current_task_id:
                self.planner.cancel()
                self._event(
                    'task_cancel_requested',
                    task_id=self.current_task_id,
                    reason='scheduler paused',
                )
            self._event('scheduler_paused')
        elif kind in {'resume', 'play'}:
            self.paused = False
            self._event('scheduler_resumed')
        elif kind == 'promote':
            task = self.tasks.get(str(command.get('task_id', '')))
            if task is None:
                raise ValueError('unknown task_id')
            task.priority = Priority.P0
            self._event('task_promoted', task_id=task.task_id, reason=str(command.get('reason', '')))
        elif kind == 'retry':
            task = self.tasks.get(str(command.get('task_id', '')))
            if task is None or task.status not in {TaskStatus.FAILED, TaskStatus.CANCELED}:
                raise ValueError('task is not retryable')
            task.status, task.error = TaskStatus.PENDING, ''
            self._event('task_retry_requested', task_id=task.task_id)
        elif kind == 'cancel':
            if self.current_task_id:
                self.planner.cancel()
                self._event('task_cancel_requested', task_id=self.current_task_id)
        elif kind == 'manual_transfer':
            if self.current_task_id or self.robot.status != 'idle':
                raise ValueError('planner is busy')
            source = str(command.get('source', '')).strip().upper()
            destination = str(command.get('destination', '')).strip().upper()
            if not source or not destination:
                raise ValueError('source and destination are required')
            number = sum(task_id.startswith('MANUAL.') for task_id in self.tasks) + 1
            task = WorkItem(
                task_id=f'MANUAL.T{number:03d}', batch_id='MANUAL', role='TEST',
                source=source, destination=destination, priority=Priority.P0,
                order=-number,
            )
            self.tasks[task.task_id] = task
            self.paused = True
            self._event('manual_transfer_requested', task_id=task.task_id, source=source, destination=destination)
            self._start(task)
        else:
            raise ValueError(f'unsupported scheduler command: {kind}')

    def snapshot(self) -> dict[str, Any]:
        plates = [plate.to_dict() for plate in sorted(self.plates.values(), key=lambda value: (value.batch_id, value.role))]
        queue = [task.to_dict() for task in sorted(self.tasks.values(), key=lambda value: (value.status.value, int(value.priority), value.order))]
        devices = []
        for location in sorted(self.capacities):
            used = sum(plate.location == location for plate in self.plates.values())
            devices.append({'id': location, 'used': used, 'capacity': self.capacities[location]})
        return {
            'schema_version': 1, 'input_revision': self.input_revision,
            'scheduler': {'status': 'paused' if self.paused else 'running', 'paused': self.paused},
            'robot': {
                'status': self.robot.status, 'mission_id': self.robot.mission_id,
                'task_id': self.robot.task_id, 'phase': self.robot.phase,
                'progress': self.robot.progress, 'message': self.robot.message,
            },
            'current_task_id': self.current_task_id, 'batches': list(self.batches.values()),
            'last_planner_result': dict(self.last_planner_result),
            'plates': plates, 'queue': queue, 'devices': devices, 'events': self._events[-100:],
        }

    def drain_events(self) -> list[dict[str, Any]]:
        events, self._pending_events = self._pending_events, []
        return events

    def _startable(self, task: WorkItem) -> bool:
        if task.status is not TaskStatus.PENDING:
            return False
        plate = self._plate_for(task)
        if plate is None or plate.location != task.source:
            return False
        if not set(task.prerequisites).issubset(plate.completed_tasks):
            return False
        capacity = self.capacities.get(task.destination)
        if capacity is not None and sum(value.location == task.destination for value in self.plates.values()) >= capacity:
            return False
        return True

    def _start(self, task: WorkItem) -> None:
        mission_id = self.planner.start(task)
        task.status = TaskStatus.RUNNING
        plate = self._plate_for(task)
        if plate is not None:
            plate.current_task, plate.task_status, plate.updated_at = task.task_id.split('.')[-1], TaskStatus.RUNNING.value, _now()
        self.current_task_id, self.current_mission_id = task.task_id, mission_id
        self.robot = PlannerReport('running', mission_id, task.task_id, 'accepted', 0.0)
        self._event('task_started', task_id=task.task_id, mission_id=mission_id)

    def _finish_current(self, report: PlannerReport) -> None:
        task = self.tasks[self.current_task_id]
        plate = self._plate_for(task)
        self.last_planner_result = {
            'status': report.status, 'mission_id': self.current_mission_id,
            'task_id': task.task_id, 'message': report.message, 'timestamp': _now(),
        }
        if report.status == 'succeeded':
            task.status = TaskStatus.DONE
            if plate is not None:
                local_id = task.task_id.split('.')[-1]
                if local_id not in plate.completed_tasks:
                    plate.completed_tasks.append(local_id)
                plate.location, plate.current_task = task.destination, ''
                plate.task_status = TaskStatus.DONE.value if task.destination == 'WASTE' else TaskStatus.PENDING.value
                plate.updated_at = _now()
            self._event('task_completed', task_id=task.task_id, mission_id=self.current_mission_id)
        else:
            task.status = TaskStatus.CANCELED if report.status == 'canceled' else TaskStatus.FAILED
            task.error = report.message
            if plate is not None:
                plate.task_status, plate.updated_at = task.status.value, _now()
            self._event('task_failed', task_id=task.task_id, mission_id=self.current_mission_id, reason=report.message)
        self.current_task_id, self.current_mission_id = '', ''
        self.robot = PlannerReport()

    def _plate_for(self, task: WorkItem) -> PlateRecord | None:
        return next((plate for plate in self.plates.values() if plate.batch_id == task.batch_id and plate.role == task.role), None)

    def _event(self, event_type: str, **payload: Any) -> None:
        self._sequence += 1
        event = {'sequence': self._sequence, 'timestamp': _now(), 'type': event_type, **payload}
        self._events.append(event)
        self._pending_events.append(event)
