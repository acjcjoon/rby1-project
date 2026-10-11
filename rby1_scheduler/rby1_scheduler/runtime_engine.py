"""Event-driven runtime scheduler independent from ROS and storage transport."""
from __future__ import annotations
from datetime import datetime, timezone
import time
from typing import Any
from .models import PlateRecord, Priority, TaskStatus, WorkItem
from .planner_client import PlannerClient, PlannerReport

def _now() -> str:
    return datetime.now(timezone.utc).isoformat()

class RuntimeEngine:
    def __init__(
        self,
        planner: PlannerClient,
        clock=time.monotonic,
        *,
        mock_process_duration_sec: float | None = None,
    ) -> None:
        self.planner = planner
        self._clock = clock
        self.mock_process_duration_sec = (
            None if mock_process_duration_sec is None
            else max(0.01, float(mock_process_duration_sec))
        )
        self.input_revision = -1
        self.paused = True
        self.plates: dict[str, PlateRecord] = {}
        self.tasks: dict[str, WorkItem] = {}
        self.batches: dict[str, dict[str, Any]] = {}
        self._batch_order: dict[str, int] = {}
        self.capacities: dict[str, int] = {}
        self.current_task_id = ''
        self.current_mission_id = ''
        self.selected_target_id = ''
        self._selected_plan_ids: set[str] = set()
        self._selection_return_to_paused = True
        self._process_start_sec: float | None = None
        self._process_end_sec: float | None = None
        self.robot = PlannerReport()
        self.last_planner_result: dict[str, Any] = {}
        self._live_origin_sec: float | None = None
        self._live_timeline: list[dict[str, Any]] = []
        self._active_timeline_index: int | None = None
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
        self._batch_order = {}
        for batch_index, batch in enumerate(raw.get('batches', [])):
            batch_id = str(batch['batch_id'])
            ratio = dict(batch.get('molar_ratio', {}))
            batches[batch_id] = {'batch_id': batch_id, 'molar_ratio': ratio, 'status': str(batch.get('status', 'waiting'))}
            self._batch_order[batch_id] = batch_index
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
        self._poll_active_task()
        if self.current_task_id and self.robot.status in {'succeeded', 'failed', 'canceled'}:
            self._finish_current(self.robot)
        if not self.paused and not self.current_task_id and self.robot.status == 'idle':
            task = self.choose()
            if task is not None:
                self._start(task)

    def choose(self) -> WorkItem | None:
        scope = self._selected_plan_ids or set(self.tasks)
        candidates = [
            task for task in self.tasks.values()
            if task.task_id in scope and self._startable(task)
        ]
        return min(
            candidates,
            key=lambda task: (
                int(task.priority),
                self._batch_order.get(task.batch_id, len(self._batch_order)),
                task.order,
                task.task_id,
            ),
        ) if candidates else None

    def apply_command(self, command: dict[str, Any]) -> None:
        kind = str(command.get('cmd', '')).lower()
        if kind == 'pause':
            self.paused = True
            self.selected_target_id = ''
            self._selected_plan_ids.clear()
            self._selection_return_to_paused = True
            self._cancel_current('scheduler paused')
            self._event('scheduler_paused')
        elif kind in {'resume', 'play'}:
            self.paused = False
            self._event('scheduler_resumed')
        elif kind == 'promote':
            task = self.tasks.get(str(command.get('task_id', '')))
            if task is None:
                raise ValueError('unknown task_id')
            task.priority = Priority.P1
            self._event('task_promoted', task_id=task.task_id, reason=str(command.get('reason', '')))
        elif kind == 'retry':
            task = self.tasks.get(str(command.get('task_id', '')))
            if task is None or task.status not in {TaskStatus.FAILED, TaskStatus.CANCELED}:
                raise ValueError('task is not retryable')
            task.status, task.error = TaskStatus.PENDING, ''
            self._event('task_retry_requested', task_id=task.task_id)
        elif kind == 'cancel':
            self.paused = True
            self.selected_target_id = ''
            self._selected_plan_ids.clear()
            self._selection_return_to_paused = True
            self._cancel_current('operator canceled the task')
        elif kind == 'toggle_task':
            task_id = str(command.get('task_id', '')).strip()
            task = self.tasks.get(task_id)
            if task is None:
                raise ValueError('unknown task_id')
            if task.kind != 'transfer':
                raise ValueError('only transfer tasks can be selected from the batch queue')
            if self.selected_target_id == task_id:
                current_belongs_to_selection = (
                    self.current_task_id in self._selected_plan_ids
                )
                self.paused = self._selection_return_to_paused
                self.selected_target_id = ''
                self._selected_plan_ids.clear()
                self._selection_return_to_paused = True
                if current_belongs_to_selection:
                    self._cancel_current(
                        'operator canceled the selected task sequence'
                    )
                self._event('operator_task_sequence_canceled', task_id=task_id)
                return
            reason = self._operator_block_reason(task)
            if reason:
                raise ValueError(reason)
            plan = self._dependency_closure(task)
            if self.mock_process_duration_sec is None:
                pending_processes = [
                    item.task_id for item in plan
                    if item.kind == 'process' and item.status is not TaskStatus.DONE
                ]
                if pending_processes:
                    raise ValueError(
                        'required process tasks are not connected to equipment: '
                        + ', '.join(pending_processes)
                    )
            for item in plan:
                if item.status in {TaskStatus.FAILED, TaskStatus.CANCELED}:
                    item.status, item.error = TaskStatus.PENDING, ''
            if not self.selected_target_id:
                self._selection_return_to_paused = self.paused
            self.selected_target_id = task_id
            self._selected_plan_ids = {item.task_id for item in plan}
            self.paused = False
            self._event(
                'operator_task_requested',
                task_id=task.task_id,
                execution_plan=[item.task_id for item in plan],
            )
            if not self.current_task_id and self.robot.status == 'idle':
                next_task = self.choose()
                if next_task is not None:
                    self._start(next_task)
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
                source=source, destination=destination, priority=Priority.P1,
                order=-number,
            )
            self.tasks[task.task_id] = task
            self.paused = True
            self.selected_target_id = ''
            self._selected_plan_ids.clear()
            self._selection_return_to_paused = True
            self._event('manual_transfer_requested', task_id=task.task_id, source=source, destination=destination)
            self._start(task)
        else:
            raise ValueError(f'unsupported scheduler command: {kind}')

    def snapshot(self) -> dict[str, Any]:
        plates = [plate.to_dict() for plate in sorted(self.plates.values(), key=lambda value: (value.batch_id, value.role))]
        queue = []
        for task in sorted(
            self.tasks.values(),
            key=lambda value: (
                value.batch_id, value.order, value.task_id,
            ),
        ):
            item = task.to_dict()
            block_reason = self._operator_block_reason(task)
            item.update({
                'active': task.task_id == self.current_task_id,
                'selected': task.task_id == self.selected_target_id,
                'operator_startable': not block_reason,
                'operator_block_reason': block_reason,
            })
            queue.append(item)
        devices = []
        for location in sorted(self.capacities):
            used = sum(plate.location == location for plate in self.plates.values())
            devices.append({'id': location, 'used': used, 'capacity': self.capacities[location]})
        live_timeline = self._live_timeline_snapshot()
        return {
            'schema_version': 1, 'input_revision': self.input_revision,
            'scheduler': {'status': 'paused' if self.paused else 'running', 'paused': self.paused},
            'robot': {
                'status': self.robot.status, 'mission_id': self.robot.mission_id,
                'task_id': self.robot.task_id, 'phase': self.robot.phase,
                'progress': self.robot.progress, 'message': self.robot.message,
            },
            'current_task_id': self.current_task_id, 'batches': list(self.batches.values()),
            'selected_target_id': self.selected_target_id,
            'last_planner_result': dict(self.last_planner_result),
            'live_timeline': {
                'status': (
                    'running' if self.current_task_id
                    else 'paused' if self.paused
                    else 'idle'
                ),
                'elapsed_min': self._live_elapsed_min(),
                'task_count': len(live_timeline),
                'timeline': live_timeline,
            },
            'plates': plates, 'queue': queue, 'devices': devices, 'events': self._events[-100:],
        }

    def drain_events(self) -> list[dict[str, Any]]:
        events, self._pending_events = self._pending_events, []
        return events

    def _startable(self, task: WorkItem) -> bool:
        if task.status is not TaskStatus.PENDING:
            return False
        if not self._dependencies_done(task):
            return False
        if task.kind == 'process':
            if self.mock_process_duration_sec is None:
                return False
            return all(
                plate is not None and plate.location == task.device
                for plate in (self._plate_for_role(task.batch_id, role) for role in task.roles)
            )
        plate = self._plate_for(task)
        if plate is None or plate.location != task.source:
            return False
        if not self._destination_available(task):
            return False
        return True

    def _operator_block_reason(self, task: WorkItem) -> str:
        """Validate selection of a target; prerequisites run automatically."""
        if task.task_id == self.selected_target_id:
            return ''
        if task.kind != 'transfer':
            return 'process tasks are executed as prerequisites'
        if task.status is TaskStatus.DONE:
            return 'task is already completed'
        if task.status is TaskStatus.RUNNING:
            return 'task is already running'
        try:
            plan = self._dependency_closure(task)
        except ValueError as exc:
            return str(exc)
        if self.mock_process_duration_sec is None and any(
            item.kind == 'process' and item.status is not TaskStatus.DONE
            for item in plan
        ):
            return 'required process equipment interface is not connected'
        return ''

    def _start(self, task: WorkItem) -> None:
        if task.kind == 'process':
            mission_id = f'mock-process-{task.task_id}-{self._sequence + 1}'
            self._process_start_sec = self._clock()
            self._process_end_sec = (
                self._process_start_sec + float(self.mock_process_duration_sec or 0.0)
            )
        else:
            mission_id = self.planner.start(task)
        task.status = TaskStatus.RUNNING
        for plate in self._plates_for_task(task):
            plate.current_task, plate.task_status, plate.updated_at = task.task_id.split('.')[-1], TaskStatus.RUNNING.value, _now()
        self.current_task_id, self.current_mission_id = task.task_id, mission_id
        self._start_live_timeline(task)
        self.robot = PlannerReport('running', mission_id, task.task_id, 'accepted', 0.0)
        self._event('task_started', task_id=task.task_id, mission_id=mission_id)

    def _finish_current(self, report: PlannerReport) -> None:
        task = self.tasks[self.current_task_id]
        plates = self._plates_for_task(task)
        self._finish_live_timeline(report.status)
        self.last_planner_result = {
            'status': report.status, 'mission_id': self.current_mission_id,
            'task_id': task.task_id, 'message': report.message, 'timestamp': _now(),
        }
        if report.status == 'succeeded':
            task.status = TaskStatus.DONE
            local_id = task.task_id.split('.')[-1]
            for plate in plates:
                if local_id not in plate.completed_tasks:
                    plate.completed_tasks.append(local_id)
                if task.kind == 'transfer':
                    plate.location = task.destination
                elif plate.role in task.produces:
                    plate.location = task.device
                plate.current_task = ''
                plate.task_status = (
                    TaskStatus.DONE.value
                    if task.kind == 'transfer' and task.destination == 'WASTE'
                    else TaskStatus.PENDING.value
                )
                plate.updated_at = _now()
            self._event('task_completed', task_id=task.task_id, mission_id=self.current_mission_id)
        else:
            task.status = TaskStatus.CANCELED if report.status == 'canceled' else TaskStatus.FAILED
            task.error = report.message
            for plate in plates:
                plate.current_task = ''
                plate.task_status, plate.updated_at = task.status.value, _now()
            self._event('task_failed', task_id=task.task_id, mission_id=self.current_mission_id, reason=report.message)
        finished_target = task.task_id == self.selected_target_id
        sequence_failed = report.status != 'succeeded' and task.task_id in self._selected_plan_ids
        self.current_task_id, self.current_mission_id = '', ''
        self._process_start_sec = self._process_end_sec = None
        self.robot = PlannerReport()
        if finished_target or sequence_failed:
            target_id = self.selected_target_id
            self.selected_target_id = ''
            self._selected_plan_ids.clear()
            self.paused = self._selection_return_to_paused
            self._selection_return_to_paused = True
            self._event(
                'operator_task_sequence_finished',
                task_id=target_id or task.task_id,
                status=report.status,
            )

    def _start_live_timeline(self, task: WorkItem) -> None:
        now = self._clock()
        if self._live_origin_sec is None:
            self._live_origin_sec = now
        elapsed = max(0.0, (now - self._live_origin_sec) / 60.0)
        is_process = task.kind == 'process'
        self._live_timeline.append({
            'task_id': task.task_id,
            'batch_id': task.batch_id,
            'kind': task.kind,
            'resource': task.device if is_process else 'ROBOT',
            'route': task.device if is_process else f'{task.source} -> {task.destination}',
            'priority': '-' if is_process else task.priority.name,
            'status': TaskStatus.RUNNING.value,
            'start_min': elapsed,
            'end_min': elapsed,
            'duration_min': 0.0,
        })
        self._active_timeline_index = len(self._live_timeline) - 1

    def _finish_live_timeline(self, status: str) -> None:
        index = self._active_timeline_index
        if index is None or index >= len(self._live_timeline):
            return
        elapsed = self._live_elapsed_min()
        item = self._live_timeline[index]
        item['end_min'] = max(float(item['start_min']), elapsed)
        item['duration_min'] = item['end_min'] - item['start_min']
        item['status'] = status
        self._active_timeline_index = None

    def _live_elapsed_min(self) -> float:
        if self._live_origin_sec is None:
            return 0.0
        return max(0.0, (self._clock() - self._live_origin_sec) / 60.0)

    def _live_timeline_snapshot(self) -> list[dict[str, Any]]:
        rows = [dict(item) for item in self._live_timeline]
        index = self._active_timeline_index
        if index is not None and index < len(rows):
            elapsed = self._live_elapsed_min()
            item = rows[index]
            item['end_min'] = max(float(item['start_min']), elapsed)
            item['duration_min'] = item['end_min'] - item['start_min']
        return rows

    def _plate_for(self, task: WorkItem) -> PlateRecord | None:
        return next((plate for plate in self.plates.values() if plate.batch_id == task.batch_id and plate.role == task.role), None)

    def _plate_for_role(self, batch_id: str, role: str) -> PlateRecord | None:
        return next((
            plate for plate in self.plates.values()
            if plate.batch_id == batch_id and plate.role == role
        ), None)

    def _plates_for_task(self, task: WorkItem) -> list[PlateRecord]:
        roles = [task.role] if task.kind == 'transfer' else list(dict.fromkeys((*task.roles, *task.produces)))
        return [
            plate for role in roles
            if (plate := self._plate_for_role(task.batch_id, role)) is not None
        ]

    def _dependencies_done(self, task: WorkItem) -> bool:
        for local_id in task.prerequisites:
            dependency = self.tasks.get(f'{task.batch_id}.{local_id}')
            if dependency is None or dependency.status is not TaskStatus.DONE:
                return False
        return True

    def _destination_available(self, task: WorkItem) -> bool:
        capacity = self.capacities.get(task.destination)
        if capacity is None:
            return True
        used = sum(
            plate.location == task.destination for plate in self.plates.values()
        )
        needed = 1
        for process in self.tasks.values():
            if (
                process.batch_id != task.batch_id
                or process.kind != 'process'
                or process.status is not TaskStatus.PENDING
                or process.device != task.destination
                or task.role not in process.roles
                or task.task_id.split('.')[-1] not in process.prerequisites
            ):
                continue
            missing_peers = sum(
                role != task.role
                and (
                    (plate := self._plate_for_role(task.batch_id, role)) is None
                    or plate.location != task.destination
                )
                for role in process.roles
            )
            fresh_outputs = sum(
                self._plate_for_role(task.batch_id, role) is None
                for role in process.produces
                if role not in process.roles
            )
            needed = max(needed, 1 + missing_peers + fresh_outputs)
        return capacity - used >= needed

    def _dependency_closure(self, target: WorkItem) -> list[WorkItem]:
        ordered: list[WorkItem] = []
        visiting: set[str] = set()
        visited: set[str] = set()

        def visit(task: WorkItem) -> None:
            if task.task_id in visited:
                return
            if task.task_id in visiting:
                raise ValueError(f'cyclic task dependency at {task.task_id}')
            visiting.add(task.task_id)
            for local_id in task.prerequisites:
                dependency_id = f'{task.batch_id}.{local_id}'
                dependency = self.tasks.get(dependency_id)
                if dependency is None:
                    raise ValueError(f'missing prerequisite: {dependency_id}')
                visit(dependency)
            visiting.remove(task.task_id)
            visited.add(task.task_id)
            ordered.append(task)

        visit(target)
        return ordered

    def _poll_active_task(self) -> None:
        if not self.current_task_id:
            self.robot = self.planner.poll()
            return
        task = self.tasks[self.current_task_id]
        if task.kind != 'process':
            self.robot = self.planner.poll()
            return
        start = self._process_start_sec
        end = self._process_end_sec
        now = self._clock()
        if start is None or end is None:
            self.robot = PlannerReport(
                'failed', self.current_mission_id, task.task_id,
                'failed', 0.0, 'mock process timer was not initialized',
            )
            return
        duration = max(0.01, end - start)
        progress = min(1.0, max(0.0, (now - start) / duration))
        if progress >= 1.0:
            self.robot = PlannerReport(
                'succeeded', self.current_mission_id, task.task_id,
                'completed', 1.0, 'mock process completed',
            )
        else:
            self.robot = PlannerReport(
                'running', self.current_mission_id, task.task_id,
                'processing', progress, 'mock equipment process running',
            )

    def _cancel_current(self, reason: str) -> None:
        if not self.current_task_id:
            return
        task = self.tasks[self.current_task_id]
        self._event('task_cancel_requested', task_id=task.task_id, reason=reason)
        if task.kind == 'process':
            self._finish_current(PlannerReport(
                'canceled', self.current_mission_id, task.task_id,
                'canceled', self.robot.progress, reason,
            ))
        else:
            self.planner.cancel()

    def _event(self, event_type: str, **payload: Any) -> None:
        self._sequence += 1
        event = {'sequence': self._sequence, 'timestamp': _now(), 'type': event_type, **payload}
        self._events.append(event)
        self._pending_events.append(event)
