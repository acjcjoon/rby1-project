"""Fast, side-effect-free schedule preview using virtual laboratory time."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any
from .lab_config import LabConfig, WorkflowStep
from .models import Priority

@dataclass
class _Task:
    task_id: str
    batch_id: str
    step: WorkflowStep
    status: str = 'pending'
    start_min: float | None = None
    end_min: float | None = None

class ScheduleSimulator:
    def __init__(self, config: LabConfig) -> None:
        self.config = config

    def run(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        locations: dict[tuple[str, str], str] = {}
        completed: dict[str, set[str]] = {}
        batch_order: dict[str, int] = {}
        for index, batch in enumerate(snapshot.get('batches', [])):
            batch_id = str(batch['batch_id'])
            batch_order[batch_id] = index
            completed[batch_id] = set()
            for role, plate in batch.get('plates', {}).items():
                role = str(role).upper()
                locations[(batch_id, role)] = str(plate.get('location', 'UNKNOWN')).upper()
                completed[batch_id].update(str(value).upper() for value in plate.get('completed_tasks', []))
            for role, location in self.config.initial_plates.items():
                locations.setdefault((batch_id, role), location)
        priority_overrides: dict[str, Priority] = {}
        for item in snapshot.get('work_queue', []):
            task_id = str(item.get('task_id', ''))
            value = item.get('priority')
            if value is not None:
                priority_overrides[task_id] = Priority[str(value).upper()] if isinstance(value, str) else Priority(int(value))
            if str(item.get('status', '')).lower() == 'done':
                batch_id, _, local_id = task_id.partition('.')
                completed.setdefault(batch_id, set()).add(local_id.upper())
        tasks: dict[str, _Task] = {}
        for batch_id in batch_order:
            for step in self.config.workflow.values():
                runtime_step = step
                task_id = f'{batch_id}.{step.step_id}'
                if task_id in priority_overrides:
                    runtime_step = WorkflowStep(**{**step.__dict__, 'priority': priority_overrides[task_id]})
                tasks[task_id] = _Task(task_id, batch_id, runtime_step, 'done' if step.step_id in completed[batch_id] else 'pending')
        now = 0.0
        timeline: list[dict[str, Any]] = []
        reserved: dict[str, int] = {}
        for _ in range(max(100, len(tasks) * 10)):
            self._finish(tasks, locations, reserved, now)
            changed = True
            while changed:
                changed = self._start_processes(tasks, locations, reserved, timeline, now)
            if not any(task.status == 'running' and task.step.kind == 'transfer' for task in tasks.values()):
                candidate = self._choose_transfer(tasks, locations, reserved, batch_order)
                if candidate is not None:
                    self._start_transfer(candidate, locations, reserved, timeline, now)
                    self._start_processes(tasks, locations, reserved, timeline, now)
            if all(task.status == 'done' for task in tasks.values()):
                return self._result('completed', now, timeline, tasks, completed)
            endings = [task.end_min for task in tasks.values() if task.status == 'running' and task.end_min is not None]
            if not endings:
                return self._result('deadlock', now, timeline, tasks, completed)
            now = min(endings)
        return self._result('limit_reached', now, timeline, tasks, completed)

    def _finish(self, tasks: dict[str, _Task], locations: dict[tuple[str, str], str], reserved: dict[str, int], now: float) -> None:
        for task in tasks.values():
            if task.status != 'running' or task.end_min is None or task.end_min > now:
                continue
            step = task.step
            task.status = 'done'
            if step.kind == 'transfer':
                locations[(task.batch_id, step.role)] = step.destination
                reserved[step.destination] = max(0, reserved.get(step.destination, 0) - 1)
            else:
                for role in step.produces:
                    locations[(task.batch_id, role)] = step.device

    def _start_processes(self, tasks: dict[str, _Task], locations: dict[tuple[str, str], str], reserved: dict[str, int], timeline: list[dict[str, Any]], now: float) -> bool:
        changed = False
        for task in sorted(tasks.values(), key=lambda value: (value.step.order, value.batch_id)):
            step = task.step
            if task.status != 'pending' or step.kind != 'process' or not self._dependencies_done(task, tasks):
                continue
            running = sum(value.status == 'running' and value.step.kind == 'process' and value.step.device == step.device for value in tasks.values())
            if running >= self.config.devices[step.device].job_capacity:
                continue
            if any(locations.get((task.batch_id, role)) != step.device for role in step.roles):
                continue
            new_roles = [role for role in step.produces if (task.batch_id, role) not in locations]
            capacity = self.config.devices[step.device].plate_capacity
            if capacity is not None and self._used(locations, step.device) + reserved.get(step.device, 0) + len(new_roles) > capacity:
                continue
            self._start(task, step.duration_min, timeline, now, step.device)
            changed = True
        return changed

    def _choose_transfer(self, tasks: dict[str, _Task], locations: dict[tuple[str, str], str], reserved: dict[str, int], batch_order: dict[str, int]) -> _Task | None:
        candidates = []
        for task in tasks.values():
            step = task.step
            if task.status != 'pending' or step.kind != 'transfer' or not self._dependencies_done(task, tasks):
                continue
            if locations.get((task.batch_id, step.role)) != step.source:
                continue
            if not self._destination_available(task, tasks, locations, reserved):
                continue
            candidates.append(task)
        return min(candidates, key=lambda task: (int(task.step.priority), batch_order[task.batch_id], task.step.order)) if candidates else None

    def _destination_available(self, task: _Task, tasks: dict[str, _Task], locations: dict[tuple[str, str], str], reserved: dict[str, int]) -> bool:
        step = task.step
        device = self.config.devices.get(step.destination)
        if device is None or device.plate_capacity is None:
            return True
        needed = 1
        for process in tasks.values():
            spec = process.step
            if process.batch_id != task.batch_id or process.status != 'pending' or spec.kind != 'process':
                continue
            if spec.device != step.destination or step.role not in spec.roles:
                continue
            missing = sum(
                role != step.role and locations.get((task.batch_id, role)) != step.destination
                for role in spec.roles
            )
            fresh = sum((task.batch_id, role) not in locations for role in spec.produces if role not in spec.roles)
            needed = max(needed, 1 + missing + fresh)
        free = device.plate_capacity - self._used(locations, step.destination) - reserved.get(step.destination, 0)
        if free < needed:
            return False
        for process in self.config.workflow.values():
            if process.kind != 'process' or process.device != step.destination or step.role not in process.roles or step.step_id not in process.after:
                continue
            for outbound in self.config.workflow.values():
                if outbound.kind != 'transfer' or outbound.role != step.role or outbound.source != step.destination or process.step_id not in outbound.after:
                    continue
                target = self.config.devices.get(outbound.destination)
                if target and target.plate_capacity is not None:
                    target_free = target.plate_capacity - self._used(locations, outbound.destination) - reserved.get(outbound.destination, 0)
                    if step.source == outbound.destination:
                        target_free += 1
                    if target_free < 1:
                        return False
        return True

    def _start_transfer(self, task: _Task, locations: dict[tuple[str, str], str], reserved: dict[str, int], timeline: list[dict[str, Any]], now: float) -> None:
        step = task.step
        locations[(task.batch_id, step.role)] = 'ROBOT'
        reserved[step.destination] = reserved.get(step.destination, 0) + 1
        self._start(task, self.config.transfer_minutes(step.source, step.destination), timeline, now, 'ROBOT')

    @staticmethod
    def _start(task: _Task, duration: float, timeline: list[dict[str, Any]], now: float, resource: str) -> None:
        task.status, task.start_min, task.end_min = 'running', now, now + duration
        step = task.step
        timeline.append({
            'task_id': task.task_id, 'kind': step.kind, 'resource': resource,
            'route': f'{step.source} -> {step.destination}' if step.kind == 'transfer' else step.device,
            'priority': step.priority.name if step.kind == 'transfer' else '-',
            'start_min': now, 'end_min': now + duration, 'duration_min': duration,
        })

    @staticmethod
    def _dependencies_done(task: _Task, tasks: dict[str, _Task]) -> bool:
        return all(tasks[f'{task.batch_id}.{item}'].status == 'done' for item in task.step.after)

    @staticmethod
    def _used(locations: dict[tuple[str, str], str], device: str) -> int:
        return sum(location == device for location in locations.values())

    @staticmethod
    def _result(status: str, now: float, timeline: list[dict[str, Any]], tasks: dict[str, _Task], completed: dict[str, set[str]]) -> dict[str, Any]:
        pending = [task.task_id for task in tasks.values() if task.status != 'done']
        return {
            'status': status, 'end_min': now, 'timeline': sorted(timeline, key=lambda row: (row['start_min'], row['task_id'])),
            'task_count': len(tasks), 'completed_before': sum(len(items) for items in completed.values()),
            'simulated_count': len(timeline), 'pending': pending,
        }
