"""Static laboratory environment and workflow loaded from lab.yaml."""
from __future__ import annotations
import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any
import yaml
from .models import Priority

@dataclass(frozen=True)
class DeviceSpec:
    device_id: str
    job_capacity: int
    plate_capacity: int | None

@dataclass(frozen=True)
class WorkflowStep:
    step_id: str
    kind: str
    after: tuple[str, ...]
    priority: Priority
    role: str = ''
    roles: tuple[str, ...] = ()
    source: str = ''
    destination: str = ''
    device: str = ''
    duration_min: float = 0.0
    produces: tuple[str, ...] = ()
    order: int = 0

@dataclass(frozen=True)
class LabConfig:
    path: Path
    transfer_default_min: float
    transfer_overrides: dict[str, float]
    devices: dict[str, DeviceSpec]
    workflow: dict[str, WorkflowStep]
    initial_plates: dict[str, str]

    def transfer_minutes(self, source: str, destination: str) -> float:
        return self.transfer_overrides.get(f'{source}->{destination}', self.transfer_default_min)

    def enrich_snapshot(self, raw: dict[str, Any]) -> dict[str, Any]:
        data = copy.deepcopy(raw)
        data['capacities'] = {
            key: spec.plate_capacity for key, spec in self.devices.items()
            if spec.plate_capacity is not None
        }
        queue = data.setdefault('work_queue', [])
        known_task_ids = {
            str(task.get('task_id', '')) for task in queue
        }
        # The external server may send only current work. Populate the complete
        # authored workflow so Mock Play and selected-task execution can run
        # process prerequisites as well as robot transfers.
        for batch in data.get('batches', []):
            batch_id = str(batch.get('batch_id', ''))
            plates = batch.get('plates', {})
            for step in self.workflow.values():
                task_id = f'{batch_id}.{step.step_id}'
                if task_id in known_task_ids:
                    continue
                affected_roles = (
                    (step.role,) if step.kind == 'transfer'
                    else tuple(dict.fromkeys((*step.roles, *step.produces)))
                )
                completed = {
                    str(value).upper()
                    for role in affected_roles
                    for value in plates.get(role, {}).get('completed_tasks', [])
                }
                queue.append({
                    'task_id': task_id,
                    'batch_id': batch_id,
                    'order': step.order,
                    'status': (
                        'done' if step.step_id in completed else 'pending'
                    ),
                })
                known_task_ids.add(task_id)
        for task in queue:
            local_id = str(task.get('task_id', '')).rsplit('.', 1)[-1]
            step = self.workflow.get(local_id)
            if step is None:
                continue
            task.setdefault('kind', step.kind)
            task.setdefault('role', step.role)
            task.setdefault('roles', list(step.roles))
            task.setdefault('produces', list(step.produces))
            task.setdefault('source', step.source)
            task.setdefault('destination', step.destination)
            task.setdefault('device', step.device)
            task.setdefault('duration_min', step.duration_min)
            task.setdefault('priority', step.priority.name)
            task.setdefault('prerequisites', list(step.after))
            task.setdefault('order', step.order)
        return data

    def summary(self) -> dict[str, Any]:
        return {
            'config_path': str(self.path),
            'transfer_default_min': self.transfer_default_min,
            'device_count': len(self.devices),
            'workflow_steps': len(self.workflow),
        }

def load_lab_config(path: str | Path) -> LabConfig:
    config_path = Path(path)
    with config_path.open(encoding='utf-8') as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError('lab.yaml root must be an object')
    transfer = raw.get('transfer', {})
    default_min = _positive(transfer.get('default_min'), 'transfer.default_min')
    overrides = {
        str(route): _positive(value, f'transfer.override.{route}')
        for route, value in transfer.get('override', {}).items()
    }
    devices: dict[str, DeviceSpec] = {}
    for key, value in raw.get('devices', {}).items():
        job_capacity = int(value.get('job_capacity', 1))
        plate_capacity = value.get('plate_capacity')
        if job_capacity < 0 or (plate_capacity is not None and int(plate_capacity) < 1):
            raise ValueError(f'invalid capacity for device {key}')
        devices[str(key)] = DeviceSpec(str(key), job_capacity, None if plate_capacity is None else int(plate_capacity))
    workflow: dict[str, WorkflowStep] = {}
    for order, item in enumerate(raw.get('workflow', [])):
        step_id = str(item['id']).upper()
        if step_id in workflow:
            raise ValueError(f'duplicate workflow step: {step_id}')
        kind = str(item['kind']).lower()
        if kind not in {'transfer', 'process'}:
            raise ValueError(f'{step_id}: kind must be transfer or process')
        priority = Priority[str(item.get('priority', 'P5')).upper()]
        workflow[step_id] = WorkflowStep(
            step_id=step_id, kind=kind,
            after=tuple(str(value).upper() for value in item.get('after', [])),
            priority=priority, role=str(item.get('role', '')).upper(),
            roles=tuple(str(value).upper() for value in item.get('roles', [])),
            source=str(item.get('source', '')).upper(),
            destination=str(item.get('destination', '')).upper(),
            device=str(item.get('device', '')).upper(),
            duration_min=_positive(item.get('duration_min'), f'{step_id}.duration_min') if kind == 'process' else 0.0,
            produces=tuple(str(value).upper() for value in item.get('produces', [])), order=order,
        )
    _validate(devices, workflow)
    return LabConfig(
        config_path, default_min, overrides, devices, workflow,
        {str(role).upper(): str(location).upper() for role, location in raw.get('initial_plates', {}).items()},
    )

def _positive(value: Any, name: str) -> float:
    number = float(value)
    if number <= 0:
        raise ValueError(f'{name} must be positive')
    return number

def _validate(devices: dict[str, DeviceSpec], workflow: dict[str, WorkflowStep]) -> None:
    for step in workflow.values():
        missing = [item for item in step.after if item not in workflow]
        if missing:
            raise ValueError(f'{step.step_id}: unknown prerequisites {missing}')
        if step.kind == 'transfer':
            if not step.role or not step.source or not step.destination:
                raise ValueError(f'{step.step_id}: transfer requires role/source/destination')
        elif not step.device or step.device not in devices or not step.roles:
            raise ValueError(f'{step.step_id}: process requires known device and roles')
