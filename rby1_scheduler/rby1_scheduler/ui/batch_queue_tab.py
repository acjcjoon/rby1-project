"""Batch-grouped live work queue and task controls."""
from __future__ import annotations

from collections import Counter
from typing import Any, Callable

from .qt import (
    QGridLayout, QGroupBox, QLabel, QPushButton, QScrollArea, QVBoxLayout,
    QWidget,
)


def build_batch_groups(state: dict[str, Any]) -> list[dict[str, Any]]:
    """Return queue entries grouped in stable batch and workflow order."""
    batch_metadata = {
        str(batch.get('batch_id', '')): batch
        for batch in state.get('batches', [])
    }
    grouped: dict[str, list[dict[str, Any]]] = {
        batch_id: [] for batch_id in batch_metadata
    }
    for task in state.get('queue', []):
        if str(task.get('kind', 'transfer')).lower() != 'transfer':
            continue
        batch_id = str(task.get('batch_id') or 'UNKNOWN')
        grouped.setdefault(batch_id, []).append(task)
    ordered_ids = list(batch_metadata)
    ordered_ids.extend(
        sorted(batch_id for batch_id in grouped if batch_id not in batch_metadata)
    )
    groups = []
    for batch_id in ordered_ids:
        tasks = sorted(
            grouped[batch_id],
            key=lambda task: (
                int(task.get('order', 0)),
                str(task.get('task_id', '')),
            ),
        )
        metadata = batch_metadata.get(batch_id, {})
        groups.append({
            'batch_id': batch_id,
            'molar_ratio': metadata.get('molar_ratio', {}),
            'batch_status': str(metadata.get('status', '')),
            'tasks': tasks,
        })
    return groups


class BatchQueueTab(QWidget):
    def __init__(self, send_command: Callable[[dict[str, Any]], None]) -> None:
        super().__init__()
        self.send_command = send_command
        self.summary = QLabel('라이브 작업 큐 연결 대기 중')
        self.help = QLabel(
            '목적지 버튼을 누르면 그 작업에 필요한 선행 공정과 이송을 순서대로 실행합니다. '
            '선택한 목적지 버튼을 다시 누르면 실행 체인 전체를 취소합니다.'
        )
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.content = QWidget()
        self.content_layout = QVBoxLayout(self.content)
        self.scroll.setWidget(self.content)
        layout = QVBoxLayout(self)
        layout.addWidget(self.summary)
        layout.addWidget(self.help)
        layout.addWidget(self.scroll)
        self._structure: tuple[tuple[str, tuple[str, ...]], ...] = ()
        self._batch_boxes: dict[str, QGroupBox] = {}
        self._task_buttons: dict[str, QPushButton] = {}

    def update_state(self, state: dict[str, Any]) -> None:
        groups = build_batch_groups(state)
        structure = tuple(
            (
                group['batch_id'],
                tuple(str(task.get('task_id', '')) for task in group['tasks']),
            )
            for group in groups
        )
        if structure != self._structure:
            self._rebuild(groups)
            self._structure = structure

        all_tasks = [task for group in groups for task in group['tasks']]
        totals = Counter(str(task.get('status', 'unknown')) for task in all_tasks)
        status_text = ' · '.join(
            f'{status.upper()} {count}'
            for status, count in sorted(totals.items())
        )
        self.summary.setText(
            f"배치 {len(groups)}개 · 작업 {len(all_tasks)}개"
            + (f' · {status_text}' if status_text else '')
        )

        active_task_id = str(state.get('current_task_id', ''))
        selected_target_id = str(state.get('selected_target_id', ''))
        for group in groups:
            batch_id = group['batch_id']
            batch_box = self._batch_boxes[batch_id]
            tasks = group['tasks']
            counts = Counter(str(task.get('status', 'unknown')) for task in tasks)
            ratio = group.get('molar_ratio', {}).get('display', '')
            title = batch_id + (f' · 몰비 {ratio}' if ratio else '')
            title += f' · 작업 {len(tasks)}건'
            state_text = ' · '.join(
                f'{status.upper()} {count}'
                for status, count in sorted(counts.items())
            ) or group.get('batch_status', '')
            batch_box.setTitle(f'{title} · {state_text}')

            for task in tasks:
                task_id = str(task.get('task_id', ''))
                button = self._task_buttons[task_id]
                status = str(task.get('status', 'unknown')).lower()
                active = task_id == active_task_id or bool(task.get('active'))
                selected = task_id == selected_target_id or bool(task.get('selected'))
                button.setText(str(task.get('destination', '-')))
                button.setChecked(active or selected)
                can_start = bool(task.get('operator_startable'))
                button.setEnabled(
                    selected or (status != 'done' and can_start)
                )
                block_reason = str(task.get('operator_block_reason', ''))
                error = str(task.get('error', ''))
                action_hint = (
                    '다시 눌러서 선행 작업 포함 실행 취소' if selected
                    else error or block_reason or (
                        '현재 작업 완료 후 선행 작업부터 실행'
                        if active_task_id
                        else '눌러서 선행 작업부터 실행'
                    )
                )
                button.setToolTip(
                    f"{task_id} · Plate {task.get('role', '-')} · "
                    f"{task.get('priority', '-')}\n"
                    f"{task.get('source', '')} → {task.get('destination', '')}\n"
                    f"상태: {status}\n{action_hint}"
                )
                button.setStyleSheet(self._button_style(status, active or selected))

    def _rebuild(self, groups: list[dict[str, Any]]) -> None:
        while self.content_layout.count():
            item = self.content_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        self._batch_boxes = {}
        self._task_buttons = {}
        for group in groups:
            batch_id = group['batch_id']
            batch_box = QGroupBox(batch_id)
            grid = QGridLayout(batch_box)
            self._batch_boxes[batch_id] = batch_box
            for index, task in enumerate(group['tasks']):
                task_id = str(task.get('task_id', ''))
                button = QPushButton(task_id)
                button.setCheckable(True)
                button.setMinimumHeight(34)
                button.clicked.connect(
                    lambda _checked=False, selected=task_id:
                    self._toggle_task(selected)
                )
                grid.addWidget(button, 0, index)
                grid.setColumnStretch(index, 1)
                self._task_buttons[task_id] = button
            self.content_layout.addWidget(batch_box)
        self.content_layout.addStretch(1)

    def _toggle_task(self, task_id: str) -> None:
        self.send_command({
            'cmd': 'toggle_task',
            'task_id': str(task_id),
            'reason': 'PyQt operator',
        })

    @staticmethod
    def _button_style(status: str, active: bool) -> str:
        if active:
            colors = ('#1565c0', '#ffffff', '#0d47a1')
        elif status == 'done':
            colors = ('#dcedc8', '#33691e', '#aed581')
        elif status == 'failed':
            colors = ('#ffebee', '#b71c1c', '#ef9a9a')
        elif status == 'canceled':
            colors = ('#fff3e0', '#e65100', '#ffcc80')
        else:
            colors = ('#f5f7fa', '#263238', '#cfd8dc')
        background, foreground, border = colors
        return (
            'QPushButton {'
            f'background: {background}; color: {foreground}; '
            f'border: 1px solid {border}; border-radius: 7px; '
            'padding: 4px; text-align: center; font-weight: 600;'
            '}'
            'QPushButton:disabled { color: #78909c; }'
        )
