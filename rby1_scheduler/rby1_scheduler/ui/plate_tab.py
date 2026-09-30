"""Plate registry, progress and molar-ratio tab."""
from __future__ import annotations
from typing import Any
from .qt import (
    QComboBox, QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

class PlateTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._plates: list[dict[str, Any]] = []
        self.search = QLineEdit()
        self.search.setPlaceholderText('바코드, Plate ID 또는 Batch 검색')
        self.status_filter = QComboBox()
        self.status_filter.addItems(['전체', 'pending', 'running', 'done', 'failed', 'blocked', 'unknown'])
        self.summary = QLabel('Plate 0개')
        self.table = QTableWidget(0, 9)
        self.table.setHorizontalHeaderLabels(['Barcode', 'Plate', 'Batch', 'Role', '몰비', '위치', '현재 Task', '상태', '완료 Task'])
        self.detail = QPlainTextEdit()
        self.detail.setReadOnly(True)
        filters = QHBoxLayout()
        filters.addWidget(self.search)
        filters.addWidget(self.status_filter)
        layout = QVBoxLayout(self)
        layout.addWidget(self.summary)
        layout.addLayout(filters)
        layout.addWidget(self.table, 3)
        layout.addWidget(QLabel('선택 Plate 상세'))
        layout.addWidget(self.detail, 2)
        self.search.textChanged.connect(self._render)
        self.status_filter.currentTextChanged.connect(self._render)
        self.table.itemSelectionChanged.connect(self._show_selected)

    def update_state(self, state: dict[str, Any]) -> None:
        self._plates = list(state.get('plates', []))
        counts: dict[str, int] = {}
        for plate in self._plates:
            status = str(plate.get('task_status', 'unknown'))
            counts[status] = counts.get(status, 0) + 1
        extra = ' · '.join(f'{key} {value}' for key, value in sorted(counts.items()))
        self.summary.setText(f'전체 Plate {len(self._plates)}개' + (f' · {extra}' if extra else ''))
        self._render()

    def _render(self, *_args) -> None:
        query = self.search.text().strip().lower()
        selected_status = self.status_filter.currentText()
        visible = []
        for plate in self._plates:
            haystack = ' '.join(str(plate.get(key, '')) for key in ('barcode', 'plate_id', 'batch_id')).lower()
            if query and query not in haystack:
                continue
            if selected_status != '전체' and plate.get('task_status') != selected_status:
                continue
            visible.append(plate)
        self.table.setRowCount(len(visible))
        for row, plate in enumerate(visible):
            ratio = plate.get('molar_ratio', {}).get('display', '')
            values = [
                plate.get('barcode'), plate.get('plate_id'), plate.get('batch_id'), plate.get('role'),
                ratio, plate.get('location'), plate.get('current_task') or '-', plate.get('task_status'),
                ', '.join(plate.get('completed_tasks', [])),
            ]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(str(value)))
        self.table.resizeColumnsToContents()

    def _show_selected(self) -> None:
        row = self.table.currentRow()
        plate_id_item = self.table.item(row, 1) if row >= 0 else None
        if plate_id_item is None:
            return
        plate = next((value for value in self._plates if value.get('plate_id') == plate_id_item.text()), None)
        if plate is None:
            return
        ratio = plate.get('molar_ratio', {}).get('display', '')
        lines = [
            f"Barcode: {plate.get('barcode', '')}", f"Plate ID: {plate.get('plate_id', '')}",
            f"Batch: {plate.get('batch_id', '')}", f"Role: {plate.get('role', '')}",
            f"몰비: {ratio}", f"현재 위치: {plate.get('location', '')}",
            f"현재 Task: {plate.get('current_task') or '-'}", f"상태: {plate.get('task_status', '')}",
            f"완료 Task: {', '.join(plate.get('completed_tasks', [])) or '-'}",
            f"마지막 갱신: {plate.get('updated_at', '') or '-'}",
        ]
        self.detail.setPlainText('\n'.join(lines))
