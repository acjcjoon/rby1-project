"""Compact Qt Gantt chart for scheduler simulation results."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from .qt import QColor, QPainter, QPen, QRectF, QSize, Qt, QToolTip, QWidget


_RESOURCE_ORDER = ('ROBOT', 'LH1', 'LH2', 'ZS', 'MPR', 'INC')
_RESOURCE_LABELS = {
    'ROBOT': '이송 로봇',
    'LH1': 'LH1',
    'LH2': 'LH2',
    'ZS': 'Zeta Sizer',
    'MPR': 'Microplate',
    'INC': 'Incubator',
}
_BATCH_COLORS = (
    '#1F5FA8', '#C4641F', '#2F7D4E', '#6E3894', '#8A7100', '#0F7B84',
    '#5A7FC0', '#D98A4E', '#5E9E76', '#9166B0', '#B09A3A', '#4E9AA0',
)


@dataclass(frozen=True)
class GanttBar:
    task_id: str
    batch_id: str
    kind: str
    resource: str
    route: str
    start_min: float
    end_min: float
    duration_min: float
    lane: int


@dataclass(frozen=True)
class GanttRow:
    resource: str
    lane_count: int
    bars: tuple[GanttBar, ...]


def build_gantt_rows(timeline: list[dict[str, Any]]) -> list[GanttRow]:
    """Group timeline entries by resource and assign non-overlapping lanes."""
    grouped: dict[str, list[dict[str, Any]]] = {}
    for item in timeline:
        try:
            start = float(item.get('start_min', 0.0))
            end = float(item.get('end_min', start))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(start) or not math.isfinite(end) or end < start:
            continue
        resource = str(item.get('resource') or item.get('route') or 'OTHER')
        grouped.setdefault(resource, []).append({
            **item,
            '_start': start,
            '_end': end,
        })

    ordered_resources = [
        value
        for value in _RESOURCE_ORDER
        if value in grouped
    ]
    ordered_resources.extend(
        sorted(value for value in grouped if value not in _RESOURCE_ORDER)
    )
    rows: list[GanttRow] = []
    for resource in ordered_resources:
        lane_ends: list[float] = []
        bars: list[GanttBar] = []
        entries = sorted(
            grouped[resource],
            key=lambda value: (
                value['_start'],
                value['_end'],
                str(value.get('task_id', '')),
            ),
        )
        for item in entries:
            start, end = item['_start'], item['_end']
            lane = next(
                (
                    index
                    for index, lane_end in enumerate(lane_ends)
                    if lane_end <= start
                ),
                len(lane_ends),
            )
            if lane == len(lane_ends):
                lane_ends.append(end)
            else:
                lane_ends[lane] = end
            task_id = str(item.get('task_id', ''))
            batch_id = task_id.partition('.')[0] or 'UNKNOWN'
            bars.append(GanttBar(
                task_id=task_id,
                batch_id=batch_id,
                kind=str(item.get('kind', '')),
                resource=resource,
                route=str(item.get('route', resource)),
                start_min=start,
                end_min=end,
                duration_min=float(item.get('duration_min', end - start)),
                lane=lane,
            ))
        rows.append(GanttRow(
            resource=resource,
            lane_count=max(1, len(lane_ends)),
            bars=tuple(bars),
        ))
    return rows


def _format_minutes(value: float) -> str:
    total = max(0, round(value))
    hours, minutes = divmod(total, 60)
    return f'{hours}:{minutes:02d}'


def _tick_step(span_min: float) -> float:
    for step in (15, 30, 60, 120, 180, 360, 720, 1440, 2880):
        if span_min / step <= 9:
            return float(step)
    return 4320.0


class GanttChart(QWidget):
    """Paint a fit-to-width resource Gantt chart with per-task tooltips."""

    _LABEL_WIDTH = 104.0
    _RIGHT_MARGIN = 18.0
    _TOP_MARGIN = 42.0
    _LANE_HEIGHT = 18.0
    _ROW_GAP = 8.0
    _AXIS_HEIGHT = 32.0

    def __init__(self) -> None:
        super().__init__()
        self._rows: list[GanttRow] = []
        self._end_min = 0.0
        self._batch_colors: dict[str, QColor] = {}
        self._hit_boxes: list[tuple[QRectF, GanttBar]] = []
        self.setMinimumHeight(250)
        self.setMouseTracking(True)
        self.setAccessibleName('시뮬레이션 장비 점유 간트 차트')

    def set_timeline(
        self,
        timeline: list[dict[str, Any]],
        end_min: float = 0.0,
    ) -> None:
        self._rows = build_gantt_rows(timeline)
        maximum = max(
            (bar.end_min for row in self._rows for bar in row.bars),
            default=0.0,
        )
        self._end_min = max(float(end_min or 0.0), maximum)
        batches = sorted({
            bar.batch_id
            for row in self._rows
            for bar in row.bars
        })
        self._batch_colors = {
            batch: QColor(_BATCH_COLORS[index % len(_BATCH_COLORS)])
            for index, batch in enumerate(batches)
        }
        self.setMinimumHeight(max(250, int(self._chart_height())))
        self.updateGeometry()
        self.update()

    def sizeHint(self) -> QSize:  # noqa: N802 - Qt API
        return QSize(860, max(250, int(self._chart_height())))

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt API
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor('#FFFFFF'))
        self._hit_boxes = []
        if not self._rows:
            painter.setPen(QColor('#6B7684'))
            painter.drawText(
                self.rect(),
                int(Qt.AlignCenter),
                'Sim을 실행하면 장비별 간트 차트가 표시됩니다.',
            )
            return

        left = self._LABEL_WIDTH
        right = max(left + 1.0, self.width() - self._RIGHT_MARGIN)
        span = max(self._end_min, 60.0)
        pixels_per_minute = (right - left) / span
        self._paint_legend(painter, left, right)

        y = self._TOP_MARGIN
        for index, row in enumerate(self._rows):
            row_height = row.lane_count * self._LANE_HEIGHT
            background = QColor(
                '#F3F5F8' if index % 2 == 0 else '#FFFFFF'
            )
            painter.fillRect(
                QRectF(
                    0.0,
                    y - 2.0,
                    float(self.width()),
                    row_height + 4.0,
                ),
                background,
            )
            painter.setPen(QColor('#48525F'))
            label = _RESOURCE_LABELS.get(row.resource, row.resource)
            painter.drawText(
                QRectF(4.0, y, left - 12.0, row_height),
                int(Qt.AlignRight | Qt.AlignVCenter),
                label,
            )
            for bar in row.bars:
                self._paint_bar(
                    painter,
                    bar,
                    y,
                    left,
                    pixels_per_minute,
                )
            y += row_height + self._ROW_GAP

        axis_y = y + 2.0
        step = _tick_step(span)
        painter.setPen(QPen(QColor('#D2D8DF'), 1.0))
        tick = 0.0
        while tick <= span + 0.001:
            x = left + tick * pixels_per_minute
            painter.drawLine(
                int(x),
                int(self._TOP_MARGIN - 3.0),
                int(x),
                int(axis_y),
            )
            painter.setPen(QColor('#6B7684'))
            painter.drawText(
                QRectF(x - 28.0, axis_y + 4.0, 56.0, 18.0),
                int(Qt.AlignHCenter | Qt.AlignTop),
                f'{tick / 60:g}',
            )
            painter.setPen(QPen(QColor('#D2D8DF'), 1.0))
            tick += step
        painter.drawLine(int(left), int(axis_y), int(right), int(axis_y))
        painter.setPen(QColor('#6B7684'))
        painter.drawText(
            QRectF(right - 12.0, axis_y + 4.0, 28.0, 18.0),
            int(Qt.AlignRight | Qt.AlignTop),
            'h',
        )

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt API
        position = (
            event.position()
            if hasattr(event, 'position')
            else event.pos()
        )
        point = (
            position.toPoint()
            if hasattr(position, 'toPoint')
            else position
        )
        for rect, bar in reversed(self._hit_boxes):
            if rect.contains(point):
                global_position = (
                    event.globalPosition().toPoint()
                    if hasattr(event, 'globalPosition')
                    else event.globalPos()
                )
                QToolTip.showText(
                    global_position,
                    f'{bar.task_id} · {bar.kind}\n'
                    f'{bar.route}\n'
                    f'{_format_minutes(bar.start_min)} → '
                    f'{_format_minutes(bar.end_min)} '
                    f'(소요 {_format_minutes(bar.duration_min)})',
                    self,
                )
                return
        QToolTip.hideText()
        super().mouseMoveEvent(event)

    def leaveEvent(self, event) -> None:  # noqa: N802 - Qt API
        QToolTip.hideText()
        super().leaveEvent(event)

    def _chart_height(self) -> float:
        rows_height = sum(
            row.lane_count * self._LANE_HEIGHT + self._ROW_GAP
            for row in self._rows
        )
        return self._TOP_MARGIN + rows_height + self._AXIS_HEIGHT

    def _paint_legend(
        self,
        painter: QPainter,
        left: float,
        right: float,
    ) -> None:
        painter.setPen(QColor('#6B7684'))
        painter.drawText(
            QRectF(4.0, 8.0, left - 12.0, 18.0),
            int(Qt.AlignRight | Qt.AlignVCenter),
            '배치',
        )
        x = left
        for batch, color in list(self._batch_colors.items())[:12]:
            item_width = max(
                62.0,
                painter.fontMetrics().horizontalAdvance(batch) + 26.0,
            )
            if x + item_width > right:
                break
            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawRoundedRect(
                QRectF(x, 11.0, 11.0, 11.0),
                2.0,
                2.0,
            )
            painter.setPen(QColor('#48525F'))
            painter.drawText(
                QRectF(x + 15.0, 7.0, item_width - 15.0, 20.0),
                int(Qt.AlignLeft | Qt.AlignVCenter),
                batch,
            )
            x += item_width
        painter.setPen(QColor('#6B7684'))
        painter.drawText(
            QRectF(left, 25.0, right - left, 16.0),
            int(Qt.AlignLeft | Qt.AlignVCenter),
            '점선 테두리 = 이송 · 막대에 마우스를 올리면 상세 표시',
        )

    def _paint_bar(
        self,
        painter: QPainter,
        bar: GanttBar,
        row_y: float,
        left: float,
        pixels_per_minute: float,
    ) -> None:
        x = left + bar.start_min * pixels_per_minute
        width = max(
            3.0,
            (bar.end_min - bar.start_min) * pixels_per_minute,
        )
        y = row_y + bar.lane * self._LANE_HEIGHT + 2.0
        rect = QRectF(x, y, width, self._LANE_HEIGHT - 4.0)
        color = self._batch_colors.get(
            bar.batch_id,
            QColor('#6B7684'),
        )
        painter.setBrush(color)
        pen = QPen(color.darker(125), 1.0)
        if bar.kind == 'transfer':
            pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.drawRoundedRect(rect, 2.5, 2.5)
        self._hit_boxes.append((
            rect.adjusted(-2.0, -2.0, 2.0, 2.0),
            bar,
        ))

        label = bar.task_id
        if width >= painter.fontMetrics().horizontalAdvance(label) + 10.0:
            painter.setPen(QColor('#FFFFFF'))
            painter.drawText(
                rect.adjusted(4.0, 0.0, -4.0, 0.0),
                int(Qt.AlignCenter),
                label,
            )
