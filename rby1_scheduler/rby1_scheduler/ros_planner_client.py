"""Non-blocking ROS Action implementation of the scheduler planner boundary."""
from __future__ import annotations

import uuid

from action_msgs.msg import GoalStatus
from rby1_interface.action import ExecuteTransfer
from rclpy.action import ActionClient

from .models import WorkItem
from .planner_client import PlannerClient, PlannerReport


class RosPlannerClient(PlannerClient):
    """Expose an ActionClient through RuntimeEngine's polling interface."""

    def __init__(self, node, action_name: str) -> None:
        self._client = ActionClient(node, ExecuteTransfer, action_name)
        self._task: WorkItem | None = None
        self._mission_id = ''
        self._goal_handle = None
        self._cancel_requested = False
        self._report = PlannerReport()
        self._terminal: PlannerReport | None = None

    def start(self, task: WorkItem) -> str:
        if self._task is not None:
            raise RuntimeError('planner is already running')

        self._task = task
        self._mission_id = f'mission-{uuid.uuid4().hex[:12]}'
        self._goal_handle = None
        self._cancel_requested = False
        self._terminal = None
        self._report = PlannerReport(
            status='running',
            mission_id=self._mission_id,
            task_id=task.task_id,
            phase='connecting',
            progress=0.0,
            message='Waiting for planner Action server',
        )

        if not self._client.server_is_ready():
            self._set_terminal(
                'failed',
                'unavailable',
                'Planner Action server is unavailable',
            )
            return self._mission_id

        goal = ExecuteTransfer.Goal()
        goal.protocol_version = ExecuteTransfer.Goal.PROTOCOL_VERSION
        goal.task_id = task.task_id
        goal.plate_id = ''
        goal.barcode = ''
        goal.source = task.source
        goal.source_slot = ''
        goal.destination = task.destination
        goal.destination_slot = ''

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback,
        )
        future.add_done_callback(self._goal_response)
        return self._mission_id

    def poll(self) -> PlannerReport:
        if self._terminal is not None:
            report = self._terminal
            self._terminal = None
            self._task = None
            self._mission_id = ''
            self._goal_handle = None
            self._cancel_requested = False
            self._report = PlannerReport()
            return report
        if self._task is None:
            return PlannerReport()
        return self._report

    def cancel(self) -> None:
        if (
            self._task is None
            or self._terminal is not None
            or self._cancel_requested
        ):
            return
        self._cancel_requested = True
        self._report = PlannerReport(
            status='running',
            mission_id=self._mission_id,
            task_id=self._task.task_id,
            phase='canceling',
            progress=self._report.progress,
            message='Cancel requested by operator',
        )
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _feedback(self, feedback_message) -> None:
        if self._task is None or self._terminal is not None:
            return
        feedback = feedback_message.feedback
        self._report = PlannerReport(
            status='running',
            mission_id=self._mission_id,
            task_id=self._task.task_id,
            phase=feedback.phase,
            progress=max(0.0, min(1.0, float(feedback.progress))),
            message=feedback.message,
        )

    def _goal_response(self, future) -> None:
        if self._task is None or self._terminal is not None:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._set_terminal('failed', 'send_failed', str(exc))
            return
        if goal_handle is None or not goal_handle.accepted:
            self._set_terminal(
                'failed',
                'rejected',
                'Planner rejected the transfer goal',
            )
            return

        self._goal_handle = goal_handle
        if self._cancel_requested:
            goal_handle.cancel_goal_async()
        else:
            self._report = PlannerReport(
                status='running',
                mission_id=self._mission_id,
                task_id=self._task.task_id,
                phase='accepted',
                progress=self._report.progress,
                message='Planner accepted the transfer goal',
            )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result)

    def _result(self, future) -> None:
        if self._task is None or self._terminal is not None:
            return
        try:
            wrapped = future.result()
            action_result = wrapped.result
            status = wrapped.status
        except Exception as exc:
            self._set_terminal('failed', 'result_failed', str(exc))
            return

        message = action_result.message
        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and action_result.error_code == ExecuteTransfer.Result.OK
        ):
            self._set_terminal('succeeded', 'completed', message, 1.0)
        elif (
            status == GoalStatus.STATUS_CANCELED
            or action_result.error_code == ExecuteTransfer.Result.CANCELED
        ):
            self._set_terminal('canceled', 'canceled', message)
        else:
            self._set_terminal('failed', 'failed', message)

    def _set_terminal(
        self,
        status: str,
        phase: str,
        message: str,
        progress: float | None = None,
    ) -> None:
        if self._task is None:
            return
        self._terminal = PlannerReport(
            status=status,
            mission_id=self._mission_id,
            task_id=self._task.task_id,
            phase=phase,
            progress=(
                self._report.progress if progress is None else progress
            ),
            message=message,
        )


__all__ = ['RosPlannerClient']
