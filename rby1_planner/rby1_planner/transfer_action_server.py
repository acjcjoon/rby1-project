"""ROS Action boundary for scheduler-requested plate transfers."""
from __future__ import annotations

from typing import Optional

from rby1_interface.action import ExecuteTransfer
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from .task import pick_up_move_and_put_down_object
from .task_lifecycle import TaskProgress, TaskRunResult, TaskRunStatus


class TransferActionServer:
    """Translate one transfer goal into the existing real planner Task."""

    def __init__(self, node, runner, action_name: str) -> None:
        self._node = node
        self._runner = runner
        self._active_goal = None
        self._completion: Optional[Future] = None
        self._goal_reserved = False
        self._cancel_before_start = False
        self._server = ActionServer(
            node,
            ExecuteTransfer,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def destroy(self) -> None:
        self._server.destroy()

    def on_progress(self, progress: TaskProgress) -> None:
        goal = self._active_goal
        if goal is None or not goal.is_active:
            return
        feedback = ExecuteTransfer.Feedback()
        feedback.phase = progress.phase
        feedback.progress = float(progress.progress)
        feedback.message = progress.message
        goal.publish_feedback(feedback)

    def on_finished(self, result: TaskRunResult) -> None:
        completion = self._completion
        if completion is not None and not completion.done():
            completion.set_result(result)

    def _goal(self, request) -> GoalResponse:
        source = request.source.strip().upper()
        destination = request.destination.strip().upper()
        invalid = (
            request.protocol_version != ExecuteTransfer.Goal.PROTOCOL_VERSION
            or not request.task_id.strip()
            or not source
            or not destination
            or source == destination
        )
        busy = (
            self._goal_reserved
            or self._active_goal is not None
            or self._runner.active
        )
        if invalid or busy:
            reason = 'invalid transfer goal' if invalid else 'planner is busy'
            self._node.get_logger().warning(
                f'Transfer goal rejected: {reason}'
            )
            return GoalResponse.REJECT
        self._goal_reserved = True
        self._cancel_before_start = False
        return GoalResponse.ACCEPT

    def _cancel(self, goal_handle) -> CancelResponse:
        if self._active_goal is goal_handle:
            if self._runner.active:
                self._runner.stop('Transfer canceled by scheduler')
            return CancelResponse.ACCEPT
        if self._goal_reserved and self._active_goal is None:
            self._cancel_before_start = True
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    async def _execute(self, goal_handle):
        self._goal_reserved = False
        self._active_goal = goal_handle
        self._completion = Future()
        request = goal_handle.request
        source = request.source.strip().upper()
        destination = request.destination.strip().upper()

        try:
            if self._cancel_before_start or goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(
                    ExecuteTransfer.Result.CANCELED,
                    'Transfer canceled before execution',
                )

            try:
                task = pick_up_move_and_put_down_object(
                    source,
                    destination,
                )
            except Exception as exc:
                goal_handle.abort()
                return self._make_result(
                    ExecuteTransfer.Result.INVALID_GOAL,
                    str(exc),
                )

            try:
                self._runner.start(task)
            except Exception as exc:
                goal_handle.abort()
                return self._make_result(
                    ExecuteTransfer.Result.EXECUTION_FAILED,
                    str(exc),
                )

            outcome = await self._completion
            if outcome.status is TaskRunStatus.SUCCEEDED:
                goal_handle.succeed()
                return self._make_result(
                    ExecuteTransfer.Result.OK,
                    outcome.message,
                )
            if outcome.status is TaskRunStatus.CANCELED:
                goal_handle.canceled()
                return self._make_result(
                    ExecuteTransfer.Result.CANCELED,
                    outcome.message,
                )
            goal_handle.abort()
            return self._make_result(
                ExecuteTransfer.Result.EXECUTION_FAILED,
                outcome.message,
            )
        finally:
            self._active_goal = None
            self._completion = None
            self._cancel_before_start = False

    @staticmethod
    def _make_result(error_code: int, message: str):
        result = ExecuteTransfer.Result()
        result.error_code = int(error_code)
        result.message = message
        return result


__all__ = ['TransferActionServer']
