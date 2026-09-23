"""Qt-free sequential Task execution driven by an ROS node timer."""
from __future__ import annotations

from dataclasses import replace
import math
import time
from typing import Callable, Optional

from .backend_contract import TaskBackendState, TaskCommandStatus

from .observation import (
    ObjectObservation,
    average_observations,
    compose_pose_right,
    compose_pose_yaw_only,
    invert_pose,
    observation_to_cartesian_target,
    rotate_vector,
    rpy_deg_to_quaternion,
)
from .navigation_protocol import (
    NavigationCommandState,
    NavigationCommandStatus,
)
from .task_commands import (
    CameraFrameLinearAbsoluteStep,
    CameraLinearAbsolutePrintStep,
    CameraLinearAbsoluteStep,
    CameraMoveToTagStep,
    CommandKind,
    DynamicTaskDefinition,
    MoveToStep,
    RewindStep,
    RunnableTaskDefinition,
    TaskCommand,
    TaskDefinition,
    list_sum,
)


class PlannerTaskRunner:
    """Run validated Task commands without blocking the ROS executor."""

    STATE_MAX_AGE_SEC = 1.0
    MOTION_TIMEOUT_MIN_SEC = 10.0
    CARTESIAN_TIMEOUT_MIN_SEC = 30.0
    MOTION_TIMEOUT_SCALE = 3.0
    MOTION_TIMEOUT_MARGIN_SEC = 5.0
    CAMERA_IDLE_TIMEOUT_SEC = 3.0
    NAVIGATION_FINALIZATION_MARGIN_SEC = 5.0

    def __init__(
        self,
        node,
        backend,
        *,
        camera=None,
        cameras=None,
        navigation=None,
        on_status: Optional[Callable[[str], None]] = None,
        on_active_changed: Optional[Callable[[bool], None]] = None,
        clock: Callable[[], float] = time.monotonic,
        period_sec: float = 0.05,
        camera_average_enabled: bool = False,
        camera_average_sample_count: int = 10,
        camera_average_outlier_count: int = 2,
    ) -> None:
        if not math.isfinite(float(period_sec)) or float(period_sec) <= 0.0:
            raise ValueError('period_sec must be a positive finite number')
        if not isinstance(camera_average_enabled, bool):
            raise ValueError('camera_average_enabled must be a boolean')
        if (
            isinstance(camera_average_sample_count, bool)
            or int(camera_average_sample_count) != camera_average_sample_count
            or int(camera_average_sample_count) < 1
        ):
            raise ValueError(
                'camera_average_sample_count must be a positive integer'
            )
        if (
            isinstance(camera_average_outlier_count, bool)
            or int(camera_average_outlier_count)
            != camera_average_outlier_count
            or int(camera_average_outlier_count) < 0
            or int(camera_average_outlier_count)
            >= int(camera_average_sample_count)
        ):
            raise ValueError(
                'camera_average_outlier_count must be a nonnegative integer '
                'smaller than camera_average_sample_count'
            )
        self.backend = backend
        self.cameras = dict(cameras or {})
        if camera is not None:
            self.cameras.setdefault('d405', camera)
        self.camera = self.cameras.get('d405', camera)
        self.navigation = navigation
        self.camera_average_enabled = camera_average_enabled
        self.camera_average_sample_count = int(camera_average_sample_count)
        self.camera_average_outlier_count = int(camera_average_outlier_count)
        self.on_status = on_status or (lambda _message: None)
        self.on_active_changed = on_active_changed or (lambda _active: None)
        self.clock = clock
        self._timer = node.create_timer(float(period_sec), self.tick)
        self._timer.cancel()
        self._task: Optional[RunnableTaskDefinition] = None
        self._index = 0
        self._command_id: Optional[str] = None
        self._active_command: Optional[TaskCommand] = None
        self._command_deadline = 0.0
        self._delay_deadline: Optional[float] = None
        self._base_deadline: Optional[float] = None
        self._base_velocity = None
        self._navigation_command_id: Optional[str] = None
        self._navigation_deadline = 0.0
        self._navigation_last_report = None
        self._stream_target: Optional[bool] = None
        self._stream_deadline = 0.0
        self._camera_marker_ns: Optional[int] = None
        self._camera_deadline: Optional[float] = None
        self._camera_idle_after_state_at: Optional[float] = None
        self._camera_idle_deadline: Optional[float] = None
        self._camera_observations: list[ObjectObservation] = []
        self._last_camera_observation = None
        self._last_camera_source: Optional[str] = None
        self._last_camera_object_id: Optional[str] = None
        self._last_camera_selected_yaw: Optional[float] = None
        self._last_camera_yaw_symmetry_deg: Optional[float] = None
        self._pending_camera_navigation_goal = None
        self._run_number = 0

    @property
    def active(self) -> bool:
        return self._task is not None

    @property
    def task_name(self) -> str:
        return self._task.name if self._task is not None else ''

    def start(self, task: RunnableTaskDefinition) -> None:
        if self.active:
            raise RuntimeError('another Task is already running')
        if not isinstance(task, (TaskDefinition, DynamicTaskDefinition)):
            raise TypeError('task must be a planner Task definition')
        if not task.commands:
            raise ValueError('Task has no commands')
        self._require_safe_state(self.backend.task_state(), require_idle=True)
        self._task = task
        self._index = 0
        self._command_id = None
        self._active_command = None
        self._command_deadline = 0.0
        self._delay_deadline = None
        self._base_deadline = None
        self._base_velocity = None
        self._navigation_command_id = None
        self._navigation_deadline = 0.0
        self._navigation_last_report = None
        self._stream_target = None
        self._stream_deadline = 0.0
        self._camera_marker_ns = None
        self._camera_deadline = None
        self._camera_idle_after_state_at = None
        self._camera_idle_deadline = None
        self._camera_observations = []
        self._last_camera_observation = None
        self._last_camera_source = None
        self._last_camera_object_id = None
        self._last_camera_selected_yaw = None
        self._last_camera_yaw_symmetry_deg = None
        self._pending_camera_navigation_goal = None
        self._run_number = 1
        self.on_active_changed(True)
        self.on_status(f'Task started: {task.name}')
        self._timer.reset()
        self.tick()

    def stop(self, reason: str = 'Task stopped') -> None:
        if not self.active:
            return
        try:
            self._cancel_active_motion()
        except Exception as exc:
            self.on_status(f'Task cancel warning: {exc}')
        finally:
            self._finish()
        self.on_status(reason)

    def close(self) -> None:
        if self.active:
            self.stop('Task runner closed')
        self._timer.cancel()

    def tick(self) -> None:
        if self._task is None:
            return
        try:
            state = self.backend.task_state()
            self._require_safe_state(
                state,
                require_idle=(
                    self._command_id is None
                    and self._navigation_command_id is None
                    and self._camera_idle_deadline is None
                ),
            )
            now = self.clock()

            if self._camera_idle_deadline is not None:
                if not self._tick_camera_idle_barrier(state, now):
                    return

            if self._stream_target is not None:
                if state.stream_enabled is self._stream_target:
                    self._stream_target = None
                elif now > self._stream_deadline:
                    raise RuntimeError('Stream state transition timed out')
                else:
                    return

            if self._base_deadline is not None:
                self._tick_base(now)
                return

            if self._navigation_command_id is not None:
                self._tick_navigation(now)
                return

            if self._delay_deadline is not None:
                if now < self._delay_deadline:
                    return
                self._delay_deadline = None
                self._complete_step()

            if self._command_id is not None:
                command_state = self.backend.poll_task_command(self._command_id)
                if command_state.status is TaskCommandStatus.PENDING:
                    if now > self._command_deadline:
                        raise RuntimeError(
                            'motion command timed out'
                            + self._timeout_detail(state)
                        )
                    return
                if command_state.status is not TaskCommandStatus.SUCCEEDED:
                    detail = command_state.message or command_state.status.value
                    raise RuntimeError(
                        f'motion command did not complete: {detail}'
                    )
                self._log_camera_target_and_actual(state)
                self._command_id = None
                self._active_command = None
                self._complete_step()
                if self._next_step_is_camera():
                    self._begin_camera_idle_barrier(state, now)
                    return

            if self._index >= len(self._task.commands):
                name = self._task.name
                self._finish()
                self.on_status(f'Task completed: {name}')
                return

            command = self._task.commands[self._index]

            if isinstance(command, RewindStep):
                self._rewind(command)
                return

            if isinstance(command, CameraMoveToTagStep):
                self._tick_camera_move_to_tag(command, state, now)
                return

            # The stock driver reports a streamed Cartesian command as kOk
            # after minimum_time without verifying target convergence. Task
            # motions therefore always use the driver's blocking command path.
            if (
                not self._step_can_run_with_stream(command)
                and state.stream_enabled is not False
            ):
                self._request_stream_transition(False, now)
                return

            if isinstance(command, CameraLinearAbsoluteStep):
                self._tick_camera_linear_absolute(command, state, now)
                return

            if isinstance(command, MoveToStep):
                if self.navigation is None:
                    raise RuntimeError('navigation client is unavailable')
                if state.stream_enabled is not True:
                    self._request_stream_transition(True, now)
                    return
                self._navigation_command_id = self.navigation.send_move_to(
                    command.x,
                    command.y,
                    command.yaw,
                    command.timeout_sec,
                )
                self._navigation_deadline = (
                    now
                    + command.timeout_sec
                    + self.NAVIGATION_FINALIZATION_MARGIN_SEC
                )
                self._navigation_last_report = None
                self.on_status(
                    f'Task step {self._index + 1}/{len(self._task.commands)}: '
                    f'move_to x={command.x:.3f} m, y={command.y:.3f} m, '
                    f'yaw={command.yaw:.3f} rad'
                )
                return

            if command.kind is CommandKind.BASE_VELOCITY:
                if state.stream_enabled is not True:
                    self._request_stream_transition(True, now)
                    return
                self._base_velocity = command.values
                self._base_deadline = now + float(command.seconds or 0.0)
                self.backend.set_velocity(*command.values)
                self.on_status(
                    f'Task step {self._index + 1}/{len(self._task.commands)}: '
                    f'base_velocity {float(command.seconds or 0.0):.3f}s'
                )
                return

            if command.kind is CommandKind.DELAY:
                self._delay_deadline = now + float(command.seconds or 0.0)
                self.on_status(
                    f'Task step {self._index + 1}/{len(self._task.commands)}: '
                    f'delay {float(command.seconds or 0.0):.3f}s'
                )
                return

            resolved = self._resolve(command, state)
            self._command_id = self.backend.start_task_command(resolved)
            self._active_command = resolved
            self._command_deadline = now + self._command_timeout_seconds(
                resolved,
                state,
            )
            self.on_status(
                f'Task step {self._index + 1}/{len(self._task.commands)}: '
                f'{command.kind.value}'
            )
        except Exception as exc:
            self._fail(str(exc))

    def _tick_camera_linear_absolute(
        self,
        command: CameraLinearAbsoluteStep,
        state: TaskBackendState,
        now: float,
    ) -> None:
        step_name = (
            'camera_linear_absolute_print'
            if isinstance(command, CameraLinearAbsolutePrintStep)
            else 'camera_linear_absolute'
        )
        if command.reuse_previous_observation:
            if (
                self._last_camera_observation is None
                or self._last_camera_source != command.camera_source
                or self._last_camera_object_id != command.object_id
                or self._task is None
                or self._index == 0
                or not isinstance(
                    self._task.commands[self._index - 1],
                    CameraLinearAbsoluteStep,
                )
            ):
                raise RuntimeError(
                    'camera observation reuse requires an immediately '
                    'preceding camera step for the same object'
                )
            observation = self._last_camera_observation
        else:
            camera = self._camera_for(command.camera_source)
            observation = self._collect_camera_observation(
                command,
                camera,
                step_name,
                now,
            )
            if observation is None:
                return

        if isinstance(command, CameraFrameLinearAbsoluteStep):
            resolved = self._resolve_camera_frame_command(
                command,
                observation,
                state,
            )
        else:
            resolved = command.resolve_observation(observation)
        resolved = self._apply_camera_yaw_symmetry(command, resolved, state)
        self._camera_marker_ns = None
        self._camera_deadline = None

        if isinstance(command, CameraLinearAbsolutePrintStep):
            self.on_status(
                f'CAM : {self._format_tcp_values(resolved.values)}'
            )
            self._complete_step()
            return

        self._command_id = self.backend.start_task_command(resolved)
        self._active_command = resolved
        self._command_deadline = now + self._command_timeout_seconds(
            resolved,
            state,
        )
        self.on_status(
            f'Task step {self._index + 1}/{len(self._task.commands)}: '
            f'camera_linear_absolute [{command.camera_source}] resolved '
            f'{command.object_id!r}'
        )

    def _tick_camera_move_to_tag(
        self,
        command: CameraMoveToTagStep,
        state: TaskBackendState,
        now: float,
    ) -> None:
        if self.navigation is None:
            raise RuntimeError('navigation client is unavailable')

        if self._pending_camera_navigation_goal is None:
            if state.stream_enabled is not False:
                self._request_stream_transition(False, now)
                return
            camera = self._camera_for(command.camera_source)
            observation = self._collect_camera_observation(
                command,
                camera,
                'camera_move_to_tag',
                now,
            )
            if observation is None:
                return

            yaw = command.relative_yaw
            cosine = math.cos(yaw)
            sine = math.sin(yaw)
            desired_x = command.desired_tag_x
            desired_y = command.desired_tag_y
            rotated_desired_x = cosine * desired_x - sine * desired_y
            rotated_desired_y = sine * desired_x + cosine * desired_y
            move_x = observation.position[0] - rotated_desired_x
            move_y = observation.position[1] - rotated_desired_y
            translation = math.hypot(move_x, move_y)
            if translation > command.max_translation_m:
                raise RuntimeError(
                    'camera-derived base translation exceeds safety limit: '
                    f'{translation:.3f} m > {command.max_translation_m:.3f} m'
                )
            self._pending_camera_navigation_goal = (
                move_x,
                move_y,
                yaw,
                command.navigation_timeout_sec,
            )
            self._camera_marker_ns = None
            self._camera_deadline = None
            self._camera_observations = []
            self.on_status(
                f'Camera base goal [{command.camera_source}] '
                f'{command.object_id!r}: tag=('
                f'{observation.position[0]:.3f}, '
                f'{observation.position[1]:.3f}) m, move=('
                f'{move_x:.3f}, {move_y:.3f}, {yaw:.3f})'
            )

        if state.stream_enabled is not True:
            self._request_stream_transition(True, now)
            return

        move_x, move_y, yaw, timeout_sec = (
            self._pending_camera_navigation_goal
        )
        self._navigation_command_id = self.navigation.send_move_to(
            move_x,
            move_y,
            yaw,
            timeout_sec,
        )
        self._navigation_deadline = (
            now + timeout_sec + self.NAVIGATION_FINALIZATION_MARGIN_SEC
        )
        self._navigation_last_report = None
        self._pending_camera_navigation_goal = None
        self.on_status(
            f'Task step {self._index + 1}/{len(self._task.commands)}: '
            f'camera_move_to_tag x={move_x:.3f} m, y={move_y:.3f} m, '
            f'yaw={yaw:.3f} rad'
        )

    def _collect_camera_observation(
        self,
        command,
        camera,
        step_name: str,
        now: float,
    ) -> Optional[ObjectObservation]:
        if self._camera_marker_ns is None:
            capture_marker = getattr(camera, 'capture_marker', None)
            if not callable(capture_marker):
                raise RuntimeError('camera must provide capture_marker()')
            marker = capture_marker()
            if isinstance(marker, bool):
                raise RuntimeError(
                    'camera capture marker must be a nonnegative integer'
                )
            try:
                marker_ns = int(marker)
            except (TypeError, ValueError) as exc:
                raise RuntimeError(
                    'camera capture marker must be a nonnegative integer'
                ) from exc
            if marker_ns < 0:
                raise RuntimeError(
                    'camera capture marker must be a nonnegative integer'
                )
            self._camera_marker_ns = marker_ns
            self._camera_deadline = now + command.detection_timeout_sec
            self._camera_observations = []
            sample_count = self._camera_sample_count()
            outlier_count = self._camera_outlier_count()
            self.on_status(
                f'Task step {self._index + 1}/{len(self._task.commands)}: '
                f'{step_name} [{command.camera_source}] waiting for new '
                f'{command.object_id!r} detections '
                f'(samples={sample_count}, outliers={outlier_count})'
            )
            return None

        getter = getattr(camera, 'get_observation', None)
        if not callable(getter):
            raise RuntimeError(
                'camera must provide get_observation(object_id, ...)'
            )
        newer_than_ns = (
            self._camera_observations[-1].stamp_ns
            if self._camera_observations
            else self._camera_marker_ns
        )
        query: dict[str, object] = {
            'target_frame': 'base',
            'newer_than_ns': newer_than_ns,
        }
        if command.max_age_sec is not None:
            query['max_age_sec'] = command.max_age_sec
        if command.minimum_confidence is not None:
            query['minimum_confidence'] = command.minimum_confidence
        observation = getter(command.object_id, **query)

        if observation is None:
            if now >= float(self._camera_deadline):
                detail = self._camera_unavailable_reason(
                    camera,
                    command.object_id,
                )
                suffix = f': {detail}' if detail else ''
                raise RuntimeError(
                    f'camera [{command.camera_source}] detection timed out '
                    f'for {command.object_id!r} after '
                    f'{command.detection_timeout_sec:.3f}s{suffix}'
                )
            return None
        if not isinstance(observation, ObjectObservation):
            raise TypeError('camera must return an ObjectObservation')
        if observation.stamp_ns <= int(newer_than_ns):
            if now >= float(self._camera_deadline):
                raise RuntimeError(
                    f'camera [{command.camera_source}] detection timed out '
                    f'for {command.object_id!r} after '
                    f'{command.detection_timeout_sec:.3f}s: '
                    'camera returned no distinct new frame'
                )
            return None
        self._camera_observations.append(observation)
        sample_count = self._camera_sample_count()
        if len(self._camera_observations) < sample_count:
            return None

        result = average_observations(
            self._camera_observations,
            outlier_count=self._camera_outlier_count(),
        )
        if self.camera_average_enabled:
            self.on_status(
                f'Camera average [{command.camera_source}] '
                f'{command.object_id!r}: collected={sample_count}, '
                f'kept={sample_count - self._camera_outlier_count()}, '
                f'discarded={self._camera_outlier_count()}'
            )
        self._last_camera_observation = result
        self._last_camera_source = command.camera_source
        self._last_camera_object_id = command.object_id
        return result

    def _resolve_camera_frame_command(
        self,
        command: CameraFrameLinearAbsoluteStep,
        observation: ObjectObservation,
        state: TaskBackendState,
    ) -> TaskCommand:
        compose = (
            compose_pose_yaw_only
            if command.yaw_only
            else compose_pose_right
        )
        controlled_position, controlled_orientation = compose(
            observation.position,
            observation.orientation_xyzw,
            command.object_to_controlled_frame_position,
            command.object_to_controlled_frame_orientation_xyzw,
        )
        ee_frame = {
            'left_arm': 'ee_left',
            'right_arm': 'ee_right',
        }.get(command.group)
        if ee_frame is None:
            raise RuntimeError(
                f'camera-frame control does not support {command.group!r}'
            )
        camera = self._camera_for(command.camera_source)
        lookup = getattr(camera, 'lookup_frame_pose', None)
        if not callable(lookup):
            raise RuntimeError('camera must provide lookup_frame_pose()')
        ee_to_controlled_position, ee_to_controlled_orientation = lookup(
            ee_frame,
            command.controlled_frame,
        )

        if command.preserve_end_effector_orientation:
            current = self._fresh_values(
                state.cartesian.get(command.group),
                state.cartesian_updated_at.get(command.group),
                state.captured_at,
                6,
            )
            if current is None:
                raise RuntimeError(
                    'fresh Cartesian state is required to preserve EE '
                    'orientation'
                )
            ee_orientation = rpy_deg_to_quaternion(current[3:6])
            mount_offset = rotate_vector(
                ee_to_controlled_position,
                ee_orientation,
            )
            ee_position = tuple(
                target - offset
                for target, offset in zip(
                    controlled_position,
                    mount_offset,
                )
            )
        else:
            controlled_to_ee = invert_pose(
                ee_to_controlled_position,
                ee_to_controlled_orientation,
            )
            ee_position, ee_orientation = compose_pose_right(
                controlled_position,
                controlled_orientation,
                *controlled_to_ee,
            )

        target = observation_to_cartesian_target(
            replace(
                observation,
                position=ee_position,
                orientation_xyzw=ee_orientation,
            ),
            target_frame='base',
        )
        return command.resolve(target)

    def _camera_for(self, source: str):
        key = str(source).strip()
        camera = self.cameras.get(key)
        if camera is None:
            raise RuntimeError(f'camera source {key!r} is unavailable')
        return camera

    def _camera_sample_count(self) -> int:
        return (
            self.camera_average_sample_count
            if self.camera_average_enabled
            else 1
        )

    def _camera_outlier_count(self) -> int:
        return (
            self.camera_average_outlier_count
            if self.camera_average_enabled
            else 0
        )

    def _log_camera_target_and_actual(
        self,
        state: TaskBackendState,
    ) -> None:
        if self._task is None or self._active_command is None:
            return
        step = self._task.commands[self._index]
        if (
            not isinstance(step, CameraLinearAbsoluteStep)
            or not step.log_target_and_actual
        ):
            return
        self._log_target_and_actual(
            self._active_command.values,
            step.group,
            state,
        )

    def _log_target_and_actual(
        self,
        target,
        group: str,
        state: TaskBackendState,
    ) -> None:
        actual = self._fresh_values(
            state.cartesian.get(group),
            state.cartesian_updated_at.get(group),
            state.captured_at,
            len(target),
        )
        actual_text = (
            self._format_tcp_values(actual)
            if actual is not None
            else '[unavailable]'
        )
        self.on_status(
            f'CAM : {self._format_tcp_values(target)}\n'
            f'REAL: {actual_text}'
        )

    @staticmethod
    def _format_tcp_values(values) -> str:
        return '[' + ', '.join(f'{float(value):.6f}' for value in values) + ']'

    def _apply_camera_yaw_symmetry(
        self,
        command: CameraLinearAbsoluteStep,
        resolved: TaskCommand,
        state: TaskBackendState,
    ) -> TaskCommand:
        symmetry = command.yaw_symmetry_deg
        if symmetry is None:
            if not command.reuse_previous_observation:
                self._last_camera_selected_yaw = None
                self._last_camera_yaw_symmetry_deg = None
            return resolved

        current = self._fresh_values(
            state.cartesian.get(command.group),
            state.cartesian_updated_at.get(command.group),
            state.captured_at,
            6,
        )
        if current is None:
            raise RuntimeError(
                'fresh Cartesian state is required for camera yaw symmetry'
            )

        raw_yaw = float(resolved.values[5])
        current_yaw = float(current[5])
        if command.reuse_previous_observation:
            if (
                self._last_camera_selected_yaw is None
                or self._last_camera_yaw_symmetry_deg != symmetry
            ):
                raise RuntimeError(
                    'camera yaw symmetry reuse requires the preceding '
                    'camera step to use the same symmetry'
                )
            selected_yaw = self._last_camera_selected_yaw
            source = 'reused'
        else:
            delta = self._wrap_periodic_degrees(
                raw_yaw - current_yaw,
                symmetry,
            )
            selected_yaw = self._normalize_degrees(current_yaw + delta)
            self._last_camera_selected_yaw = selected_yaw
            self._last_camera_yaw_symmetry_deg = symmetry
            source = 'selected'

        candidate_count = int(round(360.0 / symmetry))
        candidates = sorted({
            round(
                self._normalize_degrees(raw_yaw + index * symmetry),
                9,
            )
            for index in range(candidate_count)
        })
        self.on_status(
            f'Camera yaw symmetry {command.object_id!r}: '
            f'current={current_yaw:+.3f} deg, raw={raw_yaw:+.3f} deg, '
            f'candidates={candidates}, selected={selected_yaw:+.3f} deg '
            f'({source}, period={symmetry:.3f} deg)'
        )
        return replace(
            resolved,
            values=(*resolved.values[:5], selected_yaw),
        )

    @staticmethod
    def _normalize_degrees(value: float) -> float:
        normalized = (float(value) + 180.0) % 360.0 - 180.0
        return 0.0 if abs(normalized) < 1.0e-12 else normalized

    @staticmethod
    def _wrap_periodic_degrees(value: float, period: float) -> float:
        half_period = float(period) * 0.5
        wrapped = (float(value) + half_period) % float(period) - half_period
        return 0.0 if abs(wrapped) < 1.0e-12 else wrapped

    def _next_step_is_camera(self) -> bool:
        return (
            self._task is not None
            and self._index < len(self._task.commands)
            and isinstance(
                self._task.commands[self._index],
                (CameraLinearAbsoluteStep, CameraMoveToTagStep),
            )
        )

    @staticmethod
    def _step_can_run_with_stream(command) -> bool:
        return (
            isinstance(command, MoveToStep)
            or (
                isinstance(command, TaskCommand)
                and command.kind in (
                    CommandKind.BASE_VELOCITY,
                    CommandKind.DELAY,
                )
            )
        )

    def _tick_navigation(self, now: float) -> None:
        if self.navigation is None or self._navigation_command_id is None:
            raise RuntimeError('active navigation command is missing')
        state = self.navigation.poll(self._navigation_command_id)
        if not isinstance(state, NavigationCommandState):
            raise RuntimeError('navigation returned an invalid state')

        if state.status in {
            NavigationCommandStatus.PENDING,
            NavigationCommandStatus.ACCEPTED,
            NavigationCommandStatus.RUNNING,
        }:
            if now > self._navigation_deadline:
                raise RuntimeError('navigation command timed out')
            report = (state.status, int(state.progress * 10.0))
            if report != self._navigation_last_report:
                self._navigation_last_report = report
                self.on_status(
                    f'Navigation {state.status.value}: '
                    f'{state.progress * 100.0:.0f}% {state.message}'.rstrip()
                )
            return

        command_id = self._navigation_command_id
        self._navigation_command_id = None
        self._navigation_deadline = 0.0
        self._navigation_last_report = None
        if state.status is not NavigationCommandStatus.SUCCEEDED:
            detail = state.message or state.status.value
            raise RuntimeError(
                f'navigation command {command_id} did not complete: {detail}'
            )
        self._complete_step()
        self._request_stream_transition(False, now)

    def _begin_camera_idle_barrier(
        self,
        state: TaskBackendState,
        now: float,
    ) -> None:
        self._camera_idle_after_state_at = state.robot_state_updated_at
        self._camera_idle_deadline = now + self.CAMERA_IDLE_TIMEOUT_SEC
        self.on_status(
            f'Task step {self._index + 1}/{len(self._task.commands)}: '
            'waiting for a fresh idle robot state before camera capture'
        )

    def _tick_camera_idle_barrier(
        self,
        state: TaskBackendState,
        now: float,
    ) -> bool:
        updated_at = state.robot_state_updated_at
        previous = self._camera_idle_after_state_at
        if (
            updated_at is not None
            and (previous is None or updated_at > previous)
            and state.motion_active is False
        ):
            self._camera_idle_after_state_at = None
            self._camera_idle_deadline = None
            return True
        if now >= float(self._camera_idle_deadline):
            raise RuntimeError(
                'robot did not report a fresh idle state before camera '
                'capture'
            )
        return False

    @staticmethod
    def _camera_unavailable_reason(camera, object_id: str) -> str:
        getter = getattr(camera, 'unavailable_reason', None)
        if not callable(getter):
            return ''
        try:
            return str(getter(object_id)).strip()
        except Exception:
            return ''

    def _tick_base(self, now: float) -> None:
        if self._base_velocity is None:
            raise RuntimeError('active base velocity is missing')
        if now < float(self._base_deadline):
            # Refresh faster than the control backend's cmd_vel stale watchdog.
            self.backend.set_velocity(*self._base_velocity)
            return
        self.backend.stop(publish_immediately=True)
        self._base_deadline = None
        self._base_velocity = None
        self._complete_step()
        self._request_stream_transition(False, now)

    def _request_stream_transition(self, enabled: bool, now: float) -> None:
        self.backend.request_stream(enabled)
        self._stream_target = enabled
        self._stream_deadline = now + 3.0

    def _rewind(self, command: RewindStep) -> None:
        if self._task is None:
            raise RuntimeError('rewind requires an active Task')
        repeat_count = command.repeat_count
        if repeat_count is not None and self._run_number >= repeat_count:
            self._complete_step()
            return

        self._run_number += 1
        self._index = 0
        self._camera_marker_ns = None
        self._camera_deadline = None
        self._camera_observations = []
        self._camera_idle_after_state_at = None
        self._camera_idle_deadline = None
        self._last_camera_observation = None
        self._last_camera_source = None
        self._last_camera_object_id = None
        self._last_camera_selected_yaw = None
        self._last_camera_yaw_symmetry_deg = None
        self._pending_camera_navigation_goal = None
        total = 'infinite' if repeat_count is None else str(repeat_count)
        self.on_status(
            f'Task rewind: {self._task.name} '
            f'run {self._run_number}/{total}'
        )

    def _complete_step(self) -> None:
        self._camera_marker_ns = None
        self._camera_deadline = None
        self._camera_observations = []
        self._camera_idle_after_state_at = None
        self._camera_idle_deadline = None
        self._pending_camera_navigation_goal = None
        self._index += 1
        if self._task is not None:
            self.on_status(
                f'Task step {self._index}/{len(self._task.commands)} completed'
            )

    def _cancel_active_motion(self) -> None:
        command_cancel_requested = False
        if self._navigation_command_id is not None:
            if self.navigation is None:
                raise RuntimeError('navigation client is unavailable')
            self.navigation.cancel(self._navigation_command_id)
            command_cancel_requested = True
        if self._command_id is not None:
            try:
                self.backend.cancel_task_command(self._command_id)
                command_cancel_requested = True
            except Exception as exc:
                self.on_status(f'Task cancel warning: {exc}')
        if self._base_deadline is not None or self._stream_target is True:
            self.backend.stop(publish_immediately=True)
            try:
                self.backend.request_stream(False)
            except Exception as exc:
                self.on_status(f'Stream stop warning: {exc}')
        if not command_cancel_requested:
            self.backend.cancel_motion()

    def _finish(self) -> None:
        was_active = self.active
        self._timer.cancel()
        self._task = None
        self._index = 0
        self._command_id = None
        self._active_command = None
        self._command_deadline = 0.0
        self._delay_deadline = None
        self._base_deadline = None
        self._base_velocity = None
        self._navigation_command_id = None
        self._navigation_deadline = 0.0
        self._navigation_last_report = None
        self._stream_target = None
        self._stream_deadline = 0.0
        self._camera_marker_ns = None
        self._camera_deadline = None
        self._camera_observations = []
        self._camera_idle_after_state_at = None
        self._camera_idle_deadline = None
        self._last_camera_observation = None
        self._last_camera_source = None
        self._last_camera_object_id = None
        self._last_camera_selected_yaw = None
        self._last_camera_yaw_symmetry_deg = None
        self._pending_camera_navigation_goal = None
        self._run_number = 0
        if was_active:
            self.on_active_changed(False)

    def _fail(self, message: str) -> None:
        name = self.task_name
        try:
            self._cancel_active_motion()
        except Exception as exc:
            self.on_status(f'Task cancel warning: {exc}')
        self._finish()
        self.on_status(f'Task failed ({name}): {message}')

    def _resolve(
        self,
        command: TaskCommand,
        state: TaskBackendState,
    ) -> TaskCommand:
        if command.kind is CommandKind.JOINT_RELATIVE:
            group = str(command.group)
            current = state.joint_groups.get(group)
            updated = state.joint_updated_at.get(group)
            if current is None or not state.joint_order_verified.get(group, False):
                raise RuntimeError('fresh, name-ordered joint state is required')
            self._require_fresh(updated, state.captured_at, 'joint state')
            return replace(
                command,
                kind=CommandKind.JOINT_ABSOLUTE,
                values=tuple(list_sum(current, command.values)),
            )
        if command.kind is CommandKind.LINEAR_RELATIVE:
            group = str(command.group)
            current = state.cartesian.get(group)
            updated = state.cartesian_updated_at.get(group)
            if current is None:
                raise RuntimeError('fresh Cartesian state is required')
            self._require_fresh(updated, state.captured_at, 'Cartesian state')
            return replace(
                command,
                kind=CommandKind.LINEAR_ABSOLUTE,
                values=tuple(list_sum(current, command.values)),
            )
        return command

    def _command_timeout_seconds(
        self,
        command: TaskCommand,
        state: TaskBackendState,
    ) -> float:
        expected = float(command.minimum_time or 0.0)
        if command.kind is CommandKind.JOINT_ABSOLUTE:
            current = self._fresh_values(
                state.joint_groups.get(str(command.group)),
                state.joint_updated_at.get(str(command.group)),
                state.captured_at,
                len(command.values),
            )
            expected = max(
                expected,
                self._joint_travel_seconds(
                    current,
                    command.values,
                    command.velocity_limit,
                ),
            )
        elif command.kind is CommandKind.JOINT_ABSOLUTE_MULTI:
            for group, target in command.joint_targets:
                current = self._fresh_values(
                    state.joint_groups.get(group),
                    state.joint_updated_at.get(group),
                    state.captured_at,
                    len(target),
                )
                expected = max(
                    expected,
                    self._joint_travel_seconds(
                        current,
                        target,
                        command.velocity_limit,
                    ),
                )
        elif command.kind is CommandKind.LINEAR_ABSOLUTE:
            current = self._fresh_values(
                state.cartesian.get(str(command.group)),
                state.cartesian_updated_at.get(str(command.group)),
                state.captured_at,
                len(command.values),
            )
            if current is not None:
                translation_distance = math.sqrt(sum(
                    (float(target) - float(start)) ** 2
                    for start, target in zip(current[:3], command.values[:3])
                ))
                rotation_distance = self._rotation_distance_rad(
                    current[3:6],
                    command.values[3:6],
                )
                expected = max(
                    expected,
                    translation_distance / float(command.linear_velocity),
                    rotation_distance / float(command.angular_velocity),
                )

        state_aware_timeout = (
            expected * self.MOTION_TIMEOUT_SCALE
            + self.MOTION_TIMEOUT_MARGIN_SEC
        )
        minimum_timeout = (
            self.CARTESIAN_TIMEOUT_MIN_SEC
            if command.kind is CommandKind.LINEAR_ABSOLUTE
            else self.MOTION_TIMEOUT_MIN_SEC
        )
        return max(
            minimum_timeout,
            command.timeout_seconds,
            state_aware_timeout,
        )

    def _timeout_detail(self, state: TaskBackendState) -> str:
        command = self._active_command
        if command is None or command.kind is not CommandKind.LINEAR_ABSOLUTE:
            return ''
        current = self._fresh_values(
            state.cartesian.get(str(command.group)),
            state.cartesian_updated_at.get(str(command.group)),
            state.captured_at,
            len(command.values),
        )
        if current is None:
            return '; current Cartesian pose is unavailable'
        position_error = math.sqrt(sum(
            (float(target) - float(start)) ** 2
            for start, target in zip(current[:3], command.values[:3])
        ))
        orientation_error = self._rotation_distance_rad(
            current[3:6],
            command.values[3:6],
        )
        return (
            f'; remaining position error={position_error:.6f} m, '
            f'orientation error={orientation_error:.6f} rad, '
            f'current TCP={[round(value, 6) for value in current]}, '
            f'target TCP={[round(value, 6) for value in command.values]}'
        )

    def _fresh_values(
        self,
        values,
        updated_at: Optional[float],
        captured_at: float,
        count: int,
    ):
        if values is None or len(values) != count or updated_at is None:
            return None
        age = captured_at - updated_at
        if not math.isfinite(age) or age < 0.0 or age > self.STATE_MAX_AGE_SEC:
            return None
        result = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in result):
            return None
        return result

    @staticmethod
    def _joint_travel_seconds(current, target, velocity_limit) -> float:
        if current is None or velocity_limit is None:
            return 0.0
        max_distance_rad = max(
            math.radians(abs(float(goal) - float(start)))
            for start, goal in zip(current, target)
        )
        return max_distance_rad / float(velocity_limit)

    @staticmethod
    def _rotation_distance_rad(current_rpy, target_rpy) -> float:
        def quaternion(rpy):
            roll, pitch, yaw = (math.radians(float(value)) for value in rpy)
            cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
            return (
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            )

        start_q = quaternion(current_rpy)
        target_q = quaternion(target_rpy)
        dot = abs(sum(a * b for a, b in zip(start_q, target_q)))
        return 2.0 * math.acos(max(0.0, min(1.0, dot)))

    def _require_safe_state(
        self,
        state: TaskBackendState,
        *,
        require_idle: bool,
    ) -> None:
        if not isinstance(state, TaskBackendState):
            raise RuntimeError('backend returned an invalid Task state')
        self._require_fresh(
            state.robot_state_updated_at,
            state.captured_at,
            'robot state',
        )
        if state.emo_active is not False:
            raise RuntimeError('EMO is active or unknown')
        if state.collision_active is not False:
            raise RuntimeError('collision state is active or unknown')
        if state.control_state not in (2, 3):
            raise RuntimeError('robot control state is not ENABLE/EXECUTING')
        if require_idle and state.motion_active:
            raise RuntimeError('another manipulation motion is active')

    @staticmethod
    def _require_fresh(
        updated_at: Optional[float],
        captured_at: float,
        label: str,
        maximum_age: float = STATE_MAX_AGE_SEC,
    ) -> None:
        if updated_at is None:
            raise RuntimeError(f'{label} is unavailable')
        age = captured_at - updated_at
        if not math.isfinite(age) or age < 0.0 or age > maximum_age:
            raise RuntimeError(f'{label} is stale')


TaskRunner = PlannerTaskRunner


__all__ = ['PlannerTaskRunner', 'TaskRunner']
