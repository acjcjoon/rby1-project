"""Independent camera-aware planner node."""
from __future__ import annotations

from typing import Optional

from rclpy.node import Node

from .camera import CameraClient
from .control_client import ControlClient
from .navigation_client import NavigationClient
from .task_commands import RunnableTaskDefinition
from .task_lifecycle import TaskProgress, TaskRunResult
from .task_runner import PlannerTaskRunner
from .transfer_action_server import TransferActionServer


# camera_source -> (detections 토픽, 가공 태그 TF prefix)
CAMERA_DEFAULTS = {
    'd405_right': ('/d405_right/detections', 'right_tag_pnp_'),
    'd405_left': ('/d405_left/detections', 'left_tag_pnp_'),
    'd435': ('/d435/detections', 'd435_tag_pnp_'),
}


class PlannerNode(Node):
    """Plan tasks while composing an independent control protocol client.

    Construction only creates ROS publishers/subscribers and timers. It does
    not prepare the robot, enable its stream, or send any motion command.
    """

    def __init__(
        self,
        *,
        namespace: Optional[str] = 'rby1',
        tf_buffer=None,
    ) -> None:
        super().__init__('rby1_planner', namespace=namespace)
        self.control = ControlClient(self)

        # camera_source -> (detections 토픽, 가공 태그 TF prefix) 기본값.
        # rby1_camera/config/camera_system.yaml 의 카메라별 블록과 짝을 이룬다.
        for source, (topic, prefix) in CAMERA_DEFAULTS.items():
            self.declare_parameter(f'{source}_object_pose_topic', topic)
            self.declare_parameter(
                f'{source}_processed_tag_tf_prefix', prefix
            )
        self.declare_parameter('object_pose_target_frame', 'base')
        self.declare_parameter('object_pose_max_age_sec', 0.5)
        self.declare_parameter('object_pose_min_confidence', 0.5)
        self.declare_parameter('object_pose_future_tolerance_sec', 0.05)
        self.declare_parameter('object_pose_tf_timeout_sec', 0.0)
        self.declare_parameter('object_pose_average_enabled', True)
        self.declare_parameter('object_pose_average_sample_count', 10)
        self.declare_parameter('object_pose_average_outlier_count', 2)
        self.declare_parameter(
            'navigation_command_topic',
            '/rby1/navigation/command',
        )
        self.declare_parameter(
            'navigation_state_topic',
            '/rby1/navigation/state',
        )
        self.declare_parameter(
            'transfer_action_name',
            'planner/execute_transfer',
        )

        shared_camera_options = {
            'target_frame': str(
                self.get_parameter('object_pose_target_frame').value
            ),
            'max_age_sec': float(
                self.get_parameter('object_pose_max_age_sec').value
            ),
            'minimum_confidence': float(
                self.get_parameter('object_pose_min_confidence').value
            ),
            'future_tolerance_sec': float(
                self.get_parameter(
                    'object_pose_future_tolerance_sec'
                ).value
            ),
            'tf_timeout_sec': float(
                self.get_parameter('object_pose_tf_timeout_sec').value
            ),
        }
        # 모든 카메라가 하나의 TF buffer 를 공유한다.
        self.cameras = {}
        for source in CAMERA_DEFAULTS:
            self.cameras[source] = CameraClient(
                self,
                object_pose_topic=str(
                    self.get_parameter(f'{source}_object_pose_topic').value
                ),
                processed_tag_tf_prefix=str(
                    self.get_parameter(
                        f'{source}_processed_tag_tf_prefix'
                    ).value
                ),
                tf_buffer=tf_buffer,
                **shared_camera_options,
            )
            tf_buffer = self.cameras[source].tf_buffer
        # camera_source 를 지정하지 않은 레거시 경로의 기본 카메라 (오른손 D405).
        self.camera = self.cameras['d405_right']
        self.navigation_client = NavigationClient(
            self,
            command_topic=str(
                self.get_parameter('navigation_command_topic').value
            ),
            state_topic=str(
                self.get_parameter('navigation_state_topic').value
            ),
        )
        self.transfer_action_server = None
        self.task_runner = PlannerTaskRunner(
            self,
            self.control,
            camera=self.camera,
            cameras=self.cameras,
            navigation=self.navigation_client,
            on_status=self._task_status,
            on_progress=self._task_progress,
            on_finished=self._task_finished,
            camera_average_enabled=bool(
                self.get_parameter('object_pose_average_enabled').value
            ),
            camera_average_sample_count=int(
                self.get_parameter(
                    'object_pose_average_sample_count'
                ).value
            ),
            camera_average_outlier_count=int(
                self.get_parameter(
                    'object_pose_average_outlier_count'
                ).value
            ),
        )
        self.transfer_action_server = TransferActionServer(
            self,
            self.task_runner,
            str(self.get_parameter('transfer_action_name').value),
        )
        self.get_logger().info(
            'Planner is ready and idle; transfer Action server is available.'
        )

    def run_task(self, task: RunnableTaskDefinition) -> None:
        """Start one explicitly supplied Task through the control backend."""

        self.task_runner.start(task)

    def stop_task(self, reason: str = 'Planner Task stopped') -> None:
        self.task_runner.stop(reason)

    def close(self) -> None:
        """Cancel planner work and request a motion-safe backend shutdown."""

        try:
            self.task_runner.close()
        finally:
            if self.transfer_action_server is not None:
                self.transfer_action_server.destroy()
                self.transfer_action_server = None
            # This stops base/manipulator motion and turns Stream off, but does
            # not power off the robot or disable its servos.
            self.control.shutdown_safely(turn_stream_off=True)

    def __getattr__(self, name):
        """Expose the composed control API to the local operator UI."""

        control = self.__dict__.get('control')
        if control is not None and (
            name in control.__dict__
            or any(name in cls.__dict__ for cls in type(control).__mro__)
        ):
            return object.__getattribute__(control, name)
        raise AttributeError(name)

    def _task_status(self, message: str) -> None:
        self.get_logger().info(message)

    def _task_progress(self, progress: TaskProgress) -> None:
        if self.transfer_action_server is not None:
            self.transfer_action_server.on_progress(progress)

    def _task_finished(self, result: TaskRunResult) -> None:
        if self.transfer_action_server is not None:
            self.transfer_action_server.on_finished(result)


__all__ = ['PlannerNode']
