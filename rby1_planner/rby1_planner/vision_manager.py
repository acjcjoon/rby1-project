"""On-demand AprilTag detection processes, one pair per camera.

Each camera's ``apriltag_node`` + ``depth_tag_tf_node`` pair costs about one
CPU core while it runs. The planner therefore starts a pair only for Task
steps that read a tag through that camera and stops it afterwards. The camera
drivers and the camera mount TF publishers are not touched.

Modes
  auto  pairs run only while a Task step holds them (default)
  on    all pairs run permanently (tag viewing, offset measurement)
  off   nothing is started; camera steps fail with a clear message

A pair that is already running outside this manager (for example a manually
launched ``apriltag_vision.launch.py``) is used as it is and never stopped.
"""
from __future__ import annotations

from collections import deque
import os
import signal
import subprocess
import threading
import time
from typing import Callable, Iterable, Optional

from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


VISION_MODES = ('auto', 'on', 'off')
DETECTION_NODES = ('apriltag_node', 'depth_tag_tf_node')


def launch_command(camera: str) -> list[str]:
    """Command that starts only the detection pair of ``camera``."""

    return [
        'ros2', 'launch', 'rby1_camera', 'apriltag_vision.launch.py',
        'mount_tf:=false', f'detection_cameras:={camera}',
    ]


class VisionManager:
    STOP_GRACE_SEC = 5.0
    HEARTBEAT_MAX_AGE_SEC = 1.0
    LOG_LINES = 200

    def __init__(
        self,
        node,
        detection_topics: dict[str, str],
        *,
        mode: str = 'auto',
        clock: Callable[[], float] = time.monotonic,
        spawn: Optional[Callable[[str], object]] = None,
        subscribe: bool = True,
    ) -> None:
        mode = str(mode).strip().lower()
        if mode not in VISION_MODES:
            raise ValueError(f'tag detection mode must be one of {VISION_MODES}')
        if not detection_topics:
            raise ValueError('detection_topics must not be empty')
        self._node = node
        self._clock = clock
        self._spawn = spawn or self._spawn_process
        self._lock = threading.RLock()
        self._mode = mode
        self.cameras = tuple(detection_topics)
        self._processes: dict[str, object] = {}
        self._started_at: dict[str, float] = {}
        self._heartbeat: dict[str, float] = {}
        self._held: set[str] = set()
        self._log: deque[str] = deque(maxlen=self.LOG_LINES)
        if subscribe:
            qos = QoSProfile(depth=1)
            qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
            for camera, topic in detection_topics.items():
                node.create_subscription(
                    AprilTagDetectionArray,
                    topic,
                    lambda _message, key=camera: self._on_detections(key),
                    qos,
                )
        if self._mode == 'on':
            self._apply_mode()

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------
    @property
    def mode(self) -> str:
        return self._mode

    def set_mode(self, mode: str) -> None:
        mode = str(mode).strip().lower()
        if mode not in VISION_MODES:
            raise ValueError(f'tag detection mode must be one of {VISION_MODES}')
        with self._lock:
            if mode == 'off' and self._held:
                raise RuntimeError(
                    'a running Task is using tag detection; stop it first'
                )
            self._mode = mode
            self._apply_mode()

    def _apply_mode(self) -> None:
        if self._mode == 'on':
            for camera in self.cameras:
                self._start(camera)
        else:
            for camera in list(self._processes):
                if self._mode == 'off' or camera not in self._held:
                    self._stop(camera)

    # ------------------------------------------------------------------
    # Task runner API
    # ------------------------------------------------------------------
    def require(self, camera: str) -> None:
        """Make sure the detection pair of ``camera`` is running."""

        camera = self._known(camera)
        with self._lock:
            if self._mode == 'off':
                raise RuntimeError(
                    'tag detection is switched off (Scenario tab: Tag '
                    'detection); this step needs camera ' + camera
                )
            self._held.add(camera)
            self._start(camera)

    def ready(self, camera: str) -> tuple[bool, str]:
        """Detections are flowing and the tag TF node has joined the graph."""

        camera = self._known(camera)
        now = self._clock()
        with self._lock:
            process = self._processes.get(camera)
            if process is not None and process.poll() is not None:
                self._processes.pop(camera, None)
                raise RuntimeError(
                    f'tag detection for {camera} exited '
                    f'(code {process.returncode})'
                )
            started = self._started_at.get(camera, -float('inf'))
            heartbeat = self._heartbeat.get(camera, -float('inf'))
        if heartbeat <= started or now - heartbeat > self.HEARTBEAT_MAX_AGE_SEC:
            return False, 'waiting for detections from ' + camera
        if not self._tag_tf_node_present(camera):
            return False, f'waiting for the {camera} tag TF node'
        return True, ''

    def release_except(self, keep: Iterable[str]) -> None:
        """Drop Task holds on every camera not listed in ``keep``."""

        keep = {str(camera).strip() for camera in keep}
        with self._lock:
            for camera in sorted(self._held - keep):
                self._held.discard(camera)
                if self._mode == 'auto':
                    self._stop(camera)

    def release_all(self) -> None:
        self.release_except(())

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def running(self, camera: str) -> bool:
        with self._lock:
            process = self._processes.get(camera)
            return process is not None and process.poll() is None

    def status_text(self) -> str:
        parts = []
        for camera in self.cameras:
            if self.running(camera):
                state = 'on'
            elif self._externally_running(camera):
                state = 'external'
            else:
                state = 'off'
            parts.append(f'{camera}: {state}')
        return ', '.join(parts)

    def drain_log(self) -> list[str]:
        with self._lock:
            lines = list(self._log)
            self._log.clear()
        return lines

    def close(self) -> None:
        with self._lock:
            self._held.clear()
            processes = list(self._processes.items())
            self._processes.clear()
        for _camera, process in processes:
            self._signal(process, signal.SIGINT)
        deadline = time.monotonic() + self.STOP_GRACE_SEC
        for _camera, process in processes:
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                self._signal(process, signal.SIGKILL)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _known(self, camera: str) -> str:
        camera = str(camera).strip()
        if camera not in self.cameras:
            raise RuntimeError(f'camera source {camera!r} has no tag detection')
        return camera

    def _on_detections(self, camera: str) -> None:
        self._heartbeat[camera] = self._clock()

    def _graph_nodes(self, camera: str) -> set[str]:
        namespace = '/' + camera
        try:
            names = self._node.get_node_names_and_namespaces()
        except Exception:
            return set()
        return {name for name, ns in names if ns == namespace}

    def _tag_tf_node_present(self, camera: str) -> bool:
        return 'depth_tag_tf_node' in self._graph_nodes(camera)

    def _externally_running(self, camera: str) -> bool:
        return (
            not self.running(camera)
            and set(DETECTION_NODES) <= self._graph_nodes(camera)
        )

    def _start(self, camera: str) -> None:
        process = self._processes.get(camera)
        if process is not None and process.poll() is None:
            return
        if self._externally_running(camera):
            # Someone else owns this pair; a new detection proves it is live.
            self._started_at[camera] = self._clock()
            return
        self._started_at[camera] = self._clock()
        self._processes[camera] = self._spawn(camera)
        self._log.append(f'[vision] {camera}: tag detection started')

    def _stop(self, camera: str) -> None:
        process = self._processes.pop(camera, None)
        if process is None:
            return
        self._log.append(f'[vision] {camera}: tag detection stopped')
        if process.poll() is not None:
            return
        self._signal(process, signal.SIGINT)
        threading.Thread(
            target=self._reap,
            args=(process,),
            name=f'vision-stop-{camera}',
            daemon=True,
        ).start()

    def _reap(self, process) -> None:
        try:
            process.wait(timeout=self.STOP_GRACE_SEC)
        except subprocess.TimeoutExpired:
            self._signal(process, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._signal(process, signal.SIGKILL)

    @staticmethod
    def _signal(process, signum: int) -> None:
        try:
            os.killpg(process.pid, signum)
        except (ProcessLookupError, PermissionError, AttributeError):
            pass

    @staticmethod
    def _spawn_process(camera: str):
        return subprocess.Popen(
            launch_command(camera),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )


__all__ = ['VISION_MODES', 'VisionManager', 'launch_command']
