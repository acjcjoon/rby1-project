"""Planner-side client for the cuVSLAM stack that runs in the Isaac container.

The camera stays with the planner launcher (one D435 node publishes colour for
AprilTag and the IR pair for cuVSLAM). This client only
  * starts/stops ``jetson.launch.py`` inside the container,
  * mirrors the tracking / localization status topics,
  * requests station corrections and visual-reference recovery,
  * runs ``map_tool`` (save / localize) inside the container, and
  * stores named points in the rby1_vslam waypoint file.

Nothing here publishes base velocity. nvblox and Nav2 are not started.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
from typing import Callable, Optional

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


VSLAM_MODES = ('mapping', 'localization')
MAP_NAME_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')
CONTAINER_LAUNCH_PATTERN = 'ros2 launch rby1_vslam jetson.launch.py'
STATUS_TOPICS = {
    'tracking': '/rby1/vslam/tracking_status',
    'adapter': '/rby1/vslam/pose_adapter_status',
    'localization': '/rby1/vslam/localization_status',
    'navigation': '/rby1/vslam/navigation_status',
}
# Live pose sources shown in the VSLAM tab (same three as the station UI).
POSE_TOPICS = {
    'Camera VO': '/rby1/vslam/camera_odometry',
    'Base VO': '/rby1/vslam/odom',
    'Localization': '/rby1/vslam/localization_odom',
}
GOAL_STATUS_NAMES = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
CONTAINER_SETUP = (
    'source /opt/ros/humble/setup.bash && '
    'source /workspaces/isaac_ros-dev/install/setup.bash && '
)


def validate_map_name(name: object) -> str:
    """Return a map folder name that is safe inside the map root."""

    text = str(name or '').strip()
    if not MAP_NAME_PATTERN.match(text) or text in ('.', '..'):
        raise ValueError(
            'map name must use letters, digits, "_", "." or "-" '
            '(max 64 characters, no path separators)'
        )
    return text


def has_map_database(path) -> bool:
    path = Path(path)
    try:
        return path.is_dir() and any(
            item.is_file() for item in path.glob('*.mdb')
        )
    except OSError:
        return False


def list_maps(host_root) -> list[str]:
    """Saved cuVSLAM maps (folders holding ``*.mdb``) under the host root."""

    root = Path(host_root)
    try:
        folders = sorted(item for item in root.iterdir() if item.is_dir())
    except OSError:
        return []
    return [item.name for item in folders if has_map_database(item)]


def station_pose_blocker(status: dict, *, status_fresh: bool) -> str:
    """Why the current localization pose must not be saved ('' = it may)."""

    if not status_fresh:
        return 'localization status is unavailable or stale'
    if status.get('initialized') is not True:
        return 'localization is not initialized; stop and request a station correction'
    if status.get('healthy') is not True:
        return 'localization is unhealthy: ' + str(
            status.get('detail', 'estimate unavailable')
        )
    if (
        status.get('station_mode') is not True
        or status.get('station_converged') is not True
        or status.get('stationary_ready') is not True
    ):
        return 'stop and complete a station correction before saving a point'
    return ''


def correction_complete(status: dict, *, baseline_count: int) -> bool:
    """True once a correction newer than ``baseline_count`` has converged."""

    try:
        count = int(status.get('correction_count', 0))
    except (TypeError, ValueError):
        return False
    return (
        status.get('healthy') is True
        and status.get('station_mode') is True
        and status.get('stationary_ready') is True
        and status.get('station_converged') is True
        and count > baseline_count
    )


@dataclass(frozen=True)
class VslamJobState:
    """Result of one asynchronous operation (correction, save, localize)."""

    status: str = 'idle'  # idle | running | succeeded | failed
    label: str = ''
    message: str = ''

    @property
    def running(self) -> bool:
        return self.status == 'running'


@dataclass(frozen=True)
class PoseRow:
    """One live pose for display: XYZ [m], roll/pitch/yaw [rad]."""

    name: str
    frame_id: str
    child_frame_id: str
    values: tuple
    fresh: bool
    age_sec: float


@dataclass(frozen=True)
class VslamSnapshot:
    process: str
    mode: str
    map_name: str
    tracking_ready: bool
    tracking_detail: str
    adapter_detail: str
    localization: dict
    localization_fresh: bool
    pose: Optional[tuple]  # (x, y, yaw) of base in the map frame
    pose_fresh: bool
    correction: VslamJobState
    job: VslamJobState
    poses: tuple = ()
    navigation_process: str = 'stopped'
    navigation: dict = None
    navigation_fresh: bool = False
    goto: VslamJobState = VslamJobState()


def _rpy(q) -> tuple:
    values = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid pose quaternion')
    x, y, z, w = (value / norm for value in values)
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


def _yaw(q) -> float:
    return _rpy(q)[2]


def goto_blocker(status: dict, *, status_fresh: bool) -> str:
    """Why a Goto must not start from the current localization state."""

    if (
        not status_fresh
        or status.get('initialized') is not True
        or status.get('healthy') is not True
    ):
        return 'first complete a station correction'
    if status.get('station_mode') is True and (
        status.get('station_converged') is not True
        or status.get('stationary_ready') is not True
    ):
        return 'wait for the stationary correction at this point'
    return ''


def gate_idle(status: dict, *, status_fresh: bool) -> bool:
    return (
        status_fresh
        and status.get('enabled') is False
        and status.get('cancel_pending') is False
        and status.get('active_goal_count') == 0
    )


class VslamClient:
    """Start, observe and command the containerised cuVSLAM stack."""

    STATUS_TIMEOUT_SEC = 1.0
    STOP_GRACE_SEC = 10.0
    LOG_LINES = 400

    def __init__(
        self,
        node,
        *,
        launch_script: str,
        container: str = 'rby1-isaac-vslam',
        map_root_host: str = '/mnt/ssd/workspaces/isaac_ros-dev/maps',
        map_root_container: str = '/workspaces/isaac_ros-dev/maps',
        waypoints_file: str = '',
        map_frame: str = 'vslam_map',
        base_frame: str = 'base',
        station_service: str = '/rby1/vslam/set_station_mode',
        reset_visual_service: str = '/rby1/vslam/reset_visual_reference',
        localization_odom_topic: str = '/rby1/vslam/localization_odom',
        correction_timeout_sec: float = 30.0,
        job_timeout_sec: float = 180.0,
        navigation_script: str = '',
        allowed_cmd_raw_publishers: str = 'rby1_navigation_node',
        navigate_action: str = '/rby1/vslam/nav2/navigate_to_pose',
        gate_enable_service: str = '/rby1/vslam/enable',
        gate_cancel_service: str = '/rby1/vslam/cancel',
        goto_stage_timeout_sec: float = 10.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._node = node
        self.launch_script = str(launch_script)
        self.container = str(container).strip()
        self.map_root_host = Path(str(map_root_host))
        self.map_root_container = str(map_root_container).rstrip('/')
        self.waypoints_file = str(waypoints_file).strip()
        self.map_frame = str(map_frame)
        self.base_frame = str(base_frame)
        self.correction_timeout_sec = float(correction_timeout_sec)
        self.job_timeout_sec = float(job_timeout_sec)
        self._clock = clock
        if not self.container:
            raise ValueError('vslam container name must not be empty')

        self._lock = threading.RLock()
        self._process: Optional[subprocess.Popen] = None
        self._stopping = False
        self._stop_thread: Optional[threading.Thread] = None
        self._mode = ''
        self._map_name = ''
        self._log: deque[str] = deque(maxlen=self.LOG_LINES)
        self._statuses: dict[str, tuple[dict, float]] = {}
        self._pose = None  # (message, (x, y, yaw), received)
        self._pose_after_ns = 0
        self._correction: Optional[dict] = None
        self._correction_result = VslamJobState()
        self._job = VslamJobState()

        for name, topic in STATUS_TOPICS.items():
            node.create_subscription(
                String,
                topic,
                lambda message, key=name: self._on_status(key, message),
                10,
            )
        node.create_subscription(
            Odometry,
            localization_odom_topic,
            self._on_pose,
            qos_profile_sensor_data,
        )
        self._station_client = node.create_client(SetBool, station_service)
        self._reset_client = node.create_client(Trigger, reset_visual_service)

        # Live poses for display (Localization is handled by _on_pose above).
        self._display_poses: dict[str, tuple] = {}
        for name, topic in POSE_TOPICS.items():
            node.create_subscription(
                Odometry,
                topic,
                lambda message, key=name: self._on_display_pose(key, message),
                qos_profile_sensor_data,
            )

        # Nav2 Goto. Nav2 and its command gate run on the host next to the
        # planner's odometry navigation; the two are never used together.
        self.navigation_script = str(navigation_script).strip()
        self.allowed_cmd_raw_publishers = str(allowed_cmd_raw_publishers).strip()
        self.goto_stage_timeout_sec = float(goto_stage_timeout_sec)
        self._nav_process: Optional[subprocess.Popen] = None
        self._nav_stopping = False
        self._nav_stop_thread: Optional[threading.Thread] = None
        self._goto: Optional[dict] = None
        self._goto_result = VslamJobState()
        self._goto_sequence = 0
        self._goal_handle = None
        self._gate_enable_client = node.create_client(SetBool, gate_enable_service)
        self._gate_cancel_client = node.create_client(Trigger, gate_cancel_service)
        self._navigate_client = None
        self._navigate_goal_type = None
        try:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self._navigate_goal_type = NavigateToPose
            self._navigate_client = ActionClient(
                node, NavigateToPose, navigate_action
            )
        except ImportError as exc:
            self._append_log(f'[nav2] Goto is unavailable: {exc}')
        self._goto_timer = node.create_timer(0.1, self._tick_goto)

    # ------------------------------------------------------------------
    # Process control
    # ------------------------------------------------------------------
    @property
    def mode(self) -> str:
        return self._mode if self.process_state() != 'stopped' else ''

    def process_state(self) -> str:
        """stopped | running | stopping | exited (ended without Stop)."""

        with self._lock:
            process = self._process
            if process is None:
                return 'stopped'
            if process.poll() is None:
                return 'stopping' if self._stopping else 'running'
            if not self._stopping:
                return 'exited'
            thread = self._stop_thread
            if thread is not None and thread.is_alive():
                return 'stopping'
            return 'stopped'

    def start(self, mode: str, map_name: str = '') -> None:
        mode = str(mode).strip()
        if mode not in VSLAM_MODES:
            raise ValueError("vslam mode must be 'mapping' or 'localization'")
        state = self.process_state()
        if state in ('running', 'stopping'):
            raise RuntimeError(f'VSLAM stack is already {state}')
        if not Path(self.launch_script).is_file():
            raise RuntimeError(
                f'VSLAM launch script is missing: {self.launch_script}'
            )
        arguments = [
            'start_rviz:=false',
            # rby1_camera/camera_mount_tf_publisher already publishes
            # link_head_2 -> d435_camera_center -> d435_link.
            'publish_mount_tf:=false',
            # Nav2 and its command gate are not part of this stack.
            'require_navigation_idle:=false',
            f'mode:={mode}',
        ]
        name = ''
        if mode == 'localization':
            name = validate_map_name(map_name)
            if not has_map_database(self.map_root_host / name):
                raise ValueError(
                    f'no saved map named {name!r} under {self.map_root_host}'
                )
            arguments.append(f'map_path:={self.map_root_container}/{name}')
        if self._container_launch_running():
            raise RuntimeError(
                f'cuVSLAM is already running inside {self.container}; '
                'stop that session first'
            )
        environment = dict(os.environ)
        # The script defaults to domain 10; follow the planner's graph instead.
        environment['RBY1_ROS_DOMAIN_ID'] = environment.get('ROS_DOMAIN_ID', '0')
        environment['RBY1_ISAAC_CONTAINER'] = self.container
        process = subprocess.Popen(
            ['bash', self.launch_script, '0', *arguments],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors='replace',
            start_new_session=True,
            env=environment,
        )
        with self._lock:
            self._process = process
            self._stopping = False
            self._mode = mode
            self._map_name = name
            self._statuses.clear()
            self._pose = None
            self._correction = None
            self._correction_result = VslamJobState()
        threading.Thread(
            target=self._read_output,
            args=(process,),
            name='vslam-output',
            daemon=True,
        ).start()
        self._append_log(
            f'[vslam] started in {mode} mode'
            + (f' with map {name}' if name else '')
        )

    def stop(self) -> None:
        """Request an orderly stop; ``process_state`` reports the progress."""

        with self._lock:
            process = self._process
            if process is None or self._stopping:
                return
            # Also runs for a stack that already exited by itself: nodes may
            # be left inside the container and are cleaned up below.
            self._stopping = True
            self._stop_thread = threading.Thread(
                target=self._stop_blocking,
                args=(process,),
                name='vslam-stop',
                daemon=True,
            )
            self._stop_thread.start()
        self._append_log('[vslam] stopping')

    def close(self) -> None:
        """Stop the stack and wait for it (used when the UI exits)."""

        self.stop_goto('closing')
        self.stop_navigation()
        self.stop()
        for thread in (self._nav_stop_thread, self._stop_thread):
            if thread is not None:
                thread.join(timeout=2 * self.STOP_GRACE_SEC + 15.0)

    def _stop_blocking(self, process: subprocess.Popen) -> None:
        # ``docker exec`` without a TTY does not forward signals, so the launch
        # inside the container is interrupted explicitly.
        self._container_pkill('-INT', CONTAINER_LAUNCH_PATTERN)
        self._signal_group(process, signal.SIGINT)
        try:
            process.wait(timeout=self.STOP_GRACE_SEC)
        except subprocess.TimeoutExpired:
            self._container_pkill('-TERM', 'jetson.launch.py')
            self._signal_group(process, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._signal_group(process, signal.SIGKILL)
        # The host ``docker exec`` client can exit before the nodes inside
        # the container have shut down; report "stopped" only once they have.
        deadline = time.monotonic() + self.STOP_GRACE_SEC
        escalated = False
        while self._container_launch_running():
            if time.monotonic() > deadline:
                if escalated:
                    self._append_log(
                        '[vslam] WARNING: cuVSLAM is still running inside '
                        + self.container
                    )
                    break
                self._container_pkill('-TERM', 'jetson.launch.py')
                escalated = True
                deadline = time.monotonic() + 5.0
            time.sleep(0.3)
        self._append_log('[vslam] stopped')

    @staticmethod
    def _signal_group(process: subprocess.Popen, signum: int) -> None:
        try:
            os.killpg(process.pid, signum)
        except (ProcessLookupError, PermissionError):
            pass

    def _container_pkill(self, signal_flag: str, pattern: str) -> None:
        try:
            subprocess.run(
                ['docker', 'exec', self.container, 'pkill', signal_flag,
                 '-f', pattern],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            pass

    def _container_launch_running(self) -> bool:
        try:
            result = subprocess.run(
                ['docker', 'exec', self.container, 'pgrep', '-f',
                 CONTAINER_LAUNCH_PATTERN],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            # A missing/stopped container is reported by the launch script.
            return False
        return result.returncode == 0

    def _read_output(self, process: subprocess.Popen) -> None:
        stream = process.stdout
        if stream is None:
            return
        try:
            for line in stream:
                self._append_log(line.rstrip())
        except (OSError, ValueError):
            pass
        code = process.wait()
        with self._lock:
            expected = self._stopping
        if not expected:
            self._append_log(f'[vslam] stack exited with code {code}')

    def _append_log(self, line: str) -> None:
        if line:
            with self._lock:
                self._log.append(line)

    def drain_log(self) -> list[str]:
        with self._lock:
            lines = list(self._log)
            self._log.clear()
        return lines

    # ------------------------------------------------------------------
    # Status mirrors
    # ------------------------------------------------------------------
    def _on_status(self, name: str, message: String) -> None:
        try:
            packet = json.loads(message.data)
            if not isinstance(packet, dict):
                raise ValueError('status is not an object')
        except (ValueError, TypeError):
            return
        with self._lock:
            if name == 'localization':
                previous = self._statuses.get('localization', ({}, 0.0))[0]
                if (
                    packet.get('session_id') != previous.get('session_id')
                    or packet.get('initialized') is not True
                    or previous.get('initialized') is not True
                ):
                    # Odometry carries no estimator session; drop the last
                    # session's pose and accept only newer samples.
                    self._pose = None
                    self._pose_after_ns = (
                        self._node.get_clock().now().nanoseconds
                    )
            self._statuses[name] = (packet, self._clock())

    def _on_pose(self, message: Odometry) -> None:
        try:
            stamp_ns = (
                message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            )
            with self._lock:
                status = self._status('localization')
                if (
                    not self._fresh('localization')
                    or status.get('initialized') is not True
                    or stamp_ns <= self._pose_after_ns
                ):
                    return
            position = message.pose.pose.position
            values = (
                float(position.x),
                float(position.y),
                _yaw(message.pose.pose.orientation),
            )
            if not all(math.isfinite(value) for value in values):
                return
            with self._lock:
                self._pose = (message, values, self._clock())
        except (ValueError, TypeError):
            pass

    def _status(self, name: str) -> dict:
        return self._statuses.get(name, ({}, -math.inf))[0]

    def _fresh(self, name: str) -> bool:
        received = self._statuses.get(name, ({}, -math.inf))[1]
        return self._clock() - received < self.STATUS_TIMEOUT_SEC

    def _pose_fresh(self) -> bool:
        if self._pose is None:
            return False
        message, _values, received = self._pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self._node.get_clock().now().nanoseconds * 1e-9 - stamp
        return (
            self._clock() - received < self.STATUS_TIMEOUT_SEC
            and -0.1 <= age < self.STATUS_TIMEOUT_SEC
        )

    def tracking_ready(self) -> tuple[bool, str]:
        with self._lock:
            status = self._status('tracking')
            if not self._fresh('tracking'):
                return False, 'waiting for tracking status'
            ready = (
                status.get('ready') is True
                and status.get('tracking_ok') is True
            )
            return ready, str(status.get('detail', ''))

    def current_pose(self) -> tuple[float, float, float]:
        """Base (x, y, yaw) in the map frame after a completed correction."""

        with self._lock:
            blocker = station_pose_blocker(
                self._status('localization'),
                status_fresh=self._fresh('localization'),
            )
            if blocker:
                raise ValueError(blocker)
            if not self._pose_fresh():
                raise ValueError('localization pose is stale')
            message, values, _received = self._pose
            if (
                message.header.frame_id != self.map_frame
                or message.child_frame_id != self.base_frame
            ):
                raise ValueError(
                    'localization frames do not match the waypoint frames'
                )
            return values

    def snapshot(self) -> VslamSnapshot:
        tracking_ready, tracking_detail = self.tracking_ready()
        correction = self.correction_state()
        with self._lock:
            streams = self._status('adapter').get('streams', {})
            if not self._fresh('adapter') or not isinstance(streams, dict):
                adapter_detail = 'status unavailable'
            else:
                parts = []
                for kind, title in (('odom', 'VO'), ('slam_odom', 'SLAM')):
                    stream = streams.get(kind, {})
                    if not isinstance(stream, dict):
                        stream = {}
                    parts.append(title + ': ' + (
                        'LIVE' if stream.get('healthy') is True
                        else str(stream.get('detail', 'waiting'))
                    ))
                adapter_detail = ' | '.join(parts)
            return VslamSnapshot(
                process=self.process_state(),
                mode=self._mode,
                map_name=self._map_name,
                tracking_ready=tracking_ready,
                tracking_detail=tracking_detail,
                adapter_detail=adapter_detail,
                localization=dict(self._status('localization')),
                localization_fresh=self._fresh('localization'),
                pose=self._pose[1] if self._pose is not None else None,
                pose_fresh=self._pose_fresh(),
                correction=correction,
                job=self._job,
                poses=self.pose_rows(),
                navigation_process=self.navigation_process_state(),
                navigation=dict(self._status('navigation')),
                navigation_fresh=self._fresh('navigation'),
                goto=self.goto_state(),
            )

    # ------------------------------------------------------------------
    # Station correction / visual reference
    # ------------------------------------------------------------------
    def request_station_correction(
        self,
        timeout_sec: Optional[float] = None,
    ) -> None:
        """Ask localization to correct map->odom at the stopped robot."""

        if not self._station_client.service_is_ready():
            raise RuntimeError(
                'station correction service is unavailable; start VSLAM first'
            )
        if self._goto is not None:
            raise RuntimeError('a Goto is running; stop it first')
        self._begin_station_correction(timeout_sec)

    def _begin_station_correction(
        self,
        timeout_sec: Optional[float] = None,
    ) -> None:
        timeout = (
            self.correction_timeout_sec if timeout_sec is None
            else float(timeout_sec)
        )
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('correction timeout must be positive and finite')
        with self._lock:
            try:
                baseline = int(
                    self._status('localization').get('correction_count', 0)
                )
            except (TypeError, ValueError):
                baseline = 0
            now = self._clock()
            self._correction = {
                'future': self._station_client.call_async(
                    SetBool.Request(data=True)
                ),
                'baseline': baseline,
                'requested': now,
                'deadline': now + timeout,
            }
            self._correction_result = VslamJobState(
                'running', 'station correction', 'requested'
            )

    def correction_state(self) -> VslamJobState:
        with self._lock:
            pending = self._correction
            if pending is None:
                return self._correction_result
            status = self._status('localization')
            detail = str(status.get('detail', 'waiting for localization'))
            result = None
            future = pending['future']
            if future.done():
                failure = ''
                if future.exception() is not None:
                    failure = str(future.exception())
                else:
                    response = future.result()
                    if response is None or not response.success:
                        failure = str(
                            getattr(response, 'message', '') or
                            'service returned no response'
                        )
                if failure:
                    result = VslamJobState(
                        'failed', 'station correction', failure
                    )
            if result is None:
                received = self._statuses.get(
                    'localization', ({}, -math.inf)
                )[1]
                if (
                    future.done()
                    and self._fresh('localization')
                    and received > pending['requested']
                    and correction_complete(
                        status, baseline_count=pending['baseline']
                    )
                ):
                    result = VslamJobState(
                        'succeeded', 'station correction',
                        'correction complete; station ready',
                    )
                elif self._clock() > pending['deadline']:
                    result = VslamJobState(
                        'failed', 'station correction',
                        'timed out: ' + detail,
                    )
            if result is None:
                self._correction_result = VslamJobState(
                    'running', 'station correction', detail
                )
                return self._correction_result
            self._correction = None
            self._correction_result = result
            self._append_log(f'[vslam] station correction: {result.message}')
            return result

    def reset_visual_reference(self) -> None:
        """Anchor a restarted VO session to the current wheel estimate."""

        if not self._reset_client.service_is_ready():
            raise RuntimeError('visual reference service is unavailable')
        future = self._reset_client.call_async(Trigger.Request())

        def done(completed) -> None:
            try:
                response = completed.result()
                message = getattr(response, 'message', '')
                prefix = 'accepted' if response.success else 'rejected'
            except Exception as exc:  # service call failure is only reported
                prefix, message = 'failed', str(exc)
            self._append_log(
                f'[vslam] visual reference recovery {prefix}: {message}'
            )

        future.add_done_callback(done)

    # ------------------------------------------------------------------
    # Live poses
    # ------------------------------------------------------------------
    def _on_display_pose(self, name: str, message: Odometry) -> None:
        try:
            position = message.pose.pose.position
            values = (
                float(position.x), float(position.y), float(position.z),
                *_rpy(message.pose.pose.orientation),
            )
            if all(math.isfinite(value) for value in values):
                with self._lock:
                    self._display_poses[name] = (message, values, self._clock())
        except (ValueError, TypeError):
            pass

    def pose_rows(self) -> tuple:
        rows = []
        now = self._clock()
        ros_now = self._node.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            for name in POSE_TOPICS:
                item = self._display_poses.get(name)
                if name == 'Localization' and self._pose is None:
                    # Same rule as saving: hide a pose of an old session.
                    item = None
                if item is None:
                    rows.append(PoseRow(name, '', '', (), False, math.inf))
                    continue
                message, values, received = item
                stamp = (
                    message.header.stamp.sec
                    + message.header.stamp.nanosec * 1e-9
                )
                fresh = (
                    now - received < self.STATUS_TIMEOUT_SEC
                    and -0.1 <= ros_now - stamp < self.STATUS_TIMEOUT_SEC
                )
                rows.append(PoseRow(
                    name, message.header.frame_id, message.child_frame_id,
                    values, fresh, now - received,
                ))
        return tuple(rows)

    # ------------------------------------------------------------------
    # Nav2 process (host)
    # ------------------------------------------------------------------
    def navigation_process_state(self) -> str:
        with self._lock:
            process = self._nav_process
            if process is None:
                return 'stopped'
            if process.poll() is None:
                return 'stopping' if self._nav_stopping else 'running'
            if not self._nav_stopping:
                return 'exited'
            thread = self._nav_stop_thread
            if thread is not None and thread.is_alive():
                return 'stopping'
            return 'stopped'

    def start_navigation(self) -> None:
        """Start Nav2 + command gate on the host (no obstacle sensing)."""

        if self._navigate_client is None:
            raise RuntimeError('nav2_msgs is not installed on this host')
        state = self.navigation_process_state()
        if state in ('running', 'stopping'):
            raise RuntimeError(f'Nav2 is already {state}')
        if not Path(self.navigation_script).is_file():
            raise RuntimeError(
                f'Nav2 launch script is missing: {self.navigation_script}'
            )
        environment = dict(os.environ)
        environment['RBY1_ROS_DOMAIN_ID'] = environment.get('ROS_DOMAIN_ID', '0')
        arguments = [
            # localization_tf already runs with cuVSLAM in the container.
            'start_localization_tf:=false',
            'use_scan:=false',
        ]
        if self.allowed_cmd_raw_publishers:
            arguments.append(
                'allowed_cmd_raw_publishers:=' + self.allowed_cmd_raw_publishers
            )
        process = subprocess.Popen(
            ['bash', self.navigation_script, *arguments],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors='replace',
            start_new_session=True,
            env=environment,
        )
        with self._lock:
            self._nav_process = process
            self._nav_stopping = False
        threading.Thread(
            target=self._read_navigation_output,
            args=(process,),
            name='nav2-output',
            daemon=True,
        ).start()
        self._append_log('[nav2] started (no obstacle sensing)')

    def stop_navigation(self) -> None:
        self.stop_goto('Nav2 stopped')
        with self._lock:
            process = self._nav_process
            if process is None or self._nav_stopping:
                return
            self._nav_stopping = True
            self._nav_stop_thread = threading.Thread(
                target=self._stop_navigation_blocking,
                args=(process,),
                name='nav2-stop',
                daemon=True,
            )
            self._nav_stop_thread.start()
        self._append_log('[nav2] stopping')

    def _stop_navigation_blocking(self, process: subprocess.Popen) -> None:
        self._signal_group(process, signal.SIGINT)
        try:
            process.wait(timeout=self.STOP_GRACE_SEC)
        except subprocess.TimeoutExpired:
            self._signal_group(process, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._signal_group(process, signal.SIGKILL)
        self._append_log('[nav2] stopped')

    def _read_navigation_output(self, process: subprocess.Popen) -> None:
        stream = process.stdout
        if stream is None:
            return
        try:
            for line in stream:
                # Nav2 is verbose; keep warnings, errors and lifecycle news.
                text = line.rstrip()
                if any(key in text for key in (
                    'ERROR', 'WARN', 'FATAL', 'Managed nodes are active',
                    'process has died', 'nav2_gate',
                )):
                    self._append_log('[nav2] ' + text[-300:])
        except (OSError, ValueError):
            pass
        code = process.wait()
        with self._lock:
            expected = self._nav_stopping
        if not expected:
            self._append_log(f'[nav2] exited with code {code}')

    # ------------------------------------------------------------------
    # Goto (Nav2 NavigateToPose through the command gate)
    # ------------------------------------------------------------------
    def navigation_busy(self) -> bool:
        """True while a Goto owns the base; Tasks must not start then."""

        return self._goto is not None

    def goto_state(self) -> VslamJobState:
        with self._lock:
            return self._goto_result

    def goto(self, waypoint) -> None:
        """Drive to a saved point, stop and request a station correction."""

        if self._navigate_client is None:
            raise RuntimeError('nav2_msgs is not installed on this host')
        if self._goto is not None:
            raise RuntimeError('a Goto is already running')
        if self.navigation_process_state() != 'running':
            raise RuntimeError('start Nav2 first')
        with self._lock:
            blocker = goto_blocker(
                self._status('localization'),
                status_fresh=self._fresh('localization'),
            )
            if blocker:
                raise RuntimeError(blocker)
            if not self._fresh('navigation'):
                raise RuntimeError(
                    'navigation gate status is unavailable; wait for Nav2'
                )
        if not self._gate_enable_client.service_is_ready():
            raise RuntimeError('navigation gate service is unavailable')
        for value in (waypoint.x, waypoint.y, waypoint.yaw):
            if not math.isfinite(float(value)):
                raise ValueError('waypoint coordinates must be finite')
        with self._lock:
            self._goto_sequence += 1
            now = self._clock()
            self._goto = {
                'id': self._goto_sequence,
                'waypoint': waypoint,
                'stage': 'gate',
                'stage_started': now,
                # Start from a stopped gate, exactly like the station UI.
                'future': self._gate_enable_client.call_async(
                    SetBool.Request(data=False)
                ),
                'result': None,
            }
            self._set_goto_state('running', 'waiting for navigation to stop')
        self._append_log(f'[goto] {waypoint.name}: requested')

    def stop_goto(self, reason: str = 'stopped by operator') -> None:
        """Cancel the goal and disarm the gate; safe to call at any time."""

        with self._lock:
            pending = self._goto
            self._goto = None
            handle, self._goal_handle = self._goal_handle, None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        if pending is None and handle is None:
            return
        try:
            if self._gate_enable_client.service_is_ready():
                self._gate_enable_client.call_async(SetBool.Request(data=False))
            elif self._gate_cancel_client.service_is_ready():
                self._gate_cancel_client.call_async(Trigger.Request())
        except Exception:
            pass
        if pending is not None:
            name = pending['waypoint'].name
            with self._lock:
                self._goto_result = VslamJobState(
                    'failed', f'goto {name}', reason
                )
            self._append_log(f'[goto] {name}: {reason}')

    def _set_goto_state(self, status: str, message: str) -> None:
        name = self._goto['waypoint'].name if self._goto else ''
        label = f'goto {name}' if name else self._goto_result.label
        self._goto_result = VslamJobState(status, label, message)

    @staticmethod
    def _service_ok(future) -> bool:
        """True when done and successful; raises on a rejected request."""

        if not future.done():
            return False
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(
                str(getattr(response, 'message', '') or 'service failed')
            )
        return True

    def _goto_next(self, pending: dict, stage: str, message: str) -> None:
        pending['stage'] = stage
        pending['stage_started'] = self._clock()
        self._set_goto_state('running', message)

    def _tick_goto(self) -> None:
        with self._lock:
            pending = self._goto
            if pending is None:
                return
            try:
                self._advance_goto(pending)
            except Exception as exc:
                failure = str(exc)
            else:
                return
        self.stop_goto('blocked: ' + failure)

    def _advance_goto(self, pending: dict) -> None:
        now = self._clock()
        stage = pending['stage']
        waypoint = pending['waypoint']
        localization = self._status('localization')
        navigation = self._status('navigation')
        if self.navigation_process_state() != 'running':
            raise RuntimeError('Nav2 is not running')
        if stage not in ('travel', 'correct') and (
            now - pending['stage_started'] > self.goto_stage_timeout_sec
        ):
            raise RuntimeError(f'timed out in stage {stage!r}')

        if stage == 'gate':
            if not self._service_ok(pending['future']):
                return
            if not gate_idle(navigation, status_fresh=self._fresh('navigation')):
                return
            if not self._station_client.service_is_ready():
                raise RuntimeError('station mode service is unavailable')
            # Transit uses wheel odometry with the correction frozen.
            pending['future'] = self._station_client.call_async(
                SetBool.Request(data=False)
            )
            pending['mode_requested'] = now
            self._goto_next(pending, 'transit', 'switching to wheel-only transit')
        elif stage == 'transit':
            if not self._service_ok(pending['future']):
                return
            received = self._statuses.get('localization', ({}, -math.inf))[1]
            if (
                not self._fresh('localization')
                or received <= pending['mode_requested']
                or localization.get('station_mode') is not False
                or localization.get('navigation_ready') is not True
            ):
                return
            pending['future'] = self._gate_enable_client.call_async(
                SetBool.Request(data=True)
            )
            self._goto_next(pending, 'enable', 'enabling the navigation gate')
        elif stage == 'enable':
            if not self._service_ok(pending['future']):
                return
            if not self._navigate_client.server_is_ready():
                raise RuntimeError('Nav2 NavigateToPose action is unavailable')
            goal = self._navigate_goal_type.Goal()
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.header.frame_id = self.map_frame
            goal.pose.pose.position.x = float(waypoint.x)
            goal.pose.pose.position.y = float(waypoint.y)
            goal.pose.pose.orientation.z = math.sin(float(waypoint.yaw) / 2.0)
            goal.pose.pose.orientation.w = math.cos(float(waypoint.yaw) / 2.0)
            identifier = pending['id']
            future = self._navigate_client.send_goal_async(
                goal, feedback_callback=self._on_goto_feedback
            )
            future.add_done_callback(
                lambda done: self._on_goal_response(done, identifier)
            )
            self._goto_next(pending, 'travel', 'travelling (wheel-only localization)')
            self._append_log(f'[goto] {waypoint.name}: goal sent')
        elif stage == 'travel':
            status = pending['result']
            if status is None:
                return
            self._goal_handle = None
            if status != 4:
                raise RuntimeError(
                    'navigation ' + GOAL_STATUS_NAMES.get(status, str(status))
                )
            self._append_log(
                f'[goto] {waypoint.name}: arrived; stopping for VO correction'
            )
            pending['future'] = self._gate_enable_client.call_async(
                SetBool.Request(data=False)
            )
            self._goto_next(pending, 'arrived', 'arrived; stopping navigation')
        elif stage == 'arrived':
            if not self._service_ok(pending['future']):
                return
            if not gate_idle(navigation, status_fresh=self._fresh('navigation')):
                return
            self._begin_station_correction()
            self._goto_next(pending, 'correct', 'waiting for station correction')
        elif stage == 'correct':
            result = self.correction_state()
            if result.status == 'running':
                self._set_goto_state('running', 'correcting: ' + result.message)
                return
            name = waypoint.name
            self._goto = None
            if result.status == 'succeeded':
                self._goto_result = VslamJobState(
                    'succeeded', f'goto {name}',
                    'arrived; correction complete',
                )
            else:
                self._goto_result = VslamJobState(
                    'failed', f'goto {name}',
                    'arrived, but correction failed: ' + result.message,
                )
            self._append_log(f'[goto] {name}: {self._goto_result.message}')

    def _on_goto_feedback(self, message) -> None:
        with self._lock:
            pending = self._goto
            if pending is None or pending['stage'] != 'travel':
                return
            remaining = float(
                getattr(message.feedback, 'distance_remaining', float('nan'))
            )
            if math.isfinite(remaining):
                self._set_goto_state(
                    'running', f'travelling: {remaining:.2f} m remaining'
                )

    def _on_goal_response(self, future, identifier: int) -> None:
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError('Nav2 rejected the goal')
        except Exception as exc:
            with self._lock:
                current = self._goto is not None and self._goto['id'] == identifier
            if current:
                self.stop_goto('blocked: ' + str(exc))
            return
        with self._lock:
            current = self._goto is not None and self._goto['id'] == identifier
            if current:
                self._goal_handle = handle
        if not current:
            # Superseded or stopped while the goal was being accepted.
            handle.cancel_goal_async()
            return
        handle.get_result_async().add_done_callback(
            lambda done: self._on_goal_result(done, identifier)
        )

    def _on_goal_result(self, future, identifier: int) -> None:
        try:
            status = int(future.result().status)
        except Exception:
            status = 6
        with self._lock:
            if self._goto is not None and self._goto['id'] == identifier:
                self._goto['result'] = status

    # ------------------------------------------------------------------
    # Map save / localize (map_tool inside the container)
    # ------------------------------------------------------------------
    def list_maps(self) -> list[str]:
        return list_maps(self.map_root_host)

    def job_state(self) -> VslamJobState:
        with self._lock:
            return self._job

    def save_map(self, name: str) -> str:
        name = validate_map_name(name)
        if self.process_state() != 'running':
            raise RuntimeError('start VSLAM before saving a map')
        target = self.map_root_host / name
        try:
            occupied = target.exists() and any(target.iterdir())
        except OSError:
            occupied = True
        if occupied:
            raise ValueError(
                f'map {name!r} already exists; choose a new name'
            )
        path = f'{self.map_root_container}/{name}'
        script = (
            CONTAINER_SETUP
            + f'mkdir -p "{self.map_root_container}" && '
            + 'exec ros2 run rby1_vslam map_tool save --path "$1"'
        )
        self._start_job(f'save map {name}', script, [path])
        return name

    def localize(
        self,
        name: str,
        *,
        x: float,
        y: float,
        z: float,
        yaw: float,
    ) -> None:
        """Localize in a saved map; the hint is d435_link in the map frame."""

        name = validate_map_name(name)
        if self.process_state() != 'running':
            raise RuntimeError('start VSLAM before localizing')
        if not has_map_database(self.map_root_host / name):
            raise ValueError(
                f'no saved map named {name!r} under {self.map_root_host}'
            )
        hint = []
        for label, value in (('x', x), ('y', y), ('z', z), ('yaw', yaw)):
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f'pose hint {label} must be finite')
            hint += [f'--{label}', repr(value)]
        path = f'{self.map_root_container}/{name}'
        script = (
            CONTAINER_SETUP
            + 'exec ros2 run rby1_vslam map_tool localize --path "$@"'
        )
        self._start_job(f'localize in {name}', script, [path, *hint])

    def _start_job(self, label: str, script: str, arguments: list) -> None:
        with self._lock:
            if self._job.running:
                raise RuntimeError(
                    f'another map operation is running: {self._job.label}'
                )
            self._job = VslamJobState('running', label, 'running')
        command = [
            'docker', 'exec',
            '-e', 'ROS_DOMAIN_ID=' + os.environ.get('ROS_DOMAIN_ID', '0'),
            self.container, 'bash', '-lc', script, 'bash', *arguments,
        ]
        threading.Thread(
            target=self._run_job,
            args=(label, command),
            name='vslam-job',
            daemon=True,
        ).start()
        self._append_log(f'[vslam] {label}: started')

    def _run_job(self, label: str, command: list) -> None:
        try:
            result = subprocess.run(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors='replace',
                timeout=self.job_timeout_sec,
                check=False,
            )
            lines = [
                line.strip() for line in result.stdout.splitlines()
                if line.strip()
            ]
            message = lines[-1] if lines else f'exit code {result.returncode}'
            state = VslamJobState(
                'succeeded' if result.returncode == 0 else 'failed',
                label,
                message,
            )
        except subprocess.TimeoutExpired:
            state = VslamJobState(
                'failed', label,
                f'timed out after {self.job_timeout_sec:.0f} s; the operation '
                'may still be running in the container',
            )
        except OSError as exc:
            state = VslamJobState('failed', label, str(exc))
        with self._lock:
            self._job = state
        self._append_log(f'[vslam] {label}: {state.status}: {state.message}')

    # ------------------------------------------------------------------
    # Named points (rby1_vslam waypoint file, map frame)
    # ------------------------------------------------------------------
    def _waypoint_store(self):
        try:
            from rby1_vslam import waypoint_store
        except ImportError as exc:
            raise RuntimeError(
                'rby1_vslam is not built in this workspace: ' + str(exc)
            ) from exc
        return waypoint_store

    def waypoints_path(self) -> Path:
        if self.waypoints_file:
            path = Path(self.waypoints_file).expanduser()
        else:
            from ament_index_python.packages import (
                get_package_share_directory,
            )
            path = self._waypoint_store().default_waypoints_path(
                get_package_share_directory('rby1_vslam')
            )
        if not path.is_absolute():
            raise ValueError('vslam waypoints file must be an absolute path')
        return path

    def load_waypoints(self) -> list:
        return self._waypoint_store().load_waypoints(
            self.waypoints_path(), self.map_frame
        )

    def save_current_point(self, name: str):
        """Store the corrected current pose under ``name``; returns it."""

        store = self._waypoint_store()
        name = str(name).strip()
        waypoints = self.load_waypoints()
        if not name or any(item.name == name for item in waypoints):
            raise ValueError('use a nonempty, unique point name')
        x, y, yaw = self.current_pose()
        waypoint = store.Waypoint(name, x, y, yaw)
        store.save_waypoints(
            self.waypoints_path(), [*waypoints, waypoint], self.map_frame
        )
        return waypoint

    def delete_point(self, name: str) -> None:
        store = self._waypoint_store()
        waypoints = self.load_waypoints()
        remaining = [item for item in waypoints if item.name != name]
        if len(remaining) == len(waypoints):
            raise ValueError(f'no point named {name!r}')
        store.save_waypoints(
            self.waypoints_path(), remaining, self.map_frame
        )

    def next_point_name(self) -> str:
        return self._waypoint_store().next_name(self.load_waypoints())


__all__ = [
    'VSLAM_MODES',
    'VslamClient',
    'VslamJobState',
    'VslamSnapshot',
    'PoseRow',
    'gate_idle',
    'goto_blocker',
    'correction_complete',
    'has_map_database',
    'list_maps',
    'station_pose_blocker',
    'validate_map_name',
]
