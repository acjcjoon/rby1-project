"""Non-blocking adapter from the RBY1 SDK power/servo API."""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import importlib
import math
import threading
import time
from typing import Callable, Optional, Tuple


PowerServoPair = Tuple[bool, bool]


class PowerServoStateAdaptor:
    """Poll blocking SDK calls on a dedicated worker thread.

    ``poll()`` is intentionally non-blocking and is suitable for a ROS timer.
    Completion callbacks run from the caller of ``poll()``, keeping all state
    mutations on the ROS executor thread.
    """

    def __init__(
        self,
        *,
        address: str,
        model: str = 'm',
        power_pattern: str = '.*',
        servo_pattern: str = '.*',
        poll_period_sec: float = 0.5,
        on_state: Callable[[bool, bool], None],
        on_error: Callable[[str], None],
        clock=time.monotonic,
        robot_factory=None,
    ) -> None:
        self.address = str(address).strip()
        self.model = str(model).strip().lower()
        self.power_pattern = str(power_pattern)
        self.servo_pattern = str(servo_pattern)
        self.poll_period_sec = float(poll_period_sec)
        if not self.address:
            raise ValueError('robot address must not be empty')
        if self.model not in {'a', 'm', 'ub'}:
            raise ValueError('robot model must be a, m, or ub')
        if (
            not math.isfinite(self.poll_period_sec)
            or self.poll_period_sec <= 0.0
        ):
            raise ValueError('poll_period_sec must be positive and finite')

        self._on_state = on_state
        self._on_error = on_error
        self._clock = clock
        self._robot_factory = robot_factory
        self._robot = None
        self._connected = False
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='rby1-power-servo',
        )
        self._future: Optional[Future] = None
        self._next_poll_at = -math.inf
        self._closed = False
        self._lock = threading.RLock()
        self._last_error = ''

    def poll(self) -> None:
        """Collect a completed query and schedule the next query if due."""

        with self._lock:
            if self._closed:
                return
            future = self._future

        if future is not None:
            if not future.done():
                return
            try:
                power_enabled, servo_enabled = future.result()
            except Exception as exc:
                detail = f'Power/Servo SDK query failed: {exc}'
                if detail != self._last_error:
                    self._last_error = detail
                    self._on_error(detail)
            else:
                self._last_error = ''
                self._on_state(power_enabled, servo_enabled)
            with self._lock:
                if self._future is future:
                    self._future = None

        now = self._clock()
        with self._lock:
            if self._closed or self._future is not None:
                return
            if now < self._next_poll_at:
                return
            self._next_poll_at = now + self.poll_period_sec
            self._future = self._executor.submit(self._query)

    def close(self) -> None:
        """Stop scheduling SDK work without blocking node shutdown."""

        with self._lock:
            if self._closed:
                return
            self._closed = True
            future = self._future
            self._future = None
        if future is not None:
            future.cancel()
        self._executor.shutdown(wait=False, cancel_futures=True)

    def _query(self) -> PowerServoPair:
        robot = self._robot
        if robot is None:
            factory = self._robot_factory
            if factory is None:
                sdk = importlib.import_module('rby1_sdk')
                factory = sdk.create_robot
            robot = factory(self.address, self.model)
            self._robot = robot
        if not self._connected:
            if not bool(robot.connect()):
                raise ConnectionError(
                    f'could not connect to RBY1 at {self.address}'
                )
            self._connected = True
        try:
            return (
                bool(robot.is_power_on(self.power_pattern)),
                bool(robot.is_servo_on(self.servo_pattern)),
            )
        except Exception:
            # A failed RPC invalidates the session. The next scheduled query
            # reconnects before reading state again.
            self._connected = False
            raise
