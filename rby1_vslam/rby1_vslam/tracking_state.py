"""ROS-independent health checks for a local Isaac ROS tracker."""

import math
import uuid


class TrackingState:
    def __init__(self, timeout=0.5, require_localized=False):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('tracking timeout must be positive and finite')
        self.timeout = timeout
        self.require_localized = require_localized
        self.new_session()

    def new_session(self):
        self.session = uuid.uuid4().hex
        self.status = self.odom = self.diagnostics = None

    def _observe(self, kind, value, stamp, received):
        if not all(math.isfinite(v) for v in (stamp, received)) or stamp <= 0:
            setattr(self, kind, None)
            return False
        previous = getattr(self, kind)
        if previous is not None:
            if stamp < previous[1]:
                self.new_session()
            elif stamp == previous[1]:
                return False  # Replayed messages cannot refresh the watchdog.
        setattr(self, kind, (value, stamp, received))
        return True

    def observe_status(self, vo_state, stamp, received):
        return self._observe('status', vo_state == 1, stamp, received)

    def observe_odometry(self, stamp, received):
        return self._observe('odom', True, stamp, received)

    def observe_localization(self, localized, stamp, received):
        return self._observe('diagnostics', localized is True, stamp, received)

    def _fresh(self, sample, now, ros_now):
        return (sample is not None and 0 <= now - sample[2] <= self.timeout
                and -0.05 <= ros_now - sample[1] <= self.timeout)

    def report(self, now, ros_now):
        clock_ok = all(math.isfinite(v) for v in (now, ros_now))
        ready = clock_ok and self._fresh(self.status, now, ros_now)
        tracking = bool(ready and self.status[0] and self._fresh(self.odom, now, ros_now))
        localized = bool(clock_ok and self._fresh(self.diagnostics, now, ros_now)
                         and self.diagnostics[0])
        if not ready:
            detail = 'waiting for fresh Isaac ROS status'
        elif not tracking:
            detail = 'visual tracking lost or camera odometry expired'
        elif self.require_localized and not localized:
            detail = 'waiting for localization in the saved map'
        else:
            detail = 'local visual tracking ready'
        return {'ready': bool(ready), 'tracking_ok': bool(tracking and (
                    not self.require_localized or localized)),
                'localized': localized, 'session_id': self.session, 'detail': detail}
