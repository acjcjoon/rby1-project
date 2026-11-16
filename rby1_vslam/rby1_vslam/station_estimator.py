"""ROS-independent convergence window for stopped station corrections."""

from collections import deque
import math

from .geometry import finite, wrap, yaw


class ConvergenceWindow:
    """Require a continuous interval of consistent SE(2) correction samples."""

    def __init__(self, duration, minimum_samples, position_tolerance,
                 yaw_tolerance, maximum_gap):
        self.duration = duration
        self.minimum_samples = minimum_samples
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.maximum_gap = maximum_gap
        self.samples = deque()

    def clear(self):
        self.samples.clear()

    def add(self, stamp, position, rotation):
        x, y, _ = finite(position)
        angle = yaw(rotation)
        if self.samples:
            gap = stamp - self.samples[-1][0]
            if gap <= 0:
                return None
            if gap > self.maximum_gap:
                self.clear()
        self.samples.append((stamp, x, y, angle))
        # Retain the one sample straddling the window boundary.
        while len(self.samples) > 1 and self.samples[1][0] <= stamp - self.duration:
            self.samples.popleft()
        if (len(self.samples) < self.minimum_samples
                or stamp - self.samples[0][0] < self.duration):
            return None
        count = len(self.samples)
        mx = sum(s[1] for s in self.samples) / count
        my = sum(s[2] for s in self.samples) / count
        heading = math.atan2(sum(math.sin(s[3]) for s in self.samples),
                             sum(math.cos(s[3]) for s in self.samples))
        if any(math.hypot(s[1] - mx, s[2] - my) > self.position_tolerance
               or abs(wrap(s[3] - heading)) > self.yaw_tolerance
               for s in self.samples):
            return None
        return (mx, my, 0.), (0., 0., math.sin(heading / 2.), math.cos(heading / 2.))
