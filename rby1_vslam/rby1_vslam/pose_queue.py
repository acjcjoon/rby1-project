"""Small ROS-independent per-kind FIFO used by the pose adapter."""

from collections import deque


class PoseQueue:
    """Bound each pose stream independently while preserving FIFO order."""

    def __init__(self, kinds, capacity):
        if capacity < 1:
            raise ValueError('pose queue capacity must be positive')
        self.kinds = tuple(kinds)
        self.capacity = capacity
        self._queues = {kind: deque() for kind in self.kinds}

    def put(self, kind, item):
        queue = self._queues[kind]
        dropped = queue.popleft() if len(queue) >= self.capacity else None
        queue.append(item)
        return dropped

    def peek(self, kind):
        queue = self._queues[kind]
        return queue[0] if queue else None

    def pop(self, kind):
        return self._queues[kind].popleft()

    def depth(self, kind):
        return len(self._queues[kind])

    def __bool__(self):
        return any(self._queues.values())
