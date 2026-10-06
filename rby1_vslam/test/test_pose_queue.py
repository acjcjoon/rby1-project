"""ROS-free tests for the PoseAdapter's bounded per-kind FIFO."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rby1_vslam.pose_queue import PoseQueue


def test_pose_queue_preserves_each_stream_and_reports_no_drop_below_capacity():
    queue = PoseQueue(('odom', 'slam_odom'), 3)
    assert queue.put('odom', 1) is None
    assert queue.put('odom', 2) is None
    assert queue.put('slam_odom', 10) is None
    assert queue.depth('odom') == 2
    assert queue.peek('odom') == 1
    assert queue.pop('odom') == 1
    assert queue.pop('odom') == 2
    assert queue.pop('slam_odom') == 10
    assert not queue


def test_pose_queue_drops_only_oldest_item_of_full_stream():
    queue = PoseQueue(('odom', 'slam_odom'), 2)
    queue.put('odom', 1)
    queue.put('slam_odom', 10)
    queue.put('odom', 2)
    assert queue.put('odom', 3) == 1
    assert queue.depth('slam_odom') == 1
    assert queue.pop('odom') == 2
    assert queue.pop('odom') == 3
    assert queue.pop('slam_odom') == 10


def test_pose_queue_rejects_invalid_capacity_and_kind():
    with pytest.raises(ValueError):
        PoseQueue(('odom',), 0)
    queue = PoseQueue(('odom',), 1)
    with pytest.raises(KeyError):
        queue.put('unknown', object())
