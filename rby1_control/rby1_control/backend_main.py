"""Executable entry point for the single robot-facing control node."""
from __future__ import annotations

import time

import rclpy

from .control_node import RBY1ControlNode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RBY1ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.shutdown_safely(turn_stream_off=True)
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
