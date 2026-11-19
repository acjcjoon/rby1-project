#!/usr/bin/env python3
"""Save IR frames at a low rate (default 2 Hz) as PNG for scene analysis.

Recording raw IR into a bag at 30-90 fps fills the disk; this keeps a sparse,
timestamped sample instead. File name = header stamp in nanoseconds.

  source /opt/ros/humble/setup.bash
  PYTHONNOUSERSITE=1 python3 rby1_vslam/scripts/ir_snapshot.py OUT_DIR [--hz 2] \
      [--topic /d435/d435/infra1/image_rect_raw]
"""

import argparse
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Snapshot(Node):
    def __init__(self, out, topic, hz):
        super().__init__('ir_snapshot')
        self.out, self.period, self.last, self.count = out, 1.0 / hz, None, 0
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'saving {topic} at {hz} Hz to {out}')

    def on_image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last is not None and stamp - self.last < self.period:
            return
        if msg.encoding not in ('mono8', '8UC1'):
            self.get_logger().warn(f'unsupported encoding {msg.encoding}', once=True)
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        cv2.imwrite(os.path.join(self.out, f'{ns}.png'), img)
        self.last = stamp
        self.count += 1
        if self.count % 60 == 0:
            self.get_logger().info(f'{self.count} frames saved')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--hz', type=float, default=2.0)
    ap.add_argument('--topic', default='/d435/d435/infra1/image_rect_raw')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Snapshot(a.out, a.topic, a.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'stopped, {node.count} frames in {a.out}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
