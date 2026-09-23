#!/usr/bin/env python3
"""Publish static transforms from the end effector to RealSense via its center."""

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def finite_values(values, count, label):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must contain {count} finite numbers') from exc
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label} must contain {count} finite numbers')
    return result


def rpy_degrees_to_quaternion(rotation_rpy_deg):
    """Convert ROS fixed-axis roll, pitch, yaw angles in degrees to x, y, z, w."""
    roll, pitch, yaw = (
        math.radians(value) for value in rotation_rpy_deg)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def normalize_quaternion(quaternion, label):
    """Return a normalized xyzw quaternion."""
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError(f'{label} must be a non-zero quaternion')
    return tuple(value / norm for value in quaternion)


def rotate_vector(quaternion, vector):
    """Rotate a three-dimensional vector by a normalized xyzw quaternion."""
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    # v' = v + 2*w*(q_xyz x v) + 2*(q_xyz x (q_xyz x v)).
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def make_transform(stamp, parent_frame, child_frame, translation, rotation):
    """Build a TransformStamped from validated values."""
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    (
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ) = translation
    (
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ) = rotation
    return transform


class CameraMountTFPublisher(Node):
    def __init__(self):
        super().__init__('camera_mount_tf_publisher')

        self.declare_parameter('mount_parent_frame', 'ee_left')
        self.declare_parameter('camera_center_frame', 'camera_center')
        self.declare_parameter('camera_link_frame', 'camera_link')
        self.declare_parameter(
            'parent_to_camera_center_translation', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'parent_to_camera_center_rotation', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'camera_center_to_camera_link_offset_in_link_axes',
            [0.0, 0.0, 0.0])
        self.declare_parameter(
            'camera_center_to_camera_link_rotation', [0.0, 0.0, 0.0])

        parent_frame = str(
            self.get_parameter('mount_parent_frame').value).strip()
        center_frame = str(
            self.get_parameter('camera_center_frame').value).strip()
        camera_frame = str(
            self.get_parameter('camera_link_frame').value).strip()
        parent_to_center_translation = finite_values(
            self.get_parameter(
                'parent_to_camera_center_translation').value,
            3,
            'parent_to_camera_center_translation')
        parent_to_center_rpy = finite_values(
            self.get_parameter('parent_to_camera_center_rotation').value,
            3,
            'parent_to_camera_center_rotation')
        center_to_link_offset = finite_values(
            self.get_parameter(
                'camera_center_to_camera_link_offset_in_link_axes').value,
            3,
            'camera_center_to_camera_link_offset_in_link_axes')
        center_to_link_rpy = finite_values(
            self.get_parameter(
                'camera_center_to_camera_link_rotation').value,
            3,
            'camera_center_to_camera_link_rotation')

        frames = (parent_frame, center_frame, camera_frame)
        if not all(frames):
            raise ValueError('TF frame names must not be empty')
        if len(set(frames)) != len(frames):
            raise ValueError('TF frame names must be different')

        parent_to_center_rotation = normalize_quaternion(
            rpy_degrees_to_quaternion(parent_to_center_rpy),
            'parent_to_camera_center_rotation')
        center_to_link_rotation = normalize_quaternion(
            rpy_degrees_to_quaternion(center_to_link_rpy),
            'camera_center_to_camera_link_rotation')

        # The center-to-link offset is expressed in the rotated camera_link
        # axes. Convert it to camera_center axes before filling the TF
        # translation field. This keeps camera_center as the physical pivot.
        center_to_link_translation = rotate_vector(
            center_to_link_rotation,
            center_to_link_offset,
        )

        stamp = self.get_clock().now().to_msg()
        center_transform = make_transform(
            stamp, parent_frame, center_frame,
            parent_to_center_translation, parent_to_center_rotation)
        link_transform = make_transform(
            stamp, center_frame, camera_frame,
            center_to_link_translation, center_to_link_rotation)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform([center_transform, link_transform])
        self.get_logger().info(
            f'Camera mount TF ready: {parent_frame} -> {center_frame} -> '
            f'{camera_frame}; '
            f'parent_to_center_translation={parent_to_center_translation}; '
            f'parent_to_center_rotation={parent_to_center_rpy} deg; '
            f'center_to_link_offset_in_link_axes={center_to_link_offset}; '
            f'center_to_link_translation={center_to_link_translation}; '
            f'center_to_link_rotation={center_to_link_rpy} deg')


def main(args=None):
    rclpy.init(args=args)
    node = CameraMountTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
