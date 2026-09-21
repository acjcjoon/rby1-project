#!/usr/bin/env python3
"""Publish static transforms from the end effector to RealSense via its center."""

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
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

        self.declare_parameter('parent_frame', 'ee_left')
        self.declare_parameter('camera_center_frame', 'camera_center')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('translation_xyz', [0.0, 0.0, 0.0])
        # Preferred, intuitive format. It is left unset so old YAML files that
        # only contain rotation_xyzw continue to work.
        self.declare_parameter(
            'rotation_rpy_deg', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('rotation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter(
            'camera_center_to_link_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'camera_center_rotation_rpy_deg', [0.0, 0.0, 0.0])

        parent_frame = str(self.get_parameter('parent_frame').value).strip()
        center_frame = str(
            self.get_parameter('camera_center_frame').value).strip()
        camera_frame = str(self.get_parameter('camera_frame').value).strip()
        translation = finite_values(
            self.get_parameter('translation_xyz').value, 3, 'translation_xyz')
        center_to_link = finite_values(
            self.get_parameter('camera_center_to_link_xyz').value, 3,
            'camera_center_to_link_xyz')
        center_delta_rpy = finite_values(
            self.get_parameter('camera_center_rotation_rpy_deg').value, 3,
            'camera_center_rotation_rpy_deg')
        rpy_parameter = self.get_parameter('rotation_rpy_deg')
        if rpy_parameter.type_ != Parameter.Type.NOT_SET:
            rotation_rpy_deg = finite_values(
                rpy_parameter.value, 3, 'rotation_rpy_deg')
            rotation = rpy_degrees_to_quaternion(rotation_rpy_deg)
            rotation_source = f'RPY degrees={rotation_rpy_deg}'
        else:
            rotation = finite_values(
                self.get_parameter('rotation_xyzw').value, 4,
                'rotation_xyzw')
            rotation_source = 'legacy quaternion'

        frames = (parent_frame, center_frame, camera_frame)
        if not all(frames):
            raise ValueError('TF frame names must not be empty')
        if len(set(frames)) != len(frames):
            raise ValueError('TF frame names must be different')

        rotation = normalize_quaternion(rotation, 'rotation_xyzw')
        center_delta = normalize_quaternion(
            rpy_degrees_to_quaternion(center_delta_rpy),
            'camera_center_rotation_rpy_deg')

        # translation/rotation directly describe ee -> camera_center. Apply
        # the offset and additional rotation only on camera_center ->
        # camera_link. Rotating the offset makes camera_link orbit around the
        # configured physical center for rotations about any local axis.
        link_translation = rotate_vector(center_delta, center_to_link)

        stamp = self.get_clock().now().to_msg()
        center_transform = make_transform(
            stamp, parent_frame, center_frame,
            translation, rotation)
        link_transform = make_transform(
            stamp, center_frame, camera_frame,
            link_translation, center_delta)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform([center_transform, link_transform])
        self.get_logger().info(
            f'Camera mount TF ready: {parent_frame} -> {center_frame} -> '
            f'{camera_frame}; center_translation={translation}; '
            f'center_rotation={rotation}; source={rotation_source}; '
            f'center_to_link={center_to_link}; '
            f'center_rotation_rpy_deg={center_delta_rpy}')


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
