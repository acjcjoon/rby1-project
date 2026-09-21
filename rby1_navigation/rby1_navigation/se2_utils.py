"""Small SE(2) helpers shared by odometry navigation code."""

import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def pose2d_from_pose(pose):
    return (
        float(pose.position.x),
        float(pose.position.y),
        yaw_from_quaternion(pose.orientation),
    )


def compose(first, second):
    """Compose ``(x, y, yaw)`` transforms."""
    ax, ay, atheta = first
    bx, by, btheta = second
    cosine = math.cos(atheta)
    sine = math.sin(atheta)
    return (
        ax + cosine * bx - sine * by,
        ay + sine * bx + cosine * by,
        normalize_angle(atheta + btheta),
    )


def body_frame_error(current, target):
    """Return target translation in current body axes and wrapped yaw error."""
    dx_world = target[0] - current[0]
    dy_world = target[1] - current[1]
    cosine = math.cos(current[2])
    sine = math.sin(current[2])
    return (
        cosine * dx_world + sine * dy_world,
        -sine * dx_world + cosine * dy_world,
        normalize_angle(target[2] - current[2]),
    )


def proportional_velocity(
    current,
    target,
    *,
    linear_kp,
    angular_kp,
    max_linear,
    max_angular,
):
    """Calculate a holonomic P command with vector and yaw saturation."""
    dx_body, dy_body, yaw_error = body_frame_error(current, target)
    vx = linear_kp * dx_body
    vy = linear_kp * dy_body
    speed = math.hypot(vx, vy)
    if speed > max_linear:
        scale = max_linear / speed
        vx *= scale
        vy *= scale
    wz = max(-max_angular, min(max_angular, angular_kp * yaw_error))
    return vx, vy, wz
