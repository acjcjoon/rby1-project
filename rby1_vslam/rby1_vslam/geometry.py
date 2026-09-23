"""ROS-independent rigid-pose math (quaternions are x, y, z, w)."""
import math


def finite(values):
    result = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise ValueError('non-finite geometry')
    return result


def quaternion(q):
    q = finite(q)
    if len(q) != 4:
        raise ValueError('quaternion requires four values')
    n = math.sqrt(sum(v * v for v in q))
    if not 0.9 <= n <= 1.1:
        raise ValueError('quaternion is not approximately unit length')
    return tuple(v / n for v in q)


def multiply(a, b):
    x, y, z, w = quaternion(a)
    X, Y, Z, W = quaternion(b)
    return quaternion((w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                       w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z))


def rotate(q, p):
    x, y, z, w = quaternion(q)
    px, py, pz = finite(p)
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    return (px+w*tx+y*tz-z*ty, py+w*ty+z*tx-x*tz, pz+w*tz+x*ty-y*tx)


def compose(a_position, a_rotation, b_position, b_rotation):
    p = finite(a_position)
    r = rotate(a_rotation, b_position)
    return tuple(x+y for x, y in zip(p, r)), multiply(a_rotation, b_rotation)


def inverse(position, rotation):
    x, y, z, w = quaternion(rotation)
    q = (-x, -y, -z, w)
    return rotate(q, tuple(-v for v in finite(position))), q


def planar_correction(map_base_position, map_base_rotation, odom_base_position, odom_base_rotation):
    """map->odom in SE(2), keeping Nav2's costmap plane aligned to wheel odom."""
    mx, my, _ = finite(map_base_position)
    ox, oy, _ = finite(odom_base_position)
    angle = wrap(yaw(map_base_rotation)-yaw(odom_base_rotation))
    c, s = math.cos(angle), math.sin(angle)
    return (mx-c*ox+s*oy, my-s*ox-c*oy, 0.), (0., 0., math.sin(angle/2), math.cos(angle/2))


def yaw(q):
    x, y, z, w = quaternion(q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def offset_covariance(covariance, world_camera_q, camera_base_translation):
    """Propagate pose covariance for a deterministic right-hand frame offset.

    ROS pose errors use fixed world axes. dp_base = dp_camera - [R*t]x dtheta.
    Encoder, timing and mount-calibration uncertainty is not included.
    """
    cov = finite(covariance)
    if len(cov) != 36 or any(cov[i*6+i] < 0 for i in range(6)):
        raise ValueError('invalid covariance')
    x, y, z = rotate(world_camera_q, camera_base_translation)
    minus_skew = ((0., z, -y), (-z, 0., x), (y, -x, 0.))
    jac = [[float(i == j) for j in range(6)] for i in range(6)]
    for i in range(3):
        for j in range(3):
            jac[i][j+3] = minus_skew[i][j]
    tmp = [[sum(jac[i][k]*cov[k*6+j] for k in range(6))
            for j in range(6)] for i in range(6)]
    return [sum(tmp[i][k]*jac[j][k] for k in range(6))
            for i in range(6) for j in range(6)]
