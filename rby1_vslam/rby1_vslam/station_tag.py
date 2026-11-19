"""Ground-truth station error from a fixed AprilTag seen by the head D435 IR camera.

Two or more fixed tags per station. At registration their centres are captured
once in the base frame (reference); at every later arrival again. A planar rigid
fit of the two point sets gives the base displacement relative to the registered
pose — independent of VSLAM / wheel odometry. Only tag positions are used: one
tag's PnP orientation is noisy and the camera-to-tag distance would amplify it.
Small errors in T_base_cam largely cancel because the head pose is the same for
both captures.

cv2 is imported lazily: the operator UI must keep running if OpenCV is unusable.
"""

import math

import numpy as np

FAMILY = '36h11'


def _cv2():
    import cv2  # noqa: PLC0415
    import cv2.aruco  # noqa: F401,PLC0415
    return cv2


def detect(image, refine=True):
    """Return {tag_id: corners (4, 2) float} for tag36h11 markers in a mono8 image."""
    cv2 = _cv2()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters_create()
    if refine:
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 3
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = 0.01
    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=params)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(float) for i, c in zip(ids.ravel(), corners)}


def object_points(size):
    """Tag corners in the tag frame, in OpenCV ArUco order (TL, TR, BR, BL), z out of the tag."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def tag_pose(corners, camera_matrix, distortion, size):
    """4x4 T_cam_tag from image corners (PnP, IPPE_SQUARE)."""
    cv2 = _cv2()
    ok, rvec, tvec = cv2.solvePnP(object_points(size), np.asarray(corners, float),
                                  np.asarray(camera_matrix, float),
                                  np.asarray(distortion, float), flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError('PnP failed')
    rot, _ = cv2.Rodrigues(rvec)
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = rot, tvec.ravel()
    return out


def reprojection_px(corners, T_cam_tag, camera_matrix, distortion, size):
    cv2 = _cv2()
    rvec, _ = cv2.Rodrigues(T_cam_tag[:3, :3])
    proj, _ = cv2.projectPoints(object_points(size), rvec, T_cam_tag[:3, 3],
                                np.asarray(camera_matrix, float), np.asarray(distortion, float))
    return float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - corners) ** 2, axis=1))))


def _quat(R):
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def _rot(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def average(transforms):
    """Robust mean of 4x4 poses: median translation, sign-aligned quaternion mean.

    Returns (T_mean, translation_sd_mm, rotation_sd_deg)."""
    if not transforms:
        raise ValueError('no poses to average')
    ts = np.array([T[:3, 3] for T in transforms])
    qs = np.array([_quat(T[:3, :3]) for T in transforms])
    qs[qs @ qs[0] < 0] *= -1.0
    q = qs.mean(axis=0)
    q /= np.linalg.norm(q)
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = _rot(q), np.median(ts, axis=0)
    t_sd = float(np.linalg.norm(ts.std(axis=0)) * 1000.0)
    angles = [2.0 * math.degrees(math.acos(min(1.0, abs(float(qi @ q))))) for qi in qs]
    r_sd = float(np.sqrt(np.mean(np.square(angles))))
    return out, t_sd, r_sd


def invert(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def tag_points(tags_T_cam_tag, T_base_cam):
    """Tag centres in the base frame: {id: (x, y, z)} from {id: T_cam_tag}.

    Only the tag *positions* are used downstream. A single planar tag's
    orientation is noisy (~1-2 deg) and would be amplified by the camera-to-tag
    distance; points from two or more spread-out tags avoid that lever arm."""
    out = {}
    for tag_id, T in tags_T_cam_tag.items():
        p = T_base_cam @ np.append(np.asarray(T)[:3, 3], 1.0)
        out[int(tag_id)] = tuple(float(v) for v in p[:3])
    return out


def rigid_2d(ref_points, now_points):
    """Planar base displacement from tag centres seen at registration and now.

    Fixed tags satisfy p_ref = R(yaw) p_now + t, so (t, yaw) is the current base
    pose in the registered station frame. Returns dict with x_mm (+ ahead),
    y_mm (+ left), err_mm, yaw_deg (+ CCW), residual_mm (fit RMS, flags a moved
    tag or a bad detection), z_mm (mean height change, should be ~0) and ids."""
    ids = sorted(set(ref_points) & set(now_points))
    if len(ids) < 2:
        raise ValueError('need the same >= 2 tags in the reference and the current view')
    A = np.array([now_points[i][:2] for i in ids])
    B = np.array([ref_points[i][:2] for i in ids])
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    yaw = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    t = cb - R @ ca
    residual = np.sqrt(np.mean(np.sum((A @ R.T + t - B) ** 2, axis=1)))
    dz = np.mean([ref_points[i][2] - now_points[i][2] for i in ids])
    x, y = t * 1000.0
    return {'x_mm': float(x), 'y_mm': float(y), 'err_mm': float(math.hypot(x, y)),
            'yaw_deg': float(math.degrees(yaw)), 'residual_mm': float(residual * 1000.0),
            'z_mm': float(dz * 1000.0), 'ids': ids}


def mean_corners(frames):
    """Average per-tag corners over static frames: {id: (corners (4,2), sd_px, n)}."""
    per = {}
    for det in frames:
        for tag_id, c in det.items():
            per.setdefault(tag_id, []).append(np.asarray(c, float))
    out = {}
    for tag_id, cs in per.items():
        arr = np.array(cs)
        out[tag_id] = (arr.mean(axis=0), float(np.sqrt(np.mean(arr.var(axis=0).sum(axis=1)))), len(cs))
    return out


def reference_corners(corners_px, camera_matrix, distortion, size, T_base_cam):
    """Registration: 3D tag corners in the station base frame {id: (4, 3)} via per-tag PnP."""
    out = {}
    for tag_id, c in corners_px.items():
        T = tag_pose(c, camera_matrix, distortion, size)
        pts = (T @ np.c_[object_points(size), np.ones(4)].T).T
        out[int(tag_id)] = (T_base_cam @ pts.T).T[:, :3]
    return out


def _se2(x, y, yaw):
    T = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3], T[1, 3] = x, y
    return T


def solve_planar(ref_corners_base, corners_px, camera_matrix, distortion, T_base_cam):
    """Current base pose in the registered station frame from tag-corner bearings.

    Fixed 3D corners (captured at registration) are reprojected into the current
    image under a planar base motion (x, y, yaw); only image positions are
    compared, so the PnP depth bias of the reference enters only in proportion
    to the displacement. Returns x_mm (+ ahead), y_mm (+ left), err_mm,
    yaw_deg (+ CCW), residual_px, ids."""
    cv2 = _cv2()
    from scipy.optimize import least_squares  # noqa: PLC0415
    ids = sorted(set(ref_corners_base) & set(corners_px))
    if not ids:
        raise ValueError('none of the reference tags is visible')
    P = np.vstack([ref_corners_base[i] for i in ids])
    obs = np.vstack([np.asarray(corners_px[i], float) for i in ids])
    K = np.asarray(camera_matrix, float)
    dist = np.asarray(distortion, float)
    T_cam_base = invert(T_base_cam)

    def residual(v):
        T_now_ref = invert(_se2(*v))
        pts = (T_cam_base @ T_now_ref @ np.c_[P, np.ones(len(P))].T).T[:, :3]
        proj, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K, dist)
        return (proj.reshape(-1, 2) - obs).ravel()

    sol = least_squares(residual, np.zeros(3), x_scale=[0.01, 0.01, 0.01], method='lm')
    x, y, yaw = sol.x
    rms = float(np.sqrt(np.mean(sol.fun.reshape(-1, 2) ** 2) * 2))
    return {'x_mm': float(x * 1000), 'y_mm': float(y * 1000), 'err_mm': float(math.hypot(x, y) * 1000),
            'yaw_deg': float(math.degrees(yaw)), 'residual_px': rms, 'ids': ids}


def to_list(T):
    return [[float(v) for v in row] for row in np.asarray(T)]


def from_list(rows):
    T = np.asarray(rows, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError('expected a finite 4x4 matrix')
    return T
