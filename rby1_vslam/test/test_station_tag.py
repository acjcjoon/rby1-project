"""Synthetic check of the AprilTag ground-truth chain: render -> detect -> PnP -> base error."""
import math

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')
pytest.importorskip('cv2.aruco')

from rby1_vslam import station_tag as st  # noqa: E402

K = np.array([[385.0, 0, 320.0], [0, 385.0, 240.0], [0, 0, 1]])   # D435 IR 640x480 class
D = np.zeros(5)
SIZE = 0.16


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def pose(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


# Camera optical frame on the base: 1.4 m up, looking forward (+x base), pitched down 20 deg.
R_base_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.0]])
pitch = math.radians(20)
R_pitch = np.array([[1, 0, 0], [0, math.cos(pitch), -math.sin(pitch)], [0, math.sin(pitch), math.cos(pitch)]])
T_BASE_CAM = pose(R_base_opt @ R_pitch.T, [0.05, 0.0, 1.40])
# Tag on a stand 1.2 m ahead, centre 1.0 m high, facing the robot (tag z towards -x base).
T_WORLD_TAG = pose(np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0.0]]), [1.25, 0.05, 1.0])


# Recommended station layout: two near tags 1 m ahead (0.6 m apart) + one far tag 2.2 m ahead.
R_FACE = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0.0]])
WORLD_TAGS = {20: pose(R_FACE, [1.0, 0.30, 0.9]), 21: pose(R_FACE, [1.0, -0.30, 0.9]),
              22: pose(R_FACE, [2.2, 0.0, 1.2])}


def render(cam_tags, noise=1.0, seed=0):
    canvas = np.full((480, 640), 120.0)
    side = 400
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for tag_id, T_cam_tag in cam_tags.items():
        marker = cv2.aruco.drawMarker(dictionary, tag_id, side)
        pad = side // 4
        tag = np.full((side + 2 * pad, side + 2 * pad), 255, np.uint8)
        tag[pad:pad + side, pad:pad + side] = marker
        s = SIZE / 2 * (side + 2 * pad) / side
        obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
        rvec, _ = cv2.Rodrigues(T_cam_tag[:3, :3])
        img_pts, _ = cv2.projectPoints(obj, rvec, T_cam_tag[:3, 3], K, D)
        n = tag.shape[0] - 1
        H = cv2.getPerspectiveTransform(np.float32([[0, 0], [n, 0], [n, n], [0, n]]),
                                        np.float32(img_pts.reshape(4, 2)))
        warped = cv2.warpPerspective(tag, H, (640, 480), flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpPerspective(np.ones_like(tag, np.float32), H, (640, 480),
                                   flags=cv2.INTER_LINEAR, borderValue=0)
        canvas = canvas * (1 - mask) + warped * mask
    rng = np.random.default_rng(seed)
    return np.clip(canvas + rng.normal(0, noise, canvas.shape), 0, 255).astype(np.uint8)


def cam_tags_for_base(T_world_base):
    T_world_cam = T_world_base @ T_BASE_CAM
    return {i: st.invert(T_world_cam) @ T for i, T in WORLD_TAGS.items()}


def measure(T_world_base, frames=30):
    truth = cam_tags_for_base(T_world_base)
    per_tag = {i: [] for i in truth}
    for k in range(frames):
        det = st.detect(render(truth, seed=k + int(1000 * abs(T_world_base[0, 3]))))
        for i in truth:
            assert i in det
            per_tag[i].append(st.tag_pose(det[i], K, D, SIZE))
    return {i: st.average(v)[0] for i, v in per_tag.items()}


def test_single_frame_tag_position_close_to_truth():
    truth = cam_tags_for_base(np.eye(4))
    det = st.detect(render(truth))
    for i, T_true in truth.items():
        T = st.tag_pose(det[i], K, D, SIZE)
        assert np.linalg.norm(T[:3, 3] - T_true[:3, 3]) * 1000 < 30.0  # one frame, up to 2.2 m
        assert st.reprojection_px(det[i], T, K, D, SIZE) < 0.5


def corners_for(T_world_base, seed0, frames=30):
    truth = cam_tags_for_base(T_world_base)
    dets = [st.detect(render(truth, seed=seed0 + k)) for k in range(frames)]
    return {i: c for i, (c, sd, n) in st.mean_corners(dets).items()}


@pytest.mark.parametrize('dx,dy,dyaw', [(0.0, 0.0, 0.0), (0.015, -0.010, 1.0), (-0.030, 0.020, -2.0),
                                        (0.0, 0.020, 0.0), (0.0, 0.0, 1.5)])
def test_planar_solve_recovers_known_displacement(dx, dy, dyaw):
    ref = st.reference_corners(corners_for(np.eye(4), 0), K, D, SIZE, T_BASE_CAM)
    now = corners_for(pose(rot_z(dyaw), [dx, dy, 0.0]), 500)
    r = st.solve_planar(ref, now, K, D, T_BASE_CAM)
    assert r['x_mm'] == pytest.approx(dx * 1000, abs=1.5)
    assert r['y_mm'] == pytest.approx(dy * 1000, abs=1.5)
    assert r['yaw_deg'] == pytest.approx(dyaw, abs=0.1)
    assert r['residual_px'] < 1.0 and r['ids'] == [20, 21, 22]


def test_planar_solve_needs_a_reference_tag():
    with pytest.raises(ValueError):
        st.solve_planar({99: np.zeros((4, 3))}, {20: np.zeros((4, 2))}, K, D, T_BASE_CAM)


def test_rigid_fit_exact_points():
    ref = {1: (1.0, 0.5, 1.0), 2: (1.0, -0.5, 1.0), 3: (2.0, 0.0, 1.0)}
    c, s = math.cos(math.radians(-3)), math.sin(math.radians(-3))
    # now = inverse motion applied to ref points (base moved by +20 mm x, +3 deg yaw)
    now = {k: (c * (x - 0.02) - s * y, s * (x - 0.02) + c * y, z) for k, (x, y, z) in ref.items()}
    err = st.rigid_2d(ref, now)
    assert err['x_mm'] == pytest.approx(20.0, abs=1e-6)
    assert err['y_mm'] == pytest.approx(0.0, abs=1e-6)
    assert err['yaw_deg'] == pytest.approx(3.0, abs=1e-6)
    with pytest.raises(ValueError):
        st.rigid_2d({1: ref[1]}, {1: now[1]})


def test_average_and_list_roundtrip():
    T = pose(rot_z(30), [1, 2, 3])
    mean, t_sd, r_sd = st.average([T, T])
    assert np.allclose(mean, T) and t_sd == 0 and r_sd == pytest.approx(0, abs=1e-6)
    assert np.allclose(st.from_list(st.to_list(T)), T)
