#!/usr/bin/env python3
"""
AprilTag과 aligned depth로 태그 포즈를 발행한다.

apriltag_ros의 /detections(태그 ID + 코너 픽셀)와 RealSense의 aligned depth를 결합해
태그 포즈를 두 가지 방식으로 만들고 TF/PoseArray로 퍼블리시하는 노드.

  1) depth 기반: 코너 depth로 평면을 추정해 직접 계산  -> tag_depth_<id>
  2) PnP 기반:  apriltag_ros TF를 재가공하거나, 왜곡 보정 시
                검출 코너와 CameraInfo(K, D)로 직접 계산 -> tag_pnp_<id>

두 방식 모두 공통 설정(config/vision.yaml)의 trigger_mode / samples_per_trigger /
축 반전(apply_flip, pnp_apply_flip)을 따른다.

- 카메라 장치를 직접 열지 않는다. realsense2_camera 드라이버 노드가 송출하는 토픽만 구독한다.
- realsense2_camera는 align_depth.enable:=true 로 실행되어야 한다.
  (aligned_depth_to_color 이미지는 color와 같은 픽셀 좌표계이므로 color intrinsics로 deproject 가능)
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster


def rpy_deg_to_matrix(rpy_deg):
    """
    [roll, pitch, yaw]를 3x3 회전 행렬(Rz @ Ry @ Rx)로 변환한다.

    [180, 0, 0]은 기존 apply_flip과 동일하고 임의 축 치환도 가능하다.
    """
    r, p, y = (math.radians(a) for a in rpy_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotation_matrix_to_quaternion(R):
    """3x3 회전 행렬 -> (x, y, z, w) 쿼터니언 (Shepperd 방법)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def quaternion_to_rotation_matrix(q):
    """(x, y, z, w) 쿼터니언 -> 3x3 회전 행렬."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def average_quaternions(quats):
    """부호를 정렬한 뒤 평균 내고 정규화. 샘플들이 서로 가까울 때 유효한 근사."""
    ref = quats[0]
    acc = np.zeros(4)
    for q in quats:
        q = np.asarray(q)
        if np.dot(q, ref) < 0:
            q = -q
        acc += q
    return acc / np.linalg.norm(acc)


def quaternion_to_euler_deg(qx, qy, qz, qw):
    """(x, y, z, w) 쿼터니언 -> Roll/Pitch/Yaw [deg] (로그 출력용)."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def undistort_plumb_bob_normalized(u, v, camera_matrix, distortion,
                                   iterations=8):
    """Raw pixel -> undistorted normalized coordinate for plumb_bob."""
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    if abs(fx) <= 1.0e-12 or abs(fy) <= 1.0e-12:
        raise ValueError('camera focal lengths must be non-zero')

    # ROS plumb_bob order: [k1, k2, p1, p2, k3].
    coeffs = np.zeros(5, dtype=float)
    values = np.asarray(distortion, dtype=float).reshape(-1)
    coeffs[:min(5, values.size)] = values[:5]
    k1, k2, p1, p2, k3 = coeffs

    xd = (float(u) - cx) / fx
    yd = (float(v) - cy) / fy
    x, y = xd, yd
    for _ in range(iterations):
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        if abs(radial) <= 1.0e-12:
            raise ValueError('invalid radial distortion scale')
        delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xd - delta_x) / radial
        y = (yd - delta_y) / radial
    return np.array([x, y], dtype=float)


def rotation_vector_to_matrix(rotation_vector):
    """Small/large axis-angle vector -> rotation matrix."""
    vector = np.asarray(rotation_vector, dtype=float)
    theta = np.linalg.norm(vector)
    wx, wy, wz = vector
    skew = np.array([
        [0.0, -wz, wy],
        [wz, 0.0, -wx],
        [-wy, wx, 0.0],
    ])
    if theta <= 1.0e-12:
        return np.eye(3) + skew
    return (np.eye(3) + math.sin(theta) / theta * skew +
            (1.0 - math.cos(theta)) / (theta * theta) * (skew @ skew))


def refine_planar_pose(normalized_points, object_points, rotation,
                       translation, max_iterations=12):
    """Refine a planar homography pose by normalized reprojection error."""
    observed = np.asarray(normalized_points, dtype=float)
    objects = np.asarray(object_points, dtype=float)
    R = np.asarray(rotation, dtype=float).copy()
    t = np.asarray(translation, dtype=float).copy()

    def project(current_R, current_t):
        points = (current_R @ objects.T).T + current_t
        if np.any(points[:, 2] <= 1.0e-9):
            return None
        return points[:, :2] / points[:, 2, np.newaxis]

    for _ in range(max_iterations):
        predicted = project(R, t)
        if predicted is None:
            break
        residual = (observed - predicted).reshape(-1)
        old_error = float(residual @ residual)
        jacobian = np.empty((residual.size, 6), dtype=float)
        epsilon = 1.0e-6
        for index in range(6):
            if index < 3:
                delta_rotation = np.zeros(3)
                delta_rotation[index] = epsilon
                perturbed = project(
                    rotation_vector_to_matrix(delta_rotation) @ R, t)
            else:
                perturbed_t = t.copy()
                perturbed_t[index - 3] += epsilon
                perturbed = project(R, perturbed_t)
            if perturbed is None:
                return t, R
            jacobian[:, index] = ((perturbed - predicted) / epsilon).reshape(-1)

        try:
            step = np.linalg.lstsq(jacobian, residual, rcond=None)[0]
        except np.linalg.LinAlgError:
            break
        if not np.all(np.isfinite(step)):
            break

        # Backtracking prevents a noisy planar solution from diverging.
        accepted = False
        scale = 1.0
        for _ in range(8):
            candidate_R = rotation_vector_to_matrix(
                step[:3] * scale) @ R
            candidate_t = t + step[3:] * scale
            candidate = project(candidate_R, candidate_t)
            if candidate is not None:
                candidate_residual = (observed - candidate).reshape(-1)
                if float(candidate_residual @ candidate_residual) < old_error:
                    R, t = candidate_R, candidate_t
                    accepted = True
                    break
            scale *= 0.5
        if not accepted or np.linalg.norm(step * scale) <= 1.0e-10:
            break
    return t, R


def estimate_square_pose_from_normalized(normalized_corners, tag_size):
    """
    Estimate camera<-tag pose from four undistorted normalized corners.

    The corner/object ordering matches apriltag_ros 3.4.0. Because the tag
    is planar, its homography can be decomposed into translation and rotation.
    """
    image_points = np.asarray(normalized_corners, dtype=float)
    if image_points.shape != (4, 2):
        raise ValueError('normalized_corners must have shape (4, 2)')
    if not np.all(np.isfinite(image_points)):
        raise ValueError('normalized_corners must be finite')
    size = float(tag_size)
    if not math.isfinite(size) or size <= 0.0:
        raise ValueError('tag_size must be positive')

    half = size * 0.5
    object_points_2d = np.array([
        [-half, -half],
        [+half, -half],
        [+half, +half],
        [-half, +half],
    ], dtype=float)

    # Solve the planar homography with h33 fixed to one.
    A = []
    b = []
    for (x_obj, y_obj), (x_img, y_img) in zip(object_points_2d, image_points):
        A.append([x_obj, y_obj, 1.0, 0.0, 0.0, 0.0,
                  -x_img * x_obj, -x_img * y_obj])
        b.append(x_img)
        A.append([0.0, 0.0, 0.0, x_obj, y_obj, 1.0,
                  -y_img * x_obj, -y_img * y_obj])
        b.append(y_img)
    try:
        h = np.linalg.solve(np.asarray(A), np.asarray(b))
    except np.linalg.LinAlgError as exc:
        raise ValueError('degenerate tag corners') from exc
    H = np.array([
        [h[0], h[1], h[2]],
        [h[3], h[4], h[5]],
        [h[6], h[7], 1.0],
    ])

    h1, h2, h3 = H[:, 0], H[:, 1], H[:, 2]
    scale = 0.5 * (np.linalg.norm(h1) + np.linalg.norm(h2))
    if not math.isfinite(scale) or scale <= 1.0e-12:
        raise ValueError('invalid homography scale')
    r1 = h1 / scale
    r2 = h2 / scale
    t = h3 / scale
    if t[2] < 0.0:
        r1, r2, t = -r1, -r2, -t

    R_approx = np.column_stack((r1, r2, np.cross(r1, r2)))
    U, _, Vt = np.linalg.svd(R_approx)
    R = U @ Vt
    if np.linalg.det(R) < 0.0:
        U[:, -1] *= -1.0
        R = U @ Vt
    object_points_3d = np.column_stack(
        (object_points_2d, np.zeros(4, dtype=float)))
    return refine_planar_pose(
        image_points, object_points_3d, R, t)


class DepthTagTFNode(Node):
    def __init__(self):
        super().__init__('depth_tag_tf_node')

        # ---- 파라미터 (config/vision.yaml에서 조절) ----
        self.declare_parameter('output_frame', '')
        self.declare_parameter('tf_prefix', 'tag_depth_')
        self.declare_parameter('depth_window_radius', 1)
        self.declare_parameter('apply_distortion_correction', False)
        # 태그 프레임에 추가로 곱할 회전 오프셋 [roll, pitch, yaw] (도).
        # [180, 0, 0] = 기존 apply_flip과 동일, [0, 0, 0] = 오프셋 없음, 임의 축 치환 가능.
        self.declare_parameter('rotation_offset_rpy_deg', [180.0, 0.0, 0.0])
        self.declare_parameter('trigger_mode', False)
        self.declare_parameter('samples_per_trigger', 100)
        # 트리거 모드에서 평균 전에 버릴 outlier 비율 (0.0 = 안 버림, 0.2 = 20% 버림)
        self.declare_parameter('outlier_trim_ratio', 0.2)
        # PnP(apriltag_ros TF) 재가공 관련
        self.declare_parameter('pnp_enable', True)
        self.declare_parameter('pnp_tf_prefix', 'tag_pnp_')
        self.declare_parameter('pnp_tag_frame_format', 'tag36h11:{id}')
        self.declare_parameter('size', 0.035)
        self.declare_parameter('pnp_rotation_offset_rpy_deg', [0.0, 0.0, 0.0])

        self.output_frame = str(self.get_parameter('output_frame').value).strip()
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.win_r = int(self.get_parameter('depth_window_radius').value)
        self.apply_distortion_correction = bool(
            self.get_parameter('apply_distortion_correction').value)
        self.trigger_mode = bool(self.get_parameter('trigger_mode').value)
        self.samples_per_trigger = int(self.get_parameter('samples_per_trigger').value)
        self.outlier_trim_ratio = min(
            0.9, max(
                0.0,
                float(self.get_parameter('outlier_trim_ratio').value)))
        self.pnp_enable = bool(self.get_parameter('pnp_enable').value)
        self.pnp_tf_prefix = self.get_parameter('pnp_tf_prefix').value
        self.pnp_frame_fmt = self.get_parameter('pnp_tag_frame_format').value
        self.pnp_tag_size = float(self.get_parameter('size').value)
        if not math.isfinite(self.pnp_tag_size) or self.pnp_tag_size <= 0.0:
            raise ValueError('size must be a positive finite number')

        def offset_matrix(param_name):
            rpy = [float(v) for v in self.get_parameter(param_name).value]
            R = rpy_deg_to_matrix(rpy)
            return None if np.allclose(R, np.eye(3)) else R

        self.rot_offset = offset_matrix('rotation_offset_rpy_deg')
        self.pnp_rot_offset = offset_matrix('pnp_rotation_offset_rpy_deg')

        self.prefix_by_source = {'depth': self.tf_prefix, 'pnp': self.pnp_tf_prefix}

        # 트리거 모드 상태
        self.collecting = False
        self.frames_left = 0
        self.samples = {}          # (source, tag_id) -> {'t': [...], 'q': [...]}
        self.last_frame_id = ''

        # color intrinsics 캐시 (aligned depth는 color와 같은 좌표계)
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_matrix = None
        self.distortion = np.zeros(5, dtype=float)
        self.distortion_model = ''
        self.distortion_supported = False
        self.distortion_warning_emitted = False

        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 10)

        # detections와 depth 이미지를 타임스탬프 기준으로 근사 동기화
        detections_sub = Subscriber(self, AprilTagDetectionArray, 'detections')
        depth_sub = Subscriber(self, Image, 'depth_image')
        self.sync = ApproximateTimeSynchronizer([detections_sub, depth_sub],
                                                queue_size=10, slop=0.05)
        self.sync.registerCallback(self.detections_callback)

        # apriltag_ros가 송출한 PnP TF를 조회하기 위한 리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pubs = {
            'depth': self.create_publisher(PoseArray, 'tag_depth_poses', 10),
            'pnp': self.create_publisher(PoseArray, 'tag_pnp_poses', 10),
        }
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # 트리거 토픽 (trigger_mode=True일 때):
        #   ros2 topic pub --once /capture std_msgs/msg/Empty "{}"
        self.create_subscription(Empty, 'capture', self.capture_callback, 10)

        mode = 'trigger' if self.trigger_mode else 'realtime'
        pnp = 'on' if self.pnp_enable else 'off'
        distortion = 'on' if self.apply_distortion_correction else 'off'
        self.get_logger().info(
            f'depth_tag_tf_node started (mode: {mode}, pnp: {pnp}, '
            f'distortion correction: {distortion}). '
            f'output_frame: {self.output_frame or "input image frame"}. '
            'Waiting for camera_info / detections / depth...')

    # ---------------- 공통 유틸 ----------------

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.distortion = np.asarray(msg.d, dtype=float)
        self.distortion_model = str(msg.distortion_model)
        self.distortion_supported = (
            self.distortion_model == 'plumb_bob' and
            self.distortion.size >= 4 and
            np.all(np.isfinite(self.distortion)))
        if (self.apply_distortion_correction and
                not self.distortion_supported and
                not self.distortion_warning_emitted):
            self.get_logger().warn(
                'Distortion correction requested, but CameraInfo does not '
                f'contain supported plumb_bob coefficients (model="'
                f'{self.distortion_model}", D length={self.distortion.size}). '
                'Falling back to uncorrected pixels.')
            self.distortion_warning_emitted = True

    def read_depth_m(self, depth, u, v):
        """(u, v) 주변 윈도의 유효 depth 중앙값 [m]. 유효값 없으면 0."""
        h, w = depth.shape
        r = self.win_r
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = depth[v0:v1, u0:u1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid))

    def pixel_to_normalized(self, u, v):
        """Raw pixel -> normalized coordinate, optionally undoing D."""
        if self.apply_distortion_correction and self.distortion_supported:
            return undistort_plumb_bob_normalized(
                u, v, self.camera_matrix, self.distortion)
        return np.array([
            (float(u) - self.cx) / self.fx,
            (float(v) - self.cy) / self.fy,
        ])

    def deproject(self, u, v, d):
        """픽셀 (u, v) + depth d[m] -> 카메라 좌표계 3D 점."""
        x_norm, y_norm = self.pixel_to_normalized(u, v)
        return np.array([x_norm * d, y_norm * d, d])

    def decode_depth(self, msg: Image):
        """sensor_msgs/Image -> depth [m] 2D numpy 배열."""
        if msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            return depth.astype(np.float32) * 0.001  # mm -> m
        if msg.encoding == '32FC1':
            return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        self.get_logger().error(
            f'Unsupported depth encoding: {msg.encoding}',
            throttle_duration_sec=5.0)
        return None

    def make_transform(self, stamp, frame_id, child_frame_id, t, q):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])
        return tf_msg

    def make_pose(self, t, q):
        pose = Pose()
        pose.position.x = float(t[0])
        pose.position.y = float(t[1])
        pose.position.z = float(t[2])
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        return pose

    def transform_results_to_output_frame(self, results, source_frame, stamp):
        """카메라 기준 포즈를 output_frame 기준으로 변환한다."""
        target_frame = self.output_frame or source_frame
        if target_frame == source_frame:
            return results, source_frame

        try:
            frame_tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time.from_msg(stamp))
        except Exception:
            try:
                # 정확한 이미지 시각의 TF가 없으면 가장 최근 TF로 폴백한다.
                frame_tf = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, Time())
            except Exception as exc:
                self.get_logger().warn(
                    f'Output-frame TF lookup failed: {target_frame} <- '
                    f'{source_frame}: {exc}',
                    throttle_duration_sec=5.0)
                return None, target_frame

        tr = frame_tf.transform.translation
        rot = frame_tf.transform.rotation
        frame_t = np.array([tr.x, tr.y, tr.z])
        frame_q = np.array([rot.x, rot.y, rot.z, rot.w])
        frame_q_norm = np.linalg.norm(frame_q)
        if frame_q_norm <= 1e-12:
            self.get_logger().error(
                f'Invalid zero quaternion in TF: {target_frame} <- '
                f'{source_frame}',
                throttle_duration_sec=5.0)
            return None, target_frame
        frame_R = quaternion_to_rotation_matrix(frame_q / frame_q_norm)

        transformed = []
        for source, tag_id, t, q in results:
            output_t = frame_t + frame_R @ np.asarray(t)
            output_R = frame_R @ quaternion_to_rotation_matrix(q)
            output_q = rotation_matrix_to_quaternion(output_R)
            output_q = output_q / np.linalg.norm(output_q)
            transformed.append((source, tag_id, output_t, output_q))

        return transformed, target_frame

    # ---------------- 포즈 계산 ----------------

    def compute_depth_pose(self, det, depth, w, h):
        """코너 depth 기반 평면 추정 포즈. 실패하면 None."""
        pts_3d = []
        for corner in det.corners:
            cu, cv = int(corner.x), int(corner.y)
            if not (0 <= cu < w and 0 <= cv < h):
                return None
            d = self.read_depth_m(depth, cu, cv)
            if d <= 0:
                return None
            pts_3d.append(self.deproject(cu, cv, d))

        c_u, c_v = int(det.centre.x), int(det.centre.y)
        if not (0 <= c_u < w and 0 <= c_v < h):
            return None
        c_d = self.read_depth_m(depth, c_u, c_v)
        if c_d <= 0:
            return None

        p0, p1, _, p3 = (np.array(p) for p in pts_3d)

        v_x = p1 - p0
        n = np.linalg.norm(v_x)
        if n < 1e-6:
            return None
        v_x = v_x / n

        v_z = np.cross(v_x, p3 - p0)
        n = np.linalg.norm(v_z)
        if n < 1e-6:
            return None
        v_z = v_z / n

        v_y = np.cross(v_z, v_x)
        R = np.column_stack((v_x, v_y, v_z))

        if self.rot_offset is not None:
            R = R @ self.rot_offset

        t = self.deproject(c_u, c_v, c_d)
        return t, rotation_matrix_to_quaternion(R)

    def lookup_pnp_pose(self, tag_id, frame_id, stamp):
        """apriltag_ros가 송출한 PnP TF(<frame_id> -> tag36h11:<id>)를 조회. 실패하면 None."""
        tag_frame = self.pnp_frame_fmt.format(id=tag_id)
        try:
            tf = self.tf_buffer.lookup_transform(frame_id, tag_frame, Time.from_msg(stamp))
        except Exception:
            try:
                # 해당 시각의 TF가 아직/이미 없으면 가장 최근 값으로 폴백
                tf = self.tf_buffer.lookup_transform(frame_id, tag_frame, Time())
            except Exception as e:
                self.get_logger().warn(f'PnP TF lookup failed for {tag_frame}: {e}',
                                       throttle_duration_sec=5.0)
                return None

        tr = tf.transform.translation
        rot = tf.transform.rotation
        t = np.array([tr.x, tr.y, tr.z])
        q = np.array([rot.x, rot.y, rot.z, rot.w])

        if self.pnp_rot_offset is not None:
            q = rotation_matrix_to_quaternion(
                quaternion_to_rotation_matrix(q) @ self.pnp_rot_offset)
        return t, q

    def compute_distortion_corrected_pnp_pose(self, det):
        """Raw 검출 코너와 CameraInfo K/D로 태그 포즈를 계산한다."""
        try:
            normalized = np.array([
                self.pixel_to_normalized(corner.x, corner.y)
                for corner in det.corners
            ])
            t, R = estimate_square_pose_from_normalized(
                normalized, self.pnp_tag_size)
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.get_logger().warn(
                f'Distortion-corrected PnP failed for tag {det.id}: {exc}',
                throttle_duration_sec=5.0)
            return None

        if self.pnp_rot_offset is not None:
            R = R @ self.pnp_rot_offset
        return t, rotation_matrix_to_quaternion(R)

    # ---------------- 트리거 처리 ----------------

    def robust_average(self, entry):
        """
        Outlier를 제거하고 포즈를 평균한다.

        위치는 중앙값, 자세는 평균 쿼터니언 기준으로 먼 샘플을 제거한다.
        """
        ts = np.array(entry['t'])
        qs = [np.asarray(q) for q in entry['q']]
        n_total = len(ts)
        keep_n = max(1, int(math.ceil(n_total * (1.0 - self.outlier_trim_ratio))))

        # 위치: 성분별 중앙값에서의 거리 기준으로 가까운 keep_n개만 평균
        med = np.median(ts, axis=0)
        dist = np.linalg.norm(ts - med, axis=1)
        keep_idx = np.argsort(dist)[:keep_n]
        t_avg = ts[keep_idx].mean(axis=0)

        # 자세: 1차 평균 쿼터니언에서의 각도 거리 기준으로 가까운 keep_n개만 재평균
        q_ref = average_quaternions(qs)
        ang = np.array([2.0 * math.acos(min(1.0, abs(float(np.dot(q, q_ref))))) for q in qs])
        keep_idx_q = np.argsort(ang)[:keep_n]
        q_avg = average_quaternions([qs[i] for i in keep_idx_q])

        return t_avg, q_avg, keep_n, n_total

    def capture_callback(self, _msg):
        if not self.trigger_mode:
            self.get_logger().warn('capture ignored: trigger_mode is False (실시간 모드).')
            return
        self.samples = {}
        self.frames_left = self.samples_per_trigger
        self.collecting = True
        self.get_logger().info(f'capture started: collecting {self.samples_per_trigger} frames...')

    def finish_capture(self, stamp):
        """수집 종료: (방식, 태그)별 평균 포즈를 계산해 static TF + PoseArray로 1회 퍼블리시."""
        self.collecting = False

        pose_arrays = {}
        static_tfs = []
        for (source, tag_id), entry in sorted(self.samples.items()):
            t_avg, q_avg, n_used, n_total = self.robust_average(entry)
            child = f'{self.prefix_by_source[source]}{tag_id}'
            static_tfs.append(self.make_transform(stamp, self.last_frame_id, child, t_avg, q_avg))

            if source not in pose_arrays:
                pa = PoseArray()
                pa.header.stamp = stamp
                pa.header.frame_id = self.last_frame_id
                pose_arrays[source] = pa
            pose_arrays[source].poses.append(self.make_pose(t_avg, q_avg))

            roll, pitch, yaw = quaternion_to_euler_deg(*q_avg)
            self.get_logger().info(
                f'[{source.upper():5s} ID: {tag_id}] {n_used}/{n_total} samples avg '
                f'(trim {self.outlier_trim_ratio:.0%}) | '
                f'X:{t_avg[0]:.4f}m Y:{t_avg[1]:.4f}m Z:{t_avg[2]:.4f}m | '
                f'Roll:{roll:6.1f} Pitch:{pitch:6.1f} Yaw:{yaw:6.1f} deg')

        if static_tfs:
            # 평균 결과는 static TF로 송출 -> 다음 트리거 전까지 RViz에 계속 표시됨
            self.static_tf_broadcaster.sendTransform(static_tfs)
            for source, pa in pose_arrays.items():
                self.pose_pubs[source].publish(pa)
            self.get_logger().info(
                'capture finished: averaged poses published '
                '(static TF + PoseArray).')
        else:
            self.get_logger().warn('capture finished but no valid samples collected.')
        self.samples = {}

    # ---------------- 메인 콜백 ----------------

    def detections_callback(self, det_msg: AprilTagDetectionArray, depth_msg: Image):
        # 트리거 모드에서는 수집 중일 때만 프레임을 처리
        if self.trigger_mode and not self.collecting:
            return
        if self.fx is None:
            self.get_logger().warn('camera_info not received yet.', throttle_duration_sec=5.0)
            return
        if not det_msg.detections:
            return

        depth = self.decode_depth(depth_msg)
        if depth is None:
            return
        h, w = depth.shape
        source_frame = depth_msg.header.frame_id

        results = []  # (source, tag_id, t(xyz), q(x, y, z, w))
        for det in det_msg.detections:
            depth_pose = self.compute_depth_pose(det, depth, w, h)
            if depth_pose is not None:
                results.append(('depth', det.id, depth_pose[0], depth_pose[1]))

            if self.pnp_enable:
                if (self.apply_distortion_correction and
                        self.distortion_supported):
                    pnp_pose = self.compute_distortion_corrected_pnp_pose(det)
                else:
                    pnp_pose = self.lookup_pnp_pose(
                        det.id, depth_msg.header.frame_id,
                        depth_msg.header.stamp)
                if pnp_pose is not None:
                    results.append(('pnp', det.id, pnp_pose[0], pnp_pose[1]))

        if not results:
            return

        results, result_frame = self.transform_results_to_output_frame(
            results, source_frame, depth_msg.header.stamp)
        if results is None:
            return
        self.last_frame_id = result_frame

        # 트리거 모드: 이번 프레임 결과를 누적, 목표 프레임 수 도달 시 평균 1회 출력
        if self.trigger_mode:
            for source, tag_id, t, q in results:
                entry = self.samples.setdefault((source, tag_id), {'t': [], 'q': []})
                entry['t'].append(t)
                entry['q'].append(np.asarray(q))
            self.frames_left -= 1
            if self.frames_left > 0 and self.frames_left % 25 == 0:
                self.get_logger().info(f'collecting... {self.frames_left} frames left')
            if self.frames_left <= 0:
                self.finish_capture(depth_msg.header.stamp)
            return

        # 실시간 모드: 매 프레임 TF + PoseArray 퍼블리시
        pose_arrays = {}
        for source, tag_id, t, q in results:
            child = f'{self.prefix_by_source[source]}{tag_id}'
            self.tf_broadcaster.sendTransform(
                self.make_transform(
                    depth_msg.header.stamp, result_frame, child, t, q))
            if source not in pose_arrays:
                pa = PoseArray()
                pa.header.stamp = depth_msg.header.stamp
                pa.header.frame_id = result_frame
                pose_arrays[source] = pa
            pose_arrays[source].poses.append(self.make_pose(t, q))
        for source, pa in pose_arrays.items():
            self.pose_pubs[source].publish(pa)


def main(args=None):
    rclpy.init(args=args)
    node = DepthTagTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
