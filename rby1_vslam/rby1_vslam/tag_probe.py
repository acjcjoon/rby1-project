"""On-demand AprilTag capture from the head D435 IR stream for station ground truth.

Subscribes only while a capture runs (no permanent CPU load next to VSLAM), keeps
up to `max_frames` detections over `duration` seconds, then returns averaged
corners, intrinsics and T_base_cam (TF at the end of the capture).
"""

import math
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from . import station_tag as st

NO_IMAGE_TIMEOUT_SEC = 3.0


def _transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class TagProbe:
    def __init__(self, node, image_topic, info_topic, base_frame):
        self.node = node
        self.image_topic, self.info_topic, self.base_frame = image_topic, info_topic, base_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.job = None

    @property
    def busy(self):
        return self.job is not None

    def capture(self, callback, duration=2.0, max_frames=40):
        """Start a capture; callback(result_dict or None, error_str or None) from tick()."""
        if self.job is not None:
            callback(None, 'a tag capture is already running')
            return
        self.job = {
            'callback': callback, 'duration': duration, 'interval': duration / max(1, max_frames),
            'started': time.monotonic(), 'first': None, 'last': -math.inf,
            'frames': [], 'seen': 0, 'info': None, 'error': None,
        }
        self.job['subs'] = [
            self.node.create_subscription(Image, self.image_topic, self._on_image,
                                          qos_profile_sensor_data),
            self.node.create_subscription(CameraInfo, self.info_topic, self._on_info,
                                          qos_profile_sensor_data),
        ]

    def _on_info(self, msg):
        if self.job is not None and self.job['info'] is None:
            self.job['info'] = msg

    def _on_image(self, msg):
        job = self.job
        if job is None or job['error']:
            return
        now = time.monotonic()
        if job['first'] is None:
            job['first'] = now
        if now - job['last'] < job['interval'] or now - job['first'] > job['duration']:
            return
        job['last'] = now
        try:
            if msg.encoding not in ('mono8', '8UC1'):
                raise ValueError(f'unsupported IR encoding {msg.encoding}')
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
            job['frames'].append(st.detect(img))
            job['seen'] += 1
            job['frame_id'] = msg.header.frame_id
        except Exception as exc:  # cv2 unusable, bad image, ...
            job['error'] = f'tag detection failed: {exc}'

    def tick(self):
        job = self.job
        if job is None:
            return
        now = time.monotonic()
        error = job['error']
        if error is None and job['first'] is None and now - job['started'] > NO_IMAGE_TIMEOUT_SEC:
            error = f'no IR image on {self.image_topic}'
        done = error is not None or (job['first'] is not None
                                     and now - job['first'] > job['duration'] + 0.2)
        if not done:
            return
        for sub in job['subs']:
            self.node.destroy_subscription(sub)
        self.job = None
        result = None
        if error is None:
            try:
                result = self._finish(job)
            except Exception as exc:
                error = str(exc)
        job['callback'](result, error)

    def _finish(self, job):
        info = job['info']
        if info is None:
            raise ValueError(f'no camera_info on {self.info_topic}')
        corners = st.mean_corners(job['frames'])
        if not corners:
            raise ValueError(f'no tag36h11 tag detected in {job["seen"]} IR frames')
        frame = job.get('frame_id') or info.header.frame_id
        tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        return {
            'corners': {i: c for i, (c, sd, n) in corners.items()},
            'corner_sd_px': {i: sd for i, (c, sd, n) in corners.items()},
            'detections': {i: n for i, (c, sd, n) in corners.items()},
            'frames': job['seen'],
            'K': np.array(info.k, float).reshape(3, 3),
            'D': np.array(info.d, float) if len(info.d) else np.zeros(5),
            'frame': frame,
            'T_base_cam': _transform_matrix(tf.transform),
        }
