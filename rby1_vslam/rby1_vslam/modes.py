"""D435i acquisition modes and Isaac ROS 3.2 tracking configuration."""

import re

MODE_LABELS = {
    0: 'stereo',
    1: 'stereo_imu',
    2: 'rgbd',
}

STEREO_TOPICS = {
    'left_image_topic': '/d435/d435/infra1/image_rect_raw',
    'right_image_topic': '/d435/d435/infra2/image_rect_raw',
    'left_info_topic': '/d435/d435/infra1/camera_info',
    'right_info_topic': '/d435/d435/infra2/camera_info',
}

RGBD_TOPICS = {
    'left_image_topic': '/d435/d435/color/image_raw',
    'right_image_topic': '/d435/d435/aligned_depth_to_color/image_raw',
    'left_info_topic': '/d435/d435/color/camera_info',
    'right_info_topic': '/d435/d435/aligned_depth_to_color/camera_info',
}


def parse_tracking_mode(value):
    """Return a validated cuVSLAM tracking-mode integer."""
    if isinstance(value, bool):
        raise ValueError('tracking_mode must be 0, 1, or 2')
    try:
        mode = int(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise ValueError('tracking_mode must be 0, 1, or 2') from exc
    if mode not in MODE_LABELS:
        raise ValueError('tracking_mode must be 0 (stereo), 1 (stereo+IMU), or 2 (RGB-D)')
    return mode


def mode_label(value):
    return MODE_LABELS[parse_tracking_mode(value)]


def uses_imu(value):
    return parse_tracking_mode(value) == 1


def uses_rgbd(value):
    return parse_tracking_mode(value) == 2


def camera_topics(value):
    topics = RGBD_TOPICS if uses_rgbd(value) else STEREO_TOPICS
    return dict(topics)


def camera_profile(value):
    parts = re.split('[,xX]', str(value).strip())
    if len(parts) != 3 or any(not part.strip().isdigit() or int(part) <= 0 for part in parts):
        raise ValueError('camera profile must be positive WIDTH,HEIGHT,FPS')
    return ','.join(str(int(part)) for part in parts)


def depth_module_profile(tracking_mode, all_sensors, infra_profile, depth_profile):
    infra, depth = camera_profile(infra_profile), camera_profile(depth_profile)
    if all_sensors and infra != depth:
        raise ValueError('RealSense 4.51.1 shares depth_module.profile across IR/depth; '
                         'publish_all_sensors requires identical infra_profile and depth_profile')
    return depth if uses_rgbd(tracking_mode) or all_sensors else infra


def isaac_parameters(value, mode='mapping'):
    """Use the 3.2 API; RGB-D remains available for dataset acquisition only."""
    tracking_mode = parse_tracking_mode(value)
    if uses_rgbd(tracking_mode):
        raise ValueError('Isaac ROS 3.2 supports stereo (0) and stereo+IMU (1); '
                         'tracking_mode=2 is only supported for camera recording')
    if mode not in ('mapping', 'localization', 'odometry'):
        raise ValueError('mode must be mapping, localization or odometry')
    return {
        'num_cameras': 2,
        'min_num_images': 2,
        'enable_imu_fusion': uses_imu(tracking_mode),
        'rectified_images': True,
        'base_frame': 'd435_link',
        'camera_optical_frames': ['d435_infra1_optical_frame', 'd435_infra2_optical_frame'],
        'map_frame': 'vslam_map',
        'odom_frame': 'vslam_odom',
        # d435_link already has a parent in the robot's moving-head TF tree.
        'publish_odom_to_base_tf': False,
        'publish_map_to_odom_tf': False,
        'override_publishing_stamp': False,
        'enable_localization_n_mapping': mode != 'odometry',
        'localize_on_startup': False,
    }
