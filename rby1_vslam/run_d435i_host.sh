#!/usr/bin/env bash
# Publish D435i stereo input on the host for a host-networked Isaac ROS container.
set -Eeo pipefail

ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -r "$SCRIPT_DIR/config/d435i_host.yaml" ]]; then
  DEFAULT_CAMERA_CONFIG="$SCRIPT_DIR/config/d435i_host.yaml"
else
  DEFAULT_CAMERA_CONFIG="$SCRIPT_DIR/../config/d435i_host.yaml"
fi
CAMERA_CONFIG="${RBY1_D435_HOST_CONFIG:-$DEFAULT_CAMERA_CONFIG}"
MODE="${1:-0}"
if (($#)); then
  shift
fi

case "${MODE,,}" in
  0|mode0|stereo)
    ENABLE_IMU=false
    UNITE_IMU=0
    MODE=0
    ;;
  1|mode1|stereo_imu|stereo-imu|vio)
    ENABLE_IMU=true
    UNITE_IMU=2
    MODE=1
    ;;
  *)
    printf 'Usage: %s [0|1] [additional rs_launch.py arguments]\n' "$0" >&2
    exit 2
    ;;
esac

[[ -r "$ROS_SETUP" ]] || {
  printf 'Missing ROS setup: %s\n' "$ROS_SETUP" >&2
  exit 1
}
[[ -r "$CAMERA_CONFIG" ]] || {
  printf 'Missing D435i host config: %s\n' "$CAMERA_CONFIG" >&2
  exit 1
}
source "$ROS_SETUP"
set -u

export ROS_DOMAIN_ID="${RBY1_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-10}}"
SERIAL="${RBY1_D435_SERIAL:-}"
SERIAL_ARGUMENT=()
if [[ -n "$SERIAL" ]]; then
  SERIAL="${SERIAL#_}"
  [[ "$SERIAL" =~ ^[0-9]+$ ]] || {
    printf 'RBY1_D435_SERIAL must contain digits only\n' >&2
    exit 2
  }
  SERIAL_ARGUMENT=("serial_no:=_${SERIAL}")
fi

ros2 pkg prefix realsense2_camera >/dev/null 2>&1 || {
  printf 'Host package realsense2_camera is not installed\n' >&2
  exit 1
}

printf '[d435i-host] mode=%s ROS_DOMAIN_ID=%s\n' "$MODE" "$ROS_DOMAIN_ID"
printf '[d435i-host] enable_sync=false, inter_cam_sync_mode=0, IR emitter=off\n'
printf '[d435i-host] stereo IR 640x480@30, IMU=%s\n' "$ENABLE_IMU"

exec ros2 launch realsense2_camera rs_launch.py \
  "$@" \
  camera_namespace:=d435 \
  camera_name:=d435 \
  device_type:=d435 \
  "${SERIAL_ARGUMENT[@]}" \
  config_file:="$CAMERA_CONFIG" \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  depth_module.infra_profile:=640,480,30 \
  depth_module.inter_cam_sync_mode:=0 \
  enable_sync:=false \
  align_depth.enable:=false \
  pointcloud.enable:=false \
  enable_gyro:="$ENABLE_IMU" \
  enable_accel:="$ENABLE_IMU" \
  gyro_fps:=200 \
  accel_fps:=100 \
  unite_imu_method:="$UNITE_IMU" \
  publish_tf:=true \
  tf_publish_rate:=0.0
