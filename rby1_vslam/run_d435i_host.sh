#!/usr/bin/env bash
# Publish D435i inputs on the host for a host-networked Isaac ROS container.
set -Eeo pipefail

ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -r "$SCRIPT_DIR/config/d435i_host.yaml" ]]; then
  CAMERA_CONFIG_DIR="$SCRIPT_DIR/config"
else
  CAMERA_CONFIG_DIR="$SCRIPT_DIR/../config"
fi
usage() {
  printf '%s\n' \
    "Usage: $0 [0|1] [--nvblox] [--color] [--camera-fps 30|60|90] [--rsusb-library PATH] [additional rs_launch.py arguments]" \
    "Defaults: stereo IR 640x480@90, auto exposure, projector off." \
    "--camera-fps sets the stereo IR rate (30, 60 or 90); nvblox accepts 60 or 90 only." \
    "nvblox: native IR + depth 640x480@90, alternating projector (45 Hz each after splitting)." \
    "--color adds native RGB8 color 640x480@30 for colored nvblox reconstruction." \
    "Color does not align/resample depth or alter the IR/depth acquisition rate." \
    "Fallback: $0 0 --nvblox --camera-fps 60 (30 Hz each after splitting)." \
    "nvblox uses the existing local librealsense RSUSB build for emitter metadata." \
    "Override its path with --rsusb-library PATH or RBY1_REALSENSE_LIBRARY." \
    "Example fixed exposure at 60 Hz (values must suit the scene):" \
    "  $0 0 depth_module.infra_profile:=640,480,60 depth_module.enable_auto_exposure:=false depth_module.exposure:=8000 depth_module.gain:=16" \
    "Additional arguments override launch defaults. Keys in RBY1_D435_HOST_CONFIG take precedence over launch arguments." \
    "Start the container with use_realsense_splitter:=true when using --nvblox."
}

MODE=0
if (($#)) && [[ "$1" != -* && "$1" != *:=* ]]; then
  MODE="$1"
  shift
fi
NVBLOX=false
ENABLE_COLOR=false
CAMERA_FPS=90
REALSENSE_LIBRARY="${RBY1_REALSENSE_LIBRARY:-}"
LAUNCH_ARGUMENTS=()
while (($#)); do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --nvblox) NVBLOX=true; shift ;;
    --color) ENABLE_COLOR=true; shift ;;
    --camera-fps)
      if (($# < 2)); then
        printf '%s\n' '--camera-fps requires 30, 60 or 90' >&2
        exit 2
      fi
      CAMERA_FPS="$2"
      shift 2
      ;;
    --camera-fps=*) CAMERA_FPS="${1#*=}"; shift ;;
    --rsusb-library)
      if (($# < 2)); then
        printf '%s\n' '--rsusb-library requires a readable shared-library path' >&2
        exit 2
      fi
      REALSENSE_LIBRARY="$2"
      shift 2
      ;;
    --rsusb-library=*) REALSENSE_LIBRARY="${1#*=}"; shift ;;
    --) shift; LAUNCH_ARGUMENTS+=("$@"); break ;;
    -*) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    *) LAUNCH_ARGUMENTS+=("$1"); shift ;;
  esac
done
if [[ "$NVBLOX" == true ]]; then
  if [[ "$CAMERA_FPS" != 60 && "$CAMERA_FPS" != 90 ]]; then
    printf '%s\n' '--camera-fps must be 60 or 90 with --nvblox' >&2
    exit 2
  fi
elif [[ "$CAMERA_FPS" != 30 && "$CAMERA_FPS" != 60 && "$CAMERA_FPS" != 90 ]]; then
  printf '%s\n' '--camera-fps must be 30, 60 or 90' >&2
  exit 2
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
    usage >&2
    exit 2
    ;;
esac

ENABLE_DEPTH=false
INFRA_PROFILE="640,480,$CAMERA_FPS"
DEPTH_ARGUMENTS=()
COLOR_ARGUMENTS=()
if [[ "$ENABLE_COLOR" == true ]]; then
  COLOR_ARGUMENTS=("rgb_camera.color_profile:=640,480,30" "rgb_camera.color_format:=RGB8")
fi
LAUNCH_ENV=()
DEFAULT_CAMERA_CONFIG="$CAMERA_CONFIG_DIR/d435i_host.yaml"
if [[ "$NVBLOX" == true ]]; then
  ENABLE_DEPTH=true
  INFRA_PROFILE="640,480,$CAMERA_FPS"
  DEPTH_ARGUMENTS=("depth_module.depth_profile:=$INFRA_PROFILE")
  DEFAULT_CAMERA_CONFIG="$CAMERA_CONFIG_DIR/d435i_host_nvblox.yaml"
  REALSENSE_LIBRARY="${REALSENSE_LIBRARY:-$HOME/librealsense-rsusb-2.58.4/build-rsusb/Release/librealsense2.so.2.58.4}"
  if [[ ! -f "$REALSENSE_LIBRARY" || ! -r "$REALSENSE_LIBRARY" ]]; then
    printf 'Missing RSUSB library: %s\n' "$REALSENSE_LIBRARY" >&2
    printf '%s\n' 'Set RBY1_REALSENSE_LIBRARY or --rsusb-library to an existing compatible RSUSB librealsense build.' >&2
    printf '%s\n' 'The system backend on this Jetson does not publish the emitter metadata required by the splitter.' >&2
    exit 1
  fi
  # Keep this override local to the launched camera and its descendants.
  # ROS setup can prepend system library paths, so preload the exact RSUSB build.
  LAUNCH_ENV=("LD_PRELOAD=$REALSENSE_LIBRARY${LD_PRELOAD:+:$LD_PRELOAD}")
  # The default 512 KiB Fast DDS segment is smaller than one native depth
  # image. Reserve space for in-flight IR/depth frames without changing QoS.
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$CAMERA_CONFIG_DIR/fastdds_camera.xml}"
  [[ -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]] || {
    printf 'Missing Fast DDS profile: %s\n' "$FASTRTPS_DEFAULT_PROFILES_FILE" >&2
    exit 1
  }
fi
CAMERA_CONFIG="${RBY1_D435_HOST_CONFIG:-$DEFAULT_CAMERA_CONFIG}"

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
if [[ "$NVBLOX" == true ]]; then
  printf '[d435i-host] emitter alternates in firmware, IR/depth=%s Hz, split outputs=%s Hz each\n' \
    "$CAMERA_FPS" "$((CAMERA_FPS / 2))"
  printf '[d435i-host] Requires frame_emitter_mode in IR/depth metadata; launch container splitter.\n'
  printf '[d435i-host] process-local RSUSB library: %s\n' "$REALSENSE_LIBRARY"
  printf '[d435i-host] process-local Fast DDS profile: %s\n' "$FASTRTPS_DEFAULT_PROFILES_FILE"
else
  printf '[d435i-host] IR emitter=off, stereo IR 640x480@%s\n' "$CAMERA_FPS"
fi
if [[ "$ENABLE_COLOR" == true ]]; then
  printf '[d435i-host] native RGB8 color 640x480@30 enabled; depth remains unaligned\n'
fi
printf '[d435i-host] enable_sync=false, inter_cam_sync_mode=0, auto exposure, IMU=%s\n' "$ENABLE_IMU"
printf '[d435i-host] config: %s (file keys override launch arguments)\n' "$CAMERA_CONFIG"
if ((${#LAUNCH_ARGUMENTS[@]})); then
  printf '[d435i-host] launch overrides:'
  printf ' %q' "${LAUNCH_ARGUMENTS[@]}"
  printf '\n'
fi

exec env "${LAUNCH_ENV[@]}" ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=d435 \
  camera_name:=d435 \
  device_type:=d435 \
  "${SERIAL_ARGUMENT[@]}" \
  config_file:="$CAMERA_CONFIG" \
  enable_color:="$ENABLE_COLOR" \
  "${COLOR_ARGUMENTS[@]}" \
  enable_depth:="$ENABLE_DEPTH" \
  enable_infra1:=true \
  enable_infra2:=true \
  depth_module.infra_profile:="$INFRA_PROFILE" \
  "${DEPTH_ARGUMENTS[@]}" \
  depth_module.enable_auto_exposure:=true \
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
  tf_publish_rate:=0.0 \
  "${LAUNCH_ARGUMENTS[@]}"
