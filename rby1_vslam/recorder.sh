#!/usr/bin/env bash
# Record the selected camera dataset from a matching local camera launch.
set -Eeo pipefail

ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${RBY1_WORKSPACE_SETUP:-${ISAAC_ROS_WS:-${HOME}/rby1_ros2_ws}/install/setup.bash}"
OUTPUT_ROOT="${RBY1_BAG_DIR:-${ISAAC_ROS_WS:-${HOME}/rby1_ros2_ws}/bags}"

usage() {
  printf '%s\n' \
    "Usage: $0 MODE [OUTPUT_BAG]" \
    "" \
    "Start the matching camera mode first; MODE selects what is saved:" \
    "  mode0  stereo IR1/IR2 + RBY1 base" \
    "  mode1  stereo IR1/IR2 + raw/united IMU + RBY1 base" \
    "  mode2  color + aligned depth + RBY1 base" \
    "" \
    "Example:" \
    "  bash rby1_vslam/run_d435i_host.sh 1" \
    "  bash rby1_vslam/recorder.sh mode1" \
    "  bash rby1_vslam/recorder.sh mode2" \
    "" \
    "Default output: RBY1_BAG_DIR, else ISAAC_ROS_WS/bags, else ~/rby1_ros2_ws/bags" \
    "ROS domain:     RBY1_ROS_DOMAIN_ID, else current ROS_DOMAIN_ID, else 0"
}

die() {
  printf '[recorder] ERROR: %s\n' "$*" >&2
  exit 1
}

if [[ "${1:-}" == '--help' || "${1:-}" == '-h' ]]; then
  usage
  exit 0
fi
(( $# >= 1 && $# <= 2 )) || {
  usage >&2
  die 'Expected MODE and an optional output bag path'
}

case "${1,,}" in
  0|mode0|stereo)
    MODE='mode0'
    SENSOR_DESCRIPTION='stereo IR'
    REQUIRED_TOPICS=(
      /d435/d435/infra1/image_rect_raw
      /d435/d435/infra1/camera_info
      /d435/d435/infra2/image_rect_raw
      /d435/d435/infra2/camera_info
      /rby1/odom
    )
    SENSOR_TOPICS=("${REQUIRED_TOPICS[@]:0:4}")
    ;;
  1|mode1|stereo_imu|stereo-imu|vio)
    MODE='mode1'
    SENSOR_DESCRIPTION='stereo IR + IMU'
    REQUIRED_TOPICS=(
      /d435/d435/infra1/image_rect_raw
      /d435/d435/infra1/camera_info
      /d435/d435/infra2/image_rect_raw
      /d435/d435/infra2/camera_info
      /d435/d435/imu
      /rby1/odom
    )
    SENSOR_TOPICS=(
      "${REQUIRED_TOPICS[@]:0:5}"
      /d435/d435/accel/sample
      /d435/d435/gyro/sample
      /d435/d435/accel/imu_info
      /d435/d435/gyro/imu_info
      /d435/d435/accel/metadata
      /d435/d435/gyro/metadata
      /d435/d435/infra1/metadata
      /d435/d435/infra2/metadata
      /d435/d435/extrinsics/depth_to_infra1
      /d435/d435/extrinsics/depth_to_infra2
      /d435/d435/extrinsics/depth_to_accel
      /d435/d435/extrinsics/depth_to_gyro
    )
    ;;
  2|mode2|rgbd|rgb-d)
    MODE='mode2'
    SENSOR_DESCRIPTION='color + aligned depth'
    REQUIRED_TOPICS=(
      /d435/d435/color/image_raw
      /d435/d435/color/camera_info
      /d435/d435/aligned_depth_to_color/image_raw
      /d435/d435/aligned_depth_to_color/camera_info
      /rby1/odom
    )
    SENSOR_TOPICS=(
      "${REQUIRED_TOPICS[@]:0:4}"
      /d435/d435/color/metadata
      /d435/d435/depth/metadata
      /d435/d435/extrinsics/depth_to_color
    )
    ;;
  *)
    usage >&2
    die "Unknown mode: $1 (use mode0, mode1 or mode2)"
    ;;
esac

COMMON_TOPICS=(
  /rby1/odom
  /rby1/joint_states
  /rby1/joint_states/torso
  /rby1/joint_states/head
  /rby1/joint_states/left_arm
  /rby1/joint_states/right_arm
  /rby1/robot_state
  /rby1/battery_state
  /rby1/tool_flange/left
  /rby1/tool_flange/right
  /rby1/cmd_vel
  /rby1/cmd_raw
  /rby1/control/command
  /rby1/control/state
  /rby1/control/event
  /rby1/control/response
  /tf
  /tf_static
  /diagnostics
  /parameter_events
)
RECORD_TOPICS=("${SENSOR_TOPICS[@]}" "${COMMON_TOPICS[@]}")

OUTPUT_BAG="${2:-${OUTPUT_ROOT}/rby1_${MODE}_$(date +%Y%m%d_%H%M%S)}"
OUTPUT_PARENT="$(dirname -- "$OUTPUT_BAG")"
[[ ! -e "$OUTPUT_BAG" ]] || die "Output already exists: $OUTPUT_BAG"
mkdir -p -- "$OUTPUT_PARENT"

[[ -r "$ROS_SETUP" ]] || die "Missing ROS setup: $ROS_SETUP"
[[ -r "$WORKSPACE_SETUP" ]] || die "Missing workspace setup: $WORKSPACE_SETUP"
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u
export ROS_DOMAIN_ID="${RBY1_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
export ROS2CLI_NO_DAEMON=1

command -v ros2 >/dev/null 2>&1 || die 'ros2 command is unavailable after sourcing setup files'
ros2 bag record --help >/dev/null 2>&1 || \
  die 'rosbag2 record is unavailable; install ros-humble-ros2bag'

printf '[recorder] Mode: %s (%s)\n' "$MODE" "$SENSOR_DESCRIPTION"
printf '[recorder] ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
printf '[recorder] Checking required topics once...\n'
TOPIC_LIST="$(ros2 topic list 2>/dev/null)" || \
  die 'Could not query the ROS graph'
MISSING_TOPICS=()
for topic in "${REQUIRED_TOPICS[@]}"; do
  if ! grep -Fxq -- "$topic" <<<"$TOPIC_LIST"; then
    MISSING_TOPICS+=("$topic")
  fi
done
if ((${#MISSING_TOPICS[@]} > 0)); then
  printf '[recorder] Missing %s required topic(s):\n' "${#MISSING_TOPICS[@]}" >&2
  printf '  %s\n' "${MISSING_TOPICS[@]}" >&2
  die 'Start run_mobile_base.sh with start_camera:=false and the matching run_d435i_host.sh mode'
fi

printf '[recorder] Required %s topics are visible.\n' "$MODE"
printf '[recorder] Recording only the selected sensor set and RBY1 base topics.\n'
printf '[recorder] Output: %s\n' "$OUTPUT_BAG"
printf '[recorder] Stop safely with Ctrl-C.\n'
exec ros2 bag record \
  --storage sqlite3 \
  --output "$OUTPUT_BAG" \
  "${RECORD_TOPICS[@]}"
