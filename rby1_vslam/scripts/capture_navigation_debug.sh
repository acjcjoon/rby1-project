#!/usr/bin/env bash
# Read-only ROS diagnostics: record status/poses/commands, never issue motion.
set -Eeo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: bash capture_navigation_debug.sh [output_directory]"
  echo "Source the UPC ROS workspace and use the same ROS_DOMAIN_ID as the robot."
  echo "Start before a waypoint trial; Ctrl+C stops recording."
  exit 0
fi
[[ $# -le 1 ]] || { echo "Expected at most one output directory" >&2; exit 1; }
command -v ros2 >/dev/null || { echo "Source ROS and the UPC workspace first" >&2; exit 1; }
command -v timeout >/dev/null || { echo "GNU timeout is required" >&2; exit 1; }

OUTPUT_ROOT="${1:-${HOME}/rby1_debug}"
mkdir -p -- "$OUTPUT_ROOT"
RUN_DIR="$(mktemp -d "${OUTPUT_ROOT}/vslam_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

{
  date -u '+captured_at_utc=%Y-%m-%dT%H:%M:%SZ'
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-0}"
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-default}"
  timeout 5 ros2 node list || true
} > "$RUN_DIR/environment.txt" 2>&1

for node in \
  /rby1/vslam/waypoint_ui \
  /rby1/vslam/nav2_gate \
  /rby1/vslam/localization_tf \
  /rby1/vslam/pose_adapter \
  /rby1/vslam/upc_bridge \
  /rby1/vslam/nav2/controller_server \
  /rby1/vslam/nav2/velocity_smoother \
  /rby1/rby1_control; do
  label="${node//\//_}"
  timeout 5 ros2 param dump "$node" > "$RUN_DIR/${label}_params.yaml" 2>&1 || true
done
timeout 5 ros2 topic info /rby1/cmd_raw --verbose \
  > "$RUN_DIR/cmd_raw_publishers.txt" 2>&1 || true
timeout 5 ros2 topic list -t --include-hidden-topics \
  > "$RUN_DIR/topics.txt" 2>&1 || true
if [[ -f "$SCRIPT_DIR/../data/rby1_vslam_waypoints.yaml" ]]; then
  cp -- "$SCRIPT_DIR/../data/rby1_vslam_waypoints.yaml" "$RUN_DIR/source_waypoints.yaml"
fi

echo "[debug] Snapshots: $RUN_DIR"
echo "[debug] Starting rosbag. Wait for recorder subscriptions before the trial."
echo "[debug] This process only records; Ctrl+C stops the recorder."
exec ros2 bag record --include-hidden-topics -o "$RUN_DIR/bag" \
  /rosout /tf /tf_static \
  /rby1/odom /rby1/robot_state \
  /rby1/control/command /rby1/control/event \
  /rby1/vslam/bridge_status \
  /rby1/vslam/localization_status \
  /rby1/vslam/navigation_status \
  /rby1/vslam/camera_odometry \
  /rby1/vslam/camera_slam_odometry \
  /rby1/vslam/slam_odom \
  /rby1/vslam/nav2_cmd_vel /rby1/cmd_raw /rby1/cmd_vel \
  /rby1/vslam/nav2/navigate_to_pose/_action/status \
  /rby1/vslam/nav2/navigate_to_pose/_action/feedback
