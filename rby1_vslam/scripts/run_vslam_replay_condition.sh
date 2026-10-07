#!/usr/bin/env bash
# Replay one controlled stereo/IMU condition through the real UPC->LAB bridge.
set -Eeo pipefail

BAG=''; CONDITION=''; LAB_HOST=''; DOMAIN='10'
usage() {
  cat <<'EOF'
Usage: run_vslam_replay_condition.sh --bag DIR --condition NAME --lab-host HOST [options]

Conditions:
  baseline       recorded stereo + recorded IMU
  no_imu         recorded stereo, IMU disabled
  fixed_stereo   recorded stereo through two turns, then fixed stereo + recorded IMU
  fixed_stereo_no_imu
                 recorded stereo through two turns, then fixed stereo; IMU disabled

Options:
  --domain ID   UPC ROS_DOMAIN_ID (default: 10)

Stop the live D435 publisher first. Start a fresh LAB cuVSLAM process with
enable_imu=true for baseline/fixed_stereo and false for both no-IMU conditions.
The fixed-stereo conditions require the returned_stationary marker in the bag.
EOF
}
die() { echo "[sensor-replay] ERROR: $*" >&2; exit 2; }
while (($#)); do
  case "$1" in
    --bag) [[ $# -ge 2 ]] || die 'missing --bag value'; BAG="$2"; shift 2 ;;
    --condition) [[ $# -ge 2 ]] || die 'missing --condition value'; CONDITION="$2"; shift 2 ;;
    --lab-host) [[ $# -ge 2 ]] || die 'missing --lab-host value'; LAB_HOST="$2"; shift 2 ;;
    --domain) [[ $# -ge 2 ]] || die 'missing --domain value'; DOMAIN="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
[[ -n "$BAG" && -d "$BAG" ]] || die '--bag must point to a rosbag directory'
[[ -n "$LAB_HOST" ]] || die '--lab-host is required'
[[ "$DOMAIN" =~ ^[0-9]+$ ]] || die '--domain must be numeric'

case "$CONDITION" in
  baseline)            STEREO_MODE='replay';              IMU_MODE='replay'; ENABLE_IMU='true' ;;
  no_imu)              STEREO_MODE='replay';              IMU_MODE='none';   ENABLE_IMU='false' ;;
  fixed_stereo)        STEREO_MODE='freeze_after_marker'; IMU_MODE='replay'; ENABLE_IMU='true' ;;
  fixed_stereo_no_imu) STEREO_MODE='freeze_after_marker'; IMU_MODE='none';   ENABLE_IMU='false' ;;
  *) die 'condition must be baseline, no_imu, fixed_stereo, or fixed_stereo_no_imu' ;;
esac

ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${RBY1_WORKSPACE_SETUP:-${HOME}/rby1_ros2_ws/install/setup.bash}"
[[ -r "$ROS_SETUP" && -r "$WORKSPACE_SETUP" ]] || die 'ROS/workspace setup missing'
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
export ROS_DOMAIN_ID="$DOMAIN"
ros2 pkg prefix rby1_vslam >/dev/null 2>&1 || die 'rby1_vslam is not installed; rebuild the workspace'
bag_info="$(ros2 bag info "$BAG" 2>&1)" || die "cannot inspect rosbag: $bag_info"
grep -Fq '/rby1/vslam/replay_marker' <<<"$bag_info" || \
  die 'source bag has no /rby1/vslam/replay_marker topic; capture a new source bag'

publisher_count() {
  local output
  output="$(timeout 3 ros2 topic info "$1" 2>/dev/null || true)"
  awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}' <<<"$output" | tail -n1
}
for topic in \
  /d435/d435/infra1/image_rect_raw /d435/d435/infra2/image_rect_raw \
  /d435/d435/infra1/camera_info /d435/d435/infra2/camera_info \
  /d435/d435/imu /rby1/vslam/bridge_status; do
  count="$(publisher_count "$topic")"
  [[ "$count" =~ ^[0-9]+$ ]] || count=0
  ((count == 0)) || die "live publisher still exists on $topic; stop the D435 first"
done

REMAPS=(
  /d435/d435/infra1/image_rect_raw:=/rby1/vslam/replay_input/d435/d435/infra1/image_rect_raw
  /d435/d435/infra2/image_rect_raw:=/rby1/vslam/replay_input/d435/d435/infra2/image_rect_raw
  /d435/d435/infra1/camera_info:=/rby1/vslam/replay_input/d435/d435/infra1/camera_info
  /d435/d435/infra2/camera_info:=/rby1/vslam/replay_input/d435/d435/infra2/camera_info
  /d435/d435/imu:=/rby1/vslam/replay_input/d435/d435/imu
  /rby1/vslam/replay_marker:=/rby1/vslam/replay_input/rby1/vslam/replay_marker
)
PLAY_ARGS=(ros2 bag play "$BAG" --rate 1.0 --remap "${REMAPS[@]}")
RELAY_PID=''; BRIDGE_PID=''
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  timeout 3 ros2 service call /rby1/vslam/sensor_replay/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  [[ -z "$BRIDGE_PID" ]] || kill -INT "$BRIDGE_PID" 2>/dev/null
  [[ -z "$RELAY_PID" ]] || kill -INT "$RELAY_PID" 2>/dev/null
  [[ -z "$BRIDGE_PID" ]] || wait "$BRIDGE_PID" 2>/dev/null
  [[ -z "$RELAY_PID" ]] || wait "$RELAY_PID" 2>/dev/null
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[sensor-replay] condition=$CONDITION stereo=$STEREO_MODE imu=$IMU_MODE domain=$DOMAIN"
echo "[sensor-replay] LAB must be freshly started with --enable-imu $ENABLE_IMU"
ros2 run rby1_vslam sensor_replay_relay --ros-args \
  -p "stereo_mode:=$STEREO_MODE" -p "imu_mode:=$IMU_MODE" &
RELAY_PID=$!
ros2 launch rby1_vslam upc.launch.py \
  "lab_host:=$LAB_HOST" start_camera:=false "enable_imu:=$ENABLE_IMU" \
  base_frame:=d435_link publish_mount_tf:=false &
BRIDGE_PID=$!

deadline=$((SECONDS + 15))
until [[ "$(ros2 service type /rby1/vslam/sensor_replay/arm 2>/dev/null || true)" == \
    'std_srvs/srv/Trigger' ]]; do
  ((SECONDS < deadline)) || die 'sensor replay arm service did not appear'
  sleep 0.2
done
echo '[sensor-replay] Relay and bridge are ready. Start the UPC and LAB recorders now.'
read -r -p '[sensor-replay] After fresh LAB cuVSLAM and both recorders are ready, press Enter... ' _
bridge_status="$(timeout 4 ros2 topic echo /rby1/vslam/bridge_status \
  std_msgs/msg/String --once 2>/dev/null || true)"
grep -q '"connected":true' <<<"$bridge_status" || \
  die 'UPC bridge is not connected to LAB; do not start an incomplete replay'
response="$(ros2 service call /rby1/vslam/sensor_replay/arm std_srvs/srv/Trigger '{}')"
grep -Eqi 'success[=:][[:space:]]*[Tt]rue' <<<"$response" || \
  die "relay arm failed: $response"

echo '[sensor-replay] Full deterministic playback started.'
"${PLAY_ARGS[@]}"
stop_response="$(timeout 3 ros2 service call /rby1/vslam/sensor_replay/stop \
  std_srvs/srv/Trigger '{}' 2>&1 || true)"
grep -Eqi 'success[=:][[:space:]]*[Tt]rue' <<<"$stop_response" || \
  die "replay validation failed: $stop_response"
echo '[sensor-replay] Playback finished. Stop both recorders after their final messages drain.'
sleep 2
