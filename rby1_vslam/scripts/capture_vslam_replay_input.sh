#!/usr/bin/env bash
# Capture one reusable raw stereo/IMU sequence for controlled VSLAM comparisons.
set -Eeo pipefail

RUN_ID=''; OUTPUT="${HOME}/rby1_vslam_replay_inputs"; DOMAIN='10'
usage() {
  cat <<'EOF'
Usage: capture_vslam_replay_input.sh --run-id NAME [options]
  --domain ID   ROS_DOMAIN_ID of the UPC camera (default: 10)
  --output DIR  Parent output directory (default: ~/rby1_vslam_replay_inputs)

The D435i and mobile-base operator must already be running. This script sends
no motion commands. It records an initial 10 s stationary interval, lets the
operator rotate the base twice, then records a final 10 s stationary interval.
EOF
}
die() { echo "[sensor-capture] ERROR: $*" >&2; exit 2; }
while (($#)); do
  case "$1" in
    --run-id) [[ $# -ge 2 ]] || die 'missing --run-id value'; RUN_ID="$2"; shift 2 ;;
    --domain) [[ $# -ge 2 ]] || die 'missing --domain value'; DOMAIN="$2"; shift 2 ;;
    --output) [[ $# -ge 2 ]] || die 'missing --output value'; OUTPUT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
[[ "$RUN_ID" =~ ^[a-zA-Z0-9_-]+$ ]] || die '--run-id requires letters/digits/_/-'
[[ "$DOMAIN" =~ ^[0-9]+$ ]] || die '--domain must be numeric'

ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${RBY1_WORKSPACE_SETUP:-${HOME}/rby1_ros2_ws/install/setup.bash}"
[[ -r "$ROS_SETUP" && -r "$WORKSPACE_SETUP" ]] || die 'ROS/workspace setup missing'
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
export ROS_DOMAIN_ID="$DOMAIN"

TOPICS=(
  /d435/d435/infra1/image_rect_raw
  /d435/d435/infra2/image_rect_raw
  /d435/d435/infra1/camera_info
  /d435/d435/infra2/camera_info
  /d435/d435/imu
  /tf_static
  /rby1/vslam/replay_marker
)
for topic in "${TOPICS[@]:0:5}"; do
  timeout 5 ros2 topic echo "$topic" --once >/dev/null 2>&1 || \
    die "no message on $topic; start the D435i first"
done

mkdir -p -- "$OUTPUT"
RUN_DIR="$(mktemp -d "${OUTPUT%/}/${RUN_ID}_sensor_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
{
  printf 'run_id=%s\nROS_DOMAIN_ID=%s\n' "$RUN_ID" "$ROS_DOMAIN_ID"
  printf 'protocol=10s_initial_stationary,2x_rotation,10s_returned_stationary\n'
  date -u
} > "$RUN_DIR/environment.txt"

BAG_PID=''
finish() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  if [[ -n "$BAG_PID" ]]; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    timeout 30s tail --pid="$BAG_PID" -f /dev/null >/dev/null 2>&1 || \
      kill -TERM "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  ros2 bag info "$RUN_DIR/bag" > "$RUN_DIR/bag_info.txt" 2>&1 || true
  echo "[sensor-capture] Saved: $RUN_DIR"
  exit "$status"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

ros2 bag record -o "$RUN_DIR/bag" "${TOPICS[@]}" > "$RUN_DIR/recorder.log" 2>&1 &
BAG_PID=$!
sleep 2
kill -0 "$BAG_PID" 2>/dev/null || die 'rosbag recorder exited; inspect recorder.log'

mark() {
  local phase="$1"
  python3 -c 'import json,sys,time; p=sys.argv[1]; r={"phase":sys.argv[2],"wall_ns":time.time_ns(),"monotonic_ns":time.monotonic_ns()}; f=open(p,"a",encoding="utf-8"); f.write(json.dumps(r,separators=(",",":"))+"\n"); f.close()' \
    "$RUN_DIR/markers.jsonl" "$phase"
  timeout 5 ros2 topic pub --once /rby1/vslam/replay_marker \
    std_msgs/msg/String "{data: '$phase'}" >/dev/null 2>&1 || \
    die "failed to record phase marker: $phase"
  echo "[sensor-capture] phase=$phase"
}

read -r -p '[sensor-capture] Keep the base/head still, then press Enter... ' _
mark initial_stationary
echo '[sensor-capture] Recording 10 s initial stationary data...'
sleep 10
mark rotation_start
echo '[sensor-capture] Rotate in place exactly two turns; keep the head fixed.'
read -r -p '[sensor-capture] After two turns, stop fully and press Enter... ' _
mark returned_stationary
echo '[sensor-capture] Recording 10 s final stationary data...'
sleep 10
mark capture_end
exit 0
