#!/usr/bin/env bash
# Passive full-topic recording, independent of robot/UI launcher lifetime.
set -Eeo pipefail
ROLE=''; RUN_ID=''; OUTPUT="${HOME}/rby1_trials"; DOMAIN=''; WAYPOINTS=''
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
die() { echo "[record] $*" >&2; exit 2; }
usage() {
  cat <<'EOF'
Usage: bash record_trial.sh --role upc|lab --run-id NAME [options]
  --domain ID       Match this machine's running stack (defaults: UPC 0, LAB 85)
  --output DIR      Parent directory (default: ~/rby1_trials)
  --waypoints FILE  Also copy a custom waypoint YAML at start/end (UPC)
All discovered topics, including images and hidden action topics, are recorded.
Start BEFORE the operator stack. Stop robot via UI, then Ctrl+C here to finalize.
This script never sends robot commands. Topic recording is not service tracing.
LAB: run inside Isaac ROS; choose --output on a persistent mounted directory.
EOF
}
while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --role|--run-id|--output|--domain|--waypoints)
      [[ $# -ge 2 && -n "$2" ]] || die "Missing value: $1"
      case "$1" in
        --role) ROLE="$2" ;; --run-id) RUN_ID="$2" ;; --output) OUTPUT="$2" ;;
        --domain) DOMAIN="$2" ;; --waypoints) WAYPOINTS="$2" ;;
      esac
      shift 2 ;;
    *) die "Unknown argument: $1" ;;
  esac
done
[[ "$ROLE" == upc || "$ROLE" == lab ]] || die '--role upc|lab required'
[[ "$RUN_ID" =~ ^[a-zA-Z0-9_-]+$ ]] || die '--run-id requires letters/digits/_/-'
if [[ "$ROLE" == upc ]]; then
  ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
  WS_SETUP="${RBY1_WORKSPACE_SETUP:-${HOME}/rby1_ros2_ws/install/setup.bash}"
  DOMAIN="${DOMAIN:-${ROS_DOMAIN_ID:-0}}"
else
  ROS_SETUP="${RBY1_LAB_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
  WS_SETUP="${RBY1_WORKSPACE_SETUP:-${RBY1_LAB_WS:-/workspaces/isaac_ros-dev}/install/setup.bash}"
  DOMAIN="${DOMAIN:-${ROS_DOMAIN_ID:-85}}"
fi
[[ "$DOMAIN" =~ ^[0-9]+$ ]] || die 'Invalid domain'
[[ -r "$ROS_SETUP" && -r "$WS_SETUP" ]] || die 'ROS/workspace setup missing'
source "$ROS_SETUP"
source "$WS_SETUP"
set -u
export ROS_DOMAIN_ID="$DOMAIN"
command -v timeout >/dev/null || die 'GNU timeout is required'
mkdir -p -- "$OUTPUT"
RUN_DIR="$(mktemp -d "${OUTPUT%/}/${RUN_ID}_${ROLE}_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
if [[ "$ROLE" == upc && -z "$WAYPOINTS" ]]; then
  WAYPOINTS="$(python3 -c 'from ament_index_python.packages import get_package_share_directory; from rby1_vslam.waypoint_store import default_waypoints_path; print(default_waypoints_path(get_package_share_directory("rby1_vslam")))' 2>/dev/null || true)"
fi
{
  printf 'run_id=%s\nrole=%s\nROS_DOMAIN_ID=%s\nROS_DISTRO=%s\nwaypoints=%s\n' \
    "$RUN_ID" "$ROLE" "$DOMAIN" "${ROS_DISTRO:-unset}" "$WAYPOINTS"
  date -u; uname -a; df -h "$RUN_DIR"
  if command -v chronyc >/dev/null; then timeout 2 chronyc tracking || true; fi
} > "$RUN_DIR/environment.txt" 2>&1
snapshot() {
  local phase="$1" node
  timeout 5 ros2 topic list -t > "$RUN_DIR/topics_${phase}.txt" 2>&1 || true
  timeout 5 ros2 node list > "$RUN_DIR/nodes_${phase}.txt" 2>&1 || true
  mkdir -p "$RUN_DIR/params_${phase}"
  while IFS= read -r node; do
    [[ "$node" == /* ]] || continue
    timeout 3 ros2 param dump "$node" > "$RUN_DIR/params_${phase}/${node//\//_}.yaml" 2>&1 || true
  done < "$RUN_DIR/nodes_${phase}.txt"
  if [[ -n "$WAYPOINTS" && -f "$WAYPOINTS" ]]; then
    cp -- "$WAYPOINTS" "$RUN_DIR/waypoints_${phase}.yaml"
  fi
}
BAG_PID=''; SAMPLE_PID=''; SNAPSHOT_PID=''
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  # SIGINT finalizes the bag; recording never stops the robot.
  [[ -z "$BAG_PID" ]] || kill -INT "$BAG_PID" 2>/dev/null
  [[ -z "$SAMPLE_PID" ]] || kill -TERM "$SAMPLE_PID" 2>/dev/null
  [[ -z "$SNAPSHOT_PID" ]] || wait "$SNAPSHOT_PID"
  [[ -z "$BAG_PID" ]] || wait "$BAG_PID"
  [[ -z "$SAMPLE_PID" ]] || wait "$SAMPLE_PID"
  snapshot end
  timeout 15 ros2 bag info "$RUN_DIR/bag" > "$RUN_DIR/bag_info.txt" 2>&1 || true
  echo "[record] Saved: $RUN_DIR (check recorder.log and bag_info.txt)"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Reset SIGINT ignored by noninteractive Bash background children.
python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1],sys.argv[1:])' \
  ros2 bag record -a --include-hidden-topics -o "$RUN_DIR/bag" \
  > "$RUN_DIR/recorder.log" 2>&1 &
BAG_PID=$!
snapshot start & SNAPSHOT_PID=$!
(
  while true; do
    date -u; uptime; df -h "$RUN_DIR"
    if command -v chronyc >/dev/null; then timeout 2 chronyc tracking || true; fi
    sleep 5
  done
) > "$RUN_DIR/system_samples.txt" 2>&1 & SAMPLE_PID=$!
echo "[record] Recorder launched: $RUN_DIR"
echo '[record] Confirm subscriptions in recorder.log; keep recording through the stop + 10 seconds.'
echo '[record] Stop the robot in the UI first, then Ctrl+C here. No motion commands are sent.'
wait "$BAG_PID"
