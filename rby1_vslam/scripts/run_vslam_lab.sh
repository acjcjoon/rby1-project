#!/usr/bin/env bash
# Run the LAB side of the RBY1 TCP VSLAM pipeline inside Isaac ROS.

set -Eeuo pipefail

LAB_WS="${RBY1_LAB_WS:-/workspaces/isaac_ros-dev}"
ROS_SETUP="${RBY1_LAB_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
MODE="mapping"
MAP_PATH=""
BIND_HOST="0.0.0.0"
PORT="7447"
ROS_DOMAIN="${ROS_DOMAIN_ID:-85}"
ENABLE_IMU="true"
START_RVIZ="true"
RVIZ_CONFIG=""

usage() {
  cat <<'EOF'
Usage: run_vslam_lab.sh [options]

  --mode mapping|localization|odometry  Mode (default: mapping)
  --map-path PATH                       Required for localization
  --bind-host ADDRESS                   Default: 0.0.0.0
  --port PORT                           Default: 7447
  --domain ID                           Default: env or 85
  --enable-imu true|false               Default: true
  --no-rviz                             Do not start RViz2
  --rviz-config PATH                    Optional RViz2 config file
  --workspace PATH                      Default: /workspaces/isaac_ros-dev
  -h, --help

Run this script inside the Isaac ROS Jazzy container.
EOF
}

die() { echo "[vslam-lab] ERROR: $*" >&2; exit 1; }
require_value() { [[ $# -ge 2 ]] || die "Missing value for $1"; }

while (($#)); do
  case "$1" in
    --mode) require_value "$@"; MODE="$2"; shift 2 ;;
    --map-path) require_value "$@"; MAP_PATH="$2"; shift 2 ;;
    --bind-host) require_value "$@"; BIND_HOST="$2"; shift 2 ;;
    --port) require_value "$@"; PORT="$2"; shift 2 ;;
    --domain) require_value "$@"; ROS_DOMAIN="$2"; shift 2 ;;
    --enable-imu) require_value "$@"; ENABLE_IMU="$2"; shift 2 ;;
    --no-rviz) START_RVIZ="false"; shift ;;
    --rviz-config) require_value "$@"; RVIZ_CONFIG="$2"; shift 2 ;;
    --workspace) require_value "$@"; LAB_WS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown option: $1" ;;
  esac
done

[[ "$MODE" == "mapping" || "$MODE" == "localization" || "$MODE" == "odometry" ]] || die "Invalid mode: $MODE"
[[ "$ENABLE_IMU" == "true" || "$ENABLE_IMU" == "false" ]] || die "--enable-imu must be true or false"
[[ "$PORT" =~ ^[0-9]+$ ]] && ((PORT >= 1 && PORT <= 65535)) || die "Invalid port: $PORT"
[[ "$ROS_DOMAIN" =~ ^[0-9]+$ ]] || die "Invalid ROS domain: $ROS_DOMAIN"
if [[ "$MODE" == "localization" ]]; then
  [[ -n "$MAP_PATH" && "$MAP_PATH" == /* && -d "$MAP_PATH" ]] || die "Localization needs an existing absolute --map-path"
fi
if [[ -n "$RVIZ_CONFIG" ]]; then
  [[ -r "$RVIZ_CONFIG" ]] || die "RViz config not found: $RVIZ_CONFIG"
fi

WORKSPACE_SETUP="${LAB_WS}/install/setup.bash"
[[ -r "$ROS_SETUP" ]] || die "ROS setup not found: $ROS_SETUP"
[[ -r "$WORKSPACE_SETUP" ]] || die "Workspace is not built: $WORKSPACE_SETUP"

# ROS/colcon generated setup files read optional variables directly and are not
# safe to source while Bash nounset is enabled. Keep strict mode for this script,
# but disable nounset only while importing the ROS environments.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WORKSPACE_SETUP"
set -u
export ROS_DOMAIN_ID="$ROS_DOMAIN"

ros2 pkg prefix isaac_ros_visual_slam >/dev/null 2>&1 || die "isaac_ros_visual_slam is not installed"
VSLAM_PREFIX="$(ros2 pkg prefix rby1_vslam 2>/dev/null)" || die "rby1_vslam is not built in $LAB_WS"
VSLAM_PARAMS="${VSLAM_PREFIX}/share/rby1_vslam/config/isaac_vslam.yaml"
[[ -r "$VSLAM_PARAMS" ]] || die "Installed VSLAM config not found: $VSLAM_PARAMS"
if [[ -z "$RVIZ_CONFIG" ]]; then
  RVIZ_CONFIG="${VSLAM_PREFIX}/share/rby1_vslam/config/lab_vslam.rviz"
fi
if [[ "$START_RVIZ" == "true" ]] && grep -Eq '^[[:space:]]*publish_(map_to_odom|odom_to_base)_tf:[[:space:]]*false' "$VSLAM_PARAMS"; then
  die "Installed config disables VSLAM TF. Copy the updated package into $LAB_WS/src and rebuild rby1_vslam."
fi
if [[ "$START_RVIZ" == "true" ]]; then
  command -v rviz2 >/dev/null 2>&1 || die "rviz2 is not installed"
  [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || die "RViz2 requested but DISPLAY/WAYLAND_DISPLAY is unset"
  [[ -r "$RVIZ_CONFIG" ]] || die "RViz config not found: $RVIZ_CONFIG. Rebuild rby1_vslam."
fi

ARGS=("mode:=$MODE" "bind_host:=$BIND_HOST" "port:=$PORT" "enable_imu:=$ENABLE_IMU")
[[ -n "$MAP_PATH" ]] && ARGS+=("map_path:=$MAP_PATH")

echo "[vslam-lab] workspace=$LAB_WS"
echo "[vslam-lab] mode=$MODE, listen=$BIND_HOST:$PORT, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, IMU=$ENABLE_IMU"
[[ "$START_RVIZ" == "true" ]] && echo "[vslam-lab] RViz preset=$RVIZ_CONFIG"

CHILD_PIDS=()
CHILD_NAMES=()

start_process() {
  local name="$1"
  shift
  echo "[vslam-lab] starting $name"
  "$@" &
  CHILD_PIDS+=("$!")
  CHILD_NAMES+=("$name")
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  for ((i=${#CHILD_PIDS[@]}-1; i>=0; i--)); do
    if kill -0 "${CHILD_PIDS[i]}" 2>/dev/null; then
      echo "[vslam-lab] stopping ${CHILD_NAMES[i]}"
      kill -INT "${CHILD_PIDS[i]}" 2>/dev/null || true
    fi
  done
  for pid in "${CHILD_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  exit "$status"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_process cuvslam ros2 launch rby1_vslam lab.launch.py "${ARGS[@]}"

if [[ "$START_RVIZ" == "true" ]]; then
  RVIZ_ARGS=(-d "$RVIZ_CONFIG" -f vslam_map)
  start_process rviz2 rviz2 "${RVIZ_ARGS[@]}"
fi

while true; do
  for i in "${!CHILD_PIDS[@]}"; do
    if ! kill -0 "${CHILD_PIDS[i]}" 2>/dev/null; then
      set +e
      wait "${CHILD_PIDS[i]}"
      rc=$?
      set -e
      die "${CHILD_NAMES[i]} exited (rc=$rc)"
    fi
  done
  sleep 1
done
