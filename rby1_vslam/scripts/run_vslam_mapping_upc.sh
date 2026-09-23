#!/usr/bin/env bash
# RBY1 real-robot VSLAM mapping launcher for the UPC (ROS 2 Humble).
#
# Starts/reuses only what is needed for MANUAL mapping:
#   rby1_driver
#   robot_state_publisher
#   rby1_control
#   rby1_vslam/upc.launch.py (D435i + TCP bridge + pose adapter)
#   rby1_planner/planner_ui.launch.py (Base Jog / operator controls)
#
# Intentionally DOES NOT start:
#   rby1_navigation / Nav2
#   D405 / AprilTag
#   gripper driver
#
# Usage:
#   ./run_vslam_mapping_upc.sh --lab-host 192.168.30.50

set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VSLAM_PKG_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd -- "${VSLAM_PKG_ROOT}/.." && pwd)"
WORKSPACE_ROOT="$(cd -- "${PROJECT_ROOT}/../.." && pwd)"

ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${RBY1_WORKSPACE_SETUP:-${WORKSPACE_ROOT}/install/setup.bash}"
DRIVER_REPO="${RBY1_DRIVER_REPO:-${WORKSPACE_ROOT}/src/rby1-ros2}"

NAMESPACE="${RBY1_NAMESPACE:-rby1}"
ROBOT_IP="${RBY1_ROBOT_IP:-192.168.30.1:50051}"
ROBOT_MODEL="${RBY1_ROBOT_MODEL:-m}"
ROBOT_VERSION="${RBY1_ROBOT_VERSION:-1_3}"
ROS_DOMAIN="${ROS_DOMAIN_ID:-10}"

LAB_HOST=""
D435_SERIAL=""
CAMERA_MODE="auto"
ENABLE_IMU="false"
START_UI="true"
STARTUP_TIMEOUT_SEC=45

CONTROL_CONFIG="${PROJECT_ROOT}/rby1_control/config/default.yaml"
PLANNER_CONFIG="${PROJECT_ROOT}/rby1_planner/config/default.yaml"
DRIVER_CONFIG="${DRIVER_REPO}/rby1_driver/config/driver_parameters.yaml"

usage() {
  cat <<'EOF'
Usage:
  run_vslam_mapping_upc.sh --lab-host LAB_IP [options]

Required:
  --lab-host HOST              Reachable LAB PC IP/hostname running Jazzy mapping

Options:
  --domain ID                  UPC ROS_DOMAIN_ID (default: current env or 10)
  --robot-ip HOST:PORT         RBY1 RPC address (default: 192.168.30.1:50051)
  --robot-model a|m            Robot model (default: m)
  --robot-version VERSION      URDF version (default: 1_3)
  --d435-serial SERIAL         Pin D435i serial (digits only)
  --camera auto|start|reuse    auto: reuse existing IR camera if present
  --enable-imu true|false      Forward/use IMU in UPC bridge (default: false)
  --no-ui                      Do not launch planner UI
  --startup-timeout SEC        Readiness timeout per stage (default: 45)
  -h, --help

This mapping launcher intentionally does NOT start rby1_navigation/Nav2.
Use the Planner UI "Base" tab for slow manual mapping motion.
EOF
}

die() { echo "[mapping-upc] ERROR: $*" >&2; exit 1; }
info() { echo "[mapping-upc] $*"; }
warn() { echo "[mapping-upc] WARNING: $*" >&2; }
require_value() { [[ $# -ge 2 ]] || die "Missing value for $1"; }

while (($#)); do
  case "$1" in
    --lab-host) require_value "$@"; LAB_HOST="$2"; shift 2 ;;
    --domain) require_value "$@"; ROS_DOMAIN="$2"; shift 2 ;;
    --robot-ip) require_value "$@"; ROBOT_IP="$2"; shift 2 ;;
    --robot-model) require_value "$@"; ROBOT_MODEL="$2"; shift 2 ;;
    --robot-version) require_value "$@"; ROBOT_VERSION="$2"; shift 2 ;;
    --d435-serial) require_value "$@"; D435_SERIAL="${2#_}"; shift 2 ;;
    --camera) require_value "$@"; CAMERA_MODE="$2"; shift 2 ;;
    --enable-imu) require_value "$@"; ENABLE_IMU="$2"; shift 2 ;;
    --no-ui) START_UI="false"; shift ;;
    --startup-timeout) require_value "$@"; STARTUP_TIMEOUT_SEC="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown option: $1" ;;
  esac
done

[[ -n "$LAB_HOST" ]] || die "--lab-host is required."
[[ "$ROBOT_MODEL" == "a" || "$ROBOT_MODEL" == "m" ]] || die "--robot-model must be a or m."
[[ "$CAMERA_MODE" == "auto" || "$CAMERA_MODE" == "start" || "$CAMERA_MODE" == "reuse" ]] || die "--camera must be auto, start, or reuse."
[[ "$ENABLE_IMU" == "true" || "$ENABLE_IMU" == "false" ]] || die "--enable-imu must be true or false."
[[ "$STARTUP_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ ]] || die "--startup-timeout must be a positive integer."
[[ -z "$D435_SERIAL" || "$D435_SERIAL" =~ ^[0-9]+$ ]] || die "D435 serial must contain digits only."

ROBOT_URDF="${DRIVER_REPO}/rby1_description/urdf/rby1${ROBOT_MODEL}/model_v${ROBOT_VERSION}.urdf"

for f in "$ROS_SETUP" "$WORKSPACE_SETUP" "$DRIVER_CONFIG" "$CONTROL_CONFIG" "$PLANNER_CONFIG" "$ROBOT_URDF"; do
  [[ -r "$f" ]] || die "Required file missing: $f"
done

source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
export ROS_DOMAIN_ID="$ROS_DOMAIN"

for pkg in rby1_driver rby1_description robot_state_publisher rby1_control rby1_planner rby1_vslam tf2_ros; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || die "ROS package not found: $pkg"
done

if [[ "$START_UI" == "true" && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  die "Planner UI requested but DISPLAY/WAYLAND_DISPLAY is unset. Use X11/desktop or pass --no-ui."
fi

python3 - "$ROBOT_IP" <<'PY' || die "RBY1 RPC is unreachable at ${ROBOT_IP}"
import socket, sys
host, port = sys.argv[1].rsplit(':', 1)
with socket.create_connection((host.strip('[]'), int(port)), timeout=2.0):
    pass
PY

publisher_count() {
  local topic="$1"
  local output

  # A topic that does not exist is a normal "0 publishers" condition here.
  # Do not let ros2 topic info's non-zero exit trip set -e/pipefail.
  output="$(timeout 3 ros2 topic info "$topic" 2>/dev/null || true)"

  awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}' <<<"$output" \
    | tail -n1
}

topic_has_publisher() {
  local n
  n="$(publisher_count "$1")"
  [[ "$n" =~ ^[0-9]+$ ]] && (( n > 0 ))
}

wait_for_topic() {
  local topic="$1" label="$2" deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  while ((SECONDS < deadline)); do
    if topic_has_publisher "$topic"; then
      info "Ready: ${label} -> ${topic}"
      return 0
    fi
    sleep 0.5
  done
  die "Timed out waiting for ${label} on ${topic}"
}

wait_for_message() {
  local topic="$1" label="$2" deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  while ((SECONDS < deadline)); do
    if timeout 3 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      info "Data OK: ${label} -> ${topic}"
      return 0
    fi
    sleep 0.5
  done
  die "Timed out waiting for an actual message from ${label} on ${topic}"
}

check_tf() {
  local parent="$1" child="$2" required="${3:-true}" output
  output="$(timeout 5 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true)"
  if grep -q 'Translation:' <<<"$output"; then
    info "TF OK: ${parent} -> ${child}"
    return 0
  fi
  if [[ "$required" == "true" ]]; then
    echo "$output" >&2
    die "Required TF missing: ${parent} -> ${child}"
  fi
  warn "TF not available yet: ${parent} -> ${child}"
  return 1
}

CHILD_PIDS=()
CHILD_NAMES=()

start_process() {
  local name="$1"
  shift
  info "Starting ${name}"
  "$@" &
  CHILD_PIDS+=("$!")
  CHILD_NAMES+=("$name")
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e

  info "Stopping mapping stack started by this script..."
  timeout 2 ros2 topic pub --once "/${NAMESPACE}/cmd_raw" \
    geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  sleep 0.4

  for ((i=${#CHILD_PIDS[@]}-1; i>=0; i--)); do
    pid="${CHILD_PIDS[i]}"
    name="${CHILD_NAMES[i]}"
    if kill -0 "$pid" 2>/dev/null; then
      info "Stopping ${name} (pid=${pid})"
      kill -INT "$pid" 2>/dev/null || true
      sleep 0.3
    fi
  done
  sleep 1
  for ((i=${#CHILD_PIDS[@]}-1; i>=0; i--)); do
    pid="${CHILD_PIDS[i]}"
    kill -TERM "$pid" 2>/dev/null || true
  done
  for pid in "${CHILD_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  exit "$status"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

info "workspace=${WORKSPACE_ROOT}"
info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
info "robot=${ROBOT_MODEL} v${ROBOT_VERSION}"
info "robot_ip=${ROBOT_IP}"
info "LAB=${LAB_HOST}:7447"
info "IMU=${ENABLE_IMU}"
info "NOTE: mapping mode; rby1_navigation/Nav2 will NOT be started."

if topic_has_publisher "/${NAMESPACE}/navigation/state"; then
  die "Existing rby1_navigation is running. Stop it before mapping."
fi

RAW_PUBS="$(publisher_count "/${NAMESPACE}/cmd_raw")"
if [[ "$RAW_PUBS" =~ ^[0-9]+$ ]] && (( RAW_PUBS > 0 )); then
  die "/${NAMESPACE}/cmd_raw already has ${RAW_PUBS} publisher(s). Stop other velocity producers before mapping."
fi

# 1) Driver
if topic_has_publisher "/${NAMESPACE}/joint_states" && topic_has_publisher "/${NAMESPACE}/robot_state"; then
  info "Reusing existing RBY1 driver."
else
  start_process robot_driver \
    ros2 run rby1_driver rby1_ros2_driver \
      --ros-args \
      --params-file "$DRIVER_CONFIG" \
      -p "robot_ip:=${ROBOT_IP}" \
      -p "model:=${ROBOT_MODEL}" \
      -r "__ns:=/${NAMESPACE}"
fi

wait_for_topic "/${NAMESPACE}/joint_states" robot_driver
wait_for_topic "/${NAMESPACE}/robot_state" robot_driver
wait_for_topic "/${NAMESPACE}/odom" robot_driver

# 2) Robot state publisher / joint TF
if check_tf base link_head_2 false; then
  info "Reusing existing robot_state_publisher/TF chain."
else
  start_process robot_state_publisher \
    ros2 run robot_state_publisher robot_state_publisher \
      "$ROBOT_URDF" \
      --ros-args \
      -r "joint_states:=/${NAMESPACE}/joint_states"

  deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  until check_tf base link_head_2 false; do
    ((SECONDS >= deadline)) && die "Timed out waiting for TF base -> link_head_2"
    sleep 0.5
  done
fi

check_tf odom base false || \
  warn "Mapping can continue, but fix odom -> base before localization/Nav2."

# 3) Control backend. It owns final /rby1/cmd_vel.
if topic_has_publisher "/${NAMESPACE}/control/state"; then
  info "Reusing existing rby1_control."
else
  start_process rby1_control \
    ros2 launch rby1_control control.launch.py \
      "namespace:=${NAMESPACE}" \
      "config:=${CONTROL_CONFIG}" \
      "robot_address:=${ROBOT_IP}" \
      "robot_model:=${ROBOT_MODEL}"
fi
wait_for_topic "/${NAMESPACE}/control/state" rby1_control
wait_for_topic "/${NAMESPACE}/cmd_vel" rby1_control

# 4) UPC VSLAM acquisition + TCP bridge + pose adapter.
if topic_has_publisher "/d435/d435/infra1/image_rect_raw"; then
  case "$CAMERA_MODE" in
    start) die "D435i IR already publishing, but --camera start was requested." ;;
    auto|reuse) START_CAMERA="false"; info "Reusing existing D435i IR stream." ;;
  esac
else
  case "$CAMERA_MODE" in
    reuse) die "--camera reuse requested, but no D435i IR publisher exists." ;;
    auto|start) START_CAMERA="true" ;;
  esac
fi

VSLAM_ARGS=(
  "lab_host:=${LAB_HOST}"
  "enable_imu:=${ENABLE_IMU}"
  "start_camera:=${START_CAMERA}"
  "base_frame:=base"
)
if [[ -n "$D435_SERIAL" ]]; then
  VSLAM_ARGS+=("serial_no:=${D435_SERIAL}")
fi

if topic_has_publisher "/rby1/vslam/bridge_status"; then
  info "Reusing existing rby1_vslam UPC bridge."
else
  start_process rby1_vslam_upc \
    ros2 launch rby1_vslam upc.launch.py "${VSLAM_ARGS[@]}"
fi

wait_for_message "/rby1/vslam/bridge_status" rby1_vslam_bridge
wait_for_message "/rby1/vslam/camera_odometry" LAB_cuVSLAM_return
check_tf base d435_link true
wait_for_message "/rby1/vslam/slam_odom" pose_adapter

# 5) Planner UI for manual mapping.
if [[ "$START_UI" == "true" ]]; then
  if ros2 node list 2>/dev/null | grep -qx "/${NAMESPACE}/rby1_planner_ui"; then
    info "Planner UI already exists; not starting duplicate."
  else
    start_process planner_ui \
      ros2 launch rby1_planner planner_ui.launch.py \
        "namespace:=${NAMESPACE}" \
        "config:=${PLANNER_CONFIG}"
  fi
fi

cat <<EOF

======================================================================
 RBY1 VSLAM MAPPING STACK READY
======================================================================
LAB:        ${LAB_HOST}:7447
UPC domain: ${ROS_DOMAIN_ID}

Verified:
  driver topics: /${NAMESPACE}/joint_states, /${NAMESPACE}/robot_state, /${NAMESPACE}/odom
  TF: base -> link_head_2
  TF: base -> d435_link
  VSLAM: /rby1/vslam/camera_odometry
  base-adapted SLAM odom: /rby1/vslam/slam_odom

MANUAL MAPPING:
  1. Keep the physical EMO reachable.
  2. Planner UI: verify EMO=RELEASED and Collision=CLEAR.
  3. Power ON.
  4. Servo ON.
  5. Stream ON.
  6. Control Manager ENABLE.
  7. Confirm CONTROL is ENABLE or EXECUTING.
  8. Base tab: start slow (~0.05 m/s, yaw ~0.10-0.15 rad/s).
  9. Move smoothly through the area and return near the start for loop closure.

IMPORTANT:
  rby1_navigation/Nav2 is intentionally OFF during mapping.
  D405/AprilTag/gripper are intentionally not started.

Stop everything started by this script with Ctrl+C here.
======================================================================

EOF

while true; do
  for i in "${!CHILD_PIDS[@]}"; do
    if ! kill -0 "${CHILD_PIDS[i]}" 2>/dev/null; then
      set +e
      wait "${CHILD_PIDS[i]}"
      rc=$?
      set -e
      die "${CHILD_NAMES[i]} exited unexpectedly (rc=${rc})."
    fi
  done
  sleep 1
done
