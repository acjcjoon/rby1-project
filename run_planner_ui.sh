#!/usr/bin/env bash

# Real-robot launcher for the RB-Y1 planner stack.
# Keep nounset disabled until the ROS/colcon setup scripts have been sourced;
# those generated scripts legitimately read variables before defining them.
set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
DRIVER_REPO="${RBY1_DRIVER_REPO:-${WORKSPACE_ROOT}/src/rby1-ros2}"
INSTALL_PREFIX="${RBY1_INSTALL_PREFIX:-${WORKSPACE_ROOT}/install}"

NAMESPACE="${RBY1_NAMESPACE:-rby1}"
ROBOT_IP="${RBY1_ROBOT_IP:-192.168.30.1:50051}"
ROBOT_MODEL="${RBY1_ROBOT_MODEL:-m}"
ROBOT_VERSION="${RBY1_ROBOT_VERSION:-1_3}"

ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${RBY1_WORKSPACE_SETUP:-${INSTALL_PREFIX}/setup.bash}"
DRIVER_CONFIG="${RBY1_DRIVER_CONFIG:-${DRIVER_REPO}/rby1_driver/config/driver_parameters.yaml}"
CONTROL_CONFIG="${RBY1_CONTROL_CONFIG:-${SCRIPT_DIR}/rby1_control/config/default.yaml}"
NAVIGATION_CONFIG="${RBY1_NAVIGATION_CONFIG:-${SCRIPT_DIR}/rby1_navigation/config/navigation.yaml}"
PLANNER_CONFIG="${RBY1_PLANNER_CONFIG:-${SCRIPT_DIR}/rby1_planner/config/default.yaml}"
GRIPPER_CONFIG="${RBY1_GRIPPER_CONFIG:-${SCRIPT_DIR}/robot_hardware/rby1_gripper_driver/config/default.yaml}"
GRIPPER_TRANSPORT="${RBY1_GRIPPER_TRANSPORT:-real}"

# Homing moves both physical grippers through their full strokes. Require an
# explicit opt-in on every real-hardware startup unless the environment says
# otherwise.
GRIPPER_AUTO_HOME="${RBY1_GRIPPER_AUTO_HOME:-true}"
BUILD_MODE="${RBY1_BUILD_WORKSPACE:-auto}"
CHECK_ROBOT_CONNECTION="${RBY1_CHECK_ROBOT_CONNECTION:-true}"
STARTUP_TIMEOUT_SEC="${RBY1_STARTUP_TIMEOUT_SEC:-45}"

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "" \
    "Starts the real RB-Y1 driver, state publisher, physical gripper," \
    "RealSense, AprilTag vision, control, navigation, and planner UI." \
    "" \
    "Options:" \
    "  --namespace NAME             ROS namespace (default: rby1)" \
    "  --robot-ip HOST:PORT         Robot RPC address (default: 192.168.30.1:50051)" \
    "  --robot-model a|m            Robot model (default: m)" \
    "  --robot-version VERSION      URDF version (default: 1_3)" \
    "  --ros-setup FILE             ROS setup.bash (default: /opt/ros/humble/setup.bash)" \
    "  --workspace-setup FILE       Colcon workspace setup.bash" \
    "  --driver-config FILE         rby1_driver parameter YAML" \
    "  --control-config FILE        rby1_control parameter YAML" \
    "  --navigation-config FILE     rby1_navigation parameter YAML" \
    "  --planner-config FILE        rby1_planner parameter YAML" \
    "  --gripper-config FILE        gripper driver parameter YAML" \
    "  --gripper-transport real|sim Gripper transport (default: real)" \
    "  --gripper-auto-home BOOL     Home on startup (default: false; moves hardware)" \
    "  --build                      Always build the required workspace packages" \
    "  --no-build                   Never build; require an existing installation" \
    "  --skip-robot-check           Do not test the robot RPC port before launch" \
    "  --startup-timeout SEC        Per-topic startup timeout (default: 45)" \
    "  -h, --help                   Show this help"
}

die() {
  printf '[launcher] ERROR: %s\n' "$*" >&2
  exit 1
}

require_value() {
  local option="$1"
  local count="$2"
  if ((count < 2)); then
    printf 'Missing value for %s\n' "${option}" >&2
    exit 2
  fi
}

while (($# > 0)); do
  case "$1" in
    --namespace)
      require_value "$1" "$#"
      NAMESPACE="$2"
      shift 2
      ;;
    --robot-ip)
      require_value "$1" "$#"
      ROBOT_IP="$2"
      shift 2
      ;;
    --robot-model)
      require_value "$1" "$#"
      ROBOT_MODEL="$2"
      shift 2
      ;;
    --robot-version)
      require_value "$1" "$#"
      ROBOT_VERSION="$2"
      shift 2
      ;;
    --ros-setup)
      require_value "$1" "$#"
      ROS_SETUP="$2"
      shift 2
      ;;
    --workspace-setup)
      require_value "$1" "$#"
      WORKSPACE_SETUP="$2"
      shift 2
      ;;
    --driver-config)
      require_value "$1" "$#"
      DRIVER_CONFIG="$2"
      shift 2
      ;;
    --control-config)
      require_value "$1" "$#"
      CONTROL_CONFIG="$2"
      shift 2
      ;;
    --navigation-config)
      require_value "$1" "$#"
      NAVIGATION_CONFIG="$2"
      shift 2
      ;;
    --planner-config)
      require_value "$1" "$#"
      PLANNER_CONFIG="$2"
      shift 2
      ;;
    --gripper-config)
      require_value "$1" "$#"
      GRIPPER_CONFIG="$2"
      shift 2
      ;;
    --gripper-transport)
      require_value "$1" "$#"
      GRIPPER_TRANSPORT="$2"
      shift 2
      ;;
    --gripper-auto-home)
      require_value "$1" "$#"
      GRIPPER_AUTO_HOME="$2"
      shift 2
      ;;
    --build)
      BUILD_MODE="always"
      shift
      ;;
    --no-build)
      BUILD_MODE="never"
      shift
      ;;
    --skip-robot-check)
      CHECK_ROBOT_CONNECTION="false"
      shift
      ;;
    --startup-timeout)
      require_value "$1" "$#"
      STARTUP_TIMEOUT_SEC="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

NAMESPACE="${NAMESPACE#/}"
[[ -n "${NAMESPACE}" ]] || die 'Namespace must not be empty.'
[[ "${ROBOT_MODEL}" == "a" || "${ROBOT_MODEL}" == "m" ]] || \
  die 'Robot model must be a or m.'
[[ "${GRIPPER_TRANSPORT}" == "real" || "${GRIPPER_TRANSPORT}" == "sim" ]] || \
  die 'Gripper transport must be real or sim.'
[[ "${GRIPPER_AUTO_HOME}" == "true" || "${GRIPPER_AUTO_HOME}" == "false" ]] || \
  die 'Gripper auto-home must be true or false.'
[[ "${BUILD_MODE}" == "auto" || "${BUILD_MODE}" == "always" || "${BUILD_MODE}" == "never" ]] || \
  die 'RBY1_BUILD_WORKSPACE must be auto, always, or never.'
[[ "${CHECK_ROBOT_CONNECTION}" == "true" || "${CHECK_ROBOT_CONNECTION}" == "false" ]] || \
  die 'RBY1_CHECK_ROBOT_CONNECTION must be true or false.'
[[ "${STARTUP_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] || \
  die 'Startup timeout must be a positive integer.'

ROBOT_URDF="${DRIVER_REPO}/rby1_description/urdf/rby1${ROBOT_MODEL}/model_v${ROBOT_VERSION}.urdf"

for required_file in \
  "${DRIVER_CONFIG}" \
  "${CONTROL_CONFIG}" \
  "${NAVIGATION_CONFIG}" \
  "${PLANNER_CONFIG}" \
  "${GRIPPER_CONFIG}" \
  "${ROBOT_URDF}"; do
  [[ -r "${required_file}" ]] || die "Required file is missing: ${required_file}"
done

if [[ -r "${ROS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
elif ! command -v ros2 >/dev/null 2>&1; then
  die "ROS 2 is not available. Install ROS 2 Humble or set RBY1_ROS_SETUP/--ros-setup. Tried: ${ROS_SETUP}"
else
  printf '[launcher] ROS environment is already active; setup file not found: %s\n' "${ROS_SETUP}"
fi

command -v ros2 >/dev/null 2>&1 || die 'The ros2 command is unavailable after loading ROS.'
command -v colcon >/dev/null 2>&1 || die 'The colcon command is unavailable.'

PROJECT_PACKAGES=(
  rby1_interface
  rby1_driver
  rby1_description
  rby1_gripper_driver
  rby1_camera
  rby1_control
  rby1_navigation
  rby1_planner
)

source_workspace_if_present() {
  if [[ -r "${WORKSPACE_SETUP}" ]]; then
    # shellcheck disable=SC1090
    source "${WORKSPACE_SETUP}"
    return 0
  fi
  return 1
}

workspace_needs_build() {
  local package
  [[ -r "${WORKSPACE_SETUP}" ]] || return 0
  for package in "${PROJECT_PACKAGES[@]}"; do
    ros2 pkg prefix "${package}" >/dev/null 2>&1 || return 0
  done
  return 1
}

source_workspace_if_present || true

DO_BUILD="false"
if [[ "${BUILD_MODE}" == "always" ]]; then
  DO_BUILD="true"
elif [[ "${BUILD_MODE}" == "auto" ]] && workspace_needs_build; then
  DO_BUILD="true"
fi

if [[ "${DO_BUILD}" == "true" ]]; then
  printf '[launcher] Building required packages in %s\n' "${WORKSPACE_ROOT}"
  (
    cd "${WORKSPACE_ROOT}"
    colcon build \
      --symlink-install \
      --base-paths "${DRIVER_REPO}" "${SCRIPT_DIR}" \
      --packages-up-to "${PROJECT_PACKAGES[@]}"
  )
  [[ -r "${WORKSPACE_SETUP}" ]] || \
    die "Build completed without creating: ${WORKSPACE_SETUP}"
  # Reload the overlay so newly generated package metadata and entry points
  # are visible to this launcher process.
  # shellcheck disable=SC1090
  source "${WORKSPACE_SETUP}"
elif [[ ! -r "${WORKSPACE_SETUP}" ]]; then
  die "Workspace is not built: ${WORKSPACE_SETUP}. Re-run without --no-build."
fi

# ROS setup scripts have now finished reading optional unset variables.
set -u

RUNTIME_PACKAGES=(
  rby1_driver
  rby1_description
  robot_state_publisher
  rby1_gripper_driver
  realsense2_camera
  apriltag_ros
  rby1_camera
  rby1_control
  rby1_navigation
  rby1_planner
)

MISSING_PACKAGES=()
for package in "${RUNTIME_PACKAGES[@]}"; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    MISSING_PACKAGES+=("${package}")
  fi
done
if ((${#MISSING_PACKAGES[@]} > 0)); then
  printf '[launcher] ERROR: Missing ROS packages: %s\n' "${MISSING_PACKAGES[*]}" >&2
  printf '%s\n' \
    '[launcher] Install the external ROS dependencies, then run this script with --build.' >&2
  exit 1
fi

python3 -c 'import rby1_sdk' >/dev/null 2>&1 || \
  die 'Python module rby1_sdk is unavailable in the ROS runtime Python environment.'

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  die 'No graphical display is available (DISPLAY and WAYLAND_DISPLAY are unset).'
fi

check_robot_connection() {
  python3 - "${ROBOT_IP}" <<'PY'
import socket
import sys

address = sys.argv[1]
try:
    host, port_text = address.rsplit(':', 1)
    host = host.strip('[]')
    port = int(port_text)
except (ValueError, TypeError):
    print(f'[launcher] Invalid robot address: {address}', file=sys.stderr)
    raise SystemExit(2)

try:
    with socket.create_connection((host, port), timeout=2.0):
        pass
except OSError as exc:
    print(
        f'[launcher] Robot RPC connection failed ({address}): {exc}',
        file=sys.stderr,
    )
    raise SystemExit(1)
PY
}

if [[ "${CHECK_ROBOT_CONNECTION}" == "true" ]]; then
  printf '[launcher] Checking robot RPC at %s\n' "${ROBOT_IP}"
  check_robot_connection || \
    die 'Robot is unreachable. Check power/network/IP, or use --skip-robot-check for diagnostics.'
fi

CHILD_PIDS=()
CHILD_NAMES=()

start_process() {
  local name="$1"
  shift
  printf '[launcher] Starting %s\n' "${name}"
  "$@" &
  CHILD_PIDS+=("$!")
  CHILD_NAMES+=("${name}")
}

wait_for_topic() {
  local topic="$1"
  local owner="$2"
  local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  local index

  while ((SECONDS < deadline)); do
    for index in "${!CHILD_PIDS[@]}"; do
      if ! kill -0 "${CHILD_PIDS[index]}" 2>/dev/null; then
        wait "${CHILD_PIDS[index]}" 2>/dev/null || true
        die "${CHILD_NAMES[index]} exited while waiting for ${topic}."
      fi
    done

    if timeout 3 ros2 topic info "${topic}" 2>/dev/null | \
        grep -E 'Publisher count: [1-9][0-9]*' >/dev/null; then
      printf '[launcher] Ready: %s publishes %s\n' "${owner}" "${topic}"
      return 0
    fi
    sleep 0.5
  done

  die "Timed out after ${STARTUP_TIMEOUT_SEC}s waiting for ${owner} on ${topic}."
}

cleanup() {
  local status=$?
  local deadline
  local pid
  local alive

  trap - EXIT INT TERM
  if ((${#CHILD_PIDS[@]} == 0)); then
    exit "${status}"
  fi

  printf '\n[launcher] Stopping child processes...\n'
  kill -INT "${CHILD_PIDS[@]}" 2>/dev/null || true
  deadline=$((SECONDS + 5))
  while ((SECONDS < deadline)); do
    alive="false"
    for pid in "${CHILD_PIDS[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        alive="true"
        break
      fi
    done
    [[ "${alive}" == "false" ]] && break
    sleep 0.2
  done

  for pid in "${CHILD_PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${CHILD_PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  exit "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf '%s\n' \
  "[launcher] workspace=${WORKSPACE_ROOT}" \
  "[launcher] install=${INSTALL_PREFIX}" \
  "[launcher] namespace=/${NAMESPACE}" \
  "[launcher] robot=${ROBOT_MODEL} v${ROBOT_VERSION}" \
  "[launcher] robot_ip=${ROBOT_IP}" \
  "[launcher] gripper=${GRIPPER_TRANSPORT}, auto_home=${GRIPPER_AUTO_HOME}"

if [[ "${GRIPPER_TRANSPORT}" == "real" && "${GRIPPER_AUTO_HOME}" == "true" ]]; then
  printf '%s\n' \
    '[launcher] WARNING: REAL gripper auto-home is enabled.' \
    '[launcher] Keep both gripper strokes completely clear.'
fi

start_process robot_driver \
  ros2 run rby1_driver rby1_ros2_driver \
  --ros-args \
  --params-file "${DRIVER_CONFIG}" \
  -p "robot_ip:=${ROBOT_IP}" \
  -p "model:=${ROBOT_MODEL}" \
  -r "__ns:=/${NAMESPACE}"
wait_for_topic "/${NAMESPACE}/joint_states" robot_driver

start_process robot_state_publisher \
  ros2 run robot_state_publisher robot_state_publisher \
  "${ROBOT_URDF}" \
  --ros-args \
  -r "joint_states:=/${NAMESPACE}/joint_states"

start_process gripper_driver \
  ros2 launch rby1_gripper_driver gripper_driver.launch.py \
  "namespace:=${NAMESPACE}" \
  "config:=${GRIPPER_CONFIG}" \
  "transport:=${GRIPPER_TRANSPORT}" \
  "auto_home:=${GRIPPER_AUTO_HOME}"
wait_for_topic "/${NAMESPACE}/gripper/ready" gripper_driver

start_process realsense \
  ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true
wait_for_topic '/camera/camera/color/image_raw' realsense

start_process apriltag_vision \
  ros2 launch rby1_camera apriltag_vision.launch.py

start_process control \
  ros2 launch rby1_control control.launch.py \
  "namespace:=${NAMESPACE}" \
  "config:=${CONTROL_CONFIG}" \
  "robot_address:=${ROBOT_IP}" \
  "robot_model:=${ROBOT_MODEL}"
wait_for_topic "/${NAMESPACE}/control/state" control

start_process navigation \
  ros2 launch rby1_navigation odom_navigation.launch.py \
  "config_file:=${NAVIGATION_CONFIG}"
wait_for_topic "/${NAMESPACE}/navigation/state" navigation

printf '[launcher] Starting planner_ui\n'
set +e
ros2 launch rby1_planner planner_ui.launch.py \
  "namespace:=${NAMESPACE}" \
  "config:=${PLANNER_CONFIG}"
PLANNER_STATUS=$?
set -e

exit "${PLANNER_STATUS}"
