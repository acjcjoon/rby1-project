#!/usr/bin/env bash
# Run the Jetson/UPC side of the RBY1 TCP VSLAM pipeline (ROS 2 Humble).

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VSLAM_PKG_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd -- "${VSLAM_PKG_ROOT}/.." && pwd)"
DEFAULT_WS="$(cd -- "${PROJECT_ROOT}/../.." && pwd)"

UPC_WS="${RBY1_WS:-${DEFAULT_WS}}"
ROS_SETUP="${RBY1_UPC_ROS_SETUP:-/opt/ros/humble/setup.bash}"
LAB_HOST=""
PORT="7447"
ROS_DOMAIN="${ROS_DOMAIN_ID:-0}"
ENABLE_IMU="false"
START_CAMERA="true"
SERIAL_NO=""

usage() {
  cat <<'EOF'
Usage: run_vslam_upc.sh --lab-host LAB_IP [options]

  --lab-host HOST          Required LAB PC wired-LAN address
  --port PORT              Default: 7447
  --domain ID              Default: env or 0
  --enable-imu true|false  Default: false
  --start-camera true|false
                           Use false if the D435i is already running
  --serial SERIAL          Optional D435i serial number
  --workspace PATH         Jetson ROS 2 workspace
  -h, --help

This starts D435i, mount TF, TCP bridge and pose adapter through upc.launch.py.
It does not start the robot driver, control, Nav2 or operator UI.
EOF
}

die() { echo "[vslam-upc] ERROR: $*" >&2; exit 1; }
require_value() { [[ $# -ge 2 ]] || die "Missing value for $1"; }

while (($#)); do
  case "$1" in
    --lab-host) require_value "$@"; LAB_HOST="$2"; shift 2 ;;
    --port) require_value "$@"; PORT="$2"; shift 2 ;;
    --domain) require_value "$@"; ROS_DOMAIN="$2"; shift 2 ;;
    --enable-imu) require_value "$@"; ENABLE_IMU="$2"; shift 2 ;;
    --start-camera) require_value "$@"; START_CAMERA="$2"; shift 2 ;;
    --serial) require_value "$@"; SERIAL_NO="${2#_}"; shift 2 ;;
    --workspace) require_value "$@"; UPC_WS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown option: $1" ;;
  esac
done

[[ -n "$LAB_HOST" ]] || die "--lab-host is required"
[[ "$LAB_HOST" != "127.0.0.1" && "$LAB_HOST" != "localhost" ]] || die "Use the separate LAB PC address, not localhost"
[[ "$ENABLE_IMU" == "true" || "$ENABLE_IMU" == "false" ]] || die "--enable-imu must be true or false"
[[ "$START_CAMERA" == "true" || "$START_CAMERA" == "false" ]] || die "--start-camera must be true or false"
[[ "$PORT" =~ ^[0-9]+$ ]] && ((PORT >= 1 && PORT <= 65535)) || die "Invalid port: $PORT"
[[ "$ROS_DOMAIN" =~ ^[0-9]+$ ]] || die "Invalid ROS domain: $ROS_DOMAIN"
[[ -z "$SERIAL_NO" || "$SERIAL_NO" =~ ^[0-9]+$ ]] || die "--serial must contain digits only"

WORKSPACE_SETUP="${UPC_WS}/install/setup.bash"
[[ -r "$ROS_SETUP" ]] || die "ROS setup not found: $ROS_SETUP"
[[ -r "$WORKSPACE_SETUP" ]] || die "Workspace is not built: $WORKSPACE_SETUP"

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WORKSPACE_SETUP"
export ROS_DOMAIN_ID="$ROS_DOMAIN"

ros2 pkg prefix rby1_vslam >/dev/null 2>&1 || die "rby1_vslam is not built in $UPC_WS"

ARGS=("lab_host:=$LAB_HOST" "port:=$PORT" "enable_imu:=$ENABLE_IMU" "start_camera:=$START_CAMERA")
[[ -n "$SERIAL_NO" ]] && ARGS+=("serial_no:=$SERIAL_NO")

echo "[vslam-upc] workspace=$UPC_WS"
echo "[vslam-upc] LAB=$LAB_HOST:$PORT, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, IMU=$ENABLE_IMU"
echo "[vslam-upc] NOTE: driver/control/Nav2/UI are not started by this script."
exec ros2 launch rby1_vslam upc.launch.py "${ARGS[@]}"
