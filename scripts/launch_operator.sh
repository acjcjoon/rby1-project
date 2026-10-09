#!/usr/bin/env bash
# Shared implementation for the two UPC-local operator launchers.
set -Eeo pipefail
MODE="${1:?mode required}"; shift
die() { printf '[operator] %s\n' "$*" >&2; exit 2; }
[[ "$MODE" == mobile || "$MODE" == vslam ]] || die 'Invalid mode'
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<EOF
Mode: $MODE / UPC-local Qt UI. Run on UPC; choose only one operator launcher.
Arguments: name:=value (standard ROS launch arguments).
VSLAM requires lab_host:=<LAB-IP> and a running LAB VSLAM server.
VSLAM Nav2 requires the Humble nav2_mppi_controller package on the UPC.
VSLAM uses IMU by default; enable_imu:=false opts out on UPC (match LAB).
VSLAM forces the RSUSB librealsense build under ~/librealsense-rsusb-2.58.4.
Mobile starts only driver, safe control and UI (no TCP/camera/Nav2).
Defaults: ROS_DOMAIN_ID=current environment or 0.
Set RBY1_ROS_SETUP / RBY1_WORKSPACE_SETUP / RBY1_RSUSB_LIB_DIR for a different installation.
EOF
  exit 0
fi
LAB_HOST=''
for arg in "$@"; do
  [[ "$arg" == *:=* ]] || die "Expected name:=value, got: $arg"
  case "$arg" in
    start_ui:*) die 'UI selection is fixed to UPC-local Qt' ;;
    lab_host:=*) LAB_HOST="${arg#lab_host:=}" ;;
  esac
done
if [[ "$MODE" == vslam ]]; then
  [[ -n "$LAB_HOST" ]] || die 'Provide lab_host:=<LAB-IP>'
else
  [[ -z "$LAB_HOST" ]] || die 'Mobile mode does not use lab_host'
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  die 'UPC-local Qt needs a desktop session'
fi
ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${RBY1_WORKSPACE_SETUP:-${HOME}/rby1_ros2_ws/install/setup.bash}"
[[ -r "$ROS_SETUP" ]] || die "Missing ROS setup: $ROS_SETUP"
[[ -r "$WORKSPACE_SETUP" ]] || die "Missing workspace setup: $WORKSPACE_SETUP"
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ "$MODE" == vslam ]]; then
  ros2 pkg prefix nav2_mppi_controller >/dev/null 2>&1 || \
    die 'Missing nav2_mppi_controller; install ros-humble-nav2-mppi-controller and rebuild'
  RSUSB_LIB_DIR="${RBY1_RSUSB_LIB_DIR:-${HOME}/librealsense-rsusb-2.58.4/build-rsusb/Release}"
  [[ -r "$RSUSB_LIB_DIR/librealsense2.so.2.58" ]] || \
    die "Missing RSUSB librealsense: $RSUSB_LIB_DIR/librealsense2.so.2.58"
  export LD_LIBRARY_PATH="$RSUSB_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  printf '[operator] RealSense backend=%s\n' "$RSUSB_LIB_DIR/librealsense2.so.2.58"
fi
printf '[operator] mode=%s UI=upc-qt ROS_DOMAIN_ID=%s\n' "$MODE" "$ROS_DOMAIN_ID"
if [[ "$MODE" == mobile ]]; then
  exec ros2 launch rby1_vslam mobile_base_upc.launch.py "$@"
else
  exec ros2 launch rby1_vslam physical.launch.py enable_imu:=true "$@" start_ui:=true
fi
