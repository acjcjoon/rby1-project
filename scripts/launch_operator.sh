#!/usr/bin/env bash
# Shared implementation for the four root operator launchers.
set -Eeo pipefail
MODE="${1:?mode required}"; UI="${2:?UI required}"; shift 2
die() { printf '[operator] %s\n' "$*" >&2; exit 2; }
[[ "$MODE" == mobile || "$MODE" == vslam ]] || die 'Invalid mode'
[[ "$UI" == web || "$UI" == qt ]] || die 'Invalid UI'
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<EOF
Mode: $MODE / UI: $UI. Run on UPC; choose only one operator launcher.
Arguments: name:=value (standard ROS launch arguments).
VSLAM requires lab_host:=<LAB-IP> and a running LAB VSLAM server.
VSLAM uses IMU by default; enable_imu:=false opts out on UPC (match LAB).
Mobile starts only driver, safe control and UI (no TCP/camera/Nav2).
Defaults: ROS_DOMAIN_ID=current environment or 0; web port=8080.
Set RBY1_ROS_SETUP / RBY1_WORKSPACE_SETUP for a different installation.
Recording is separate: bash record_trial.sh --role upc --run-id trial01
EOF
  exit 0
fi
LAB_HOST=''
for arg in "$@"; do
  [[ "$arg" == *:=* ]] || die "Expected name:=value, got: $arg"
  case "$arg" in
    ui_backend:*|start_ui:*) die 'UI selection is fixed by this launcher' ;;
    lab_host:=*) LAB_HOST="${arg#lab_host:=}" ;;
  esac
done
if [[ "$MODE" == vslam ]]; then
  [[ -n "$LAB_HOST" ]] || die 'Provide lab_host:=<LAB-IP>'
else
  [[ -z "$LAB_HOST" ]] || die 'Mobile mode does not use lab_host'
fi
if [[ "$UI" == qt && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  die 'UPC UI needs a desktop session; use web for headless operation'
fi
ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${RBY1_WORKSPACE_SETUP:-${HOME}/rby1_ros2_ws/install/setup.bash}"
[[ -r "$ROS_SETUP" ]] || die "Missing ROS setup: $ROS_SETUP"
[[ -r "$WORKSPACE_SETUP" ]] || die "Missing workspace setup: $WORKSPACE_SETUP"
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
printf '[operator] mode=%s UI=%s ROS_DOMAIN_ID=%s\n' "$MODE" "$UI" "$ROS_DOMAIN_ID"
if [[ "$UI" == web ]]; then
  echo '[operator] Open http://<UPC-IP>:8080 (or your web_port override).'
fi
if [[ "$MODE" == mobile ]]; then
  exec ros2 launch rby1_web mobile_base_web.launch.py "$@" ui_backend:="$UI"
else
  exec ros2 launch rby1_vslam physical.launch.py enable_imu:=true start_rviz:=false "$@" ui_backend:="$UI" start_ui:=true
fi
