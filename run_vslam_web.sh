#!/usr/bin/env bash
# Full UPC VSLAM/Nav2 stack with browser operator. LAB host remains necessary
# for visual SLAM, while run_mobile_base_web.sh has no LAB/TCP dependency.
set -eo pipefail
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  printf '%s\n' \
    'Usage: bash run_vslam_web.sh lab_host:=<LAB-IP> [name:=value ...]' \
    'Starts the physical VSLAM/Nav2 stack and browser operator on UPC.' \
    'Open http://<UPC-IP>:8080 on your laptop.' \
    'For manual driving without LAB/TCP, use run_mobile_base_web.sh.'
  exit 0
fi
source "${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${RBY1_WORKSPACE_SETUP:-${HOME}/rby1_ros2_ws/install/setup.bash}"
set -u
exec ros2 launch rby1_vslam physical.launch.py ui_backend:=web start_rviz:=false "$@"
