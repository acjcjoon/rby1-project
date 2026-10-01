#!/usr/bin/env bash
# Headless UPC launcher; forwards ROS launch arguments without a LAB host.
set -eo pipefail
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  printf '%s\n' \
    'Usage: bash run_mobile_base_web.sh [name:=value ...]' \
    'Example: bash run_mobile_base_web.sh web_port:=8080' \
    'Then open http://<UPC-IP>:8080 on a laptop on the same network.' \
    'Requires built rby1_driver, rby1_control and rby1_web packages.' \
    'Set RBY1_WORKSPACE_SETUP if the workspace is not ~/rby1_ros2_ws.'
  exit 0
fi
source "${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${RBY1_WORKSPACE_SETUP:-${HOME}/rby1_ros2_ws/install/setup.bash}"
set -u
exec ros2 launch rby1_web mobile_base_web.launch.py "$@"
