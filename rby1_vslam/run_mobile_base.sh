#!/usr/bin/env bash
# Host-side RBY1 driver, control and manual base UI. Camera is optional.
set -Eeo pipefail

ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${RBY1_WORKSPACE_SETUP:-${ISAAC_ROS_WS:-${HOME}/rby1_ros2_ws}/install/setup.bash}"
[[ -r "$ROS_SETUP" ]] || { printf 'Missing ROS setup: %s\n' "$ROS_SETUP" >&2; exit 1; }
[[ -r "$WORKSPACE_SETUP" ]] || { printf 'Missing workspace setup: %s\n' "$WORKSPACE_SETUP" >&2; exit 1; }
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u
export ROS_DOMAIN_ID="${RBY1_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-10}}"
exec ros2 launch rby1_vslam mobile_base.launch.py "$@"
