#!/usr/bin/env bash
# Run inside the Jetson's Isaac ROS 3.2 environment after building this package.
set -Eeo pipefail

ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${RBY1_WORKSPACE_SETUP:-${ISAAC_ROS_WS:-${HOME}/rby1_ros2_ws}/install/setup.bash}"
[[ -r "$ROS_SETUP" ]] || { printf 'Missing ROS setup: %s\n' "$ROS_SETUP" >&2; exit 1; }
[[ -r "$WORKSPACE_SETUP" ]] || { printf 'Missing workspace setup: %s\n' "$WORKSPACE_SETUP" >&2; exit 1; }
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u
export ROS_DOMAIN_ID="${RBY1_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
[[ "$ROS_DISTRO" == humble ]] || { printf 'Isaac ROS 3.2 requires Humble\n' >&2; exit 1; }
exec ros2 launch rby1_vslam physical.launch.py "$@"
