#!/usr/bin/env bash
# Run only inside the existing Isaac ROS 3.2 container, after explicit approval.
# No apt update/upgrade and no changes to the host camera/CUDA installations.
set -euo pipefail

packages=(
  ros-humble-nvblox-ros=3.2.5-0jammy
  ros-humble-nvblox-nav2=3.2.5-0jammy
  ros-humble-nvblox-rviz-plugin=3.2.5-0jammy
  ros-humble-nvblox-msgs=3.2.5-0jammy
  ros-humble-nvblox-ros-common=3.2.5-0jammy
  ros-humble-nvblox-ros-python-utils=3.2.5-0jammy
  ros-humble-realsense2-camera-msgs=4.58.4-1jammy.20260907.223031
)

if [[ "${1:-}" != --install-approved ]]; then
  exec apt-get --simulate --no-upgrade --no-remove --no-install-recommends \
    install "${packages[@]}"
fi

[[ -f /.dockerenv && -d /workspaces/isaac_ros-dev ]] || {
  printf 'This installer is restricted to the Isaac ROS container.\n' >&2
  exit 1
}
exec sudo apt-get --yes --no-upgrade --no-remove --no-install-recommends \
  install "${packages[@]}"
