#!/usr/bin/env bash
# RGB-D dataset only; Isaac ROS 3.2 does not support RGB-D VSLAM.
set -Eeo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
printf '[mobile-base] mode2: RGB-D 640x480@30 with aligned depth\n'
exec "${BASH}" "${ROOT_DIR}/run_mobile_base.sh" "$@" \
  publish_all_sensors:=false \
  tracking_mode:=2 \
  color_profile:=640,480,30 \
  depth_profile:=640,480,30
