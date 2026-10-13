#!/usr/bin/env bash
# cuVSLAM mode 1 dataset: D435i stereo IR + IMU.
set -Eeo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
printf '[mobile-base] mode1: stereo IR 640x360@30 + IMU\n'
exec "${BASH}" "${ROOT_DIR}/run_mobile_base.sh" "$@" \
  publish_all_sensors:=false \
  tracking_mode:=1 \
  infra_profile:=640,360,30
