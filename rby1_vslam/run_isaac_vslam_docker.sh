#!/usr/bin/env bash
# Start cuVSLAM, camera-to-base pose adaptation and global TF in the bind-only container.
set -Eeo pipefail

CONTAINER="${RBY1_ISAAC_CONTAINER:-rby1-isaac-vslam}"
DOMAIN="${RBY1_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-10}}"
MODE="${1:-0}"
if (($#)); then
  shift
fi

case "${MODE,,}" in
  0|mode0|stereo) MODE=0 ;;
  1|mode1|stereo_imu|stereo-imu|vio) MODE=1 ;;
  *)
    printf 'Usage: %s [0|1] [additional jetson.launch.py arguments]\n' "$0" >&2
    exit 2
    ;;
esac

[[ "$DOMAIN" =~ ^[0-9]+$ ]] && ((DOMAIN <= 232)) || {
  printf 'RBY1_ROS_DOMAIN_ID must be an integer from 0 through 232\n' >&2
  exit 2
}

[[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" == true ]] || {
  printf 'Isaac ROS container is not running: %s\n' "$CONTAINER" >&2
  exit 1
}

printf '[isaac-docker] container=%s tracking_mode=%s ROS_DOMAIN_ID=%s\n' \
  "$CONTAINER" "$MODE" "$DOMAIN"
printf '[isaac-docker] camera remains on the host; starting cuVSLAM and base TF adaptation\n'

DOCKER_TTY=(-i)
if [[ -t 0 && -t 1 ]]; then
  DOCKER_TTY+=(-t)
fi

exec docker exec "${DOCKER_TTY[@]}" "$CONTAINER" \
  env ROS_DOMAIN_ID="$DOMAIN" \
  bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    mode="$1"
    shift
    exec ros2 launch rby1_vslam jetson.launch.py \
      start_camera:=false \
      start_localization_tf:=true \
      "tracking_mode:=${mode}" \
      "$@"
  ' bash "$MODE" "$@"
