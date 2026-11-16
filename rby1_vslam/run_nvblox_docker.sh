#!/usr/bin/env bash
# Run the mapper beside the existing VSLAM container process. No TF publisher.
set -Eeo pipefail
CONTAINER="${RBY1_ISAAC_CONTAINER:-rby1-isaac-vslam}"
DOMAIN="${RBY1_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-10}}"
[[ "$DOMAIN" =~ ^[0-9]+$ ]] && ((DOMAIN <= 232)) || {
  printf 'ROS_DOMAIN_ID must be an integer from 0 through 232\n' >&2
  exit 2
}
[[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER")" == true ]] || {
  printf 'Isaac ROS container is not running: %s\n' "$CONTAINER" >&2
  exit 1
}
DOCKER_TTY=(-i)
if [[ -t 0 && -t 1 ]]; then DOCKER_TTY+=(-t); fi
exec docker exec "${DOCKER_TTY[@]}" "$CONTAINER" env ROS_DOMAIN_ID="$DOMAIN" \
  bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$(ros2 pkg prefix --share rby1_vslam)/config/fastdds_camera.xml}"
    [[ -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]] || {
      printf "Missing Fast DDS profile: %s\n" "$FASTRTPS_DEFAULT_PROFILES_FILE" >&2
      exit 1
    }
    printf "[nvblox-docker] Fast DDS profile: %s\n" "$FASTRTPS_DEFAULT_PROFILES_FILE"
    exec ros2 launch rby1_vslam nvblox.launch.py "$@"
  ' bash "$@"
