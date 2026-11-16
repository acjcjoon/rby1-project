#!/usr/bin/env bash
# Detached-camera RGB reconstruction, isolated from the robot navigation domain.
set -Eeo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
usage() {
  printf '%s\n' \
    "Usage: $0 camera|mapping|rviz [additional arguments]" \
    "Run each command in a separate terminal. Default ROS domain: 11." \
    "Override with RBY1_HANDHELD_ROS_DOMAIN_ID; never mix with the robot TF stack." \
    "camera: native IR/depth 848x480@90 alternating, RGB8 640x480@30." \
    "mapping: splitter, full-6DoF camera VO, colored nvblox mesh (3 cm voxels)." \
    "rviz: colored mesh, camera pose/path and RGB image."
}
ACTION="${1:-}"
if [[ "$ACTION" == --help || "$ACTION" == -h ]]; then usage; exit 0; fi
if [[ "$ACTION" != camera && "$ACTION" != mapping && "$ACTION" != rviz ]]; then
  usage >&2; exit 2
fi
shift
DOMAIN="${RBY1_HANDHELD_ROS_DOMAIN_ID:-11}"
[[ "$DOMAIN" =~ ^[0-9]+$ ]] && ((DOMAIN <= 232)) || {
  printf 'RBY1_HANDHELD_ROS_DOMAIN_ID must be an integer from 0 through 232\n' >&2
  exit 2
}
export ROS_DOMAIN_ID="$DOMAIN" RBY1_ROS_DOMAIN_ID="$DOMAIN"
printf '[handheld] action=%s ROS_DOMAIN_ID=%s\n' "$ACTION" "$DOMAIN"
case "$ACTION" in
  camera)
    exec bash "$SCRIPT_DIR/run_d435i_host.sh" 0 --nvblox --color \
      depth_module.infra_profile:=848,480,90 \
      depth_module.depth_profile:=848,480,90 "$@"
    ;;
  mapping)
    CONTAINER="${RBY1_ISAAC_CONTAINER:-rby1-isaac-vslam}"
    [[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" == true ]] || {
      printf 'Isaac ROS container is not running: %s\n' "$CONTAINER" >&2; exit 1
    }
    DOCKER_TTY=(-i)
    if [[ -t 0 && -t 1 ]]; then DOCKER_TTY+=(-t); fi
    exec docker exec "${DOCKER_TTY[@]}" "$CONTAINER" env ROS_DOMAIN_ID="$DOMAIN" \
      bash -lc '
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        source /workspaces/isaac_ros-dev/install/setup.bash
        export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$(ros2 pkg prefix --share rby1_vslam)/config/fastdds_camera.xml}"
        [[ -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]] || {
          printf "Missing Fast DDS profile: %s\n" "$FASTRTPS_DEFAULT_PROFILES_FILE" >&2; exit 1
        }
        printf "[handheld] Fast DDS profile: %s\n" "$FASTRTPS_DEFAULT_PROFILES_FILE"
        exec ros2 launch rby1_vslam handheld_nvblox.launch.py "$@"
      ' bash "$@"
    ;;
  rviz)
    if [[ -r "$SCRIPT_DIR/config/handheld_nvblox.rviz" ]]; then
      export RBY1_NVBLOX_RVIZ_CONFIG="$SCRIPT_DIR/config/handheld_nvblox.rviz"
    else
      export RBY1_NVBLOX_RVIZ_CONFIG="$SCRIPT_DIR/../config/handheld_nvblox.rviz"
    fi
    exec bash "$SCRIPT_DIR/run_nvblox_rviz.sh" "$@"
    ;;
esac
