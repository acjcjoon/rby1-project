#!/usr/bin/env bash
# RViz runs on the host; the existing container need not expose the X11 socket.
set -Eeo pipefail
source "${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${RBY1_WORKSPACE_SETUP:-${HOME}/rby1_ros2_ws/install/setup.bash}"
export ROS_DOMAIN_ID="${RBY1_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-10}}"
SHARE="$(ros2 pkg prefix --share rby1_vslam)"

# nvblox 3.2 creates its voxel material on the first mesh message as well.
# Ubuntu's Humble RViz can omit GLSL150 resources despite GPU support, which
# otherwise aborts RViz with "Unable to locate ... rviz/glsl150/box.geom".
# Use nvblox's built-in fallback in that case. Meshes, ESDF PointCloud2 and
# costmaps retain their normal rendering; only voxel-layer boxes become points.
RVIZ_RENDERING_SHARE="$(ros2 pkg prefix --share rviz_rendering)"
if [[ ! -r "$RVIZ_RENDERING_SHARE/ogre_media/materials/glsl150/box.geom" ||
      ! -d "$RVIZ_RENDERING_SHARE/ogre_media/materials/scripts150" ]]; then
  export ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL=1
  echo "[nvblox RViz] GLSL150 box shader resources are missing; using the supported material fallback."
fi

RVIZ_CONFIG="${RBY1_NVBLOX_RVIZ_CONFIG:-$SHARE/config/nvblox.rviz}"
[[ -r "$RVIZ_CONFIG" ]] || { printf 'Missing RViz config: %s\n' "$RVIZ_CONFIG" >&2; exit 1; }
exec rviz2 -d "$RVIZ_CONFIG" "$@"
