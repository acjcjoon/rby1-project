# VSLAM waypoint data

The operator UI creates `rby1_vslam_waypoints.yaml` here on the first save
when the package is built with `colcon build --symlink-install`.
For a regular installation it uses the installed package's
`share/rby1_vslam/data` directory instead.

The YAML contains `schema_version`, `frame_id` and named `x`, `y`, `yaw` poses.
Positions are in metres; yaw is in radians. These poses are separate from
the cuVSLAM map and the Nav2 occupancy map.

An explicit `waypoints_file` parameter overrides this default. Existing files
under `~/rby1_maps` are not automatically copied or overwritten.
