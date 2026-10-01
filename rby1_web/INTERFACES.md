# Operator integration contract (v1)

`rby1_web` is a client of the existing project interfaces. It does not own
SLAM, localization, navigation planning, driver services or final cmd_vel.
The dependency points from `rby1_web` to `rby1_vslam` and `rby1_control`.
Keep VSLAM/core modules free of web/Qt imports; this also keeps LAB deployment
independent of the web package. VSLAM launch files optionally select the
web client at runtime; build `rby1_web` explicitly on UPC.

## ROS boundary

| Interface | Existing type | Web usage |
|---|---|---|
| `/rby1/control/command` | `std_msgs/String`, control schema v1 | Existing `encode_message`; velocity, stop, power, servo, manager, stream |
| `/rby1/control/state` | `std_msgs/String`, control schema v1 | `snapshot` and freshness |
| `/rby1/control/event`, `/rby1/control/response` | `std_msgs/String`, control schema v1 | Hardware operation feedback |
| `/rby1/vslam/bridge_status` | `std_msgs/String` JSON | TCP/tracking status |
| `/rby1/vslam/localization_status` | `std_msgs/String` JSON | Global pose health |
| `/rby1/vslam/navigation_status` | `std_msgs/String` JSON | Gate ownership and cancellation state |
| `/rby1/vslam/enable` | `std_srvs/SetBool` | Arm the existing guarded navigation path |
| `/rby1/vslam/cancel` | `std_srvs/Trigger` | Disable forwarding and cancel all navigation goals |
| `/rby1/vslam/nav2/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Existing Nav2 goal/feedback/result |
| `/rby1/vslam/nav2/map` | `nav_msgs/OccupancyGrid` | Optional map, transient-local QoS |
| `/rby1/vslam/nav2/plan` | `nav_msgs/Path` | Map-frame global path |
| `vslam_map → base` | TF | Fresh current pose for display and waypoint save |

Topic/action/service/frame/file names are parameters, exposed by
`operator_web.launch.py`. `rby1_control` remains the only final `/rby1/cmd_vel`
publisher. HTTP handlers never touch ROS entities, actions or waypoint files;
they enqueue bounded commands, which the ROS executor dispatches.

Named poses reuse `rby1_vslam.waypoint_store`, its schema v1 and atomic saving.
`x/y` are meters, `yaw` is radians in `vslam_map`; degrees are display-only.
Manual base commands use `vx/vy` m/s and `wz` rad/s, preserving holonomic motion.

## HTTP boundary

| Endpoint | Payload |
|---|---|
| `GET /` | Operator page and per-server anti-cross-origin token |
| `GET /api/state` | `schema_version:1`, `mode`, `connected`, `snapshot`, `operator`, `events`, `limits` |
| `GET /api/map` | Optional downsampled occupancy grid: dimensions, resolution, `[x,y,yaw]` origin, cells |
| `POST /api/command` | `schema_version:1`, `session`, increasing `seq`, `operation`, `arguments`, optional hold `gesture` |

POST requires `X-RBY1-Token` from the page. Velocity requests require a hold
gesture and fresh backend state; no VSLAM/TCP health condition is applied.
Allowlisted operations: `set_velocity`, `stop`, `power_on`, `servo_on`,
`enable`, `disable`, `stream_on`, `stream_off`; VSLAM mode additionally allows
`waypoint_save`, `waypoint_delete`, `waypoint_reload`, `navigate`, `nav_cancel`.
Named operations take `arguments.name`. No browser-selected filesystem path,
shell command, generic ROS method or manipulation command is accepted.

`202 {queued:true}` acknowledges the mailbox, not hardware success. State,
Nav2 results and event messages report operation outcomes. State polling sends
`X-RBY1-Session` for the navigation browser heartbeat. A different browser
cannot keep the originating browser's navigation lease alive.

## Merging into main

Keep the current ROS names/control schema/waypoint schema unchanged. Adapt
upstream changes in `vslam_adapter.py` and the control-topic parameters, rather
than forking a backend in the web package. Include `rby1_web` in the UPC build
target and install its static HTML as package data. No frontend npm toolchain
or new message-generation package is required.

The web screen covers the VSLAM Qt operator's workflow. The Qt source and
`operator_qt.launch.py` remain a temporary fallback until real-robot web
operation is verified; they can then be removed with the PyQt dependency.
Do not remove `rby1_planner`'s manipulation/scenario UI as part of this migration.
