# RB-Y1 Planner

`rby1_planner` is an independent planning package. It does not import or
inherit Python classes from `rby1_control`.

`PlannerNode` subclasses only `rclpy.node.Node` and composes these local
components:

- `ControlClient` for the versioned control topic protocol;
- `CameraClient` for AprilTag/TF observations;
- `NavigationClient` for relative SE(2) goals;
- `PlannerTaskRunner` for non-blocking task execution;
- the planner-owned Qt operator UI.

The control wire value objects and serializer are implemented locally in
`backend_contract.py`, `control_commands.py`, and `control_protocol.py`. This
keeps installation and imports independent while preserving protocol version
1 compatibility with `rby1_control`.

## Navigation ownership

For a `move_to` step, the planner enables the robot stream through
`ControlClient`, asks `rby1_navigation` to execute the relative goal, waits for
its correlated result, then disables the stream. Navigation publishes only
`/rby1/cmd_raw`; it never publishes final `cmd_vel` or calls driver services.

## Camera-centred base alignment

Demo7 uses D435 tag observations in `base` and the current
`base <- d405_right_camera_center` TF to align the wrist camera horizontally.
Its subsequent D405 approach/contact, gripper close, and lift target the
hand passed as `arm` (required; the gripper is derived from it). Both wrists carry a
D405 (`camera_source` `d405_right` / `d405_left`); camera steps that omit
`camera_source` use the wrist camera of their arm.
The torso and arm stay fixed relative to base during navigation; Z is not
controlled by this base step.

`Task.move_base_to_detected_tag()` accepts two nonnegative metre values:
`threshold_x_minus` and `threshold_y_plus`. For demo7, define the measured error
as `tag_position - camera_center_position`, expressed in the **base axes**, not
the camera axes. Navigation is skipped when both conditions hold:

```text
-threshold_x_minus <= error_x <= 0
0 <= error_y <= threshold_y_plus
```

Outside that rectangle, only the excess is corrected, reaching the nearest
boundary as the calculated goal. For example, with thresholds `(0.05, 0.03)` m,
error `(-0.08, +0.02)` m requests base motion `(-0.03, 0)` m. There is no
tolerance in the opposite directions (+X and -Y). General calls with a nonzero
`relative_yaw` still execute the requested rotation, even if XY correction is
zero; threshold axes are the base axes at step start.

Both defaults are zero (exact XY alignment). To set the demo7 values in the UI
registry, use this entry in `task.py`'s `build_tasks()`:

```python
'object_handover_demo7': object_handover_demo7(
    threshold_x_minus=0.05,
    threshold_y_plus=0.03,
),
```

This is a single observation/goal calculation, not repeated visual servoing.
Navigation completion uses the navigation node's odometry tolerances; it does
not remeasure the tag to certify that the physical residual is inside the
rectangle. The following arm step uses a fresh D405 observation.

## On-demand tag detection

Each camera's `apriltag_node` + `depth_tag_tf_node` pair costs about one CPU
core, so `run_planner_ui.sh` starts only the camera mount TF publishers and
the planner (`vision_manager.py`) owns the detection pairs.

| `tag_detection_mode` | Behaviour |
|---|---|
| `auto` (default) | A Task step that reads a tag starts that camera's pair, waits for detections, and the pair stops once no later step of the Task uses that camera (also on failure or CANCEL) |
| `on` | All three pairs run permanently: use it to view tags in RViz or to measure offsets outside a Task |
| `off` | Nothing is started; camera steps fail with a clear message |

The mode is the `tag_detection_mode` parameter and can be changed while
running from **Tag detection** in the Scenario tab. Start-up adds roughly
1-2 s to the first tag step per camera. A pair that is already running outside
the planner (a manual `ros2 launch rby1_camera apriltag_vision.launch.py`) is
used as it is and never stopped.

## VSLAM tab

The planner UI has a **VSLAM** tab backed by `vslam_client.py`. It starts and
stops cuVSLAM inside the Isaac container (`rby1_vslam/run_isaac_vslam_docker.sh`,
`jetson.launch.py`) and uses the IR stereo pair that `run_planner_ui.sh` now
publishes from the same head D435 node as the AprilTag colour stream. nvblox
is not started.

| Group | Function |
|---|---|
| cuVSLAM Stack | Start in `mapping` (new map) or `localization` (saved map) mode, Stop |
| Status | VO tracking, camera-to-base conversion, localization state, `vslam_map -> base` pose |
| Localization | `Stop + correct at this point` (stops the base, waits for a converged correction), VO reference recovery, `Localize in saved map` with a `d435_link` pose hint |
| Map | Save the current map under a new name (`vslam_map_root_*` parameters) |
| Points | Save / delete named points in `rby1_vslam/data/rby1_vslam_waypoints.yaml`. Saving needs a completed correction at the current position |

| Live poses | Camera VO, Base VO and Localization as XYZ / roll / pitch / yaw with freshness, like the station UI |
| Navigation | `Start Nav2` / `Stop Nav2` (host, `rby1_vslam/run_navigation.sh`, no obstacle sensing), `Goto selected point -> stop -> VO correction`, `STOP Goto` |

### Goto (Nav2)

Goto follows the station UI: stop the gate, switch localization to wheel-only
transit, enable the gate, send `NavigateToPose` in `vslam_map`, and on arrival
disable the gate and request a station correction. It needs a running VSLAM
stack with a completed correction and a running Nav2.

Nav2 and `rby1_navigation` are installed side by side but never drive
together: Goto is refused while a Task runs, a Task cannot start during a
Goto, and Base tab jog is blocked during a Goto. The Nav2 gate is started with
`allowed_cmd_raw_publishers:=rby1_navigation_node`, so the planner's idle
odometry navigation node no longer counts as a competing `cmd_raw` publisher;
any other publisher still blocks the gate.

Base driving stays on the Base tab. The Scenario Task `offline_map_creation`
runs the whole pre-operation mapping session: head to the transit pitch, start
mapping, correct at the start position, then hand the base to the operator.
While that step runs the Base tab is enabled (Joints stays locked); drive to
each point, press `Stop + correct`, `Save Point`, and finally
`Finish mapping (continue Task)`, after which the Task saves the map as
`map_<date>_<time>`. Turn Stream on from the top panel before jogging. CANCEL
ends the Task without saving; the stack keeps running either way.

Task steps for `task.py`: `vslam_start(mode, map_name)`,
`vslam_station_correct()`, `vslam_operator_mapping()`, `vslam_save_map(name)`.

## Run

```bash
# Headless
ros2 launch rby1_planner planner.launch.py

# Operator UI
ros2 launch rby1_planner planner_ui.launch.py
```

Launching the planner is idle by default. A task begins only after an explicit
`PlannerNode.run_task()` call or an operator action in the planner UI.
