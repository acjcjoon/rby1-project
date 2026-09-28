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
`base <- d405_camera_center` TF to align the wrist camera horizontally. Its
subsequent D405 approach/contact, gripper close, and lift still target `ee_left`.
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

## Run

```bash
# Headless
ros2 launch rby1_planner planner.launch.py

# Operator UI
ros2 launch rby1_planner planner_ui.launch.py
```

Launching the planner is idle by default. A task begins only after an explicit
`PlannerNode.run_task()` call or an operator action in the planner UI.
