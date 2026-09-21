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

## Run

```bash
# Headless
ros2 launch rby1_planner planner.launch.py

# Operator UI
ros2 launch rby1_planner planner_ui.launch.py
```

Launching the planner is idle by default. A task begins only after an explicit
`PlannerNode.run_task()` call or an operator action in the planner UI.
