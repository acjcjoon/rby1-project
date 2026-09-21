# RB-Y1 Control

`rby1_control` is the only robot-facing application control node. It contains
no GUI and no planner frontend node.

## Runtime boundary

```text
/rby1/cmd_raw -> MobileBaseController -> /rby1/cmd_vel -> rby1_driver
planner control protocol -> RBY1ControlNode -> driver services/actions
RBY1 SDK -> PowerServoStateAdaptor -> RobotSession -> StateManager
```

`RBY1ControlNode` composes:

- `MobileBaseController`: raw-command input, stale-command watchdog, safety
  gate, and final `cmd_vel` publication;
- `ManipulatorController`: ordered joint feedback, Cartesian feedback, and
  pose math;
- `GripperController`: dual-gripper target and feedback tracking;
- `RobotSession` and `StateManager`: unified driver/SDK state ownership;
- `PowerServoStateAdaptor`: non-blocking SDK power/servo polling;
- `ControlTransport`: the versioned `std_msgs/String` planner protocol.

All publishers, subscriptions, clients, actions, and timers are created on the
single `/rby1/rby1_control` node. Helper classes are not ROS nodes.

## Launch

```bash
ros2 launch rby1_control control.launch.py \
  robot_address:=192.168.30.1:50051 robot_model:=m
```

If `robot_address` is empty, SDK power/servo feedback remains unknown and the
node logs that the adaptor is disabled. The SDK query itself runs on a worker
thread so a slow RPC cannot block the ROS executor.

The package deliberately has no `control_ui`, debug launch, frontend node, or
Qt dependency. The operator UI and all task-planning behavior live in
`rby1_planner`.
