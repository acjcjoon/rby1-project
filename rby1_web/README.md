# UPC web operator for rby1_vslam

Run the robot driver, existing `rby1_control` safety backend and HTTP operator
on UPC. A laptop needs only a browser and network access to UPC; no ROS/DDS,
LAB PC, SLAM, camera, Nav2, RViz or desktop display is required.
`rby1_vslam` owns SLAM, localization, Nav2 and the velocity gate; `rby1_web`
owns browser operation. The same server has two launch modes: standalone
manual driving and VSLAM operation with named waypoints/navigation.
The VSLAM Qt operator is now optional (`ui_backend:=qt`) while the web UI
is verified on hardware. The separate manipulation/scenario planner UI is
outside this VSLAM operator's scope.

## Two independent launch paths

**Manual driving with no LAB/TCP/VSLAM connection:**

```bash
bash run_mobile_base_web.sh
```

Only driver + `rby1_control` + web server run. No `lab_host` argument, bridge
subscription, tracking check, Nav2 or camera is involved. Robot RPC connection
and the existing robot-state safety checks still apply. A missing LAB PC or
disconnected visual SLAM does not prevent manual driving.

**Full VSLAM/Nav2 browser operation:**

```bash
bash run_vslam_web.sh lab_host:=192.168.30.50
# Equivalent: ros2 launch rby1_vslam physical.launch.py lab_host:=192.168.30.50
```

LAB still provides visual SLAM for localization/navigation. This default
launch opens no Qt/RViz window; use `start_rviz:=true` for optional RViz.
To attach only the web operator to an existing driver/control/VSLAM stack:

```bash
ros2 launch rby1_web operator_web.launch.py
```

Do not run both web launch modes or duplicate driver/control processes in
the same namespace. Stop the previous stack before changing launch modes.

The full UI provides TCP/tracking/localization/gate status, current map pose,
occupancy map (when supplied), Nav2 path, current-pose waypoint save, reload,
delete, navigation to a selected waypoint, and navigation cancel/STOP.
Without an occupancy map, it still draws pose/waypoints/path in `vslam_map`.
It uses the existing waypoint YAML file; no migration is required.

Manual driving in VSLAM mode does not require healthy TCP/tracking. It first
disables/cancels Nav2 and waits for the gate to relinquish `cmd_raw`. Only
navigation uses the existing gate's SLAM/localization checks. If the gate
process itself stops reporting while running, restart with the standalone
manual launch rather than racing an unknown navigation producer.

Navigation requires the originating browser to poll state. Closing it or
losing the connection cancels navigation after 1.5 s; browser blur/pagehide
also sends STOP. Any browser may stop/cancel. Goal-enable and acceptance
callbacks are fenced so a canceled goal cannot start from a late response.
Map saving/localization on LAB remains in the existing `rby1_vslam map_tool`.

## Build on UPC (ROS 2 Humble)

Use the workspace containing this repository and the existing `rby1-ros2`
driver source, with its normal ROS/SDK dependencies installed:

```bash
source /opt/ros/humble/setup.bash
cd ~/rby1_ros2_ws
colcon build --symlink-install --packages-up-to rby1_web
source install/setup.bash
ros2 launch rby1_web mobile_base_web.launch.py
```

Or, after building, from the project directory:

```bash
bash run_mobile_base_web.sh
```

The script sources `~/rby1_ros2_ws/install/setup.bash` by default. Override
`RBY1_WORKSPACE_SETUP` / `RBY1_ROS_SETUP` for another installation.
Default robot RPC address: `192.168.30.1:50051`, model: `m`.

```bash
ros2 launch rby1_web mobile_base_web.launch.py \
  robot_address:=192.168.30.1:50051 robot_model:=m web_port:=8080
```

Find UPC's reachable LAN/Wi-Fi IP with `hostname -I`, and open
`http://<UPC-IP>:8080` on the laptop. The laptop connects to UPC's network
interface, not the robot RPC IP. Bind is `0.0.0.0`; `web_host:=<UPC-LAN-IP>`
can restrict the listening interface. UPC's firewall must allow this TCP
port from the laptop. No ROS network configuration is needed on the laptop.

## Operating

Startup does not automatically power, enable servos or start the stream.
Use the web buttons in order, waiting for successful feedback:

1. Power ON.
2. Servo ON.
3. Control Enable.
4. Stream ON.
5. Hold a forward/back/turn button; release to stop. Space or STOP also stops.

Power and servo buttons use the driver's existing `all` target, so they
apply to the entire robot, including the body joints. No arm/gripper motion
commands or gripper homing are exposed. STOP sends base zero velocity;
it is not a hardware emergency stop. Stream OFF and Control Disable are
available separately. Backend events show asynchronous service success/failure;
an HTTP `202` means queued, not that the hardware operation succeeded.

The default server limits are 0.2 m/s and 0.4 rad/s, with initial sliders
at 0.1 m/s and 0.2 rad/s. Change `max_linear_speed` / `max_angular_speed`
only when appropriate for the physical robot. Forward/back, lateral movement
and turn controls preserve the existing holonomic base interface. Combined
linear velocity is bounded by the configured limit. VSLAM operator mode uses
the former Qt mapping limits: 0.08 m/s and 0.2 rad/s, initial 0.02 / 0.08.

Only one browser may hold motion control at a time. Any browser may STOP.
Web input expires after 0.3 s without a new command; stale backend feedback
also drops motion. The existing backend retains its 0.35 s watchdog and
robot-state safety checks. Browser blur, page hiding and pointer cancellation
send STOP. Terminating the launch closes the whole stack.

See [INTERFACES.md](INTERFACES.md) for the fixed integration boundary and
main-branch merge considerations. ROS-free mailbox/HTTP tests run with:

```bash
python3 -m unittest discover -s rby1_web/test -v
```

Run this launch on its own, after stopping the planner/navigation/physical
stack. Do not launch a second driver/control node or another velocity producer
in the same namespace. If the driver is already running alone, pass
`start_driver:=false` (the control backend is still started).

## Network boundary

This minimal HTTP UI is for a trusted lab LAN. Anyone who can access the
page can operate it; there is no user login or TLS. It rejects cross-origin
browser command submissions using a per-server page token and custom header.
Do not expose port 8080 to the public Internet. Remote access can use an
SSH tunnel with `web_host:=127.0.0.1`:

```bash
ssh -L 8080:127.0.0.1:8080 <user>@<UPC-IP>
# Open http://localhost:8080 on the laptop.
```
