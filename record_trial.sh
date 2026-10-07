#!/usr/bin/env bash
# Passive low-overhead diagnostics, independent of robot/UI launcher lifetime.
set -Eeo pipefail
ROLE=''; RUN_ID=''; OUTPUT="${HOME}/rby1_trials"; DOMAIN=''; WAYPOINTS=''; PORT=7447
FULL_BAG=false
NAV2_DEBUG=false
MAPPING_DEBUG=false
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
die() { echo "[record] $*" >&2; exit 2; }
usage() {
  cat <<'EOF'
Usage: bash record_trial.sh --role upc|lab --run-id NAME [options]
  --domain ID       Match this machine's running stack (defaults: UPC 0, LAB 85)
  --output DIR      Parent directory (default: ~/rby1_trials)
  --waypoints FILE  Also copy a custom waypoint YAML at start/end (UPC)
  --port PORT       VSLAM TCP port for socket snapshots (default: 7447)
  --full-bag        Record every topic, including raw images (high overhead)
  --nav2-debug      Add scan/costmaps/footprints for a short Nav2 trial
  --mapping-debug   UPC interactive marks: initial still -> mapping -> returned still
By default only low-bandwidth state/pose/TF/action topics are bagged.
Timing metadata and an automatic local bottleneck report are also saved.
Start BEFORE the operator stack. Stop robot via UI, then Ctrl+C here to finalize.
With --mapping-debug, start after VSLAM tracking is healthy and follow the prompts.
This script never sends robot commands. Topic recording is not service tracing.
LAB: run inside Isaac ROS; choose --output on a persistent mounted directory.
EOF
}
while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --full-bag) FULL_BAG=true; shift ;;
    --nav2-debug) NAV2_DEBUG=true; shift ;;
    --mapping-debug) MAPPING_DEBUG=true; shift ;;
    --role|--run-id|--output|--domain|--waypoints|--port)
      [[ $# -ge 2 && -n "$2" ]] || die "Missing value: $1"
      case "$1" in
        --role) ROLE="$2" ;; --run-id) RUN_ID="$2" ;; --output) OUTPUT="$2" ;;
        --domain) DOMAIN="$2" ;; --waypoints) WAYPOINTS="$2" ;; --port) PORT="$2" ;;
      esac
      shift 2 ;;
    *) die "Unknown argument: $1" ;;
  esac
done
[[ "$ROLE" == upc || "$ROLE" == lab ]] || die '--role upc|lab required'
[[ "$NAV2_DEBUG" == false || "$ROLE" == upc ]] || die '--nav2-debug is UPC-only'
[[ "$MAPPING_DEBUG" == false || "$ROLE" == upc ]] || die '--mapping-debug is UPC-only'
[[ "$MAPPING_DEBUG" == false || -t 0 ]] || die '--mapping-debug needs an interactive terminal'
[[ "$RUN_ID" =~ ^[a-zA-Z0-9_-]+$ ]] || die '--run-id requires letters/digits/_/-'
[[ "$PORT" =~ ^[0-9]+$ ]] && ((PORT >= 1 && PORT <= 65535)) || die 'Invalid port'
if [[ "$ROLE" == upc ]]; then
  ROS_SETUP="${RBY1_ROS_SETUP:-/opt/ros/humble/setup.bash}"
  WS_SETUP="${RBY1_WORKSPACE_SETUP:-${HOME}/rby1_ros2_ws/install/setup.bash}"
  DOMAIN="${DOMAIN:-${ROS_DOMAIN_ID:-0}}"
else
  ROS_SETUP="${RBY1_LAB_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
  WS_SETUP="${RBY1_WORKSPACE_SETUP:-${RBY1_LAB_WS:-/workspaces/isaac_ros-dev}/install/setup.bash}"
  DOMAIN="${DOMAIN:-${ROS_DOMAIN_ID:-85}}"
fi
[[ "$DOMAIN" =~ ^[0-9]+$ ]] || die 'Invalid domain'
[[ -r "$ROS_SETUP" && -r "$WS_SETUP" ]] || die 'ROS/workspace setup missing'
source "$ROS_SETUP"
source "$WS_SETUP"
set -u
export ROS_DOMAIN_ID="$DOMAIN"
command -v timeout >/dev/null || die 'GNU timeout is required'
command -v python3 >/dev/null || die 'python3 is required'
python3 -c 'import rclpy; import rosidl_runtime_py' || die 'ROS Python modules unavailable'
if [[ -f "$ROOT/rby1_vslam/scripts/timing_observer.py" ]]; then
  TIMING_SCRIPTS="$ROOT/rby1_vslam/scripts"
else
  # Also support placing record_trial.sh at the rby1_vslam package root on LAB.
  TIMING_SCRIPTS="$ROOT/scripts"
fi
TIMING_OBSERVER="$TIMING_SCRIPTS/timing_observer.py"
TIMING_ANALYZER="$TIMING_SCRIPTS/analyze_vslam_timing.py"
TIMING_PLOTTER="$TIMING_SCRIPTS/plot_vslam_timing.py"
[[ -f "$TIMING_OBSERVER" && -f "$TIMING_ANALYZER" ]] || \
  die 'timing_observer.py/analyze_vslam_timing.py missing under rby1_vslam/scripts'
mkdir -p -- "$OUTPUT"
RUN_DIR="$(mktemp -d "${OUTPUT%/}/${RUN_ID}_${ROLE}_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
CAPTURE_ID="$(basename -- "$RUN_DIR")"
if [[ "$ROLE" == upc && -z "$WAYPOINTS" ]]; then
  WAYPOINTS="$(python3 -c 'from ament_index_python.packages import get_package_share_directory; from rby1_vslam.waypoint_store import default_waypoints_path; print(default_waypoints_path(get_package_share_directory("rby1_vslam")))' 2>/dev/null || true)"
fi
{
  printf 'run_id=%s\nrole=%s\nROS_DOMAIN_ID=%s\nROS_DISTRO=%s\nwaypoints=%s\n' \
    "$RUN_ID" "$ROLE" "$DOMAIN" "${ROS_DISTRO:-unset}" "$WAYPOINTS"
  printf 'capture_id=%s\nVSLAM_TCP_PORT=%s\n' "$CAPTURE_ID" "$PORT"
  printf 'full_bag=%s\n' "$FULL_BAG"
  printf 'nav2_debug=%s\n' "$NAV2_DEBUG"
  printf 'mapping_debug=%s\n' "$MAPPING_DEBUG"
  git -C "$ROOT" rev-parse HEAD 2>/dev/null || true
  git -C "$ROOT" status --short 2>/dev/null || true
  date -u; uname -a; df -h "$RUN_DIR"
  if command -v chronyc >/dev/null; then timeout 2 chronyc tracking || true; fi
  if command -v timedatectl >/dev/null; then timeout 2 timedatectl timesync-status || true; fi
} > "$RUN_DIR/environment.txt" 2>&1
snapshot() {
  local phase="$1" node prefix waypoint_value
  timeout 5 ros2 topic list -t --include-hidden-topics \
    > "$RUN_DIR/topics_${phase}.txt" 2>&1 || true
  timeout 5 ros2 node list > "$RUN_DIR/nodes_${phase}.txt" 2>&1 || true
  mkdir -p "$RUN_DIR/params_${phase}"
  while IFS= read -r node; do
    [[ "$node" == /* ]] || continue
    timeout 3 ros2 param dump "$node" > "$RUN_DIR/params_${phase}/${node//\//_}.yaml" 2>&1 || true
  done < "$RUN_DIR/nodes_${phase}.txt"
  if [[ -n "$WAYPOINTS" && -f "$WAYPOINTS" ]]; then
    cp -- "$WAYPOINTS" "$RUN_DIR/waypoints_${phase}.yaml"
  fi
  if [[ "$ROLE" == upc ]]; then
    timeout 3 ros2 param get /rby1/vslam/waypoint_ui waypoints_file \
      > "$RUN_DIR/waypoints_parameter_${phase}.txt" 2>&1 || true
    waypoint_value="$(sed -n 's/^String value is: //p' \
      "$RUN_DIR/waypoints_parameter_${phase}.txt" | head -n 1)"
    if [[ "$waypoint_value" == /* && -f "$waypoint_value" ]]; then
      cp -- "$waypoint_value" "$RUN_DIR/waypoints_runtime_${phase}.yaml"
    fi
    prefix="$(timeout 3 ros2 pkg prefix rby1_vslam 2>/dev/null || true)"
    if [[ "$prefix" == /* && -d "$prefix/share/rby1_vslam/config" ]]; then
      mkdir -p "$RUN_DIR/runtime_config_${phase}"
      for name in navigation.yaml navigate_to_pose.xml navigate_through_poses.xml; do
        if [[ -f "$prefix/share/rby1_vslam/config/$name" ]]; then
          cp -- "$prefix/share/rby1_vslam/config/$name" \
            "$RUN_DIR/runtime_config_${phase}/$name"
        fi
      done
    fi
  fi
}
ready_snapshot() {
  local attempt nodes
  for ((attempt=0; attempt<60; attempt++)); do
    nodes="$(timeout 3 ros2 node list 2>/dev/null || true)"
    if grep -qx '/rby1/vslam/nav2/planner_server' <<< "$nodes" \
        && grep -qx '/rby1/vslam/nav2/controller_server' <<< "$nodes"; then
      snapshot ready
      : > "$RUN_DIR/params_ready_complete"
      echo '[record] Nav2 ready parameter snapshot complete; the goal may be sent.'
      return
    fi
    sleep 1
  done
  echo 'Nav2 planner/controller did not appear within 60 seconds.' \
    > "$RUN_DIR/params_ready_unavailable.txt"
}
BAG_PID=''; OBSERVER_PID=''; SAMPLE_PID=''; SNAPSHOT_PID=''; READY_PID=''
wait_bounded() {
  local pid="$1" seconds="$2" label="$3"
  [[ -n "$pid" ]] || return
  if ! timeout "${seconds}s" tail --pid="$pid" -f /dev/null >/dev/null 2>&1; then
    echo "[record] WARNING: $label did not exit within ${seconds}s; terminating it." >&2
    kill -TERM "$pid" 2>/dev/null || true
    if ! timeout 5s tail --pid="$pid" -f /dev/null >/dev/null 2>&1; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  fi
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  # SIGINT finalizes the bag; recording never stops the robot.
  [[ -z "$BAG_PID" ]] || kill -INT "$BAG_PID" 2>/dev/null
  [[ -z "$OBSERVER_PID" ]] || kill -INT "$OBSERVER_PID" 2>/dev/null
  [[ -z "$SAMPLE_PID" ]] || kill -TERM "$SAMPLE_PID" 2>/dev/null
  [[ -z "$READY_PID" ]] || kill -TERM "$READY_PID" 2>/dev/null
  [[ -z "$SNAPSHOT_PID" ]] || kill -TERM "$SNAPSHOT_PID" 2>/dev/null
  wait_bounded "$BAG_PID" 30 'rosbag recorder'
  wait_bounded "$OBSERVER_PID" 10 'timing observer'
  wait_bounded "$SAMPLE_PID" 5 'system sampler'
  wait_bounded "$READY_PID" 5 'ready parameter snapshot'
  wait_bounded "$SNAPSHOT_PID" 5 'start parameter snapshot'
  snapshot end
  if [[ -f "$RUN_DIR/params_ready_complete" ]]; then
    diff -ru "$RUN_DIR/params_ready" "$RUN_DIR/params_end" \
      > "$RUN_DIR/params_ready_to_end.diff" 2>&1 || true
  elif [[ -f "$RUN_DIR/params_start_complete" ]]; then
    if [[ -d "$RUN_DIR/params_ready" ]]; then
      echo 'Ready snapshot was interrupted; using the start snapshot as the diff baseline.' \
        > "$RUN_DIR/params_ready_incomplete.txt"
    fi
    diff -ru "$RUN_DIR/params_start" "$RUN_DIR/params_end" \
      > "$RUN_DIR/params_start_to_end.diff" 2>&1 || true
  else
    echo 'Both initial parameter snapshots were interrupted; inspect params_end directly.' \
      > "$RUN_DIR/params_baseline_unavailable.txt"
  fi
  timeout 15 ros2 bag info "$RUN_DIR/bag" > "$RUN_DIR/bag_info.txt" 2>&1 || true
  if python3 "$TIMING_ANALYZER" "$RUN_DIR" --output-dir "$RUN_DIR" \
      > "$RUN_DIR/analysis.log" 2>&1; then
    echo "[record] Timing report: $RUN_DIR/timing_report.txt"
    if [[ "$MAPPING_DEBUG" == true && -f "$TIMING_PLOTTER" ]]; then
      if python3 -c 'import matplotlib' >/dev/null 2>&1; then
        mkdir -p "$RUN_DIR/plots"
        if python3 "$TIMING_PLOTTER" --report "$RUN_DIR/timing_report.json" \
            --output-dir "$RUN_DIR/plots" > "$RUN_DIR/plotting.log" 2>&1; then
          echo "[record] Mapping/jitter plots: $RUN_DIR/plots"
        else
          echo "[record] WARNING: plot generation failed; inspect $RUN_DIR/plotting.log" >&2
        fi
      else
        echo '[record] matplotlib unavailable; report/CSVs were saved but PNG plots were skipped.' >&2
        echo "[record] Generate later: python3 $TIMING_PLOTTER --report $RUN_DIR/timing_report.json --output-dir $RUN_DIR/plots" >&2
      fi
    fi
  else
    echo "[record] WARNING: timing analysis failed; inspect $RUN_DIR/analysis.log" >&2
  fi
  echo "[record] Saved: $RUN_DIR (check timing_report.txt, recorder.log and bag_info.txt)"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Reset SIGINT ignored by noninteractive Bash background children.  Raw image
# recording is opt-in: serializing another copy of both 30 Hz images on LAB can
# push the Python bridge behind and trip its acquisition-age safety check.
BAG_ARGS=(--include-hidden-topics -o "$RUN_DIR/bag")
if [[ "$FULL_BAG" == true ]]; then
  BAG_ARGS=(-a "${BAG_ARGS[@]}")
else
  BAG_ARGS+=(
    /rosout /tf /tf_static /diagnostics
    /parameter_events
    /d435/d435/infra1/camera_info /d435/d435/infra2/camera_info
    /d435/d435/imu
    /rby1/vslam/bridge_status /rby1/vslam/timing
    /rby1/vslam/camera_odometry /rby1/vslam/camera_slam_odometry
  )
  if [[ "$ROLE" == upc ]]; then
    BAG_ARGS+=(
      /rby1/vslam/odom /rby1/vslam/slam_odom
      /rby1/odom /rby1/robot_state
      /rby1/control/command /rby1/control/event
      /rby1/vslam/localization_status /rby1/vslam/navigation_status
      /rby1/vslam/navigation_event
      /rby1/vslam/nav2_cmd_vel /rby1/cmd_raw /rby1/cmd_vel
      /rby1/vslam/nav2/cmd_vel_nav
      /rby1/vslam/nav2/navigate_to_pose/_action/status
      /rby1/vslam/nav2/navigate_to_pose/_action/feedback
      /rby1/vslam/nav2/navigate_through_poses/_action/status
      /rby1/vslam/nav2/navigate_through_poses/_action/feedback
    )
    if [[ "$NAV2_DEBUG" == true ]]; then
      BAG_ARGS+=(
        /scan
        /rby1/vslam/nav2/compute_path_to_pose/_action/status
        /rby1/vslam/nav2/follow_path/_action/status
        /rby1/vslam/nav2/follow_path/_action/feedback
        /rby1/vslam/nav2/plan
        /rby1/vslam/nav2/received_global_plan
        /rby1/vslam/nav2/transformed_global_plan
        /rby1/vslam/nav2/local_plan
        /rby1/vslam/nav2/local_costmap/costmap_raw
        /rby1/vslam/nav2/global_costmap/costmap_raw
        /rby1/vslam/nav2/local_costmap/published_footprint
        /rby1/vslam/nav2/global_costmap/published_footprint
      )
    fi
  else
    BAG_ARGS+=(/visual_slam/status)
  fi
fi
python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1],sys.argv[1:])' \
  ros2 bag record "${BAG_ARGS[@]}" > "$RUN_DIR/recorder.log" 2>&1 &
BAG_PID=$!
OBSERVER_ARGS=(--role "$ROLE" --output "$RUN_DIR/events.jsonl" --capture-id "$CAPTURE_ID")
if [[ "$NAV2_DEBUG" == true ]]; then
  OBSERVER_ARGS+=(--nav2-debug)
fi
python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1],sys.argv[1:])' \
  python3 "$TIMING_OBSERVER" "${OBSERVER_ARGS[@]}" > "$RUN_DIR/observer.log" 2>&1 &
OBSERVER_PID=$!
(snapshot start; : > "$RUN_DIR/params_start_complete") & SNAPSHOT_PID=$!
if [[ "$ROLE" == upc && "$MAPPING_DEBUG" == false ]]; then
  ready_snapshot & READY_PID=$!
fi
(
  while true; do
    python3 -c 'import json,time; print(json.dumps({"wall_ns":time.time_ns(),"monotonic_ns":time.monotonic_ns()}))'
    date -u; uptime; df -h "$RUN_DIR"
    head -n 1 /proc/stat
    awk '/^(MemTotal|MemAvailable|SwapTotal|SwapFree):/ {print}' /proc/meminfo
    cat /proc/net/dev
    if command -v chronyc >/dev/null; then timeout 2 chronyc tracking || true; fi
    if command -v timedatectl >/dev/null; then timeout 2 timedatectl timesync-status || true; fi
    if command -v ss >/dev/null; then timeout 2 ss -tinp "( sport = :$PORT or dport = :$PORT )" || true; fi
    ps -eo pid,pcpu,pmem,comm,args --sort=-pcpu | head -n 25 || true
    if command -v nvidia-smi >/dev/null; then
      timeout 2 nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used \
        --format=csv,noheader || true
    fi
    sleep 5
  done
) > "$RUN_DIR/system_samples.txt" 2>&1 & SAMPLE_PID=$!
echo "[record] Recorder launched: $RUN_DIR"
echo "[record] role=$ROLE ROS_DOMAIN_ID=$ROS_DOMAIN_ID TCP_PORT=$PORT"
echo '[record] Confirm recorder.log and observer.log. No motion commands are sent.'
write_marker() {
  local phase="$1"
  python3 -c 'import json,sys,time; record={"event":"phase_marker","phase":sys.argv[2],"wall_ns":time.time_ns(),"monotonic_ns":time.monotonic_ns()}; stream=open(sys.argv[1],"a",encoding="utf-8"); stream.write(json.dumps(record,separators=(",",":"))+"\n"); stream.close()' \
    "$RUN_DIR/markers.jsonl" "$phase"
  echo "[record] phase=$phase"
}
ensure_collectors_alive() {
  kill -0 "$BAG_PID" 2>/dev/null || die 'rosbag recorder stopped; inspect recorder.log'
  kill -0 "$OBSERVER_PID" 2>/dev/null || die 'timing observer stopped; inspect observer.log'
}
if [[ "$MAPPING_DEBUG" == true ]]; then
  echo '[record] Waiting for the initial graph/parameter snapshot before jitter measurement...'
  wait "$SNAPSHOT_PID"
  SNAPSHOT_PID=''
  ensure_collectors_alive
  write_marker initial_stationary
  echo '[record] Keep the robot completely still for at least 10 seconds.'
  read -r -p '[record] Press Enter immediately before starting mapping motion... ' _
  ensure_collectors_alive
  write_marker mapping_motion
  read -r -p '[record] Return to the start pose, stop fully, then press Enter... ' _
  ensure_collectors_alive
  write_marker returned_stationary
  echo '[record] Keep the robot completely still for at least 10 more seconds.'
  read -r -p '[record] Press Enter to finish the capture... ' _
  ensure_collectors_alive
  write_marker capture_end
  exit 0
fi
echo '[record] Keep recording through the stop + 10 seconds.'
echo '[record] Stop the robot in the UI first, then Ctrl+C here.'
wait "$BAG_PID"
