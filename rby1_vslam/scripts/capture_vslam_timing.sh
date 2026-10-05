#!/usr/bin/env bash
# Passive UPC/LAB capture: ROS metadata + small-topic bag + host/TCP snapshots.
set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROLE=""
OUTPUT_ROOT="${HOME}/rby1_timing"
RUN_LABEL="trial"
PORT=7447
INTERFACE=any
PCAP=false

usage() {
  cat <<'EOF'
Usage: bash capture_vslam_timing.sh --role upc|lab [options]
  --output DIR        Output parent (default: ~/rby1_timing)
  --run-id LABEL      Same trial label on both PCs (letters/digits/_/-)
  --port PORT         VSLAM TCP port (default: 7447)
  --pcap              Also capture TCP headers with tcpdump (no sudo added)
  --interface IFACE   tcpdump interface (default: any; prefer actual wired NIC)
  -h, --help

Source ROS/workspace and match the local pipeline ROS_DOMAIN_ID first.
Run on UPC and inside the Isaac ROS container on LAB before the trial.
Ctrl+C stops these collectors only. No motion/enable/cancel commands are sent.
EOF
}
die() { echo "[timing] ERROR: $*" >&2; exit 1; }
while (($#)); do
  case "$1" in
    --role|--output|--run-id|--port|--interface)
      [[ $# -ge 2 && -n "$2" ]] || die "Missing value for $1"
      case "$1" in
        --role) ROLE="$2" ;;
        --output) OUTPUT_ROOT="$2" ;;
        --run-id) RUN_LABEL="$2" ;;
        --port) PORT="$2" ;;
        --interface) INTERFACE="$2" ;;
      esac
      shift 2 ;;
    --pcap) PCAP=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown option: $1" ;;
  esac
done
[[ "$ROLE" == upc || "$ROLE" == lab ]] || die "--role upc|lab is required"
[[ "$RUN_LABEL" =~ ^[a-zA-Z0-9_-]+$ ]] || die "Invalid --run-id"
[[ "$PORT" =~ ^[0-9]+$ ]] && ((PORT >= 1 && PORT <= 65535)) || die "Invalid port"
command -v ros2 >/dev/null || die "Source ROS and workspace first"
command -v python3 >/dev/null || die "python3 is required"
command -v timeout >/dev/null || die "GNU timeout is required"
python3 -c 'import rclpy; import rosidl_runtime_py' || die "ROS Python modules unavailable"
[[ "$PCAP" == false ]] || command -v tcpdump >/dev/null || die "--pcap needs tcpdump"
[[ -f "$SCRIPT_DIR/analyze_vslam_timing.py" ]] || die "Missing analyze_vslam_timing.py next to this script"

mkdir -p -- "$OUTPUT_ROOT"
RUN_DIR="$(mktemp -d "${OUTPUT_ROOT}/${RUN_LABEL}_${ROLE}_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
CAPTURE_ID="$(basename -- "$RUN_DIR")"
PIDS=()
NAMES=()
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  # Allow JSON flush and rosbag/pcap finalization, then bound cleanup time.
  for ((attempt=0; attempt<50; attempt++)); do
    alive=false
    for pid in "${PIDS[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then alive=true; fi
    done
    [[ "$alive" == true ]] || break
    sleep 0.1
  done
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
  if python3 "$SCRIPT_DIR/analyze_vslam_timing.py" "$RUN_DIR" \
      --output-dir "$RUN_DIR" > "$RUN_DIR/analysis.log" 2>&1; then
    echo "[timing] Report: $RUN_DIR/timing_report.txt"
  else
    echo "[timing] WARNING: automatic analysis failed; inspect $RUN_DIR/analysis.log" >&2
  fi
  echo "[timing] Saved: $RUN_DIR"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Background children inherit ignored SIGINT in noninteractive Bash; restore
# SIGINT with Python before exec so Ctrl+C cleanup flushes bag/JSONL/pcap.
start_child() {
  local label="$1"; shift
  python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1],sys.argv[1:])' \
    "$@" > "$RUN_DIR/${label}.log" 2>&1 &
  PIDS+=("$!"); NAMES+=("$label")
}

snapshot_clock() {
  date -u '+utc=%Y-%m-%dT%H:%M:%S.%NZ'
  if command -v chronyc >/dev/null; then timeout 2 chronyc tracking || true; fi
  if command -v timedatectl >/dev/null; then timeout 2 timedatectl timesync-status || true; fi
}

{
  printf 'role=%s\nrun_id=%s\ncapture_id=%s\n' "$ROLE" "$RUN_LABEL" "$CAPTURE_ID"
  printf 'ROS_DOMAIN_ID=%s\nROS_DISTRO=%s\nRMW_IMPLEMENTATION=%s\n' \
    "${ROS_DOMAIN_ID:-0}" "${ROS_DISTRO:-unset}" "${RMW_IMPLEMENTATION:-default}"
  git -C "$SCRIPT_DIR/.." rev-parse HEAD 2>/dev/null || true
  git -C "$SCRIPT_DIR/.." status --short 2>/dev/null || true
  uname -a
  snapshot_clock
} > "$RUN_DIR/environment.txt" 2>&1

start_child metadata python3 "$SCRIPT_DIR/timing_observer.py" \
  --role "$ROLE" --output "$RUN_DIR/events.jsonl" --capture-id "$CAPTURE_ID"

BAG_TOPICS=(/rosout /tf /tf_static /clock /diagnostics \
  /rby1/vslam/bridge_status /rby1/vslam/timing /rby1/vslam/camera_odometry \
  /rby1/vslam/camera_slam_odometry /rby1/vslam/odom)
if [[ "$ROLE" == upc ]]; then
  BAG_TOPICS+=(/rby1/odom /rby1/robot_state /rby1/control/command /rby1/control/event \
    /rby1/vslam/slam_odom /rby1/vslam/localization_status /rby1/vslam/navigation_status \
    /rby1/vslam/nav2_cmd_vel /rby1/cmd_raw /rby1/cmd_vel \
    /rby1/vslam/nav2/navigate_to_pose/_action/status \
    /rby1/vslam/nav2/navigate_to_pose/_action/feedback)
  NODES=(/rby1/vslam/upc_bridge /rby1/vslam/pose_adapter /rby1/vslam/localization_tf \
    /rby1/vslam/nav2_gate /rby1/vslam/waypoint_ui /rby1/rby1_control \
    /rby1/vslam/nav2/controller_server /rby1/vslam/nav2/velocity_smoother)
else
  BAG_TOPICS+=(/visual_slam/status)
  NODES=(/rby1/vslam/lab_bridge /visual_slam)
fi
start_child rosbag ros2 bag record --include-hidden-topics \
  -o "$RUN_DIR/bag" "${BAG_TOPICS[@]}"
if [[ "$PCAP" == true ]]; then
  start_child tcpdump tcpdump -i "$INTERFACE" -n -s 128 -U \
    -w "$RUN_DIR/tcp.pcap" "tcp port $PORT"
fi

# Snapshots are taken after live collectors start to avoid a startup blind spot.
timeout 5 ros2 node list > "$RUN_DIR/nodes.txt" 2>&1 || true
timeout 5 ros2 topic list -t --include-hidden-topics > "$RUN_DIR/topics.txt" 2>&1 || true
timeout 5 ros2 topic info /rby1/cmd_raw --verbose > "$RUN_DIR/cmd_raw_publishers.txt" 2>&1 || true
for node in "${NODES[@]}"; do
  timeout 5 ros2 param dump "$node" > "$RUN_DIR/${node//\//_}_params.yaml" 2>&1 || true
done

echo "[timing] Collecting $ROLE: $RUN_DIR"
echo "[timing] Check metadata.log and rosbag.log for subscriptions before trial."
echo "[timing] Ctrl+C ends capture; keep both PCs collecting through the stop event."
while true; do
  for index in "${!PIDS[@]}"; do
    kill -0 "${PIDS[index]}" 2>/dev/null || die "${NAMES[index]} stopped; inspect ${NAMES[index]}.log"
  done
  {
    python3 -c 'import json,time; print(json.dumps({"wall_ns":time.time_ns(),"monotonic_ns":time.monotonic_ns()}))'
    snapshot_clock
    cat /proc/loadavg
    head -n 1 /proc/stat
    awk '/^(MemTotal|MemAvailable|SwapTotal|SwapFree):/ {print}' /proc/meminfo
    cat /proc/net/dev
    if command -v ss >/dev/null; then timeout 2 ss -tinp "( sport = :$PORT or dport = :$PORT )" || true; fi
    ps -eo pid,pcpu,pmem,comm,args --sort=-pcpu | head -n 25 || true
    if command -v nvidia-smi >/dev/null; then
      timeout 2 nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used \
        --format=csv,noheader || true
    fi
  } >> "$RUN_DIR/system_samples.txt" 2>&1
  sleep 1
done
