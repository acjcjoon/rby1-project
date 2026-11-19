#!/usr/bin/env bash
# One-shot launcher for the station stack: base+UI, D435i, Isaac cuVSLAM, Nav2 (+ optional bag).
#
# Replaces the four terminals of README "실행: 호스트 터미널 4개". Components start in
# order and each waits for the previous one's topic. Output is prefixed per component
# and saved under ~/rby1_bags/logs/stack_<time>/. Ctrl-C (or any component exiting)
# stops everything in reverse order, including the launch inside the Isaac container.
#
#   bash rby1_vslam/run_station_stack.sh [--fps 30|60|90] [--waypoints FILE]
#        [--record NAME] [--no-nav] [--dry-run]
set -Eeo pipefail   # -u only after sourcing ROS (its setup scripts use unset vars)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${RBY1_ISAAC_CONTAINER:-rby1-isaac-vslam}"
export ROS_DOMAIN_ID="${RBY1_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-10}}"
export RBY1_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export RBY1_D435_SERIAL="${RBY1_D435_SERIAL:-244222075961}"

FPS=90
WAYPOINTS="${HOME}/rby1_bags/exp3_stations.yaml"
RECORD=''
START_NAV=true
DRY_RUN=false
MAX_LINEAR=0.1
MAX_ANGULAR=0.2
READY_TIMEOUT=40

usage() {
  sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<EOF

Options
  --fps N           D435i IR stereo rate: 30, 60 or 90 (default 90)
  --waypoints FILE  station file for the UI (default ~/rby1_bags/exp3_stations.yaml)
  --record NAME     also record a bag ~/rby1_bags/NAME_<time> once VSLAM is up
  --no-nav          do not start Nav2 (manual driving / VO experiments)
  --dry-run         print the commands only
Environment
  RBY1_ROS_DOMAIN_ID (default 10), RBY1_D435_SERIAL, RBY1_ISAAC_CONTAINER
EOF
}

while (($#)); do
  case "$1" in
    --fps) FPS="${2:?}"; shift 2 ;;
    --waypoints) WAYPOINTS="${2:?}"; shift 2 ;;
    --record) RECORD="${2:?}"; shift 2 ;;
    --no-nav) START_NAV=false; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ "$FPS" =~ ^(30|60|90)$ ]] || { echo '--fps must be 30, 60 or 90' >&2; exit 2; }
[[ "$WAYPOINTS" = /* ]] || WAYPOINTS="$(pwd)/$WAYPOINTS"
[[ -z "$RECORD" || "$RECORD" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo '--record: use letters, digits, _ . -' >&2; exit 2; }

source /opt/ros/humble/setup.bash
source "${HOME}/rby1_ros2_ws/install/setup.bash"
set -u

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${HOME}/rby1_bags/logs/stack_${STAMP}"
BAG_TOPICS=(
  /rby1/odom /rby1/cmd_vel /rby1/vslam/odom /rby1/vslam/slam_odom
  /rby1/vslam/camera_odometry /rby1/vslam/localization_odom
  /rby1/vslam/localization_status /rby1/vslam/navigation_status
  /rby1/vslam/tracking_status /rby1/vslam/experiment_event /tf /tf_static
)

CAMERA_ARGS=(0)
[[ "$FPS" != 90 ]] && CAMERA_ARGS+=(--camera-fps "$FPS")
declare -A CMD=(
  [base]="bash '$HERE/run_mobile_base.sh' start_camera:=false max_linear_speed:=$MAX_LINEAR max_angular_speed:=$MAX_ANGULAR waypoints_file:='$WAYPOINTS'"
  [cam]="bash '$HERE/run_d435i_host.sh' ${CAMERA_ARGS[*]}"
  [vslam]="bash '$HERE/run_isaac_vslam_docker.sh' 0 start_rviz:=false"
  [nav]="bash '$HERE/run_navigation.sh' start_localization_tf:=false use_scan:=false"
  [bag]="ros2 bag record --storage sqlite3 -o '${HOME}/rby1_bags/${RECORD}_${STAMP}' ${BAG_TOPICS[*]}"
)
ORDER=(base cam vslam)
$START_NAV && ORDER+=(nav)
[[ -n "$RECORD" ]] && ORDER+=(bag)

if $DRY_RUN; then
  printf 'ROS_DOMAIN_ID=%s  fps=%s  waypoints=%s\n' "$ROS_DOMAIN_ID" "$FPS" "$WAYPOINTS"
  for name in "${ORDER[@]}"; do printf '[%s] %s\n' "$name" "${CMD[$name]}"; done
  exit 0
fi

mkdir -p "$LOG_DIR"
declare -A PID=()
STOPPING=false

say() { printf '\033[1;36m[stack]\033[0m %s\n' "$*" | tee -a "$LOG_DIR/stack.log"; }

# bash starts background jobs of a script with SIGINT ignored, and that is inherited by
# ros2 launch, which would then ignore our orderly SIGINT. Restore the default first.
UNIGNORE_INT='import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGQUIT, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])'

start() {
  local name="$1"
  # Own session/process group so Ctrl-C reaches only this script; we stop children in order.
  setsid python3 -c "$UNIGNORE_INT" bash -c "exec ${CMD[$name]}" </dev/null \
    > >(stdbuf -oL sed -u "s/^/[$name] /" | tee -a "$LOG_DIR/$name.log") 2>&1 &
  PID[$name]=$!
  say "started $name (pid ${PID[$name]}, log $LOG_DIR/$name.log)"
}

wait_topic() {   # name topic : non-fatal readiness check
  local name="$1" topic="$2" deadline=$((SECONDS + READY_TIMEOUT))
  say "waiting for $topic ..."
  while ((SECONDS < deadline)); do
    kill -0 "${PID[$name]}" 2>/dev/null || { say "$name exited during startup"; return 1; }
    if timeout 5 ros2 topic echo --once --no-daemon --qos-reliability best_effort \
        "$topic" >/dev/null 2>&1; then
      say "$topic is live"
      return 0
    fi
  done
  say "WARNING: no message on $topic after ${READY_TIMEOUT}s; continuing (check $LOG_DIR/$name.log)"
}

stop_one() {
  local name="$1" pid="${PID[$1]:-}"
  [[ -n "$pid" ]] || return 0
  if [[ "$name" == vslam ]]; then
    # docker exec without a TTY does not forward signals: stop the launch inside the container.
    docker exec "$CONTAINER" pkill -INT -f 'ros2 launch rby1_vslam jetson.launch.py' 2>/dev/null || true
  fi
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in $(seq 1 50); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
  if kill -0 "$pid" 2>/dev/null; then
    say "$name did not stop in 10 s; terminating"
    kill -TERM -- "-$pid" 2>/dev/null || true
    [[ "$name" == vslam ]] && docker exec "$CONTAINER" pkill -TERM -f 'jetson.launch.py' 2>/dev/null || true
    sleep 2
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  say "stopped $name"
}

cleanup() {
  $STOPPING && return
  STOPPING=true
  trap '' INT TERM
  say 'stopping (reverse order) ...'
  local i
  for ((i = ${#ORDER[@]} - 1; i >= 0; i--)); do stop_one "${ORDER[$i]}"; done
  say "all stopped. logs: $LOG_DIR"
}
trap 'cleanup; exit 130' INT TERM
trap cleanup EXIT

say "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  fps=$FPS  waypoints=$WAYPOINTS  logs=$LOG_DIR"
[[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" == true ]] || {
  say "Isaac ROS container is not running: $CONTAINER (docker start $CONTAINER)"; exit 1; }
if docker exec "$CONTAINER" pgrep -f 'ros2 launch rby1_vslam jetson.launch.py' >/dev/null 2>&1; then
  say "cuVSLAM is already running inside $CONTAINER; stop it first (old terminal 3)"; exit 1
fi

start base
wait_topic base /rby1/odom || exit 1
start cam
wait_topic cam /d435/d435/infra1/camera_info || exit 1
start vslam
wait_topic vslam /rby1/vslam/camera_odometry || exit 1
if $START_NAV; then
  start nav
  sleep 3
fi
if [[ -n "$RECORD" ]]; then
  start bag
fi
say 'stack is up. Ctrl-C here stops everything.'

# Fail together: if any component exits, stop the rest.
while true; do
  for name in "${ORDER[@]}"; do
    if ! kill -0 "${PID[$name]}" 2>/dev/null; then
      say "$name exited; stopping the stack"
      exit 1
    fi
  done
  sleep 1
done
