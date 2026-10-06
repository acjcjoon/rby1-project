#!/usr/bin/env bash
# Compatibility wrapper for the canonical passive recorder's Nav2 profile.
set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
RECORDER="$PROJECT_ROOT/record_trial.sh"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: bash capture_navigation_debug.sh [output_directory]

UPC-only passive Nav2/VSLAM diagnostics. Start the robot stack first, then run
this script. Wait for the "Nav2 ready parameter snapshot complete" line before
sending the waypoint. Stop the robot in the UI and press Ctrl+C here while the
stack is still running so final parameter dumps succeed.

The default output parent is ~/rby1_debug. No services, actions, parameters, or
motion commands are sent by this script.
EOF
  exit 0
fi
[[ $# -le 1 ]] || { echo 'Expected at most one output directory' >&2; exit 2; }
[[ -f "$RECORDER" ]] || {
  echo "record_trial.sh not found at $RECORDER" >&2
  exit 2
}

OUTPUT_ROOT="${1:-${HOME}/rby1_debug}"
exec bash "$RECORDER" \
  --role upc \
  --run-id nav2_debug \
  --output "$OUTPUT_ROOT" \
  --nav2-debug
