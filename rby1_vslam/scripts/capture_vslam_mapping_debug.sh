#!/usr/bin/env bash
# UPC mapping loop capture with operator phase markers and offline jitter analysis.
set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
RECORDER="$PROJECT_ROOT/record_trial.sh"

if [[ ! -f "$RECORDER" ]]; then
  echo "record_trial.sh not found at $RECORDER" >&2
  exit 2
fi

exec bash "$RECORDER" --role upc --mapping-debug "$@"
