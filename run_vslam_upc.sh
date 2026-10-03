#!/usr/bin/env bash
# Full UPC TCP VSLAM/Nav2 stack with the existing Qt/RViz operator.
exec "${BASH}" "$(dirname -- "${BASH_SOURCE[0]}")/scripts/launch_operator.sh" vslam qt "$@"
