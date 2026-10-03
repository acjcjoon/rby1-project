#!/usr/bin/env bash
# Full UPC TCP VSLAM/Nav2 stack from a browser.
exec "${BASH}" "$(dirname -- "${BASH_SOURCE[0]}")/scripts/launch_operator.sh" vslam web "$@"
