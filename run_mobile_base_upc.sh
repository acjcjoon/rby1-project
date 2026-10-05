#!/usr/bin/env bash
# Local Qt mobile base; no LAB/TCP/VSLAM/Nav2.
exec "${BASH}" "$(dirname -- "${BASH_SOURCE[0]}")/scripts/launch_operator.sh" mobile "$@"
