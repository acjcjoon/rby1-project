#!/usr/bin/env bash
# UPC mobile base from a browser; no LAB/TCP/VSLAM/Nav2.
exec "${BASH}" "$(dirname -- "${BASH_SOURCE[0]}")/scripts/launch_operator.sh" mobile web "$@"
