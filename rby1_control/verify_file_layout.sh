#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
required=(
  package.xml
  setup.py
  setup.cfg
  resource/rby1_control
  config/default.yaml
  launch/control.launch.py
  rby1_control/__init__.py
  rby1_control/backend_contract.py
  rby1_control/command_model.py
  rby1_control/control_node.py
  rby1_control/control_transport.py
  rby1_control/topic_protocol.py
  rby1_control/gripper_controller.py
  rby1_control/manipulator_controller.py
  rby1_control/mobile_base_controller.py
  rby1_control/power_servo_state_adaptor.py
  rby1_control/robot_session.py
  rby1_control/state_manager.py
  rby1_control/backend_main.py
)

missing=0
for item in "${required[@]}"; do
  if [[ ! -f "$ROOT/$item" ]]; then
    echo "MISSING: $item"
    missing=1
  else
    echo "OK: $item"
  fi
done

for excluded in \
  launch/control_debug.launch.py \
  rby1_control/frontend_node.py \
  rby1_control/main.py \
  rby1_control/main_window.py \
  rby1_control/qt_compat.py \
  rby1_control/scenario_ui.py; do
  if [[ -e "$ROOT/$excluded" ]]; then
    echo "UNEXPECTED: $excluded" >&2
    missing=1
  fi
done

if [[ $missing -ne 0 ]]; then
  echo "File-layout check failed." >&2
  exit 1
fi

echo "File-layout check passed. No ROS node or GUI was executed."
