#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
export ROS_DOMAIN_ID="${RESCUE_CAMERA_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${RESCUE_CAMERA_LOCALHOST_ONLY:-0}"
source ./env.sh
mkdir -p logs
exec 9>logs/local_fire_control.lock
if ! flock -n 9; then
  echo 'Local fire control is already running.' >&2
  exit 1
fi
if pgrep -f '[/]rescue_vision/(mission|fpga_bridge|fire_aim|local_actuator|stereo_depth)( |$)' >/dev/null; then
  echo 'Existing Mission/bridge/aim/actuator node found. Stop the previous controller first.' >&2
  exit 1
fi
echo 'Reusing existing perception. Defaults are dry-run, tracking disabled, pump disarmed.'
exec ros2 launch rescue_vision local_fire_control.launch.py workspace_dir:="$RESCUE_WS_ROOT" "$@"
