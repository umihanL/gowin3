#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
if [ ! -f /opt/tros/humble/setup.bash ]; then
  echo 'This preview requires the X5 TogetheROS Humble installation.' >&2
  exit 2
fi
source /opt/tros/humble/setup.bash
# An existing project terminal can contain domain 71. Always match run_mipi_vision.sh.
export ROS_DOMAIN_ID="${RESCUE_CAMERA_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${RESCUE_CAMERA_LOCALHOST_ONLY:-0}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/logs/ros}"
mkdir -p "$ROS_LOG_DIR"
# Keep one copy of this preview; the inherited lock is released when it exits.
exec 9>/tmp/rescue-mipi-preview.lock
if ! flock -n 9; then
  echo 'The MIPI preview is already running. Do not launch a second copy.' >&2
  echo 'For the managed instance: systemctl status rescue-mipi-preview --no-pager' >&2
  exit 2
fi
echo "Previewing the existing camera in ROS domain $ROS_DOMAIN_ID."
echo 'This starts JPEG encoding and the web viewer only; no camera driver is started.'
echo 'Open http://192.168.127.10:8000/TogetheROS/ for the current direct Ethernet connection.'
# Launch the source file directly so this helper does not require a package rebuild.
exec ros2 launch "$PWD/src/rescue_vision/launch/mipi_preview.launch.py" "$@"
