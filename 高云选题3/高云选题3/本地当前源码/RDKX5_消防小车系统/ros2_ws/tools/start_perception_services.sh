#!/usr/bin/env bash
# Explicit, on-demand real-camera/YOLO preview. No driving or actuator nodes.
set -eo pipefail
cd "$(dirname "$0")/.."
perception_workspace="$(pwd -P)"
if [ "$(id -u)" != 0 ]; then
  echo 'Run as root on X5.' >&2
  exit 1
fi
mkdir -p logs/ros
start_service() {
  local unit="$1" script="$2"
  shift 2
  if systemctl is-active --quiet "$unit"; then
    echo "$unit already active"
    return
  fi
  if systemctl cat "$unit.service" >/dev/null 2>&1; then
    systemctl start "$unit.service"
    return
  fi
  systemd-run --unit="$unit" --collect \
    --working-directory="$perception_workspace" \
    --setenv="ROS_LOG_DIR=$perception_workspace/logs/ros" \
    --property=KillMode=control-group --property=KillSignal=SIGINT \
    --property=TimeoutStopSec=15s \
    /bin/bash "$perception_workspace/tools/$script" "$@"
}
start_service rescue-mipi-camera run_mipi_camera.sh
start_service rescue-mipi-vision run_mipi_vision.sh
start_service rescue-yolo-preview run_detection_preview.sh --bind 0.0.0.0 --port 8086
if systemctl cat rescue-presentation.service >/dev/null 2>&1; then
  systemctl start rescue-presentation.service
fi
echo 'Startup requested. These services do not start FPGA, navigation, microphone tasks, gimbal or pump.'
echo 'Check: systemctl status rescue-mipi-camera rescue-mipi-vision rescue-yolo-preview --no-pager'
echo 'Without install_offline_autostart.sh these services are temporary; rerun after reboot.'
