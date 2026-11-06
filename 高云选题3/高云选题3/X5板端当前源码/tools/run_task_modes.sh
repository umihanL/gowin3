#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
export ROS_DOMAIN_ID="${RESCUE_CAMERA_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY="${RESCUE_CAMERA_LOCALHOST_ONLY:-0}"
# These task nodes use startup parameters, not runtime parameter RPCs. Set to 1
# before launching to restore `ros2 param` inspection; YOLO gating stays enabled.
export RESCUE_PARAMETER_SERVICES="${RESCUE_PARAMETER_SERVICES:-0}"
source ./env.sh
mkdir -p logs
exec 9>logs/task_modes.lock
flock -n 9 || { echo 'Task profiles already running.' >&2; exit 1; }
if pgrep -f '[/]rescue_vision/(mission|fpga_bridge|extension_task|fire_aim|local_actuator|stereo_depth|interaction|scene|task_interaction|task_scene|voice|navigation|fpga_navigation|mipi_building_ocr|auto_fire)( |$)' >/dev/null; then
  echo 'Stop the previous task/controller launch first; task owners cannot run twice.' >&2
  exit 1
fi
vision_ready=false
for discovery_attempt in 1 2 3; do
  if timeout 12 ros2 param get /fast_mipi_perception mission_gating >/dev/null &&
      timeout 12 ros2 param set /fast_mipi_perception mission_gating true; then
    vision_ready=true
    break
  fi
done
if [ "$vision_ready" != true ]; then
  echo 'Start/update tools/run_mipi_vision.sh first; this launcher reuses the existing camera and YOLO.' >&2
  exit 1
fi
trap 'timeout 3 ros2 param set /fast_mipi_perception mission_gating false >/dev/null 2>&1 || true' EXIT
echo 'Task profiles: default visual inspection; ov5640 FPGA owns motion. Extension UART follows config/extension_uart.json. Traffic recognition runs only in its selected task.'
dialogue_config="${RESCUE_EDGE_DIALOGUE_CONFIG:-$RESCUE_WS_ROOT/config/edge_dialogue.local.json}"
if [ -f "$dialogue_config" ]; then
  ros2 launch rescue_vision task_modes.launch.py workspace_dir:="$RESCUE_WS_ROOT" \
    edge_dialogue_config:="$dialogue_config" "$@"
else
  ros2 launch rescue_vision task_modes.launch.py workspace_dir:="$RESCUE_WS_ROOT" "$@"
fi
