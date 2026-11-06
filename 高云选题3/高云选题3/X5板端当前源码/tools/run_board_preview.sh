#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source ./env.sh
if [ ! -f install/setup.bash ]; then
  echo 'Build first: bash tools/build_board.sh' >&2
  exit 2
fi
# Explicit preview parameters and separate audio configuration prevent an edited
# hardware configuration from opening microphone/speakers during software study.
exec ros2 launch rescue_vision rescue.launch.py workspace_dir:="$RESCUE_WS_ROOT" \
  camera_enabled:=false detector_enabled:=false ocr_enabled:=false \
  uart_dry_run:=true navigation_execute_enabled:=false \
  voice_config:="$RESCUE_WS_ROOT/config/voice.board_preview.json" \
  telemetry_config:="$RESCUE_WS_ROOT/config/telemetry.board_preview.json"
