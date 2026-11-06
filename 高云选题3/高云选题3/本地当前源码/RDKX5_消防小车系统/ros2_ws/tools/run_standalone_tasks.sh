#!/usr/bin/env bash
# Same commissioning profile as interactive startup. No network/desktop needed.
set -eo pipefail
cd "$(dirname "$0")/.."
standalone_workspace="$(pwd -P)"
if [ "$#" -ne 0 ]; then
  echo 'This profile takes no launch overrides. Use RESCUE_STANDALONE_VOICE_CONFIG, RESCUE_STANDALONE_FIXED_CAMERA_CONFIG or RESCUE_STANDALONE_FIXED_CAMERA_LANDING_CONFIG for explicit configuration files.' >&2
  exit 2
fi
standalone_voice_config="${RESCUE_STANDALONE_VOICE_CONFIG:-$standalone_workspace/config/voice.local.json}"
if [ ! -r "$standalone_voice_config" ]; then
  echo "Missing local voice configuration: $standalone_voice_config" >&2
  exit 2
fi
standalone_fixed_config_args=(--workspace "$standalone_workspace")
if [ -n "${RESCUE_STANDALONE_FIXED_CAMERA_CONFIG:-}" ]; then
  standalone_fixed_config_args+=(--config "$RESCUE_STANDALONE_FIXED_CAMERA_CONFIG")
fi
# A bad existing local file is an error, not permission to change geometry.
# Command substitution preserves a nonzero validator result under set -e.
standalone_fixed_config="$(python3 "$standalone_workspace/tools/fixed_camera_start_config.py" "${standalone_fixed_config_args[@]}")"
standalone_fixed_landing_args=(--workspace "$standalone_workspace" --kind landing)
if [ -n "${RESCUE_STANDALONE_FIXED_CAMERA_LANDING_CONFIG:-}" ]; then
  standalone_fixed_landing_args+=(--config "$RESCUE_STANDALONE_FIXED_CAMERA_LANDING_CONFIG")
fi
standalone_fixed_landing_config="$(python3 "$standalone_workspace/tools/fixed_camera_start_config.py" "${standalone_fixed_landing_args[@]}")"
# Real UART readback is independent of the current-boot mechanical reference.
# Resolver errors must stop startup instead of silently falling back to dry-run.
standalone_gimbal_output="$(python3 "$standalone_workspace/tools/gimbal_start_profile.py" --workspace "$standalone_workspace" --runtime-dir "${RESCUE_GIMBAL_RUNTIME_DIR:-/run/rescue}" --fixed-camera-config "$standalone_fixed_config")"
mapfile -t standalone_gimbal_args <<< "$standalone_gimbal_output"
if [ "${#standalone_gimbal_args[@]}" -ne 4 ]; then
  echo 'Live gimbal startup did not produce the required configuration.' >&2
  exit 2
fi
# Mission and FireAim must use the same confirmed mechanical ranges as the
# motor owner. The old 90/90 example is unrelated to this 0/30 installation.
standalone_task_geometry="$(python3 "$standalone_workspace/tools/gimbal_task_profiles.py" \
  --workspace "$standalone_workspace" \
  --io-config "${standalone_gimbal_args[0]#io_config:=}" \
  --control-config "${standalone_gimbal_args[1]#control_config:=}" \
  --runtime-dir "${RESCUE_GIMBAL_RUNTIME_DIR:-/run/rescue}")"
mapfile -t standalone_task_geometry_args <<< "$standalone_task_geometry"
if [ "${#standalone_task_geometry_args[@]}" -ne 2 ]; then
  echo 'Mechanical task configuration did not produce mount and servo settings.' >&2
  exit 2
fi
standalone_stereo_output="$(python3 "$standalone_workspace/tools/stereo_start_profile.py" --workspace "$standalone_workspace")"
mapfile -t standalone_stereo_args <<< "$standalone_stereo_output"
exec /bin/bash "$standalone_workspace/tools/run_task_modes.sh" \
  voice_config:="$standalone_voice_config" \
  "${standalone_gimbal_args[@]}" \
  "${standalone_task_geometry_args[@]}" \
  "${standalone_stereo_args[@]}" \
  manual_pump_test_enabled:=true \
  fixed_camera_aim_config:="$standalone_fixed_config" \
  fixed_camera_landing_config:="$standalone_fixed_landing_config" \
  task_initial_mode:=inspect fixed_camera_observation:=true \
  road_raw_input_topic:=/rescue/camera/road_raw road_process_on_receive:=true \
  ocr_fixed_camera_preview:=true navigation_execute_enabled:=true \
  uart_dry_run:=true
