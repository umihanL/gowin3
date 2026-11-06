#!/usr/bin/env bash
# Install a persistent local perception/interaction stack; never start it here.
set -eo pipefail
cd "$(dirname "$0")/.."
autostart_source_workspace="$(pwd -P)"
autostart_workspace=""
autostart_output=""
usage() {
  cat <<'HELP'
Usage: bash tools/install_offline_autostart.sh [--workspace /absolute/workspace]
       bash tools/install_offline_autostart.sh --output-dir DIR [--workspace /root/rescue/ros2_ws]

Install mode requires root on X5. It backs up existing unit files, installs four
services and rescue-standalone.target, and enables that target for the next boot.
It does not start, stop or restart any service, change pinmux, download models,
enable motor/pump outputs, or change the current microphone configuration.

Output mode only writes reviewable unit files. It never calls systemctl or checks
board devices. Its default target workspace is /root/rescue/ros2_ws. Optional
existing Qwen/dashboard services are discovered only during an actual install.
HELP
}
while [ "$#" -gt 0 ]; do
  case "$1" in
    --workspace|--output-dir)
      option="$1"
      [ "$#" -ge 2 ] || { usage >&2; exit 2; }
      if [ "$option" = --workspace ]; then autostart_workspace="$2"; else autostart_output="$2"; fi
      shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [ -z "$autostart_workspace" ]; then
  if [ -n "$autostart_output" ]; then autostart_workspace=/root/rescue/ros2_ws;
  else autostart_workspace="$autostart_source_workspace"; fi
fi
if [[ ! "$autostart_workspace" =~ ^/[A-Za-z0-9_./-]+$ ]] || \
   [[ "/${autostart_workspace#/}/" == *'/../'* ]] || [ "$autostart_workspace" = / ]; then
  echo 'The target workspace must be an absolute ASCII path without spaces, parent traversal or systemd specifiers.' >&2
  exit 2
fi
autostart_workspace="${autostart_workspace%/}"
autostart_required_units='rescue-mipi-camera.service rescue-mipi-vision.service rescue-yolo-preview.service rescue-task-interaction.service'
autostart_optional_units=''
if [ -n "$autostart_output" ]; then
  mkdir -p -- "$autostart_output"
  autostart_units_dir="$(cd "$autostart_output" && pwd -P)"
  echo 'Output-only mode: optional installed Qwen/dashboard units have not been inspected.'
else
  [ "$(id -u)" = 0 ] || { echo 'Run installation as root on the X5.' >&2; exit 1; }
  command -v systemctl >/dev/null || { echo 'systemd is required.' >&2; exit 1; }
  for script in run_mipi_camera.sh run_mipi_vision.sh run_detection_preview.sh run_standalone_tasks.sh run_task_modes.sh; do
    [ -r "$autostart_workspace/tools/$script" ] || { echo "Missing local script: $script" >&2; exit 1; }
  done
  for local_file in .venv/bin/python install/setup.bash config/voice.local.json config/actuator_io.mixed.template.json; do
    [ -r "$autostart_workspace/$local_file" ] || { echo "Missing local workspace file: $local_file. Install/build locally first." >&2; exit 1; }
  done
  for optional in rescue-presentation.service rescue-edge-llm.service; do
    if systemctl cat "$optional" >/dev/null 2>&1; then
      autostart_optional_units="$autostart_optional_units $optional"
      echo "Will request existing optional service at boot: $optional (its unit is not modified)."
    else
      echo "Optional service absent: $optional; the core stack will still start. No model or service is downloaded."
    fi
  done
  autostart_units_dir=/etc/systemd/system
fi

autostart_backup_dir=''
backup_unit() {
  local target="$autostart_units_dir/$1"
  if [ -e "$target" ] || [ -L "$target" ]; then
    if [ -z "$autostart_backup_dir" ]; then
      if [ -n "$autostart_output" ]; then
        autostart_backup_dir="$autostart_units_dir/backups/$(date +%Y%m%dT%H%M%S)-$$"
      else
        autostart_backup_dir="/root/.local/state/rescue_robot/autostart-unit-backups/$(date +%Y%m%dT%H%M%S)-$$"
      fi
      mkdir -p -- "$autostart_backup_dir"
    fi
    cp -a -- "$target" "$autostart_backup_dir/$1"
  fi
}
write_service() {
  local unit="$1" description="$2" after="$3" command="$4" environment="$5"
  backup_unit "$unit"
  # Replace a local symlink as a file without writing through to another unit.
  local temporary="$autostart_units_dir/.$unit.new.$$"
  cat > "$temporary" <<EOF
[Unit]
Description=$description
RequiresMountsFor=$autostart_workspace
After=local-fs.target $after
PartOf=rescue-standalone.target
StartLimitIntervalSec=0

[Service]
Type=simple
WorkingDirectory=$autostart_workspace
Environment=PYTHONUNBUFFERED=1
Environment=ROS_LOG_DIR=$autostart_workspace/logs/ros
$environment
ExecStart=/bin/bash $autostart_workspace/tools/$command
Restart=on-failure
RestartSec=5
KillMode=control-group
KillSignal=SIGINT
TimeoutStopSec=20
EOF
  chmod 644 "$temporary"
  mv -f -- "$temporary" "$autostart_units_dir/$unit"
}
write_service rescue-mipi-camera.service 'Rescue GS130W local stereo camera' '' 'run_mipi_camera.sh' ''
write_service rescue-mipi-vision.service 'Rescue local YOLO perception' 'rescue-mipi-camera.service' 'run_mipi_vision.sh' ''
write_service rescue-yolo-preview.service 'Rescue local detection preview' 'rescue-mipi-vision.service' 'run_detection_preview.sh --bind 0.0.0.0 --port 8086' ''
write_service rescue-task-interaction.service 'Rescue local interaction and fixed-camera observation' 'rescue-mipi-vision.service' 'run_standalone_tasks.sh' 'EnvironmentFile=-/etc/default/rescue-standalone'
backup_unit rescue-standalone.target
autostart_target_temporary="$autostart_units_dir/.rescue-standalone.target.new.$$"
cat > "$autostart_target_temporary" <<EOF
[Unit]
Description=Rescue standalone local perception and interaction
Wants=$autostart_required_units$autostart_optional_units
After=local-fs.target

[Install]
WantedBy=multi-user.target
EOF
chmod 644 "$autostart_target_temporary"
mv -f -- "$autostart_target_temporary" "$autostart_units_dir/rescue-standalone.target"
if [ -n "$autostart_backup_dir" ]; then echo "Previous unit files saved: $autostart_backup_dir"; fi
if [ -n "$autostart_output" ]; then
  echo "Generated five unit files in $autostart_units_dir. No service has been changed or enabled."
else
  systemctl daemon-reload
  systemctl enable rescue-standalone.target
  echo 'Enabled rescue-standalone.target for the next boot; running services were not started, stopped or restarted.'
  echo 'This commissioning profile keeps UART, navigation, motors and pump in dry-run/disabled mode.'
  echo 'Voice uses voice.local.json (currently push-to-talk); no audio configuration was changed.'
  echo 'Once ready, explicitly start with: systemctl start rescue-standalone.target'
fi
