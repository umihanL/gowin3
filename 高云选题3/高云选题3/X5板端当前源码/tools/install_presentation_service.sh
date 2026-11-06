#!/usr/bin/env bash
# Install only the presentation bridge. Existing camera/task services are independent.
set -eo pipefail
cd "$(dirname "$0")/.."
workspace_dir="$(pwd -P)"
if [ "$(id -u)" != 0 ]; then
  echo 'Run as root on the X5: sudo bash tools/install_presentation_service.sh' >&2
  exit 1
fi
if [[ ! "$workspace_dir" =~ ^/[A-Za-z0-9_./-]+$ ]]; then
  echo 'The systemd installer needs an ASCII workspace path without spaces.' >&2
  exit 1
fi
if [ ! -x .venv/bin/python ] || [ ! -f tools/presentation_server.py ] || [ ! -f tools/run_presentation.sh ]; then
  echo 'Missing X5 virtual environment or presentation files.' >&2
  exit 1
fi
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
source ./env.sh
.venv/bin/python -c 'import rclpy; from rescue_interfaces.msg import DetectionArray; from rescue_vision.task_interaction import parse_task_text'
unit_file=/etc/systemd/system/rescue-presentation.service
backup_dir=/root/.local/state/rescue_robot/presentation-service-backups
if [ -f "$unit_file" ]; then
  mkdir -p "$backup_dir"
  cp -- "$unit_file" "$backup_dir/rescue-presentation.$(date +%Y%m%dT%H%M%S).service"
fi
cat > "$unit_file" <<EOF
[Unit]
Description=Rescue robot live control dashboard
After=network.target

[Service]
Type=simple
WorkingDirectory=$workspace_dir
ExecStart=/usr/bin/bash $workspace_dir/tools/run_presentation.sh --bind 0.0.0.0 --port 8090
Environment=PYTHONUNBUFFERED=1
Restart=on-failure
RestartSec=3
TimeoutStopSec=8

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable rescue-presentation.service
systemctl restart rescue-presentation.service
systemctl --no-pager --full status rescue-presentation.service
echo 'Live robot control: http://X5-IP:8090 . Camera/task services must already be configured separately.'
