#!/usr/bin/env bash
# Live services reuse the explicitly configured, current-boot hardware profile.
set -eo pipefail
cd "$(dirname "$0")/.."
interaction_workspace="$(pwd -P)"
if [ "$(id -u)" != 0 ]; then
  echo 'Run as root on the X5.' >&2
  exit 1
fi
bash tools/start_perception_services.sh
if systemctl cat rescue-edge-llm.service >/dev/null 2>&1; then
  systemctl start rescue-edge-llm.service
fi
if systemctl is-active --quiet rescue-task-interaction.service; then
  echo 'rescue-task-interaction already active'
elif systemctl cat rescue-task-interaction.service >/dev/null 2>&1; then
  systemctl start rescue-task-interaction.service
else
  systemd-run --unit=rescue-task-interaction --collect \
    --working-directory="$interaction_workspace" \
    --setenv="ROS_LOG_DIR=$interaction_workspace/logs/ros" \
    --setenv=PYTHONUNBUFFERED=1 \
    --property=KillMode=control-group --property=KillSignal=SIGINT \
    --property=TimeoutStopSec=20s \
    /bin/bash "$interaction_workspace/tools/run_standalone_tasks.sh"
fi
python3 - <<'PY'
import json, sys, time, urllib.request
sys.path.insert(0, 'tools')
from interaction_start_status import describe
deadline = time.monotonic() + 45
last_reason = 'waiting_for_service'
while time.monotonic() < deadline:
    try:
        with urllib.request.urlopen('http://127.0.0.1:8090/api/state', timeout=2) as response:
            state = json.load(response)
        control = state.get('control', {})
        voice_topic = state.get('topics', {}).get('voice', {})
        voice = voice_topic.get('data') or {}
        voice_intentionally_off = (voice_topic.get('fresh') is True
                                   and voice.get('voice_enabled') is False
                                   and (voice.get('voice_control') or {}).get('status') != 'pending')
        if control.get('command_ready') and (control.get('listen_ready') or voice_intentionally_off):
            print('\n'.join(describe(state)))
            break
        last_reason = control.get('read_only_reason') or 'voice_not_ready'
    except (OSError, ValueError) as exc:
        last_reason = type(exc).__name__
    time.sleep(1)
else:
    raise SystemExit('Interaction did not become ready: ' + last_reason +
                     '. Check: journalctl -u rescue-task-interaction -n 60 --no-pager')
PY
echo '功能就绪情况以实时上位机反馈为准；交互端点在线不等于电机和瞄准标定已验收。'
echo 'For later boots install_offline_autostart.sh enables the persistent stack; this entry also reuses existing units.'
