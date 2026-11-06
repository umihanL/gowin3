#!/usr/bin/env bash
# Optional background software preview; does not enable startup at boot.
set -eo pipefail
cd "$(dirname "$0")/.."
source ./env.sh
case "${1:-status}" in
  start)
    if systemctl is-active --quiet rescue-preview.service; then
      echo 'Software preview is already running.'
    else
      systemd-run --unit=rescue-preview --collect \
        --description='Rescue ROS 2 software preview (no physical I/O)' \
        --uid="$(id -u)" \
        --property=KillMode=control-group --property=KillSignal=SIGINT \
        --property=TimeoutStopSec=15s --working-directory="$RESCUE_WS_ROOT" \
        /bin/bash "$RESCUE_WS_ROOT/tools/run_board_preview.sh"
    fi
    ;;
  stop) systemctl stop rescue-preview.service ;;
  status) systemctl --no-pager --full status rescue-preview.service ;;
  logs) journalctl -u rescue-preview.service -n 80 --no-pager ;;
  *) echo 'Usage: bash tools/preview_service.sh {start|stop|status|logs}' >&2; exit 2 ;;
esac
