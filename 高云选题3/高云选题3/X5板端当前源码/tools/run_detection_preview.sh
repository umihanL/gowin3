#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
# Camera, perception and this display must share the same ROS network domain.
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
if [ ! -x .venv/bin/python ]; then
  echo 'Missing X5 workspace Python: .venv/bin/python' >&2
  exit 1
fi
if ! command -v flock >/dev/null 2>&1; then
  echo 'flock is required (provided by util-linux).' >&2
  exit 1
fi
source ./env.sh
mkdir -p logs
exec 9>logs/detection_preview.lock
if ! flock -n 9; then
  echo 'The YOLO detection preview is already running for this workspace.' >&2
  exit 1
fi
exec .venv/bin/python tools/detection_preview.py "$@"
