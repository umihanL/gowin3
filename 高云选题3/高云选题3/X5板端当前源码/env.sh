#!/usr/bin/env bash
# Source this file in every X5 development terminal: source ./env.sh
export RESCUE_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# System services may have no HOME environment; ROS needs an explicit log path.
export ROS_LOG_DIR="${ROS_LOG_DIR:-$RESCUE_WS_ROOT/logs/ros}"
mkdir -p "$ROS_LOG_DIR" || { return 1 2>/dev/null || exit 1; }
if [ ! -f /opt/ros/humble/setup.bash ] || [ ! -f "$RESCUE_WS_ROOT/.venv/bin/activate" ]; then
  echo 'This environment needs ROS 2 Humble and the application .venv on the X5.' >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/humble/setup.bash
source "$RESCUE_WS_ROOT/.venv/bin/activate"
if [ -f "$RESCUE_WS_ROOT/install/setup.bash" ]; then
  source "$RESCUE_WS_ROOT/install/setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
