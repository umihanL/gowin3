#!/usr/bin/env bash
set -eo pipefail
export OMP_WAIT_POLICY="${OMP_WAIT_POLICY:-PASSIVE}"
export GOMP_SPINCOUNT="${GOMP_SPINCOUNT:-0}"
cd "$(dirname "$0")/.."
source ./tools/image_transport_env.sh
# Official camera currently uses the default domain (0) and localhost_only=0.
# Set these BEFORE env.sh, whose generic project default is domain 71.
export ROS_DOMAIN_ID="${RESCUE_CAMERA_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${RESCUE_CAMERA_LOCALHOST_ONLY:-0}"
source ./env.sh
if [ ! -f install/setup.bash ]; then
  echo 'Build first: bash tools/build_board.sh' >&2
  exit 2
fi
mkdir -p logs
exec 9>logs/mipi_vision.lock
if ! flock -n 9; then
  echo 'MIPI perception is already running. Stop its service/terminal before restarting.' >&2
  exit 1
fi
echo "Subscribing to the existing MIPI camera in ROS domain $ROS_DOMAIN_ID."
echo '2D detection only; this command does not start another camera driver.'
vision_lens_config="$(python3 tools/stereo_start_profile.py --workspace "$RESCUE_WS_ROOT" --lens-only)"
exec ros2 launch rescue_vision mipi_vision.launch.py \
  workspace_dir:="$RESCUE_WS_ROOT" "$vision_lens_config" "$@"
