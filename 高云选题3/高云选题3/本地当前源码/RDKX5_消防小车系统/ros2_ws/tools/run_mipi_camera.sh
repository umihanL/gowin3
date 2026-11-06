#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source ./tools/image_transport_env.sh
source /opt/tros/humble/setup.bash
export ROS_DOMAIN_ID="${RESCUE_CAMERA_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${RESCUE_CAMERA_LOCALHOST_ONLY:-0}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/logs/ros}"
mkdir -p "$ROS_LOG_DIR"
exec 9>/tmp/rescue-mipi-camera.lock
if ! flock -n 9 || pgrep -f '^/opt/tros/humble/lib/mipi_cam/mipi_cam( |$)' >/dev/null; then
  echo 'An MIPI camera driver is already running. Do not open the hardware twice.' >&2
  exit 2
fi
echo "Starting one GS130W camera driver in ROS domain $ROS_DOMAIN_ID (WARN logging)."
# Keep the validated raw stream and channel order; downstream swaps the physical eyes.
exec bash /opt/tros/humble/share/hobot_stereonet/script/run_cam.sh \
  --image_width 1280 --image_height 1088 --framerate 30.0 \
  --rotation 90.0 --cal_rotation 90.0 --gdc_enable False \
  --lpwm_enable True --frame_ts_type realtime --out_format nv12 \
  --channel 2 --channel2 0 --log_level WARN "$@"
