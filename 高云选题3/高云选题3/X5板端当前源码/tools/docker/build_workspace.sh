#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
# Build artifacts stay in the isolated Linux container, not the Windows tree.
colcon --log-base /tmp/rescue_ros_log build --base-paths /workspace/src \
  --build-base /tmp/rescue_ros_build --install-base /tmp/rescue_ros_install \
  --event-handlers console_direct+
source /tmp/rescue_ros_install/setup.bash
python3 - <<'PY'
import cv2, rclpy, serial
import rescue_interfaces.msg
from rescue_vision.stereo import StereoDepth
print('ROS_IMPORT_SMOKE_OK', 'opencv=' + cv2.__version__)
PY
