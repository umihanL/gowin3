#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo 'ROS 2 Humble was not found. Verify the board OS and install the matching official ROS environment first.' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
# System OpenCV/CvBridge and numpy must remain ABI-compatible. The private
# system-site-packages venv preserves RDK/ROS binary modules and isolates OCR pip packages.
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-colcon-common-extensions \
  python3-numpy python3-opencv python3-serial python3-yaml python3-pil espeak-ng libportaudio2 alsa-utils \
  ros-humble-cv-bridge ros-humble-rosidl-default-generators ros-humble-geometry-msgs
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install 'numpy<2' 'onnxruntime==1.20.1' 'pyclipper==1.3.0.post6' 'shapely>=1.8,<3' 'PyYAML>=6,<7' 'six>=1.16' 'Pillow>=9,<12'
# Avoid installing a second OpenCV wheel (and inadvertently numpy2) over ROS CvBridge.
python -m pip install --no-deps 'rapidocr-onnxruntime==1.4.4'
# scipy 1.10.1 was validated on X5/Python 3.10; voice FIR resampling needs it
# when a USB microphone exposes 48 kHz rather than native 16 kHz input.
python -m pip install 'vosk==0.3.45' 'sounddevice>=0.4,<0.6' 'scipy==1.10.1'
# Local SenseVoice primary ASR; keep ROS/OCR NumPy and OpenCV dependencies intact.
python -m pip install --no-deps 'sherpa-onnx==1.13.8' 'sherpa-onnx-core==1.13.8'
python -m colcon build
echo 'Build complete. Activate .venv and source install/setup.bash in each runtime terminal.'
