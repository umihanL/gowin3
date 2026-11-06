#!/usr/bin/env bash
set -euo pipefail
# Read-only preflight. This does not install packages or open serial/camera devices.
python3 - <<'PY'
import platform,sys,json,importlib.util,pathlib,glob,os
data=dict(machine=platform.machine(),python=sys.version,ros_distro=os.environ.get('ROS_DISTRO'),
          os_release=pathlib.Path('/etc/os-release').read_text(),
          installed_ros=glob.glob('/opt/ros/*'),video_nodes=glob.glob('/dev/video*'),
          serial_nodes=glob.glob('/dev/ttyS*')+glob.glob('/dev/ttyUSB*')+glob.glob('/dev/ttyACM*'))
for name in ('rclpy','cv2','serial','numpy','onnxruntime','rapidocr_onnxruntime','hobot_dnn','hbm_runtime'):
    data[name+'_available']=importlib.util.find_spec(name) is not None
print(json.dumps(data,ensure_ascii=False,indent=2))
PY
