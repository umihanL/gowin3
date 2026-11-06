#!/usr/bin/env bash
# Read-only board audit: no credentials, no network changes, no physical devices opened.
set -u
python3 - <<'PY'
import glob, importlib.util, json, os, pathlib, platform, shutil, subprocess, sys

def run(argv):
    try:
        result=subprocess.run(argv,capture_output=True,text=True,timeout=8)
        return {'returncode':result.returncode,'stdout':result.stdout.strip(),'stderr':result.stderr.strip()}
    except Exception as exc:return {'error':str(exc)}

report={
 'architecture':platform.machine(), 'python':sys.version,
 'os_release':pathlib.Path('/etc/os-release').read_text(),
 'user':run(['id','-un']), 'network':run(['ip','-br','addr']),
 'routes':run(['ip','route']), 'ssh_active':run(['systemctl','is-active','ssh']),
 'ros_setup':glob.glob('/opt/ros/*/setup.bash'),
 'tros_setup':glob.glob('/opt/tros/*/setup.bash'),
 'ros_distro_in_current_shell':os.environ.get('ROS_DISTRO'),
 'disk':run(['df','-h','/']), 'memory':run(['free','-h']),
 'tools':{name:shutil.which(name) for name in ['ros2','colcon','cmake','gcc','g++','git','unzip']},
 'device_paths':{'video':glob.glob('/dev/video*'),'uart':glob.glob('/dev/ttyS*')+glob.glob('/dev/ttyUSB*')+glob.glob('/dev/ttyACM*')},
}
report['modules_in_current_python']={}
for name in ['rclpy','cv2','numpy','serial','onnxruntime','vosk','hobot_dnn','hbm_runtime']:
    try:report['modules_in_current_python'][name]=importlib.util.find_spec(name) is not None
    except (ImportError,ValueError):report['modules_in_current_python'][name]=False
print(json.dumps(report,ensure_ascii=False,indent=2))
PY
