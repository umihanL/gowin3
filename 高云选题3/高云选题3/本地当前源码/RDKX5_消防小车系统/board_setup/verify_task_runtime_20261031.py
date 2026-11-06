"""Read-only deployed-file and generated ROS interface checks on X5."""
import hashlib
import json
from pathlib import Path
import sys
root = Path('/root/rescue/ros2_ws').resolve()
stage = Path('/root/rescue/deployment_20261031/task_runtime')
manifest = json.loads((stage/'final_manifest.json').read_text())
checked, bad, preserved = [], [], []
for entry in manifest['files']:
    name = entry['path']
    if entry['policy'] == 'create_if_missing':
        preserved.append(name)
        continue
    p = root/name
    if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != entry['sha256']:
        bad.append(name)
    checked.append(name)
installed = root/'install/rescue_vision/lib/python3.10/site-packages/rescue_vision'
for name in ('fire_aim_node.py','fixed_camera_aim.py','mission_node.py','stereo_depth_node.py',
             'pd42_protocol.py','pd42_actuator_io.py','mixed_actuator_io.py','voice_node.py'):
    if (installed/name).read_bytes() != (root/'src/rescue_vision/rescue_vision'/name).read_bytes():
        bad.append('installed:'+name)
interface_files = list((root/'install/rescue_interfaces').rglob('_detection_array.py'))
interface_ready = bool(interface_files and all('stereo_calibration_id' in p.read_text() for p in interface_files))
uart = json.loads((root/'config/extension_uart.json').read_text())
result=dict(checked=len(checked), preserved=preserved, mismatches=bad,
            generated_stereo_interface=interface_ready, fpga_uart_enabled=uart['enabled'], fpga_port=uart['port'],
            installed_callback_matches=True if not any('installed:' in x for x in bad) else False,
            read_only=True, motion_commands_sent=False)
(stage/'final_files_verified.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result))
sys.exit(1 if bad or not interface_ready else 0)
