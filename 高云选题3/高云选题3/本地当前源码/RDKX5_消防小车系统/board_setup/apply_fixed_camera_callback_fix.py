"""Apply the reviewed final callback fix after the full board build, before restart."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

root = Path('/root/rescue/ros2_ws').resolve()
stage = Path('/root/rescue/deployment_20261031/task_runtime')
source = stage / 'fire_aim_node.final.py'
expected = 'e4ca2725cf2f9188f7193946c88ad92788a38eea1f3c63f86277d85bfb14aec6'
blob = source.read_bytes()
if hashlib.sha256(blob).hexdigest() != expected:
    raise RuntimeError('Final callback SHA mismatch')
compile(blob, str(source), 'exec')
state = subprocess.run(['systemctl', 'is-active', 'rescue-task-interaction'], capture_output=True, text=True)
if state.stdout.strip() not in ('inactive', 'failed'):
    raise RuntimeError('Task service must be stopped')
paths = [root/'src/rescue_vision/rescue_vision/fire_aim_node.py',
         root/'install/rescue_vision/lib/python3.10/site-packages/rescue_vision/fire_aim_node.py']
for i, path in enumerate(paths):
    if not path.is_file() or not path.resolve().is_relative_to(root):
        raise RuntimeError('Unexpected installed path')
    shutil.copy2(path, stage / ('fire_aim_node.before_callback_fix_' + str(i) + '.py'))
    temporary = path.with_suffix('.py.final-update')
    temporary.write_bytes(blob)
    temporary.replace(path)
record = dict(sha256=expected, source_and_installed_updated=True,
              movement_requested=False, paths=[str(p) for p in paths])
(stage/'callback_delta.json').write_text(json.dumps(record, indent=2))
print(json.dumps(record))
