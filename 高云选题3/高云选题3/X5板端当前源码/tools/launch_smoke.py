"""Check all ten nodes remain alive and the installed web service responds."""
from pathlib import Path
import json
import os
import signal
import subprocess
import time
import urllib.request
import urllib.error
import rclpy
from rclpy.node import Node

root=Path(__file__).resolve().parents[1]
os.environ['ROS_DOMAIN_ID']='172';os.environ['ROS_LOCALHOST_ONLY']='1'
expected={'stereo_camera','perception','fire_aim','mission','fpga_bridge','scene','interaction','navigation','voice','telemetry'}
log=(root/'logs/launch_smoke.log').open('w')
p=subprocess.Popen(['ros2','launch','rescue_vision','rescue.launch.py','workspace_dir:='+str(root)],
                   stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
result={'passed':False,'hardware_enabled':False}
rclpy.init();node=Node('launch_check')
try:
    limit=time.monotonic()+15
    while time.monotonic()<limit:
        if p.poll() is not None:raise RuntimeError('Launch exited before all nodes became available')
        rclpy.spin_once(node,timeout_sec=.1)
        names=set(node.get_node_names())
        if expected<=names:
            # A node name may appear while its constructor is still loading models/config.
            # Demand functional HTTP readiness and a stable interval, not just discovery.
            try:
                urllib.request.urlopen('http://127.0.0.1:8088/api/state',timeout=.5)
            except urllib.error.HTTPError as exc:
                if exc.code != 401:continue
            except (OSError,TimeoutError):continue
            token=Path('~/.local/state/rescue_robot/dashboard.token').expanduser().read_text().strip()
            req=urllib.request.Request('http://127.0.0.1:8088/api/state',headers={'X-Rescue-Token':token})
            with urllib.request.urlopen(req,timeout=2) as response:
                payload=json.loads(response.read())
            stable_until=time.monotonic()+2
            while time.monotonic()<stable_until:
                rclpy.spin_once(node,timeout_sec=.1)
                if p.poll() is not None:raise RuntimeError('Launch stopped during stable-ready check')
            if not expected<=set(node.get_node_names()):raise RuntimeError('A node disappeared after initial discovery')
            with urllib.request.urlopen('http://127.0.0.1:8088/',timeout=2) as response:
                page=response.read().decode('utf-8')
            if '消防' not in page:raise RuntimeError('Installed web assets missing')
            result.update(passed=True,nodes=sorted(expected),stable_ready_s=2,
                http_auth_checked=True,http_state_received=True,installed_web_assets=True);break
    if not result['passed']:raise RuntimeError('Missing nodes: '+repr(expected-set(node.get_node_names())))
except Exception as exc:
    result.update(passed=False,error=repr(exc))
finally:
    p.send_signal(signal.SIGINT)
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    log.close();node.destroy_node();rclpy.shutdown()
    log_text=(root/'logs/launch_smoke.log').read_text()
    if 'Traceback' in log_text or '[ERROR]' in log_text:result.update(passed=False,error='Launch log contains node errors')
    (root/'reports/launch_smoke.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result)
    if not result['passed']:raise SystemExit(1)
