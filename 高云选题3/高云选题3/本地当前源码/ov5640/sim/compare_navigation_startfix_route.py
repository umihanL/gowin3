"""Measure the previous release on the same independently reconstructed route."""
from pathlib import Path
import subprocess,json,hashlib
ROOT=Path(__file__).resolve().parents[1]
base='impl/baseline_navigation_startfix_20261027/'
sources=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v']
sources=[base+n if n in ('src/vehicle/vehicle_motion.v','src/vehicle/vehicle_vision_fusion.v') else n for n in sources]
inputs={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in [*sources,'sim/navigation_route_camera_tb.sv','sim/navigation_startfix_reported_route.csv']}
with (ROOT/'sim/navigation_startfix_reported_previous.log').open('w') as f:
 p=subprocess.run(['iverilog','-g2012','-s','navigation_route_camera_tb','-Pnavigation_route_camera_tb.CAMERA_MODE=2',
                   '-Pnavigation_route_camera_tb.LOCALIZE=1','-o','sim/navigation_startfix_reported_previous.vvp',
                   'sim/navigation_route_camera_tb.sv',*sources],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
 assert p.returncode==0
 p=subprocess.run(['vvp','sim/navigation_startfix_reported_previous.vvp','+ROUTE=sim/navigation_startfix_reported_route.csv',
                   '+CSV=sim/navigation_startfix_reported_previous_trace.csv'],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=600)
print('Previous release route simulation exit:',p.returncode,flush=True)
(ROOT/'sim/navigation_startfix_reported_previous.json').write_text(json.dumps(dict(
    exit_code=p.returncode,inputs=inputs,log='sim/navigation_startfix_reported_previous.log',
    sha256=hashlib.sha256((ROOT/'sim/navigation_startfix_reported_previous.log').read_bytes()).hexdigest()),indent=2)+'\n')
