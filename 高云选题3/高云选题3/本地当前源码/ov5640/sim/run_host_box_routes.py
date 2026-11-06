"""Junction from the photograph, both directions and video frame phases."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import subprocess,json,re,hashlib,time
ROOT=Path(__file__).resolve().parents[1]
sources=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v']
cases=[dict(route=name,speed=speed,phase=phase,camera=camera)
       for name in json.loads((ROOT/'sim/host_navigation_box_routes.json').read_text())
       for speed,phase,camera in [(80,0,0),(160,1,0),(160,2,2)]]
def run(c):
 name='{route}_speed{speed}_phase{phase}_camera{camera}'.format(**c)
 output=f'sim/host_navigation_{name}.vvp';log=ROOT/f'sim/host_navigation_{name}.log'
 params=[f'-Pnavigation_route_camera_tb.{key}={c[field]}' for key,field in [('NAV_SPEED','speed'),('CAMERA_PHASE','phase'),('CAMERA_MODE','camera')]]
 start=time.monotonic()
 with log.open('w',encoding='utf-8') as f:
  for cmd in [['iverilog','-g2012','-s','navigation_route_camera_tb',*params,'-o',output,'sim/navigation_route_camera_tb.sv',*sources],
              ['vvp',output,f'+ROUTE=sim/host_navigation_{c["route"]}_route.csv',f'+CSV=sim/host_navigation_{name}_trace.csv']]:
   result=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=600)
   if result.returncode:raise RuntimeError(f'{name}: see {log}')
 text=log.read_text();m=re.search(r'PASS actual graph navigation.*count=(\d+) maxError=([\d.]+) fallbackTicks=(\d+) handbacks=(\d+)',text)
 assert m,name
 r=dict(**c,name=name,passed=True,max_nose_error_mm=float(m[2]),seconds=round(time.monotonic()-start,2),log=log.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(log.read_bytes()).hexdigest())
 print('PASS',name,'noseError',m[2],flush=True);return r
with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(run,cases))
(ROOT/'sim/host_navigation_box_runs.json').write_text(json.dumps(results,indent=2)+'\n')
