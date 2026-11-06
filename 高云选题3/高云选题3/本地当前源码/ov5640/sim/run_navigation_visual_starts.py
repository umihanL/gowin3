"""Reported 976cm path plus independently displaced starts on S/outer roads."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import hashlib,json,re,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sources=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v']
cases=[dict(route='s_forward',longitudinal=0,yaw=v) for v in (-80,80)]
cases += [dict(route='s_reverse',longitudinal=0,yaw=0)]

def run(case):
 name=case['route']+'_long'+str(case['longitudinal'])+'_yaw'+str(case['yaw'])
 output=f'sim/navigation_visual_{name}.vvp';log=f'sim/navigation_visual_{name}.log'
 route='sim/navigation_reported_route.csv' if case['route']=='reported' else f'sim/navigation_robust_{case["route"]}_route.csv'
 args=[f'-Pnavigation_route_camera_tb.START_YAW_TENTHS={case["yaw"]}', '-Pnavigation_route_camera_tb.CAMERA_MODE=2','-Pnavigation_route_camera_tb.LOCALIZE=1',
       f'-Pnavigation_route_camera_tb.START_LONG_MM={case["longitudinal"]}']
 started=time.monotonic()
 with (ROOT/log).open('w',encoding='utf-8') as f:
  inputs={n:sha(ROOT/n) for n in [*sources,'src/vehicle/vehicle_config.vh','sim/navigation_route_camera_tb.sv',route]}
  for cmd in (['iverilog','-g2012','-s','navigation_route_camera_tb',*args,'-o',output,'sim/navigation_route_camera_tb.sv',*sources],
              ['vvp',output,'+ROUTE='+route,f'+CSV=sim/navigation_visual_{name}_trace.csv']):
   p=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=600)
   if p.returncode:raise RuntimeError(name+' failed: '+log)
 text=(ROOT/log).read_text();match=re.search(r'count=(\d+) maxError=([\d.]+) fallbackTicks=(\d+) handbacks=(\d+)',text)
 assert match,log
 result=dict(**case,name=name,passed=True,seconds=round(time.monotonic()-started,3),
             max_nose_error_mm=float(match[2]),inputs=inputs,log=log,sha256=hashlib.sha256((ROOT/log).read_bytes()).hexdigest())
 if case['route']=='reported':
  import csv,math
  rows=list(csv.reader((ROOT/f'sim/navigation_visual_{name}_trace.csv').open()))
  entry=next(r for r in rows if 149<=int(r[4])<=151)
  exit=next(r for r in rows if int(r[4])>=161)
  turn=(float(exit[3])-float(entry[3])+math.pi)%(2*math.pi)-math.pi
  assert -2.0<turn < -0.9,(name,'reported junction must physically turn right',turn)
  result['reported_right_turn_radians']=turn
 print('PASS',name,'max error',match[2],flush=True);return result
if __name__=='__main__':
 with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(run,cases))
 (ROOT/'sim/navigation_visual_starts_runs.json').write_text(json.dumps(results,indent=2)+'\n')
