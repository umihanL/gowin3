"""Independent bicycle plus camera-ground intersections on actual graph paths.
The two reported junctions are separate assertions, not artificial turn hints.
"""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import csv,hashlib,json,math,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
sources=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v']
cases=[('reported',0,0,0),('reported',-60,0,0),('reported',60,0,0),
 ('outer',0,0,1),('outer',0,80,0),('outer',0,-80,0),
 ('s_forward',0,0,0),('s_reverse',0,0,0)]
def wrap(v):return (v+math.pi)%(2*math.pi)-math.pi
def run(case):
 route,offset,yaw,noise=case;name=f'{route}_long{offset}_yaw{yaw}_noise{noise}';start=time.monotonic()
 path='sim/navigation_startfix_reported_route.csv' if route=='reported' else f'sim/navigation_robust_{route}_route.csv'
 out=f'sim/navigation_turnfix_{name}.vvp';log=ROOT/f'sim/navigation_turnfix_{name}.log';trace=ROOT/f'sim/navigation_turnfix_{name}_trace.csv'
 inputs={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in [*sources,'src/vehicle/vehicle_config.vh','sim/navigation_route_camera_tb.sv',path]}
 params=['-Pnavigation_route_camera_tb.CAMERA_MODE=2','-Pnavigation_route_camera_tb.LOCALIZE=1',
  f'-Pnavigation_route_camera_tb.START_LONG_MM={offset}',f'-Pnavigation_route_camera_tb.START_YAW_TENTHS={yaw}',f'-Pnavigation_route_camera_tb.START_NOISE={noise}']
 with log.open('w') as f:
  for cmd in (['iverilog','-g2012','-s','navigation_route_camera_tb',*params,'-o',out,'sim/navigation_route_camera_tb.sv',*sources],
              ['vvp',out,'+ROUTE='+path,'+CSV='+trace.relative_to(ROOT).as_posix()]):
   p=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=700)
   if p.returncode:print('FAIL',name,log.read_text()[-1200:],flush=True);return dict(name=name,passed=False,log=log.as_posix())
 rows=list(csv.reader(trace.open()));checks={}
 if route=='reported':
  # Actual image-route coordinates: outer upper-right N->W; circled E->S.
  for label,entry_i,exit_i,side in [('upper_right_left',22,58,1),('circled_right',153,169,-1)]:
   a=next(r for r in rows if int(r[4])>=entry_i);b=next(r for r in rows if int(r[4])>=exit_i)
   turn=wrap(float(b[3])-float(a[3]));assert 0.65<side*turn<1.95,(name,label,turn)
   region=[r for r in rows if entry_i<=int(r[4])<exit_i and int(r[11])==1 and side*wrap(float(b[3])-float(r[3]))>0.3]
   assert region,(name,label,'no reached entry samples')
   # Allow the wheel's initial zero crossing, then reject a reversal.
   correct_seen=False
   for r in region:
    if side*int(r[5])>=3:correct_seen=True
    if correct_seen:assert side*int(r[5])>=0,(name,label,'steering reversed',r)
   checks[label]=dict(turn_radians=turn,no_midturn_reversal=True)
 result=dict(name=name,passed=True,seconds=round(time.monotonic()-start,2),max_nose_error_mm=max(float(r[8]) for r in rows),junctions=checks,inputs=inputs,sha256=hashlib.sha256(log.read_bytes()).hexdigest(),log=log.relative_to(ROOT).as_posix(),trace=trace.relative_to(ROOT).as_posix())
 print('PASS',name,'max error',result['max_nose_error_mm'],flush=True);return result
if __name__=='__main__':
 with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(run,cases))
 (ROOT/'sim/navigation_turnfix_routes_runs.json').write_text(json.dumps(results,indent=2)+'\n')
 raise SystemExit(not all(r['passed'] for r in results))
