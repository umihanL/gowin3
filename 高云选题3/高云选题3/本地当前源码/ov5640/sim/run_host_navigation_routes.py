"""Actual map route regressions with independent vehicle and partial camera."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import hashlib,json,re,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
sources=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v']
cases=[dict(route=name,edges=edge,speed=160,camera=0,handback=0) for name in ('outer','junction','s_forward','s_reverse','photo') for edge in (0,1,2)]
cases += [dict(route=name,edges=1,speed=160,camera=0,handback=1) for name in ('s_forward_exit','s_reverse_exit')]
cases += [dict(route=name,edges=0,speed=160,camera=2,handback=0) for name in ('outer','junction','s_forward','s_reverse')]
cases += [dict(route='junction',edges=2,speed=160,camera=1,handback=0)]
cases += [dict(route=name,edges=1,speed=speed,camera=0,handback=0) for name,speed in (('outer',80),('junction',60))]
cases += [dict(route='s_forward',edges=1,speed=160,camera=0,handback=0,startup_gap=350)]
def run(case):
 name='{route}_edge{edges}_speed{speed}_camera{camera}'.format(**case)
 if case.get('startup_gap'):name+='_startup_gap'
 logfile=ROOT/f'sim/host_navigation_route_{name}.log';output=f'sim/host_navigation_route_{name}.vvp'
 params=[f'-Pnavigation_route_camera_tb.{key}={case[field]}' for key,field in [('EDGE_MODE','edges'),('NAV_SPEED','speed'),('CAMERA_MODE','camera'),('EXPECT_HANDBACK','handback')]]
 params += [f'-Pnavigation_route_camera_tb.STARTUP_GAP_TICKS={case.get("startup_gap",0)}']
 commands=[['iverilog','-g2012','-s','navigation_route_camera_tb',*params,'-o',output,'sim/navigation_route_camera_tb.sv',*sources],['vvp',output,f'+ROUTE=sim/navigation_robust_{case["route"]}_route.csv',f'+CSV=sim/host_navigation_route_{name}_trace.csv']]
 started=time.monotonic()
 with logfile.open('w',encoding='utf-8') as log:
  for cmd in commands:
   p=subprocess.run(cmd,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=600)
   log.write(p.stdout);log.flush()
   if p.returncode:raise RuntimeError(f'{name}: exit={p.returncode}, see {logfile}')
 match=re.search(r'PASS actual graph navigation.*count=(\d+) maxError=([\d.]+) fallbackTicks=(\d+) handbacks=(\d+)',p.stdout)
 assert match,(name,p.stdout)
 result=dict(**case,name=name,passed=True,seconds=round(time.monotonic()-started,2),samples=int(match[1]),max_nose_error_mm=float(match[2]),fallback_ticks=int(match[3]),handbacks=int(match[4]),log=logfile.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(logfile.read_bytes()).hexdigest())
 print('PASS',name,'noseError',match[2],'handbacks',match[4],flush=True)
 return result
if __name__=='__main__':
 with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(run,cases))
 (ROOT/'sim/host_navigation_route_runs.json').write_text(json.dumps(results,indent=2)+'\n')
 print('PASS actual graph suite:',len(results),'runs',flush=True)
