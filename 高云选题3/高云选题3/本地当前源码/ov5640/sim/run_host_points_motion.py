"""Replay the routes emitted by the actual UART/UI planning integration test."""
from pathlib import Path
import csv,hashlib,json,re,subprocess,time
R=Path(__file__).resolve().parents[1]
sha=lambda n:hashlib.sha256((R/n).read_bytes()).hexdigest()
sources=['src/vehicle/'+n+'.v' for n in ('vehicle_motion','vehicle_tracker','vehicle_odometry','vehicle_sincos','vehicle_route_profile','vehicle_bypass')]+['src/imu/imu_math.v']
inputs={n:sha(n) for n in ['sim/inertial_zebra_route_tb.sv',*sources,*[p.relative_to(R).as_posix() for p in (R/'src').rglob('*.vh')]]}
subprocess.run(['iverilog','-g2012','-s','inertial_zebra_route_tb','-I','src/navigation','-o','sim/host_points_motion.vvp','sim/inertial_zebra_route_tb.sv',*sources],cwd=R,check=True)
results=[]
for n in range(1,10):
 deadline=time.monotonic()+1800
 while True:
  p=R/f'sim/host_points_{n}.log'
  if p.exists() and 'PASS numbered point' in p.read_text():break
  if time.monotonic()>deadline:raise TimeoutError(f'waiting for point {n} planning test')
  time.sleep(1)
 rows=list(csv.DictReader((R/f'sim/host_point_{n}_route.csv').open()))
 route=f'sim/host_point_{n}_motion_input.csv'
 (R/route).write_text(''.join(f"{r['x_q8']},{r['y_q8']},{r['lock']}\n" for r in rows))
 log=f'sim/host_point_{n}_motion.log';trace=f'sim/host_point_{n}_motion_trace.csv';start=time.monotonic()
 with (R/log).open('w') as f:
  q=subprocess.run(['vvp','-i','sim/host_points_motion.vvp',f'+ROUTE={route}',f'+CSV={trace}'],cwd=R,stdout=f,stderr=subprocess.STDOUT,timeout=600)
 text=(R/log).read_text();assert q.returncode==0 and 'PASS inertial route' in text,(n,text)
 m=re.search(r'status=(\d+) index=(\d+)/(\d+) maxCross=([\d.]+) maxPose=([\d.]+)',text);assert m
 metrics=dict(zip(['status','index','count','max_cross_mm','max_pose_mm'],map(float,m.groups())))
 results.append(dict(point=n,passed=True,seconds=round(time.monotonic()-start,3),metrics=metrics,route=route,route_sha256=sha(route),trace=trace,log=log,log_sha256=sha(log)))
 print('PASS motion point',n,metrics,flush=True)
assert all(sha(n)==h for n,h in inputs.items())
(R/'sim/host_points_motion_runs.json').write_text(json.dumps(dict(inputs=inputs,cases=results),indent=2)+'\n')
