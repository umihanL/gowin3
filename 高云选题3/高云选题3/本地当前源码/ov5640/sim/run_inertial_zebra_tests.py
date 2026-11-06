"""Source-bound checks for pure navigation and zebra-only camera control."""
from pathlib import Path
import argparse,hashlib,json,subprocess,time
R=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--only',nargs='+');p.add_argument('--snapshot',action='store_true');a=p.parse_args()
vehicle=[x.relative_to(R).as_posix() for x in (R/'src/vehicle').rglob('*.v')]+['src/imu/imu_math.v']
nav=[x.relative_to(R).as_posix() for x in (R/'src/navigation').glob('*.v')]
vision=['src/portrait/vision_video.v','src/portrait/vision_zebra.v']
cases=[('modes','navigation_modes_filter_tb',vehicle,[]),
       ('avoidance','navigation_avoidance_tb',vehicle,[]),
       ('zebra','vision_zebra_tb',['src/portrait/vision_zebra.v'],[]),
       ('camera','inertial_zebra_camera_tb',vision+vehicle,[]),
       ('settings','vehicle_settings_tb',nav+['src/vehicle/motor/ps2_test_control.v','src/imu/imu_math.v'],['+avoidance_snapshot'] if a.snapshot else []),
       ('uart','vehicle_host_link_tb',vehicle,[]),
       ('host','host_navigation_integration_tb',vehicle,[])]
sha=lambda n:hashlib.sha256((R/n).read_bytes()).hexdigest()
results=[]
for name,top,sources,runtime in cases:
 if a.only and name not in a.only:continue
 deps=set([f'sim/{top}.sv',*sources,*[x.relative_to(R).as_posix() for x in (R/'src').rglob('*.vh')]])
 frozen={n:sha(n) for n in sorted(deps)};start=time.monotonic()
 log=f'sim/inertial_zebra_{name}.log';out=f'sim/inertial_zebra_{name}.vvp'
 with (R/log).open('w',encoding='utf-8') as f:
  for command in (['iverilog','-g2012','-s',top,'-I','src/navigation','-I','src/vga','-o',out,f'sim/{top}.sv',*sources],['vvp','-i',out,*runtime]):
   run=subprocess.run(command,cwd=R,stdout=f,stderr=subprocess.STDOUT,timeout=1200)
   assert run.returncode==0,log
 text=(R/log).read_text();assert 'PASS ' in text and '$finish called' in text and 'FATAL:' not in text,log
 assert frozen=={n:sha(n) for n in sorted(deps)},'Test sources changed'
 results.append(dict(name=name,top=top,passed=True,seconds=round(time.monotonic()-start,3),log=log,log_sha256=sha(log),inputs=frozen))
 print('PASS',name,results[-1]['seconds'],'seconds',flush=True)
record=R/'sim/inertial_zebra_runs.json'
if a.only and record.exists():
 names={r['name'] for r in results};results=[r for r in json.loads(record.read_text()) if r['name'] not in names]+results
record.write_text(json.dumps(results,indent=2)+'\n')
