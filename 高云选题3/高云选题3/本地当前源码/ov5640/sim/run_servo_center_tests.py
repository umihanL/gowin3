"""Source-bound tests of centre settings and the actual actuator output."""
from pathlib import Path
import argparse,hashlib,json,subprocess,time
from concurrent.futures import ThreadPoolExecutor,as_completed
R=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--only',nargs='+');p.add_argument('--snapshot',action='store_true');options=p.parse_args()
vehicle=[p.relative_to(R).as_posix() for p in (R/'src/vehicle').rglob('*.v')]+['src/imu/imu_math.v']
nav=[p.relative_to(R).as_posix() for p in (R/'src/navigation').glob('*.v')]
cases=[('servo_pwm_tenths_tb',['src/vehicle/motor/servo_pwm.v'],[]),
 ('servo_center_tb',vehicle+['src/navigation/vehicle_settings.v'],[]),
 ('vehicle_settings_tb',nav+['src/vehicle/motor/ps2_test_control.v','src/imu/imu_math.v'],['+servo_snapshot'] if options.snapshot else []),
 ('vehicle_io_tb',vehicle+['sim/ps2_receiver_model.sv','sim/vl53l0x_sensor_model.v','sim/vl53l1x_sensor_model.v'],[])]
sha=lambda n:hashlib.sha256((R/n).read_bytes()).hexdigest()
def run(case):
 top,sources,args=case
 deps=sorted(set([f'sim/{top}.sv',*sources,*[p.relative_to(R).as_posix() for p in (R/'src').rglob('*.vh')],
  'src/vehicle/servo_offset.mem','src/vehicle/rear_ratio.mem','src/navigation/nav_font_rows.mem']))
 frozen={n:sha(n) for n in deps};start=time.monotonic()
 log=f'sim/servo_center_{top}.log';out=f'sim/servo_center_{top}.vvp'
 with (R/log).open('w',encoding='utf-8') as stream:
  for cmd in (['iverilog','-g2012','-s',top,'-I','src/navigation','-o',out,f'sim/{top}.sv',*sources],['vvp','-i',out,*args]):
   result=subprocess.run(cmd,cwd=R,stdout=stream,stderr=subprocess.STDOUT,timeout=1200)
   assert result.returncode==0,log
 text=(R/log).read_text();assert 'PASS ' in text and '$finish called' in text and 'FATAL:' not in text,log
 assert frozen=={n:sha(n) for n in deps},'Sources changed during test'
 record=dict(top=top,passed=True,seconds=round(time.monotonic()-start,3),log=log,log_sha256=sha(log),inputs=frozen)
 print('PASS',top,record['seconds'],'seconds',flush=True)
 return record
with ThreadPoolExecutor(max_workers=3) as pool:
 results=[f.result() for f in as_completed([pool.submit(run,case) for case in cases if not options.only or case[0] in options.only])]
if options.only and (R/'sim/servo_center_20261030_runs.json').exists():
 tops={r['top'] for r in results}
 results=[r for r in json.loads((R/'sim/servo_center_20261030_runs.json').read_text()) if r['top'] not in tops]+results
(R/'sim/servo_center_20261030_runs.json').write_text(json.dumps(results,indent=2)+'\n')
