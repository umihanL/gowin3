"""Settings, response and exact baseline comparison regressions."""
from pathlib import Path
import subprocess, argparse, json, hashlib
from PIL import Image
r=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--snapshot',action='store_true');args=p.parse_args()
assert (r/'sim/motor_pi_before_settings.v').is_file(), 'Missing historical regression fixture'
cases={
 'vehicle_settings_tb':['sim/vehicle_settings_tb.sv',*[str(p.relative_to(r)) for p in (r/'src/navigation').glob('*.v')],
                        'src/vehicle/motor/ps2_test_control.v','src/imu/imu_math.v'],
 'motor_response_tb':['sim/motor_response_tb.sv','sim/motor_pi_before_settings.v',
                      'src/vehicle/motor/motor_pi.v','src/vehicle/motor/motor_target_counts.v']}
with (r/'sim/settings_test_results.txt').open('w',encoding='utf-8') as log:
 for top,files in cases.items():
  for cmd in (['iverilog','-g2012','-s',top,'-I','src/navigation','-o','sim/'+top+'.vvp',*files],
              ['vvp','sim/'+top+'.vvp',*(['+snapshot'] if args.snapshot and top=='vehicle_settings_tb' else [])]):
   q=subprocess.run(cmd,cwd=r,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=600)
   print(q.stdout,end='',flush=True);log.write(q.stdout);log.flush();q.check_returncode()
if args.snapshot:
 for name in ('home','modes','keypad','drive'):
  Image.open(r/f'sim/settings_{name}.ppm').save(r/f'docs/settings_{name}.png')
