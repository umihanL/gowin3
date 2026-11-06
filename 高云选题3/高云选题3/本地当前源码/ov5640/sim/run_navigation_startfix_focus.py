"""Launch acquisition, branch direction and handoff checks for this release."""
from pathlib import Path
import hashlib,json,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
cases=['navigation_start_acquire_tb','navigation_branch_direction_tb','navigation_next_entry_tb',
       'navigation_handoff_tb','navigation_response_tb','navigation_corner_tb','vision_fusion_tb',
       'navigation_start_preload_tb']
results=[]
for top in cases:
 log=f'sim/navigation_startfix_{top}.log';out=f'sim/navigation_startfix_{top}.vvp'
 rtl=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v'] if top=='navigation_start_preload_tb' else ['src/vehicle/vehicle_vision_fusion.v']
 sources=[f'sim/{top}.sv',*rtl,'src/vehicle/vehicle_config.vh']
 started=time.monotonic()
 with (ROOT/log).open('w') as f:
  for cmd in (['iverilog','-g2012','-s',top,'-o',out,*sources[:-1]],['vvp',out]):
   p=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=180)
   assert p.returncode==0,(top,log)
 text=(ROOT/log).read_text();assert 'PASS ' in text and '$finish called' in text
 results.append(dict(top=top,passed=True,log=log,seconds=round(time.monotonic()-started,3),
                     inputs={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in sources},
                     sha256=hashlib.sha256((ROOT/log).read_bytes()).hexdigest()))
 print('PASS',top,flush=True)
(ROOT/'sim/navigation_startfix_focus_runs.json').write_text(json.dumps(results,indent=2)+'\n')
