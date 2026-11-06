"""Retain reproducible evidence that both public-port bugs fail on the old RTL."""
from pathlib import Path
import hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[1]
old='impl/baseline_navigation_startfix_20261027/src/vehicle/vehicle_vision_fusion.v'
results=[]
for top,expected in [('navigation_start_acquire_tb','START_NOISE_WAIT'),
                     ('navigation_branch_direction_tb','committed branch reversed mid-turn')]:
 log=f'sim/navigation_startfix_before_{top}.log';out=f'sim/navigation_startfix_before_{top}.vvp'
 with (ROOT/log).open('w') as f:
  p=subprocess.run(['iverilog','-g2012','-s',top,'-o',out,f'sim/{top}.sv',old],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
  assert p.returncode==0
  p=subprocess.run(['vvp',out],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
 assert p.returncode!=0 and expected in (ROOT/log).read_text(),log
 results.append(dict(top=top,expected_failure=True,message=expected,log=log,
                     inputs={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in [old,f'sim/{top}.sv']},
                     sha256=hashlib.sha256((ROOT/log).read_bytes()).hexdigest()))
 print('Reproduced previous-release fault:',expected,flush=True)
(ROOT/'sim/navigation_startfix_before_runs.json').write_text(json.dumps(results,indent=2)+'\n')
