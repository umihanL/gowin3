"""Tighter independent nose-position guard than the historical 10mm bound."""
from pathlib import Path
import hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[1]
original=(ROOT/'sim/vision_turn_motion_tb.sv').read_text()
assert 'wx+219*$cos(angle)<790' in original
bench=ROOT/'sim/host_physical_entry_tb.sv'
bench.write_text(original.replace('wx+219*$cos(angle)<790','wx+219*$cos(angle)<798'))
sources=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v']
results=[]
for side in (-1,1):
 log=f'sim/host_physical_entry_{side}.log';vvp=f'sim/host_physical_entry_{side}.vvp'
 with (ROOT/log).open('w',encoding='utf-8') as output:
  for cmd in (['iverilog','-g2012','-s','vision_turn_motion_tb',f'-Pvision_turn_motion_tb.SIDE={side}',
               '-Pvision_turn_motion_tb.USE_PROFILE=1','-o',vvp,str(bench),*sources],['vvp',vvp]):
   p=subprocess.run(cmd,cwd=ROOT,stdout=output,stderr=subprocess.STDOUT,timeout=300)
   assert p.returncode==0,log
 assert 'PASS integrated' in (ROOT/log).read_text(),log
 results.append(dict(side=side,passed=True,early_guard_mm=2,log=log,sha256=hashlib.sha256((ROOT/log).read_bytes()).hexdigest()))
 print('PASS physical entry guard side',side,flush=True)
(ROOT/'sim/host_physical_entry_runs.json').write_text(json.dumps(results,indent=2)+'\n')
