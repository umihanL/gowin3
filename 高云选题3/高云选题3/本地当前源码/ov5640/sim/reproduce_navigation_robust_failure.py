"""Recompile last delivered RTL and prove the persistent-edge handoff fault."""
from pathlib import Path
import hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[1]
baseline='impl/baseline_navigation_robust_20261025/src/vehicle/vehicle_vision_fusion.v'
bench=(ROOT/'sim/navigation_handoff_tb.sv').read_text().replace(' .route_guidance_valid(trust),.recovery_active(recovery),','')
# The previous design has no public recovery wire; use its observable state.
bench=bench.replace('wire blocked,active,recovery;','wire blocked,active;wire recovery=state==2;')
path=ROOT/'sim/navigation_handoff_old_tb.sv';path.write_text(bench)
exe='sim/navigation_handoff_reproduction.vvp';logfile=ROOT/'sim/navigation_robust_old_failure.log'
p=subprocess.run(['iverilog','-g2012','-s','navigation_handoff_tb','-o',exe,str(path),baseline],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
assert p.returncode==0,p.stdout
p=subprocess.run(['vvp',exe],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=120)
logfile.write_text(p.stdout)
assert p.returncode!=0 and 'MAP_HANDOFF_FAILED' in p.stdout,p.stdout
result=dict(source=baseline,source_sha256=hashlib.sha256((ROOT/baseline).read_bytes()).hexdigest(),exit_code=p.returncode,log=logfile.relative_to(ROOT).as_posix(),expected_failure='MAP_HANDOFF_FAILED')
(ROOT/'sim/navigation_robust_reproduction.json').write_text(json.dumps(result,indent=2)+'\n')
print(p.stdout,end='');print('PASS last delivered hardware defect reproduced',flush=True)
