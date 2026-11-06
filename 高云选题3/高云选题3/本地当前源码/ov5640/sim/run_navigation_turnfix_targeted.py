"""Targeted controls and existing navigation behavior, with immutable logs."""
from pathlib import Path
import concurrent.futures,hashlib,json,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
fusion=['src/vehicle/vehicle_vision_fusion.v']
plant=[p.relative_to(ROOT).as_posix() for p in (ROOT/'src/vehicle').glob('*.v')]+['src/imu/imu_math.v']
cases=[('ownership','navigation_turn_ownership_tb',[],fusion),
 ('branch','navigation_branch_direction_tb',[],fusion),
 ('next_entry','navigation_next_entry_tb',[],fusion),
 ('handoff','navigation_handoff_tb',[],fusion),
 ('preload','navigation_start_preload_tb',[],plant),
 ('missing_lane','navigation_start_preload_tb',['-Pnavigation_start_preload_tb.NO_LANE=1'],plant),
 ('junction_fit','navigation_start_preload_tb',['-Pnavigation_start_preload_tb.JUNCTION_FIT=1'],plant),
 ('entry_observer','visual_entry_observer_tb',[],['src/vehicle/vehicle_visual_localizer.v'])]
cases += [('authority'+str(i),'navigation_authority_tb',[f'-Pnavigation_authority_tb.CASE={i}'],fusion) for i in range(4)]
def run(case):
 name,top,params,sources=case;start=time.monotonic();out=f'sim/navigation_turnfix_{name}.vvp';log=ROOT/f'sim/navigation_turnfix_{name}.log'
 inputs={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in [*sources,f'sim/{top}.sv','src/vehicle/vehicle_config.vh']}
 with log.open('w') as f:
  for cmd in (['iverilog','-g2012','-s',top,*params,'-o',out,f'sim/{top}.sv',*sources],['vvp',out]):
   p=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=600)
   if p.returncode:print('FAIL',name,log.read_text()[-1800:],flush=True);return dict(name=name,passed=False)
 print('PASS',name,flush=True)
 return dict(name=name,passed=True,seconds=round(time.monotonic()-start,2),inputs=inputs,log=log.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(log.read_bytes()).hexdigest())
if __name__=='__main__':
 with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(run,cases))
 (ROOT/'sim/navigation_turnfix_targeted_runs.json').write_text(json.dumps(results,indent=2)+'\n')
 raise SystemExit(not all(r['passed'] for r in results))
