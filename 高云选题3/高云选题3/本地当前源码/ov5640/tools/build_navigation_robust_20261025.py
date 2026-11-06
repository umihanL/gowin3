"""Freeze the navigation-only fix and build with the measured faster mode."""
from pathlib import Path
import datetime,hashlib,json,os,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'impl/baseline_navigation_robust_20261025'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
baseline=json.loads((BASE/'manifest.json').read_text())
changed=[n for n,h in baseline.items() if n.startswith('src/') and sha(ROOT/n)!=h]
assert set(changed)=={'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_vision_fusion.v','src/vehicle/vehicle_route_profile.v'},changed
assert all(sha(BASE/n)==h for n,h in baseline.items()),'Backup integrity failed'
inputs=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in ('.v','.vo','.vh','.mem','.cst','.sdc','.ipc','.mod')]
inputs += [ROOT/'ov5640.gprj',ROOT/'build.tcl']
manifest=dict(frozen_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),files={p.relative_to(ROOT).as_posix():sha(p) for p in inputs})
(ROOT/'impl/navigation_robust_20261025_inputs.json').write_text(json.dumps(manifest,indent=2)+'\n')
environment=os.environ.copy();environment.pop('GOWIN_INCREMENTAL_PR',None)
start=time.monotonic();log=ROOT/'impl/navigation_robust_20261025_build.log'
with log.open('w',encoding='utf-8') as output:
 result=subprocess.run(['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe','build.tcl'],cwd=ROOT,env=environment,stdout=output,stderr=subprocess.STDOUT)
metrics=dict(exit_code=result.returncode,seconds=round(time.monotonic()-start,3),mode='standard_syn_and_pnr',incremental_checkpoint_used=False,checkpoint_sha256=baseline.get('impl/pnr/ov5640.pr'),changed_source_files=changed)
(ROOT/'impl/navigation_robust_20261025_build_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print(json.dumps(metrics,indent=2),flush=True)
raise SystemExit(result.returncode)
