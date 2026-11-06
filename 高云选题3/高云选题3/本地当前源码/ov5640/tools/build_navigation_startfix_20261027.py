"""Build precisely the navigation launch/corner fix, preserving the old release."""
from pathlib import Path
import datetime,hashlib,json,os,subprocess,time
ROOT=Path(__file__).resolve().parents[1];LABEL='navigation_startfix_20261027'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
baseline=ROOT/'impl/baseline_navigation_startfix_20261027'
before=json.loads((baseline/'manifest.json').read_text())
assert all(sha(baseline/n)==h for n,h in before.items()),'Baseline snapshot corrupted'
inputs=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in ('.v','.vo','.vh','.mem','.cst','.sdc','.ipc','.mod')]
inputs += [ROOT/'ov5640.gprj',ROOT/'build.tcl']
files={p.relative_to(ROOT).as_posix():sha(p) for p in inputs}
changed=sorted(n for n,h in files.items() if before.get(n)!=h)
assert set(changed)=={'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_vision_fusion.v'},changed
old_sha=sha(ROOT/'impl/pnr/ov5640.fs');assert old_sha==before['impl/pnr/ov5640.fs']
frozen=dict(frozen_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),files=files,changed=changed)
(ROOT/f'impl/{LABEL}_inputs.json').write_text(json.dumps(frozen,indent=2)+'\n')
env=os.environ.copy();env.pop('GOWIN_INCREMENTAL_PR',None)
start=time.monotonic()
with (ROOT/f'impl/{LABEL}_build.log').open('w',encoding='utf-8') as log:
 result=subprocess.run(['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe','build.tcl'],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
metrics=dict(exit_code=result.returncode,seconds=round(time.monotonic()-start,3),
 mode='standard_syn_and_pnr',incremental_checkpoint_used=False,
 reason='Existing incremental_pnr_slow.flag records that incremental matching was slower.',
 previous_bitstream_sha256=old_sha,changed_source_files=changed)
(ROOT/f'impl/{LABEL}_build_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
assert all(sha(ROOT/n)==h for n,h in files.items()),'Hardware sources changed during build'
print(json.dumps(metrics,indent=2),flush=True)
raise SystemExit(result.returncode)
