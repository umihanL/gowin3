"""Freeze navigation-only inputs and build the reviewed bitstream."""
from pathlib import Path
import datetime
import hashlib
import json
import os
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
LABEL='navigation_followup_20261026'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
baseline=ROOT/'impl/baseline_navigation_followup_20261026'
manifest=json.loads((baseline/'manifest.json').read_text(encoding='utf-8'))
assert all(sha(baseline/name)==digest for name,digest in manifest.items()),'Rollback snapshot integrity failed'
previous=json.loads((ROOT/'impl/navigation_entry_20261026_inputs.json').read_text(encoding='utf-8'))['files']
inputs=[p for p in (ROOT/'src').rglob('*') if p.is_file() and
        p.suffix in ('.v','.vo','.vh','.mem','.cst','.sdc','.ipc','.mod')]
inputs += [ROOT/'ov5640.gprj',ROOT/'build.tcl']
files={p.relative_to(ROOT).as_posix():sha(p) for p in inputs}
changed=[name for name,digest in files.items() if previous.get(name)!=digest]
allowed={'src/vehicle/vehicle_motion.v','src/vehicle/vehicle_route_profile.v',
         'src/vehicle/vehicle_subsystem.v','src/vehicle/vehicle_vision_fusion.v'}
assert set(changed)==allowed,changed
previous_bitstream_sha=sha(ROOT/'impl/pnr/ov5640.fs')
verified_bitstreams={manifest['impl/pnr/ov5640.fs']}
full_backup=ROOT/'impl/navigation_followup_full_pnr_backup_20261026/manifest.json'
if full_backup.exists():
    verified_bitstreams.add(json.loads(full_backup.read_text(encoding='utf-8'))['ov5640.fs'])
assert previous_bitstream_sha in verified_bitstreams, 'Current bitstream differs from verified backups'
(ROOT/f'impl/{LABEL}_inputs.json').write_text(json.dumps(
    dict(frozen_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),files=files),
    indent=2)+'\n',encoding='utf-8')

environment=os.environ.copy()
environment.pop('GOWIN_INCREMENTAL_PR',None)
start=time.monotonic()
with (ROOT/f'impl/{LABEL}_build.log').open('w',encoding='utf-8') as output:
    result=subprocess.run(['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe','build.tcl'],
                          cwd=ROOT,env=environment,stdout=output,stderr=subprocess.STDOUT)
metrics=dict(exit_code=result.returncode,seconds=round(time.monotonic()-start,3),
             mode='standard_syn_and_pnr',incremental_checkpoint_used=False,
             mode_reason='Measured incremental route matching exceeded 920 seconds; use the previously faster standard PNR flow',
             previous_bitstream_sha256=previous_bitstream_sha,changed_source_files=changed)
(ROOT/f'impl/{LABEL}_build_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n',encoding='utf-8')
assert all(sha(ROOT/name)==digest for name,digest in files.items()),'Hardware inputs changed during build'
print(json.dumps(metrics,indent=2),flush=True)
raise SystemExit(result.returncode)
