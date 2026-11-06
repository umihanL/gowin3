"""Freeze the navigation authority fix and run the measured faster implementation."""
from pathlib import Path
import datetime, hashlib, json, os, subprocess, time

ROOT = Path(__file__).resolve().parents[1]
LABEL = 'navigation_entry_20261026'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
old = json.loads((ROOT/'impl/navigation_authority_20261026_inputs.json').read_text())['files']
backup = ROOT/'impl/baseline_navigation_entry_20261026'
backup_manifest = json.loads((backup/'manifest.json').read_text())
assert all(sha(backup/n) == h for n, h in backup_manifest.items()), 'Backup integrity failed'
inputs = [p for p in (ROOT/'src').rglob('*') if p.is_file() and
          p.suffix in ('.v', '.vo', '.vh', '.mem', '.cst', '.sdc', '.ipc', '.mod')]
inputs += [ROOT/'ov5640.gprj', ROOT/'build.tcl']
files = {p.relative_to(ROOT).as_posix(): sha(p) for p in inputs}
changed = [n for n, h in files.items() if old.get(n) != h]
allowed = {'src/vehicle/vehicle_vision_fusion.v','src/vehicle/vehicle_motion.v','src/vehicle/vehicle_route_profile.v','src/vehicle/vehicle_subsystem.v'}
assert set(changed) == allowed, changed
assert sha(ROOT/'impl/pnr/ov5640.fs') == backup_manifest['impl/pnr/ov5640.fs'], 'Preserve current bitstream before rebuilding'
manifest = dict(frozen_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), files=files)
(ROOT/f'impl/{LABEL}_inputs.json').write_text(json.dumps(manifest, indent=2)+'\n')
environment = os.environ.copy()
environment.pop('GOWIN_INCREMENTAL_PR', None)
start = time.monotonic()
with (ROOT/f'impl/{LABEL}_build.log').open('w', encoding='utf-8') as output:
    result = subprocess.run(['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe', 'build.tcl'],
                            cwd=ROOT, env=environment, stdout=output, stderr=subprocess.STDOUT)
metrics = dict(exit_code=result.returncode, seconds=round(time.monotonic()-start, 3),
               mode='standard_syn_and_pnr', incremental_checkpoint_used=False,
               mode_reason='Prior measured incremental matching was slower for this nearly full design',
               previous_bitstream_sha256=backup_manifest['impl/pnr/ov5640.fs'], changed_source_files=changed)
(ROOT/f'impl/{LABEL}_build_metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
assert all(sha(ROOT/n) == h for n, h in files.items()), 'Inputs changed during build'
print(json.dumps(metrics, indent=2), flush=True)
raise SystemExit(result.returncode)
