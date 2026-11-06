"""Preserve the failed incremental evidence and reuse current synthesis."""
from pathlib import Path
import hashlib
import json
import subprocess
import time
ROOT=Path(__file__).resolve().parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
previous=json.loads((ROOT/'impl/navigation_response_20261025_build_metrics.json').read_text())
assert previous['exit_code']!=0
(ROOT/'impl/navigation_response_20261025_incremental_metrics.json').write_text(json.dumps(previous,indent=2)+'\n')
(ROOT/'impl/navigation_response_20261025_build.log').rename(ROOT/'impl/navigation_response_20261025_incremental.log')
(ROOT/'impl/incremental_pnr_slow.flag').write_text('Incremental matching exceeded 920 seconds on 2026-10-25. Use normal PnR unless explicitly overridden.\n')
manifest_path=ROOT/'impl/navigation_response_20261025_inputs.json'
manifest=json.loads(manifest_path.read_text())
for name,digest in manifest['files'].items():
    if name!='build.tcl':assert sha(ROOT/name)==digest,name
manifest['files']['build.tcl']=sha(ROOT/'build.tcl')
manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
vg=ROOT/'impl/gwsynthesis/ov5640.vg'
netlist_hash=sha(vg)
start=time.monotonic()
with (ROOT/'impl/navigation_response_20261025_build.log').open('w',encoding='utf-8') as log:
    result=subprocess.run(['C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe',
                           'tools/route_navigation_20261025.tcl'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
assert sha(vg)==netlist_hash,'Synthesis changed during route-only fallback'
metrics=dict(exit_code=result.returncode,seconds=round(time.monotonic()-start,3),
    mode='standard_pnr_reusing_current_synthesis',synthesis_netlist_sha256=netlist_hash,
    incremental_attempt=previous,fallback_script_sha256=sha(ROOT/'tools/route_navigation_20261025.tcl'),
    changed_source_files=previous['changed_source_files'])
(ROOT/'impl/navigation_response_20261025_build_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print(json.dumps(metrics,indent=2),flush=True)
raise SystemExit(result.returncode)
