"""Try navigation-only incremental PNR while preserving a verified full build."""
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "impl/baseline_navigation_followup_20261026"
BACKUP = ROOT / "impl/navigation_followup_full_pnr_backup_20261026/pnr"
LABEL = "navigation_followup_20261026_incremental"
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()

def restore(saved):
    for name, digest in saved.items():
        source, target = BACKUP / name, ROOT / "impl/pnr" / name
        assert sha(source) == digest, f"Backup integrity: {name}"
        if target.exists():
            target.chmod(0o666)
        shutil.copy2(source, target)

if "--restore" in sys.argv:
    restore(json.loads((BACKUP.parent / "manifest.json").read_text(encoding="utf-8")))
    print("Verified full-build outputs restored", flush=True)
    raise SystemExit(0)

assert not BACKUP.exists(), f"Backup already exists: {BACKUP}"
manifest = json.loads((BASELINE / "manifest.json").read_text(encoding="utf-8"))
checkpoint_rel = "impl/pnr/ov5640.pr"
assert sha(BASELINE / checkpoint_rel) == manifest[checkpoint_rel]

pnr = ROOT / "impl/pnr"
BACKUP.mkdir(parents=True)
saved = {}
for source in pnr.iterdir():
    if source.is_file():
        target = BACKUP / source.name
        shutil.copy2(source, target)
        saved[source.name] = sha(source)
(BACKUP.parent / "manifest.json").write_text(
    json.dumps(saved, indent=2) + "\n", encoding="utf-8"
)

log = ROOT / f"impl/{LABEL}_build.log"
environment = os.environ.copy()
environment["GOWIN_INCREMENTAL_PR"] = str((BASELINE / checkpoint_rel).resolve())
started = time.monotonic()
build_epoch = time.time()
with log.open("w", encoding="utf-8") as output:
    result = subprocess.run(
        ["C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe", "build.tcl"],
        cwd=ROOT, env=environment, stdout=output, stderr=subprocess.STDOUT,
    )

timing_path = pnr / "ov5640.tr"
timing = timing_path.read_text(encoding="utf-8", errors="replace") if timing_path.exists() else ""
counts = {}
for kind in ("Setup", "Hold"):
    match = re.search(rf"<Numbers of {kind} Violated Endpoints>:(\d+)", timing)
    counts[kind] = int(match.group(1)) if match else None
new_fs = pnr / "ov5640.fs"
fresh_fs = new_fs.exists() and new_fs.stat().st_mtime >= build_epoch
passed = result.returncode == 0 and fresh_fs and counts == {"Setup": 0, "Hold": 0}

if not passed:
    restore(saved)

metrics = {
    "exit_code": result.returncode,
    "seconds": round(time.monotonic() - started, 3),
    "mode": "incremental_pnr_from_frozen_previous_checkpoint",
    "checkpoint": str((BASELINE / checkpoint_rel).relative_to(ROOT)).replace("\\", "/"),
    "checkpoint_sha256": sha(BASELINE / checkpoint_rel),
    "bitstream_generated": fresh_fs,
    "setup_violated_endpoints": counts["Setup"],
    "hold_violated_endpoints": counts["Hold"],
    "bitstream_sha256": sha(new_fs) if new_fs.exists() else None,
    "passed": passed,
    "full_build_restored_on_failure": not passed,
}
(ROOT / f"impl/{LABEL}_build_metrics.json").write_text(
    json.dumps(metrics, indent=2) + "\n", encoding="utf-8"
)
print(json.dumps(metrics, indent=2), flush=True)
if not passed:
    raise SystemExit(result.returncode or 1)
