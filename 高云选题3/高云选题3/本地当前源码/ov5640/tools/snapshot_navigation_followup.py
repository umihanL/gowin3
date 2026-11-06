"""Save an immutable pre-change snapshot for the navigation follow-up."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "impl" / "baseline_navigation_followup_20261026"
assert not DEST.exists(), f"Snapshot path already exists: {DEST}"

inputs = [p for p in (ROOT / "src").rglob("*") if p.is_file()]
inputs += list((ROOT / "sim").glob("*.sv"))
inputs += list((ROOT / "sim").glob("*.py"))
inputs += [ROOT / name for name in (
    "build.tcl", "ov5640.gprj", "impl/pnr/ov5640.fs", "impl/pnr/ov5640.p",
    "impl/pnr/ov5640.pr", "impl/pnr/ov5640.db",
    "impl/pnr/ov5640_download.json", "impl/pnr/ov5640_current_audit.json",
    "docs/navigation_entry_20261026_validation.json",
)]

manifest = {}
for source in inputs:
    if not source.exists():
        continue
    name = source.relative_to(ROOT).as_posix()
    target = DEST / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    assert hashlib.sha256(target.read_bytes()).hexdigest() == digest
    manifest[name] = digest

(DEST / "manifest.json").write_text(
    json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
)
print(f"Saved {len(manifest)} files to {DEST.relative_to(ROOT)}")
