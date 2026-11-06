"""Preserve the accepted hardware, route checkpoint and test sources once."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT/'impl/baseline_navigation_20261025'
assert not (DEST/'manifest.json').exists(), 'Baseline already exists; do not replace it'
inputs = [p for p in (ROOT/'src').rglob('*') if p.is_file()]
inputs += list((ROOT/'sim').glob('*.sv')) + list((ROOT/'sim').glob('*.py'))
inputs += [ROOT/n for n in ('build.tcl', 'ov5640.gprj', 'impl/pnr/ov5640.fs',
    'impl/pnr/ov5640.p', 'impl/pnr/ov5640.pr', 'impl/pnr/ov5640.db',
    'impl/pnr/ov5640_download.json', 'impl/pnr/ov5640_current_audit.json')]
manifest = {}
for source in inputs:
    name = source.relative_to(ROOT).as_posix()
    target = DEST/name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    assert hashlib.sha256(target.read_bytes()).hexdigest() == digest
    manifest[name] = digest
(DEST/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
print('Saved and hashed', len(manifest), 'baseline files including incremental .pr checkpoint')
