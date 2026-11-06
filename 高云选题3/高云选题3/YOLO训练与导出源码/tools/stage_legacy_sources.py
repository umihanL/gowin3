"""Copy the immutable V5 data into this standalone training project."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
LEGACY = Path('D:/Github_OpenCode_TreeBird/AckermannRescueSim/ml')
OUT = ROOT / 'source_data/v5_legacy'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    source = LEGACY / 'datasets/wallpaper_gs130w_v1'
    manifest = json.loads((source / 'manifest.json').read_text(encoding='utf-8'))
    assert manifest['classes'] == ['fire', 'firefighter', 'person']
    assert not manifest['is_partial_probe']
    for rel, expected in manifest['files'].items():
        if digest(source / rel) != expected:
            raise ValueError(f'Legacy source changed: {rel}')
    copies = [(source, OUT / 'packaged'), (LEGACY / 'rendered', OUT / 'rendered'),
              (LEGACY / 'source_annotations', OUT / 'source_annotations'),
              (LEGACY / 'source_snapshots/wallpaper_v5', OUT / 'scene_snapshot')]
    for src, dst in copies:
        if dst.exists():
            raise ValueError(f'Snapshot already exists; inspect before replacing: {dst}')
        shutil.copytree(src, dst, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        print(f'Copied {src.name} -> {dst.relative_to(ROOT)}', flush=True)
    shutil.copy2(LEGACY / 'render_plan.json', OUT / 'render_plan.json')
    for rel, expected in manifest['files'].items():
        assert digest(OUT / 'packaged' / rel) == expected
    (OUT / 'snapshot.json').write_text(json.dumps({
        'original_dataset': str(source),
        'original_manifest_sha256': digest(source / 'manifest.json'),
        'copied_manifest_sha256': digest(OUT / 'packaged/manifest.json'),
        'verified_package_files': len(manifest['files']),
        'copy_policy': 'Independent file copies, not hardlinks to the original project',
        'note': 'Archived historical JSON records original absolute paths. Active merged metadata uses portable paths.'
    }, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Legacy snapshot verified; original source was not modified.')


if __name__ == '__main__':
    main()
