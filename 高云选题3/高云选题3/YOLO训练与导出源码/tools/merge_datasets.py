"""Merge V5/V6 and optional V7 train-only data without changing held-out sets.

Run after staging V5 and packaging all V6 renders. This creates independent
copies and a portable active dataset. Historical source manifests stay intact.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import copy
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import struct
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
NAMES = ['fire', 'firefighter', 'person']
SPLITS = ('train', 'val', 'test')
SOURCES = {'v5': ROOT / 'source_data/v5_legacy', 'v6': ROOT / 'source_data/v6_ground_floor'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def write(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def pixel_hash(path):
    with Image.open(path) as image:
        image = image.convert('RGB')
        return hashlib.sha256(struct.pack('<II', *image.size) + image.tobytes()).hexdigest()


def normalized_labels(path):
    return tuple(sorted(line.strip() for line in path.read_text(encoding='utf-8').splitlines() if line.strip()))


def load_source(version, base):
    package = base / 'packaged'
    manifest = read(package / 'manifest.json')
    if manifest['classes'] != NAMES or manifest.get('is_partial_probe'):
        raise ValueError(f'Wrong classes or partial package: {version}')
    for rel, expected in manifest['files'].items():
        if sha(package / rel) != expected:
            raise ValueError(f'Source integrity failure: {version}/{rel}')
    rows = [json.loads(line) for line in (package / 'metadata.jsonl').read_text(encoding='utf-8').splitlines() if line]
    for row in rows:
        row['_source_version'] = version
        row['_pixel_sha256'] = pixel_hash(package / row['image'])
        row['_labels'] = normalized_labels(package / row['label'])
    return manifest, rows


def merge(output, include_people=False, baseline=None, include_observation=False):
    output = output.resolve()
    staging = output.with_name(output.name + '_building')
    if output.exists() or staging.exists():
        raise ValueError(f'Use an empty new output path; refusing to overwrite {output} or {staging}')
    sources = dict(SOURCES)
    baseline_rows = None
    if include_observation:
        include_people = True
    if include_people:
        sources['v7'] = ROOT / 'source_data/v7_people_focus'
        if include_observation:
            sources['v8'] = ROOT / 'source_data/v8_observation_focus'
        baseline = (baseline or ROOT / ('dataset_people_v2' if include_observation else 'dataset')).resolve()
        if baseline.parent != output.parent:
            raise ValueError('Keep the expanded dataset beside its frozen baseline in this bundle')
        baseline_manifest = read(baseline / 'manifest.json')
        expected_sources = {'v5', 'v6', 'v7'} if include_observation else {'v5', 'v6'}
        if set(baseline_manifest.get('sources', {})) != expected_sources:
            raise ValueError('Expanded dataset must freeze exactly the preceding version sources')
        for record in baseline_manifest['files']:
            if sha(baseline / record['path']) != record['sha256']:
                raise ValueError(f'Baseline integrity failure: {record["path"]}')
        baseline_rows = [json.loads(line) for line in (baseline / 'metadata.jsonl').read_text(encoding='utf-8').splitlines() if line]
        baseline_groups = {r['group_id']: r['split'] for r in baseline_rows}
        baseline_poses = {r['pose_group'] for r in baseline_rows}
        baseline_pairs = {r['source_pair_id'] for r in baseline_rows}
    # Validation completes before final output is created.
    source_docs, rows = {}, []
    for version, base in sources.items():
        manifest, part = load_source(version, base)
        if version == ('v8' if include_observation else 'v7'):
            for row in part:
                if row['split'] != 'train' or baseline_groups.get(row['group_id']) != 'train':
                    raise ValueError('New captures must remain within original training acquisition groups')
                if row['pose_group'] in baseline_poses or row['pair_id'] in baseline_pairs:
                    raise ValueError('New captures must use new pose/pair identifiers')
                if row['capture_plan'].get('parent_pair_id'):
                    raise ValueError('New viewpoints cannot claim to be an old stereo capture')
        source_docs[version] = manifest
        rows.extend(part)
        print(f'Verified {version}: {len(part)} source images', flush=True)
    ancestry = {key: {} for key in ('group_id', 'pose_group', 'lineage_pair_id')}
    pixels, grouped = {}, defaultdict(dict)
    for row in rows:
        version, split = row['_source_version'], row['split']
        lineage = row['capture_plan'].get('parent_pair_id', row['pair_id'])
        for key, value in [('group_id', row['group_id']), ('pose_group', row['pose_group']), ('lineage_pair_id', lineage)]:
            if ancestry[key].setdefault(value, split) != split:
                raise ValueError(f'Cross-version split leakage: {key}={value}')
        if row['variant'] != 'original' and split != 'train':
            raise ValueError('Augmentation leaked into evaluation')
        digest = row['_pixel_sha256']
        if digest in pixels:
            previous = pixels[digest]
            if previous['split'] != split:
                raise ValueError('Identical RGB pixels occur across splits')
            if previous['_labels'] != row['_labels']:
                raise ValueError(f'Identical RGB has conflicting labels: {previous["sample_id"]}/{row["sample_id"]}')
        else:
            pixels[digest] = row
        key = version, row['pair_id'], row['variant']
        if row['eye'] in grouped[key]:
            raise ValueError(f'Duplicate eye: {key}')
        grouped[key][row['eye']] = row
    kept, dropped, family_pixels, families = [], [], {}, defaultdict(dict)
    for (version, pair_id, variant), eyes in grouped.items():
        if set(eyes) != {'L', 'R'}:
            raise ValueError(f'Incomplete stereo pair: {version}/{pair_id}/{variant}')
        families[(version, pair_id)][variant] = eyes
    for (version, pair_id), variants in families.items():
        if 'original' not in variants:
            raise ValueError(f'Augmentation without source RGB: {version}/{pair_id}')
        # Keep the original LR pair and all of its derived images together.
        # Otherwise a duplicated original might be removed while its distinct
        # augmentation remains, leaving a dangling parent_sample_id.
        key = tuple((variant, eyes['L']['_pixel_sha256'], eyes['R']['_pixel_sha256'])
                    for variant, eyes in sorted(variants.items()))
        if key in family_pixels:
            retained = family_pixels[key]
            for variant, eyes in variants.items():
                dropped.append(dict(source_version=version, source_pair_id=pair_id, variant=variant,
                                    retained_pair_id=retained, split=eyes['L']['split']))
        else:
            family_pixels[key] = f'{version}__{pair_id}'
            for eyes in variants.values():
                kept.extend([eyes['L'], eyes['R']])
    staging.mkdir(parents=True)
    all_metadata, tracked = [], []
    for index, source_row in enumerate(kept):
        row = copy.deepcopy(source_row)
        version = row.pop('_source_version')
        row.pop('_labels')
        row['pixel_sha256'] = row.pop('_pixel_sha256')
        base = sources[version]
        package = base / 'packaged'
        prefix = f'{version}__'
        source_name, source_pair = row['sample_id'], row['pair_id']
        original_image, original_label, original_ann = [package / row[k] for k in ('image', 'label', 'annotation')]
        row.update(source_version=version, source_sample_id=source_name, source_pair_id=source_pair,
                   source_dataset=f'../source_data/{base.name}/packaged',
                   source_label_sha256=sha(original_label), source_annotation_sha256=sha(original_ann),
                   source_glb_sha256=source_docs[version]['source_glb_sha256'],
                   layout_id='rescue_v5_v6', lineage_pair_id=row['capture_plan'].get('parent_pair_id', source_pair),
                   sample_id=prefix + source_name, pair_id=prefix + source_pair,
                   parent_sample_id=prefix + row['parent_sample_id'], storage='independent_copy')
        stem, split = row['sample_id'], row['split']
        row['image'] = f'images/{split}/{stem}{original_image.suffix}'
        row['label'] = f'labels/{split}/{stem}.txt'
        row['annotation'] = f'annotations/{split}/{stem}.json'
        for key in ('raw_image', 'raw_mask', 'camera_metadata_path'):
            source_ref = base / 'rendered' / split / Path(row[key]).name
            if not source_ref.is_file():
                raise FileNotFoundError(source_ref)
            row[key] = f'../source_data/{base.name}/rendered/{split}/{source_ref.name}'
        row['calibration_path'] = f'../source_data/{base.name}/rendered/calibration.json'
        row['render_plan_path'] = f'../source_data/{base.name}/render_plan.json'
        ann = read(original_ann)
        ann['sample_id'] = stem
        ann['mask_path'] = row['raw_mask']
        ann['source_version'] = version
        for key, original in [('image', original_image), ('label', original_label)]:
            target = staging / row[key]
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original, target)
            if sha(target) != sha(original):
                raise ValueError(f'Copy changed bytes: {target}')
        write(staging / row['annotation'], ann)
        tracked.extend(row[k] for k in ('image', 'label', 'annotation'))
        all_metadata.append(row)
        if (index + 1) % 400 == 0:
            print(f'Merged {index + 1}/{len(kept)} images', flush=True)
    duplicates = defaultdict(list)
    for row in all_metadata:
        duplicates[row['pixel_sha256']].append(row['sample_id'])
    dedup = {'policy': 'Reject cross-split pixels or contradictory labels. Deduplicate only complete identical capture families (original LR plus every augmentation); retain documented partial matches to keep stereo/augmentation parents complete.',
             'dropped_pair_variants': dropped,
             'retained_pixel_duplicates': [dict(pixel_sha256=k, sample_ids=v, reason='retain complete stereo/augmentation families') for k, v in duplicates.items() if len(v) > 1]}
    write(staging / 'deduplication.json', dedup)
    stats, totals = {}, Counter()
    for split in SPLITS:
        subset = [r for r in all_metadata if r['split'] == split]
        counts, tiny, short_sides, by_source = Counter(), Counter(), [], defaultdict(Counter)
        for row in subset:
            ann = read(staging / row['annotation'])['instances']
            totals['boxes'] += len(ann)
            totals['images'] += 1
            for obj in ann:
                counts[obj['class_name']] += 1
                tiny[obj['class_name']] += int(obj['tiny'])
                x0, y0, x1, y1 = obj['bbox_xyxy']
                short_sides.append(min(x1 - x0, y1 - y0))
            by_source[row['source_version']]['images'] += 1
            by_source[row['source_version']]['original_images' if row['variant'] == 'original' else 'augmented_images'] += 1
        stats[split] = dict(images=len(subset), original_images=sum(r['variant'] == 'original' for r in subset),
                            augmented_images=sum(r['variant'] != 'original' for r in subset),
                            pairs=len({r['pair_id'] for r in subset}), negative_images=sum(r['instance_count'] == 0 for r in subset),
                            class_instances={c: counts[c] for c in NAMES}, tiny_instances={c: tiny[c] for c in NAMES},
                            by_source=dict(by_source), bbox_short_side_px=dict(min=min(short_sides), median=statistics.median(short_sides), max=max(short_sides),
                            bins=dict(zip(('1_3', '4_11', '12_31', '32_plus'), [sum(f(v) for v in short_sides) for f in (lambda v:v<4, lambda v:4<=v<12, lambda v:12<=v<32, lambda v:v>=32)]))))
        (staging / f'{split}.txt').write_text(''.join('./' + r['image'] + '\n' for r in subset), encoding='utf-8')
    write(staging / 'statistics.json', stats)
    (staging / 'metadata.jsonl').write_text(''.join(json.dumps(r, ensure_ascii=False) + '\n' for r in all_metadata), encoding='utf-8')
    (staging / 'data.yaml').write_text('# Resolve path relative to this YAML using train.py.\npath: .\ntrain: train.txt\nval: val.txt\ntest: test.txt\nnames:\n  0: fire\n  1: firefighter\n  2: person\n', encoding='utf-8')
    # Representative, train-only BPU calibration candidates. Actual calibration
    # conversion must use the preprocessing of the selected RDK toolchain.
    bins = defaultdict(list)
    for row in all_metadata:
        if row['split'] == 'train' and row['variant'] == 'original' and row['eye'] == 'L':
            bins[(row['source_version'], row['capture_plan']['lighting_profile'])].append(row)
    calibration, selected_poses = [], set()
    while any(bins.values()) and len(calibration) < 100:
        for key in sorted(bins):
            while bins[key]:
                row = bins[key].pop(0)
                if row['pose_group'] not in selected_poses:
                    calibration.append(row['image'])
                    selected_poses.add(row['pose_group'])
                    break
            if len(calibration) >= 100:
                break
    (staging / 'calibration.txt').write_text(''.join('./' + p + '\n' for p in calibration), encoding='utf-8')
    tracked += ['train.txt', 'val.txt', 'test.txt', 'calibration.txt', 'data.yaml', 'metadata.jsonl', 'statistics.json', 'deduplication.json']
    if include_people:
        frozen_evaluation = {}
        for split in ('val', 'test'):
            if (baseline / f'{split}.txt').read_bytes() != (staging / f'{split}.txt').read_bytes():
                raise ValueError(f'Expanded data changed the frozen {split} image list')
            old_rows = [r for r in baseline_rows if r['split'] == split]
            new_rows = [r for r in all_metadata if r['split'] == split]
            if [r['sample_id'] for r in old_rows] != [r['sample_id'] for r in new_rows]:
                raise ValueError(f'Expanded data changed the frozen {split} sample order')
            for old, new in zip(old_rows, new_rows):
                for key in ('image', 'label', 'annotation'):
                    if old[key] != new[key] or sha(baseline / old[key]) != sha(staging / new[key]):
                        raise ValueError(f'Expanded data changed frozen {split} {key}: {old["sample_id"]}')
            frozen_evaluation[split] = dict(images=len(new_rows), image_list_sha256=sha(staging / f'{split}.txt'),
                                            images_labels_annotations_byte_identical=True)
    manifest = dict(schema='rescue-merged-detect-v1', classes=NAMES, sources={
        v: dict(dataset=f'../source_data/{p.name}/packaged', manifest_sha256=sha(p/'packaged/manifest.json'), glb_sha256=source_docs[v]['source_glb_sha256']) for v,p in sources.items()},
        split_policy='Original spatial group_id, pose_group and parent_pair_id remain in their original split across V5/V6. Optional V7/V8 new views are train-only in original train cells; old val/test preserved byte-for-byte. LR eyes/light variants never split; augmentation train-only.',
        deduplication_file='deduplication.json', totals=dict(totals),
        camera_assumptions=source_docs['v5']['camera_config'],
        boundary='Known printed artwork under synthetic views/light; not real-camera or unseen-art generalization. No building number class.',
        files=[dict(path=p, sha256=sha(staging/p), bytes=(staging/p).stat().st_size) for p in tracked])
    if include_people:
        manifest.update(baseline_dataset='../' + baseline.name,
                        baseline_manifest_sha256=sha(baseline / 'manifest.json'),
                        frozen_evaluation=frozen_evaluation, expansion=('V8 close-observation, illumination, obliquity, physical occlusion train supplement' if include_observation else 'V7 people/firefighter distance-angle-light train supplement'))
    write(staging/'manifest.json', manifest)
    staging.rename(output)
    print(json.dumps(dict(output=str(output), totals=dict(totals), dropped_pair_variants=len(dropped), retained_duplicate_groups=len(dedup['retained_pixel_duplicates']), statistics=stats), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'dataset')
    parser.add_argument('--include-people', action='store_true', help='Add V7 and enforce unchanged baseline val/test')
    parser.add_argument('--include-observation', action='store_true', help='Add V8 on top of V7; freeze dataset_people_v2')
    parser.add_argument('--baseline', type=Path)
    opts = parser.parse_args()
    merge(opts.output, opts.include_people, opts.baseline, opts.include_observation)
