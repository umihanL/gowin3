"""Promote repaired synthetic captures to train, preserving a frozen baseline.

The explicit user role change is recorded separately from the frozen acquisition
plan. These samples cease to be independent validation for descendant models.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import copy
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import shutil
import struct
import sys
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
CLASSES = ['fire', 'firefighter', 'person']
SCHEMA = 'frozen-baseline-plus-train-captures-v1'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def lines(path):
    return [x for x in Path(path).read_text(encoding='utf-8-sig').splitlines() if x.strip()]


def rows(path):
    return [json.loads(x) for x in lines(path)]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, obj):
    Path(path).write_text(json.dumps(obj, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def pixels(path):
    with Image.open(path) as im:
        im = im.convert('RGB')
        return hashlib.sha256(struct.pack('<II', *im.size)+im.tobytes()).hexdigest()


def inside(root, relative):
    result = (root/relative).resolve()
    if not result.is_relative_to(root.resolve()):
        raise ValueError('Path outside data root: '+str(relative))
    return result


def check_anchors(root, records):
    for record in records:
        path = inside(root, record['path'])
        if not path.is_file() or sha(path) != record['sha256']:
            raise ValueError('Changed tracked file: '+str(path))


def verify_source(source):
    audit = read(source/'independent_audit.json')
    if not audit['passed'] or audit['partial'] or audit['manifest_sha256'] != sha(source/'manifest.jsonl'):
        raise ValueError('Capture audit must pass for the current repaired manifest')
    for name in ['independent_material_audit.json', 'independent_rgb_visual_review.json']:
        review = read(source/name)
        if not review['passed'] or review['manifest_sha256'] != sha(source/'manifest.jsonl'):
            raise ValueError('Missing current material/visual review')
    captures = rows(source/'manifest.jsonl')
    if len(captures) != 96:
        raise ValueError('Expected all 96 frozen captures')
    for row in captures:
        for key in ['image', 'label', 'annotation', 'mask']:
            if sha(inside(source, row[key])) != row[key+'_sha256']:
                raise ValueError('Capture changed: '+row['id']+'/'+key)
    return captures


def build(output, baseline, source):
    output, baseline, source = output.resolve(), baseline.resolve(), source.resolve()
    staging = output.with_name(output.name+'_building')
    if output.exists() or staging.exists():
        raise FileExistsError('Refusing to overwrite dataset or staging directory')
    if output.parent != baseline.parent or source.parent != output.parent/'source_data':
        raise ValueError('Keep dataset and bundled source in the project')
    base = read(baseline/'manifest.json')
    if set(base['sources']) != {'v5','v6','v7','v8'}:
        raise ValueError('Expected the frozen V8 baseline')
    base_audit = read(baseline/'independent_validation.json')
    if not base_audit['passed'] or base_audit['manifest_sha256'] != sha(baseline/'manifest.json'):
        raise ValueError('Baseline full audit missing or stale')
    check_anchors(baseline, base['files'])
    captures = verify_source(source)
    previous = rows(baseline/'metadata.jsonl')
    seen_pixels = {r['pixel_sha256'] for r in previous}
    staging.mkdir()
    metadata = copy.deepcopy(previous)
    for row in previous:
        for key in ['image','label','annotation']:
            dest = inside(staging,row[key]); dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(inside(baseline,row[key]),dest)
    for row in captures:
        digest = pixels(source/row['image'])
        if digest in seen_pixels:
            raise ValueError('Exact pixel duplicate in new capture: '+row['id'])
        seen_pixels.add(digest)
        sid = 'v9__'+row['id']
        item = dict(sample_id=sid, source_version='v9', source_sample_id=row['id'],
            source_pair_id=row['pair_id'], pair_id='v9__'+row['pair_id'],
            parent_sample_id=sid, lineage_pair_id='v9__'+row['pair_id'],
            group_id='v9__'+row['scene_id'], pose_group='v9__'+row['pose_id'],
            split='train', variant='original', eye=row['eye'], width=640, height=480,
            source_dataset='../source_data/'+source.name, pixel_sha256=digest,
            image_sha256=row['image_sha256'], source_label_sha256=row['label_sha256'],
            source_annotation_sha256=row['annotation_sha256'],
            capture_plan=dict(lighting_profile=row['lighting'], sampling_distance_m=row['nominal_distance_m'],
                obliquity_deg=abs(row['signed_angle_deg']), occluder=row['occlusion']!='none'),
            synthetic=True, evaluation_only=False, use='train',
            original_acquisition_role='evaluation_only; superseded by explicit user request',
            camera=row['camera'], storage='independent_copy')
        for key,extension in [('image','.png'),('label','.txt'),('annotation','.json')]:
            item[key]=f'{key}s/train/{sid}{extension}'
            dest=staging/item[key];dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source/row[key],dest)
        metadata.append(item)
    for split in ['val','test','calibration']:
        shutil.copy2(baseline/(split+'.txt'),staging/(split+'.txt'))
    original_train = (baseline/'train.txt').read_text(encoding='utf-8-sig')
    (staging/'train.txt').write_text(original_train.rstrip('\n')+'\n'+''.join('./'+r['image']+'\n' for r in metadata[len(previous):]),encoding='utf-8')
    shutil.copy2(baseline/'data.yaml',staging/'data.yaml')
    (staging/'metadata.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in metadata),encoding='utf-8')
    stats={}
    for split in ['train','val','test']:
        part=[r for r in metadata if r['split']==split];counts=Counter();negative=0
        for row in part:
            objects=read(staging/row['annotation'])['instances'];negative+=not objects
            counts.update(CLASSES[x['class_id']] for x in objects)
        stats[split]=dict(images=len(part),class_instances=dict(counts),negative_images=negative,
            by_source=dict(Counter(r['source_version'] for r in part)))
    write(staging/'statistics.json',stats)
    tracked=[]
    for row in metadata:
        for key in ['image','label','annotation']:
            tracked.append(dict(path=row[key],sha256=sha(staging/row[key])))
    for name in ['data.yaml','metadata.jsonl','statistics.json','train.txt','val.txt','test.txt','calibration.txt']:
        tracked.append(dict(path=name,sha256=sha(staging/name)))
    role=dict(user_instruction='加入训练集然后继续开始训练吧',
        changed_at=datetime.now(timezone(timedelta(hours=8))).isoformat(),new_role='training_only',
        independent_validation_for_new_models=False,synthetic=True,real_capture=False,
        source='../source_data/'+source.name,source_manifest_sha256=sha(source/'manifest.jsonl'),
        source_audit_sha256=sha(source/'independent_audit.json'),images=96,
        historical_plan_preserved=True,invalid_pre_fix_outputs_excluded=True)
    write(staging/'source_role_change.json',role)
    tracked.append(dict(path='source_role_change.json',sha256=sha(staging/'source_role_change.json')))
    manifest=dict(schema=SCHEMA,classes=CLASSES,baseline='../'+baseline.name,
        baseline_manifest_sha256=sha(baseline/'manifest.json'),baseline_audit_sha256=sha(baseline/'independent_validation.json'),
        capture_source='../source_data/'+source.name,capture_manifest_sha256=sha(source/'manifest.jsonl'),
        capture_audit_sha256=sha(source/'independent_audit.json'),files=tracked,
        sources={**base['sources'],'v9':dict(dataset='../source_data/'+source.name,role='train')},
        frozen_val_test=True,new_images=96)
    write(staging/'manifest.json',manifest)
    staging.rename(output)
    print(json.dumps({'dataset':str(output),'images':len(metadata),'splits':{k:v['images'] for k,v in stats.items()}},ensure_ascii=False),flush=True)


def audit(dataset, output=None):
    dataset=dataset.resolve();manifest=read(dataset/'manifest.json')
    print('Auditing dataset anchors and source captures...',flush=True)
    if manifest['schema'] != SCHEMA or manifest['classes'] != CLASSES:
        raise ValueError('Unsupported extension schema')
    baseline=(dataset/manifest['baseline']).resolve();source=(dataset/manifest['capture_source']).resolve()
    if baseline.parent!=dataset.parent or source.parent!=dataset.parent/'source_data':
        raise ValueError('Unexpected source location')
    for key,path in [('baseline_manifest_sha256',baseline/'manifest.json'),('baseline_audit_sha256',baseline/'independent_validation.json'),
                     ('capture_manifest_sha256',source/'manifest.jsonl'),('capture_audit_sha256',source/'independent_audit.json')]:
        if sha(path)!=manifest[key]:raise ValueError('Changed source anchor: '+key)
    if not read(baseline/'independent_validation.json')['passed']:
        raise ValueError('Baseline audit failed')
    check_anchors(baseline,read(baseline/'manifest.json')['files'])
    check_anchors(dataset,manifest['files'])
    captures={r['id']:r for r in verify_source(source)}
    print('File hashes and source capture audits passed; checking images and labels...',flush=True)
    before=rows(baseline/'metadata.jsonl');current=rows(dataset/'metadata.jsonl')
    if current[:len(before)]!=before or len(current)!=len(before)+len(captures):
        raise ValueError('Changed baseline metadata or missing new captures')
    role=read(dataset/'source_role_change.json')
    if role['new_role']!='training_only' or role['independent_validation_for_new_models']:
        raise ValueError('Source role must be explicitly promoted to train')
    for split in ['val','test','calibration']:
        if (dataset/(split+'.txt')).read_bytes()!=(baseline/(split+'.txt')).read_bytes():
            raise ValueError('Frozen list changed: '+split)
    expected_train=lines(baseline/'train.txt')+['./'+r['image'] for r in current[len(before):]]
    if lines(dataset/'train.txt')!=expected_train:raise ValueError('Train inventory differs')
    split_paths={s:lines(dataset/(s+'.txt')) for s in ['train','val','test']}
    all_paths=[p for part in split_paths.values() for p in part]
    if len(set(all_paths))!=len(all_paths):raise ValueError('Duplicate/cross-split image path')
    pixel_splits={};groups={};pairs=defaultdict(set);boxes=Counter();split_counts=Counter();inventory=defaultdict(set)
    old_by_id={r['sample_id']:r for r in before}
    for row_index,row in enumerate(current,1):
        split=row['split'];split_counts[split]+=1
        if './'+row['image'] not in split_paths[split]:raise ValueError('Row not in split list')
        for key in ['image','label','annotation']:
            p=inside(dataset,row[key]);inventory[key].add(p)
            if row['source_version']!='v9':
                if sha(p)!=sha(baseline/old_by_id[row['sample_id']][key]):raise ValueError('Changed old sample')
            else:
                src=captures[row['source_sample_id']]
                if split!='train' or sha(p)!=sha(source/src[key]):raise ValueError('New sample not unchanged train copy')
        with Image.open(dataset/row['image']) as im:
            im.load()
            if im.size!=(640,480):raise ValueError('Unexpected resolution')
        ph=pixels(dataset/row['image'])
        if ph!=row['pixel_sha256']:raise ValueError('Pixel SHA mismatch')
        if ph in pixel_splits and pixel_splits[ph]!=split:raise ValueError('Pixel leakage')
        pixel_splits[ph]=split
        for kind in ['group_id','pose_group','lineage_pair_id']:
            key=(kind,row[kind]);old=groups.setdefault(key,split)
            if old!=split:raise ValueError('Group leakage')
        pairs[(split,row['pair_id'],row['variant'])].add(row['eye'])
        ann=read(dataset/row['annotation'])['instances'];labels=lines(dataset/row['label'])
        if len(ann)!=len(labels):raise ValueError('Box count mismatch')
        expected=[]
        for obj in ann:
            x0,y0,x1,y1=obj['bbox_xyxy'];cid=obj['class_id']
            if cid not in range(3) or not (0<=x0<x1<=640 and 0<=y0<y1<=480):raise ValueError('Invalid bbox')
            expected.append([cid,(x0+x1)/1280,(y0+y1)/960,(x1-x0)/640,(y1-y0)/480])
            boxes[CLASSES[cid]]+=1
        actual=sorted([list(map(float,x.split())) for x in labels]);expected=sorted(expected)
        if any(len(a)!=5 or any(abs(x-y)>1e-6 for x,y in zip(a,b)) for a,b in zip(actual,expected)):
            raise ValueError('Label geometry differs from annotations')
        if row_index%1000==0 or row_index==len(current):
            print(f'Checked {row_index}/{len(current)} images and labels',flush=True)
    if any(eyes!={'L','R'} for eyes in pairs.values()):raise ValueError('Broken stereo family')
    for key,suffixes in [('image',{'.png','.jpg','.jpeg'}),('label',{'.txt'}),('annotation',{'.json'})]:
        actual={p.resolve() for p in (dataset/(key+'s')).rglob('*') if p.is_file() and p.suffix.lower() in suffixes}
        if actual!=inventory[key]:
            raise ValueError(f'Untracked {key} inventory: {len(actual-inventory[key])} extra, {len(inventory[key]-actual)} missing')
    expected_counts={'train':4656,'val':252,'test':252}
    if dict(split_counts)!=expected_counts:raise ValueError('Unexpected split totals')
    report=dict(passed=True,manifest_sha256=sha(dataset/'manifest.json'),schema=SCHEMA,
        checked_images=len(current),boxes=sum(boxes.values()),class_instances=dict(boxes),split_images=dict(split_counts),
        old_data_byte_identical=True,old_val_test_lists_byte_identical=True,new_images=96,new_role='train',
        source_capture_audit_passed=True,no_exact_cross_split_pixels=True,no_group_leakage=True,
        training_performed=False,real_capture=False)
    write(output or dataset/'independent_validation.json',report)
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return True


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['build','audit'])
    p.add_argument('--dataset',type=Path,default=ROOT/'dataset_firefighter_v4')
    p.add_argument('--baseline',type=Path,default=ROOT/'dataset_observation_v3')
    p.add_argument('--source',type=Path,default=ROOT/'source_data/firefighter_holdout_v1')
    p.add_argument('--output',type=Path);a=p.parse_args()
    if a.mode=='build':build(a.dataset,a.baseline,a.source)
    else:audit(a.dataset,a.output)
