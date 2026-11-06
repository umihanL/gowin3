"""Independent audit of portable V5/V6 data and its train-only V7 extension.

Reads images, labels, source manifests and lineage metadata without importing
torch or the merger. Writes a compact audit JSON and exits nonzero on failure.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path, PureWindowsPath
import re
import struct
import sys

import numpy as np
from PIL import Image

CLASSES = ["fire", "firefighter", "person"]
SPLITS = ("train", "val", "test")
DEFAULT_ROOT = Path(__file__).resolve().parents[1] / "dataset"
DEFAULT_ORIGINAL = Path("D:/Github_OpenCode_TreeBird/AckermannRescueSim/ml/datasets/wallpaper_gs130w_v1")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def sha(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            value.update(block)
    return value.hexdigest()


def file_records(manifest):
    records = manifest.get("files", {})
    if isinstance(records, dict):
        return [{"path": path, "sha256": digest} for path, digest in records.items()]
    return records


def relative_path(base, value, boundary):
    if not isinstance(value, str) or not value or Path(value).is_absolute() or PureWindowsPath(value).is_absolute():
        raise ValueError(f"Expected portable relative path, got {value!r}")
    if "\\" in value:
        raise ValueError(f"Use portable forward-slash paths: {value}")
    resolved = (base / value).resolve()
    if not resolved.is_relative_to(boundary):
        raise ValueError(f"Path escapes permitted bundle: {value}")
    return resolved


def read_lines(path):
    return [line.strip() for line in Path(path).read_text(encoding="utf-8-sig").splitlines() if line.strip()]


def canonical_label(data):
    return tuple(sorted(line.strip() for line in data.decode("utf-8-sig").splitlines() if line.strip()))


def check_people_extension(root, rows, manifest, source_rows, require, checked_sha, baseline_dataset=None):
    """Freeze old held-out data and require genuinely new V7 train identities."""
    if "v7" not in manifest.get("sources", {}):
        return {"applicable": False}
    extension_version = "v8" if "v8" in manifest.get("sources", {}) else "v7"
    expected_baseline_sources = {"v5", "v6", "v7"} if extension_version == "v8" else {"v5", "v6"}
    root = Path(root).resolve()
    if baseline_dataset is None:
        baseline = relative_path(root, manifest.get("baseline_dataset", "../dataset"), root.parent)
    else:
        baseline = Path(baseline_dataset).resolve()
    if not require(baseline != root and (baseline / "manifest.json").is_file(), "V7 requires a separate baseline dataset manifest"):
        return {"applicable": True, "baseline": str(baseline), "available": False}
    expected_anchor = manifest.get("baseline_manifest_sha256")
    require(bool(expected_anchor), "V7 manifest lacks the immutable baseline manifest SHA")
    require(checked_sha(baseline / "manifest.json") == expected_anchor, "Baseline dataset manifest anchor changed")
    base_manifest = read_json(baseline / "manifest.json")
    require(set(base_manifest.get("sources", {})) == expected_baseline_sources, "Extension baseline must contain exactly the previous sources")
    for version in sorted(expected_baseline_sources):
        require(manifest["sources"].get(version) == base_manifest.get("sources", {}).get(version), f"Original source provenance changed in V7 extension: {version}")
    base_rows = [json.loads(line) for line in read_lines(baseline / "metadata.jsonl")]
    base_files = {r["path"]: r for r in file_records(base_manifest)}
    require("metadata.jsonl" in base_files, "Baseline manifest omits metadata.jsonl")
    if "metadata.jsonl" in base_files:
        require(checked_sha(baseline / "metadata.jsonl") == base_files["metadata.jsonl"]["sha256"], "Baseline metadata changed")
    current = {r["sample_id"]: r for r in rows}
    require(len({r["sample_id"] for r in base_rows}) == len(base_rows), "Duplicate baseline sample IDs")
    train_groups = {r["group_id"] for r in base_rows if r["split"] == "train"}
    held_out_groups = {r["group_id"] for r in base_rows if r["split"] in ("val", "test")}
    legacy_rows = [r for version in sorted(expected_baseline_sources) for r in source_rows.get(version, {}).values()]
    old_ids = {r["sample_id"] for r in legacy_rows}
    old_pairs = {r["pair_id"] for r in legacy_rows}
    old_poses = {r["pose_group"] for r in legacy_rows}
    v7 = list(source_rows.get(extension_version, {}).values())
    require(bool(v7), "V7 source has no samples")
    for row in v7:
        sample = row["sample_id"]
        require(row["split"] == "train", f"V7 sample is not train-only: {sample}")
        require(row["group_id"] in train_groups and row["group_id"] not in held_out_groups, f"V7 spatial group is not an original train group: {sample}")
        require(row["sample_id"] not in old_ids, f"V7 reuses an old source sample ID: {sample}")
        require(row["pair_id"] not in old_pairs, f"V7 reuses an old capture pair ID: {sample}")
        require(row["pose_group"] not in old_poses, f"V7 reuses an old pose group: {sample}")
        require(row["capture_plan"].get("parent_pair_id", row["pair_id"]) == row["pair_id"], f"New V7 pose incorrectly inherits an old capture lineage: {sample}")
    preserved = Counter()
    # All old RGB/label bytes remain, including the training data. Held-out
    # ordering and complete file inventories are additionally frozen below.
    for before in base_rows:
        sample = before["sample_id"]
        after = current.get(sample)
        if not require(after is not None, f"Old sample missing from V7 extension: {sample}"):
            continue
        for field in ("source_version", "source_sample_id", "split", "group_id", "pose_group", "variant", "eye", "image", "label"):
            require(after[field] == before[field], f"Old sample identity/path changed: {sample}/{field}")
        for kind in (("image", "label", "annotation") if "annotation" in before else ("image", "label")):
            old_path = relative_path(baseline, before[kind], baseline)
            new_path = relative_path(root, after[kind], root)
            record = base_files.get(before[kind])
            require(record is not None, f"Baseline manifest omits {kind}: {sample}")
            if require(old_path.is_file() and new_path.is_file(), f"Missing old/new preserved {kind}: {sample}"):
                if record:
                    require(checked_sha(old_path) == record["sha256"], f"Baseline {kind} modified: {sample}")
                require(checked_sha(new_path) == checked_sha(old_path), f"Old {kind} bytes changed in V7 extension: {sample}")
        preserved[before["split"]] += 1
    for split in ("val", "test"):
        old_list, new_list = baseline / f"{split}.txt", root / f"{split}.txt"
        require(old_list.read_bytes() == new_list.read_bytes(), f"V7 changes frozen {split} image list bytes/order")
        require([r["sample_id"] for r in rows if r["split"] == split] == [r["sample_id"] for r in base_rows if r["split"] == split], f"V7 changes frozen {split} sample membership/order")
        for kind in (("images", "labels", "annotations") if (baseline / "annotations").exists() else ("images", "labels")):
            before_files = {p.relative_to(baseline).as_posix() for p in (baseline / kind / split).rglob("*") if p.is_file()}
            after_files = {p.relative_to(root).as_posix() for p in (root / kind / split).rglob("*") if p.is_file()}
            require(before_files == after_files, f"V7 changes frozen {split} {kind} file inventory")
    return {"applicable": True, "baseline": str(baseline), "baseline_manifest_sha256": checked_sha(baseline / "manifest.json"),
            "preserved_old_images": dict(preserved), "extension_version": extension_version, "new_source_samples": len(v7), "v7_source_samples": len(v7) if extension_version == "v7" else len(source_rows.get("v7", {})),
            "new_pose_groups": len({r["pose_group"] for r in v7}), "new_spatial_groups": sorted({r["group_id"] for r in v7}),
            "v7_pose_groups": len({r["pose_group"] for r in source_rows.get("v7", {}).values()}), "v7_spatial_groups": sorted({r["group_id"] for r in source_rows.get("v7", {}).values()}),
            "held_out_policy": "Original val/test image-list bytes, sample order, file inventory, RGB bytes and label bytes must remain unchanged"}


def audit(dataset, output, original_v1, baseline_dataset=None):
    root = dataset.resolve()
    bundle = root.parent
    errors, warnings = [], []
    checks = Counter()
    max_errors = 150

    def require(condition, message):
        checks["assertions"] += 1
        if not condition:
            checks["failed_assertions"] += 1
            if len(errors) < max_errors:
                errors.append(message)
        return bool(condition)

    cache = {}
    def checked_sha(path):
        path = Path(path)
        if path not in cache:
            cache[path] = sha(path)
        return cache[path]

    manifest = read_json(root / "manifest.json")
    require(manifest.get("schema") == "rescue-merged-detect-v1", "Unsupported merged manifest schema")
    require(manifest.get("classes") == CLASSES, "Detector must have exactly fire/firefighter/person classes")
    versions = set(manifest.get("sources", {}))
    require(versions in ({"v5", "v6"}, {"v5", "v6", "v7"}, {"v5", "v6", "v7", "v8"}), "Expected V5/V6 sources, optionally train-only V7 and V8")
    manifest_files = {}
    for record in file_records(manifest):
        path = relative_path(root, record["path"], root)
        require(record["path"] not in manifest_files, f"Duplicate manifest file: {record['path']}")
        manifest_files[record["path"]] = record
        if require(path.is_file(), f"Missing manifested file: {record['path']}"):
            require(checked_sha(path) == record["sha256"], f"Manifest hash mismatch: {record['path']}")
            if "bytes" in record:
                require(path.stat().st_size == record["bytes"], f"Manifest byte size mismatch: {record['path']}")
            checks["merged_manifest_files"] += 1
    for path in ("metadata.jsonl", "data.yaml", "statistics.json", "train.txt", "val.txt", "test.txt", "calibration.txt", manifest.get("deduplication_file", "deduplication.json")):
        require(path in manifest_files, f"Required metadata file omitted from integrity manifest: {path}")
    source_rows, source_manifests, source_dirs, source_checks = {}, {}, {}, {}
    for version, source in manifest.get("sources", {}).items():
        directory = relative_path(root, source["dataset"], bundle)
        source_dirs[version] = directory
        source_manifest_path = directory / "manifest.json"
        require(checked_sha(source_manifest_path) == source["manifest_sha256"], f"Source manifest anchor changed: {version}")
        source_manifest = read_json(source_manifest_path)
        source_manifests[version] = source_manifest
        require(source_manifest.get("classes") == CLASSES, f"Source {version} has unsupported classes")
        if source.get("glb_sha256"):
            require(source_manifest.get("source_glb_sha256") == source["glb_sha256"], f"Source GLB identity differs: {version}")
        count = 0
        for record in file_records(source_manifest):
            path = relative_path(directory, record["path"], directory)
            if require(path.is_file(), f"Missing {version} source snapshot file: {record['path']}"):
                require(checked_sha(path) == record["sha256"], f"Source snapshot hash changed: {version}/{record['path']}")
                count += 1
        rows = [json.loads(line) for line in read_lines(directory / "metadata.jsonl")]
        source_rows[version] = {row["sample_id"]: row for row in rows}
        require(len(source_rows[version]) == len(rows), f"Duplicate source IDs: {version}")
        source_checks[version] = {"snapshot_manifest_sha256": checked_sha(source_manifest_path), "snapshot_files_checked": count, "source_samples": len(rows)}
    original_status = "not_available_on_this_machine"
    if original_v1 and original_v1.is_dir() and "v5" in source_dirs:
        old = read_json(original_v1 / "manifest.json")
        require(checked_sha(original_v1 / "manifest.json") == manifest["sources"]["v5"]["manifest_sha256"], "Original V1 manifest differs from archived snapshot")
        for record in file_records(old):
            path = relative_path(original_v1, record["path"], original_v1.resolve())
            if require(path.is_file(), f"Original V1 file missing: {record['path']}"):
                require(checked_sha(path) == record["sha256"], f"Original V1 was modified: {record['path']}")
                checks["original_v1_immutable_files"] += 1
        original_status = "checked_against_original_manifest"
    else:
        warnings.append("External original V1 directory unavailable; bundled source snapshot hashes were checked. This does not prevent portable use.")
    rows = [json.loads(line) for line in read_lines(root / "metadata.jsonl")]
    extension = check_people_extension(root, rows, manifest, source_rows, require, checked_sha, baseline_dataset)
    v5_pair_lineage={r["pair_id"]:(r["split"],r["group_id"],r["pose_group"]) for r in source_rows.get("v5",{}).values()}
    row_ids, rows_by_id, represented_sources = set(), {}, set()
    groups, poses, lineages, byte_splits, pixel_splits = [defaultdict(set) for _ in range(5)]
    pixel_rows, stereo, split_images = [defaultdict(list) for _ in range(3)]
    split_counts, source_counts, class_counts, tiny_counts = Counter(), Counter(), Counter(), Counter()
    label_bytes = {}; annotations_by_id = {}; seen_raw = set()
    for row in rows:
        sample = row["sample_id"]
        version, split = row["source_version"], row["split"]
        require(sample not in row_ids, f"Duplicate sample ID: {sample}"); row_ids.add(sample); rows_by_id[sample] = row
        require(version in versions and sample.startswith(version+"__"), f"Invalid source prefix: {sample}")
        require(split in SPLITS and row["eye"] in ("L", "R"), f"Invalid split/eye: {sample}")
        require(row.get("layout_id") == "rescue_v5_v6", f"Missing common layout identity: {sample}")
        require(row["pair_id"] == version+"__"+row["source_pair_id"], f"Pair prefix differs from source: {sample}")
        require(row["source_dataset"] == manifest["sources"][version]["dataset"], f"Source dataset mismatch: {sample}")
        require(row["parent_sample_id"].startswith(version+"__"), f"Augmentation parent not namespaced: {sample}")
        source = source_rows[version].get(row["source_sample_id"])
        if not require(source is not None, f"Missing source sample: {sample}"):
            continue
        represented_sources.add((version,row["source_sample_id"]))
        for field in ("eye", "split", "group_id", "pose_group", "variant", "width", "height", "image_sha256"):
            require(row[field] == source[field], f"Source field changed: {sample}/{field}")
        require(row["source_pair_id"] == source["pair_id"], f"Source pair identity changed: {sample}")
        require(row["capture_plan"] == source["capture_plan"], f"Capture plan changed: {sample}")
        lineage = row["capture_plan"].get("parent_pair_id", row["source_pair_id"]) if version == "v6" else row["source_pair_id"]
        require(row["lineage_pair_id"] == lineage, f"Wrong cross-version lineage pair: {sample}")
        if version=="v6":
            require(lineage in v5_pair_lineage, f"V6 parent pair is absent from V5 source: {sample}")
            if lineage in v5_pair_lineage:
                require(v5_pair_lineage[lineage]==(split,row["group_id"],row["pose_group"]),f"V6 parent capture grouping differs from V5: {sample}")
        # Shared spatial IDs deliberately have no version prefix in leakage maps.
        groups[row["group_id"]].add(split); poses[row["pose_group"]].add(split); lineages[lineage].add(split)
        for value in (row["group_id"],row["pose_group"],lineage):
            require(bool(value), f"Empty grouping identity: {sample}")
        byte_splits[row["image_sha256"]].add(split); pixel_splits[row["pixel_sha256"]].add(split); pixel_rows[row["pixel_sha256"]].append(row)
        stereo[(version,row["source_pair_id"],row["variant"])].append(row)
        split_images[split].append(row["image"]);split_counts[split]+=1;source_counts[version+"/"+split]+=1
        paths = {key:relative_path(root,row[key],root) for key in ("image","label","annotation")}
        for key,path in paths.items():
            require(row[key] in manifest_files, f"Unmanifested {key}: {sample}")
            require(path.is_file(), f"Missing {key}: {sample}")
        with Image.open(paths["image"]) as image:
            image.load(); require(image.size==(row["width"],row["height"]), f"Actual dimensions mismatch: {sample}")
            pixel_hash=hashlib.sha256(struct.pack("<II",image.width,image.height)+image.convert("RGB").tobytes()).hexdigest()
        require(checked_sha(paths["image"])==row["image_sha256"], f"Image metadata SHA mismatch: {sample}")
        require(pixel_hash==row["pixel_sha256"], f"Decoded RGB SHA mismatch: {sample}")
        source_label=relative_path(source_dirs[version],source["label"],source_dirs[version])
        source_annotation=relative_path(source_dirs[version],source["annotation"],source_dirs[version])
        require(checked_sha(source_label)==row["source_label_sha256"], f"Source label anchor mismatch: {sample}")
        require(checked_sha(source_annotation)==row["source_annotation_sha256"], f"Source annotation anchor mismatch: {sample}")
        actual_label=paths["label"].read_bytes();label_bytes[sample]=actual_label
        require(actual_label==source_label.read_bytes(), f"Labels changed during merge: {sample}")
        annotation=read_json(paths["annotation"]); source_ann=read_json(source_annotation);annotations_by_id[sample]=annotation
        require(annotation["instances"]==source_ann["instances"], f"Visible/tiny instances changed: {sample}")
        require(annotation["width"]==row["width"] and annotation["height"]==row["height"], f"Annotation dimensions mismatch: {sample}")
        raw_label_lines=actual_label.decode("utf-8-sig").splitlines();label_lines=[line.strip() for line in raw_label_lines if line.strip()]
        require(len(label_lines)==len(annotation["instances"])==row["instance_count"], f"Label instance count mismatch: {sample}")
        if not label_lines:
            checks["empty_negative_labels"]+=1
        for i,line in enumerate(label_lines):
            values=line.split()
            if not require(len(values)==5, f"Not a detect label: {sample}:{i+1}"):
                continue
            require(values[0] in ("0","1","2"), f"Invalid detection class: {sample}:{i+1}")
            if values[0] not in ("0","1","2"):continue
            cls=int(values[0]); coords=[float(x) for x in values[1:]]
            require(all(math.isfinite(v) for v in coords), f"Nonfinite bbox: {sample}:{i+1}")
            x,y,w,h=coords;eps=2e-6
            require(w>0 and h>0 and w<=1 and h<=1 and x-w/2>=-eps and x+w/2<=1+eps and y-h/2>=-eps and y+h/2<=1+eps,f"Out-of-frame bbox: {sample}:{i+1}")
            if i>=len(annotation["instances"]):continue
            obj=annotation["instances"][i];bx0,by0,bx1,by1=obj["bbox_xyxy"]
            expected=[(bx0+bx1)/2/row["width"],(by0+by1)/2/row["height"],(bx1-bx0)/row["width"],(by1-by0)/row["height"]]
            require(cls==obj["class_id"] and obj["class_name"]==CLASSES[cls], f"Annotation/label class mismatch: {sample}:{i+1}")
            require(all(abs(a-b)<=2e-6 for a,b in zip(coords,expected)), f"Annotation/label box mismatch: {sample}:{i+1}")
            require(0<=bx0<bx1<=row["width"] and 0<=by0<by1<=row["height"] and obj["area_px"]>0, f"Bad pixel box/area: {sample}:{i+1}")
            class_counts[CLASSES[cls]]+=1;tiny_counts[CLASSES[cls]]+=bool(obj["tiny"])
        require(row["tiny_count"]==sum(bool(o["tiny"]) for o in annotation["instances"]), f"Tiny count changed: {sample}")
        if row["variant"]=="original":
            require(row.get("augmentation") is None, f"Original has augmentation: {sample}")
            require(row["image_sha256"]==row["raw_image_sha256"], f"Original RGB bytes differ from render: {sample}")
        else:
            require(split=="train", f"Augmentation in held-out split: {sample}")
            require(bool(row.get("augmentation")), f"Missing augmentation parameters: {sample}")
            checks["training_augmented_images"]+=1
        for key in ("raw_image","raw_mask","camera_metadata_path"):
            path=relative_path(root,row[key],bundle)
            require(path.is_file(), f"Missing portable raw provenance: {sample}/{key}")
            if path not in seen_raw and path.is_file():
                expected_hash=row.get(key+"_sha256")
                if expected_hash:require(checked_sha(path)==expected_hash, f"Raw provenance SHA mismatch: {sample}/{key}")
                seen_raw.add(path)
        if "mask_path" in annotation:
            require(relative_path(root,annotation["mask_path"],bundle)==relative_path(root,row["raw_mask"],bundle),f"Annotation mask path differs from row: {sample}")
        calibration_path=row.get("calibration_path",row.get("rig_metadata",{}).get("calibration_path"))
        require(bool(calibration_path), f"Missing portable explicit calibration_path: {sample}")
        if calibration_path:require(relative_path(root,calibration_path,bundle).is_file(),f"Missing calibration file: {sample}")
        camera=row["camera"]
        transform=np.asarray(camera.get("camera_to_world_blender",[]),dtype=float)
        inverse=np.asarray(camera.get("world_to_camera_blender",[]),dtype=float)
        if require(transform.shape==(4,4) and inverse.shape==(4,4),f"Missing camera transform: {sample}"):
            require(np.allclose(transform@inverse,np.eye(4),atol=2e-5),f"Camera inverse mismatch: {sample}")
            require(np.isfinite(transform).all() and np.isfinite(inverse).all(),f"Nonfinite camera transform: {sample}")
        intrinsics=np.asarray(camera.get("K",[]),dtype=float)
        if require(intrinsics.shape==(3,3),f"Missing camera intrinsics: {sample}"):
            require(np.isfinite(intrinsics).all() and intrinsics[0,0]>0 and intrinsics[1,1]>0 and np.allclose(intrinsics[2],[0,0,1]),f"Invalid provisional intrinsics: {sample}")
        checks["images"]+=1
    for name,table in (("spatial_group",groups),("pose_group",poses),("lineage_pair",lineages),("image_sha256",byte_splits),("pixel_sha256",pixel_splits)):
        for key,splits in table.items():require(len(splits)==1, f"Cross-split leakage {name} {key}: {sorted(splits)}")
        checks[name+"_groups"]=len(table)
    for key,members in stereo.items():
        require(len(members)==2 and {r["eye"] for r in members}=={"L","R"}, f"Incomplete retained stereo pair: {key}")
        if len(members)==2:
            a,b=members
            require(a["capture_plan"]==b["capture_plan"] and a.get("augmentation")==b.get("augmentation"),f"Stereo scene/augmentation mismatch: {key}")
            require(a["split"]==b["split"], f"Stereo split mismatch: {key}")
            positions=[np.asarray(r["camera"]["position_world"],dtype=float) for r in members]
            require(abs(float(np.linalg.norm(positions[0]-positions[1]))-.08)<1e-5, f"Stereo baseline not nominal80mm: {key}")
        checks["retained_pair_variants"]+=1
    for row in rows:
        if row["variant"]=="original":continue
        parent=rows_by_id.get(row["parent_sample_id"])
        if require(parent is not None, f"Missing augmentation parent: {row['sample_id']}"):
            require(parent["variant"]=="original" and parent["split"]==row["split"] and parent["group_id"]==row["group_id"] and parent["pose_group"]==row["pose_group"],f"Augmentation parent lineage differs: {row['sample_id']}")
            require(label_bytes[parent["sample_id"]]==label_bytes[row["sample_id"]], f"Photometric augmentation changed labels: {row['sample_id']}")
    duplicate_doc=read_json(relative_path(root,manifest.get("deduplication_file","deduplication.json"),root))
    declared={d["pixel_sha256"]:set(d["sample_ids"]) for d in duplicate_doc.get("retained_pixel_duplicates",[])}
    actual_duplicates={key:{r["sample_id"] for r in rs} for key,rs in pixel_rows.items() if len(rs)>1}
    require(declared==actual_duplicates, "Retained pixel duplicate log does not exactly match decoded images")
    for key,samples in actual_duplicates.items():
        signatures={canonical_label(label_bytes[s]) for s in samples}
        require(len(signatures)==1, f"Identical RGB pixels have conflicting labels: {key}")
    for item in duplicate_doc.get("dropped_pair_variants",[]):
        version,pair,variant=item["source_version"],item["source_pair_id"],item["variant"]
        source_members=[r for r in source_rows[version].values() if r["pair_id"]==pair and r["variant"]==variant]
        retained=[r for r in rows if r["pair_id"]==item["retained_pair_id"] and r["variant"]==variant]
        require(len(source_members)==2 and len(retained)==2, f"Invalid pair deduplication alias: {version}/{pair}/{variant}")
        for old in source_members:
            represented_sources.add((version,old["sample_id"]))
            matches=[r for r in retained if r["eye"]==old["eye"]]
            if not matches:continue
            keep=matches[0]; require(old["split"]==keep["split"]==item["split"], f"Dropped pair alias crosses split: {version}/{pair}")
            with Image.open(source_dirs[version]/old["image"]) as image:
                old_pixel=hashlib.sha256(struct.pack("<II",image.width,image.height)+image.convert("RGB").tobytes()).hexdigest()
            require(old_pixel==keep["pixel_sha256"], f"Dropped pair RGB is not duplicate: {version}/{old['sample_id']}")
            require(canonical_label((source_dirs[version]/old["label"]).read_bytes())==canonical_label(label_bytes[keep["sample_id"]]), f"Dropped pair labels differ: {version}/{old['sample_id']}")
    dropped_families=defaultdict(set)
    for item in duplicate_doc.get("dropped_pair_variants",[]):
        dropped_families[(item["source_version"],item["source_pair_id"])].add(item["variant"])
    for (version,pair),variants in dropped_families.items():
        source_variants={r["variant"] for r in source_rows[version].values() if r["pair_id"]==pair}
        require(variants==source_variants,f"Deduplication removed only part of stereo/augmentation family: {version}/{pair}")
    all_source_samples={(v,s) for v,records in source_rows.items() for s in records}
    require(represented_sources==all_source_samples,"Source samples are missing without a pair-deduplication alias")
    for split in SPLITS:
        listing=read_lines(root/f"{split}.txt")
        require(all(line.startswith("./images/") and "\\" not in line for line in listing),f"Split list not portable: {split}")
        listed=[line.removeprefix("./") for line in listing]
        require(listed==split_images[split],f"Split list order/content differs from metadata: {split}")
        require(len(set(listed))==len(listed),f"Duplicate split image path: {split}")
        actual={p.relative_to(root).as_posix() for p in (root/"images"/split).rglob("*") if p.is_file()}
        require(actual==set(listed),f"Unlisted or stale image files: {split}")
    yaml=(root/"data.yaml").read_text(encoding="utf-8-sig")
    require(all(re.search(rf"(?m)^\s*{i}\s*:\s*['\"]?{name}['\"]?\s*(?:#.*)?$",yaml) for i,name in enumerate(CLASSES)),"data.yaml does not declare the exact3named classes")
    require(not re.search(r"(?m)^\s*3\s*:",yaml),"data.yaml includes an unwanted fourth class")
    for split in SPLITS:require(bool(re.search(rf"(?m)^\s*{split}\s*:\s*['\"]?{split}\.txt['\"]?\s*(?:#.*)?$",yaml)),f"data.yaml must name portable {split}.txt")
    path_match=re.search(r"(?m)^path\s*:\s*(.*?)\s*(?:#.*)?$",yaml)
    if path_match:
        value=path_match.group(1).strip().strip('\"\'')
        require(not Path(value).is_absolute() and not PureWindowsPath(value).is_absolute(),"data.yaml embeds an absolute dataset root")
    calibration_lines=read_lines(root/"calibration.txt")
    image_lookup={r["image"]:r for r in rows};calibration_poses=[]
    require(len(calibration_lines)<=100,"Calibration list exceeds100images")
    for line in calibration_lines:
        require(line.startswith("./images/") and "\\" not in line,"Calibration path is not portable: "+line)
        row=image_lookup.get(line.removeprefix("./"))
        if require(row is not None,"Calibration image absent from dataset: "+line):
            require(row["split"]=="train" and row["variant"]=="original" and row["eye"]=="L","Calibration must be train-original-left only: "+line)
            calibration_poses.append(row["pose_group"])
    require(len(calibration_poses)==len(set(calibration_poses)),"Calibration repeats a pose across versions")
    checks["calibration_images"]=len(calibration_lines)
    stats=read_json(root/"statistics.json")
    for split in SPLITS:
        if split in stats:require(stats[split].get("images")==split_counts[split],f"Statistics image count mismatch: {split}")
    report={"schema_version":1,"passed":not errors,"dataset":str(root),"manifest_sha256":checked_sha(root/"manifest.json"),
            "checks":dict(checks),"counts":{"by_split":dict(split_counts),"by_source_split":dict(source_counts),"class_instances":dict(class_counts),"tiny_instances_retained":dict(tiny_counts)},
            "source_snapshots":source_checks,"external_original_v1_audit":original_status,"people_extension":extension,
            "retained_duplicate_pixel_groups":len(actual_duplicates),"dropped_pair_variants":len(duplicate_doc.get("dropped_pair_variants",[])),
            "errors":errors,"warnings":warnings+["Annotations are synthetic mask-derived ground truth, not model predictions or real-camera accuracy.","Original tiny targets remain labelled; map versions reuse artwork and are not independent appearance domains."],
            "limits":"Checks source snapshot integrity, merged label preservation, RGB equality, stereo lineage and portable paths; it does not train a detector or re-render scene geometry."}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"passed":report["passed"],"report":str(output),"images":checks["images"],"by_split":dict(split_counts),"errors":errors[:12],"error_count":checks["failed_assertions"]},ensure_ascii=False))
    return report["passed"]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset",type=Path,default=DEFAULT_ROOT)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--original-v1",type=Path,default=DEFAULT_ORIGINAL,help="Optional original-machine immutability comparison; bundled sources are always verified")
    parser.add_argument("--baseline-dataset",type=Path,help="For V7, immutable V5/V6 baseline; defaults to manifest baseline_dataset or sibling dataset")
    args=parser.parse_args();output=args.output or args.dataset/"independent_validation.json"
    try:
        manifest=read_json(args.dataset/'manifest.json')
        if manifest.get('schema')=='frozen-baseline-plus-train-captures-v1':
            from firefighter_train_extension import audit as audit_extension
            ok=audit_extension(args.dataset,output)
        else:
            ok=audit(args.dataset,output,args.original_v1,args.baseline_dataset)
    except Exception as error:
        output.parent.mkdir(parents=True,exist_ok=True)
        report={"passed":False,"errors":[f"{type(error).__name__}: {error}"]}
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        print(json.dumps(report,ensure_ascii=False));ok=False
    return 0 if ok else 1


if __name__=="__main__":
    sys.exit(main())
