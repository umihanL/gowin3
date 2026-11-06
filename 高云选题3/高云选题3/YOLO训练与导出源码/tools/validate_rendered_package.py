"""Validate packaged stereo detection data, geometry metadata, visible masks and splits."""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

from package_dataset import (CLASSES, SPLITS, load_registry, read_instance_mask,
                             read_json, sha256, visible_annotations, write_json, yolo_text)


def validate(input_path, output):
    root = Path(input_path).resolve()
    errors, warnings, checks = [], [], defaultdict(int)

    def require(condition, message):
        checks["assertions"] += 1
        if not condition:
            errors.append(message)

    manifest = read_json(root / "manifest.json")
    classes = manifest["classes"]
    require(classes in (CLASSES, CLASSES + ["building_sign"]), "Unsupported detection class schema")
    rows = [json.loads(line) for line in (root / "metadata.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    registry_doc, registry = load_registry(root / "instance_registry.json", manifest["scene_instance_count"])
    encoding = registry_doc.get("mask_encoding", registry_doc.get("encoding", "integer")) if isinstance(registry_doc, dict) else "integer"
    for relative, expected in manifest["files"].items():
        path = (root / relative).resolve()
        require(path.is_relative_to(root), f"Unsafe manifest path: {relative}")
        require(path.is_file(), f"Missing file: {relative}")
        if path.is_file():
            require(sha256(path) == expected, f"SHA256 mismatch: {relative}")
    groups, poses, pairs, hashes = defaultdict(set), defaultdict(set), defaultdict(list), defaultdict(set)
    originals, names, by_split = {}, set(), defaultdict(list)
    seen_mask = {}
    for row in rows:
        name, split = row["sample_id"], row["split"]
        require(name not in names, f"Duplicate sample id: {name}")
        names.add(name)
        require(split in SPLITS, f"Invalid split: {name}")
        require(row["eye"] in ("L", "R"), f"Invalid eye: {name}")
        groups[row["group_id"]].add(split)
        poses[row["pose_group"]].add(split)
        hashes[row["image_sha256"]].add(split)
        pairs[(row["pair_id"], row["variant"])].append(row)
        by_split[split].append(row["image"])
        image_path = root / row["image"]
        with Image.open(image_path) as image:
            require(image.size == (row["width"], row["height"]), f"Image shape mismatch: {name}")
            image.verify()
        require(sha256(image_path) == row["image_sha256"], f"Metadata image SHA mismatch: {name}")
        if row["variant"] == "original":
            originals[name] = row
            require(row["augmentation"] is None, f"Original has augmentation metadata: {name}")
            require(row["image_sha256"] == row["raw_image_sha256"], f"Original differs from raw image: {name}")
        else:
            require(split == "train", f"Augmentation in held-out split: {name}")
            require(row["variant"] == "photometric_a01", f"Unknown augmentation: {name}")
        annotation = read_json(root / row["annotation"])
        mask_path = row["raw_mask"]
        if mask_path not in seen_mask:
            require(Path(mask_path).is_file(), f"Missing raw mask: {mask_path}")
            require(sha256(mask_path) == row["raw_mask_sha256"], f"Raw mask SHA mismatch: {name}")
            mask = read_instance_mask(mask_path, encoding)
            require(mask.shape == (row["height"], row["width"]), f"Mask shape mismatch: {name}")
            seen_mask[mask_path] = visible_annotations(mask, registry)
        actual = seen_mask[mask_path]
        require(annotation["instances"] == actual, f"Annotation differs from visible mask: {name}")
        label = (root / row["label"]).read_text(encoding="utf-8")
        require(label == yolo_text(actual, row["width"], row["height"]), f"YOLO labels differ from mask: {name}")
        require(row["instance_count"] == len(actual), f"Instance count mismatch: {name}")
        require(row["tiny_count"] == sum(item["tiny"] for item in actual), f"Tiny count mismatch: {name}")
        for item in actual:
            require(item["area_px"] > 0, f"Empty instance: {name}")
            checks["visible_instances"] += 1
            checks["tiny_instances_retained"] += int(item["tiny"])
        camera = row["camera"]
        transform = np.asarray(camera.get("camera_to_world_blender", []), dtype=float)
        inverse = np.asarray(camera.get("world_to_camera_blender", []), dtype=float)
        if transform.size == 16 and inverse.size == 16:
            transform, inverse = transform.reshape(4, 4), inverse.reshape(4, 4)
            require(np.isfinite(transform).all() and np.isfinite(inverse).all(), f"Nonfinite camera pose: {name}")
            require(np.allclose(transform @ inverse, np.eye(4), atol=2e-5), f"Inconsistent camera pose inverse: {name}")
            require(np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=2e-5), f"Nonorthogonal camera rotation: {name}")
        else:
            require(False, f"Missing actual renderer camera transforms: {name}")
        matrix = np.asarray(camera.get("K", camera.get("intrinsics", [])), dtype=float)
        if matrix.size == 9:
            matrix = matrix.reshape(3, 3)
            require(np.isfinite(matrix).all() and matrix[0, 0] > 0 and matrix[1, 1] > 0, f"Invalid K: {name}")
            require(np.allclose(matrix[2], [0, 0, 1], atol=1e-7), f"Invalid K last row: {name}")
            require(0 <= matrix[0, 2] <= row["width"] and 0 <= matrix[1, 2] <= row["height"], f"K principal point outside frame: {name}")
        else:
            require(False, f"Missing 3x3 rendered camera K: {name}")
        checks["images"] += 1
    for label, table in (("group_id", groups), ("pose_group", poses), ("image_sha256", hashes)):
        for key, splits in table.items():
            require(len(splits) == 1, f"Cross-split leakage {label}={key}: {sorted(splits)}")
    for key, items in pairs.items():
        require(len(items) == 2 and {x["eye"] for x in items} == {"L", "R"}, f"Missing/duplicate stereo eye: {key}")
        if len(items) != 2:
            continue
        left, right = sorted(items, key=lambda x: x["eye"])
        require(left["split"] == right["split"] and left["group_id"] == right["group_id"], f"Pair group mismatch: {key}")
        require(left["capture_plan"] == right["capture_plan"], f"Different same-pair scene/light plans: {key}")
        require(left["augmentation"] == right["augmentation"], f"Photometric pair parameters differ: {key}")
        kl, kr = np.asarray(left["camera"].get("K", [])), np.asarray(right["camera"].get("K", []))
        require(kl.shape == kr.shape and np.allclose(kl, kr, atol=1e-7), f"Rectified pair K mismatch: {key}")
        # A camera position or camera-to-world matrix must come from the renderer.
        positions = []
        for item in (left, right):
            camera = item["camera"]
            position = camera.get("position", camera.get("position_world"))
            if position is None:
                transform = np.asarray(camera.get("camera_to_world", camera.get("camera_to_world_blender", camera.get("matrix_world", []))), dtype=float)
                position = transform.reshape(4, 4)[:3, 3] if transform.size == 16 else None
            if position is not None:
                positions.append(np.asarray(position, dtype=float))
        if len(positions) == 2:
            distance = float(np.linalg.norm(positions[0] - positions[1]))
            require(abs(distance - manifest["camera_config"]["baseline_m"]) < 1e-5, f"Stereo baseline mismatch {key}: {distance}")
            tl = np.asarray(left["camera"].get("camera_to_world_blender", []), dtype=float)
            tr = np.asarray(right["camera"].get("camera_to_world_blender", []), dtype=float)
            if tl.size == 16 and tr.size == 16:
                tl, tr = tl.reshape(4, 4), tr.reshape(4, 4)
                require(np.allclose(tl[:3, :3], tr[:3, :3], atol=2e-5), f"Stereo eyes are not parallel: {key}")
                in_camera = tl[:3, :3].T @ (positions[1] - positions[0])
                require(np.allclose(in_camera, [manifest["camera_config"]["baseline_m"], 0, 0], atol=2e-5), f"Stereo baseline not along positive camera X: {key}")
        else:
            require(False, f"Missing actual eye world positions: {key}")
        checks["stereo_pairs_including_augmentation"] += 1
    for row in rows:
        if row["variant"] == "original":
            continue
        parent = originals.get(row["parent_sample_id"])
        require(parent is not None, f"Missing augmentation parent: {row['sample_id']}")
        if parent:
            require(parent["split"] == row["split"] and parent["group_id"] == row["group_id"], f"Augmentation parent leakage: {row['sample_id']}")
            require((root / parent["label"]).read_bytes() == (root / row["label"]).read_bytes(), f"Photometric labels changed: {row['sample_id']}")
    for split in SPLITS:
        listed = [line.removeprefix("./") for line in (root / f"{split}.txt").read_text(encoding="utf-8").splitlines() if line]
        require(listed == by_split[split], f"Explicit list differs from metadata: {split}")
        require(len(set(listed)) == len(listed), f"Duplicate image in split list: {split}")
        actual_images = {p.relative_to(root).as_posix() for p in (root / "images" / split).glob("*") if p.is_file()}
        require(actual_images == set(listed), f"Stale/unlisted images in {split}")
    require(len(originals) == 2 * manifest["expected_pair_count"], "Original image count does not match render plan")
    require(registry_doc.get("classes", CLASSES) == classes, "Registry class order differs from manifest")
    image_rows = {row["image"]: row for row in rows}
    calibration_list = [p.removeprefix("./") for p in (root / "calibration.txt").read_text(encoding="utf-8").splitlines() if p]
    calibration_poses = []
    for path in calibration_list:
        row = image_rows.get(path)
        require(row is not None, f"Unknown calibration image: {path}")
        if row:
            require(row["split"] == "train" and row["variant"] == "original", f"Calibration includes held-out/augmented image: {path}")
            calibration_poses.append(row["pose_group"])
    require(len(calibration_poses) == len(set(calibration_poses)), "Calibration list repeats a pose group")
    require(len(calibration_list) == manifest["calibration_image_count"], "Calibration list count differs from manifest")
    if manifest.get("is_partial_probe"):
        warnings.append("Partial QA package: does not contain the entire render plan.")
    warnings.append("All splits share source wallpaper artwork; holdout measures new capture groups, not new artwork or real camera generalization.")
    warnings.append("Geometry and labels are validated synthetic data; model training and real GS130W performance remain unverified.")
    report = {"passed": not errors, "input": str(root), "checks": dict(checks), "errors": errors,
              "warnings": warnings, "manifest_sha256": sha256(root / "manifest.json")}
    write_json(output, report)
    print(json.dumps(report, ensure_ascii=False))
    return not errors


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Packaged dataset directory")
    parser.add_argument("--output", type=Path, help="Validation JSON path; default <input>/validation.json")
    args = parser.parse_args()
    try:
        ok = validate(args.input, args.output or args.input / "validation.json")
    except Exception as error:
        report = {"passed": False, "errors": [f"{type(error).__name__}: {error}"]}
        write_json(args.output or args.input / "validation.json", report)
        print(json.dumps(report, ensure_ascii=False))
        ok = False
    sys.exit(0 if ok else 1)
