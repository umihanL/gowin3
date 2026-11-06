"""Package rendered stereo frames using visible instance-mask pixels, never projected boxes.

Python + NumPy + Pillow only.  --input is render_plan.json; raw images remain in
the rendered tree.  Every positive mask instance, including one-pixel targets,
receives a YOLO label.  One paired photometric variant is added to train only.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import os
from pathlib import Path
import re
import shutil

import numpy as np
from PIL import Image, ImageFilter

CLASSES = ["fire", "firefighter", "person"]
SPLITS = ("train", "val", "test")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def safe_id(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", value):
        raise ValueError(f"Unsafe sample identifier: {value!r}")
    return value


def load_registry(path, expected_count=None):
    data = read_json(path)
    classes = data.get("classes", CLASSES) if isinstance(data, dict) else CLASSES
    records = data.get("instances", data.get("registry", [])) if isinstance(data, dict) else data
    if isinstance(records, dict):
        records = [dict(v, instance_id=int(k)) for k, v in records.items()]
    result = {}
    identities = set()
    for record in records:
        raw_id = record.get("instance_id", record.get("scene_id", record.get("global_id", record.get("id"))))
        number = int(raw_id)
        if number <= 0 or number in result:
            raise ValueError(f"Invalid or duplicate global instance id {raw_id}")
        class_id = int(record["class_id"])
        if class_id not in range(len(classes)):
            raise ValueError(f"Invalid class for instance {number}")
        if record.get("class_name", classes[class_id]) != classes[class_id]:
            raise ValueError(f"Class name/ID mismatch for instance {number}")
        # A wall can contain several people/flames: do not deduplicate by node.
        node = record.get("node_name", record.get("node", record.get("object_name")))
        source = record.get("source_instance_id", record.get("source_instance", record.get("source_id", record.get("annotation_id"))))
        if node is not None and source is not None:
            key = (str(node), str(record.get("slot", 0)), str(source))
            if key in identities:
                raise ValueError(f"Duplicate node/source artwork identity: {key}")
            identities.add(key)
        result[number] = dict(record, instance_id=number, class_id=class_id, class_name=classes[class_id])
    if expected_count is not None and len(result) != expected_count:
        raise ValueError(f"Registry has {len(result)} scene instances; render plan expects {expected_count}")
    if not result:
        raise ValueError("Instance registry is empty")
    return data, result


def read_instance_mask(path, encoding="integer"):
    with Image.open(path) as image:
        pixels = np.asarray(image)
    if pixels.ndim == 2:
        return pixels.astype(np.int32)
    if pixels.ndim != 3 or pixels.shape[2] < 3:
        raise ValueError(f"Unsupported instance mask shape {pixels.shape}: {path}")
    rgb = pixels[:, :, :3].astype(np.int32)
    if "R+256*G" in encoding:
        if np.any(rgb[:, :, 2] != 0):
            raise ValueError(f"Nonzero B channel in 16-bit RG instance mask: {path}")
        return rgb[:, :, 0] + (rgb[:, :, 1] << 8)
    if encoding in ("rgb24", "rgb24_le", "rgb_little_endian", "rgb_id"):
        return rgb[:, :, 0] + (rgb[:, :, 1] << 8) + (rgb[:, :, 2] << 16)
    if np.array_equal(rgb[:, :, 0], rgb[:, :, 1]) and np.array_equal(rgb[:, :, 0], rgb[:, :, 2]):
        return rgb[:, :, 0]
    raise ValueError(f"RGB mask requires an explicit RGB encoding declaration: {path}")


def visible_annotations(mask, registry, tiny_px=12, tiny_area=25):
    height, width = mask.shape
    result = []
    for number in np.unique(mask):
        number = int(number)
        if number == 0:
            continue
        if number not in registry:
            raise ValueError(f"Mask contains unknown instance id {number}")
        ys, xs = np.nonzero(mask == number)
        x0, y0, x1, y1 = int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1
        record = registry[number]
        result.append({
            "instance_id": number, "class_id": record["class_id"],
            "class_name": record["class_name"],
            "bbox_xyxy": [x0, y0, x1, y1], "area_px": int(len(xs)),
            "bbox_area_px": (x1 - x0) * (y1 - y0),
            "tiny": min(x1 - x0, y1 - y0) < tiny_px or len(xs) < tiny_area,
            "touches_image_border": x0 == 0 or y0 == 0 or x1 == width or y1 == height,
            "source": record,
        })
    return result


def yolo_text(instances, width, height):
    lines = []
    for item in instances:
        x0, y0, x1, y1 = item["bbox_xyxy"]
        numbers = ((x0 + x1) / (2 * width), (y0 + y1) / (2 * height),
                   (x1 - x0) / width, (y1 - y0) / height)
        lines.append(str(item["class_id"]) + " " + " ".join(f"{v:.9f}" for v in numbers))
    return "\n".join(lines) + ("\n" if lines else "")


def link_or_copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if sha256(source) != sha256(destination):
            raise ValueError(f"Existing image differs from raw source; use a new output directory: {destination}")
        return "existing"
    try:
        os.link(source, destination)
        return "hardlink"
    except OSError:
        shutil.copy2(source, destination)
        return "copy"


def augmentation_parameters(seed, pair_id):
    hashed = hashlib.sha256(f"{seed}:{pair_id}:a01".encode()).digest()
    rng = np.random.default_rng(int.from_bytes(hashed[:8], "little"))
    return {
        "exposure_ev": float(rng.uniform(-0.35, 0.35)),
        "white_balance_rgb": [float(v) for v in rng.uniform(0.93, 1.07, 3)],
        "noise_sigma_255": float(rng.uniform(0.3, 2.2)),
        "gaussian_blur_radius": float(rng.uniform(0, 0.6)),
        "jpeg_quality": int(rng.integers(87, 98)),
        "seed": int.from_bytes(hashed[8:12], "little"),
    }


def augment_image(source, destination, parameters, eye):
    with Image.open(source) as image:
        pixels = np.asarray(image.convert("RGB"), dtype=np.float32)
    gains = np.asarray(parameters["white_balance_rgb"], np.float32) * (2 ** parameters["exposure_ev"])
    # Exposure, WB, noise strength and blur are shared. Independent sensor-noise
    # draws avoid incorrectly treating two sensors as having identical noise.
    rng = np.random.default_rng(parameters["seed"] + (0 if eye == "L" else 1))
    pixels = pixels * gains + rng.normal(0, parameters["noise_sigma_255"], pixels.shape)
    image = Image.fromarray(np.clip(pixels, 0, 255).astype(np.uint8))
    image = image.filter(ImageFilter.GaussianBlur(parameters["gaussian_blur_radius"]))
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination, quality=parameters["jpeg_quality"], subsampling=0)


def find_camera_metadata(rendered, split, pair_id):
    candidates = [rendered / split / f"{pair_id}_meta.json",
                  rendered / split / f"{pair_id}.json",
                  rendered / split / f"{pair_id}_metadata.json"]
    for path in candidates:
        if path.exists():
            metadata = read_json(path)
            if isinstance(metadata.get("calibration"), str):
                calibration = read_json((path.parent / metadata["calibration"]).resolve())
                metadata["calibration_data"] = calibration
                for camera in metadata.get("eyes", {}).values():
                    camera["K"] = calibration["K"]
                    camera["D"] = calibration.get("D", [])
            return path, metadata
    raise FileNotFoundError(f"Rendered camera metadata missing for {pair_id}: expected *_meta.json")


def camera_for_eye(metadata, eye):
    eyes = metadata.get("eyes", metadata.get("cameras", {}))
    camera = eyes.get(eye, eyes.get("left" if eye == "L" else "right")) if isinstance(eyes, dict) else None
    if camera is None:
        camera = metadata.get(eye, metadata.get("left" if eye == "L" else "right"))
    if not isinstance(camera, dict):
        raise ValueError(f"Missing rendered {eye} camera metadata")
    return camera


def package(input_path, output, rendered=None, registry_path=None, augment=True, limit=None,
            merge_building_numbers=False):
    input_path, output = Path(input_path).resolve(), Path(output).resolve()
    plan = read_json(input_path)
    rendered = Path(rendered).resolve() if rendered else input_path.parent / "rendered"
    registry_path = Path(registry_path).resolve() if registry_path else rendered / "instance_registry.json"
    registry_doc, registry = load_registry(registry_path)
    classes = list(registry_doc.get("classes", CLASSES))
    if merge_building_numbers:
        expected = CLASSES + ["building_01", "building_02", "building_03"]
        if classes != expected:
            raise ValueError(f"Building source classes must be {expected}, got {classes}")
        for record in registry.values():
            if record["class_id"] >= 3:
                record["source_class_id"] = record["class_id"]
                record["source_class_name"] = record["class_name"]
                record["building_number"] = record.get("building_number", record["class_name"][-2:])
                record["class_id"], record["class_name"] = 3, "building_sign"
        classes = CLASSES + ["building_sign"]
        registry_doc = dict(registry_doc, classes=classes, source_classes=expected,
                            class_mapping={str(i): min(i, 3) for i in range(6)},
                            instances=list(registry.values()),
                            number_policy="Identity metadata is ground truth, not a readable-number prediction. Detector learns one building_sign class; ROI reader must reject unreadable text.")
    # The planner counts targets whose source bounding-box center lies in a
    # panel's UV triangles. The renderer reserves IDs for every source drawing
    # on that material slot, including portions extending across a UV crop.
    # Those counts intentionally differ: pruning to the planner count can drop
    # a partially visible person whose center is outside a cropped facade.
    source_lookup = defaultdict(list)
    for record in registry.values():
        source_lookup[(record.get("node"), record.get("source_instance"))].append(record)
    encoding = registry_doc.get("mask_encoding", registry_doc.get("encoding", "integer")) if isinstance(registry_doc, dict) else "integer"
    pairs, group_splits, pose_splits, pair_ids = plan["pairs"], {}, {}, set()
    # Validate the full plan before writing so pair/augmentation leakage cannot
    # hide behind an incomplete render batch.
    for pair in pairs:
        pair_id, split = safe_id(pair["id"]), pair["split"]
        if split not in SPLITS or pair_id in pair_ids:
            raise ValueError(f"Duplicate pair or invalid split: {pair_id}/{split}")
        pair_ids.add(pair_id)
        target = pair.get("sampling_target")
        if target is not None:
            matches = source_lookup[(target["node"], target["source_instance"])]
            if len(matches) != 1 or matches[0]["class_id"] != target["class_id"]:
                raise ValueError(f"Sampling target does not uniquely map to renderer registry: {pair_id}")
        for field, table in (("group_id", group_splits), ("pose_group", pose_splits)):
            value = pair[field]
            if table.setdefault(value, split) != split:
                raise ValueError(f"Split leakage in {field}={value}")
    all_pair_count = len(pairs)
    if limit is not None:
        if limit < 1:
            raise ValueError("--limit must be positive")
        pairs = pairs[:limit]
    output.mkdir(parents=True, exist_ok=True)
    generated, all_metadata = [], []
    stats = {s: {"images": 0, "original_images": 0, "augmented_images": 0, "pairs": 0,
                 "negative_images": 0, "class_instances": {c: 0 for c in classes},
                 "tiny_instances": {c: 0 for c in classes}, "bbox_short_side_px": []} for s in SPLITS}
    split_images = defaultdict(list)
    for index, pair in enumerate(pairs):
        pair_id, split = pair["id"], pair["split"]
        camera_path, camera_meta = find_camera_metadata(rendered, split, pair_id)
        if camera_meta.get("pair", pair) != pair:
            raise ValueError(f"Rendered pose/light metadata differs from current plan: {pair_id}")
        calibration = camera_meta.get("calibration_data", {})
        if calibration.get("plan_sha256", sha256(input_path)) != sha256(input_path):
            raise ValueError(f"Rendered calibration belongs to a different render plan: {pair_id}")
        parameters = augmentation_parameters(plan["seed"], pair_id)
        for eye in ("L", "R"):
            stem = f"{pair_id}_{eye}"
            source = rendered / split / f"{stem}.png"
            source_mask = rendered / split / f"{stem}_instance.png"
            with Image.open(source) as image:
                width, height = image.size
                image.verify()
            if (width, height) != (int(plan["config"]["width"]), int(plan["config"]["height"])):
                raise ValueError(f"Unexpected frame dimensions: {source}")
            mask = read_instance_mask(source_mask, encoding)
            if mask.shape != (height, width):
                raise ValueError(f"Frame/mask shape mismatch: {source_mask}")
            annotations = visible_annotations(mask, registry)
            label_text = yolo_text(annotations, width, height)
            variants = [("original", stem, ".png")]
            if split == "train" and augment:
                variants.append(("photometric_a01", stem + "_a01", ".jpg"))
            for variant, name, extension in variants:
                image_rel = Path("images") / split / (name + extension)
                label_rel = Path("labels") / split / (name + ".txt")
                annotation_rel = Path("annotations") / split / (name + ".json")
                if variant == "original":
                    storage = link_or_copy(source, output / image_rel)
                else:
                    augment_image(source, output / image_rel, parameters, eye)
                    storage = "derived_jpeg"
                (output / label_rel).parent.mkdir(parents=True, exist_ok=True)
                (output / label_rel).write_text(label_text, encoding="utf-8")
                annotation_doc = {"sample_id": name, "width": width, "height": height,
                                  "bbox_format": "xyxy, zero-based, x1/y1 exclusive",
                                  "mask_path": str(source_mask), "mask_encoding": encoding,
                                  "tiny_definition": "bbox short side <12px OR visible area <25px; retained in labels",
                                  "instances": annotations}
                write_json(output / annotation_rel, annotation_doc)
                item = {
                    "sample_id": name, "pair_id": pair_id, "eye": eye, "split": split,
                    "group_id": pair["group_id"], "pose_group": pair["pose_group"],
                    "variant": variant, "parent_sample_id": stem,
                    "image": image_rel.as_posix(), "label": label_rel.as_posix(),
                    "annotation": annotation_rel.as_posix(), "width": width, "height": height,
                    "image_sha256": sha256(output / image_rel), "raw_image": str(source),
                    "raw_image_sha256": sha256(source), "raw_mask": str(source_mask),
                    "raw_mask_sha256": sha256(source_mask), "storage": storage,
                    "camera": camera_for_eye(camera_meta, eye), "rig_metadata": camera_meta,
                    "camera_metadata_path": str(camera_path),
                    "capture_plan": pair, "augmentation": parameters if variant != "original" else None,
                    "instance_count": len(annotations), "tiny_count": sum(x["tiny"] for x in annotations),
                }
                all_metadata.append(item)
                generated.extend([image_rel, label_rel, annotation_rel])
                split_images[split].append(image_rel.as_posix())
                summary = stats[split]
                summary["images"] += 1
                summary["original_images" if variant == "original" else "augmented_images"] += 1
                summary["negative_images"] += int(not annotations)
                for instance in annotations:
                    cls = instance["class_name"]
                    summary["class_instances"][cls] += 1
                    summary["tiny_instances"][cls] += int(instance["tiny"])
                    x0, y0, x1, y1 = instance["bbox_xyxy"]
                    summary["bbox_short_side_px"].append(min(x1 - x0, y1 - y0))
        stats[split]["pairs"] += 1
        if (index + 1) % 25 == 0:
            print(f"Packaged {index + 1}/{len(pairs)} stereo pairs", flush=True)
    for split in SPLITS:
        lengths = stats[split].pop("bbox_short_side_px")
        stats[split]["bbox_short_side_px"] = ({"min": min(lengths), "median": float(np.median(lengths)),
                                                   "max": max(lengths),
                                                   "bins": {"1_3": sum(v < 4 for v in lengths),
                                                            "4_11": sum(4 <= v < 12 for v in lengths),
                                                            "12_31": sum(12 <= v < 32 for v in lengths),
                                                            "32_plus": sum(v >= 32 for v in lengths)}} if lengths else {})
        # Explicit lists use paths relative to the list itself, supported by Ultralytics.
        name = Path(f"{split}.txt")
        (output / name).write_text("".join("./" + p + "\n" for p in split_images[split]), encoding="utf-8")
        generated.append(name)
    metadata_path = output / "metadata.jsonl"
    metadata_path.write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in all_metadata), encoding="utf-8")
    # An absolute path makes invocation from arbitrary VS Code working dirs reliable.
    yaml = "# Synthetic bootstrap; exact GS130W calibration is not yet measured.\n"
    yaml += "path: " + json.dumps(output.as_posix(), ensure_ascii=False) + "\n"
    yaml += "train: train.txt\nval: val.txt\ntest: test.txt\nnames:\n"
    yaml += "".join(f"  {i}: {name}\n" for i, name in enumerate(classes))
    (output / "data.yaml").write_text(yaml, encoding="utf-8")
    write_json(output / "statistics.json", stats)
    write_json(output / "instance_registry.json", registry_doc)
    generated.extend(map(Path, ["metadata.jsonl", "data.yaml", "statistics.json", "instance_registry.json"]))
    # Calibration is a train-only bootstrap list. Round-robin light/class bins
    # prevent selecting the first adjacent trajectory frames by accident.
    calibration_bins = defaultdict(list)
    for row in all_metadata:
        if row["split"] == "train" and row["variant"] == "original" and row["eye"] == "L":
            instances = read_json(output / row["annotation"])["instances"]
            counts = Counter(x["class_id"] for x in instances)
            label = max(counts, key=counts.get) if instances else -1
            calibration_bins[(row["capture_plan"].get("lighting_profile", "unknown"), label)].append(row)
    calibration_rows, calibration_poses = [], set()
    while any(calibration_bins.values()) and len(calibration_rows) < 100:
        for key in sorted(calibration_bins):
            while calibration_bins[key]:
                row = calibration_bins[key].pop(0)
                if row["pose_group"] not in calibration_poses:
                    calibration_poses.add(row["pose_group"])
                    calibration_rows.append(row)
                    break
            if len(calibration_rows) >= 100:
                break
    (output / "calibration.txt").write_text("".join("./" + row["image"] + "\n" for row in calibration_rows), encoding="utf-8")
    generated.append(Path("calibration.txt"))
    manifest = {"schema_version": 1, "dataset_name": output.name, "classes": classes,
                "plan_path": str(input_path), "plan_sha256": sha256(input_path),
                "source_glb_sha256": plan.get("source_glb_sha256"),
                "registry_path": str(registry_path), "registry_sha256": sha256(registry_path),
                "scene_instance_count": len(registry), "expected_pair_count": len(pairs),
                "sampling_target_center_count": plan.get("scene_target_count"),
                "registered_instance_slots": len(registry),
                "instance_count_note": "Planner counts source bbox centers inside panel UV triangles; renderer reserves all material-slot artwork IDs to retain partial targets. Unused registered IDs need not appear in frames.",
                "full_plan_pair_count": all_pair_count, "is_partial_probe": len(pairs) < all_pair_count,
                "camera_config": plan["config"], "assumptions": plan.get("assumptions", []),
                "split_strategy": plan.get("split_strategy"), "augment_train_only": augment,
                "calibration_list": "calibration.txt", "calibration_image_count": len(calibration_rows),
                "calibration_note": "Synthetic bootstrap only; original train images, at most one per pose_group. Replace/supplement with real GS130W training/calibration captures before deployment.",
                "label_policy": "All visible raster-mask instances retained, including tiny and truncated targets",
                "files": {p.as_posix(): sha256(output / p) for p in generated}}
    write_json(output / "manifest.json", manifest)
    print(json.dumps({"output": str(output), "images": len(all_metadata), "pairs": len(pairs), "statistics": stats}, ensure_ascii=False))
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="render_plan.json")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--rendered", type=Path, help="Default: <plan directory>/rendered")
    parser.add_argument("--registry", type=Path, help="Default: <rendered>/instance_registry.json")
    parser.add_argument("--no-augment", action="store_true", help="Disable the default train-only paired a01 variant")
    parser.add_argument("--limit", type=int, help="Package only first N pairs for QA; manifest marks the result as partial")
    parser.add_argument("--merge-building-numbers", action="store_true", help="Map source building_01/02/03 identities to one building_sign detection class")
    args = parser.parse_args()
    package(args.input, args.output, args.rendered, args.registry, not args.no_augment, args.limit,
            args.merge_building_numbers)
