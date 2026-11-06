"""Independently audit an evaluation-only synthetic firefighter acquisition.

No builder imports, model inference, training, or GPU use. All final visible
instance boxes are recomputed from the masks, including undersized targets.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np
from PIL import Image

PROJECT = Path(__file__).resolve().parents[1]
NAMES = ["fire", "firefighter", "person"]


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pixel_sha(image):
    image = image.convert("RGB")
    return hashlib.sha256(struct.pack("<II", *image.size) + image.tobytes()).hexdigest()


def jsonl(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8-sig").splitlines() if line.strip()]


def glb_node_geometry(path, node_name):
    """Read the source GLB directly, independently of Blender/builder helpers."""
    data = Path(path).read_bytes()
    magic, version, length = struct.unpack_from("<4sII", data)
    if magic != b"glTF" or version != 2 or length != len(data):
        raise ValueError("Invalid source GLB")
    chunks = {}
    offset = 12
    while offset < length:
        size, kind = struct.unpack_from("<II", data, offset)
        chunks[kind] = data[offset + 8:offset + 8 + size]
        offset += size + 8
    doc = json.loads(chunks[0x4E4F534A])
    raw = chunks[0x004E4942]

    def accessor(index):
        a = doc["accessors"][index]
        v = doc["bufferViews"][a["bufferView"]]
        dtype = {5126: "<f4", 5125: "<u4", 5123: "<u2", 5121: "u1"}[a["componentType"]]
        width = {"VEC3": 3, "VEC2": 2, "SCALAR": 1}[a["type"]]
        dt = np.dtype(dtype)
        result = np.ndarray((a["count"], width), dtype=dt, buffer=raw,
                            offset=v.get("byteOffset", 0) + a.get("byteOffset", 0),
                            strides=(v.get("byteStride", dt.itemsize * width), dt.itemsize))
        return result.copy()

    node = next(n for n in doc["nodes"] if n.get("name") == node_name)
    if any(key in node for key in ["matrix", "translation", "rotation", "scale"]):
        raise ValueError("Unexpected node transform needs explicit audit support")
    primitive = doc["meshes"][node["mesh"]]["primitives"][0]
    positions = accessor(primitive["attributes"]["POSITION"])
    uv = accessor(primitive["attributes"]["TEXCOORD_0"])
    indices = accessor(primitive["indices"]).reshape(-1, 3)
    return node, positions, uv, indices


def stats(values):
    return dict(min=float(min(values)), median=float(np.median(values)), max=float(max(values))) if values else {}


def audit(root, baseline, partial=False):
    errors, warnings = [], []
    checks = Counter()

    def require(condition, message):
        checks["assertions"] += 1
        if not condition:
            errors.append(message)

    def local_path(relative):
        path = (root / relative).resolve()
        require(path.is_relative_to(root), f"Path escapes holdout root: {relative}")
        return path

    # The existing merged manifest freezes metadata and acquisition lists.
    old_manifest = read(baseline / "manifest.json")
    old_files = {entry["path"]: entry["sha256"] for entry in old_manifest["files"]}
    baseline_integrity = {}
    for name in ["metadata.jsonl", "train.txt", "val.txt", "test.txt", "data.yaml"]:
        baseline_integrity[name] = sha(baseline / name) == old_files[name]
        require(baseline_integrity[name], f"Existing baseline has changed: {name}")
    old_rows = jsonl(baseline / "metadata.jsonl")
    old_bytes = {r["image_sha256"] for r in old_rows}
    old_pixels = {r["pixel_sha256"] for r in old_rows}
    old_ids = {r["sample_id"] for r in old_rows}
    old_poses = {r["pose_group"] for r in old_rows}
    require(len(old_rows) == 5064, "Unexpected active baseline image count")
    for name in ["train.txt", "val.txt", "test.txt", "data.yaml"]:
        require("firefighter_holdout" not in (baseline / name).read_text(encoding="utf8"), f"Holdout included in active {name}")

    manifest_path = root / ("probe_manifest.jsonl" if partial else "manifest.jsonl")
    rows = jsonl(manifest_path)
    plan_path = root / "capture_plan.json"
    plan = read(plan_path)
    freeze = read(root / "plan_freeze.json")
    witness = read(root / "independent_plan_witness.json")
    source_manifest_path = root / "source_manifest.json"
    source_manifest = read(source_manifest_path) if source_manifest_path.exists() else {}
    source_hash_checks = {}
    for relative, expected in source_manifest.get("frozen_files", {}).items():
        path = local_path(relative)
        source_hash_checks[relative] = path.is_file() and sha(path) == expected
        require(source_hash_checks[relative], f"Frozen acquisition source changed: {relative}")
    require(plan.get("selection_model_blind") is True, "Plan is not explicitly model blind")
    require(plan.get("evaluation_only") is True, "Plan is not evaluation only")
    require("No dataset merge or training on this holdout." in plan.get("assumptions", []), "Missing prohibition on training merge")
    require(sha(plan_path) == freeze["plan_sha256"] == witness["files"]["capture_plan.json"], "Acquisition plan changed after independent witness")
    require(sha(root / "source_geometry.json") == freeze["source_geometry_sha256"] == witness["files"]["source_geometry.json"], "Source geometry changed after independent witness")
    source_hash_checks["capture_plan_independent_witness"] = sha(plan_path) == witness["files"]["capture_plan.json"]
    source_hash_checks["geometry_independent_witness"] = sha(root / "source_geometry.json") == witness["files"]["source_geometry.json"]
    if not partial:
        dataset_freeze = read(root / "dataset_freeze.json")
        require(sha(manifest_path) == dataset_freeze["manifest_sha256"], "Published manifest changed after dataset freeze")
        require(dataset_freeze["plan_sha256"] == freeze["plan_sha256"], "Frozen dataset uses wrong acquisition plan")
        for relative, expected in dataset_freeze["source_and_tool_sha256"].items():
            require(sha(local_path(relative)) == expected, f"Final frozen source/tool changed: {relative}")
        material_review = read(root / "independent_material_audit.json")
        rgb_review = read(root / "independent_rgb_visual_review.json")
        for title, review in [("material", material_review), ("RGB visual", rgb_review)]:
            require(review["passed"] is True, f"Independent {title} review failed")
            require(review["manifest_sha256"] == sha(manifest_path), f"Independent {title} review belongs to a different manifest")
    plan_pairs = {p["pair_id"]: p for p in plan["pairs"]}
    scenes = {s["id"]: s for s in plan["scenes"]}

    source_glb = PROJECT / "source_data/v6_ground_floor/scene/community_fire_rescue_v6.glb"
    source_annotations_root = PROJECT / "source_data/v6_ground_floor/source_annotations"
    source_annotations = read(source_annotations_root / "annotations.json")
    fire_texture = next(t for t in source_annotations["textures"] if t["source_name"] == "facade_fire.png")
    fighter = next(item for item in fire_texture["instances"] if item["class_id"] == 1)
    node, vertices, uv, indices = glb_node_geometry(source_glb, "B_0_print_1_01")
    plane_width = float(np.linalg.norm(vertices[1] - vertices[0]))
    plane_height = float(np.linalg.norm(vertices[3] - vertices[0]))
    uv_width = float(np.linalg.norm(uv[1] - uv[0]))
    uv_height = float(np.linalg.norm(uv[3] - uv[0]))
    x0, y0, x1, y1 = fighter["bbox_xyxy"]
    source_physical = dict(node_name=node["name"], source_glb_sha256=sha(source_glb),
                           plane_width_m=plane_width, plane_height_m=plane_height,
                           firefighter_width_m=(x1-x0) / fire_texture["width"] * plane_width / uv_width,
                           firefighter_height_m=(y1-y0) / fire_texture["height"] * plane_height / uv_height,
                           source_firefighter_bbox=fighter["bbox_xyxy"],
                           vertices_gltf=vertices.tolist(), uv=uv.tolist())
    source_hash_checks["original_glb"] = sha(root / "source/community_fire_rescue_v6.glb") == sha(source_glb) == freeze["source_glb_sha256"]
    require(source_hash_checks["original_glb"], "Source GLB is not frozen original")
    for original in source_annotations_root.rglob("*"):
        if original.is_file():
            copied = root / "source/source_annotations" / original.relative_to(source_annotations_root)
            source_key = "original_annotations/" + original.relative_to(source_annotations_root).as_posix()
            source_hash_checks[source_key] = copied.is_file() and sha(copied) == sha(original)
            require(source_hash_checks[source_key], f"Source artwork/mask differs from original: {original.name}")
    geometry = read(root / "source_geometry.json")
    vertices_map = np.column_stack([vertices[:, 0], -vertices[:, 2], vertices[:, 1]])
    require(np.allclose(geometry["vertices_map"], vertices_map, atol=1e-7), "Original GLB vertices changed")
    require(np.allclose(geometry["uv_gltf"], uv, atol=1e-8), "Original facade UV changed")
    require(np.array_equal(geometry["triangles"], indices), "Original facade triangles changed")
    for key in ["firefighter_width_m", "firefighter_height_m"]:
        require(abs(geometry[key] - source_physical[key]) < 1e-7, f"Physical source dimension mismatch: {key}")
    require(geometry["scale_factor"] == 1, "Source has been enlarged")
    for scene in scenes.values():
        transform = np.asarray(scene["transform_from_map"])
        require(transform.shape == (4, 4) and np.allclose(transform[3], [0, 0, 0, 1]), f"Invalid scene transform: {scene['id']}")
        require(np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-8) and abs(np.linalg.det(transform[:3, :3])-1) < 1e-8,
                f"Scene transform introduces scale/distortion: {scene['id']}")

    def original_point(source_object, texture):
        a, b, c, d = source_object["bbox_xyxy"]
        point_uv = np.array([(a+c)/2/texture["width"], (b+d)/2/texture["height"], 1])
        for triangle in indices:
            weights = np.linalg.solve(np.vstack([uv[triangle].T, np.ones(3)]), point_uv)
            if np.min(weights) >= -1e-6:
                return weights @ vertices_map[triangle]
        return None

    ids, seen_bytes, seen_pixels = set(), {}, {}
    pairs, conditions, occlusion_checks = defaultdict(list), [], []
    counts = Counter()
    all_images, all_labels, all_masks, all_annotations = set(), set(), set(), set()
    for row in rows:
        sample = row["id"]
        require(sample not in ids, f"Duplicate sample id: {sample}")
        ids.add(sample)
        require(sample not in old_ids and row["pose_id"] not in old_poses, f"Existing capture ID reused: {sample}")
        require(row["eye"] in ["L", "R"], f"Invalid stereo eye: {sample}")
        pair = plan_pairs[row["pair_id"]]
        require(all(row[k] == pair[k] for k in ["pose_id", "scene_id", "lighting", "occlusion", "designated_target_id"]), f"Capture row differs from frozen plan: {sample}")
        texture = next(t for t in source_annotations["textures"] if Path(t["source_name"]).stem == pair["wallpaper"])
        source_objects = {item["label_value"]: item for item in texture["instances"]}
        scene_transform = np.asarray(scenes[row["scene_id"]]["transform_from_map"])
        actual_metadata = read(root / "metadata" / f"{sample}.json")
        require(actual_metadata["pair"] == pair, f"Rendered metadata differs from frozen capture plan: {sample}")
        require(actual_metadata["plan_sha256"] == freeze["plan_sha256"], f"Wrong plan used by renderer: {sample}")
        require(actual_metadata["camera"] == row["camera"], f"Published camera differs from rendered metadata: {sample}")
        paths = {k: local_path(row[k]) for k in ["image", "label", "mask", "annotation"]}
        for key, values in [("image", all_images), ("label", all_labels), ("mask", all_masks), ("annotation", all_annotations)]:
            values.add(paths[key])
            require(paths[key].is_file(), f"Missing {key}: {sample}")
            if row.get(key+"_sha256"):
                require(sha(paths[key]) == row[key+"_sha256"], f"Published hash mismatch: {sample}/{key}")
        with Image.open(paths["image"]) as image:
            image.load()
            require(image.size == (640, 480), f"Image shape mismatch: {sample}")
            ihash, phash = sha(paths["image"]), pixel_sha(image)
        require(ihash not in old_bytes, f"Byte duplicate of baseline: {sample}")
        require(phash not in old_pixels, f"Pixel duplicate of baseline: {sample}")
        require(ihash not in seen_bytes, f"Duplicate holdout bytes: {sample}/{seen_bytes.get(ihash)}")
        require(phash not in seen_pixels, f"Duplicate holdout pixels: {sample}/{seen_pixels.get(phash)}")
        seen_bytes[ihash], seen_pixels[phash] = sample, sample
        if row.get("image_sha256"):
            require(ihash == row["image_sha256"], f"Image hash mismatch: {sample}")
        raw = np.asarray(Image.open(paths["mask"]))
        if raw.ndim == 3:
            require(raw.shape == (480, 640, 3) and not np.any(raw[:, :, 2]), f"Invalid mask encoding: {sample}")
            mask = raw[:, :, 0].astype(np.uint16) + raw[:, :, 1].astype(np.uint16)*256
        else:
            require(raw.shape == (480, 640), f"Mask shape mismatch: {sample}")
            mask = raw.astype(np.uint16)
        ann = read(paths["annotation"])
        objects = ann["instances"]
        present = set(map(int, np.unique(mask))) - {0}
        require({item["instance_id"] for item in objects} == present, f"Missing/excess mask instances: {sample}")
        require(len(objects) == len(present), f"Duplicate instance labels: {sample}")
        lines = [line.split() for line in paths["label"].read_text(encoding="utf8").splitlines() if line.strip()]
        require(len(lines) == len(objects), f"YOLO count mismatch: {sample}")
        require(ann["width"] == 640 and ann["height"] == 480, f"Annotation dimensions: {sample}")
        target_id = row["designated_target_id"]
        require(target_id == ann["designated_target_id"], f"Designated ID mismatch: {sample}")
        if pair["negative"]:
            require(target_id is None and all(item["class_id"] != 1 for item in objects), f"Negative frame contains firefighter label: {sample}")
        else:
            require(target_id in present, f"Designated firefighter not visible: {sample}")
        origin = np.asarray(row["camera"]["position_world"], dtype=float)
        require(origin.shape == (3,) and np.isfinite(origin).all(), f"Camera origin invalid: {sample}")
        cam_world = np.asarray(row["camera"]["camera_to_world_blender"])
        cam_inverse = np.asarray(row["camera"]["world_to_camera_blender"])
        require(np.allclose(cam_world @ cam_inverse, np.eye(4), atol=1e-5), f"Invalid camera transform inverse: {sample}")
        require(np.allclose(cam_world[:3, 3], origin, atol=1e-7), f"Camera position/transform mismatch: {sample}")
        require(np.allclose(cam_world[:3, :3].T @ cam_world[:3, :3], np.eye(3), atol=1e-5), f"Camera rotation not orthogonal: {sample}")
        expected_eye = np.asarray(pair["position"]) + cam_world[:3, 0] * (.04 if row["eye"] == "R" else -.04)
        require(np.allclose(origin, expected_eye, atol=1e-5), f"Actual eye differs from frozen rig: {sample}")
        for item, line in zip(objects, lines):
            iid = item["instance_id"]
            ys, xs = np.nonzero(mask == iid)
            bbox = [int(xs.min()), int(ys.min()), int(xs.max())+1, int(ys.max())+1]
            x0, y0, x1, y1 = bbox
            require(item["bbox_xyxy"] == bbox, f"Visible bbox mismatch: {sample}/{iid}")
            require(item["visible_area_px"] == len(xs), f"Visible area mismatch: {sample}/{iid}")
            require(0 <= x0 < x1 <= 640 and 0 <= y0 < y1 <= 480, f"Out of bounds bbox: {sample}/{iid}")
            require(len(line) == 5, f"YOLO format: {sample}/{iid}")
            numeric = list(map(float, line))
            expected_yolo = [item["class_id"], (x0+x1)/1280, (y0+y1)/960, (x1-x0)/640, (y1-y0)/480]
            require(np.isfinite(numeric).all() and np.allclose(numeric, expected_yolo, atol=1e-6), f"YOLO bbox/class mismatch: {sample}/{iid}")
            require(item["class_name"] == NAMES[item["class_id"]], f"Class name mismatch: {sample}/{iid}")
            require(iid in source_objects, f"Unknown original source label: {sample}/{iid}")
            if iid in source_objects:
                require(item["class_id"] == source_objects[iid]["class_id"], f"Original source class mismatch: {sample}/{iid}")
                source_point = original_point(source_objects[iid], texture)
                if source_point is not None:
                    expected_point = (scene_transform @ np.r_[source_point, 1])[:3]
                    require(np.allclose(item["position_world"], expected_point, atol=1e-6), f"Target center not from original physical UV: {sample}/{iid}")
                    expected_normal = scene_transform[:3, :3] @ [0, -1, 0]
                    require(np.allclose(item["normal_world"], expected_normal, atol=1e-6), f"Wall normal differs from source: {sample}/{iid}")
                else:
                    require(item["position_world"] is None and item["distance_m"] is None and item["angle_deg"] is None,
                            f"Cropped source center extrapolated outside facade: {sample}/{iid}")
                    require(item["observation_qualified"] is False, f"Unknown geometry qualified: {sample}/{iid}")
                    require(iid != target_id, f"Designated target geometry unknown: {sample}/{iid}")
                    checks["geometry_unknown_visible_instances_retained"] += 1
                    checks["boxes_recomputed_from_visible_mask"] += 1
                    counts[NAMES[item["class_id"]]] += 1
                    continue
            delta = origin - np.asarray(item["position_world"], dtype=float)
            normal = np.asarray(item["normal_world"], dtype=float)
            distance = float(np.linalg.norm(delta))
            cosine = abs(float(np.dot(delta, normal))) / (distance * float(np.linalg.norm(normal)))
            angle = math.degrees(math.acos(float(np.clip(cosine, 0, 1))))
            height, short_side = y1-y0, min(x1-x0, y1-y0)
            qualified = .4 <= distance <= 1 and height >= 40 and short_side >= 12 and angle <= 55
            require(abs(item["distance_m"] - distance) < 1e-5, f"Distance mismatch: {sample}/{iid}")
            require(abs(item["angle_deg"] - angle) < 1e-4, f"Angle mismatch: {sample}/{iid}")
            require(item["height_px"] == height and item["short_side_px"] == short_side, f"Pixels mismatch: {sample}/{iid}")
            require(bool(item["observation_qualified"]) == qualified, f"Observation condition mismatch: {sample}/{iid}")
            counts[NAMES[item["class_id"]]] += 1
            checks["boxes_recomputed_from_visible_mask"] += 1
            if iid == target_id:
                require(item["class_id"] == 1, f"Designated target is not firefighter: {sample}")
                conditions.append(dict(sample_id=sample, pose_id=row["pose_id"], pair_id=row["pair_id"],
                                       scene_id=row["scene_id"], eye=row["eye"], lighting=row["lighting"],
                                       occlusion=row["occlusion"], distance_m=distance, angle_deg=angle,
                                       height_px=height, short_side_px=short_side, visible_area_px=len(xs),
                                       observation_qualified=qualified))
                if row["occlusion"] == "lower_panel":
                    reference_path = root / "reference_masks" / f"{sample}.png"
                    require(reference_path.is_file(), f"Missing unoccluded reference mask: {sample}")
                    if reference_path.exists():
                        reference = np.asarray(Image.open(reference_path))
                        require(reference.shape == (480, 640, 3) and not np.any(reference[:, :, 1:]), f"Invalid reference mask: {sample}")
                        reference_area = int(np.count_nonzero(reference[:, :, 0] == iid))
                        loss = 1 - len(xs) / reference_area if reference_area else 0
                        require(0 < len(xs) < reference_area, f"Occluder does not hide designated target: {sample}")
                        occlusion_checks.append(dict(sample_id=sample, visible_area_px=len(xs), unoccluded_area_px=reference_area, fraction_hidden=loss))
        pairs[row["pair_id"]].append(row)
        checks["images"] += 1
    for pair_id, items in pairs.items():
        require(len(items) == 2 and {r["eye"] for r in items} == {"L", "R"}, f"Invalid stereo pair: {pair_id}")
        if len(items) == 2:
            left, right = sorted(items, key=lambda r: r["eye"])
            lp, rp = [np.asarray(r["camera"]["position_world"]) for r in [left, right]]
            require(abs(float(np.linalg.norm(lp-rp))-.08) < 1e-5, f"Stereo baseline mismatch: {pair_id}")
            require(all(left[k] == right[k] for k in ["pose_id", "scene_id", "lighting", "occlusion"]), f"Pair source group mismatch: {pair_id}")
    for name, paths, suffix in [("images", all_images, "*.png"), ("masks", all_masks, "*.png"), ("labels", all_labels, "*.txt"), ("annotations", all_annotations, "*.json")]:
        if not partial:
            require(set((root / name).glob(suffix)) == paths, f"Missing/stale inventory: {name}")
    expected_count = plan["expected_image_count"]
    if not partial:
        require(len(rows) == expected_count, f"Plan RGB quota mismatch: expected {expected_count}, actual {len(rows)}")
        require(set(ids) == {p["pair_id"]+"_"+eye for p in plan["pairs"] for eye in ["L", "R"]}, "Frozen plan image inventory mismatch")
        require(len(conditions) == plan["positive_pair_count"]*2, "Not every positive image retains the designated firefighter")
        expected_negatives = {p["pair_id"]+"_"+eye for p in plan["pairs"] if p["negative"] for eye in ["L", "R"]}
        require(set(rgb_review["negative_images_reviewed"]) == expected_negatives, "Not all negative RGB controls reviewed")

    warnings.extend([
        "这是独立台架场景、未参与训练的新机位合成验证；全部样本使用相同原壁纸图案，不能证明新图案泛化。",
        "不是实拍，不能替代真实 GS130W 曝光、失焦、畸变、反光与打印误差验证。",
        "双目左右眼、同姿态光照/遮挡变体相关，统计独立采集单位应以姿态分组，不能把所有图当作独立试验。",
        "byte/pixel 哈希排除精确重复，不证明语义或壁纸图案独立。",
    ])
    report = dict(passed=not errors, partial=partial, audited_at_utc=datetime.now(timezone.utc).isoformat(),
                  audit_tool_sha256=sha(Path(__file__)), root=str(root), baseline=str(baseline),
                  manifest_sha256=sha(manifest_path), capture_plan_sha256=sha(plan_path),
                  checks=dict(checks), errors=errors, warnings=warnings,
                  counts=dict(images=len(rows), positive_images=len(conditions), negative_images=len(rows)-len(conditions), stereo_pairs=len(pairs), unique_poses=len({r["pose_id"] for r in rows}),
                              scenes=len({r["scene_id"] for r in rows}), boxes=sum(counts.values()), by_class=dict(counts)),
                  original_physical_scale=source_physical, source_hash_checks=source_hash_checks,
                  independent_plan_witness=witness,
                  occlusion_checks=occlusion_checks,
                  source_class_exclusivity="One integer visible instance ID per pixel; each source ID has exactly one fixed class. Boxes may overlap geometrically without duplicate class labels.",
                  baseline_metadata_and_lists_unchanged=baseline_integrity,
                  deduplication=dict(baseline_images_compared=len(old_rows),
                                     method="baseline manifest-verified metadata byte SHA256 and RGB dimensions+pixels SHA256; no old image decoding required",
                                     baseline_metadata_sha256=sha(baseline / "metadata.jsonl")),
                  designated_firefighter=dict(qualified=sum(c["observation_qualified"] for c in conditions),
                                              total=len(conditions), distance_m=stats([c["distance_m"] for c in conditions]),
                                              angle_deg=stats([c["angle_deg"] for c in conditions]),
                                              height_px=stats([c["height_px"] for c in conditions]),
                                              short_side_px=stats([c["short_side_px"] for c in conditions]),
                                              lighting_counts=dict(Counter(c["lighting"] for c in conditions)),
                                              occlusion_counts=dict(Counter(c["occlusion"] for c in conditions))),
                  failed_conditions_retained=[c for c in conditions if not c["observation_qualified"]],
                  designated_target_conditions=conditions)
    if not partial:
        report["rgb_negative_controls_review"] = dict(passed=rgb_review["passed"], reviewed_images=rgb_review["negative_images_reviewed"],
                                                     method=rgb_review["method"], review_sha256=sha(root / "independent_rgb_visual_review.json"))
        report["material_binding_review"] = material_review
        report["prior_audit_revocation"] = "independent_audit_revoked_rgb_mismatch.json preserves the original structural PASS revoked after RGB negative-control visual mismatch. Twelve negative images were repaired under the unchanged frozen acquisition plan."
    output = root / ("independent_probe_audit.json" if partial else "independent_audit.json")
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf8")
    text = ("# 独立审计\n\n" + ("通过" if not errors else "未通过") + "。\n\n"
            f"核对 {len(rows)} 张图、{len(pairs)} 对双目、{sum(counts.values())} 个标注框。全部可见框由最终 ID 掩膜重新计算，与 YOLO/JSON 逐一核对。\n\n"
            f"指定消防员符合 0.4～1.0 米、高≥40 px、短边≥12 px、墙面夹角≤55°的图：{sum(c['observation_qualified'] for c in conditions)}/{len(conditions)}。低于门槛的样本仍保留。\n\n"
            "当前融合训练数据清单与元数据保持原清单哈希；对全部 5,064 张旧图的已冻结文件/像素哈希排重。\n\n"
            "全部12张负例RGB经接触表目视核验；保存场景的Base Color实际连接图像字节哈希与计划原壁纸一致。首次结构检查后发现的负例材质错误已修复，撤回记录保留。\n\n"
            + "\n\n".join(warnings) + ("\n\n错误：\n"+"\n".join(errors) if errors else "") + "\n")
    if not partial:
        (root / "独立审计.md").write_text(text, encoding="utf8")
    print(json.dumps({k: report[k] for k in ["passed", "counts", "errors", "designated_firefighter"]}, ensure_ascii=False))
    return not errors


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=PROJECT / "source_data/firefighter_holdout_v1")
    parser.add_argument("--baseline", type=Path, default=PROJECT / "dataset_observation_v3")
    parser.add_argument("--probe", action="store_true", help="Audit published probe subset; never declares complete holdout acceptance")
    args = parser.parse_args()
    raise SystemExit(0 if audit(args.root.resolve(), args.baseline.resolve(), args.probe) else 1)
