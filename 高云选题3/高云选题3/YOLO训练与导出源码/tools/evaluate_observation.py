"""Fixed-threshold, per-instance observation diagnostics. Never trains or opens test.

Distances use the individual artwork centre recovered from the frozen GLB UV
geometry and the actual eye extrinsic. The plan's sampling target is NOT used.
Only recall is assigned to GT distance/angle/size bins: an unmatched prediction
has no known GT distance, so per-GT-bin precision would be misleading.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import html
import json
import math
import os
from pathlib import Path
import struct
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CLASSES = ["fire", "firefighter", "person"]
CN = {"fire": "火焰", "firefighter": "消防员", "person": "人物", "all": "全部"}
SOURCES = {
    "v5": ("v5_legacy", "scene_snapshot/community.glb"),
    "v6": ("v6_ground_floor", "scene/community_fire_rescue_v6.glb"),
    "v7": ("v7_people_focus", "scene/community_fire_rescue_v6.glb"),
    "benchmark": ("observation_benchmark_v1", "scene/community_fire_rescue_v6.glb"),
    "v8": ("v8_observation_focus", "scene/community_fire_rescue_v6.glb"),
}
DEFAULT_MODEL = ROOT / "runs/wallpaper_yolo11n_people_v2_20261008/paused_for_review_20261008_193528/weights/best.pt"


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf8")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def glb_geometry(glb_path, annotation_path):
    """Recover bbox-centre point and normal, keyed by exact scene artwork ID.

    This repository exports facade vertices baked into GLTF coordinates. An
    unexpected transform is deliberately unresolved rather than guessed.
    """
    payload = Path(glb_path).read_bytes()
    json_size = struct.unpack_from("<I", payload, 12)[0]
    doc = json.loads(payload[20:20 + json_size])
    binary = payload[28 + json_size:]
    annotations = read_json(annotation_path)
    by_material = {"PRINT_" + t["source_name"].removeprefix("facade_").removesuffix(".png"): t
                   for t in annotations["textures"]}

    def accessor(index):
        a = doc["accessors"][index]
        v = doc["bufferViews"][a["bufferView"]]
        dtype = {5126: "<f4", 5125: "<u4", 5123: "<u2", 5121: "u1"}[a["componentType"]]
        width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]
        item = np.dtype(dtype).itemsize
        return np.ndarray((a["count"], width), dtype, buffer=binary,
                          offset=v.get("byteOffset", 0) + a.get("byteOffset", 0),
                          strides=(v.get("byteStride", width * item), item)).copy()

    result = {}
    for node in doc["nodes"]:
        if "mesh" not in node or any(k in node for k in ("matrix", "rotation", "translation", "scale")):
            continue
        for slot, prim in enumerate(doc["meshes"][node["mesh"]]["primitives"]):
            material = doc["materials"][prim.get("material", 0)]["name"]
            if material not in by_material:
                continue
            texture = by_material[material]
            vertices = accessor(prim["attributes"]["POSITION"]).astype(float)
            vertices = vertices[:, [0, 2, 1]] * np.array([1, -1, 1])
            uv = accessor(prim["attributes"]["TEXCOORD_0"]).astype(float)
            triangles = accessor(prim["indices"]).ravel().reshape(-1, 3)
            for inst in texture["instances"]:
                x0, y0, x1, y1 = inst["bbox_xyxy"]
                point = [(x0 + x1) / (2 * texture["width"]), (y0 + y1) / (2 * texture["height"]), 1]
                for tri in triangles:
                    matrix = np.vstack((uv[tri].T, np.ones(3)))
                    if abs(np.linalg.det(matrix)) < 1e-10:
                        continue
                    weights = np.linalg.solve(matrix, point)
                    if min(weights) < -1e-5:
                        continue
                    normal = np.cross(vertices[tri[1]] - vertices[tri[0]], vertices[tri[2]] - vertices[tri[0]])
                    if np.linalg.norm(normal) < 1e-10:
                        continue
                    result[(node["name"], slot, inst["id"])] = {
                        "position": (weights @ vertices[tri]).tolist(),
                        "normal": (normal / np.linalg.norm(normal)).tolist(),
                        "class_id": inst["class_id"],
                    }
                    break
    return result


def instance_geometry(instance, row, geometry):
    source = instance.get("source", {})
    key = (source.get("node"), int(source.get("slot", 0)), source.get("source_instance"))
    item = geometry.get(key)
    camera = row.get("camera", {}).get("position_world")
    if item is None or camera is None or item["class_id"] != instance["class_id"]:
        return {"distance_m": None, "obliquity_deg": None, "geometry_status": "unknown"}
    delta = np.asarray(camera, float) - np.asarray(item["position"], float)
    distance = float(np.linalg.norm(delta))
    if distance < 1e-8 or not math.isfinite(distance):
        return {"distance_m": None, "obliquity_deg": None, "geometry_status": "unknown"}
    angle = math.degrees(math.acos(float(np.clip(abs(np.dot(delta / distance, item["normal"])), 0, 1))))
    return {"distance_m": distance, "obliquity_deg": angle, "geometry_status": "resolved",
            "target_world": item["position"], "wall_normal_world": item["normal"]}


def iou(a, b):
    w = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = w * h
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def match_predictions(gt, predictions, threshold=0.5):
    """Confidence-ordered, class-aware, one-to-one greedy IoU matching.

    Secondary cross-class matching diagnoses confusion only, never grants TP.
    Each wrong-class prediction can explain at most one remaining GT.
    """
    statuses = [{"status": "missed", "prediction_index": None} for _ in gt]
    used_gt, used_pred = set(), set()
    order = sorted(range(len(predictions)), key=lambda n: -predictions[n]["confidence"])
    for pi in order:
        pred = predictions[pi]
        candidates = [(iou(pred["bbox_xyxy"], obj["bbox_xyxy"]), gi) for gi, obj in enumerate(gt)
                      if gi not in used_gt and pred["class_id"] == obj["class_id"]]
        overlap, gi = max(candidates, default=(0, -1))
        if overlap >= threshold:
            used_gt.add(gi)
            used_pred.add(pi)
            statuses[gi] = {"status": "matched", "prediction_index": pi, "iou": overlap}
    true_prediction_indices = sorted(used_pred)
    for pi in order:
        if pi in used_pred:
            continue
        pred = predictions[pi]
        candidates = [(iou(pred["bbox_xyxy"], obj["bbox_xyxy"]), gi) for gi, obj in enumerate(gt)
                      if gi not in used_gt and pred["class_id"] != obj["class_id"]]
        overlap, gi = max(candidates, default=(0, -1))
        if overlap >= threshold:
            used_gt.add(gi)
            used_pred.add(pi)
            statuses[gi] = {"status": "confused", "prediction_index": pi, "iou": overlap,
                            "predicted_class_id": pred["class_id"]}
    return statuses, true_prediction_indices


def distance_bin(distance):
    if distance is None:
        return "unknown"
    if distance < .4:
        return "<0.4m"
    if distance < .6:
        return "0.4-0.6m"
    if distance <= 1.0:
        return "0.6-1.0m"
    if distance <= 1.6:
        return "1.0-1.6m"
    return ">1.6m"


def angle_bin(angle):
    if angle is None:
        return "unknown"
    if angle < 20:
        return "0-20deg"
    if angle < 40:
        return "20-40deg"
    return "40-90deg"


def size_bin(height, short_side):
    if height < 40:
        return "height<40px"
    if short_side < 12:
        return "height>=40,short<12px"
    if short_side < 16:
        return "height>=40,12<=short<16px"
    return "height>=40,short>=16px"


def priority_observable(obj, short_side=12):
    distance = obj.get("distance_m")
    return (distance is not None and .4 <= distance <= 1.0 and
            obj["height_px"] >= 40 and obj["short_side_px"] >= short_side)


def select_v8_playback(rows):
    """Twelve left-eye images: six metadata-chosen poses, both light variants.

    No model results are consulted. Use normal/dim and all available angle /
    occlusion combinations for firefighter-target original captures.
    """
    eligible = [r for r in rows if r["split"] == "train" and r["source_version"] == "v8"
                and r["variant"] == "original" and r["eye"] == "L"
                and r["capture_plan"]["sampling_target"]["class_id"] == 1]
    requested = [("frontal_0_20", False), ("frontal_0_20", True), ("oblique_20_40", False),
                 ("oblique_40_55", False), ("oblique_40_55", True), ("oblique_20_40", False)]
    used = set()
    selected = []
    for angle, occluded in requested:
        choices = sorted({r["pose_group"] for r in eligible if r["pose_group"] not in used
                          and r["capture_plan"]["angle_bin"] == angle
                          and bool(r["capture_plan"].get("occluder")) == occluded})
        if not choices:
            raise RuntimeError(f"Missing required playback metadata stratum: {angle}, occluded={occluded}")
        pose = choices[0]
        batch = sorted([r for r in eligible if r["pose_group"] == pose], key=lambda r: r["sample_id"])
        if len(batch) != 2 or len({r["capture_plan"]["lighting_profile"] for r in batch}) != 2:
            raise RuntimeError("Playback pose must have two original left-eye lighting variants")
        used.add(pose)
        selected.extend(batch)
    return selected


def summary(instances, image_records):
    classes = {}
    for cls in ["all", *CLASSES]:
        cid = CLASSES.index(cls) if cls != "all" else None
        objects = [o for o in instances if cid is None or o["class_id"] == cid]
        pred = sum(sum(cid is None or p["class_id"] == cid for p in r["predictions"]) for r in image_records)
        tp = sum(o["status"] == "matched" for o in objects)
        classes[cls] = {"gt": len(objects), "tp": tp, "fn": len(objects)-tp, "predictions": pred,
                        "fp": pred-tp, "precision": tp/pred if pred else None,
                        "recall": tp/len(objects) if objects else None,
                        "confused": sum(o["status"] == "confused" for o in objects)}
    bins = {}
    definitions = {
        "distance": ["<0.4m", "0.4-0.6m", "0.6-1.0m", "1.0-1.6m", ">1.6m", "unknown"],
        "angle": ["0-20deg", "20-40deg", "40-90deg", "unknown"],
        "size": ["height<40px", "height>=40,short<12px", "height>=40,12<=short<16px", "height>=40,short>=16px"],
        "lighting": ["neutral", "warm_side", "cool_dim", "sun_back", "bright_overhead", "unknown"],
        "observation": ["all_gt", "size_only_h40_s12", "size_only_h40_s16", "distance_0.4-1.0m", "distance_0.4-1.0m_h40_s12", "distance_0.4-1.0m_h40_s16"],
    }
    definitions["lighting"] += sorted({o["lighting_bin"] for o in instances}-set(definitions["lighting"]))
    for dimension, labels in definitions.items():
        entries = []
        for label in labels:
            if dimension == "observation":
                if label == "all_gt":
                    selected = instances
                elif label.startswith("size_only"):
                    selected = [o for o in instances if o["height_px"] >= 40 and o["short_side_px"] >= (16 if label.endswith("16") else 12)]
                elif label == "distance_0.4-1.0m":
                    selected = [o for o in instances if o["distance_m"] is not None and .4 <= o["distance_m"] <= 1]
                else:
                    selected = [o for o in instances if priority_observable(o, 16 if label.endswith("16") else 12)]
            else:
                selected = [o for o in instances if o[dimension + "_bin"] == label]
            for cls in ["all", *CLASSES]:
                objects = [o for o in selected if cls == "all" or o["class_id"] == CLASSES.index(cls)]
                tp = sum(o["status"] == "matched" for o in objects)
                entries.append({"bin": label, "class": cls, "gt": len(objects), "tp": tp,
                                "recall": tp/len(objects) if objects else None,
                                "confused": sum(o["status"] == "confused" for o in objects)})
        bins[dimension] = entries
    return {"images": len(image_records), "classes": classes, "bins": bins,
            "geometry_unknown": sum(o["geometry_status"] != "resolved" for o in instances)}


def percent(value):
    return "无样本" if value is None else f"{100*value:.1f}%"


def create_report(output, stats, instances, image_records, dataset, mode, provenance):
    from PIL import Image, ImageDraw
    title = {"val": "观察距离专项评估", "train-v7": "V7 训练集困难样本诊断（非泛化成绩）",
             "benchmark": "独立观察条件验收（原验证空间的新视角）",
             "train-playback": "新消防员视角回放（训练空间重合，非独立验证）"}[mode]
    primary = [r for r in stats["bins"]["observation"] if r["bin"] == "distance_0.4-1.0m_h40_s12" and r["class"] != "all"]
    coverage = "主要观察条件（0.4–1.0m、目标高≥40px、短边≥12px）的类别覆盖：" + "；".join(
        f"{CN[r['class']]} {r['tp']}/{r['gt']}" + ("，没有样本，无法评估" if r["gt"] == 0 else "") for r in primary) + "。"
    if mode in ("val", "benchmark"):
        coverage += " 近景缺样本的类别仍需独立采集评估；不能用训练集的高召回补作验证成绩。"
    diagnostic = ""
    if mode == "benchmark":
        designated = [o for o in instances if o.get("designated_target")]
        class_details = []
        for cls in range(3):
            selected = [o for o in designated if o["class_id"] == cls]
            pose_count = len({o["pose_group"] for o in selected})
            class_details.append(f"{CN[CLASSES[cls]]} {pose_count} 个位姿、{sum(o['status']=='matched' for o in selected)}/{len(selected)} 个预定目标检出")
        diagnostic = ("独立采集先固定几何条件、再进行模型预测：" + "；".join(class_details) +
                      "。每个位姿包含两种光照与左右眼，4张图片不等于4个独立场景。原验证空间未找到符合距离、角度和尺寸条件的消防员可达机位，该类近景仍未独立验证。")
    if mode == "val":
        near_firefighters = [o for o in instances if o["class_id"] == 1 and o["distance_m"] is not None and .4 <= o["distance_m"] <= 1]
        if near_firefighters:
            diagnostic = (f"原验证集近距消防员 {len(near_firefighters)} 个，短边范围 "
                          f"{min(o['short_side_px'] for o in near_firefighters)}–{max(o['short_side_px'] for o in near_firefighters)}px，"
                          f"墙面夹角 {min(o['obliquity_deg'] for o in near_firefighters):.1f}–{max(o['obliquity_deg'] for o in near_firefighters):.1f}°；"
                          "这些侧看成细线的目标不能代表正面楼前观察效果。距离近不等于目标像素足够。")
    lines = [f"# {title}", "", "本次诊断仅进行 CPU 推理，未启动训练，也未调用测试集。", "",
             "固定 conf=0.25、NMS IoU=0.7、匹配 IoU≥0.5；640 输入，置信度从高到低同类一对一匹配。",
             "本报告 P/R 不是 Ultralytics 自动选取 F1 工作点的 P/R，不能与之前表格直接相减。",
             "距离使用每个壁纸目标源框中心到实际单眼光心的三维距离；角度是该视线与墙面法线的锐角。",
             "仅真值目标能确定距离、夹角及真实尺寸，因此这些分箱只报 GT/TP/召回；不伪造分箱精确率。",
             "左右眼、同姿态不同光照存在相关性，GT 框数不是独立现实试验次数。所有远处、极小和遮挡目标保留。",
             "距离与像素条件是观察策略候选门槛，不是相机标定结果或真实 GS130W 成功保证。", "",
             "**" + coverage + "**", "",
             diagnostic, "",
             "unknown 保留在总计及像素分组。窄墙会裁去部分源壁纸，部分人物仍可见但源框中心不落在墙面UV覆盖内；此时不把墙外外推点冒充目标位置。",
             "光照按生成配置分组，并未与距离、角度解耦；分组差异不能单独归因于照明或角度。", "",
             f"图片 {stats['images']} 张，GT {stats['classes']['all']['gt']} 个，几何无法解析 {stats['geometry_unknown']} 个。", "",
             "|类别|GT|TP|预测框|精确率|召回率|类别混淆|", "|---|---:|---:|---:|---:|---:|---:|"]
    for cls, row in stats["classes"].items():
        lines.append(f"|{CN[cls]}|{row['gt']}|{row['tp']}|{row['predictions']}|{percent(row['precision'])}|{percent(row['recall'])}|{row['confused']}|")
    sections = {"observation": "主要观察区间", "distance": "距离", "angle": "墙面夹角", "size": "目标像素尺寸", "lighting": "光照"}
    html_tables = []
    for dimension, caption in sections.items():
        lines.extend(["", f"## {caption}", "", "|分组|类别|GT|TP|召回率|混淆|", "|---|---|---:|---:|---:|---:|"])
        cells = []
        for row in stats["bins"][dimension]:
            vals = [row["bin"], CN[row["class"]], row["gt"], row["tp"], percent(row["recall"]), row["confused"]]
            lines.append("|" + "|".join(map(str, vals)) + "|")
            cells.append("<tr>" + "".join(f"<td>{html.escape(str(v))}</td>" for v in vals) + "</tr>")
        html_tables.append(f"<section><h2>{caption}</h2><div class=table><table><thead><tr><th>分组</th><th>类别</th><th>GT</th><th>TP</th><th>召回率</th><th>混淆</th></tr></thead><tbody>{''.join(cells)}</tbody></table></div></section>")
    by_sample = collections.defaultdict(list)
    for obj in instances:
        by_sample[obj["sample_id"]].append(obj)
    record_by_id = {r["sample_id"]: r for r in image_records}
    failures = [o for o in instances if o["status"] != "matched" and o["class_id"] in (1, 2)]
    failures.sort(key=lambda o: (not o["priority_observable"], o["status"] != "confused", -o["height_px"]))
    candidates = failures
    if mode in ("benchmark", "train-playback"):
        # Every intentional target was chosen by geometry before prediction.
        # Include these whether correct or failed, avoiding success cherry-picks.
        designated = [o for o in instances if o.get("designated_target")]
        designated.sort(key=lambda o: (o["class_id"], o["sample_id"]))
        queues = [iter([o for o in designated if o["class_id"] == cls]) for cls in range(3)]
        candidates = []
        for _ in range(max((sum(o["class_id"] == cls for o in designated) for cls in range(3)), default=0)):
            for queue in queues:
                item = next(queue, None)
                if item is not None:
                    candidates.append(item)
        candidates += failures
    chosen = []
    seen = set()
    for obj in candidates:
        key = obj["sample_id"]
        if key not in seen:
            chosen.append(obj); seen.add(key)
        if len(chosen) >= 12:
            break
    (output / "examples").mkdir(exist_ok=True)
    cards = []
    examples = []
    for index, obj in enumerate(chosen):
        rec = record_by_id[obj["sample_id"]]
        image = Image.open(dataset / rec["image"]).convert("RGB")
        draw = ImageDraw.Draw(image)
        for gt in by_sample[obj["sample_id"]]:
            colour = "#4ade80" if gt["status"] == "matched" else "#ff647c"
            draw.rectangle(gt["bbox_xyxy"], outline=colour, width=2)
        for pred in rec["predictions"]:
            draw.rectangle(pred["bbox_xyxy"], outline="#47b5ff", width=1)
            draw.text((pred["bbox_xyxy"][0], max(0, pred["bbox_xyxy"][1]-12)), f"{CLASSES[pred['class_id']]} {pred['confidence']:.2f}", fill="#47b5ff")
        x0,y0,x1,y1 = obj["bbox_xyxy"]
        crop = image.crop((max(0,x0-35),max(0,y0-35),min(image.width,x1+35),min(image.height,y1+35)))
        crop.thumbnail((240,180)); crop = crop.resize((crop.width*2,crop.height*2))
        target = f"examples/{index:02d}_{obj['sample_id']}.jpg"
        image.save(output / target, quality=92)
        crop_target = target.removesuffix(".jpg") + "_crop.jpg"
        crop.save(output / crop_target, quality=92)
        distance = "unknown" if obj["distance_m"] is None else f"{obj['distance_m']:.2f}m"
        angle = "unknown" if obj["obliquity_deg"] is None else f"{obj['obliquity_deg']:.1f}°"
        caption = f"{CN[CLASSES[obj['class_id']]]} · {obj['status']} · {distance} · {angle} · {obj['width_px']}×{obj['height_px']}px · {obj['lighting_profile']}"
        if mode == "train-playback":
            caption += " · " + ("有实际遮挡板" if obj.get("capture_occluder") else "无遮挡板")
        cards.append(f"<article><a href='{target}'><img src='{target}' alt='{html.escape(caption)}'></a><img class=crop src='{crop_target}' alt='目标局部放大'><p>{html.escape(caption)}</p><small>{html.escape(obj['sample_id'])}</small></article>")
        examples.append({"image":target,"crop":crop_target,"target":obj})
    lines.extend(["", "## 示例解释", "", "绿色框是检出的真值；红色框是真值漏检/混淆；蓝色框是模型预测。局部放大仅供查看，不参与推理。",
                  "验证/训练诊断样例优先选择主要观察条件下的失败及混淆；专用验收/回放先按类别展示预定目标，不论预测成败。均不是随机成功率展示。",
                  "", "[离线交互报告](review.html) · [原始结果](metrics.json) · [逐目标数据](instances.jsonl)", ""])
    (output / "报告.md").write_text("\n".join(lines), encoding="utf8")
    write_json(output / "examples.json", examples)
    overall = stats["classes"]["all"]
    doc = f"""<!doctype html><html lang=zh-CN><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'><title>{title}</title>
<style>body{{margin:0;background:#edf2f8;color:#14233a;font:15px 'Segoe UI','Microsoft Yahei',sans-serif}}main{{max-width:1300px;margin:32px auto;padding:0 22px}}h1{{font-size:30px}}p{{line-height:1.8}}header,section,article{{background:white;border:1px solid #dce5f0;border-radius:16px;padding:22px;margin:18px 0}}.badge{{display:inline-block;padding:8px 14px;background:#e6efff;border-radius:12px;margin:3px}}.notice{{border-left:4px solid #e9a23b;padding-left:14px}}.table{{overflow:auto}}table{{border-collapse:collapse;width:100%;font-size:13px}}th,td{{text-align:left;padding:8px;border-bottom:1px solid #e5ebf3}}th{{background:#f2f6fc}}tr:hover{{background:#eff7ff}}.gallery{{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:16px}}article{{margin:0}}img{{max-width:100%;border-radius:8px}}.crop{{max-height:160px;margin-top:10px}}select{{font:inherit;padding:8px;border:1px solid #aabbd0;border-radius:8px}}small{{color:#526a88}}</style>
<main><header><h1>{title}</h1><p class=notice>本次诊断仅 CPU 评估 · 未启动训练 · 未调用测试集。{ {'val':'固定完整 252 张验证图；没有移除远处或极小真值。','train-v7':'训练集诊断只用于定向补样本，不能作为泛化能力成绩。','benchmark':'按预定几何条件采集的新视角，用于条件覆盖验收；同一壁纸且帧间相关，不能替代随机整体泛化评估。','train-playback':'当前冻结模型未见这些新图片，但拍摄空间属于训练格，不能作为独立验证；未来加入V8训练后，这份旧结果不代表新模型。'}[mode]}</p>
<span class=badge>{stats['images']} 张图片</span><span class=badge>{overall['gt']} 个 GT</span><span class=badge>P {percent(overall['precision'])}</span><span class=badge>R {percent(overall['recall'])}</span>
<p class=notice><b>{html.escape(coverage)}</b></p>
<p>{html.escape(diagnostic)}</p>
<p>conf=0.25 / IoU≥0.5 / 640 输入。P/R 与此前自动 F1 工作点口径不同。每个目标独立计算距离和墙面夹角；分箱只显示召回率。不存在的分箱显示“无样本”。</p><p>绿色真值检出，红色真值漏检或混淆，蓝色模型预测。验证/训练诊断优先展示失败；专用验收先展示预定目标，不论预测成败。示例不是随机成功率。</p>
<p>unknown 保留在总计及像素分组；部分源框中心落在窄墙 UV 裁切外，不将墙外外推点当作可见目标。光照、距离、角度存在混杂，分组差异不是独立因果效果。</p>
<label>类别筛选 <select id=classFilter><option value=''>全部类别</option><option>火焰</option><option>消防员</option><option>人物</option><option>全部</option></select></label></header>
{''.join(html_tables)}<section><h2>预测对照、漏检与混淆示例</h2><div class=gallery>{''.join(cards)}</div></section>
<p>仅当前壁纸与合成相机条件；GS130W 实拍、真实畸变及光照尚待验证。<a href=metrics.json>完整 JSON</a> · <a href='报告.md'>中文报告</a></p></main>
<script>document.getElementById('classFilter').addEventListener('change',e=>document.querySelectorAll('tbody tr').forEach(r=>r.hidden=!!e.target.value&&r.children[1].textContent!==e.target.value));</script></html>"""
    (output / "review.html").write_text(doc, encoding="utf8")


def run(args):
    os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".ultralytics"))
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(args.threads)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    dataset = args.dataset.resolve()
    rows = [json.loads(line) for line in (dataset / "metadata.jsonl").read_text(encoding="utf8").splitlines()]
    if args.mode == "benchmark":
        manifest = read_json(dataset / "manifest.json")
        plan = read_json(Path(manifest["plan_path"]))
        if not plan.get("evaluation_only") or any(p["split"] != "val" for p in plan["pairs"]):
            raise RuntimeError("Benchmark must be evaluation-only, all original-val-cell captures")
        expected = len(plan["pairs"]) * 2
        if any(r["split"] != "val" or r["variant"] != "original" for r in rows):
            raise RuntimeError("Benchmark package contains non-validation or augmented rows")
        for row in rows:
            row.update(source_version="benchmark", source_pair_id=row["pair_id"])
    elif args.mode == "train-playback":
        rows = select_v8_playback(rows)
        expected = 12
        write_json(output / "selection_before_inference.json", {
            "selection_model_blind": True, "training_space_overlap": True, "independent_validation": False,
            "rule": "First sorted unused pose per declared angle/occluder stratum; both lights, left eye only",
            "samples": [{"sample_id": r["sample_id"], "pose_group": r["pose_group"], "group_id": r["group_id"],
                         "image_sha256": r["image_sha256"], "angle_bin": r["capture_plan"]["angle_bin"],
                         "lighting_profile": r["capture_plan"]["lighting_profile"],
                         "occluded": bool(r["capture_plan"].get("occluder"))} for r in rows]})
    else:
        rows = [r for r in rows if r["split"] == ("val" if args.mode == "val" else "train") and
                (args.mode == "val" or (r["source_version"] == "v7" and r["variant"] == "original"))]
        expected = 252 if args.mode == "val" else 720
    if len(rows) != expected:
        raise RuntimeError(f"Frozen {args.mode} image count expected {expected}, got {len(rows)}")
    if args.mode == "val":
        listed = [(dataset / p).resolve() for p in (dataset / "val.txt").read_text(encoding="utf8").splitlines()]
        if [str((dataset / r["image"]).resolve()) for r in rows] != list(map(str, listed)):
            raise RuntimeError("Validation metadata and frozen val.txt order differ")
    provenance = {"schema": "observation-evaluation-v1", "model": str(args.model.resolve()), "model_sha256": digest(args.model),
                  "dataset": str(dataset), "dataset_manifest_sha256": digest(dataset / "manifest.json"),
                  "mode": args.mode, "device": "cpu", "confidence": .25, "match_iou": .5, "nms_iou": .7,
                  "imgsz": 640, "test_evaluated": False, "training_started": False,
                  "sample_selection": {"val": "all frozen val", "train-v7": "V7 original train images only",
                                       "benchmark": "new model-blind original-val-space observation acceptance captures",
                                       "train-playback": "six V8 firefighter-target poses selected by angle/occlusion metadata, both lights, left eye only; train-space overlap, not independent validation"}[args.mode],
                  "geometry": "per-instance source bbox centre mapped through frozen GLB UV triangles; actual per-eye world origin",
                  "distance_definition": "3D Euclidean eye-to-artwork-centre metres",
                  "angle_definition": "acute angle between wall normal and eye-to-artwork-centre ray degrees",
                  "precision_in_gt_bins": "undefined; reported only for entire set and classes",
                  "geometry_files": {}}
    geometries = {}
    for version in sorted({r["source_version"] for r in rows}):
        folder, glb = SOURCES[version]
        source = ROOT / "source_data" / folder
        glb_path, annotation_path = source / glb, source / "source_annotations/annotations.json"
        geometries[version] = glb_geometry(glb_path, annotation_path)
        provenance["geometry_files"][version] = {"glb_sha256": digest(glb_path), "annotations_sha256": digest(annotation_path),
                                                 "resolved_source_targets": len(geometries[version])}
    selected_hash = hashlib.sha256("\n".join(r["sample_id"]+":"+r["image_sha256"] for r in rows).encode()).hexdigest()
    provenance["selected_images_fingerprint"] = selected_hash
    cache = output / "predictions.jsonl"
    meta_path = output / "prediction_provenance.json"
    if cache.exists():
        if not args.reuse_predictions or read_json(meta_path) != provenance:
            raise RuntimeError("Prediction output exists; reuse requires exact provenance and --reuse-predictions")
        image_records = [json.loads(line) for line in cache.read_text(encoding="utf8").splitlines()]
        if [r["sample_id"] for r in image_records] != [r["sample_id"] for r in rows]:
            raise RuntimeError("Incomplete prediction cache")
    else:
        model = YOLO(str(args.model))
        image_records = []
        start = time.monotonic()
        write_json(meta_path, provenance)
        with cache.open("w", encoding="utf8") as stream:
            for i in range(0, len(rows), args.batch):
                batch = rows[i:i + args.batch]
                results = model.predict([str(dataset / r["image"]) for r in batch], device="cpu", imgsz=640,
                                        conf=.25, iou=.7, max_det=300, verbose=False, save=False)
                for row, result in zip(batch, results):
                    boxes = result.boxes
                    preds = [{"class_id": int(cls), "confidence": float(conf), "bbox_xyxy": [float(v) for v in box]}
                             for box, cls, conf in zip(boxes.xyxy.tolist(), boxes.cls.tolist(), boxes.conf.tolist())]
                    record = {"sample_id": row["sample_id"], "image": row["image"], "predictions": preds}
                    image_records.append(record)
                    stream.write(json.dumps(record, ensure_ascii=False) + "\n")
                stream.flush()
                if i % 40 == 0 or i+args.batch >= len(rows):
                    print(f"{args.mode}: {min(i+args.batch,len(rows))}/{len(rows)} images, {time.monotonic()-start:.1f}s", flush=True)
    instances = []
    for row, record in zip(rows, image_records):
        gt = read_json(dataset / row["annotation"])["instances"]
        statuses, true_indices = match_predictions(gt, record["predictions"])
        for obj, status in zip(gt, statuses):
            box = obj["bbox_xyxy"]
            source = obj.get("source", {})
            item = {"sample_id": row["sample_id"], "source_pair_id": row["source_pair_id"], "group_id": row["group_id"],
                    "pose_group": row["pose_group"], "source_version": row["source_version"], "image": row["image"],
                    "instance_id": obj["instance_id"], "node": source.get("node"), "source_instance": source.get("source_instance"),
                    "class_id": obj["class_id"], "class_name": CLASSES[obj["class_id"]], "bbox_xyxy": box,
                    "height_px": box[3]-box[1], "width_px": box[2]-box[0], "short_side_px": min(box[3]-box[1],box[2]-box[0]),
                    "visible_area_px": obj["area_px"], "touches_image_border": obj.get("touches_image_border"),
                    "capture_occluder": bool(row["capture_plan"].get("occluder")),
                    "capture_angle_bin": row["capture_plan"].get("angle_bin"),
                    "lighting_profile": row["capture_plan"].get("lighting_profile", "unknown"), **status,
                    **instance_geometry(obj, row, geometries[row["source_version"]])}
            designated = row["capture_plan"].get("sampling_target")
            item["designated_target"] = bool(designated and source.get("node") == designated["node"] and source.get("source_instance") == designated["source_instance"])
            item.update(distance_bin=distance_bin(item["distance_m"]), angle_bin=angle_bin(item["obliquity_deg"]),
                        size_bin=size_bin(item["height_px"],item["short_side_px"]), lighting_bin=item["lighting_profile"],
                        priority_observable=priority_observable(item))
            if status["prediction_index"] is not None:
                item["prediction"] = record["predictions"][status["prediction_index"]]
            instances.append(item)
    with (output / "instances.jsonl").open("w", encoding="utf8") as stream:
        for item in instances:
            stream.write(json.dumps(item, ensure_ascii=False)+"\n")
    if args.mode == "train-v7":
        hard = [o for o in instances if o["status"] != "matched" and o["class_id"] in (1, 2)]
        hard.sort(key=lambda o: (not o["priority_observable"], o["status"] != "confused", -o["height_px"]))
        with (output / "hard_examples.jsonl").open("w", encoding="utf8") as stream:
            for item in hard:
                stream.write(json.dumps({"diagnostic_scope":"train-only; not generalization", **item}, ensure_ascii=False)+"\n")
    stats = summary(instances, image_records)
    if args.mode in ("benchmark", "train-playback"):
        designated = [i for i in instances if i["designated_target"]]
        designated_summary = summary(designated, image_records)
        # Only recalls have a defensible denominator for designated targets.
        for entry in designated_summary["classes"].values():
            for key in ["predictions", "precision", "fp"]:
                entry.pop(key)
        write_json(output / "designated_target_recall.json", {
            "scope": "Only the predeclared target, selected by geometry before predictions; no per-target-subset precision",
            **designated_summary})
    write_json(output / "metrics.json", {"provenance": provenance, **stats})
    create_report(output, stats, instances, image_records, dataset, args.mode, provenance)
    print(json.dumps({"output": str(output), "images": stats["images"], "classes": stats["classes"], "geometry_unknown": stats["geometry_unknown"]}, ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=ROOT / "dataset_people_v2")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--mode", choices=["val", "train-v7", "benchmark", "train-playback"], default="val")
    parser.add_argument("--output", type=Path, default=ROOT / "reports/observation_v1/val")
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--reuse-predictions", action="store_true")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
