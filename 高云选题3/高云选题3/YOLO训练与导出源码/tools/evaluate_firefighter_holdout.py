"""Compare immutable model snapshots on every predeclared independent-scene image.

No training, image selection based on predictions, or GPU operations are used.
GT-conditioned subsets have recall only: false positives lack true geometry.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import html
import json
import os
from pathlib import Path
import time

from evaluate_observation import CLASSES, CN, match_predictions

ROOT = Path(__file__).resolve().parents[1]
MODEL_LABELS = {"old_v7_best": "上轮最佳 · 第30轮", "current_v8_best": "本轮快照 · 第16轮"}


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, obj):
    Path(path).write_text(json.dumps(obj, ensure_ascii=False, indent=2) + "\n", encoding="utf8")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def percentage(value):
    return "无样本" if value is None else f"{value * 100:.1f}%"


def qualified(obj):
    distance = obj.get("distance_m")
    angle = obj.get("angle_deg")
    return (distance is not None and .4 <= distance <= 1.0 and
            angle is not None and 0 <= angle <= 55 and
            obj["height_px"] >= 40 and obj["short_side_px"] >= 12)


def recall_count(objects):
    tp = sum(o["status"] == "matched" for o in objects)
    confusion = collections.Counter(CLASSES[o["predicted_class_id"]] for o in objects if o["status"] == "confused")
    return {"gt": len(objects), "tp": tp, "fn": len(objects) - tp,
            "missed_without_class_match": sum(o["status"] == "missed" for o in objects),
            "confused": sum(confusion.values()), "confused_as": dict(confusion),
            "recall": tp / len(objects) if objects else None}


def summarize(instances, records):
    classes = {}
    for label in ["all", *CLASSES]:
        cid = None if label == "all" else CLASSES.index(label)
        selected = [o for o in instances if cid is None or o["class_id"] == cid]
        count = sum(sum(cid is None or p["class_id"] == cid for p in r["predictions"]) for r in records)
        result = recall_count(selected)
        result.update(predictions=count, fp=count-result["tp"], precision=result["tp"] / count if count else None)
        classes[label] = result
    designated = [o for o in instances if o["designated_target"]]
    firefighter = [o for o in instances if o["class_id"] == 1]
    definitions = {
        "scope": {
            "firefighter_all": firefighter,
            "designated_firefighter": designated,
            "firefighter_0.4-1m_h40_s12_angle55": [o for o in firefighter if qualified(o)],
            "designated_0.4-1m_h40_s12_angle55": [o for o in designated if qualified(o)],
        },
        "distance": {
            "<0.4m": [o for o in firefighter if o.get("distance_m") is not None and o["distance_m"] < .4],
            "0.4-0.7m": [o for o in firefighter if o.get("distance_m") is not None and .4 <= o["distance_m"] < .7],
            "0.7-1.0m": [o for o in firefighter if o.get("distance_m") is not None and .7 <= o["distance_m"] <= 1],
            ">1.0m": [o for o in firefighter if o.get("distance_m") is not None and o["distance_m"] > 1],
            "unknown": [o for o in firefighter if o.get("distance_m") is None],
        },
        "angle": {
            "0-20deg": [o for o in firefighter if o.get("angle_deg") is not None and o["angle_deg"] < 20],
            "20-40deg": [o for o in firefighter if o.get("angle_deg") is not None and 20 <= o["angle_deg"] < 40],
            "40-55deg": [o for o in firefighter if o.get("angle_deg") is not None and 40 <= o["angle_deg"] <= 55],
            ">55deg": [o for o in firefighter if o.get("angle_deg") is not None and o["angle_deg"] > 55],
            "unknown": [o for o in firefighter if o.get("angle_deg") is None],
        },
        "lighting": {label: [o for o in firefighter if o["lighting"] == label]
                     for label in sorted({o["lighting"] for o in instances})},
        "occlusion": {label: [o for o in firefighter if o["occlusion"] == label]
                      for label in sorted({o["occlusion"] for o in instances})},
        "pixels": {
            "h<40px": [o for o in firefighter if o["height_px"] < 40],
            "h>=40_short<12px": [o for o in firefighter if o["height_px"] >= 40 and o["short_side_px"] < 12],
            "h>=40_12<=short<16px": [o for o in firefighter if o["height_px"] >= 40 and 12 <= o["short_side_px"] < 16],
            "h>=40_short>=16px": [o for o in firefighter if o["height_px"] >= 40 and o["short_side_px"] >= 16],
        },
    }
    bins = {dimension: [{"bin": label, **recall_count(objects)} for label, objects in groups.items()]
            for dimension, groups in definitions.items()}
    poses = collections.defaultdict(list)
    for obj in designated:
        poses[obj["pose_id"]].append(obj)
    pose_summary = [{"pose_id": pid, "images": len(objects), **recall_count(objects)}
                    for pid, objects in sorted(poses.items())]
    gt_by_image = collections.defaultdict(list)
    for obj in instances:
        gt_by_image[obj["sample_id"]].append(obj)
    negative_records = [r for r in records if r.get("designated_target_id") is None]
    if any(any(o["class_id"] == 1 for o in gt_by_image[r["sample_id"]]) for r in negative_records):
        raise ValueError("Negative control unexpectedly has firefighter GT")
    negative_summary = {"images": len(negative_records),
                        "firefighter_false_positive_images": sum(any(p["class_id"] == 1 for p in r["predictions"]) for r in negative_records),
                        "firefighter_false_positive_boxes": sum(sum(p["class_id"] == 1 for p in r["predictions"]) for r in negative_records)}
    return {"images": len(records), "classes": classes, "bins": bins, "negative_controls": negative_summary,
            "designated_pose_summary": pose_summary,
            "poses_with_every_target_image_detected": sum(p["tp"] == p["gt"] for p in pose_summary),
            "poses_with_at_least_one_target_image_detected": sum(p["tp"] > 0 for p in pose_summary),
            "unique_camera_poses": len(poses)}


def enrich_instances(row, annotation, predictions):
    gt = annotation["instances"]
    statuses, _ = match_predictions(gt, predictions, .5)
    designated_id = row["designated_target_id"]
    out = []
    for obj, status in zip(gt, statuses):
        x0, y0, x1, y1 = obj["bbox_xyxy"]
        item = {**obj, **status, "sample_id": row["id"], "pose_id": row["pose_id"],
                "pair_id": row["pair_id"], "scene_id": row["scene_id"], "eye": row["eye"],
                "lighting": row["lighting"], "occlusion": row["occlusion"],
                "height_px": y1-y0, "short_side_px": min(x1-x0, y1-y0),
                "designated_target": designated_id is not None and str(obj["instance_id"]) == str(designated_id)}
        item["observation_qualified"] = qualified(item)
        out.append(item)
    if sum(o["designated_target"] for o in out) != (0 if designated_id is None else 1):
        raise ValueError(f"Missing or duplicate designated target in {row['id']}; do not silently drop failed captures")
    return out


def verify_rows(dataset, rows):
    ids = [r["id"] for r in rows]
    if not ids or len(ids) != len(set(ids)):
        raise ValueError("Manifest must have nonempty unique IDs")
    pairs = collections.defaultdict(list)
    seen_hashes = set()
    for row in rows:
        actual = sha(dataset / row["image"])
        if row.get("image_sha256") and actual != row["image_sha256"]:
            raise ValueError(f"Image changed: {row['id']}")
        row["image_sha256"] = actual
        annotation_sha = sha(dataset / row["annotation"])
        if row.get("annotation_sha256") and annotation_sha != row["annotation_sha256"]:
            raise ValueError(f"Annotation changed: {row['id']}")
        row["annotation_sha256"] = annotation_sha
        if actual in seen_hashes:
            raise ValueError("Duplicate image bytes in benchmark")
        seen_hashes.add(actual)
        pairs[row["pair_id"]].append(row)
    if any(sorted(r["eye"] for r in group) != ["L", "R"] for group in pairs.values()):
        raise ValueError("Each pair must contain exactly both eyes")
    return {"images": len(rows), "stereo_pairs": len(pairs), "camera_poses": len({r["pose_id"] for r in rows}),
            "scenes": len({r["scene_id"] for r in rows}),
            "image_annotation_fingerprint": hashlib.sha256("\n".join(r["id"]+":"+r["image_sha256"]+":"+r["annotation_sha256"] for r in rows).encode()).hexdigest()}


def predict(snapshot, rows, dataset, output, reuse=False):
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(1)
    output.mkdir(parents=True, exist_ok=True)
    cache = output / "predictions.jsonl"
    provenance_path = output / "prediction_provenance.json"
    provenance = {"snapshot_sha256": snapshot["sha256"], "snapshot_path": snapshot["snapshot"],
                  "samples": [{"id": r["id"], "image_sha256": r["image_sha256"]} for r in rows],
                  "device": "cpu", "threads": 1, "imgsz": 640, "conf": .25, "nms_iou": .7, "max_det": 300}
    if sha(snapshot["snapshot"]) != snapshot["sha256"]:
        raise ValueError("Frozen model hash changed")
    if cache.exists():
        if not reuse or read_json(provenance_path) != provenance:
            raise ValueError("Existing predictions need --reuse-predictions and exact provenance")
        records = [json.loads(line) for line in cache.read_text(encoding="utf8").splitlines()]
        if [r["sample_id"] for r in records] != [r["id"] for r in rows]:
            raise ValueError("Prediction cache incomplete or misordered")
        return records
    model = YOLO(snapshot["snapshot"])
    if [model.names[i] for i in range(3)] != CLASSES:
        raise ValueError("Model classes do not match benchmark")
    write_json(provenance_path, provenance)
    records = []
    start = time.monotonic()
    with cache.open("w", encoding="utf8") as stream:
        for i, row in enumerate(rows):
            result = model.predict(str(dataset / row["image"]), device="cpu", imgsz=640, conf=.25, iou=.7,
                                   max_det=300, verbose=False, save=False)[0]
            boxes = result.boxes
            preds = [{"class_id": int(cid), "confidence": float(conf), "bbox_xyxy": list(map(float, box))}
                     for box, cid, conf in zip(boxes.xyxy.tolist(), boxes.cls.tolist(), boxes.conf.tolist())]
            record = {"sample_id": row["id"], "image": row["image"], "predictions": preds,
                      "designated_target_id": row["designated_target_id"]}
            records.append(record)
            stream.write(json.dumps(record, ensure_ascii=False)+"\n")
            stream.flush()
            if i % 12 == 0 or i == len(rows)-1:
                print(f"{Path(snapshot['snapshot']).stem}: {i+1}/{len(rows)} images, {time.monotonic()-start:.1f}s", flush=True)
    if sha(snapshot["snapshot"]) != snapshot["sha256"]:
        raise ValueError("Frozen model changed during inference")
    return records


def make_visuals(dataset, output, rows, all_instances, all_records):
    from PIL import Image, ImageDraw
    model_images = {}
    for model, instances in all_instances.items():
        folder = output / model / "overlays"
        folder.mkdir(exist_ok=True)
        by_id = collections.defaultdict(list)
        for obj in instances:
            by_id[obj["sample_id"]].append(obj)
        model_images[model] = {}
        for row, record in zip(rows, all_records[model]):
            image = Image.open(dataset / row["image"]).convert("RGB")
            draw = ImageDraw.Draw(image)
            for obj in by_id[row["id"]]:
                color = "#00e687" if obj["status"] == "matched" else "#ff596b"
                box = obj["bbox_xyxy"]
                draw.rectangle(box, outline=color, width=3 if obj["designated_target"] else 2)
                if obj["designated_target"]:
                    draw.text((box[0], min(image.height-12,box[3]+3)), "TARGET GT", fill=color)
            for pred in record["predictions"]:
                box = pred["bbox_xyxy"]
                draw.rectangle(box, outline="#57bdff", width=1)
                caption = f"{CLASSES[pred['class_id']]} {pred['confidence']:.2f}"
                draw.text((max(0,box[0]), max(0,box[1]-12)), caption, fill="#57bdff", stroke_width=1, stroke_fill="#152438")
            path = folder / (row["id"]+".jpg")
            image.save(path, quality=91)
            target = next((o for o in by_id[row["id"]] if o["designated_target"]), None)
            model_images[model][row["id"]] = {"overlay": path.relative_to(output).as_posix(),
                                             "status": target["status"] if target else ("false_alarm" if any(p["class_id"]==1 for p in record["predictions"]) else "negative_clear"),
                                             "qualified": target["observation_qualified"] if target else False}
    # Original images are copied for a portable report; all evaluation images are retained.
    originals = output / "originals"
    originals.mkdir(exist_ok=True)
    from shutil import copyfile
    for row in rows:
        copyfile(dataset / row["image"], originals / (row["id"]+".png"))
    return model_images


def create_report(output, rows, stats, provenance, all_instances, visuals):
    snapshots = provenance["snapshots"]
    counts = provenance["dataset_counts"]
    old_epoch = snapshots["models"]["old_v7_best"]["epoch"]
    current_epoch = snapshots["models"]["current_v8_best"]["epoch"]
    lines = ["# 消防员独立场景近景验证", "",
             "**合成验证，不是实拍。** 新场景、摄影位姿和图像未进入训练；使用的消防员、人物与火焰原画与训练相同。",
             "本报告验证同壁纸在新场景/近景观察条件下的表现，不能证明对新消防员形象或真实 GS130W 的泛化。", "",
             f"完整预定样本：{counts['images']} 张、{counts['stereo_pairs']} 对、{counts['camera_poses']} 个机位、{counts['scenes']} 个场景。左右眼、光照及遮挡变体相关，图片数不是独立实验次数。",
             f"模型冻结于 {snapshots['created_at']}；上轮第{old_epoch}轮最佳与本轮第{current_epoch}轮最佳快照。当前训练已按用户要求暂停；暂停时最佳仍为第{current_epoch}轮。本轮评估最佳快照，不是第25轮last。",
             "固定 conf=0.25、NMS IoU=0.7、匹配 IoU≥0.5、输入640。全图全部三类GT保留，置信度从高到低同类一对一匹配；错误类别重叠单独计混淆。",
             "指定目标和距离/角度/光照/遮挡分箱仅统计GT召回；未匹配预测没有真实距离，因此不计算这些GT子集的精确率。全体消防员精确率包含所有消防员预测。", "",
             "|模型|类别|GT|TP|预测数|误检FP|漏检FN|精确率|召回率|类别混淆|", "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    summary_rows = []
    for model, result in stats.items():
        for cls, entry in result["classes"].items():
            vals = [MODEL_LABELS[model], CN[cls], entry["gt"], entry["tp"], entry["predictions"], entry["fp"], entry["fn"], percentage(entry["precision"]), percentage(entry["recall"]), entry["confused"]]
            lines.append("|"+"|".join(map(str, vals))+"|")
            summary_rows.append("<tr>"+"".join("<td>"+html.escape(str(v))+"</td>" for v in vals)+"</tr>")
    dimension_titles = {"scope":"主要观察条件与指定目标", "distance":"真实单眼距离", "angle":"视线与墙面法线夹角", "lighting":"照明配置", "occlusion":"遮挡配置", "pixels":"可见框像素尺寸"}
    tables = []
    for dimension, caption in dimension_titles.items():
        lines += ["", f"## {caption}", "", "|模型|消防员分组|GT|TP|FN|召回率|混淆|", "|---|---|---:|---:|---:|---:|---:|"]
        trows=[]
        for model, result in stats.items():
            for row in result["bins"][dimension]:
                vals=[MODEL_LABELS[model],row["bin"],row["gt"],row["tp"],row["fn"],percentage(row["recall"]),row["confused"]]
                lines.append("|"+"|".join(map(str, vals))+"|")
                trows.append("<tr>"+"".join("<td>"+html.escape(str(v))+"</td>" for v in vals)+"</tr>")
        tables.append(f"<details><summary>{caption}</summary><div class=scroll><table><thead><tr><th>模型</th><th>消防员分组</th><th>GT</th><th>TP</th><th>FN</th><th>召回率</th><th>混淆</th></tr></thead><tbody>{''.join(trows)}</tbody></table></div></details>")
    lines += ["", "## 无消防员负例", "", "负例在预测前固定，包括普通人物或空白楼面；统计其中消防员误报框及出现误报的图片。小样本零误报不表示实际零误报。", ""]
    negative_lines = []
    for model, result in stats.items():
        n=result["negative_controls"]
        sentence=f"{MODEL_LABELS[model]}：{n['images']} 张负例，{n['firefighter_false_positive_images']} 张出现消防员误报，共 {n['firefighter_false_positive_boxes']} 个误报框。"
        lines.append(sentence)
        negative_lines.append(sentence)
    lines += ["", "## 解释与复现", "", "主要观察条件：实际单眼距离0.4～1.0米、可见框高≥40px、短边≥12px、墙面夹角≤55°；未知距离或角度不算合格。",
              "目标距离来自标注对象中心到单眼光心的3D欧氏距离，目标高度和短边来自最终可见像素框；没有利用放大图片达到阈值。",
              "遮挡是该采集配置的标签，不表示画面中每个人都被遮住；下部遮挡主要针对预定消防员。",
              "此处是预定条件覆盖验收，不是随机分布抽样或连续视频，不能由静态帧结果推算3/5帧确认成功率。",
              "原图/掩膜/标注/计划均保持可复查；模型预测没有参与取景、挑选或删除样本。",
              "这批数据独立存放，不加入本轮训练集、原验证集或原测试集；查看本报告后的进一步调参会使它成为开发验证集，最终验收还需封存的新样本。",
              "", "[离线交互报告](review.html) · [指标与来源](metrics.json) · [逐帧索引](samples.json)", ""]
    (output / "报告.md").write_text("\n".join(lines),encoding="utf8")
    by_pair=collections.defaultdict(list)
    ref_instances=collections.defaultdict(list)
    for obj in next(iter(all_instances.values())):
        ref_instances[obj["sample_id"]].append(obj)
    for row in rows:
        target=next((o for o in ref_instances[row["id"]] if o["designated_target"]), None)
        by_pair[row["pair_id"]].append({**{k:row[k] for k in ["id","eye","pair_id","pose_id","scene_id","lighting","occlusion"]},
            "distance":target["distance_m"] if target else None, "angle":target["angle_deg"] if target else None,
            "height":target["height_px"] if target else None, "short":target["short_side_px"] if target else None,
            "original":"originals/"+row["id"]+".png", "models":{m:visuals[m][row["id"]] for m in stats}})
    samples=[{"id":pid,"eyes":sorted(group,key=lambda r:r["eye"])} for pid,group in by_pair.items()]
    write_json(output / "samples.json",samples)
    json_data=json.dumps(samples,ensure_ascii=False).replace("<","\\u003c")
    cards=[]
    for model,result in stats.items():
        ff=result["classes"]["firefighter"]
        designated=next(r for r in result["bins"]["scope"] if r["bin"]=="designated_firefighter")
        cards.append(f"<div class=kpi><span>{MODEL_LABELS[model]}</span><strong>{percentage(ff['recall'])}</strong><span>全体消防员召回 · {ff['tp']}/{ff['gt']}</span><p>精确率 {percentage(ff['precision'])} · 指定目标 {designated['tp']}/{designated['gt']}<br>各变体均检出的机位 {result['poses_with_every_target_image_detected']}/{result['unique_camera_poses']}</p></div>")
    doc="""<!doctype html><html lang=zh-CN><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'><title>消防员独立场景验证</title>
<style>:root{font-family:'Segoe UI','Microsoft YaHei',sans-serif;color:#16304c;background:#f0f5fa}*{box-sizing:border-box}body{margin:0}main{max-width:1320px;margin:auto;padding:32px 24px}header,.panel{padding:26px;background:#fff;border:1px solid #dce5ee;border-radius:20px;margin-bottom:20px}h1{font-size:32px;letter-spacing:-1px;margin:10px 0}h2{font-size:21px}p{line-height:1.8}.eyebrow{font-size:12px;letter-spacing:2px;color:#147b83;font-weight:700}.notice{border-left:4px solid #e8a84b;padding:10px 16px;background:#fff8e9;border-radius:3px 10px 10px 3px}.kpis{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}.kpi{border:1px solid #dbe7ef;border-radius:14px;padding:22px;background:linear-gradient(130deg,#f3fbfb,#f3f6ff)}.kpi span{display:block}.kpi strong{display:block;font-size:42px;margin:8px 0;color:#11687d}.kpi p{font-size:13px;margin-bottom:0}.scroll{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:13px;white-space:nowrap}th,td{padding:10px;border-bottom:1px solid #e4edf4;text-align:left}th{background:#f4f7fb}summary{cursor:pointer;padding:14px 0;font-weight:650}details{border-bottom:1px solid #e3eaf1}.controls{display:flex;gap:12px;flex-wrap:wrap;margin:18px 0;align-items:end}label{display:grid;gap:6px;font-size:13px;color:#587087}select,button{border:1px solid #bfcfdd;background:white;color:#203b56;font:inherit;padding:10px 12px;border-radius:9px;max-width:100%;cursor:pointer}button{background:#145f76;color:white;border-color:#145f76}button:disabled{opacity:.4;cursor:default}.gallery{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}figure{margin:0;min-width:0;border:1px solid #dfe8ef;padding:12px;border-radius:14px;background:#f7fafc}figure img{width:100%;display:block;border-radius:8px}figcaption{font-size:13px;line-height:1.8;padding-top:10px;overflow-wrap:anywhere}.legend{font-size:13px}.green{color:#178250}.red{color:#ca304e}.blue{color:#1880b5}.muted{color:#64778d;font-size:13px}.pager{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin-top:16px}a{color:#176e94}#pairTitle{overflow-wrap:anywhere}@media(max-width:650px){main{padding:14px 10px}header,.panel{padding:18px}.kpis,.gallery{grid-template-columns:1fr}h1{font-size:25px}.kpi strong{font-size:35px}.controls>*{max-width:100%}}</style>
<main><header><div class=eyebrow>FIREFIGHTER · HELD-OUT SCENE</div><h1>消防员独立场景近景验证</h1><p>新场景、完整预定样本、冻结模型对照。重点检查楼前近距观察中的漏检与人物混淆。</p>
<p class=notice><b>这是合成验证，不是实拍。</b>场景和机位独立，消防员等壁纸原画与训练相同；结果不能证明真实 GS130W 或新消防员形象的表现。当前训练已按用户要求暂停；本轮对照是21:50冻结的第16轮最佳快照，暂停时最佳仍为第16轮，并非使用第25轮last。</p>
<p>__COUNTS__。左右眼、同机位光照与遮挡变体相关，图片数不等于独立试验次数。取景计划在预测前固定，未按成功结果筛图。</p><div class=kpis>__CARDS__</div></header>
<section class=panel><h2>新旧模型 · 全部真值</h2><p class=muted>conf=0.25 / NMS IoU=0.7 / 匹配 IoU≥0.5 / 640 输入 / CPU 单线程。P/R 与训练日志自动 F1 工作点口径不同。</p><div class=scroll><table><thead><tr><th>模型</th><th>类别</th><th>GT</th><th>TP</th><th>预测数</th><th>FP</th><th>FN</th><th>精确率</th><th>召回率</th><th>混淆</th></tr></thead><tbody>__SUMMARY__</tbody></table></div></section>
<section class=panel><h2>逐对检查 · 左右目与预测框</h2><p class=legend><span class=green>绿色：检出的真值</span>　<span class=red>红色：漏检或混淆的真值</span>　<span class=blue>蓝色：模型预测</span>。TARGET GT 标记预定消防员。</p>
<div class=controls><label>模型<select id=model><option value=current_v8_best>本轮快照 · 第16轮</option><option value=old_v7_best>上轮最佳 · 第30轮</option></select></label><label>显示<select id=display><option value=overlay>真值 + 预测</option><option value=original>原始相机图</option></select></label><label>光照<select id=light><option value=''>全部</option><option value=normal>常规光</option><option value=dim>暗光</option></select></label><label>遮挡<select id=occ><option value=''>全部</option><option value=none>无遮挡</option><option value=lower_panel>下部遮挡</option></select></label><label>指定目标结果<select id=result><option value=''>全部</option><option value=failed>任一目漏检或混淆</option><option value=matched>双目均检出</option><option value=negative>无消防员负例</option></select></label></div>
<h3 id=pairTitle></h3><div id=gallery class=gallery></div><div class=pager><button id=prev>上一对</button><span id=counter></span><button id=next>下一对</button></div><p class=muted>筛选只改变浏览内容；顶部统计始终包含完整预定验证集。原图链接可打开所有像素。静态变体不是连续视频，不能据此推算3/5帧策略的成功率。</p></section>
<section class=panel><h2>无消防员负例</h2><p>__NEGATIVES__</p><p class=muted>普通人物及空白楼面负例在预测前固定。小样本零误报不表示实际零误报。</p></section>
<section class=panel><h2>消防员分组明细</h2><p class=muted>主要观察条件为单眼距离0.4～1.0m、高≥40px、短边≥12px、墙面夹角≤55°；未知角度不算合格。分组是GT条件，只报召回；未匹配预测没有真值距离。遮挡组标记采集配置，不代表所有人均被遮挡。</p>__TABLES__</section>
<section class=panel><h2>使用边界</h2><p>这批数据独立存放，未加入本轮训练集或原验证/测试集。看过结果后若据此调参，它将成为开发验证集；最终实车验收仍需新封存的实拍。相机内参和立体基线沿用临时模拟参数，不能当作相机标定。</p><p><a href=metrics.json>指标及模型指纹 JSON</a> · <a href=报告.md>中文报告</a> · <a href=samples.json>完整逐帧索引</a></p></section></main>
<script>const SAMPLES=__DATA__;let index=0;const $=id=>document.getElementById(id);const statusLabel={matched:'检出',missed:'漏检',confused:'类别混淆',false_alarm:'负例出现消防员误报',negative_clear:'负例无消防员误报'};
function selected(){return SAMPLES.filter(p=>{const r=p.eyes[0],m=$('model').value;return (!$('light').value||r.lighting===$('light').value)&&(!$('occ').value||r.occlusion===$('occ').value)&&(!$('result').value||($('result').value==='negative'?r.distance===null:(r.distance!==null&&($('result').value==='failed'?p.eyes.some(e=>e.models[m].status!=='matched'):p.eyes.every(e=>e.models[m].status==='matched')))));});}
function render(){const list=selected();index=Math.max(0,Math.min(index,list.length-1));const pair=list[index];$('counter').textContent=list.length?`${index+1} / ${list.length} 对`:'0 对';$('prev').disabled=!list.length||index===0;$('next').disabled=!list.length||index===list.length-1;if(!pair){$('pairTitle').textContent='该筛选条件下没有样本';$('gallery').replaceChildren();return;}$('pairTitle').textContent=pair.id;const m=$('model').value;const nodes=pair.eyes.map(r=>{const f=document.createElement('figure'),a=document.createElement('a'),im=document.createElement('img'),c=document.createElement('figcaption');a.href=r.original;a.target='_blank';im.src=$('display').value==='original'?r.original:r.models[m].overlay;im.alt=`${r.eye==='L'?'左':'右'}目 ${pair.id}`;a.append(im);c.textContent=`${r.eye==='L'?'左':'右'}目 · ${statusLabel[r.models[m].status]} · 距离 ${r.distance.toFixed(2)} m · 夹角 ${r.angle.toFixed(1)}° · 高 ${r.height} / 短边 ${r.short} px · ${r.models[m].qualified?'满足主观察条件':'未满足主观察条件'} · ${r.lighting==='dim'?'暗光':'常规光'} · ${r.occlusion==='none'?'无遮挡':'下部遮挡'}`;f.append(a,c);return f;});$('gallery').replaceChildren(...nodes);}
['model','light','occ','result'].forEach(id=>$(id).addEventListener('change',()=>{index=0;render();}));$('display').addEventListener('change',render);$('prev').onclick=()=>{index--;render()};$('next').onclick=()=>{index++;render()};render();</script></html>"""
    counts_text=f"{counts['images']} 张图 / {counts['stereo_pairs']} 对 / {counts['camera_poses']} 个机位 / {counts['scenes']} 个场景"
    doc=doc.replace("r.distance.toFixed(2)","(r.distance===null?'—':r.distance.toFixed(2))").replace("r.angle.toFixed(1)","(r.angle===null?'—':r.angle.toFixed(1))")
    doc=doc.replace("${r.height}","${r.height??'—'}").replace("${r.short}","${r.short??'—'}")
    doc=doc.replace("r.models[m].qualified?'满足主观察条件':'未满足主观察条件'","r.distance===null?'无消防员负例':(r.models[m].qualified?'满足主观察条件':'未满足主观察条件')")
    doc=doc.replace("第16轮",f"第{current_epoch}轮").replace("第30轮",f"第{old_epoch}轮")
    doc=doc.replace("21:50冻结",html.escape(snapshots['created_at'])+" 冻结")
    for key,value in {"__COUNTS__":counts_text,"__CARDS__":"".join(cards),"__SUMMARY__":"".join(summary_rows),"__TABLES__":"".join(tables),"__DATA__":json_data,"__NEGATIVES__":"<br>".join(map(html.escape,negative_lines))}.items():
        doc=doc.replace(key,value)
    (output / "review.html").write_text(doc,encoding="utf8")


def run(args):
    # Set before importing torch/Ultralytics; do not claim or use the training GPU.
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    os.environ.setdefault("YOLO_CONFIG_DIR",str(ROOT / ".ultralytics"))
    dataset=args.dataset.resolve(); output=args.output.resolve()
    usage_path=dataset/'CURRENT_USAGE.json'
    if usage_path.exists() and read_json(usage_path).get('role')=='training':
        raise ValueError('These captures were promoted to training by the user. This independent-holdout evaluator is retired; use a separate training-diagnostic report or obtain new held-out data.')
    output.mkdir(parents=True,exist_ok=True)
    rows=[json.loads(line) for line in (dataset / "manifest.jsonl").read_text(encoding="utf-8-sig").splitlines()]
    counts=verify_rows(dataset,rows)
    snapshots=read_json(args.snapshots)
    MODEL_LABELS.update(old_v7_best=f"上轮最佳 · 第{snapshots['models']['old_v7_best']['epoch']}轮",
                        current_v8_best=f"本轮快照 · 第{snapshots['models']['current_v8_best']['epoch']}轮")
    audit=read_json(args.audit)
    if audit.get("partial") or not (audit.get("passed") is True or audit.get("status") in ("PASS","pass","passed")):
        raise ValueError("Rendered dataset must pass its independent audit before inference")
    if audit.get("manifest_sha256") != sha(dataset / "manifest.jsonl"):
        raise ValueError("Independent audit does not cover the current frozen manifest")
    provenance={"schema":"firefighter-heldout-evaluation-v1","dataset":str(dataset),"dataset_counts":counts,
                "manifest_sha256":sha(dataset / "manifest.jsonl"),"audit_path":str(args.audit.resolve()),
                "audit_sha256":sha(args.audit),"snapshots":snapshots,"training_started":False,
                "test_set_used":False,"data_added_to_training":False,"real_photos":False,
                "training_state_at_evaluation":"paused at explicit user request; do not resume",
                "snapshot_training_continues_is_historical":True,
                "scope":"Independent synthetic scene, same source artwork; no novel identity or real-camera guarantee",
                "cpu_threads":1,"conf":.25,"nms_iou":.7,"match_iou":.5,"imgsz":640,
                "evaluation_tool_sha256":sha(Path(__file__)),
                "matching_tool_sha256":sha(ROOT/"tools/evaluate_observation.py")}
    # This record is written before inference and pins the complete sample list.
    selection={"selection_model_blind":True,"manifest_sha256":provenance["manifest_sha256"],
               "samples":[{"id":r["id"],"image_sha256":r["image_sha256"],"annotation_sha256":r["annotation_sha256"]} for r in rows]}
    selection_path=output / "selection_before_inference.json"
    if selection_path.exists() and read_json(selection_path)!=selection:
        raise ValueError("Predeclared sample selection changed")
    write_json(selection_path,selection)
    all_instances={};all_records={};stats={}
    for name,snapshot in snapshots["models"].items():
        if name not in MODEL_LABELS:
            raise ValueError("Unexpected snapshot identity")
        records=predict(snapshot,rows,dataset,output/name,args.reuse_predictions)
        instances=[]
        for row,record in zip(rows,records):
            instances.extend(enrich_instances(row,read_json(dataset/row["annotation"]),record["predictions"]))
        (output/name/"instances.jsonl").write_text("".join(json.dumps(o,ensure_ascii=False)+"\n" for o in instances),encoding="utf8")
        all_instances[name]=instances;all_records[name]=records;stats[name]=summarize(instances,records)
        stats[name].update(snapshot_epoch=snapshot["epoch"], frozen_at=snapshots["created_at"], snapshot_sha256=snapshot["sha256"])
        write_json(output/name/"metrics.json",stats[name])
    visuals=make_visuals(dataset,output,rows,all_instances,all_records)
    write_json(output/"metrics.json",{"provenance":provenance,"models":stats})
    create_report(output,rows,stats,provenance,all_instances,visuals)
    print(json.dumps({"output":str(output),"counts":counts,"firefighter":{n:s['classes']['firefighter'] for n,s in stats.items()}},ensure_ascii=False),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset",type=Path,default=ROOT/"source_data/firefighter_holdout_v1")
    parser.add_argument("--output",type=Path,default=ROOT/"reports/firefighter_holdout_v1")
    parser.add_argument("--snapshots",type=Path,default=ROOT/"reports/firefighter_holdout_v1/checkpoints/snapshots.json")
    parser.add_argument("--audit",type=Path,default=ROOT/"source_data/firefighter_holdout_v1/independent_audit.json")
    parser.add_argument("--reuse-predictions",action="store_true")
    run(parser.parse_args())


if __name__=="__main__":
    main()
