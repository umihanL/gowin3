"""Summarize completed people expansion audits and an actual restarted GPU run."""
from pathlib import Path
import csv
import hashlib
import json
import yaml

ROOT = Path(__file__).resolve().parents[1]
RUN = "wallpaper_yolo11n_people_v2_20261008"


def read(path):
    return json.loads((ROOT / path).read_text(encoding="utf-8-sig"))


def sha(path):
    return hashlib.sha256((ROOT / path).read_bytes()).hexdigest()


def main():
    reports = {"source": "source_data/v7_people_focus/render_audit.json",
               "merged": "dataset_people_v2/independent_validation.json",
               "ui": "dataset_people_v2/review_ui_validation.json",
               "loader": "people_v2_loader_validation.json"}
    audits = {key: read(path) for key, path in reports.items()}
    if not (audits["source"]["all_pass"] and audits["source"]["complete"]
            and all(audits[k]["passed"] for k in ("merged", "ui", "loader"))):
        raise RuntimeError("Expansion audits must pass first")
    manifest = read("dataset_people_v2/manifest.json")
    if sha("dataset_people_v2/manifest.json") != audits["merged"]["manifest_sha256"]:
        raise RuntimeError("Merged manifest differs from audited version")
    if sha("dataset/manifest.json") != manifest["baseline_manifest_sha256"]:
        raise RuntimeError("Baseline manifest changed")
    state = read(f"runs/_jobs/{RUN}/status.json")
    if state["phase"] not in ("training", "evaluating", "completed"):
        raise RuntimeError("New training has not entered a confirmed running state")
    rows = list(csv.DictReader((ROOT / f"runs/{RUN}/results.csv").open(encoding="utf-8")))
    if not rows or not all((ROOT / f"runs/{RUN}/weights/{name}.pt").is_file() for name in ("best", "last")):
        raise RuntimeError("Wait for the first completed epoch and checkpoint")
    stats = read("dataset_people_v2/statistics.json")
    added = read("source_data/v7_people_focus/packaged/statistics.json")
    stopped = read("runs/wallpaper_yolo11n_gpu_20261008/stopped_for_data_expansion/snapshot.json")
    model_rel = "runs/wallpaper_yolo11n_gpu_20261008/stopped_for_data_expansion/best.pt"
    if sha(model_rel) != stopped["files"]["best.pt"]["sha256"]:
        raise RuntimeError("Preserved initialization changed")
    actual_args = yaml.safe_load((ROOT / f"runs/{RUN}/args.yaml").read_text(encoding="utf-8"))
    if (Path(state["dataset"]).resolve() != (ROOT / "dataset_people_v2/data.yaml").resolve()
            or Path(state["initialization_model"]).resolve() != (ROOT / model_rel).resolve()
            or Path(actual_args["model"]).resolve() != (ROOT / model_rel).resolve()
            or actual_args.get("resume")
            or actual_args.get("optimizer") != "AdamW"
            or actual_args.get("lr0") != 0.0003
            or str(actual_args.get("device")) not in ("0", "[0]")):
        raise RuntimeError("Actual training does not match the intended GPU fine-tune")
    report = {"status": "expanded_and_gpu_finetune_running", "run": RUN,
              "baseline_completed_epochs": stopped["last_complete_epoch"],
              "initialization_model": model_rel, "initialization_sha256": sha(model_rel),
              "new_optimizer": True, "resume_old_optimizer": False,
              "dataset": "dataset_people_v2/data.yaml", "baseline_dataset": "dataset/data.yaml",
              "added_stereo_pairs": audits["source"]["expected_pairs"],
              "added_images": added["train"]["images"],
              "images": {s: stats[s]["images"] for s in ("train", "val", "test")},
              "class_instances": {s: stats[s]["class_instances"] for s in stats},
              "totals": manifest["totals"], "frozen_evaluation": manifest["frozen_evaluation"],
              "first_epoch": rows[0], "completed_epochs_at_report": len(rows),
              "actual_train_args": {"file": f"runs/{RUN}/args.yaml", "sha256": sha(f"runs/{RUN}/args.yaml")},
              "audits": {k: {"file": p, "sha256": sha(p)} for k, p in reports.items()},
              "source_code": {p: sha(p) for p in ["train.py", "tiny_target_training.py", "config/yolo11n_people_finetune.yaml", "tools/run_training_job.py"]},
              "final_accuracy_available": False,
              "note": "Live run continues. Rendering/label checks and first-epoch success do not prove improved held-out accuracy; compare the unchanged val split after training."}
    (ROOT / "people_expansion_validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = ["# 人物数据补充与再训练检查记录", "",
             f"旧训练已按要求停止在完整第 {stopped['last_complete_epoch']} 轮，旧最佳权重和检查点独立保存。", "",
             f"新增 {report['added_stereo_pairs']} 对双目原图，含训练增强共 {report['added_images']:,} 张新图片。", "",
             "| 划分 | 融合后图片 |", "| --- | ---: |"]
    lines += [f"| {s} | {report['images'][s]:,} |" for s in ("train", "val", "test")]
    lines += ["", f"总计 {manifest['totals']['images']:,} 张、{manifest['totals']['boxes']:,} 个标注框。旧验证、测试集清单及内容完全保留。",
              "", "源渲染审计、融合数据独立审计、离线预览检查、全部训练图实际加载检查均通过。",
              "", f"新 GPU 任务 `{RUN}` 已完成至少一轮训练与验证，并写出 best.pt / last.pt；从旧最佳权重开始新的优化器与轮次，不是恢复旧任务。",
              "", "新配置：AdamW，lr0=0.0003，warmup_bias_lr=0，640，batch 8，最多80轮，patience20。",
              "", "[新数据标注预览](dataset_people_v2/review.html) · [训练实时状态](TRAINING_STATUS.md) · [完整检查摘要](people_expansion_validation.json)",
              "", "完整训练仍在进行，是否改善漏检需要比较冻结验证集，不能由新增图片数量推断。最终测试集将在训练结束后评估。"]
    (ROOT / "人物补充检查记录.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"images": report["images"], "added_images": report["added_images"], "status": report["status"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
