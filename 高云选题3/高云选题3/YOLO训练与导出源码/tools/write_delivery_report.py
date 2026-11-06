"""Summarize existing audits without rerendering data or training a model."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(relative):
    return json.loads((ROOT / relative).read_text(encoding="utf-8-sig"))


def sha(relative):
    return hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()


def main():
    files = {
        "render": "source_data/v6_ground_floor/render_audit.json",
        "dataset": "dataset/independent_validation.json",
        "review": "dataset/review_ui_validation.json",
        "code": "training_code_validation.json",
        "paths": "path_integration_validation.json",
    }
    audits = {key: read(path) for key, path in files.items()}
    if not (audits["render"].get("all_pass") and audits["render"].get("complete")
            and all(audits[key].get("passed") for key in ("dataset", "review", "paths"))
            and audits["code"].get("status") == "passed"):
        raise RuntimeError("All required audit reports must pass before delivery.")
    manifest = read("dataset/manifest.json")
    if audits["dataset"]["manifest_sha256"] != sha("dataset/manifest.json"):
        raise RuntimeError("Dataset manifest changed after audit.")
    for path, expected in audits["code"]["files"].items():
        if sha(path) != expected:
            raise RuntimeError(f"Training file changed after code checks: {path}")
    stats = read("dataset/statistics.json")
    counts = {split: stats[split]["images"] for split in ("train", "val", "test")}
    init = read("weights/PRETRAINED_SOURCE.json")
    if sha("weights/yolo11n.pt") != init["sha256"]:
        raise RuntimeError("Initialization checkpoint hash mismatch.")
    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "status": "data_and_training_code_ready",
        "model": "YOLO11n Detect",
        "classes": manifest["classes"],
        "counts": counts,
        "totals": manifest["totals"],
        "added_stereo_pairs": audits["render"]["expected_pairs"],
        "added_packaged_images": sum(stats[s]["by_source"].get("v6", {}).get("images", 0) for s in stats),
        "audits": {k: {"file": v, "sha256": sha(v)} for k, v in files.items()},
        "dataset_manifest_sha256": sha("dataset/manifest.json"),
        "initialization_checkpoint": init,
        "training_environment_installed_this_task": False,
        "actual_training_performed": False,
        "actual_model_metrics_available": False,
        "rdk_x5_bpu_bin_available": False,
        "validation_boundary": "Dataset integrity and offline review passed; training code uses API stubs plus actual file/config resolution. No torch model load, training, model evaluation or GS130W real-camera test.",
    }
    (ROOT / "delivery_validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    text = f"""# 交付检查记录

已交付 YOLO11n Detect 三类训练工程：fire、firefighter、person。楼栋号/楼名不加入 YOLO 类别。

最终 {manifest['totals']['images']:,} 张图片、{manifest['totals']['boxes']:,} 个标注框。训练/验证/测试为 {counts['train']:,}/{counts['val']:,}/{counts['test']:,} 张。新增地图原始数据 240 对双目，含增强的新版本数据 816 张。数据包含左右眼及训练增强，并非同等数量的独立场景。

| 检查项 | 结果 | 记录 |
| --- | --- | --- |
| 新地图 RGB、mask、相机与来源 | 240 对完整，通过 | [render_audit.json](source_data/v6_ground_floor/render_audit.json) |
| 融合图片/标签/掩膜、来源、划分与左右目 | 全量通过 | [independent_validation.json](dataset/independent_validation.json) |
| 原 V1 保持、同视角跨版本分组、极小框保留 | 包含在独立审计中 | [数据集说明](数据集说明.md) |
| 离线浏览器筛选、左右目、桌面与手机布局 | 通过 | [review_ui_validation.json](dataset/review_ui_validation.json) |
| 训练代码、参数分流、续训与设备保护 | {audits['code']['tests_run']} 项回归通过，使用 API stub | [training_code_validation.json](training_code_validation.json) |
| 中文路径、不同工作目录的数据配置解析 | 实际文件检查通过 | [path_integration_validation.json](path_integration_validation.json) |

默认训练入口为 [train.py](train.py)，数据入口为 [dataset/data.yaml](dataset/data.yaml)。[README](README.md) 提供启动顺序，[TRAINING_GUIDE](TRAINING_GUIDE.md) 提供安装和运行命令。[离线标注预览](dataset/review.html) 中的框是标注真值。

`weights/yolo11n.pt` 是从本机已有文件复制的 COCO 初始化权重，SHA-256 已核对；远端发行来源未重新核验，也未用 PyTorch 加载。详细来源见 [PRETRAINED_SOURCE.json](weights/PRETRAINED_SOURCE.json)。

本次未安装 torch/Ultralytics、未执行实际训练或模型评估，因此没有本项目训练完成的 best.pt、实测 mAP 或 RDK X5 BPU .bin。代码检查不代表训练精度。安装训练环境后可先执行 03 小样本测试，再执行 04 正式训练。

合成数据使用当前壁纸图案和暂定 GS130W 成像参数；实际摄像头仍需采集、标定并留独立实拍测试集。大量远处人物是极小目标，需结合云台对准和靠近重拍评估可识别距离。

机器可读检查摘要及文件指纹见 [delivery_validation.json](delivery_validation.json)。
"""
    (ROOT / "交付检查记录.md").write_text(text, encoding="utf-8")
    print("Delivery records written; data/code ready, actual training not performed.")


if __name__ == "__main__":
    main()
