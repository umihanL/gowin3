"""Portable three-class YOLO11n Detect training entry point (never pose/seg).

--check-only imports no torch/Ultralytics and performs no model download.
"""
from __future__ import annotations

import argparse
from collections.abc import Mapping
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
NAMES = {0: "fire", 1: "firefighter", 2: "person"}
MODES = ("train", "val", "test", "predict", "export-reference", "resume")
TRANSFORM_POLICY = "preserve-visible-tiny-v1"


def read_yaml(path):
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("Install requirements-check.txt in the project .venv first (no torch needed).") from exc
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    return value


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def names_dict(value):
    return dict(enumerate(value)) if isinstance(value, list) else {int(k): v for k, v in value.items()}


def resolve_dataset(data_path):
    """Our relative path contract is explicitly relative to the source YAML.

    Ultralytics receives only generated absolute paths, never this relative spec.
    Full label/image/group/hash validation belongs to validate_merged_dataset.py.
    """
    data_path = Path(data_path).resolve()
    spec = read_yaml(data_path)
    if names_dict(spec.get("names", {})) != NAMES:
        raise ValueError("Dataset must have exactly 0: fire, 1: firefighter, 2: person.")
    raw_root = Path(spec.get("path", "."))
    dataset = (raw_root if raw_root.is_absolute() else data_path.parent / raw_root).resolve()
    splits = {}
    for split in ("train", "val", "test"):
        value = spec.get(split)
        if not isinstance(value, str) or Path(value).suffix.lower() != ".txt":
            raise ValueError(f"{split} must name an explicit .txt image list")
        listing = Path(value)
        listing = listing if listing.is_absolute() else dataset / listing
        images = []
        for line in listing.read_text(encoding="utf-8-sig").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            image = Path(line)
            if not image.is_absolute():
                if not line.startswith("./"):
                    raise ValueError(f"Use ./ relative to its list or an absolute image path: {listing}: {line}")
                image = listing.parent / line[2:]
            image = image.resolve()
            if not image.is_file():
                raise FileNotFoundError(image)
            images.append(image)
        if not images or len(images) != len(set(images)):
            raise ValueError(f"Empty or duplicate image list: {listing}")
        splits[split] = images
    for a, b in (("train", "val"), ("train", "test"), ("val", "test")):
        if set(splits[a]) & set(splits[b]):
            raise ValueError(f"Image-path leakage between {a} and {b}")
    return dataset, splits


def full_validation(dataset):
    validator = ROOT / "tools/validate_merged_dataset.py"
    if not validator.is_file():
        raise FileNotFoundError(f"Full dataset validator is required: {validator}")
    subprocess.run([sys.executable, str(validator), "--dataset", str(dataset)],
                   check=True, cwd=str(ROOT))


def runtime_spec(data_path, splits, project, smoke=False):
    """Create deterministic absolute lists/YAML without touching the dataset."""
    selected = {k: v[:24 if k == "train" else 12] if smoke and k != "test" else v
                for k, v in splits.items()}
    content = {k: "".join(p.as_posix() + "\n" for p in v) for k, v in selected.items()}
    data_path = Path(data_path).resolve()
    manifest = data_path.parent / "manifest.json"
    input_fingerprint = {"original_yaml_sha256": file_hash(data_path),
                         "dataset_manifest_sha256": file_hash(manifest) if manifest.is_file() else None}
    identity = hashlib.sha256(json.dumps({"source": str(data_path),
                                           "lists": content, **input_fingerprint}, sort_keys=True).encode()).hexdigest()[:16]
    output = Path(project).resolve() / "_runtime" / identity
    output.mkdir(parents=True, exist_ok=True)
    spec = {"path": output.as_posix(), "names": NAMES, "nc": 3}
    for split, text in content.items():
        listing = output / f"{split}.txt"
        listing.write_text(text, encoding="utf-8")
        spec[split] = listing.as_posix()
    import yaml
    resolved = output / "data.resolved.yaml"
    resolved.write_text(yaml.safe_dump(spec, allow_unicode=True, sort_keys=False), encoding="utf-8")
    (output / "source.json").write_text(json.dumps({"original_data_yaml": str(Path(data_path).resolve()),
                                                    **input_fingerprint,
                                                    "smoke_only_not_accuracy_evaluation": smoke,
                                                    "counts": {k: len(v) for k, v in selected.items()}}, indent=2), encoding="utf-8")
    return resolved


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=MODES, default="train")
    p.add_argument("--data", type=Path, default=ROOT / "dataset/data.yaml")
    p.add_argument("--config", type=Path, default=ROOT / "config/yolo11n_detect.yaml")
    p.add_argument("--model", default="yolo11n.pt", help="Detect initialization: yolo11n.pt or a local Detect checkpoint")
    p.add_argument("--weights", type=Path, help="Trained checkpoint; resume requires last.pt from an interrupted run")
    p.add_argument("--source", type=Path, help="Local image/directory/video for predict")
    p.add_argument("--project", type=Path, default=ROOT / "runs")
    p.add_argument("--name", default="wallpaper_yolo11n")
    p.add_argument("--device", default="0", help="GPU index, or cpu")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=8)
    p.add_argument("--workers", type=int, default=0)
    p.add_argument("--patience", type=int, default=25)
    p.add_argument("--seed", type=int, default=20261008)
    p.add_argument("--conf", type=float, default=.25, help="Prediction confidence only; tune with val, never test")
    p.add_argument("--iou", type=float, default=.7)
    p.add_argument("--smoke", action="store_true", help="Train only: 1 epoch, first 24 train/12 val images, batch<=2, imgsz320")
    p.add_argument("--from-scratch", action="store_true", help="Train only: initialize yolo11n.yaml without pretrained-weight download")
    p.add_argument("--check-only", action="store_true", help="Full dataset/config checks; no torch import, model loading or runtime YAML writes")
    return p.parse_args(argv)


def prepare(args):
    if args.epochs < 1 or args.imgsz < 32 or args.imgsz % 32 or args.batch < 1 or args.workers < 0 or args.patience < 0:
        raise ValueError("epochs/batch positive; imgsz >=32 and divisible by32; workers/patience >=0")
    if not 0 <= args.conf <= 1 or not 0 < args.iou <= 1:
        raise ValueError("conf must be in [0,1]; iou must be in (0,1]")
    if any(t in str(value).lower() for value in (args.model, args.weights or "")
           for t in ("-pose", "-seg", "-cls", "-obb")):
        raise ValueError("Use YOLO11n Detect. Pose/seg/cls/obb labels and weights are unsupported.")
    if args.mode != "train" and args.weights is None:
        raise ValueError(f"--mode {args.mode} requires your trained --weights checkpoint")
    if args.weights is not None and not args.weights.is_file():
        raise FileNotFoundError(args.weights)
    if args.mode == "predict" and (args.source is None or not args.source.exists()):
        raise ValueError("Predict requires an existing local --source image, directory or video")
    if (args.smoke or args.from_scratch) and args.mode != "train":
        raise ValueError("--smoke/--from-scratch apply only to --mode train")
    if args.from_scratch and args.weights:
        raise ValueError("--from-scratch cannot load --weights")
    config = read_yaml(args.config)
    if config.get("task") != "detect":
        raise ValueError("Training config task must be detect")
    if "data" in config or "model" in config or "resume" in config:
        raise ValueError("Set data/model/resume through the explicit CLI, not the augmentation config")
    geometry = ("degrees", "translate", "scale", "shear", "perspective", "flipud",
                "fliplr", "mosaic", "mixup", "copy_paste", "cutmix", "close_mosaic")
    if any(config.get(key, 0) != 0 for key in geometry) or config.get("multi_scale", False):
        raise ValueError("Tiny-target training requires geometry augmentation disabled; use the provided config")
    config.update(epochs=args.epochs, imgsz=args.imgsz, batch=args.batch, workers=args.workers,
                  patience=args.patience, device=args.device, project=str(args.project.resolve()),
                  name=args.name, seed=args.seed)
    if args.smoke:
        config.update(epochs=1, imgsz=320, batch=min(args.batch, 2), save_period=-1,
                      name=args.name + "_smoke", plots=False, workers=0)
    if args.from_scratch:
        config["pretrained"] = False
    dataset, splits = resolve_dataset(args.data)
    if args.data.resolve() != (dataset / "data.yaml").resolve():
        raise ValueError("Use the canonical dataset/data.yaml so full validation checks the exact selected lists.")
    full_validation(dataset)
    return config, splits


def check_model(model, trained=False):
    if model.task != "detect":
        raise ValueError(f"Loaded model task is {model.task}, expected detect")
    if trained and names_dict(model.names) != NAMES:
        raise ValueError(f"Checkpoint class names differ from the three-class dataset: {model.names}")


def validate_device(torch_module, device):
    if device == "cpu":
        return
    try:
        indices = [int(value) for value in device.split(",")]
    except ValueError as exc:
        raise ValueError("Use --device cpu or a CUDA device index such as 0.") from exc
    if not torch_module.cuda.is_available():
        raise RuntimeError("CUDA training was selected but torch reports CUDA unavailable. Install a compatible CUDA torch build, or pass --device cpu.")
    count = torch_module.cuda.device_count()
    if not indices or any(index < 0 or index >= count for index in indices):
        raise ValueError(f"Requested CUDA index {device}; available device count is {count}.")


def training_trainer():
    # Lazy import keeps --check-only independent of torch and Ultralytics.
    from tiny_target_training import TinyTargetDetectionTrainer
    return TinyTargetDetectionTrainer


def verify_resume_policy(weights):
    policy_path = Path(weights).resolve().parent.parent / "augmentation_policy.json"
    if not policy_path.is_file():
        raise ValueError("Resume requires the run's augmentation_policy.json; cannot silently change a previous training transform")
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    if policy.get("policy_version") != TRANSFORM_POLICY or policy.get("trainer") != "TinyTargetDetectionTrainer":
        raise ValueError("Resume augmentation policy differs; start a new fine-tune instead")
    expected_hash = file_hash(ROOT / "tiny_target_training.py")
    if policy.get("source_sha256") != expected_hash:
        raise ValueError("Resume custom trainer source changed; restore the original source or start a new fine-tune")


def save_metrics_summary(result, split):
    """Persist the real DetMetrics values returned by Ultralytics validation."""
    if not isinstance(result.results_dict, Mapping):
        raise RuntimeError("Validation did not return a DetMetrics results_dict")
    box = result.box
    class_ids = [int(i) for i in box.ap_class_index]
    evaluated = {}
    names = names_dict(result.names)
    for position, class_id in enumerate(class_ids):
        evaluated[class_id] = {"class_id": class_id, "class_name": names[class_id],
                               "precision": float(box.p[position]), "recall": float(box.r[position]),
                               "AP50": float(box.ap50[position]), "AP50_95": float(box.ap[position])}
    summary = {"split": split, "aggregate": {str(k): float(v) for k, v in result.results_dict.items()},
               "per_class": [evaluated.get(i, {"class_id": i, "class_name": name,
                                               "evaluated": False, "reason": "No AP entry returned for this class"})
                             for i, name in sorted(names.items())],
               "source": "Actual Ultralytics DetMetrics returned by model.val; not estimated or copied from training plots"}
    output = Path(result.save_dir) / "metrics_summary.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(f"Validation metrics saved: {output}")
    return output


def execute(args, config, resolved, yolo_factory):
    initialization = "yolo11n.yaml" if args.from_scratch else args.model
    if initialization == "yolo11n.pt" and args.weights is None:
        pretrained = ROOT / "weights/yolo11n.pt"
        pretrained.parent.mkdir(parents=True, exist_ok=True)
        initialization = str(pretrained)
    model = yolo_factory(str(args.weights.resolve()) if args.weights else initialization, task="detect")
    check_model(model, trained=args.mode != "train")
    common = dict(imgsz=config["imgsz"], batch=config["batch"], device=args.device,
                  workers=args.workers, project=str(args.project.resolve()), seed=args.seed)
    if args.mode == "train":
        return model.train(trainer=training_trainer(), **config, data=str(resolved))
    if args.mode == "resume":
        # Resume restores optimizer, scaler, epoch, batch, imgsz and augmentation
        # from last.pt. Passing a fresh training config is NOT true resume.
        ckpt = getattr(model, "ckpt", {}) or {}
        if ckpt.get("epoch", -1) < 0 or ckpt.get("optimizer") is None:
            raise ValueError("Checkpoint has no resumable optimizer/epoch; use an interrupted run's last.pt. For a new fine-tune use --mode train --model <checkpoint>.")
        verify_resume_policy(args.weights)
        previous_data = Path(ckpt.get("train_args", {}).get("data", ""))
        if not previous_data.is_file():
            raise ValueError("Checkpoint's resolved data YAML is missing (possibly a moved training run). Restore that run path for exact resume, or start a new fine-tune with --mode train --model <checkpoint>.")
        previous_dataset, previous_splits = resolve_dataset(previous_data)
        _, current_splits = resolve_dataset(resolved)
        if previous_splits != current_splits:
            raise ValueError("Resume dataset differs from the checkpoint; start a new run instead of resuming.")
        previous_source = previous_data.parent / "source.json"
        current_source = Path(resolved).parent / "source.json"
        if previous_source.is_file() and current_source.is_file():
            previous_stamp = json.loads(previous_source.read_text(encoding="utf-8"))
            current_stamp = json.loads(current_source.read_text(encoding="utf-8"))
            for key in ("original_yaml_sha256", "dataset_manifest_sha256", "smoke_only_not_accuracy_evaluation"):
                if previous_stamp.get(key) != current_stamp.get(key):
                    raise ValueError(f"Resume dataset fingerprint differs: {key}")
        return model.train(trainer=training_trainer(), resume=True, device=args.device, workers=args.workers)
    if args.mode in ("val", "test"):
        result = model.val(data=str(resolved), split=args.mode, plots=True,
                           name=f"{args.name}_{args.mode}", **common)
        save_metrics_summary(result, args.mode)
        return result
    if args.mode == "predict":
        return model.predict(source=str(args.source.resolve()), imgsz=args.imgsz,
                             device=args.device, conf=args.conf, iou=args.iou,
                             project=str(args.project.resolve()), name=f"{args.name}_predict",
                             save=True, save_txt=True, save_conf=True, stream=True)
    # Ultralytics exports beside its checkpoint: use a dedicated copy so users'
    # original run directories remain untouched by this optional export.
    output = ROOT / "weights/export_reference"
    output.mkdir(parents=True, exist_ok=True)
    export_weights = output / (args.weights.stem + "_" + file_hash(args.weights)[:12] + ".pt")
    if not export_weights.exists():
        shutil.copy2(args.weights, export_weights)
    elif file_hash(export_weights) != file_hash(args.weights):
        raise RuntimeError(f"Existing export checkpoint hash mismatch: {export_weights}")
    export_model = yolo_factory(str(export_weights), task="detect")
    check_model(export_model, trained=True)
    return export_model.export(format="onnx", imgsz=args.imgsz, batch=1, dynamic=False,
                               simplify=False, opset=11, nms=False, device=args.device)


def main(argv=None):
    args = arguments(argv)
    config, splits = prepare(args)
    print(json.dumps({"mode": args.mode, "classes": NAMES,
                      "dataset_images": {k: len(v) for k, v in splits.items()},
                      "training_config": config,
                      "transform_policy": TRANSFORM_POLICY,
                      "resume_note": "Resume restores saved training settings; --epochs/batch/imgsz/config do not replace them."},
                     ensure_ascii=False, indent=2))
    if args.check_only:
        print("CHECK PASSED: no torch import, model download, training, export or runtime-data rewrite.")
        return
    os.environ.setdefault("YOLO_AUTOINSTALL", "false")
    try:
        import torch
        validate_device(torch, args.device)
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("Training environment is absent. Run 01_setup_environment.cmd first.") from exc
    resolved = runtime_spec(args.data, splits, args.project, args.smoke)
    result = execute(args, config, resolved, YOLO)
    if args.mode == "predict":
        for _ in result:
            pass  # consume lazy inference so images/labels are actually saved
    if args.mode == "export-reference":
        print(f"Reference ONNX: {result}. This is NOT an RDK X5 BPU-ready model.")
    else:
        print("Finished. Synthetic validation/test scores must be reported separately from held-out real-camera scores.")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(2)
