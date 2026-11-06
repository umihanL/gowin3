"""Read a stable best-checkpoint snapshot and evaluate val on CPU during training."""
import csv
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".ultralytics"))
os.environ.setdefault("YOLO_AUTOINSTALL", "false")
os.environ.setdefault("OMP_NUM_THREADS", "2")
os.environ.setdefault("MKL_NUM_THREADS", "2")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=ROOT / 'runs/wallpaper_yolo11n_gpu_20261008')
    parser.add_argument('--data-root', type=Path, default=ROOT / 'dataset')
    parser.add_argument('--paused', action='store_true')
    options = parser.parse_args()
    import torch
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    from ultralytics import YOLO
    from train import save_metrics_summary
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import yaml

    run = options.run.resolve()
    data_root = options.data_root.resolve()
    rows = list(csv.DictReader((run / "results.csv").open(encoding="utf-8")))
    latest = int(rows[-1]["epoch"])
    output = ROOT / "runs/diagnostics" / f"{run.name}_through_epoch{latest}_{datetime.now():%Y%m%d_%H%M%S}"
    output.mkdir(parents=True)
    source = run / "weights/best.pt"
    before = source.stat()
    frozen = output / "best_snapshot.pt"
    shutil.copy2(source, frozen)
    after = source.stat()
    if (before.st_mtime_ns, before.st_size) != (after.st_mtime_ns, after.st_size):
        raise RuntimeError("Checkpoint changed during copy; retry with a stable snapshot")
    (output / "results_snapshot.csv").write_text((run / "results.csv").read_text(encoding="utf-8"), encoding="utf-8")
    args = yaml.safe_load((run / "args.yaml").read_text(encoding="utf-8"))
    model = YOLO(str(frozen), task="detect")
    checkpoint_epoch = model.ckpt.get("epoch", -1) + 1
    report = dict(latest_complete_epoch_at_snapshot=latest, evaluated_best_checkpoint_epoch=checkpoint_epoch,
                  checkpoint_sha256=hashlib.sha256(frozen.read_bytes()).hexdigest(),
                  run=str(run), dataset=str(data_root),
                  split="val", device="cpu", test_set_used=False, training_interrupted=options.paused)
    (output / "snapshot.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"SNAPSHOT_DIR {output}", flush=True)
    print(f"Evaluating best checkpoint from epoch {checkpoint_epoch} on val, CPU only", flush=True)
    result = model.val(data=args["data"], split="val", imgsz=640, batch=4,
                       device="cpu", workers=0, plots=True, verbose=True,
                       project=str(output), name="validation", exist_ok=False)
    save_metrics_summary(result, "val")
    epochs = [int(r["epoch"]) for r in rows]
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8), layout="constrained")
    for key, label in [("metrics/mAP50(B)", "mAP@0.50"), ("metrics/mAP50-95(B)", "mAP@0.50:0.95")]:
        ax[0].plot(epochs, [float(r[key]) for r in rows], label=label)
    for key, label in [("metrics/precision(B)", "Precision"), ("metrics/recall(B)", "Recall")]:
        ax[1].plot(epochs, [float(r[key]) for r in rows], label=label)
    for key, label in [("train/box_loss", "Train box loss"), ("val/box_loss", "Validation box loss")]:
        ax[2].plot(epochs, [float(r[key]) for r in rows], label=label)
    for panel in ax:
        panel.set_xlabel("Completed epoch")
        panel.grid(alpha=.2)
        panel.legend(fontsize=9)
    ax[0].set_ylim(0, 1)
    ax[1].set_ylim(0, 1)
    fig.suptitle(f"Training snapshot through epoch {latest} | synthetic validation only")
    fig.savefig(output / "learning_curves.png", dpi=150)
    plt.close(fig)
    # A fixed validation example containing all three classes and reasonable
    # target sizes, selected from labels before looking at model predictions.
    candidates = []
    for line in (data_root / "metadata.jsonl").read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        if row["split"] != "val" or row["eye"] != "L" or row["source_version"] != "v6":
            continue
        ann = json.loads((data_root / row["annotation"]).read_text(encoding="utf-8"))
        objects = ann["instances"]
        represented = {o["class_id"] for o in objects}
        if len(represented) == 3:
            score = sum(not o["tiny"] for o in objects)
            candidates.append((score, row["sample_id"], row["image"]))
    if candidates:
        _, sid, image = sorted(candidates, reverse=True)[0]
        prediction = model.predict(source=str(data_root / image), device="cpu", imgsz=640,
                                   conf=.25, verbose=False)[0]
        prediction.save(filename=str(output / "validation_example_prediction.jpg"))
        report["example_sample_id"] = sid
        report["example_confidence_threshold"] = .25
        report["example_selection"] = "V6 validation L with all 3 GT classes, most non-tiny labels; selected before predictions"
        (output / "snapshot.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"DIAGNOSTIC_COMPLETE {output}", flush=True)
    (ROOT / 'runs/diagnostic_current_path.txt').write_text(str(output), encoding='utf-8')


if __name__ == "__main__":
    main()
