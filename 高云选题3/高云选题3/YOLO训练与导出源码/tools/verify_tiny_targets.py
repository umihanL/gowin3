"""Read every actual train image on CPU and verify custom transform target counts.

Uses the pinned Ultralytics loader, without constructing a model or training.
Ultralytics may create its normal labels.cache; source images/labels are read-only.
"""
from __future__ import annotations
import argparse
import collections
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", type=Path, default=ROOT / "dataset/data.yaml")
    p.add_argument("--output", type=Path, default=ROOT / "tiny_target_loader_validation.json")
    p.add_argument("--limit", type=int, default=0, help="0 checks all train images")
    args = p.parse_args()
    import numpy as np
    import torch
    import ultralytics
    from ultralytics.cfg import get_cfg
    from train import read_yaml, resolve_dataset, runtime_spec, file_hash
    from tiny_target_training import make_dataset, POLICY_VERSION

    # CPU-only preprocessing can be slow with many implicit tensor thread pools.
    torch.set_num_threads(1)
    _, splits = resolve_dataset(args.data)
    resolved = runtime_spec(args.data, splits, ROOT / "runs")
    data = read_yaml(resolved)
    hyp = get_cfg(overrides={**read_yaml(ROOT / "config/yolo11n_detect.yaml"),
                             "imgsz": 640, "batch": 8, "workers": 0, "device": "cpu"})
    dataset = make_dataset(hyp, data["train"], 8, data, "train")
    total = min(args.limit, len(dataset)) if args.limit else len(dataset)
    counts = collections.Counter()
    started = time.monotonic()
    for index in range(total):
        source = Path(dataset.im_files[index])
        parts = list(source.parts)
        parts[max(i for i, part in enumerate(parts) if part == "images")] = "labels"
        label_path = Path(*parts).with_suffix(".txt")
        rows = [list(map(float, row.split())) for row in label_path.read_text(encoding="utf-8").splitlines() if row.strip()]
        labels = np.array(rows, dtype=np.float32).reshape(-1, 5)
        output = dataset[index]
        assert len(output["cls"]) == len(rows), f"Lost target labels: {source}"
        np.testing.assert_array_equal(output["cls"].numpy().reshape(-1), labels[:, 0])
        # Native size 640x480 -> LetterBox640 adds80px vertically, no rescale.
        width, height = output["ori_shape"][1], output["ori_shape"][0]
        assert (width, height) == (640, 480), f"Unexpected native source size: {source}"
        expected = labels[:, 1:].copy()
        expected[:, 1] = (expected[:, 1] * 480 + 80) / 640
        expected[:, 3] *= 480 / 640
        np.testing.assert_allclose(output["bboxes"].numpy(), expected, atol=2e-7,
                                   err_msg=f"Box coordinates changed unexpectedly: {source}")
        if len(rows):
            before_tiny = int(np.sum(np.minimum(labels[:, 3] * 640, labels[:, 4] * 480) <= 2.0001))
            after_tiny = int(np.sum(np.min(output["bboxes"].numpy()[:, 2:] * 640, axis=1) <= 2.0001))
            assert before_tiny == after_tiny, f"Tiny labels changed: {source}"
            counts["tiny_boxes_before"] += before_tiny
            counts["tiny_boxes_after"] += after_tiny
        counts["images"] += 1
        counts["boxes_before"] += len(rows)
        counts["boxes_after"] += len(output["cls"])
        counts["negative_images"] += int(not rows)
        if (index + 1) % 250 == 0:
            print(f"CPU_TRANSFORM_CHECK {index+1}/{total}", flush=True)
    report = {"passed": True, "policy_version": POLICY_VERSION, "ultralytics": ultralytics.__version__,
              "torch": torch.__version__, "device": "CPU preprocessing only", "model_loaded": False,
              "training_performed": False, "all_train_images_checked": total == len(dataset),
              "counts": dict(counts), "seconds": time.monotonic() - started,
              "data_yaml_sha256": file_hash(args.data), "trainer_source_sha256": file_hash(ROOT / "tiny_target_training.py"),
              "transforms": [type(t).__name__ for t in dataset.transforms.transforms],
              "boundary": "Verifies data loading, label count and coordinates. It does not establish small-object detection accuracy."}
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
