"""Ultralytics 8.3.253 public Dataset/Trainer overrides preserving tiny boxes.

No library-global monkeypatches. Geometry is the standard aspect-preserving
LetterBox only; HSV changes pixels but not boxes. Original labels remain intact.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import ultralytics
from ultralytics.data.augment import Compose, Format, LetterBox, RandomHSV
from ultralytics.data.dataset import YOLODataset
from ultralytics.models.yolo.detect.train import DetectionTrainer
from ultralytics.utils import colorstr
from ultralytics.utils.torch_utils import unwrap_model

POLICY_VERSION = "preserve-visible-tiny-v1"
SUPPORTED_ULTRALYTICS = "8.3.253"
GEOMETRY_OFF = ("degrees", "translate", "scale", "shear", "perspective", "flipud",
                "fliplr", "mosaic", "mixup", "copy_paste", "cutmix", "close_mosaic")


def assert_supported_policy(hyp):
    if ultralytics.__version__ != SUPPORTED_ULTRALYTICS:
        raise RuntimeError(f"Tiny-target trainer is checked against ultralytics=={SUPPORTED_ULTRALYTICS}; got {ultralytics.__version__}. Use requirements-train.txt.")
    if getattr(hyp, "task", "detect") != "detect":
        raise ValueError("Tiny-target preservation supports Detect only")
    changed = {key: getattr(hyp, key, 0) for key in GEOMETRY_OFF if getattr(hyp, key, 0) != 0}
    if changed or getattr(hyp, "multi_scale", False):
        raise ValueError(f"Tiny-target preservation requires geometry/multi_scale disabled: {changed}")
    if getattr(hyp, "augmentations", None):
        raise ValueError("Custom Albumentations are not part of the audited tiny-target transform")
    if getattr(hyp, "classes", None) is not None or getattr(hyp, "single_cls", False):
        raise ValueError("Do not filter/merge classes in the three-class tiny-target dataset")


class CountPreservingCompose(Compose):
    """Assert the preprocessing never silently drops still-visible targets."""

    def __call__(self, labels):
        count = len(labels["cls"])
        result = super().__call__(labels)
        if len(result["cls"]) != count or len(result["bboxes"]) != count:
            raise RuntimeError("Tiny-target transform changed the number of annotations")
        boxes = result["bboxes"].numpy()
        if count and (not np.isfinite(boxes).all() or not (boxes[:, 2:] > 0).all()):
            raise RuntimeError("Tiny-target transform produced a non-finite or non-positive box")
        return result


class TinyTargetDataset(YOLODataset):
    def build_transforms(self, hyp=None):
        assert_supported_policy(hyp)
        if self.use_segments or self.use_keypoints or self.use_obb:
            raise ValueError("Expected detection boxes, without segments/keypoints/OBB")
        steps = [LetterBox(new_shape=(self.imgsz, self.imgsz), scaleup=self.augment)]
        if self.augment:
            steps.append(RandomHSV(hgain=hyp.hsv_h, sgain=hyp.hsv_s, vgain=hyp.hsv_v))
        steps.append(Format(bbox_format="xywh", normalize=True, return_mask=False,
                            return_keypoint=False, return_obb=False, batch_idx=True,
                            bgr=hyp.bgr if self.augment else 0.0))
        # Inherited close_mosaic() calls this override again, so resume cannot
        # accidentally restore upstream RandomPerspective's >2px filter.
        return CountPreservingCompose(steps)


def make_dataset(args, img_path, batch, data, mode="train", stride=32):
    """Same constructor fields as 8.3.253 build_yolo_dataset, custom subclass."""
    assert_supported_policy(args)
    return TinyTargetDataset(
        img_path=img_path, imgsz=args.imgsz, batch_size=batch,
        augment=mode == "train", hyp=args, rect=args.rect or mode == "val",
        cache=args.cache or None, single_cls=False, stride=stride,
        pad=0.0 if mode == "train" else 0.5, prefix=colorstr(f"{mode}: "),
        task="detect", classes=None, data=data,
        fraction=args.fraction if mode == "train" else 1.0,
    )


class TinyTargetDetectionTrainer(DetectionTrainer):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        assert_supported_policy(self.args)
        policy = {
            "policy_version": POLICY_VERSION,
            "trainer": type(self).__name__,
            "ultralytics_version": ultralytics.__version__,
            "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "train_transform": ["LetterBox", "RandomHSV", "Format", "target_count_assertion"],
            "validation_transform": ["LetterBox", "Format", "target_count_assertion"],
            "geometry_augmentation": {key: getattr(self.args, key, 0) for key in GEOMETRY_OFF},
            "label_policy": "No minimum-pixel-size, aspect-ratio or visible-area box filtering. Every original positive box must survive LetterBox/HSV/Format.",
            "evaluation_limit": "Keeping labels does not make 1-2 pixel objects reliably detectable.",
        }
        (self.save_dir / "augmentation_policy.json").write_text(
            json.dumps(policy, ensure_ascii=False, indent=2), encoding="utf-8")

    def build_dataset(self, img_path, mode="train", batch=None):
        stride = max(int(unwrap_model(self.model).stride.max() if self.model else 0), 32)
        return make_dataset(self.args, img_path, batch, self.data, mode, stride)
