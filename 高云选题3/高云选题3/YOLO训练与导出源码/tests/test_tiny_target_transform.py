"""Real Ultralytics CPU transform tests: no model, GPU, network or training."""
from __future__ import annotations
import copy
import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@unittest.skipUnless(importlib.util.find_spec("ultralytics"), "Install the pinned training environment for real transform tests")
class TinyTargetTransform(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import numpy as np
        from ultralytics.cfg import get_cfg
        from ultralytics.utils.instance import Instances
        from tiny_target_training import TinyTargetDataset
        from train import read_yaml, ROOT
        cls.np, cls.Instances = np, Instances
        cls.hyp = get_cfg(overrides=read_yaml(ROOT / "config/yolo11n_detect.yaml"))
        cls.dataset = TinyTargetDataset.__new__(TinyTargetDataset)
        cls.dataset.imgsz, cls.dataset.augment = 640, True
        cls.dataset.use_segments = cls.dataset.use_keypoints = cls.dataset.use_obb = False

    def sample(self, empty=False):
        np = self.np
        boxes = np.array([[0, 0, 1, 1], [638, 478, 640, 480], [100, 200, 103, 201],
                          [300, 220, 301, 340], [100, 100, 160, 180]], dtype=np.float32)
        if empty:
            boxes = np.zeros((0, 4), dtype=np.float32)
        return {"img": np.full((480, 640, 3), 128, dtype=np.uint8),
                "cls": np.arange(len(boxes), dtype=np.float32).reshape(-1, 1) % 3,
                "instances": self.Instances(boxes.copy(), np.zeros((0, 1000, 2), dtype=np.float32),
                                             bbox_format="xyxy", normalized=False)}, boxes

    def test_one_pixel_and_extreme_aspect_boxes_survive_letterbox_hsv(self):
        np = self.np
        transforms = self.dataset.build_transforms(self.hyp)
        self.assertEqual([type(t).__name__ for t in transforms.transforms], ["LetterBox", "RandomHSV", "Format"])
        for seed in range(5):
            np.random.seed(seed)
            labels, boxes = self.sample()
            out = transforms(labels)
            self.assertEqual(tuple(out["img"].shape), (3, 640, 640))
            self.assertEqual(len(out["cls"]), 5)
            expected = np.column_stack(((boxes[:, 0] + boxes[:, 2]) / 2,
                                         (boxes[:, 1] + boxes[:, 3]) / 2 + 80,
                                         boxes[:, 2] - boxes[:, 0], boxes[:, 3] - boxes[:, 1])) / 640
            np.testing.assert_allclose(out["bboxes"].numpy(), expected, atol=1e-7)

    def test_empty_negatives_survive(self):
        labels, _ = self.sample(empty=True)
        output = self.dataset.build_transforms(self.hyp)(labels)
        self.assertEqual(len(output["cls"]), 0)
        self.assertEqual(tuple(output["bboxes"].shape), (0, 4))

    def test_close_mosaic_keeps_custom_policy(self):
        self.dataset.close_mosaic(copy.copy(self.hyp))
        self.assertEqual([type(t).__name__ for t in self.dataset.transforms.transforms], ["LetterBox", "RandomHSV", "Format"])
        labels, _ = self.sample()
        self.assertEqual(len(self.dataset.transforms(labels)["cls"]), 5)

    def test_geometry_request_is_rejected(self):
        changed = copy.copy(self.hyp)
        changed.translate = .03
        with self.assertRaisesRegex(ValueError, "geometry"):
            self.dataset.build_transforms(changed)


if __name__ == "__main__":
    unittest.main()
