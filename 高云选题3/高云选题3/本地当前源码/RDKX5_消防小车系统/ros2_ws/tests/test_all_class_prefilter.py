"""All-class background pruning must preserve the deployed sigmoid semantics."""
import unittest

import numpy as np

from test_legacy_detector_candidates import (
    heads, previous_candidates, previous_detections,
)
from rescue_vision.legacy_detector import _decode_candidates, detections


class AllClassPrefilterTests(unittest.TestCase):
    meta = dict(gain=.5, left=0, top=48, width=1280, height=1088)

    def assert_exact(self, outputs, conf, selected=None):
        # Saturated float16/non-finite logits are deliberate regression inputs.
        with np.errstate(over='ignore', invalid='ignore'):
            expected = previous_candidates(outputs, conf, selected)
            actual = _decode_candidates(outputs, conf, selected)
        for candidate, reference in zip(actual, expected):
            self.assertEqual(candidate.dtype, reference.dtype)
            self.assertEqual(candidate.shape, reference.shape)
            self.assertEqual(candidate.tobytes(), reference.tobytes())

    def test_empty_background_all_classes_and_filters(self):
        for dtype in (np.float16, np.float32, np.float64):
            outputs = heads(81, count=0, dtype=dtype)
            for selected in (None, (), (0, 1), (0, 2), (1, 2), (0, 1, 2)):
                with self.subTest(dtype=dtype, selected=selected):
                    self.assert_exact(outputs, .25, selected)

    def test_boundaries_nonfinite_values_and_saturated_ties(self):
        for dtype in (np.float16, np.float32, np.float64):
            outputs = heads(83, count=0, dtype=dtype)
            threshold = dtype(np.log(.25 / .75))
            below = np.nextafter(threshold, dtype(-np.inf))
            above = np.nextafter(threshold, dtype(np.inf))
            values = np.asarray([
                [-2, -2, -2], [-80, below, -80], [-80, threshold, -80],
                [-80, above, -80], [0, 0, 0], [50, 70, 60],
                [16.6, 16.7, 16.8], [-np.inf, -np.inf, -np.inf],
                [np.inf, 80, 90], [np.nan, 0, 0], [0, np.nan, 0],
                [0, 0, np.nan], [np.nan, np.nan, np.nan],
                [-80, 0, -80], [-80, -80, 0],
            ], dtype=dtype)
            for head in outputs[::2]:
                head.reshape(-1, 3)[110:110 + len(values)] = values
            with np.errstate(over='ignore', invalid='ignore'):
                for conf in (.12, np.nextafter(.25, 0), .25,
                             np.nextafter(.25, 1), .5, .999999):
                    for selected in (None, (), (0, 1), (0, 2), (1, 2), (0, 1, 2)):
                        with self.subTest(dtype=dtype, conf=conf, selected=selected):
                            self.assert_exact(outputs, conf, selected)

    def test_dense_random_and_mixed_precision_heads_are_byte_exact(self):
        rng = np.random.default_rng(1783)
        for seed in range(4):
            outputs = heads(seed, count=0)
            for index in (0, 2, 4):
                dtype = np.float64 if seed == 3 and index == 4 else np.float32
                outputs[index] = rng.uniform(-90, 90, outputs[index].shape).astype(dtype)
            for conf in (.25, .7, .9999):
                for selected in (None, (0, 2), (1, 2)):
                    with self.subTest(seed=seed, conf=conf, selected=selected):
                        self.assert_exact(outputs, conf, selected)

    def test_final_detection_boxes_scores_order_and_max_det_remain_equal(self):
        for seed in range(4):
            outputs = heads(seed, count=45)
            # A saturated first-class tie must not be replaced by raw argmax.
            outputs[0].reshape(-1, 3)[320] = [50, 70, 60]
            for conf, iou, max_det in ((.25, .7, 300), (.5, .3, 12), (.8, 1., 300)):
                with self.subTest(seed=seed, conf=conf, iou=iou, max_det=max_det):
                    self.assertEqual(detections(outputs, self.meta, conf, iou, max_det),
                                     previous_detections(outputs, self.meta, conf, iou, max_det))


if __name__ == '__main__':
    unittest.main()
