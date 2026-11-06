"""Compare sparse DFL decoding with the previous full-dense detection pipeline."""
import sys
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.legacy_detector import (
    BoardDetector, CLASSES, _DFL_BINS, _decode_candidates, _validated_heads,
    decode_dense, detections, nms,
)


def previous_candidates(outputs, conf, class_ids):
    """Frozen pre-prefilter sparse decoder, including native-dtype comparisons."""
    parsed_heads = _validated_heads(outputs)
    dtype = np.result_type(np.float32, *(head[0].dtype for head in parsed_heads))
    all_boxes, all_scores, all_ids = [], [], []
    for cls, box, stride, side in parsed_heads:
        probabilities = 1.0 / (1.0 + np.exp(-np.clip(cls.reshape(-1, 3), -80, 80)))
        ids = probabilities.argmax(axis=1)
        scores = probabilities[np.arange(len(ids)), ids]
        eligible = scores >= conf
        if class_ids is not None:
            eligible &= np.isin(ids, class_ids)
        chosen = np.flatnonzero(eligible)
        if not len(chosen):
            continue
        logits = box.reshape(-1, 4, 16)[chosen].astype(np.float32)
        exp = np.exp(logits - logits.max(axis=-1, keepdims=True))
        distance = (exp / exp.sum(axis=-1, keepdims=True) * _DFL_BINS).sum(axis=-1)
        anchors = np.stack((chosen % side + .5, chosen // side + .5), axis=1).astype(np.float32)
        xyxy = np.concatenate((anchors - distance[:, :2], anchors + distance[:, 2:]), axis=1) * stride
        all_boxes.append(xyxy.astype(dtype, copy=False))
        all_scores.append(scores[chosen].astype(dtype, copy=False))
        all_ids.append(ids[chosen])
    if not all_boxes:
        return np.empty((0, 4), dtype=dtype), np.empty(0, dtype=dtype), np.empty(0, dtype=np.int64)
    return np.concatenate(all_boxes), np.concatenate(all_scores), np.concatenate(all_ids)


def previous_detections(outputs, meta, conf=.25, iou=.7, max_det=300):
    """Frozen old full-dense path: reference for coordinate/order/score equivalence."""
    dense = decode_dense(outputs)
    ids = dense[:, 4:].argmax(axis=1)
    scores = dense[np.arange(len(dense)), 4 + ids]
    chosen = np.flatnonzero(scores >= conf)
    boxes, scores, ids = dense[chosen, :4], scores[chosen], ids[chosen]
    keep = []
    for cls in range(3):
        indices = np.flatnonzero(ids == cls)
        keep.extend(indices[nms(boxes[indices], scores[indices], iou)].tolist())
    keep = sorted(keep, key=lambda j: -scores[j])[:max_det]
    result = []
    for j in keep:
        box = boxes[j].copy()
        box[[0, 2]] = np.clip((box[[0, 2]] - meta['left']) / meta['gain'], 0, meta['width'])
        box[[1, 3]] = np.clip((box[[1, 3]] - meta['top']) / meta['gain'], 0, meta['height'])
        if box[2] <= box[0] or box[3] <= box[1]:
            continue
        result.append(dict(class_id=int(ids[j]), class_name=CLASSES[ids[j]],
                           confidence=float(scores[j]), bbox_xyxy=box.tolist()))
    return result


def heads(seed=0, count=35, dtype=np.float32):
    rng = np.random.default_rng(seed)
    result = []
    for side in (80, 40, 20):
        cls = np.full((1, side, side, 3), -20., dtype=dtype)
        chosen = rng.choice(side * side, size=count, replace=False)
        cls.reshape(-1, 3)[chosen] = rng.normal(0, 4, (count, 3))
        # Varied DFL distributions produce overlap, clipped and out-of-image boxes.
        box = rng.normal(0, 3, (1, side, side, 64)).astype(dtype)
        result.extend((cls, box))
    return result


class CandidateDecoderTests(unittest.TestCase):
    meta = dict(gain=.5, left=0, top=48, width=1280, height=1088)

    def assert_candidates_match_previous(self, outputs, conf, selected):
        expected = previous_candidates(outputs, conf, selected)
        actual = _decode_candidates(outputs, conf, selected)
        for candidate, reference in zip(actual, expected):
            self.assertEqual(candidate.dtype, reference.dtype)
            self.assertEqual(candidate.shape, reference.shape)
            self.assertEqual(candidate.tobytes(), reference.tobytes())

    def test_prefilter_boundary_extreme_confidences_and_sigmoid_ties(self):
        for dtype in (np.float16, np.float32, np.float64):
            outputs = heads(21, count=0, dtype=dtype)
            below = np.nextafter(dtype(-2), dtype(-np.inf))
            above = np.nextafter(dtype(-2), dtype(np.inf))
            values = np.asarray([
                [-2, -2, -2], [below, -80, -80], [above, -80, -80],
                [-1.1, -80, -80], [-1.0986123, -80, -80],
                [0, 0, 0], [16.6, 16.7, 16.8], [50, 70, 60],
                [np.inf, 90, 80], [-np.inf, -80, -80],
                [np.nan, 70, 60], [70, np.nan, 60],
                [-80, 0, -80], [-80, -80, 0],
            ], dtype=dtype)
            for head in outputs[::2]:
                head.reshape(-1, 3)[120:120 + len(values)] = values
            with np.errstate(over='ignore', invalid='ignore'):
                for conf in (1e-30, .12, np.nextafter(.25, 0), .25,
                             np.nextafter(.25, 1), .5, .9999999999):
                    for selected in ((0,), (1,), (2,), (0, 2), (), None):
                        with self.subTest(dtype=dtype, conf=conf, selected=selected):
                            self.assert_candidates_match_previous(outputs, conf, selected)

    def test_prefilter_random_logits_and_mixed_dtypes_are_byte_identical(self):
        rng = np.random.default_rng(193)
        for seed in range(4):
            outputs = heads(seed, count=0)
            for index in (0, 2, 4):
                dtype = np.float64 if seed == 3 and index == 4 else np.float32
                shape = outputs[index].shape
                outputs[index] = rng.uniform(-90, 90, shape).astype(dtype)
            for selected in ((0,), (1,), (2,)):
                for conf in (.25, .7, .9999):
                    with self.subTest(seed=seed, selected=selected, conf=conf):
                        self.assert_candidates_match_previous(outputs, conf, selected)

    def test_random_outputs_match_previous_three_class_results_exactly(self):
        for seed in range(6):
            outputs = heads(seed, dtype=np.float64 if seed == 5 else np.float32)
            for conf, iou, max_det in ((.25, .7, 300), (.5, .3, 12), (.8, 1., 300)):
                with self.subTest(seed=seed, conf=conf, iou=iou):
                    self.assertEqual(detections(outputs, self.meta, conf, iou, max_det),
                                     previous_detections(outputs, self.meta, conf, iou, max_det))

    def test_fire_only_matches_filtered_winners_and_never_promotes_runner_up(self):
        outputs = heads(12)
        all_items = previous_detections(outputs, self.meta, max_det=10000)
        timings = {}
        fire = detections(outputs, self.meta, class_ids=(0,), timings=timings)
        self.assertEqual(fire, [item for item in all_items if item['class_id'] == 0])
        self.assertLess(timings['decoded_candidates'], 8400)
        outputs = heads(count=0)
        outputs[0].reshape(-1, 3)[200] = [2., 4., 3.]
        self.assertEqual(detections(outputs, self.meta, class_ids=(0,)), [])
        self.assertEqual(detections(outputs, self.meta)[0]['class_id'], 1)

    def test_saturated_sigmoid_ties_keep_legacy_first_class(self):
        outputs = heads(count=0)
        outputs[0].reshape(-1, 3)[300] = [50., 70., 60.]
        expected = previous_detections(outputs, self.meta)
        self.assertEqual(expected[0]['class_id'], 0)
        self.assertEqual(detections(outputs, self.meta, class_ids=(0,)), expected)

    def test_empty_candidates_and_class_filters_validate_all_heads(self):
        outputs = heads(count=0)
        timing = {}
        self.assertEqual(detections(outputs, self.meta, timings=timing), [])
        self.assertEqual(timing['decoded_candidates'], 0)
        self.assertEqual(detections(heads(), self.meta, class_ids=()), [])
        for invalid in ((3,), (-1,), (True,), (.0,), ('0',)):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                detections(outputs, self.meta, class_ids=invalid)
        outputs[-1] = outputs[-1].astype(np.int32)
        with self.assertRaisesRegex(ValueError, 'floating-point'):
            detections(outputs, self.meta)

    def test_predict_timing_and_return_contract_without_hardware(self):
        detector = BoardDetector.__new__(BoardDetector)
        detector.class_ids, detector.conf, detector.iou = (0,), .25, .7
        outputs = heads(5)
        with patch.object(detector, 'forward', return_value=outputs) as forward:
            result = detector.predict(np.zeros((1088, 1280, 3), dtype=np.uint8))
        self.assertIsInstance(result, list)
        self.assertTrue(all(item['class_id'] == 0 for item in result))
        self.assertEqual(forward.call_args.args[0].shape, (614400,))
        self.assertEqual(forward.call_args.args[0].dtype, np.uint8)
        for stage in ('letterbox_s', 'nv12_s', 'preprocess_s', 'inference_s',
                      'candidate_decode_s', 'nms_format_s', 'postprocess_s', 'total_s'):
            self.assertGreaterEqual(detector.last_timings[stage], 0)
        self.assertGreaterEqual(detector.last_timings['total_s'],
                                detector.last_timings['preprocess_s'] + detector.last_timings['inference_s'])

    def test_prepared_frames_keep_original_tensor_and_detection_contract(self):
        from rescue_vision.legacy_detector import letterbox, nv12_tensor
        detector = BoardDetector.__new__(BoardDetector)
        detector.class_ids, detector.conf, detector.iou = (0,), .25, .7
        image = np.random.default_rng(20).integers(0, 256, (1088, 1280, 3), dtype=np.uint8)
        image.setflags(write=False)
        padded, meta = letterbox(image)
        expected = nv12_tensor(padded)
        first = detector.prepare(image)
        detector.prepare(np.zeros_like(image))
        np.testing.assert_array_equal(first[0], expected)
        self.assertEqual(first[1], meta)
        outputs = heads(7)
        with patch.object(detector, 'forward', return_value=outputs) as forward:
            result = detector.predict_prepared(first)
        np.testing.assert_array_equal(forward.call_args.args[0], expected)
        self.assertEqual(result, detections(outputs, meta, class_ids=(0,)))


if __name__ == '__main__':
    unittest.main()
