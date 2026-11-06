"""Software geometry checks only; synthetic calibration is never a deployment file."""
import copy
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "rescue_vision"))
from rescue_vision.stereo import CalibrationError, FrameError, StereoDepth, StereoResult


def calibration():
    return dict(calibrated=True, calibration_id="TEST_SYNTHETIC_ONLY", image_size=[320, 192],
                camera_model="pinhole", translation_unit="m",
                K1=[[240., 0, 160.], [0, 240., 96.], [0, 0, 1]],
                K2=[[240., 0, 160.], [0, 240., 96.], [0, 0, 1]],
                D1=[0., 0, 0, 0, 0], D2=[0., 0, 0, 0, 0],
                R=np.eye(3).tolist(), T=[-.06, 0, 0])


def textured_pair(disparity=16):
    rng = np.random.default_rng(734)
    left = rng.integers(0, 256, (192, 320), dtype=np.uint8)
    left = cv2.GaussianBlur(left, (3, 3), .5)
    right = rng.integers(0, 256, left.shape, dtype=np.uint8)
    if disparity > 0:
        right[:, :-disparity] = left[:, disparity:]
    else:
        right[:, -disparity:] = left[:, :disparity]
    return left, right


class StereoTests(unittest.TestCase):
    def setUp(self):
        self.stereo = StereoDepth(calibration(), {"num_disparities": 64})

    def test_known_depth_and_optical_axes(self):
        left, right = self.stereo.rectify(*textured_pair(16))
        result = self.stereo.compute(left, right, .005)
        sampled = self.stereo.sample_bbox(result, [180, 110, 230, 160])
        self.assertTrue(sampled["valid"], sampled)
        self.assertAlmostEqual(sampled["depth_m"], .9, delta=.015)
        self.assertGreater(sampled["xyz_m"][0], 0)
        self.assertGreater(sampled["xyz_m"][1], 0)
        self.assertEqual(sampled["frame_id"], "rectified_left_optical")

    def test_different_disparity_changes_metric_distance(self):
        result = self.stereo.compute(*self.stereo.rectify(*textured_pair(8)))
        point = self.stereo.sample_bbox(result, [130, 60, 210, 130])
        self.assertTrue(point["valid"], point)
        self.assertAlmostEqual(point["depth_m"], 1.8, delta=.025)

    def test_disparity_opposite_calibrated_baseline_is_rejected(self):
        result = self.stereo.compute(*textured_pair(-16))
        point = self.stereo.sample_bbox(result, [130, 60, 210, 130])
        self.assertFalse(point["valid"], point)

    def test_positive_translation_accepts_matching_negative_disparity(self):
        reverse_order = calibration()
        reverse_order["T"] = [.06, 0, 0]
        stereo = StereoDepth(reverse_order, {"num_disparities": 64})
        self.assertEqual(stereo.disparity_sign, -1)
        result = stereo.compute(*stereo.rectify(*textured_pair(-16)), .005)
        point = stereo.sample_bbox(result, [130, 60, 210, 130])
        self.assertTrue(point["valid"], point)
        self.assertAlmostEqual(point["depth_m"], .9, delta=.015)
        self.assertTrue(np.all(result.disparity_px[result.valid_mask] < 0))

    def test_textureless_patch_is_rejected(self):
        left = np.full((192, 320), 127, np.uint8)
        result = self.stereo.compute(left, left.copy())
        self.assertFalse(result.valid_mask.any())
        self.assertFalse(self.stereo.sample_bbox(result, [100, 50, 200, 130])["valid"])

    def test_occluded_right_patch_cannot_return_roi_depth(self):
        left, right = textured_pair(16)
        right[30:165, 74:245] = 127
        result = self.stereo.compute(left, right)
        self.assertFalse(self.stereo.sample_bbox(result, [120, 65, 210, 125])["valid"])

    def test_uncalibrated_template_never_produces_metres(self):
        stereo = StereoDepth({"calibrated": False, "K1": None})
        self.assertFalse(stereo.calibrated)
        self.assertEqual(stereo.disparity_sign, 1)
        with self.assertRaises(CalibrationError):
            stereo.rectify(*textured_pair())
        with self.assertRaises(CalibrationError):
            stereo.compute(*textured_pair())
        self.assertEqual(stereo.sample_bbox(None, [1, 1, 2, 2])["reason"], "uncalibrated")

    def test_units_baseline_rotation_and_intrinsics(self):
        for field, value in [("translation_unit", "mm"), ("T", [60, 0, 0]),
                             ("T", [0, -.06, 0]),
                             ("R", [[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
                             ("K1", [[0, 0, 160], [0, 240, 96], [0, 0, 1]])]:
            bad = copy.deepcopy(calibration())
            bad[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(CalibrationError):
                StereoDepth(bad, {"num_disparities": 64})

    def test_resolution_and_synchronization_must_match(self):
        left, right = textured_pair()
        with self.assertRaises(FrameError):
            self.stereo.compute(left, right, .021)
        with self.assertRaises(FrameError):
            self.stereo.compute(left, right, float("nan"))
        with self.assertRaises(FrameError):
            self.stereo.rectify(left[:, :300], right[:, :300])

    def test_invalid_bbox_and_mixed_depth(self):
        result = self.stereo.compute(*textured_pair())
        for bbox in ([0, 0, 0, 0], [-1, 5, 20, 30], [10, 10, float("nan"), 20]):
            self.assertFalse(self.stereo.sample_bbox(result, bbox)["valid"])
        xyz = np.zeros((192, 320, 3), np.float32)
        xyz[..., 2] = .5
        xyz[:, 160:, 2] = 1.5
        mixed = StereoResult(np.ones((192, 320), np.float32), np.ones((192, 320), bool),
                             xyz, np.zeros((192, 320), np.uint8))
        self.assertEqual(self.stereo.sample_bbox(mixed, [100, 60, 220, 130])["reason"],
                         "mixed_or_unstable_depth")


if __name__ == "__main__":
    unittest.main(verbosity=2)
