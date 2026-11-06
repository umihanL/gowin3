"""Geometry and guardrails for candidate monocular EEPROM correction."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "rescue_vision"))
from rescue_vision.lens_correction import LensCorrector


def fixture():
    source = {"width": 320, "height": 256, "distortion_model": "plumb_bob",
              "K": [180, 0, 160, 0, 181, 128, 0, 0, 1],
              "D": [-.24, .08, 0, 0, 0, 0, 0, 0],
              "R": [1, 0, 0, 0, 1, 0, 0, 0, 1],
              "P": [180, 0, 160, 0, 0, 181, 128, 0, 0, 0, 1, 0]}
    other = deepcopy(source)
    other["K"][0] = 183
    return {"schema_version": 1,
            "provenance": {"calibration_rotation_degrees": 90, "unrelated_evidence": "preserve"},
            "driver": {"parameters": {"rotation": "90.0"}},
            "role_to_source": {"left": "/image_right_raw", "right": "/image_left_raw"},
            "sources": {"/image_right_raw": source, "/image_left_raw": other}}


class LensCorrectionTests(unittest.TestCase):
    def test_distorted_vertical_line_becomes_straight(self):
        config = fixture()
        camera = config["sources"]["/image_right_raw"]
        xyz = np.array([[.55, y, 1] for y in np.linspace(-.55, .55, 150)])
        projected, _ = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3),
                                         np.array(camera["K"], float).reshape(3, 3),
                                         np.array(camera["D"], float))
        raw = np.zeros((256, 320), np.uint8)
        cv2.polylines(raw, [np.round(projected).astype(np.int32)], False, 255, 2)
        result = LensCorrector(config).correct(raw, "left")
        raw_y, raw_x = np.nonzero(raw > 100)
        y, x = np.nonzero(result.image > 100)
        self.assertGreater(np.std(raw_x), 1.5)
        self.assertLess(np.std(x), 1.2)
        expected_x = result.camera_matrix[0, 0] * .55 + result.camera_matrix[0, 2]
        self.assertLess(abs(np.mean(x) - expected_x), 1.0)
        self.assertGreater(len(y), 100)

    def test_role_mapping_caches_independent_eyes_and_preserves_metadata(self):
        config = fixture()
        correction = LensCorrector(config)
        image = np.zeros((256, 320, 3), np.uint8)
        left = correction.correct(image, "left", output_size=(160, 128))
        correction.correct(image, "left", output_size=(160, 128))
        right = correction.correct(image, "right", output_size=(160, 128))
        self.assertEqual(correction.maps_built, 2)
        self.assertEqual(left.image.shape, (128, 160, 3))
        self.assertEqual(left.metadata["source_topic"], "/image_right_raw")
        self.assertEqual(right.metadata["source_topic"], "/image_left_raw")
        self.assertFalse(np.allclose(left.newK, right.newK))
        self.assertEqual(left.metadata["source_metadata"]["camera_info"],
                         config["sources"]["/image_right_raw"])
        left.metadata["source_metadata"]["camera_info"]["K"][0] = 1
        again = correction.correct(image)
        self.assertEqual(again.metadata["source_metadata"]["camera_info"]["K"][0], 180)
        self.assertFalse(again.metadata["metric_depth_verified"])

    def test_resolution_mismatch_rejected_before_building_maps(self):
        correction = LensCorrector(fixture())
        for shape in ((128, 160, 3), (320, 256, 3), (255, 320, 3)):
            with self.subTest(shape=shape), self.assertRaisesRegex(ValueError, "resolution"):
                correction.correct(np.zeros(shape, np.uint8))
        self.assertEqual(correction.maps_built, 0)

    def test_rotation_and_source_mismatch_rejected(self):
        image = np.zeros((256, 320), np.uint8)
        correction = LensCorrector(fixture())
        for metadata in ({"rotation": 0}, {"rotation_degrees": 270},
                         {"cal_rotation": float("nan")}, {"source_topic": "/image_left_raw"}):
            with self.subTest(metadata=metadata), self.assertRaises(ValueError):
                correction.correct(image, input_metadata=metadata)
        config = fixture()
        config["driver"]["parameters"]["rotation"] = 0
        with self.assertRaisesRegex(ValueError, "rotation"):
            LensCorrector(config)

    def test_already_rectified_image_not_corrected_twice(self):
        correction = LensCorrector(fixture())
        image = np.arange(256 * 320, dtype=np.uint8).reshape(256, 320)
        K = np.array([[200, 0, 160], [0, 200, 128], [0, 0, 1]], float)
        result = correction.correct(image, already_rectified=True, rectified_camera_matrix=K)
        np.testing.assert_array_equal(result.image, image)
        np.testing.assert_array_equal(result.newK, K)
        self.assertEqual(correction.maps_built, 0)
        self.assertFalse(result.metadata["correction_applied_this_call"])
        resized = correction.correct(image, input_metadata={"rectified": True, "camera_matrix": K.tolist()},
                                     output_size=(160, 128))
        np.testing.assert_allclose(resized.newK, [[100, 0, 80], [0, 100, 64], [0, 0, 1]])
        with self.assertRaisesRegex(ValueError, "actual rectified"):
            correction.correct(image, already_rectified=True)
        for flag in ("rectified", "lens_correction_applied", "gdc_enable"):
            result = correction.correct(image, rectified_camera_matrix=K,
                                        input_metadata={flag: "True"})
            np.testing.assert_array_equal(result.image, image)
        self.assertEqual(correction.maps_built, 0)
        config = fixture()
        config["driver"]["parameters"]["gdc_enable"] = "True"
        with self.assertRaisesRegex(ValueError, "actual rectified"):
            LensCorrector(config).correct(image)
        corrected = correction.correct(image, input_metadata={"gdc_enable": "False"})
        self.assertTrue(corrected.metadata["correction_applied_this_call"])

    def test_fisheye_and_truncated_or_nonfinite_coefficients_are_rejected(self):
        for model, D in (("equidistant", [.1, 0, 0, 0]), ("plumb_bob", [0] * 5),
                         ("plumb_bob", [float("nan")] + [0] * 7)):
            config = fixture()
            config["sources"]["/image_right_raw"].update(distortion_model=model, D=D)
            with self.subTest(model=model, D=D), self.assertRaises(ValueError):
                LensCorrector(config)

    def test_explicit_roles_override_saved_mapping(self):
        correction = LensCorrector(fixture(), role_to_source={"observing": "/image_left_raw"})
        result = correction.correct(np.zeros((256, 320), np.uint8), role="observing")
        self.assertEqual(result.metadata["source_topic"], "/image_left_raw")
        with self.assertRaisesRegex(ValueError, "Unknown"):
            correction.correct(np.zeros((256, 320), np.uint8), role="left")

    def test_real_eeprom_snapshot_and_half_resolution_maps(self):
        path = ROOT / "config" / "gs130w_intrinsics_20261010.json"
        config = json.loads(path.read_text(encoding="utf-8"))
        correction = LensCorrector.from_json(path)
        self.assertEqual(config["provenance"]["eeprom_header"]["angle"], 1)
        for role in ("left", "right"):
            result = correction.correct(np.zeros((1088, 1280, 3), np.uint8), role,
                                         output_size=(640, 544), input_metadata={"rotation": 90})
            self.assertEqual(result.image.shape, (544, 640, 3))
            self.assertTrue(np.isfinite(result.newK).all())
            self.assertGreater(result.newK[0, 0], 100)
            self.assertIn("candidate", result.metadata["calibration_status"])


if __name__ == "__main__":
    unittest.main()
