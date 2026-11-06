"""Synthetic camera geometry only; never a deployed metric calibration."""
import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.stereo import CalibrationError, DEFAULT_CONFIG, StereoDepth, StereoResult
from rescue_vision.stereo_detection_geometry import CorrectedDetectionGeometry
from test_stereo import calibration, textured_pair


def geometry(angle=.1):
    rotation, _ = cv2.Rodrigues(np.array((.02, angle, .01)))
    rect_k = np.array(((240., 0, 160.), (0, 240., 96.), (0, 0, 1)))
    corrected_k = np.array(((220., 0, 158.), (0, 225., 94.), (0, 0, 1)))
    stereo = SimpleNamespace(calibrated=True, image_size=(320, 192), R1=rotation,
                             P1=np.c_[rect_k, np.zeros(3)], config=dict(DEFAULT_CONFIG),
                             calibration_id='TEST_SYNTHETIC_ONLY')
    return stereo, corrected_k


def facade_result(stereo, z=1.):
    yy, xx = np.indices(stereo.image_size[::-1], dtype=np.float64)
    pixels = np.stack((xx, yy, np.ones_like(xx)), axis=-1)
    rect_rays = pixels @ np.linalg.inv(stereo.P1[:, :3]).T
    left_rays = rect_rays @ stereo.R1
    physical = left_rays * (z / left_rays[..., 2:3])
    rect_xyz = physical @ stereo.R1.T
    shape = stereo.image_size[::-1]
    return StereoResult(np.full(shape, 12., np.float32), np.ones(shape, bool), rect_xyz,
                        np.zeros(shape, np.uint8))


class CorrectedStereoTests(unittest.TestCase):
    def setUp(self):
        self.stereo, self.K = geometry()
        self.adapter = CorrectedDetectionGeometry(self.stereo, self.K, (320, 192))
        self.result = facade_result(self.stereo, .85)

    def test_nonidentity_rotation_and_corrected_K_use_center_ray(self):
        answer = self.adapter.sample_bbox(self.result, (170, 70, 210, 120))
        self.assertTrue(answer['valid'], answer)
        expected = np.linalg.inv(self.K) @ np.array((190., 95., 1.)) * .85
        np.testing.assert_allclose(answer['xyz_m'], expected, atol=1e-10)
        self.assertAlmostEqual(answer['depth_m'], .85, places=10)
        self.assertAlmostEqual(answer['range_m'], np.linalg.norm(expected), places=10)
        self.assertEqual(answer['frame_id'], 'undistorted_left_optical')

    def test_known_disparity_matches_metric_depth_through_real_stereo(self):
        stereo = StereoDepth(calibration(), {'num_disparities': 64})
        adapter = CorrectedDetectionGeometry(stereo, stereo.K1, stereo.image_size)
        result = stereo.compute(*stereo.rectify(*textured_pair(16)), .005)
        answer = adapter.sample_bbox(result, (180, 110, 230, 160))
        self.assertTrue(answer['valid'], answer)
        self.assertAlmostEqual(answer['depth_m'], .9, delta=.015)
        self.assertGreater(answer['xyz_m'][0], 0)
        self.assertGreater(answer['xyz_m'][1], 0)

    def test_asymmetric_valid_pixels_do_not_shift_target_xy(self):
        self.result.valid_mask[:, 225:] = False
        answer = self.adapter.sample_bbox(self.result, (170, 70, 210, 120))
        self.assertTrue(answer['valid'], answer)
        self.assertAlmostEqual(answer['xyz_m'][0], (190 - 158) / 220 * .85, places=10)

    def test_outside_projected_rectangle_cannot_contaminate_depth(self):
        adapter = self.adapter
        inside = ((adapter._u >= 170) & (adapter._u < 210)
                  & (adapter._v >= 70) & (adapter._v < 120))
        self.result.xyz_m[~inside] *= 4
        answer = adapter.sample_bbox(self.result, (170, 70, 210, 120))
        self.assertTrue(answer['valid'], answer)
        self.assertAlmostEqual(answer['depth_m'], .85, places=10)
        # These outside pixels alone must never become evidence for this flame.
        self.result.valid_mask[inside] = False
        self.assertFalse(adapter.sample_bbox(self.result, (170, 70, 210, 120))['valid'])

    def test_stereo_xyz_projecting_into_different_box_is_rejected(self):
        self.result.xyz_m[..., 0] += 5
        answer = self.adapter.sample_bbox(self.result, (170, 70, 210, 120))
        self.assertFalse(answer['valid'], answer)

    def test_nonfinite_negative_depth_and_disparity_rejected(self):
        for field, value in (('xyz_m', np.nan), ('xyz_m', -1.), ('disparity_px', -16.),
                             ('disparity_px', np.nan)):
            result = copy.deepcopy(self.result)
            getattr(result, field)[...] = value
            with self.subTest(field=field, value=value):
                self.assertFalse(self.adapter.sample_bbox(result, (170, 70, 210, 120))['valid'])

    def test_mixed_depth_and_low_valid_fraction_are_rejected(self):
        stereo, K = geometry(0.)
        stereo.R1 = np.eye(3)
        K = stereo.P1[:, :3]
        adapter = CorrectedDetectionGeometry(stereo, K, (320, 192))
        result = facade_result(stereo, .5)
        result.xyz_m[:, 160:] *= 3
        answer = adapter.sample_bbox(result, (100, 60, 220, 130))
        self.assertEqual(answer['reason'], 'mixed_or_unstable_depth')
        result = facade_result(stereo)
        result.valid_mask[...] = False
        result.valid_mask[85:90, 158:163] = True
        answer = adapter.sample_bbox(result, (100, 60, 220, 130))
        self.assertEqual(answer['reason'], 'insufficient_reliable_depth')

    def test_reject_frame_shape_and_calibration_contract_mismatch(self):
        bad = copy.deepcopy(self.result)
        bad.frame_id = 'undistorted_left_optical'
        self.assertEqual(self.adapter.sample_bbox(bad, (170, 70, 210, 120))['reason'],
                         'stereo_result_geometry_mismatch')
        bad = copy.deepcopy(self.result)
        bad.valid_mask = bad.valid_mask[:-1]
        self.assertFalse(self.adapter.sample_bbox(bad, (170, 70, 210, 120))['valid'])
        for kwargs in (dict(frame_id='left_camera_optical'), dict(image_size=(640, 480)),
                       dict(corrected_K=np.zeros((3, 3)))):
            args = dict(stereo=self.stereo, corrected_K=self.K, image_size=(320, 192))
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                CorrectedDetectionGeometry(**args)
        uncalibrated = SimpleNamespace(calibrated=False)
        with self.assertRaises(CalibrationError):
            CorrectedDetectionGeometry(uncalibrated, self.K, (320, 192))

    def test_bbox_and_target_boundaries(self):
        for box in ((-1, 10, 30, 50), (10, 10, 321, 50), (0, 0, 0, 0),
                    (0, 0, np.nan, 20), 'bad'):
            with self.subTest(box=box):
                self.assertFalse(self.adapter.sample_bbox(self.result, box)['valid'])
        for target in ((500, 40), (np.nan, 95), (190, 95, 1)):
            self.assertFalse(self.adapter.sample_bbox(self.result, (170, 70, 210, 120), target)['valid'])
        answer = self.adapter.sample_bbox(self.result, (170, 70, 210, 120), (190, 105))
        self.assertTrue(answer['valid'], answer)
        self.assertAlmostEqual(answer['xyz_m'][1], (105 - 94) / 225 * .85)

    def test_bbox_outside_rectified_field_is_rejected(self):
        stereo, K = geometry(.7)
        adapter = CorrectedDetectionGeometry(stereo, K, (320, 192))
        answer = adapter.sample_bbox(facade_result(stereo), (250, 60, 310, 140))
        self.assertEqual(answer['reason'], 'roi_outside_rectified_view')


if __name__ == '__main__':
    unittest.main(verbosity=2)
