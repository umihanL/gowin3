"""Analytic rigid-nozzle cases, strict input contracts, no hardware access."""
from copy import deepcopy
from pathlib import Path
import json
import math
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.legacy_aiming import CameraCalibration
from rescue_vision.nozzle_geometry import GeometryNozzleReference, CORRECTED_FRAME


def document():
    return dict(nozzle_calibrated=True, calibration_id='synthetic_nozzle',
        camera_nozzle_mount_id='synthetic_mount', camera_model='pinhole', physical_eye='left',
        coordinate_frame=CORRECTED_FRAME, image_size_wh=[640, 480],
        camera_matrix=[[400., 0, 310.], [0, 420., 230.], [0, 0, 1.]],
        distortion_coefficients=[0.] * 5, translation_unit='m',
        distance_reference='left_camera_euclidean_m', nozzle_axis='+z',
        nozzle_origin_camera_m=[0., 0., 0.], R_camera_from_nozzle=np.eye(3).tolist(),
        valid_range_m=[.4, 2.])


def camera(doc=None):
    doc = deepcopy(doc or document())
    doc.update(camera_calibrated=True, eye='left', calibration_id='synthetic_camera')
    return CameraCalibration(doc)


class NozzleGeometryTests(unittest.TestCase):
    def setUp(self):
        self.doc = document()
        self.camera = camera()

    def reference(self, doc=None, distance=.8, **kwargs):
        return GeometryNozzleReference(doc or self.doc).reference(self.camera, (640, 480), distance, **kwargs)

    def test_zero_offset_aligns_to_principal_point_and_never_permits_pump(self):
        result = self.reference()
        np.testing.assert_allclose(result['aim_pixel'], [310, 230])
        np.testing.assert_allclose(result['reference_xyz_m'], [0, 0, .8])
        self.assertEqual(result['reference_angle_deg'], {'yaw_right_positive': 0., 'pitch_up_positive': -0.})
        self.assertFalse(result['pump_eligible'])
        self.assertFalse(result['physical_hit_verified'])
        self.assertFalse(result['gravity_compensated'])

    def test_right_offset_reference_is_right_and_servo_error_is_left(self):
        self.doc['nozzle_origin_camera_m'] = [.06, 0., 0.]
        result = self.reference(distance=.6)
        self.assertAlmostEqual(result['aim_pixel'][0], 310 + 400 * .06 / math.sqrt(.6 ** 2 - .06 ** 2))
        error = self.camera.angle_error([310, 230], result['aim_pixel'], (640, 480))
        self.assertAlmostEqual(error['yaw_right_positive'], -math.degrees(math.asin(.06 / .6)))

    def test_down_offset_reference_is_down_and_pitch_error_is_up(self):
        self.doc['nozzle_origin_camera_m'] = [0., .03, 0.]
        result = self.reference(distance=.6)
        self.assertGreater(result['aim_pixel'][1], 230.)
        error = self.camera.angle_error([310, 230], result['aim_pixel'], (640, 480))
        self.assertAlmostEqual(error['pitch_up_positive'], math.degrees(math.asin(.03 / .6)))

    def test_translation_parallel_to_axis_changes_travel_not_reference(self):
        self.doc['nozzle_origin_camera_m'] = [0., 0., .1]
        result = self.reference(distance=.7)
        self.assertAlmostEqual(result['nozzle_travel_m'], .6)
        np.testing.assert_allclose(result['aim_pixel'], [310, 230])

    def test_measured_boresight_gives_range_invariant_reference(self):
        theta = math.radians(12.)
        self.doc['R_camera_from_nozzle'] = [[math.cos(theta), 0, math.sin(theta)],
            [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]]
        near, far = self.reference(distance=.4), self.reference(distance=2.)
        np.testing.assert_allclose(near['aim_pixel'], far['aim_pixel'])
        self.assertAlmostEqual(near['aim_pixel'][0], 310 + 400 * math.tan(theta))
        self.assertAlmostEqual(near['reference_angle_deg']['yaw_right_positive'], 12.)

    def test_tilted_offset_ray_preserves_range_and_reprojects_its_axis(self):
        theta = math.radians(-8.)
        self.doc['nozzle_origin_camera_m'] = [.04, .03, .02]
        self.doc['R_camera_from_nozzle'] = [[1, 0, 0],
            [0, math.cos(theta), -math.sin(theta)], [0, math.sin(theta), math.cos(theta)]]
        result = self.reference(distance=1.2)
        point = np.asarray(result['reference_xyz_m'])
        axis = np.asarray(result['nozzle_axis_camera'])
        np.testing.assert_allclose(np.cross(point - self.doc['nozzle_origin_camera_m'], axis), 0, atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(point), 1.2)
        np.testing.assert_allclose(self.camera.K @ point / point[2], result['aim_pixel'] + [1.])

    def test_parallax_decreases_with_distance(self):
        self.doc['nozzle_origin_camera_m'] = [.05, 0, 0]
        self.assertGreater(self.reference(distance=.4)['aim_pixel'][0], self.reference(distance=2.)['aim_pixel'][0])

    def test_xyz_uses_euclidean_distance_and_reports_visual_angle_error(self):
        xyz = [.3, .4, math.sqrt(.75)]
        result = self.reference(distance=None, target_xyz_m=xyz)
        self.assertAlmostEqual(result['target_range_m'], 1.)
        self.assertAlmostEqual(result['error_to_reference_deg']['yaw_right_positive'],
                               math.degrees(math.atan2(.3, math.sqrt(.75))))
        self.assertAlmostEqual(result['error_to_reference_deg']['pitch_up_positive'],
                               math.degrees(math.atan2(-.4, math.sqrt(.84))))
        self.reference(distance=1., target_xyz_m=xyz)
        with self.assertRaises(ValueError):
            self.reference(distance=xyz[2], target_xyz_m=xyz)

    def test_invalid_target_positions_are_rejected(self):
        for xyz in ([0, 0, -1], [0, 0, 0], [0, float('nan'), 1], [0, 0], [[0, 0, 1]]):
            with self.subTest(xyz=xyz), self.assertRaises(ValueError):
                self.reference(distance=None, target_xyz_m=xyz)

    def test_unmeasured_template_and_wrong_calibration_flags_are_disabled(self):
        template = json.loads((Path(__file__).resolve().parents[1] / 'config/nozzle.template.json').read_text(encoding='utf-8'))
        for doc in (template, dict(self.doc, nozzle_calibrated='true'), dict(self.doc, nozzle_calibrated=1)):
            geometry = GeometryNozzleReference(doc)
            self.assertFalse(geometry.ready)
            with self.assertRaises(ValueError):
                geometry.reference(self.camera, (640, 480), .8)

    def test_malformed_mount_calibrations_fail_closed(self):
        cases = [('R_camera_from_nozzle', np.diag([1., 1., -1.]).tolist()),
                 ('R_camera_from_nozzle', np.diag([1., 1., 2.]).tolist()),
                 ('R_camera_from_nozzle', [[float('nan')] * 3] * 3),
                 ('nozzle_origin_camera_m', [0, 0]), ('nozzle_origin_camera_m', [0, 0, float('inf')]),
                 ('camera_model', 'fisheye'), ('physical_eye', 'right'),
                 ('coordinate_frame', 'rectified_left_optical'), ('translation_unit', 'mm'),
                 ('distance_reference', 'depth_z_m'), ('nozzle_axis', '+x'),
                 ('calibration_id', ''), ('camera_nozzle_mount_id', None),
                 ('image_size_wh', [True, 480]), ('camera_matrix', np.eye(2).tolist()),
                 ('distortion_coefficients', [.1] * 5), ('valid_range_m', [.4, .4]),
                 ('valid_range_m', [None, None]), ('valid_range_m', [2., .4])]
        for key, value in cases:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                GeometryNozzleReference(dict(self.doc, **{key: value}))
        with self.assertRaises(ValueError):
            GeometryNozzleReference(dict(self.doc, nozzle_origin_camera_m=[.4, 0, 0]))

    def test_only_supported_finite_numeric_ranges_are_accepted(self):
        for distance in (.399, 2.001, 0, -1, float('nan'), float('inf'), True, '.8', None):
            with self.subTest(distance=distance), self.assertRaises(ValueError):
                self.reference(distance=distance)
        self.reference(distance=.4)
        self.reference(distance=2.)

    def test_reference_behind_camera_or_outside_fov_is_rejected(self):
        for theta in (90., 180.):
            angle = math.radians(theta)
            self.doc['R_camera_from_nozzle'] = [[math.cos(angle), 0, math.sin(angle)],
                [0, 1, 0], [-math.sin(angle), 0, math.cos(angle)]]
            with self.subTest(theta=theta), self.assertRaises(ValueError):
                self.reference()

    def test_current_camera_must_match_frame_resolution_eye_and_matrix(self):
        geometry = GeometryNozzleReference(self.doc)
        for field, value in [('coordinate_frame', 'left_camera_optical'), ('physical_eye', 'right'),
                             ('eye', 'right'), ('camera_model', 'fisheye'), ('calibration_id', '')]:
            changed = deepcopy(self.camera); changed.doc[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                geometry.reference(changed, (640, 480), .8)
        for field, value in [('ready', False), ('K', self.camera.K * 2), ('dist', np.ones(5)),
                             ('size', (320, 240))]:
            changed = deepcopy(self.camera); setattr(changed, field, value)
            with self.subTest(field=field), self.assertRaises(ValueError):
                geometry.reference(changed, (640, 480), .8)
        with self.assertRaises(ValueError):
            geometry.reference(self.camera, (320, 240), .8)
        with self.assertRaises(ValueError):
            geometry.reference(self.camera, (640, 480), .8, frame_id='rectified_left_optical')

    def test_callers_cannot_mutate_validated_geometry_through_document(self):
        self.doc['nozzle_origin_camera_m'] = np.array([0., 0., 0.])
        self.doc['R_camera_from_nozzle'] = np.eye(3)
        geometry = GeometryNozzleReference(self.doc)
        self.doc['nozzle_origin_camera_m'][0] = 99.
        self.doc['R_camera_from_nozzle'][0, 2] = 99.
        # Both Python lists and ndarray inputs must be detached from callers.
        np.testing.assert_allclose(geometry.reference(self.camera, (640, 480), .8)['aim_pixel'], [310, 230])


if __name__ == '__main__':
    unittest.main()
