"""Real fire-node callbacks with synthetic geometry; no ROS/hardware I/O."""
from copy import deepcopy
from types import SimpleNamespace as NS
import math
import unittest
from unittest.mock import patch
import numpy as np

import test_fire_aim_geometry as support
from rescue_vision.aim_geometry import VariableRateServo
from rescue_vision.fire_aim_geometry import CORRECTED_FRAME, corrected_camera, load_landing
from rescue_vision.legacy_aiming import AimConfig, FireAimTracker
from rescue_vision.legacy_visual_servo import ServoConfig
from rescue_vision.nozzle_geometry import GeometryNozzleReference


def nozzle_document():
    return dict(nozzle_calibrated=True, calibration_id='synthetic_rigid_mount',
        camera_nozzle_mount_id='rig_test', camera_model='pinhole', physical_eye='left',
        coordinate_frame=CORRECTED_FRAME, image_size_wh=[1280, 1088],
        camera_matrix=np.array(support.camera_info().k).reshape(3, 3).tolist(),
        distortion_coefficients=[0.] * 5, translation_unit='m',
        distance_reference='left_camera_euclidean_m', nozzle_axis='+z',
        nozzle_origin_camera_m=[.025, .01, .02], R_camera_from_nozzle=np.eye(3).tolist(),
        valid_range_m=[.4, 2.])


class FireAimCompensationCallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = support.isolated_node_class()

    def setUp(self):
        # Reuse the ROS stubs and callback fixture without inheriting its tests.
        support.FireAimCallbackTests.setUp(self)
        self.node.fixed_range, self.node.fixed_verified = 0., False
        self.node.nozzle = GeometryNozzleReference(nozzle_document())
        self.node.reference_signature = None
        self.node.landing = None
        self.node.servo_cfg = ServoConfig(enabled=True, mounting_verified=True)
        self.node.planner = VariableRateServo(self.node.servo_cfg)
        self.camera = corrected_camera(self.node.camera_info, CORRECTED_FRAME, (1280, 1088))

    def fire(self, *, distance=.7, center=None, valid=True, width=40):
        if center is None:
            center = self.node.nozzle.reference(self.camera, (1280, 1088), distance)['aim_pixel']
        ray = np.linalg.solve(self.camera.K, [*center, 1.])
        xyz = ray / np.linalg.norm(ray) * distance
        cx, cy = center
        result = support.detection((cx - width / 2, cy - 20, cx + width / 2, cy + 20), xyz=xyz, valid=valid)
        result.class_id, result.confidence = 0, .9
        return result

    def receive(self, number, fires):
        self.node.test_now = 100. + number * .1
        self.node.mission_received = number * .1
        stamp_ns = 100_000_000_000 + number * 100_000_000
        message = NS(header=NS(frame_id=CORRECTED_FRAME,
            stamp=NS(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)),
            sequence=number, width=1280, height=1088, rectified=False,
            detector_available=True, detections=fires)
        with patch.object(self.module.time, 'monotonic', return_value=number * .1):
            self.node.receive(message)
        return self.node.publisher[-1]

    def test_matching_stereo_range_enters_geometry_and_servo_without_pump_permission(self):
        reference = self.node.nozzle.reference(self.camera, (1280, 1088), .7)['aim_pixel']
        center = [reference[0] + 20, reference[1] - 10]
        for number in range(3):
            result = self.receive(number, [self.fire(center=center)])
        self.assertEqual(result['compensation_kind'], 'rigid_nozzle_geometry')
        self.assertEqual(result['range_source'], 'matched_stereo_euclidean')
        self.assertAlmostEqual(result['target_range_m'], .7)
        np.testing.assert_allclose(result['aim']['aim_pixel'], reference)
        np.testing.assert_allclose(result['aim']['position_xyz_m'],
                                   result['geometry_compensation']['target_xyz_m'])
        self.assertEqual(result['aim']['position_frame'], CORRECTED_FRAME)
        expected = self.camera.angle_error(center, reference, (1280, 1088))
        for key, value in expected.items():
            self.assertAlmostEqual(result['aim']['angle_error_deg'][key], value)
            self.assertAlmostEqual(result['geometry_compensation']['error_to_reference_deg'][key], value)
        self.assertIsNotNone(result['servo_plan']['command'])
        self.assertGreater(result['servo_plan']['command']['pan_target_deg'], 90.)
        self.assertGreater(result['servo_plan']['command']['tilt_target_deg'], 90.)
        self.assertFalse(result['landing_compensated'])
        self.assertFalse(result['pump_candidate'])
        self.assertFalse(result['pump_allowed'])

    def test_locked_geometric_target_still_cannot_enable_water(self):
        for number in range(7):
            result = self.receive(number, [self.fire()])
        self.assertTrue(result['image_locked'])
        self.assertEqual(result['compensation_kind'], 'rigid_nozzle_geometry')
        self.assertFalse(result['pump_candidate'])
        self.assertFalse(result['geometry_compensation']['pump_eligible'])
        self.assertFalse(result['physical_hit_verified'])

    def test_new_distance_updates_reference_for_same_tracked_fire(self):
        near = self.receive(0, [self.fire(distance=.5, center=[675, 550])])
        far = self.receive(1, [self.fire(distance=1.2, center=[675, 550])])
        self.assertEqual(near['compensation_kind'], 'rigid_nozzle_geometry')
        self.assertEqual(far['compensation_kind'], 'rigid_nozzle_geometry')
        self.assertGreater(near['aim']['aim_pixel'][0], far['aim']['aim_pixel'][0])
        self.assertGreater(near['aim']['aim_pixel'][1], far['aim']['aim_pixel'][1])
        self.assertAlmostEqual(far['target_range_m'], 1.2)
        self.assertFalse(far['image_locked'])

    def test_measured_water_table_takes_priority_without_adding_geometric_offset(self):
        self.node.landing = load_landing(support.landing_document())
        for number in range(7):
            result = self.receive(number, [self.fire(center=[630, 550])])
        self.assertTrue(result['landing_compensated'])
        self.assertEqual(result['compensation_kind'], 'measured_water_landing')
        self.assertIsNone(result['geometry_compensation'])
        np.testing.assert_allclose(result['aim']['aim_pixel'], [630, 550])
        self.assertTrue(result['image_locked'])
        self.assertTrue(result['pump_candidate'])
        self.assertFalse(result['pump_allowed'])

    def test_water_table_for_different_mount_is_rejected_but_geometry_can_align(self):
        table = support.landing_document()
        table['camera_nozzle_mount_id'] = 'a_different_physical_mount'
        self.node.landing = load_landing(table)
        for number in range(7):
            result = self.receive(number, [self.fire()])
        self.assertFalse(result['landing_compensated'])
        self.assertIn('mount IDs differ', result['landing_reason'])
        self.assertEqual(result['compensation_kind'], 'rigid_nozzle_geometry')
        self.assertTrue(result['image_locked'])
        self.assertFalse(result['pump_candidate'])

    def test_no_fire_detection_never_reuses_previous_reference_or_servo_command(self):
        for number in range(3):
            result = self.receive(number, [self.fire(center=[690, 550])])
        self.assertIsNotNone(result['servo_plan']['command'])
        result = self.receive(3, [])
        self.assertFalse(result['image_locked'])
        self.assertIsNone(result['geometry_compensation'])
        self.assertIsNone(result['target_range_m'])
        self.assertIsNone(result['servo_plan']['command'])
        self.assertFalse(result['pump_candidate'])

    def test_ambiguous_depth_association_blocks_even_pixel_angle_fallback(self):
        self.node.servo_cfg = ServoConfig(enabled=True, mounting_verified=True,
                                         pan_deg_per_px=.1, tilt_deg_per_px=.1)
        self.node.planner = VariableRateServo(self.node.servo_cfg)
        # Both overlapping fire boxes match the tracked box. The visual tracker
        # can choose the nearer centre, but its depth/reference is not unique.
        for number in range(4):
            result = self.receive(number, [self.fire(center=[650, 550], width=80),
                                          self.fire(center=[664, 550], width=80)])
        self.assertEqual(result['compensation_kind'], 'target_association_unverified')
        self.assertIsNone(result['geometry_compensation'])
        self.assertIsNone(result['aim']['angle_error_deg'])
        self.assertFalse(result['image_locked'])
        self.assertIsNone(result['servo_plan']['command'])
        self.assertFalse(result['pump_candidate'])

    def test_depth_loss_clears_old_lock_and_depth_recovery_requires_new_evidence(self):
        for number in range(7):
            result = self.receive(number, [self.fire()])
        self.assertTrue(result['image_locked'])
        result = self.receive(7, [self.fire(valid=False)])
        self.assertEqual(result['compensation_kind'], 'image_alignment_only')
        self.assertEqual(result['range_source'], 'unavailable')
        self.assertFalse(result['image_locked'])
        self.assertIsNone(result['target_range_m'])
        self.assertIsNone(result['geometry_compensation'])
        self.assertIsNone(result['aim']['position_xyz_m'])
        self.assertFalse(result['pump_candidate'])
        recovered = self.receive(8, [self.fire()])
        self.assertEqual(recovered['compensation_kind'], 'rigid_nozzle_geometry')
        self.assertFalse(recovered['image_locked'])
        self.assertEqual(recovered['aim']['hits_in_window'], 1)


class ReferenceNoiseDwellTests(unittest.TestCase):
    def setUp(self):
        self.tracker = FireAimTracker(AimConfig(reference_reset_radius_px=2.))
        self.fire = dict(class_id=0, confidence=.9, bbox_xyxy=[620, 524, 660, 564])

    def update(self, number, dx):
        now = 100. + number * .1
        return self.tracker.update([self.fire], (1280, 1088), number, now, now, [640 + dx, 544])

    def test_small_range_noise_allows_dwell_instead_of_permanent_reacquisition(self):
        for number, dx in enumerate([0., .2, -.2, .5, -.4, .3, -.1]):
            result = self.update(number, dx)
        self.assertTrue(result['image_locked'])
        self.assertLess(abs(result['pixel_error']['x_right']), 1.)

    def test_cumulative_reference_motion_resets_lock_even_when_each_step_is_small(self):
        for number in range(7):
            result = self.update(number, 0.)
        self.assertTrue(result['image_locked'])
        for number, dx in enumerate([.8, 1.6], start=7):
            self.assertTrue(self.update(number, dx)['image_locked'])
        result = self.update(9, 2.4)
        self.assertFalse(result['image_locked'])
        self.assertIsNone(self.tracker.stable_since)
        np.testing.assert_allclose(self.tracker.target_reference, [642.4, 544.])
        for number in range(10, 16):
            result = self.update(number, 2.4)
        self.assertTrue(result['image_locked'])


if __name__ == '__main__':
    unittest.main()
