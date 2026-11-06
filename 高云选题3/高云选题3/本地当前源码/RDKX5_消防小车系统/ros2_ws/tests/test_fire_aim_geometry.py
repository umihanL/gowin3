"""Safety and geometry contracts for corrected-camera fire proposals, no ROS I/O."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import importlib.util
import json
import sys
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.fire_aim_geometry import (CORRECTED_FRAME, corrected_camera, select_camera,
    load_landing, matched_detection, target_range, landing_reference, mission_fresh, pump_candidate)
from rescue_vision.legacy_aiming import FireAimTracker, AimConfig


def camera_info():
    matrix = np.array([[436.82464992353863, 0, 640.2224645421431],
                       [0, 502.6057228760608, 539.0150333700744], [0, 0, 1.]])
    return NS(header=NS(frame_id=CORRECTED_FRAME), width=1280, height=1088,
              k=matrix.ravel().tolist(), d=[0.] * 5, r=np.eye(3).ravel().tolist(),
              p=np.column_stack((matrix, np.zeros(3))).ravel().tolist(), binning_x=0, binning_y=0)


def landing_document():
    return dict(landing_calibrated=True, calibration_id='measured_test', eye='left', physical_eye='left',
        coordinate_frame=CORRECTED_FRAME, image_size_wh=[1280, 1088],
        camera_matrix=np.array(camera_info().k).reshape(3, 3).tolist(),
        distance_reference='left_camera_euclidean_m', pump_setting_id='pump_test',
        camera_nozzle_mount_id='rig_test', measurements=[
            dict(distance_m=.4, landing_pixel=[620, 540]), dict(distance_m=1., landing_pixel=[640, 560])])


def detection(box=(620, 524, 660, 564), xyz=(0, 0, .7), valid=True):
    return NS(bbox_xyxy=list(box), depth_valid=valid, position_m=NS(x=xyz[0], y=xyz[1], z=xyz[2]))


class FireAimGeometryTests(unittest.TestCase):
    def setUp(self):
        self.info = camera_info()
        self.camera = corrected_camera(self.info, CORRECTED_FRAME, (1280, 1088))

    def test_corrected_angle_uses_new_matrix_and_no_raw_fallback(self):
        result = self.camera.angle_error([700, 544], [640, 544], (1280, 1088))
        expected = np.degrees(np.arctan((700 - self.camera.K[0, 2]) / self.camera.K[0, 0])
                              - np.arctan((640 - self.camera.K[0, 2]) / self.camera.K[0, 0]))
        self.assertAlmostEqual(result['yaw_right_positive'], expected)
        self.assertGreater(result['yaw_right_positive'], 7.)
        with self.assertRaises(ValueError):
            select_camera(CORRECTED_FRAME, (1280, 1088), False, None, NS(calibrated=True))
        with self.assertRaises(ValueError):
            select_camera(CORRECTED_FRAME, (1280, 1088), True, self.info, NS(calibrated=True))

    def test_bad_camera_info_is_rejected(self):
        cases = []
        for attr, value in [('width', 640), ('d', [.1] * 5), ('k', [float('nan')] * 9),
                            ('r', [0.] * 9), ('p', [0.] * 12), ('binning_x', 2)]:
            item = deepcopy(self.info); setattr(item, attr, value); cases.append(item)
        item = deepcopy(self.info); item.header.frame_id = 'left_camera_optical'; cases.append(item)
        for item in cases:
            with self.assertRaises(ValueError):
                corrected_camera(item, CORRECTED_FRAME, (1280, 1088))

    def test_range_belongs_to_matched_fire_and_is_euclidean(self):
        first, second = detection(xyz=(.3, .4, .5)), detection((100, 100, 150, 160), xyz=(0, 0, 2))
        self.assertIs(matched_detection([first, second], first.bbox_xyxy), first)
        self.assertIsNone(matched_detection([first, second], None))
        value, source = target_range(first)
        self.assertAlmostEqual(value, np.sqrt(.5)); self.assertEqual(source, 'matched_stereo_euclidean')
        self.assertEqual(target_range(detection(xyz=(float('nan'), 0, 1)), fixed_range_m=.7, fixed_verified=True),
                         (None, 'invalid_depth'))

    def test_fixed_range_requires_explicit_verification_and_target(self):
        fire = detection(valid=False)
        self.assertEqual(target_range(fire, fixed_range_m=.7), (None, 'unavailable'))
        self.assertEqual(target_range(None, fixed_range_m=.7, fixed_verified=True), (None, 'unavailable'))
        self.assertEqual(target_range(fire, fixed_range_m=.7, fixed_verified=True),
                         (.7, 'verified_fixed_observation_assumption'))

    def test_landing_interpolates_only_matching_measured_geometry(self):
        table = load_landing(landing_document())
        reference, identifier = landing_reference(table, self.camera, CORRECTED_FRAME, (1280, 1088), .7)
        np.testing.assert_allclose(reference, [630, 550]); self.assertEqual(identifier, 'measured_test')
        for distance in (.39, 1.01, float('nan')):
            with self.assertRaises(ValueError):
                landing_reference(table, self.camera, CORRECTED_FRAME, (1280, 1088), distance)
        for key, value in [('coordinate_frame', 'left_camera_optical'), ('eye', 'right'),
                           ('physical_eye', 'right'), ('pump_setting_id', None),
                           ('distance_reference', 'depth_z_m')]:
            document = landing_document(); document[key] = value
            with self.assertRaises(ValueError):
                landing_reference(load_landing(document), self.camera, CORRECTED_FRAME, (1280, 1088), .7)
        document = landing_document(); document['camera_matrix'][0][0] += .1
        with self.assertRaises(ValueError):
            landing_reference(load_landing(document), self.camera, CORRECTED_FRAME, (1280, 1088), .7)

    def test_unmeasured_or_insufficient_landing_is_never_ready(self):
        self.assertIsNone(load_landing({'landing_calibrated': False}))
        document = landing_document(); document['measurements'] = document['measurements'][:1]
        with self.assertRaises(ValueError):
            load_landing(document)

    def test_landing_rejects_distortion_change_even_when_K_matches(self):
        document=landing_document();document['distortion_coefficients']=[.1,0,0,0,0]
        with self.assertRaisesRegex(ValueError,'distortion'):
            landing_reference(load_landing(document),self.camera,CORRECTED_FRAME,(1280,1088),.7)

    def test_loaded_landing_geometry_cannot_be_changed_by_source_mutation(self):
        document=landing_document();table=load_landing(document)
        document['coordinate_frame']='left_camera_optical'
        document['measurements'][0]['landing_pixel'][0]=100
        point,_=landing_reference(table,self.camera,CORRECTED_FRAME,(1280,1088),.7)
        np.testing.assert_allclose(point,[630,550])

    def test_mission_freshness_expires_even_without_new_mode_message(self):
        state = dict(mode='fire', gimbal_ready=True, traffic_active=False)
        self.assertTrue(mission_fresh(state, 10., 10.49))
        self.assertFalse(mission_fresh(state, 10., 10.51))
        self.assertFalse(mission_fresh(state, None, 10.))
        self.assertFalse(mission_fresh(state, 10., 9.))
        self.assertFalse(mission_fresh(dict(state, gimbal_ready=False), 10., 10.1))
        self.assertFalse(mission_fresh(dict(state, traffic_active=True), 10., 10.1))
        self.assertFalse(mission_fresh(dict(state, traffic_active='false'), 10., 10.1))

    def test_three_of_five_and_dwell_are_image_lock_not_physical_hit(self):
        tracker = FireAimTracker(AimConfig(max_frame_age_s=.5), self.camera)
        fire = dict(class_id=0, confidence=.9, bbox_xyxy=[620, 524, 660, 564])
        for frame in range(7):
            now = 100 + frame * .1
            result = tracker.update([fire], (1280, 1088), frame, now, now)
            if frame < 6:
                self.assertFalse(result['image_locked'])
        self.assertTrue(result['image_locked'])
        self.assertFalse(result['physical_hit_verified'])
        kwargs = dict(valid=True, compensated=True, range_m=.7, calibration_id='measured')
        self.assertTrue(pump_candidate(result, **kwargs))
        for key, value in [('valid', False), ('compensated', False), ('range_m', None), ('calibration_id', None)]:
            changed = dict(kwargs); changed[key] = value
            self.assertFalse(pump_candidate(result, **changed))
        stale = tracker.update([fire], (1280, 1088), 6, now, now + .1)
        self.assertFalse(stale['image_locked'])
        self.assertFalse(pump_candidate(stale, **kwargs))


def isolated_node_class():
    """Use real node callbacks with minimal ROS type stubs; no ports or threads."""
    common = NS(IMAGE_QOS=None, param=lambda *args: args[-1], read_json=lambda value: value,
        seconds=lambda stamp: stamp.sec + stamp.nanosec / 1e9,
        age=lambda node, stamp: node.test_now - (stamp.sec + stamp.nanosec / 1e9),
        publish_json=lambda publisher, document: publisher.append(json.loads(json.dumps(document, allow_nan=False))),
        spin=lambda factory: None)
    modules = {'rclpy': NS(), 'rclpy.node': NS(Node=object),
        'rclpy.qos': NS(QoSProfile=object, ReliabilityPolicy=NS(RELIABLE=1), DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
        'sensor_msgs': NS(), 'sensor_msgs.msg': NS(CameraInfo=object),
        'std_msgs': NS(), 'std_msgs.msg': NS(String=object),
        'rescue_interfaces': NS(), 'rescue_interfaces.msg': NS(DetectionArray=object),
        'rescue_vision.node_common': common}
    path = Path(__file__).resolve().parents[1] / 'src/rescue_vision/rescue_vision/fire_aim_node.py'
    spec = importlib.util.spec_from_file_location('rescue_vision._fire_aim_callbacks_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class FireAimCallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = isolated_node_class()

    def setUp(self):
        from rescue_vision.legacy_visual_servo import ServoConfig
        from rescue_vision.aim_geometry import VariableRateServo
        self.node = self.module.FireAim.__new__(self.module.FireAim)
        self.node.mission = dict(mode='fire', gimbal_ready=True, traffic_active=False, gimbal_target_deg=[90, 90])
        self.node.mission_received = 0.
        self.node.last_detection_received = None
        self.node.mission_timeout = self.node.max_age = .5
        self.node.fixed_range, self.node.fixed_verified = .7, True
        self.node.camera_info = camera_info()
        self.node.stereo = NS(calibrated=False)
        self.node.tracker = self.node.coordinate_mode = None
        self.node.servo_cfg = ServoConfig()
        self.node.planner = VariableRateServo(self.node.servo_cfg)
        self.node.landing = load_landing(landing_document())
        self.node.publisher = []
        self.node.test_now = 100.
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(self.node.test_now * 1e9)))

    def frame(self, number):
        fire = detection((610, 530, 650, 570), valid=False)
        fire.class_id, fire.confidence = 0, .9
        return NS(header=NS(frame_id=CORRECTED_FRAME, stamp=NS(sec=100, nanosec=number * 100_000_000)),
            sequence=number, width=1280, height=1088, rectified=False, detector_available=True, detections=[fire])

    def test_proposal_lock_contract_and_mission_expiration(self):
        for number in range(7):
            self.node.test_now = 100. + number * .1
            self.node.mission_received = number * .1
            with patch.object(self.module.time, 'monotonic', return_value=number * .1):
                self.node.receive(self.frame(number))
        result = self.node.publisher[-1]
        self.assertTrue(result['image_locked'])
        self.assertTrue(result['aim']['image_locked'])
        self.assertTrue(result['pump_candidate'])
        self.assertFalse(result['pump_allowed'])
        self.assertFalse(result['physical_hit_verified'])
        self.assertEqual(result['servo_reference_target_deg'], [90., 90.])
        self.assertEqual(result['coordinate_frame'], CORRECTED_FRAME)
        self.assertEqual(result['aim']['coordinate_space'], CORRECTED_FRAME)
        self.assertEqual(result['range_source'], 'verified_fixed_observation_assumption')
        with patch.object(self.module.time, 'monotonic', return_value=1.11):
            self.node.watchdog()
        self.assertFalse(self.node.publisher[-1]['valid'])
        self.assertFalse(self.node.publisher[-1]['pump_candidate'])
        self.assertIsNone(self.node.tracker)

    def test_missing_camera_info_blocks_even_with_raw_stereo_calibration(self):
        self.node.camera_info = None
        self.node.stereo = NS(calibrated=True)
        with patch.object(self.module.time, 'monotonic', return_value=.1):
            self.node.receive(self.frame(0))
        self.assertFalse(self.node.publisher[-1]['valid'])
        self.assertIn('camera_geometry_rejected', self.node.publisher[-1]['reason'])

    def test_changed_camera_info_immediately_invalidates_previous_lock(self):
        with patch.object(self.module.time, 'monotonic', return_value=.1):
            self.node.receive(self.frame(0))
        info = camera_info(); info.k[0] += 1; info.p[0] += 1
        self.node.info(info)
        self.assertFalse(self.node.publisher[-1]['valid'])
        self.assertEqual(self.node.publisher[-1]['reason'], 'camera_geometry_changed')


if __name__ == '__main__':
    unittest.main()
