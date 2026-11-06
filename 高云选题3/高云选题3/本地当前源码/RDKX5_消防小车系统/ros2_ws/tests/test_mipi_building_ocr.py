import unittest
from copy import deepcopy
from types import SimpleNamespace as NS
import numpy as np

from rescue_vision.mipi_building_ocr import (BuildingObservation, ExactCaptureJoin,
    corrected_observation_image, observation_signature)
from rescue_vision.fire_aim_geometry import CORRECTED_FRAME, corrected_camera


def corrected_info():
    K = np.array([[600., 0, 640], [0, 600., 544], [0, 0, 1]])
    return NS(header=NS(frame_id=CORRECTED_FRAME), width=1280, height=1088,
              k=K.ravel().tolist(), d=[0.]*5, r=np.eye(3).ravel().tolist(),
              p=np.column_stack((K, np.zeros(3))).ravel().tolist())


CONFIG = {'buildings': [
    {'building_id': 'A', 'number': '01', 'name': '安居楼'},
    {'building_id': 'K', 'number': '11', 'name': '绿荫楼'}]}


def reading(text='1号楼'):
    return dict(text=text, confidence=.98, quad=[[10, 20], [100, 20], [100, 45], [10, 45]])


def detection(stamp, sequence=7):
    return dict(capture_stamp_ns=round(stamp*1e9), capture_stamp=stamp,
        sequence=sequence, coordinate_frame='left', detector_available=True,
        detections=[dict(class_name='fire', confidence=.9, bbox=[10, 50, 50, 100])])


class MipiBuildingOCRTests(unittest.TestCase):
    def mission(self, **changes):
        return dict(dict(stamp=100., mode='inspect', traffic_active=False, gimbal_ready=True,
            motion_allowed=False, gimbal_target_deg=[0, 0]), **changes)

    def test_task_gate_requires_fresh_pose_and_inspection(self):
        self.assertIsNotNone(observation_signature(self.mission(), 100.1, .1))
        for changes in (dict(mode='patrol'), dict(traffic_active=True), dict(gimbal_ready=False),
                        dict(motion_allowed=True), dict(stamp=99.), dict(stamp=float('nan'))):
            self.assertIsNone(observation_signature(self.mission(**changes), 100.1, .1))
        self.assertIsNone(observation_signature(self.mission(), 100.1, .6))
        self.assertNotEqual(observation_signature(self.mission(), 100.1, .1),
            observation_signature(self.mission(gimbal_target_deg=[10, 0]), 100.1, .1))

    def test_explicit_fixed_camera_preview_only_relaxes_pose_gate(self):
        self.assertIsNotNone(observation_signature(self.mission(gimbal_ready=False), 100.1, .1,
            require_settled_pose=False))
        for changes in (dict(mode='patrol'), dict(traffic_active=True), dict(motion_allowed=True)):
            self.assertIsNone(observation_signature(self.mission(**changes), 100.1, .1,
                require_settled_pose=False))

    def test_exact_capture_and_newest_only_no_nearest_timestamp(self):
        join = ExactCaptureJoin()
        join.push_raw(100_000_000_000, 'image100')
        join.push_detection(100_000_000_001, detection(100.))
        self.assertIsNone(join.take(100.1, 99.))
        join.push_detection(100_000_000_000, detection(100.))
        join.push_raw(100_100_000_000, 'image100.1')
        join.push_detection(100_100_000_000, detection(100.1))
        self.assertEqual(join.take(100.2, 99.)[1], 'image100.1')
        self.assertIsNone(join.take(100.2, 99.))

    def test_bounded_raw_cache_and_age_transition(self):
        join = ExactCaptureJoin(raw_limit=2)
        for i in range(3):
            stamp = 100_000_000_000 + i*100_000_000
            join.push_raw(stamp, i)
            join.push_detection(stamp, detection(stamp/1e9))
        self.assertEqual(len(join.raw), 2)
        self.assertIsNone(join.take(100.3, 100.25))
        self.assertIsNone(join.take(101., 99.))

    def finish(self, observer, stamp, readings=None, **changes):
        kwargs = dict(current_generation=1, active=True, coordinate_frame='left')
        kwargs.update(changes)
        return observer.finish(readings if readings is not None else [reading()], detection(stamp),
            round(stamp*1e9), stamp+.5, 1, **kwargs)

    def test_low_rate_distinct_frames_confirm_without_refreshing_stamp(self):
        observer = BuildingObservation(CONFIG)
        self.assertFalse(self.finish(observer, 100.)['building']['confirmed'])
        self.assertFalse(self.finish(observer, 101.)['building']['confirmed'])
        result = self.finish(observer, 102.)
        self.assertTrue(result['building']['confirmed'])
        self.assertEqual(result['stamp'], 102.)
        self.assertEqual(result['processed_stamp'], 102.5)
        self.assertEqual(result['matched_detection']['capture_stamp'], 102.)
        self.assertFalse(result['object_building_association_verified'])

    def test_ambiguous_or_missing_sign_invalidates_confirmation(self):
        for readings in ([], [reading('1号楼'), reading('11号楼')]):
            observer = BuildingObservation(CONFIG)
            for stamp in (100., 101., 102.):
                self.finish(observer, stamp)
            self.assertFalse(self.finish(observer, 103., readings)['building']['confirmed'])
            self.assertFalse(self.finish(observer, 104.)['building']['confirmed'])

    def test_stale_result_changed_pose_wrong_geometry_rejected(self):
        for changes in (dict(active=False), dict(current_generation=2), dict(coordinate_frame='right')):
            result = self.finish(BuildingObservation(CONFIG), 100., **changes)
            self.assertFalse(result['valid'])
            self.assertIsNone(result['matched_detection'])
        observer = BuildingObservation(CONFIG)
        result = observer.finish([reading()], detection(100.), 100_000_000_000, 104., 1,
            current_generation=1, active=True, coordinate_frame='left')
        self.assertEqual(result['status'], 'ocr_result_expired')

    def test_explicit_timestamp_mismatch_rejected(self):
        result = BuildingObservation(CONFIG).finish([reading()], detection(100.1), 100_000_000_000,
            100.5, 1, current_generation=1, active=True, coordinate_frame='left')
        self.assertFalse(result['valid'])


class CorrectedObservationTests(unittest.TestCase):
    def setUp(self):
        self.camera = corrected_camera(corrected_info(), CORRECTED_FRAME, (1280, 1088))
        self.original = np.random.default_rng(7).integers(0, 256, (1088, 1280, 3), dtype=np.uint8)
        self.payload = self.original.tobytes()
        self.frame = NS(header=NS(frame_id=CORRECTED_FRAME, stamp=NS(sec=100, nanosec=5)),
                        width=1280, height=1088, step=3840, encoding='bgr8', data=self.payload)

    def test_lossless_original_pixels_are_readonly_views_without_double_correction(self):
        image = corrected_observation_image(self.frame, self.camera)
        np.testing.assert_array_equal(image, self.original)
        self.assertTrue(np.shares_memory(image, np.frombuffer(self.payload, dtype=np.uint8)))
        self.assertFalse(image.flags.writeable)
        with self.assertRaises(ValueError):
            image[0, 0, 0] = 1
        self.assertEqual((self.frame.header.stamp.sec, self.frame.header.stamp.nanosec), (100, 5))

    def test_no_geometry_raw_pixels_or_incorrect_dimensions_are_rejected(self):
        with self.assertRaises(ValueError):
            corrected_observation_image(self.frame, None)
        for key, value in [('width', 640), ('height', 544), ('step', 3841),
                           ('encoding', 'rgb8'), ('encoding', 'nv12'), ('data', self.payload[:-1])]:
            item = deepcopy(self.frame)
            setattr(item, key, value)
            with self.subTest(key=key), self.assertRaises(ValueError):
                corrected_observation_image(item, self.camera)
        item = deepcopy(self.frame)
        item.header.frame_id = 'left_camera_optical'
        with self.assertRaises(ValueError):
            corrected_observation_image(item, self.camera)

    def test_low_rate_image_pairs_in_either_arrival_order_amid_fast_detections(self):
        for image_first in (False, True):
            join = ExactCaptureJoin()
            stamp = 100_000_000_000
            if image_first:
                join.push_raw(stamp, self.frame)
            join.push_detection(stamp, detection(100.))
            # Ten newer YOLO results must not relabel the older exact image.
            for index in range(1, 11):
                later = stamp + index*30_000_000
                join.push_detection(later, detection(later/1e9, sequence=7+index))
            if not image_first:
                join.push_raw(stamp, self.frame)
            selected = join.take(100.35, 99., .5)
            self.assertEqual(selected[0], stamp)
            self.assertIs(selected[1], self.frame)
            self.assertEqual(selected[2]['sequence'], 7)
            self.assertEqual(selected[2]['capture_stamp_ns'], stamp)
            self.assertIsNone(join.take(100.36, 99., .5))


if __name__ == '__main__':
    unittest.main()
