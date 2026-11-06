"""Isolated ROS sidecar integration using synthetic images, never live cameras.

Run only with RESCUE_RUN_ROS_TEST=1 ROS_DOMAIN_ID=83 and the workspace sourced.
No hardware driver, detector model, PWM, GPIO or live-domain topic is started.
The calibration below is synthetic and is created only in a temporary folder.
"""
from contextlib import contextmanager
import copy
import json
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'Explicit isolated X5 test only')
class StereoDepthBoardTests(unittest.TestCase):
    @contextmanager
    def harness(self, enabled=True):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83',
                         'Never inject synthetic frames into the live robot domain')
        import cv2
        import numpy as np
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import Image, CameraInfo
        from rescue_interfaces.msg import Detection, DetectionArray
        from rescue_vision.node_common import IMAGE_QOS
        from rescue_vision.stereo_depth_node import StereoDepthNode

        width, height = 320, 160
        K = np.array(((240., 0, 160.), (0, 240., 80.), (0, 0, 1)))
        topics = dict(left='/test/stereo/physical_left', right='/test/stereo/physical_right')
        runtime = dict(enabled=enabled, physical_eye_mapping_verified=True,
                       synchronization_verified=True, metric_depth_verified=True,
                       verification_id='TEST_SYNTHETIC_ONLY', require_fire_mode=False,
                       physical_left_topic=topics['left'], physical_right_topic=topics['right'],
                       input_rotation_deg=0., process_fps=10., opencv_threads=2,
                       max_frame_age_s=.5, max_pair_skew_s=.012, queue_size=16,
                       matcher=dict(num_disparities=64))
        calibration = dict(calibrated=True, calibration_id='TEST_SYNTHETIC_ONLY',
                           camera_model='pinhole', translation_unit='m', image_size=[width, height],
                           K1=K.tolist(), K2=K.tolist(), D1=[0.] * 8, D2=[0.] * 8,
                           R=np.eye(3).tolist(), T=[-.06, 0., 0.])
        source = dict(width=width, height=height, distortion_model='rational_polynomial',
                      K=K.ravel().tolist(), D=[0.] * 8)
        snapshot = dict(sources={topics['left']: source, topics['right']: copy.deepcopy(source)},
                        provenance=dict(calibration_rotation_degrees=0.),
                        driver=dict(parameters=dict(rotation=0., gdc_enable='False')))
        with tempfile.TemporaryDirectory(prefix='stereo_synthetic_ros_') as temporary:
            paths = {}
            for key, document in (('runtime_config', runtime), ('stereo_calibration', calibration),
                                  ('lens_intrinsics', snapshot)):
                path = Path(temporary) / (key + '.json')
                path.write_text(json.dumps(document), encoding='utf-8')
                paths[key] = path
            args = ['--ros-args']
            for key, path in paths.items():
                args.extend(('-p', key + ':=' + str(path)))
            rclpy.init(args=args)
            observer = probe = executor = None
            try:
                observer = StereoDepthNode()
                probe = Node('stereo_depth_integration_probe')
                outputs = []
                output_sub = probe.create_subscription(DetectionArray, '/perception/depth_detections',
                                                       outputs.append, IMAGE_QOS)
                detections_pub = probe.create_publisher(DetectionArray, '/perception/detections', IMAGE_QOS)
                raw_pubs = {eye: probe.create_publisher(Image, topic, IMAGE_QOS)
                            for eye, topic in topics.items()}
                info_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
                info_pub = probe.create_publisher(CameraInfo, '/vision/camera_info', info_qos)
                executor = SingleThreadedExecutor()
                executor.add_node(observer)
                executor.add_node(probe)
                harness = SimpleNamespace(observer=observer, probe=probe, executor=executor,
                                          outputs=outputs, detections_pub=detections_pub,
                                          raw_pubs=raw_pubs, info_pub=info_pub, width=width, height=height,
                                          Image=Image, Detection=Detection, DetectionArray=DetectionArray,
                                          output_sub=output_sub, cv2=cv2, np=np, topics=topics)
                self.spin_until(harness, lambda: detections_pub.get_subscription_count() >= 1
                                and observer.publisher.get_subscription_count() >= 1, 3.)
                if enabled:
                    self.spin_until(harness, lambda: all(p.get_subscription_count() >= 1
                                                        for p in raw_pubs.values()), 3.)
                info = CameraInfo()
                info.header.frame_id = 'undistorted_left_optical'
                info.width, info.height = width, height
                corrected_K = cv2.getOptimalNewCameraMatrix(K, np.zeros(8), (width, height),
                                                             0., (width, height))[0]
                info.k = corrected_K.ravel().tolist()
                info.d = [0.] * 5
                info.r = np.eye(3).ravel().tolist()
                info.p = np.c_[corrected_K, np.zeros(3)].ravel().tolist()
                info.distortion_model = 'plumb_bob'
                harness.info = info
                info_pub.publish(info)
                self.spin_until(harness, lambda: observer.camera is not None, 3.)
                yield harness
            finally:
                if executor is not None:
                    if observer is not None:
                        executor.remove_node(observer)
                    if probe is not None:
                        executor.remove_node(probe)
                if observer is not None:
                    observer.destroy_node()
                if probe is not None:
                    probe.destroy_node()
                if executor is not None:
                    executor.shutdown()
                rclpy.shutdown()
                if observer is not None and observer.worker is not None:
                    self.assertFalse(observer.worker.is_alive(), 'Depth worker must stop cleanly')

    def spin_until(self, harness, predicate, timeout=2.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            harness.executor.spin_once(timeout_sec=.01)
        self.assertTrue(predicate(), getattr(harness.observer, 'last_status', 'timeout'))

    @staticmethod
    def spin_for(harness, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            harness.executor.spin_once(timeout_sec=.005)

    def detection(self, harness, sequence, stamp_ns=None, frame='undistorted_left_optical'):
        message = harness.DetectionArray()
        stamp_ns = stamp_ns or (harness.probe.get_clock().now().nanoseconds - 40_000_000)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        message.header.frame_id = frame
        message.sequence = sequence
        message.width, message.height = harness.width, harness.height
        message.rectified = False
        message.detector_available = True
        message.status = 'SYNTHETIC_TEST'
        for label, class_id in (('fire', 0), ('person', 2)):
            detection = harness.Detection()
            detection.class_id, detection.class_name, detection.confidence = class_id, label, .9
            detection.bbox_xyxy = [180., 65., 230., 115.]
            # A stale input range must always be reset/recomputed by the sidecar.
            detection.depth_valid, detection.depth_m = True, 999.
            detection.position_m.x = detection.position_m.y = detection.position_m.z = 999.
            detection.depth_reason = 'stale_input_depth'
            message.detections.append(detection)
        return message

    def image(self, harness, pixels, stamp_ns):
        message = harness.Image()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        message.header.frame_id = 'synthetic_raw_camera'
        message.width, message.height, message.step = harness.width, harness.height, harness.width
        message.encoding = 'mono8'
        message.data.frombytes(pixels.tobytes())
        return message

    def textured_pair(self, harness, reverse=False):
        np = harness.np
        rng = np.random.default_rng(734)
        left = rng.integers(0, 256, (harness.height, harness.width), dtype=np.uint8)
        left = harness.cv2.GaussianBlur(left, (3, 3), .5)
        right = rng.integers(0, 256, left.shape, dtype=np.uint8)
        right[:, :-16] = left[:, 16:]
        return (right, left) if reverse else (left, right)

    def send_trial(self, harness, sequence, *, missing_right=False, reverse=False,
                   skew_ns=0, detection_offset_ns=0, frame='undistorted_left_optical'):
        # Separate this exposure from the previous case's 12 ms pairing window.
        self.spin_for(harness, .04)
        stamp = harness.probe.get_clock().now().nanoseconds - 60_000_000
        left, right = self.textured_pair(harness, reverse)
        harness.raw_pubs['left'].publish(self.image(harness, left, stamp))
        if not missing_right:
            harness.raw_pubs['right'].publish(self.image(harness, right, stamp + skew_ns))
        self.spin_until(harness, lambda: stamp in harness.observer.join.frames['left'])
        if not missing_right:
            self.spin_until(harness, lambda: stamp + skew_ns in harness.observer.join.frames['right'])
        # This is the actual raw=True DDS subscription, not a fabricated ndarray
        # callback: the CDR parser keeps an immutable serialized-byte pixel view.
        view = harness.observer.join.frames['left'][stamp]
        self.assertIsInstance(view.data, memoryview)
        self.assertTrue(view.data.readonly)
        source = self.detection(harness, sequence, stamp + detection_offset_ns, frame)
        harness.detections_pub.publish(source)
        self.spin_until(harness, lambda: any(item.sequence == sequence for item in harness.outputs))
        return source, next(item for item in harness.outputs if item.sequence == sequence)

    def assert_preserved(self, source, output):
        import struct
        self.assertEqual(output.sequence, source.sequence)
        self.assertEqual(output.header, source.header)
        self.assertEqual((output.width, output.height, output.rectified, output.detector_available),
                         (source.width, source.height, source.rectified, source.detector_available))
        self.assertEqual(len(output.detections), len(source.detections))
        for original, enriched in zip(source.detections, output.detections):
            self.assertEqual((enriched.class_id, enriched.class_name),
                             (original.class_id, original.class_name))
            # ROS wire format is float32; the source Python literal is float64.
            self.assertEqual(enriched.confidence, struct.unpack('f', struct.pack('f', original.confidence))[0])
            self.assertEqual(list(enriched.bbox_xyxy), list(original.bbox_xyxy))

    def test_disabled_forwards_original_identity_without_raw_subscription(self):
        with self.harness(enabled=False) as harness:
            self.assertEqual(harness.observer.gate, 'disabled')
            self.assertIsNone(harness.observer.worker)
            self.assertTrue(all(p.get_subscription_count() == 0 for p in harness.raw_pubs.values()))
            source = self.detection(harness, 417)
            harness.detections_pub.publish(source)
            self.spin_until(harness, lambda: len(harness.outputs) == 1)
            output = harness.outputs[0]
            self.assertEqual(output.stereo_calibration_id, '')
            self.assert_preserved(source, output)
            for item in output.detections:
                self.assertFalse(item.depth_valid)
                self.assertEqual(item.depth_reason, 'disabled')
                self.assertEqual(item.depth_m, 0.)
                self.assertEqual((item.position_m.x, item.position_m.y, item.position_m.z), (0., 0., 0.))

    def test_verified_pair_measures_point_on_same_corrected_detection_ray(self):
        with self.harness() as harness:
            source, output = self.send_trial(harness, 9001)
            self.assert_preserved(source, output)
            fire, person = output.detections
            self.assertTrue(fire.depth_valid, output.status + ': ' + fire.depth_reason)
            self.assertEqual(output.stereo_calibration_id, 'TEST_SYNTHETIC_ONLY')
            self.assertAlmostEqual(fire.depth_m, .9, delta=.02)
            self.assertEqual(fire.depth_reason, 'verified_stereo_wallpaper_surface')
            K = harness.np.asarray(harness.info.k).reshape(3, 3)
            ray = harness.np.linalg.inv(K) @ harness.np.array((205., 90., 1.))
            harness.np.testing.assert_allclose([fire.position_m.x, fire.position_m.y, fire.position_m.z],
                                               ray * fire.depth_m, atol=1e-6)
            self.assertFalse(person.depth_valid, 'Only the selected fire class should gain range')
            self.assertEqual(person.depth_m, 0.)
            self.assertGreaterEqual(harness.observer.counts['computed'], 1)

    def test_missing_wrong_eye_and_nonmatching_exposures_never_gain_range(self):
        with self.harness() as harness:
            cases = [(dict(missing_right=True), 'matching_right_frame_missing'),
                     (dict(detection_offset_ns=1), 'exact_left_frame_missing'),
                     (dict(skew_ns=30_000_000), 'matching_right_frame_missing'),
                     (dict(frame='undistorted_right_optical'), 'depth_rejected'),
                     (dict(reverse=True), None)]
            for index, (kwargs, expected) in enumerate(cases):
                with self.subTest(kwargs=kwargs):
                    source, output = self.send_trial(harness, 9100 + index, **kwargs)
                    self.assert_preserved(source, output)
                    self.assertTrue(all(not d.depth_valid for d in output.detections), output.status)
                    self.assertEqual(output.stereo_calibration_id, '')
                    if expected:
                        self.assertIn(expected, output.detections[0].depth_reason)

    def test_changed_camera_geometry_and_stale_detection_are_rejected(self):
        with self.harness() as harness:
            bad_info = copy.deepcopy(harness.info)
            # Both K and P change consistently: it is a valid pinhole model,
            # but it does not describe this verified preview lens mapping.
            bad_info.k[0] += 10.
            bad_info.p[0] += 10.
            harness.info_pub.publish(bad_info)
            self.spin_until(harness, lambda: harness.observer.camera is None)
            _, output = self.send_trial(harness, 9200)
            self.assertFalse(output.detections[0].depth_valid)
            self.assertEqual(output.detections[0].depth_reason, 'camera_geometry_unavailable')
            before = len(harness.outputs)
            stale = self.detection(harness, 9201,
                                   harness.probe.get_clock().now().nanoseconds - 700_000_000)
            harness.detections_pub.publish(stale)
            self.spin_for(harness, .1)
            self.assertEqual(len(harness.outputs), before, 'Expired frame must not refresh aiming state')


if __name__ == '__main__':
    unittest.main(verbosity=2)
