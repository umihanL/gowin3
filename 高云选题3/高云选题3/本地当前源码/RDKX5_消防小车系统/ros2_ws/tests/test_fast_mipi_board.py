"""Board-only ROS integration, on isolated domain 83 with synthetic frames.

Does not open cameras or publish into the live robot domain. Run explicitly:
RESCUE_RUN_ROS_TEST=1 ROS_DOMAIN_ID=83 python -m unittest discover -s tests -p test_fast_mipi_board.py
"""
import os
import hashlib
import time
import threading
import unittest
from pathlib import Path


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'Explicit isolated X5 test only')
class FastPipelineTests(unittest.TestCase):
    def test_success_in_one_stage_cannot_clear_other_stage_failures(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        import rclpy
        from rescue_vision.fast_mipi_perception_node import FastMipiPerception
        rclpy.init(args=['--ros-args', '-p', 'detector_enabled:=false',
                        '-p', 'lens_enabled:=false'])
        observer = FastMipiPerception()
        try:
            for stage in ('capture', 'geometry', 'preview', 'inference'):
                observer.warn('injected test failure', stage)
            observer.clear_error('inference')
            self.assertEqual(observer.stage_errors['inference'], '')
            for stage in ('capture', 'geometry', 'preview'):
                self.assertIn(stage + ': injected test failure', observer.last_error)
            observer.clear_error('capture')
            observer.clear_error('geometry')
            self.assertEqual(observer.last_error, 'preview: injected test failure')
            observer.clear_error('preview')
            self.assertEqual(observer.last_error, '')
        finally:
            observer.destroy_node()
            rclpy.shutdown()
        self.assertTrue(all(not worker.is_alive() for worker in observer.workers))

    def test_pipelined_pixels_pairing_latest_only_and_shutdown(self):
        self.check_pipeline(True)

    def test_sequential_reference_pixels_pairing_and_shutdown(self):
        self.check_pipeline(False)

    def check_pipeline(self, pipeline_enabled):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83', 'Never inject synthetic detections into live ROS')
        import cv2
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import Image, CompressedImage, CameraInfo
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from rescue_interfaces.msg import DetectionArray
        from rescue_vision.fast_mipi_perception_node import FastMipiPerception
        from rescue_vision.lens_correction import LensCorrector
        from rescue_vision.mipi_image import image_to_bgr
        from rescue_vision.node_common import IMAGE_QOS
        root = Path(__file__).resolve().parents[1]
        rclpy.init(args=['--ros-args', '-p', 'detector_enabled:=false', '-p',
            'lens_intrinsics:=' + str(root/'config/gs130w_intrinsics_20261010.json'),
            '-p', 'pipeline_enabled:=' + str(pipeline_enabled).lower(),
            '-p', 'remap_backend:=' + os.environ.get('RESCUE_TEST_REMAP_BACKEND', 'opencv')])
        observer = FastMipiPerception()

        # Independent, original sequential pixel path; no hardware/model needed.
        reference_lens = LensCorrector.from_json(root/'config/gs130w_intrinsics_20261010.json',
            role_to_source={'left': '/image_right_raw', 'right': '/image_left_raw'}, alpha=0.0)
        sources, expected_jpegs, expected_pixels = [], {}, {}
        for variant in range(3):
            image = np.zeros((1088, 1280, 3), np.uint8)
            image[:, :, 0] = np.arange(1280, dtype=np.uint16) % 256
            image[200:600, 100 + variant*100:400 + variant*100] = (40, 160, 220)
            image[600:1000, :, 1] = 40 + variant*80
            i420 = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape(-1)
            count = 1280*1088
            nv12 = np.empty(count*3//2, dtype=np.uint8)
            nv12[:count] = i420[:count]
            nv12[count::2] = i420[count:count+count//4]
            nv12[count+1::2] = i420[count+count//4:]
            message = Image()
            message.width, message.height, message.step = 1280, 1088, 1280
            message.encoding = 'nv12'
            message.data.frombytes(nv12.tobytes())
            sources.append(message)
            raw = image_to_bgr(message.data, 1280, 1088, 1280, 'nv12')
            expected = reference_lens.correct(raw, role='left', output_size=(1280, 1088),
                input_metadata={'rotation': 90.0, 'source_topic': '/image_right_raw'}).image
            expected_pixels[hashlib.sha256(expected).digest()] = variant
            ok, encoded = cv2.imencode('.jpg', expected, [cv2.IMWRITE_JPEG_QUALITY, 90])
            self.assertTrue(ok)
            expected_jpegs[variant] = encoded.tobytes()

        inference_blocked = threading.Event()
        backlog_ready = threading.Event()
        prepared_while_blocked = []
        original_prepared_put = observer.prepared.put

        def record_prepared_put(frame):
            result = original_prepared_put(frame)
            # Signal only after the slot contains two newer versions. A signal
            # from prepare() itself could race its subsequent put().
            if inference_blocked.is_set():
                prepared_while_blocked.append(frame.sequence)
                if len(prepared_while_blocked) >= 2:
                    backlog_ready.set()
            return result

        observer.prepared.put = record_prepared_put

        class ProbeDetector:
            last_timings = {}
            preprocess_backend = 'geometry_test_stub'
            backlog_attempted = False
            def prepare(self, image):
                self.shape = image.shape
                self.writeable = image.flags.writeable
                # Missing hash rejects even one changed pixel.
                variant = expected_pixels[hashlib.sha256(image).digest()]
                return variant
            def predict_prepared(self, variant):
                if pipeline_enabled and not self.backlog_attempted:
                    self.backlog_attempted = True
                    inference_blocked.set()
                    backlog_ready.wait(timeout=5.)
                    inference_blocked.clear()
                else:
                    time.sleep(.12)
                return [dict(class_id=0, class_name='fire', confidence=.9,
                             bbox_xyxy=[100 + variant, 200, 400, 600])]
            def predict(self, image):
                return self.predict_prepared(self.prepare(image))

        observer.detector = ProbeDetector()
        probe = Node('fast_pipeline_probe')
        received, pictures, sent = {}, {}, {}
        geometry = []
        probe.create_subscription(CameraInfo, '/vision/camera_info', geometry.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        key = lambda message: (message.header.stamp.sec, message.header.stamp.nanosec, message.header.frame_id)
        probe.create_subscription(DetectionArray, '/perception/detections',
            lambda msg: received.__setitem__(key(msg), msg), IMAGE_QOS)
        probe.create_subscription(CompressedImage, '/vision/preview/compressed',
            lambda msg: pictures.__setitem__(key(msg), msg), IMAGE_QOS)
        publisher = probe.create_publisher(Image, '/image_right_raw', IMAGE_QOS)
        def send():
            variant = len(sent) % len(sources)
            message = sources[variant]
            message.header.stamp = probe.get_clock().now().to_msg()
            message.header.frame_id = 'test_source'
            sent[key(message)[:2]] = variant
            publisher.publish(message)
        probe.create_timer(1/30, send)
        executor = SingleThreadedExecutor()
        executor.add_node(observer)
        executor.add_node(probe)
        try:
            deadline = time.monotonic() + 15
            matching = set()
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.1)
                matching = received.keys() & pictures.keys()
                if len(matching) >= 5 and (not pipeline_enabled or observer.dropped_prepared > 0):
                    break
            self.assertGreaterEqual(len(matching), 5, observer.last_error)
            self.assertEqual(observer.detector.shape, (1088, 1280, 3))
            self.assertFalse(observer.detector.writeable)
            if os.environ.get('RESCUE_TEST_REMAP_BACKEND') == 'native':
                self.assertEqual(observer.lens.last_remap_backend, 'native_fixed_bilinear')
            self.assertEqual(observer.pipeline_enabled, pipeline_enabled)
            self.assertTrue(geometry, 'Aiming needs corrected-image intrinsics')
            info = geometry[-1]
            self.assertEqual(info.header.frame_id, 'undistorted_left_optical')
            self.assertEqual((info.width, info.height), (1280, 1088))
            np.testing.assert_array_equal(info.d, [0.]*5)
            expected_k = reference_lens.correct(raw, role='left').camera_matrix
            np.testing.assert_array_equal(np.array(info.k).reshape(3,3), expected_k)
            if pipeline_enabled:
                self.assertTrue(backlog_ready.is_set(), 'Synthetic overload was not established')
                self.assertGreater(observer.dropped_prepared, 0, 'Slow inference must skip prepared frames')
            self.assertGreaterEqual(observer.performance['queue_wait_ms'], 0)
            self.assertAlmostEqual(observer.performance['processing_latency_ms'],
                observer.performance['stage_ms']['total'] + observer.performance['queue_wait_ms'], places=1)
            sequences = []
            for identifier in matching:
                detection, frame = received[identifier], pictures[identifier]
                variant = sent[identifier[:2]]
                sequences.append(detection.sequence)
                self.assertEqual(identifier[2], 'undistorted_left_optical')
                self.assertEqual((detection.width, detection.height), (1280, 1088))
                decoded = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(decoded.shape, (1088, 1280, 3))
                self.assertEqual(bytes(frame.data), expected_jpegs[variant],
                                 'JPEG must be byte-identical to original sequential full-resolution geometry')
                self.assertEqual(list(detection.detections[0].bbox_xyxy), [100 + variant, 200, 400, 600])
                self.assertFalse(detection.rectified)
                self.assertFalse(detection.detections[0].depth_valid)
            self.assertEqual(len(set(sequences)), len(sequences))
            chronological = [received[item].sequence for item in sorted(matching)]
            self.assertEqual(chronological, sorted(chronological))
        finally:
            backlog_ready.set()
            executor.remove_node(observer)
            executor.remove_node(probe)
            observer.destroy_node()
            probe.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
        self.assertTrue(all(not worker.is_alive() for worker in observer.workers))


if __name__ == '__main__':
    unittest.main()
