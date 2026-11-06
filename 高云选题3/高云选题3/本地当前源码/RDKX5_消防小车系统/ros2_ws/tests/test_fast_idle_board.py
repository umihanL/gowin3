"""Idle frames preserve ROS observations without decoding unused image pixels.

Run only on isolated domain 83; synthetic inputs never enter the live system.
"""
import json
import os
from pathlib import Path
import unittest
from unittest.mock import patch


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'Explicit isolated X5 test only')
class FastIdleTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        import rclpy
        from rescue_vision.fast_mipi_perception_node import FastMipiPerception
        root = Path(__file__).resolve().parents[1]
        rclpy.init(args=['--ros-args', '-p', 'detector_enabled:=false',
            '-p', 'mission_gating:=true', '-p', 'lens_intrinsics:=' +
            str(root / 'config/gs130w_intrinsics_20261010.json')])
        self.node = FastMipiPerception()

        class Publisher:
            subscribers = 0
            def __init__(self):
                self.messages = []
            def get_subscription_count(self):
                return self.subscribers
            def publish(self, message):
                self.messages.append(message)

        class Detector:
            prepares = calls = 0
            last_timings = {}
            def prepare(self, image):
                self.prepares += 1
                return image
            def predict_prepared(self, image):
                self.calls += 1
                return []

        self.node.preview_pub = Publisher()
        self.node.detection_pub = Publisher()
        self.node.geometry_pub = Publisher()
        self.node.detector = Detector()

    def tearDown(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()

    def source(self):
        from sensor_msgs.msg import Image
        image = Image()
        image.header.stamp = self.node.get_clock().now().to_msg()
        image.header.frame_id = 'idle_test_raw'
        image.width, image.height, image.step = 1280, 1088, 1280
        image.encoding = 'mono8'
        image.data.frombytes(bytes([83]) * (1280 * 1088))
        return image

    def mission(self, mode):
        from std_msgs.msg import String
        self.node.mission_state(String(data=json.dumps(dict(
            stamp=self.node.get_clock().now().nanoseconds / 1e9,
            mode=mode, traffic_active=False))))

    def test_idle_publishes_fresh_disabled_observation_with_original_geometry(self):
        self.mission('standby')
        with patch.object(self.node.lens, 'correct', wraps=self.node.lens.correct) as correct:
            first = self.node.prepare(self.source(), 1)
            self.assertIsNotNone(first.image, 'The first frame must validate actual geometry')
            self.node.infer(first)
            self.assertEqual(correct.call_count, 1)
            source = self.source()
            with patch('rescue_vision.fast_mipi_perception_node.image_to_bgr',
                       side_effect=AssertionError('Idle pixels must not be decoded')):
                second = self.node.prepare(source, 2)
                self.node.infer(second)
            self.assertIsNone(second.image)
            self.assertEqual(correct.call_count, 1)
        self.assertEqual(second.lens_metadata, first.lens_metadata)
        original, idle = self.node.detection_pub.messages
        self.assertEqual(idle.header.stamp, source.header.stamp)
        self.assertEqual(idle.sequence, 2)
        self.assertFalse(idle.detector_available)
        self.assertEqual(idle.detections, [])
        self.assertEqual(idle.status, original.status)
        self.assertEqual((idle.width, idle.height), (original.width, original.height))
        self.assertEqual(idle.header.frame_id, original.header.frame_id)
        self.assertFalse(idle.rectified)
        self.assertEqual(len(self.node.geometry_pub.messages), 1)
        self.assertEqual(list(self.node.geometry_pub.messages[0].k),
                         first.lens_metadata['camera_matrix'])
        self.assertFalse(self.node.performance['pixel_processing'])
        self.assertFalse(self.node.performance['detector_active'])
        self.assertEqual(self.node.performance['stage_ms']['decode'], 0)
        self.assertEqual(self.node.performance['stage_ms']['lens'], 0)
        self.assertEqual(self.node.idle_geometry_count, 1)

    def test_returning_yolo_demand_never_uses_idle_pixels(self):
        from rclpy.parameter import Parameter
        self.node.process(self.source(), 1)
        idle = self.node.prepare(self.source(), 2)
        self.assertIsNone(idle.image)
        # This enables inference even for pre-transition timestamps. It is a
        # stronger race than an ordinary mission switch with its time fence.
        self.node.set_parameters([Parameter('mission_gating', Parameter.Type.BOOL, False)])
        self.node.infer(idle)
        self.assertEqual(self.node.detector.calls, 0)
        self.assertFalse(self.node.detection_pub.messages[-1].detector_available)
        self.node.process(self.source(), 3)
        self.assertEqual(self.node.detector.calls, 1)
        self.assertTrue(self.node.performance['pixel_processing'])
        self.assertTrue(self.node.detection_pub.messages[-1].detector_available)

    def test_returning_preview_uses_next_real_frame_only(self):
        self.node.process(self.source(), 1)
        idle = self.node.prepare(self.source(), 2)
        self.node.preview_pub.subscribers = 1
        with patch.object(self.node.previews, 'put') as put:
            self.node.infer(idle)
            put.assert_not_called()
            self.node.process(self.source(), 3)
            put.assert_called_once()
            header, image = put.call_args.args[0]
            self.assertEqual(image.shape, (1088, 1280, 3))
            self.assertEqual(header.stamp, self.node.detection_pub.messages[-1].header.stamp)
            self.assertFalse(image.flags.writeable)

    def test_geometry_error_requires_real_processing_and_expired_results_stay_rejected(self):
        self.node.process(self.source(), 1)
        self.node.warn('injected geometry error', 'geometry')
        with patch.object(self.node.lens, 'correct', wraps=self.node.lens.correct) as correct:
            recovery = self.node.prepare(self.source(), 2)
            self.assertIsNotNone(recovery.image)
            correct.assert_called_once()
        self.node.clear_error('geometry')
        idle = self.node.prepare(self.source(), 3)
        self.assertIsNone(idle.image)
        before = len(self.node.detection_pub.messages)
        with patch('rescue_vision.fast_mipi_perception_node.age', return_value=1.01):
            with self.assertRaisesRegex(ValueError, 'one-second capture-age'):
                self.node.infer(idle)
        self.assertEqual(len(self.node.detection_pub.messages), before)


if __name__ == '__main__':
    unittest.main()
