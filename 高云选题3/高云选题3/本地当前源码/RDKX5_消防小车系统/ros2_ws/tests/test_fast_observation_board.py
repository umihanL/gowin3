"""Lossless OCR reuse, exact frame identity and bounded on-demand transport."""
import os
import unittest
from unittest.mock import patch

import test_fast_idle_board as idle_fixture


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'Explicit isolated X5 test only')
class FastObservationTests(unittest.TestCase):
    source = idle_fixture.FastIdleTests.source
    mission = idle_fixture.FastIdleTests.mission
    tearDown = idle_fixture.FastIdleTests.tearDown

    def setUp(self):
        idle_fixture.FastIdleTests.setUp(self)
        from rclpy.parameter import Parameter
        self.node.set_parameters([Parameter('mission_gating', Parameter.Type.BOOL, False)])

        class Publisher:
            subscribers = 1
            def __init__(self):
                self.messages = []
            def get_subscription_count(self):
                return self.subscribers
            def publish(self, message):
                self.messages.append(message)

        self.node.observation_pub = Publisher()

    def test_full_resolution_pixels_and_headers_are_identical_to_detection_frame(self):
        source = self.source()
        prepared = self.node.prepare(source, 29)
        events = []
        info_publish = self.node.geometry_pub.publish
        detection_publish = self.node.detection_pub.publish
        observation_publish = self.node.observation_pub.publish
        with patch.object(self.node.geometry_pub, 'publish',
                          side_effect=lambda msg: (events.append('info'), info_publish(msg))), \
             patch.object(self.node.detection_pub, 'publish',
                          side_effect=lambda msg: (events.append('detections'), detection_publish(msg))), \
             patch.object(self.node.observation_pub, 'publish',
                          side_effect=lambda msg: (events.append('pixels'), observation_publish(msg))):
            self.node.infer(prepared)
        self.assertEqual(events, ['info', 'detections', 'pixels'])
        picture = self.node.observation_pub.messages[-1]
        detection = self.node.detection_pub.messages[-1]
        self.assertEqual(picture.header, detection.header)
        self.assertEqual(picture.header.stamp, source.header.stamp)
        self.assertEqual(picture.header.frame_id, 'undistorted_left_optical')
        self.assertEqual((picture.width, picture.height), (1280, 1088))
        self.assertEqual(picture.encoding, 'bgr8')
        self.assertEqual(picture.step, 1280 * 3)
        self.assertEqual(bytes(picture.data), prepared.image.tobytes())
        self.assertEqual(self.node.observation_count, 1)
        self.assertFalse(prepared.image.flags.writeable)

    def test_no_subscribers_avoids_image_allocation(self):
        self.node.observation_pub.subscribers = 0
        prepared = self.node.prepare(self.source(), 1)
        with patch('rescue_vision.fast_mipi_perception_node.Image',
                   side_effect=AssertionError('No consumer must mean no image copy')):
            self.node.infer(prepared)
        self.assertEqual(self.node.observation_pub.messages, [])
        self.assertEqual(self.node.observation_count, 0)

    def test_rate_limit_keeps_inference_running_and_restores_without_stale_frames(self):
        self.assertEqual(self.node.observation_fps, 2.0)
        for index, now in enumerate((10.0, 10.1, 10.49, 10.51), start=1):
            prepared = self.node.prepare(self.source(), index)
            with patch('rescue_vision.fast_mipi_perception_node.time.monotonic', return_value=now):
                self.node.infer(prepared)
        self.assertEqual(self.node.detector.calls, 4)
        self.assertEqual(len(self.node.observation_pub.messages), 2)
        received_stamps = [message.header.stamp for message in self.node.observation_pub.messages]
        detections = self.node.detection_pub.messages
        self.assertEqual(received_stamps, [detections[0].header.stamp, detections[3].header.stamp])

    def test_inactive_mode_has_no_observation_even_with_subscribers(self):
        from rclpy.parameter import Parameter
        self.node.set_parameters([Parameter('mission_gating', Parameter.Type.BOOL, True)])
        self.mission('standby')
        self.node.process(self.source(), 1)
        self.node.process(self.source(), 2)
        self.assertEqual(self.node.observation_pub.messages, [])
        self.assertEqual(self.node.idle_geometry_count, 1)
        self.assertEqual(self.node.detector.calls, 0)

    def test_gate_revocation_after_detection_and_expired_observation_are_rejected(self):
        prepared = self.node.prepare(self.source(), 1)
        self.node.observation_pub.subscribers = 0
        self.node.infer(prepared)
        detection = self.node.detection_pub.messages[-1]
        self.assertTrue(detection.detector_available)
        self.node.observation_pub.subscribers = 1
        with patch.object(self.node, 'yolo_allowed', return_value=False):
            self.node.publish_observation(prepared, detection)
        with patch('rescue_vision.fast_mipi_perception_node.age', return_value=1.01):
            self.node.publish_observation(prepared, detection)
        self.assertEqual(self.node.observation_pub.messages, [])
        self.assertEqual(self.node.observation_count, 0)


if __name__ == '__main__':
    unittest.main()
