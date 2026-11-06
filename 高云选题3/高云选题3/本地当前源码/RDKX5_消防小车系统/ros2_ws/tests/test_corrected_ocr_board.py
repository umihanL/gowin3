"""Opt-in isolated ROS checks for reuse of the detector's lossless image."""
from concurrent.futures import Future
from copy import deepcopy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'Explicit isolated X5 test only')
class CorrectedOCRBoardTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        import rclpy
        from rescue_vision.mipi_building_ocr_node import MipiBuildingOCR
        from test_mipi_building_ocr import CONFIG

        class Reader:
            def __init__(self, *args, **kwargs):
                self.last_timings, self.images = {}, []
            def predict(self, image):
                self.images.append(image)
                return []

        class Recorder:
            def __init__(self): self.messages = []
            def publish(self, message): self.messages.append(json.loads(message.data))

        self.tmp = tempfile.TemporaryDirectory()
        config = Path(self.tmp.name) / 'buildings.json'
        config.write_text(json.dumps(CONFIG), encoding='utf-8')
        rclpy.init(args=['--ros-args', '-p', 'corrected_input:=true',
                        '-p', 'buildings_config:=' + str(config)])
        # Even with lens_enabled's default True, the corrected path must never
        # instantiate a second geometry processor or convert the source pixels.
        self.reader_patch = patch('rescue_vision.mipi_building_ocr_node.OCRReader', Reader)
        self.lens_patch = patch('rescue_vision.mipi_building_ocr_node.LensCorrector.from_json',
                                side_effect=AssertionError('second lens correction'))
        self.reader_patch.start()
        self.lens_patch.start()
        self.node = MipiBuildingOCR()
        self.node.output = Recorder()
        self.refresh_mission()

    def tearDown(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()
        self.reader_patch.stop()
        self.lens_patch.stop()
        self.tmp.cleanup()

    def refresh_mission(self):
        from std_msgs.msg import String
        self.node.on_mission(String(data=json.dumps(dict(stamp=self.node.now(), mode='inspect',
            traffic_active=False, gimbal_ready=True, motion_allowed=False, gimbal_target_deg=[0, 0]))))

    def enable_geometry(self):
        from test_mipi_building_ocr import corrected_info
        self.node.on_camera_info(corrected_info())

    def pair(self, *, raw_frame=False, with_detection=False):
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Image
        from rescue_interfaces.msg import DetectionArray
        frame = Image()
        frame.header.stamp = self.node.get_clock().now().to_msg()
        frame.header.frame_id = 'left_camera_optical' if raw_frame else 'undistorted_left_optical'
        frame.width, frame.height, frame.step, frame.encoding = 1280, 1088, 3840, 'bgr8'
        frame.data.frombytes(bytes([17, 43, 211]) * (1280*1088))
        detections = DetectionArray()
        detections.header = deepcopy(frame.header)
        detections.width, detections.height = 1280, 1088
        detections.sequence, detections.detector_available = 9, True
        if with_detection:
            import numpy as np
            from rescue_interfaces.msg import Detection
            item = Detection()
            item.class_name, item.confidence = 'fire', .95
            item.bbox_xyxy = np.array([10.25, 20.5, 110.75, 140.125], dtype=np.float32)
            detections.detections = [item]
        self.node.on_detection(detections)
        self.node.on_raw(serialize_message(frame))
        return frame, detections

    def test_nonempty_ros_float32_detections_remain_json_serializable(self):
        """A fixed ROS float32[4] array must become Python numbers, not np scalars."""
        self.enable_geometry()
        _, detection = self.pair(with_detection=True)
        future = Future()
        with patch.object(self.node.worker, 'submit', return_value=future):
            self.node.tick()
        future.set_result(([dict(text='1号楼', confidence=.98,
            quad=[[10., 20.], [100., 20.], [100., 45.], [10., 45.]])], 1.))
        self.node.tick()
        result = next(m for m in reversed(self.node.output.messages) if m.get('valid'))
        bbox = result['matched_detection']['detections'][0]['bbox']
        self.assertEqual(bbox, [10.25, 20.5, 110.75, 140.125])
        self.assertTrue(all(type(value) is float for value in bbox))
        self.assertEqual(result['sequence'], detection.sequence)
        self.assertEqual(result['readings'][0]['text'], '1号楼')
        self.assertEqual(self.node.counts['completed'], 1)
        self.assertEqual(self.node.counts['invalid'], 0)
        json.dumps(result, allow_nan=False)

    def test_dialogue_cancels_inflight_generation_and_resumes_fresh_only(self):
        import time
        from std_msgs.msg import String
        self.enable_geometry()
        self.node.on_mission(String(data=json.dumps(dict(stamp=self.node.now(), mode='inspect',
            traffic_active=False, gimbal_ready=True, motion_allowed=False,
            auto_fire_active=False, pump_armed=False, gimbal_target_deg=[0, 0]))))
        self.pair()
        future = Future()
        with patch.object(self.node.worker, 'submit', return_value=future):
            self.node.tick()
        generation = self.node.generation
        self.node.on_dialogue(String(data=json.dumps(dict(stamp=self.node.now(),
            busy=True, state='thinking'))))
        self.assertTrue(self.node.yielding_to_dialogue)
        self.assertGreater(self.node.generation, generation)
        self.assertIsNone(self.node.raw_subscription)
        future.set_result(([], 1.))
        self.node.tick()
        self.assertFalse(any(m.get('valid') for m in self.node.output.messages))
        self.assertEqual(self.node.output.messages[-1]['status'], 'yielding_to_dialogue')
        self.assertIsNone(self.node.pending)
        self.assertFalse(self.node.join.raw)
        self.node.dialogue_received = time.monotonic()-3.
        self.node.next_allowed = 0.
        self.node.refresh()
        self.assertFalse(self.node.yielding_to_dialogue)
        self.assertIsNotNone(self.node.raw_subscription)
        self.assertFalse(self.node.join.raw)

    def test_current_geometry_required_then_original_pixels_reach_ocr_once(self):
        self.assertIsNone(self.node.raw_subscription)
        self.enable_geometry()
        self.assertIsNotNone(self.node.raw_subscription)
        self.assertEqual(self.node.input_topic, '/vision/observation/image')
        frame, _ = self.pair()
        with patch('rescue_vision.mipi_building_ocr_node.image_to_bgr',
                   side_effect=AssertionError('unexpected color conversion')):
            self.node.tick()
            self.assertIsNotNone(self.node.pending)
            self.node.pending[0].result(timeout=2)
            self.node.tick()
        image = self.node.reader.images[0]
        self.assertEqual(image.shape, (1088, 1280, 3))
        self.assertEqual(image[400, 500].tolist(), [17, 43, 211])
        self.assertFalse(image.flags.writeable)
        result = next(m for m in reversed(self.node.output.messages) if m.get('valid'))
        expected = frame.header.stamp.sec*1_000_000_000 + frame.header.stamp.nanosec
        self.assertEqual(result['capture_stamp_ns'], expected)
        self.assertEqual(result['matched_detection']['capture_stamp_ns'], expected)
        self.assertEqual(result['sequence'], 9)
        self.assertTrue(result['corrected_input'])
        self.assertEqual(self.node.counts['raw_frames'], 0)
        self.assertEqual(self.node.counts['corrected_frames'], 1)

    def test_raw_label_is_rejected_and_geometry_change_cancels_old_result(self):
        from test_mipi_building_ocr import corrected_info, detection
        self.enable_geometry()
        self.pair(raw_frame=True)
        self.assertFalse(self.node.join.raw)
        self.assertEqual(self.node.counts['invalid'], 1)
        previous = self.node.generation
        stamp = self.node.get_clock().now().nanoseconds
        evidence = detection(stamp/1e9)
        evidence['capture_stamp_ns'] = stamp
        evidence['coordinate_frame'] = self.node.coordinate_frame
        done = Future()
        done.set_result(([], 1.))
        self.node.pending = done, stamp, evidence, previous
        changed = corrected_info()
        changed.k[0] += 1
        changed.p[0] += 1
        self.node.on_camera_info(changed)
        self.assertGreater(self.node.generation, previous)
        self.node.tick()
        self.assertFalse(self.node.output.messages[-1]['valid'])
        self.assertEqual(self.node.output.messages[-1]['status'], 'task_or_pose_changed')

    def test_reader_released_during_inference_and_reacquires_fresh_pair(self):
        """Expensive DDS image delivery is not needed during an in-flight OCR.

        The next result must still originate from a fresh exact YOLO capture;
        disabling the reader must not change task generation or allow reuse.
        """
        self.enable_geometry()
        first_frame, _ = self.pair()
        generation = self.node.generation
        future = Future()
        with patch.object(self.node.worker, 'submit', return_value=future):
            self.node.tick()
        self.assertIsNone(self.node.raw_subscription)
        self.assertFalse(self.node.join.raw)
        self.assertFalse(self.node.join.detections)
        self.assertEqual(self.node.generation, generation)
        # Messages already queued before removing the reader cannot repopulate
        # the join or accidentally submit work while this sample is in flight.
        self.pair()
        self.assertFalse(self.node.join.raw)
        self.assertFalse(self.node.join.detections)
        self.refresh_mission()
        self.node.tick()
        self.assertIsNone(self.node.raw_subscription)
        future.set_result(([], 1.))
        self.node.tick()
        self.assertIsNone(self.node.raw_subscription, 'the unchanged OCR interval still applies')
        self.node.next_allowed = 0
        self.refresh_mission()
        self.node.tick()
        self.assertIsNotNone(self.node.raw_subscription)
        second_frame, _ = self.pair()
        second = Future()
        with patch.object(self.node.worker, 'submit', return_value=second):
            self.node.tick()
        second_stamp = second_frame.header.stamp.sec*1_000_000_000 + second_frame.header.stamp.nanosec
        first_stamp = first_frame.header.stamp.sec*1_000_000_000 + first_frame.header.stamp.nanosec
        self.assertGreater(second_stamp, first_stamp)
        self.assertEqual(self.node.pending[1], second_stamp)
        self.assertEqual(self.node.generation, generation)
        self.assertIsNone(self.node.raw_subscription)
        second.set_result(([], 1.))

    def test_task_invalidated_while_inflight_never_reopens_reader(self):
        from std_msgs.msg import String
        self.enable_geometry()
        self.pair()
        future = Future()
        with patch.object(self.node.worker, 'submit', return_value=future):
            self.node.tick()
        self.node.on_mission(String(data=json.dumps(dict(stamp=self.node.now(), mode='standby',
            traffic_active=False, gimbal_ready=True, motion_allowed=False))))
        future.set_result(([], 1.))
        self.node.next_allowed = 0
        self.node.tick()
        self.assertIsNone(self.node.raw_subscription)
        self.assertIsNone(self.node.signature)
        self.assertFalse(self.node.output.messages[-1]['valid'])


if __name__ == '__main__':
    unittest.main()
