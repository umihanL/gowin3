"""ROS node callback regression: quiet idle heartbeats, immediate loss messages.

Isolated domain 83 only. Publishers are replaced with in-process recorders;
there are no camera, UART, PWM, GPIO or actuator nodes.
"""
import json
import os
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'explicit isolated ROS test only')
class FireEventRateBoardTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        import rclpy
        from rescue_vision.fire_target_node import FireTargets
        from rescue_vision.fire_aim_node import FireAim
        from test_fire_aim_geometry import camera_info
        self.elapsed = 0.
        self.timer = patch('rescue_vision.fire_aim_node.time.monotonic', side_effect=lambda: 100.+self.elapsed)
        self.timer.start(); self.addCleanup(self.timer.stop)
        rclpy.init(args=['--ros-args', '-p', 'require_selection:=true'])
        self.targets = FireTargets(); self.aim = FireAim()
        self.targets.get_clock = self.aim.get_clock = self.clock
        self.aim.camera_info = camera_info()
        self.target_messages, self.selection_messages, self.proposals = [], [], []
        test = self
        class Capture:
            def __init__(self, output, callback=None): self.output, self.callback = output, callback
            def publish(self, message):
                self.output.append((test.elapsed, json.loads(message.data)))
                if self.callback: self.callback(message)
        self.targets.targets_pub = Capture(self.target_messages)
        self.targets.selection_pub = Capture(self.selection_messages, self.aim.selection)
        self.targets.ack_pub = Capture([])
        self.aim.publisher = Capture(self.proposals)
        self.mode_name = 'idle'
        self.sequence = 0

    def tearDown(self):
        import rclpy
        self.aim.destroy_node(); self.targets.destroy_node(); rclpy.shutdown()

    def clock(self):
        from builtin_interfaces.msg import Time
        ns = 1_000_000_000_000 + round(self.elapsed*1e9)
        return NS(now=lambda: NS(nanoseconds=ns,
            to_msg=lambda: Time(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)))

    @staticmethod
    def message(value):
        from std_msgs.msg import String
        return String(data=json.dumps(value))

    def mode(self, value=None):
        if value is not None: self.mode_name = value
        state = self.message(dict(mode=self.mode_name, traffic_active=False, gimbal_ready=True,
            gimbal_target_deg=[90.,90.], stamp=self.clock().now().nanoseconds/1e9))
        self.aim.mode(state); self.targets.mode(state)

    def frame(self, available=False, fire=False):
        from rescue_interfaces.msg import Detection, DetectionArray
        self.sequence += 1
        value = DetectionArray()
        value.header.stamp = self.clock().now().to_msg()
        value.header.frame_id = 'undistorted_left_optical'
        value.sequence = self.sequence; value.width = 1280; value.height = 1088
        value.detector_available = available
        if fire:
            item = Detection(); item.class_id = 0; item.confidence = .95
            item.bbox_xyxy = [620.,524.,660.,564.]
            value.detections = [item]
        return value

    def feed(self, available=False, fire=False):
        frame = self.frame(available, fire)
        self.targets.receive(frame); self.aim.receive(frame)
        self.targets.tick(); self.aim.watchdog()

    def choose(self):
        self.targets.select(self.message(dict(command_id='select_1', issued_at=self.clock().now().nanoseconds/1e9,
            ttl_s=2., selector='only')))

    def test_idle_callback_flood_emits_only_five_hz_invalid_heartbeats(self):
        for step in range(1000):
            self.elapsed = step/1000
            self.mode(); self.feed()
        for output in (self.target_messages, self.selection_messages, self.proposals):
            self.assertLessEqual(len(output), 6, 'Callback rate must not amplify unchanged idle state')
            self.assertGreaterEqual(len(output), 5, 'Invalid state still needs a 5 Hz heartbeat')
            self.assertTrue(all(document['valid'] is False for _, document in output))
        before = (len(self.target_messages), len(self.selection_messages), len(self.proposals))
        self.elapsed = .9991; self.mode('patrol')
        self.assertEqual((len(self.target_messages), len(self.selection_messages), len(self.proposals)),
                         tuple(count+1 for count in before), 'Real mode transitions publish immediately')

    def test_valid_frames_not_throttled_and_target_loss_revokes_lock_immediately(self):
        self.mode('fire')
        self.elapsed = .05; self.mode(); self.feed(True, True)
        self.elapsed = .10; self.mode(); self.feed(True, True); self.choose()
        for number in range(3, 12):
            self.elapsed = number*.1; self.mode(); self.feed(True, True)
        self.assertTrue(self.selection_messages[-1][1]['valid'])
        self.assertTrue(self.proposals[-1][1]['image_locked'])
        selection_count, proposal_count = len(self.selection_messages), len(self.proposals)
        for number in range(1, 21):
            self.elapsed = 1.1+number*.005; self.mode(); self.feed(True, True)
        self.assertEqual(len(self.selection_messages)-selection_count, 20,
                         'All selected exposures must pass even above invalid heartbeat rate')
        self.assertEqual(sum(document['valid'] for _, document in self.proposals[proposal_count:]), 20)
        before = len(self.proposals)
        self.elapsed += .0001
        self.feed(True, False)
        self.assertGreater(len(self.proposals), before, 'First target loss cannot wait for a heartbeat')
        self.assertFalse(self.selection_messages[-1][1]['valid'])
        self.assertFalse(self.proposals[-1][1]['valid'])
        self.assertFalse(self.proposals[-1][1]['image_locked'])
        self.assertFalse(self.proposals[-1][1]['pump_candidate'])
        self.assertEqual(self.proposals[-1][0], self.elapsed)

    def test_distinct_invalid_reasons_and_identity_changes_publish_immediately(self):
        self.mode('fire')
        self.elapsed = .01
        self.aim.invalidate('camera_geometry_changed')
        first = len(self.proposals)
        self.aim.invalidate('camera_geometry_changed')
        self.assertEqual(len(self.proposals), first)
        self.elapsed = .0101
        self.aim.invalidate('camera_geometry_rejected: changed matrix')
        self.assertEqual(len(self.proposals), first+1)
        self.aim.selection_history.identity = ('fixture', 1, 1)
        self.aim.invalidate('camera_geometry_rejected: changed matrix')
        self.assertEqual(len(self.proposals), first+2)


if __name__ == '__main__': unittest.main()
