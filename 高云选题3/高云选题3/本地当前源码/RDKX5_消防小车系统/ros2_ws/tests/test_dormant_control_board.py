"""Dormant ROS resources disappear without changing active control contracts.

Use RESCUE_RUN_ROS_TEST=1 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1.
Real UART construction is intercepted; no PWM/GPIO node is created.
"""
import os
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'Explicit isolated ROS test only')
class DormantControlBoardTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        import rclpy
        self.rclpy = rclpy
        rclpy.init(args=[])
        self.nodes = []
        self.serial = Mock()
        self.serial.in_waiting = 4
        self.serial.read.return_value = b''
        self.serial.write.side_effect = lambda data: len(data)
        self.serial_patch = patch('serial.Serial', return_value=self.serial)
        self.serial_ctor = self.serial_patch.start()
        self.addCleanup(self.serial_patch.stop)

    def tearDown(self):
        for node in reversed(self.nodes):
            node.destroy_node()
        self.rclpy.shutdown()

    def bridge(self, *, dry):
        from rescue_vision.fpga_bridge_node import FpgaBridge
        with patch('rescue_vision.fpga_bridge_node.param',
                   side_effect=lambda node, name, default: {'dry_run': dry, 'port': '/forbidden/test-uart'}.get(name, default)):
            node = FpgaBridge()
        self.nodes.append(node)
        return node

    def mission(self, *, task_profiles):
        from rescue_vision.mission_node import Mission
        values = dict(task_profiles_enabled=task_profiles,
                      mount_config=str(ROOT/'config/mount.template.json'),
                      traffic_config=str(ROOT/'config/traffic.json'))
        with patch('rescue_vision.mission_node.param',
                   side_effect=lambda node, name, default: values.get(name, default)):
            node = Mission()
        self.nodes.append(node)
        return node

    @staticmethod
    def periods(node):
        return sorted(timer.timer_period_ns for timer in node.timers)

    def test_dry_uart_omits_receive_but_keeps_stop_heartbeat(self):
        from rescue_vision import protocol
        node = self.bridge(dry=True)
        self.serial_ctor.assert_not_called()
        self.assertIsNone(node.receive_timer)
        self.assertEqual(self.periods(node), [100_000_000])
        with patch.object(node, 'send') as send:
            node.tick()
        send.assert_called_once()
        kind, payload, metadata = send.call_args.args
        self.assertEqual(kind, protocol.PERMISSION)
        self.assertFalse(metadata['allowed'])
        self.assertEqual(payload, protocol.motion_payload(metadata['event_id'], False, metadata['lease_ms']))

    def test_connected_uart_keeps_50hz_receive_and_10hz_stop(self):
        from rescue_vision import protocol
        node = self.bridge(dry=False)
        self.serial_ctor.assert_called_once_with('/forbidden/test-uart', 115200, timeout=0, write_timeout=.05)
        self.assertEqual(self.periods(node), [20_000_000, 100_000_000])
        node.receive_timer.callback()
        self.serial.read.assert_called_once_with(4)
        with patch.object(node, 'send') as send:
            node.tick()
        self.assertEqual(send.call_args.args[0], protocol.PERMISSION)
        self.assertFalse(send.call_args.args[2]['allowed'])

    def test_task_profiles_omit_stereo_reader_but_keep_observers_and_control(self):
        node = self.mission(task_profiles=True)
        topics = {sub.topic_name for sub in node.subscriptions}
        self.assertNotIn('/stereo/pair', topics)
        self.assertIn('/road/observation', topics)
        self.assertIn('/traffic/observation', topics)
        self.assertEqual(self.periods(node), [100_000_000])
        with patch.object(node.bridge, 'imgmsg_to_cv2', side_effect=AssertionError('Task mode must not decode stereo')):
            node.image(object())

    def test_legacy_mode_keeps_stereo_reader_and_fresh_image_detection(self):
        from rescue_interfaces.msg import StereoPair
        node = self.mission(task_profiles=False)
        topics = {sub.topic_name for sub in node.subscriptions}
        self.assertIn('/stereo/pair', topics)
        self.assertNotIn('/road/observation', topics)
        self.assertNotIn('/traffic/observation', topics)
        self.assertEqual(self.periods(node), [100_000_000])
        node.traffic.event_id = 1
        msg = StereoPair()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.sequence = 25
        pixels, observation = object(), {'state': 'GREEN'}
        with patch.object(node.bridge, 'imgmsg_to_cv2', return_value=pixels) as decode, \
                patch.object(node.detector, 'detect', return_value=observation) as detect:
            node.image(msg)
        decode.assert_called_once_with(msg.left, 'bgr8')
        self.assertIs(detect.call_args.args[0], pixels)
        self.assertEqual(detect.call_args.kwargs['frame_id'], 25)
        self.assertIs(node.observation, observation)


if __name__ == '__main__':
    unittest.main()
