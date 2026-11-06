"""Real Humble imports and live DDS observer checks, isolated domain 83 only."""
import importlib.util
import json
import os
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'Explicit isolated X5 test only')
class PresentationQoSBoardTests(unittest.TestCase):
    def test_dashboard_observers_remove_event_waitables_but_keep_data_and_freshness(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location('presentation_qos_test', root/'tools/presentation_server.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        state = module.PresentationState()
        result = {}

        def inspect(node, stop_event, executor, is_ok):
            from rclpy.node import Node
            from std_msgs.msg import String
            probe = Node('dashboard_qos_probe')
            publisher = probe.create_publisher(String, '/mission/state', 10)
            try:
                self.assertEqual(len(list(node.subscriptions)), len(module.TOPICS))
                self.assertTrue(all(not s.event_handlers for s in node.subscriptions))
                # Command publishers retain default events and original QoS.
                self.assertTrue(node.command.event_handlers)
                executor.add_node(node); executor.add_node(probe)
                deadline = time.monotonic()+4.
                while time.monotonic() < deadline:
                    publisher.publish(String(data=json.dumps(dict(stamp=time.time(), mode='inspect'))))
                    executor.spin_once(timeout_sec=.02)
                    if state.envelope('mission')['fresh']:
                        break
                self.assertTrue(state.envelope('mission')['fresh'])
                self.assertEqual(state.envelope('mission')['data']['mode'], 'inspect')
                deadline = time.monotonic()+2.
                while time.monotonic() < deadline:
                    publisher.publish(String(data=json.dumps(dict(stamp=time.time()-10., mode='idle'))))
                    executor.spin_once(timeout_sec=.02)
                    if (state.envelope('mission')['data'] or {}).get('mode') == 'idle':
                        break
                envelope = state.envelope('mission')
                self.assertEqual(envelope['data']['mode'], 'idle')
                self.assertFalse(envelope['fresh'])
                self.assertEqual(state.pending, module.deque())
                result['passed'] = True
                result['subscriptions'] = len(list(node.subscriptions))
            finally:
                executor.remove_node(node); executor.remove_node(probe)
                executor.shutdown(); probe.destroy_node()

        with patch.object(module, 'spin_until_stopped', side_effect=inspect):
            module.ros_loop(state, threading.Event())
        self.assertTrue(result.get('passed'), state.ros)
        print('Humble dashboard observer test: %d subscriptions, zero subscription QoS waitables' % result['subscriptions'])


if __name__ == '__main__':
    unittest.main()
