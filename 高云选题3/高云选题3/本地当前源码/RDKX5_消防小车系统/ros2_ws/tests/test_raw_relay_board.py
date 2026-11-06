"""Prove rclpy serialized relay preserves every byte on isolated domain 83."""
import json
import os
import time
import unittest
from unittest.mock import Mock, patch


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST') == '1', 'Isolated board test only')
class RawRelayBoardTests(unittest.TestCase):
    def test_full_image_header_and_original_serialization_survive_relay(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from rescue_vision.fast_mipi_perception_node import FastMipiPerception
        from rescue_vision.node_common import IMAGE_QOS
        rclpy.init(args=['--ros-args', '-p', 'detector_enabled:=false', '-p', 'lens_enabled:=false'])
        observer, probe = FastMipiPerception(), Node('relay_byte_probe')
        executor = SingleThreadedExecutor()
        received = []
        probe.create_subscription(Image, '/rescue/camera/road_raw', received.append, IMAGE_QOS, raw=True)
        executor.add_node(probe)
        try:
            end = time.monotonic()+5
            while observer.road_relay_pub.get_subscription_count() == 0 and time.monotonic() < end:
                executor.spin_once(timeout_sec=.02)
            self.assertEqual(observer.road_relay_pub.get_subscription_count(), 1)
            frame = Image()
            frame.header.stamp = observer.get_clock().now().to_msg()
            frame.header.frame_id = 'unchanged_physical_source'
            frame.width, frame.height, frame.step, frame.encoding = 1280, 1088, 1280, 'nv12'
            frame.data.frombytes(bytes(range(256))*(1280*1088*3//2//256))
            serialized = serialize_message(frame)
            observer.road_demand(String(data=json.dumps(dict(stamp=observer.get_clock().now().nanoseconds/1e9,
                process_fps_limit=3., source_topic='/image_right_raw', raw_input_topic='/rescue/camera/road_raw',
                process_on_receive=True, raw_subscription_active=True))))
            observer.receive_raw(serialized)
            while not received and time.monotonic() < end:
                executor.spin_once(timeout_sec=.02)
            self.assertEqual(received, [serialized])
            self.assertEqual(observer.road_relay_count, 1)
            # Duplicate capture is rejected before it can reach the relay.
            observer.road_relay.next_due = None
            observer.receive_raw(serialized)
            self.assertEqual(observer.road_relay_count, 1)
            # A transport failure in the auxiliary output cannot kill YOLO's
            # capture callback; the next valid image must still enter its slot.
            bad_publisher = Mock()
            bad_publisher.get_subscription_count.return_value = 1
            bad_publisher.publish.side_effect = RuntimeError('injected relay DDS failure')
            before = observer.input_count
            with patch.object(observer, 'road_relay_pub', bad_publisher):
                frame.header.stamp = observer.get_clock().now().to_msg()
                observer.receive_raw(serialize_message(frame))
            self.assertEqual(observer.input_count, before+1)
            self.assertIn('injected relay DDS failure', observer.road_relay_error)
            self.assertEqual(observer.stage_errors['capture'], '')
            observer.road_relay.next_due = None
            frame.header.stamp = observer.get_clock().now().to_msg()
            observer.receive_raw(serialize_message(frame))
            self.assertEqual(observer.input_count, before+2)
            self.assertEqual(observer.road_relay_count, 2)
            self.assertEqual(observer.road_relay_error, '')
        finally:
            executor.remove_node(probe)
            executor.shutdown()
            observer.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
