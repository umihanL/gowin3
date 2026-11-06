"""Real ROS types + serialized camera adapter, isolated domain 83 only.

No camera/servo/GPIO drivers are started. Synthetic input is passed directly to
the public subscription callbacks and published output messages are captured.
"""
import json
import os
import time
import unittest
from array import array


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST')=='1','Explicit isolated X5 ROS test only')
class RoadVisionBoardTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'),'83')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'),'1')
        import rclpy
        from rescue_vision.road_vision_node import RoadVision
        rclpy.init()
        self.node = RoadVision()
        self.road=[]
        self.traffic=[]
        class Capture:
            def __init__(self, output): self.output=output
            def publish(self, msg): self.output.append(json.loads(msg.data))
        self.node.traffic_pub=Capture(self.traffic)

    def tearDown(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()

    def mission(self, **values):
        from std_msgs.msg import String
        state=dict(stamp=self.node.get_clock().now().nanoseconds/1e9,mode='patrol',
                   camera_pose='traffic',gimbal_ready=True,traffic_active=True)
        state.update(values)
        self.node.on_mission(String(data=json.dumps(state)))

    def image(self, content=None, age=0.):
        import cv2
        import numpy as np
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Image
        if content is None:
            content=np.full((360,480,3),25,np.uint8)
            for i in range(7):
                cv2.rectangle(content,(70+i*47,170),(88+i*47,285),(235,235,235),-1)
        msg=Image()
        msg.header.frame_id='camera_raw'
        msg.width=480;msg.height=360;msg.encoding='bgr8';msg.step=480*3
        # bytes assignment invokes per-byte Python validation in generated ROS
        # setters (~430 ms on X5), which would make this synthetic capture stale.
        # A typed array uses the public fast setter; timestamp after preparation.
        msg.data=array('B',content.tobytes())
        stamp=self.node.get_clock().now().nanoseconds-round(age*1e9)
        msg.header.stamp.sec=stamp//1_000_000_000
        msg.header.stamp.nanosec=stamp%1_000_000_000
        return serialize_message(msg),stamp

    def feed(self, **kw):
        payload,stamp=self.image(**kw)
        original=bytes(payload)
        self.node.on_raw(payload)
        self.node.process()
        self.assertEqual(payload,original)
        return payload,stamp

    def test_raw_cdr_pose_stamp_and_unique_identity(self):
        self.feed()
        self.assertFalse(self.road)
        self.mission(gimbal_ready=False)
        self.feed()
        self.assertFalse(self.road)
        self.mission()
        payload,stamp=self.feed()
        self.assertEqual(len(self.traffic),1)
        self.assertFalse(hasattr(self.node,'road_pub'))
        out=self.traffic[-1]
        self.assertEqual(out['status'],'unknown')
        self.assertEqual(out['sequence'],str(stamp))
        self.assertEqual(out['stamp']['sec']*1_000_000_000+out['stamp']['nanosec'],stamp)
        self.assertEqual(out['physical_eye'],'left')
        self.assertNotIn('capture_time',out)
        self.node.on_raw(payload);self.node.process()
        self.assertEqual(len(self.traffic),1,'Repeated exposure must not add confirmation frames')
        self.mission(mode='inspect',traffic_active=False)
        self.feed()
        self.assertEqual(len(self.traffic),1)

    def test_bad_age_cannot_poison_next_valid_capture(self):
        self.mission()
        self.feed(age=.6)
        self.feed(age=-10)
        self.assertFalse(self.traffic)
        self.feed()
        self.assertEqual(len(self.traffic),1)
        self.node.mission_received=time.monotonic()-1
        self.feed()
        self.assertEqual(len(self.traffic),1)

    def test_traffic_only_after_ready_and_preserves_detector_contract(self):
        import cv2
        import numpy as np
        image=np.full((360,480,3),185,np.uint8)
        cv2.rectangle(image,(220,90),(280,240),(10,10,10),-1)
        cv2.circle(image,(250,200),10,(0,255,0),-1)
        self.mission(traffic_active=True,gimbal_ready=False)
        self.feed(content=image)
        self.assertFalse(self.traffic)
        self.mission(traffic_active=True,camera_pose='traffic')
        self.feed(content=image)
        self.assertEqual(self.traffic[-1]['status'],'green')
        self.assertEqual(self.traffic[-1]['fixture_bbox'],[220,90,281,241])
        self.assertFalse(self.road)
        self.assertNotIn('capture_time',self.traffic[-1])

    def test_raw_reader_exists_only_for_fresh_settled_traffic(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from rclpy.serialization import deserialize_message
        from rescue_vision.node_common import IMAGE_QOS
        probe=Node('road_reader_lifecycle_probe')
        publisher=probe.create_publisher(Image,self.node.source_topic,IMAGE_QOS)
        try:
            def wait_for_reader(expected, refresh=None):
                deadline=time.monotonic()+2
                while time.monotonic()<deadline:
                    if refresh is not None:self.mission(**refresh)
                    rclpy.spin_once(self.node,timeout_sec=.02)
                    if publisher.get_subscription_count()==expected:return
                self.assertEqual(publisher.get_subscription_count(),expected)

            self.assertIsNone(self.node.raw_subscription)
            wait_for_reader(0)
            self.mission(mode='inspect',traffic_active=False)
            wait_for_reader(0)
            self.mission(gimbal_ready=False)
            wait_for_reader(0)
            self.mission()
            wait_for_reader(1,refresh={})
            first_reader=self.node.raw_subscription
            # Switching analyses does not tear down/recreate an unchanged input.
            self.mission(traffic_active=True,camera_pose='traffic')
            self.assertIs(self.node.raw_subscription,first_reader)
            wait_for_reader(1,refresh={'traffic_active':True,'camera_pose':'traffic'})
            self.mission(mode='inspect',traffic_active=False)
            self.assertIsNone(self.node.raw_subscription)
            self.assertIsNone(self.node.latest)
            wait_for_reader(0)
            # Restoring traffic mode reinstates a DDS reader and actual delivery.
            self.mission()
            wait_for_reader(1,refresh={})
            self.assertIsNot(self.node.raw_subscription,first_reader)
            deadline=time.monotonic()+2
            while not self.traffic and time.monotonic()<deadline:
                self.mission()
                raw,_=self.image()
                publisher.publish(deserialize_message(raw,Image))
                rclpy.spin_once(self.node,timeout_sec=.02)
                self.node.process()
            self.assertTrue(self.traffic,'Raw DDS delivery must recover after resubscription')
            # Timeout must remove the reader even without a new mission message.
            self.node.mission_received=time.monotonic()-1
            self.node.process()
            self.assertIsNone(self.node.raw_subscription)
            wait_for_reader(0)
        finally:
            probe.destroy_node()


if __name__=='__main__':
    unittest.main()
