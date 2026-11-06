"""Actual classifiers and node callbacks; fake ROS transport, no hardware.

Exercise the isolation that prevents fixed-camera commissioning images from
becoming mission/traffic control evidence. These are not board accuracy tests.
"""
import importlib.util
import json
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from test_raw_image_cdr import fixture


class Publisher:
    def __init__(self): self.values=[]; self.subscribers=0
    def get_subscription_count(self): return self.subscribers


class FakeNode:
    def __init__(self, name):
        self.now_ns=10_000_000_000; self.publishers={}; self.timers=[]; self.subscriptions=[]
        self.subscription_topics=[]
    def create_publisher(self, kind, topic, qos):
        pub=Publisher(); self.publishers[topic]=pub; return pub
    def create_subscription(self, kind, topic, callback, qos, **options):
        sub=object(); self.subscriptions.append(sub); self.subscription_topics.append(topic); return sub
    def destroy_subscription(self, sub): self.subscriptions.remove(sub)
    def create_timer(self, period, callback): self.timers.append((period,callback))
    def get_clock(self):
        return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=self.now_ns))
    def get_logger(self): return types.SimpleNamespace(info=lambda _:None,warning=lambda _:None)


class FixedCameraObservationTests(unittest.TestCase):
    def setUp(self):
        self.params={}
        modules={name:types.ModuleType(name) for name in
                 ('rclpy','rclpy.node','sensor_msgs','sensor_msgs.msg','std_msgs','std_msgs.msg',
                  'rescue_vision.node_common')}
        modules['rclpy.node'].Node=FakeNode
        modules['sensor_msgs.msg'].Image=object
        modules['std_msgs.msg'].String=types.SimpleNamespace
        common=modules['rescue_vision.node_common']
        common.param=lambda node,name,default:self.params.get(name,default)
        common.read_json=lambda path:{}
        common.publish_json=lambda pub,value:pub.values.append(json.loads(json.dumps(value,allow_nan=False)))
        common.spin=lambda _:None
        common.IMAGE_QOS=object()
        spec=importlib.util.spec_from_file_location('rescue_vision._road_preview_tests',
            ROOT/'src/rescue_vision/rescue_vision/road_vision_node.py')
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules):spec.loader.exec_module(self.module)

    def node(self, fixed=False):
        self.params['fixed_camera_observation']=fixed
        node=self.module.RoadVision();node.refresh_active();return node

    def image(self, node, kind='zebra', age=0., stamp=None):
        image=np.full((360,480,3),25,np.uint8)
        if kind=='zebra':
            for i in range(7):cv2.rectangle(image,(70+i*47,170),(88+i*47,285),(235,235,235),-1)
        else:
            image[:]=185
            cv2.rectangle(image,(220,90),(280,240),(10,10,10),-1)
            cv2.circle(image,(250,200),10,(0,255,0),-1)
        stamp=node.now_ns-round(age*1e9) if stamp is None else stamp
        return fixture(width=480,height=360,encoding='bgr8',step=480*3,
                       pixels=image.tobytes(),sec=stamp//1_000_000_000,nanosec=stamp%1_000_000_000)

    def feed(self,node,**kwargs):
        data=self.image(node,**kwargs);before=bytes(data)
        node.on_raw(data);node.process();self.assertEqual(data,before)
        return data

    def official_empty(self,node):
        self.assertNotIn('/road/observation',node.publishers)
        self.assertNotIn('/road/preview',node.publishers)
        self.assertFalse(node.traffic_pub.values)

    def test_fixed_camera_has_only_traffic_preview_without_mission_or_pose(self):
        node=self.node(fixed=True)
        self.assertIsNotNone(node.raw_subscription)
        self.assertEqual(node.process_fps,10.)
        self.feed(node)
        self.official_empty(node)
        traffic=node.traffic_preview_pub.values[-1]
        self.assertEqual(traffic['status'],'unknown')
        for output in (traffic,):
            self.assertTrue(output['observation_only'])
            self.assertFalse(output['control_eligible'])
            self.assertFalse(output['hardware_pose_verified'])
            self.assertEqual(output['camera_pose'],'unverified_fixed')
            self.assertNotIn('gimbal_ready',output)
            self.assertNotIn('vehicle_stopped',output)

    def test_extension_traffic_only_on_selected_task_unverified_preview_then_formal(self):
        node=self.node(fixed=True)
        node.on_mission(types.SimpleNamespace(data=json.dumps(dict(stamp=10.,mode='inspect',extension_enabled=True,
            extension_task='vision',traffic_active=False))))
        self.assertIsNone(node.raw_subscription)
        node.on_mission(types.SimpleNamespace(data=json.dumps(dict(stamp=10.,mode='standby',extension_enabled=True,
            extension_task='traffic',traffic_active=True,gimbal_ready=False,camera_pose='traffic'))))
        self.assertEqual(node.active,'fixed_preview')
        self.feed(node,kind='green')
        self.assertFalse(node.traffic_pub.values)
        self.assertTrue(node.traffic_preview_pub.values[-1]['observation_only'])
        node.now_ns+=100_000_000
        node.on_mission(types.SimpleNamespace(data=json.dumps(dict(stamp=10.1,mode='standby',extension_enabled=True,
            extension_task='traffic',traffic_active=True,gimbal_ready=True,camera_pose='traffic'))))
        self.assertEqual(node.active,'traffic')
        self.feed(node,kind='green')
        self.assertTrue(node.traffic_pub.values)
        self.assertNotIn('observation_only',node.traffic_pub.values[-1])

    def test_green_preview_never_becomes_release_evidence(self):
        node=self.node(fixed=True)
        node.on_mission(types.SimpleNamespace(data=json.dumps(dict(mode='patrol',camera_pose='traffic',
            gimbal_ready=True,traffic_active=True,stamp=10.))))
        self.feed(node,kind='green')
        self.assertEqual(node.traffic_preview_pub.values[-1]['status'],'green')
        self.official_empty(node)
        self.assertTrue(all(topic in {'/traffic/observation',
                           '/traffic/preview','/road/visualization','/diagnostics/road'} for topic in node.publishers))

    def test_visualization_only_encodes_for_subscriber_and_stops_after_leaving(self):
        node=self.node(fixed=True)
        with patch.object(self.module, 'make_visualization', wraps=self.module.make_visualization) as encode:
            self.feed(node)
            encode.assert_not_called()
            node.visualization_pub.subscribers=1
            node.now_ns+=100_000_000
            self.feed(node,kind='green')
            self.assertEqual(encode.call_count,1)
            value=node.visualization_pub.values[-1]
            frame=value['frame']
            self.assertEqual(frame['width'],480)
            self.assertEqual(frame['height'],360)
            self.assertIsNone(frame.get('road'))
            self.assertEqual(frame['traffic'],node.traffic_preview_pub.values[-1])
            self.assertEqual(frame['traffic']['status'],'green')
            for key in ('traffic',):
                for field in ('capture_ros_s','sequence','frame_id','source_topic'):
                    self.assertEqual(frame[key][field],frame[field])
            self.assertTrue(frame['observation_only'])
            self.assertFalse(frame['control_eligible'])
            node.visualization_pub.subscribers=0
            node.now_ns+=100_000_000
            self.feed(node)
            self.assertEqual(encode.call_count,1)
            self.assertEqual(node.visualization_encoded,1)
            self.official_empty(node)

    def test_visualization_failure_does_not_suppress_detector_evidence(self):
        node=self.node(fixed=True)
        node.visualization_pub.subscribers=1
        with patch.object(self.module, 'make_visualization', side_effect=ValueError('bad jpeg')):
            self.feed(node)
        self.assertEqual(node.traffic_preview_pub.values[-1]['status'],'unknown')
        self.assertEqual(node.counts['processed'],1)
        self.assertEqual(node.counts['invalid'],0)
        self.assertEqual(node.visualization_errors,1)
        self.assertFalse(node.visualization_pub.values)
        self.official_empty(node)

    def test_stale_future_duplicate_and_pre_activation_captures_do_not_publish(self):
        node=self.node(fixed=True)
        self.feed(node,age=.6)
        self.feed(node,age=-.1)
        self.feed(node,age=.01)  # Captured before observation was activated.
        self.assertFalse(node.traffic_preview_pub.values)
        payload=self.feed(node)
        node.on_raw(payload);node.process()
        self.assertEqual(len(node.traffic_preview_pub.values),1)
        self.official_empty(node)

    def test_processing_deadline_discards_result(self):
        node=self.node(fixed=True)
        original=node.detect
        def slow(bgr):
            result=original(bgr)
            node.now_ns+=400_000_000
            return result
        with patch.object(node,'detect',slow):self.feed(node)
        self.assertFalse(node.traffic_preview_pub.values)
        self.feed(node)
        self.assertEqual(len(node.traffic_preview_pub.values),1)
        self.official_empty(node)

    def test_latest_capture_only_and_diagnostics_do_not_assert_pose(self):
        node=self.node(fixed=True)
        first=self.image(node)
        node.now_ns+=30_000_000;second=self.image(node,kind='green')
        node.on_raw(first);node.on_raw(second)
        self.assertIs(node.latest,second)
        node.process();node.report()
        self.assertEqual(node.counts['skipped'],1)
        self.assertEqual(node.traffic_preview_pub.values[-1]['status'],'green')
        self.assertTrue(node.diagnostics.values[-1]['preview_only'])
        self.assertEqual(node.diagnostics.values[-1]['active_mode'],'fixed_preview')
        self.assertTrue(node.diagnostics.values[-1]['requires_confirmed_traffic_pose'])
        self.assertFalse(node.diagnostics.values[-1]['crosswalk_enabled'])

    def test_normal_mode_still_requires_ready_mission_and_publishes_no_preview(self):
        node=self.node()
        self.assertIsNone(node.raw_subscription)
        self.feed(node);self.official_empty(node)
        def mission(**kw):
            state=dict(mode='patrol',camera_pose='traffic',gimbal_ready=False,traffic_active=True,stamp=10.)
            state.update(kw);node.on_mission(types.SimpleNamespace(data=json.dumps(state)))
        mission();self.feed(node);self.official_empty(node)
        mission(gimbal_ready=True);self.feed(node)
        self.assertEqual(len(node.traffic_pub.values),1)
        self.assertNotIn('observation_only',node.traffic_pub.values[-1])
        self.assertFalse(node.traffic_preview_pub.values)
        node.mission_received=time.monotonic()-1;node.process()
        self.assertIsNone(node.raw_subscription)

    def test_relay_consumes_original_pixels_immediately_without_timer_duplication(self):
        self.params.update(raw_input_topic='/rescue/camera/road_raw', process_on_receive=True)
        node=self.node(fixed=True)
        self.assertEqual(node.process_fps, 10.)
        self.assertIn('/rescue/camera/road_raw', node.subscription_topics)
        self.assertNotIn('/image_right_raw', node.subscription_topics)
        payload=self.image(node)
        original=bytes(payload)
        node.on_raw(payload)
        self.assertEqual(payload,original)
        self.assertIsNone(node.latest)
        self.assertEqual(len(node.traffic_preview_pub.values),1)
        evidence=node.traffic_preview_pub.values[-1]
        self.assertEqual(evidence['source_topic'],'/image_right_raw')
        self.assertEqual(evidence['raw_input_topic'],'/rescue/camera/road_raw')
        self.assertEqual(evidence['sequence'],str(node.now_ns))
        self.official_empty(node)
        # A timer at the opposite phase cannot delay this evidence or publish
        # it again; duplicate exposures retain their original rejection rule.
        node.now_ns+=300_000_000
        node.process()
        node.on_raw(payload)
        self.assertEqual(len(node.traffic_preview_pub.values),1)
        node.now_ns+=33_333_333
        node.on_raw(self.image(node,kind='green'))
        self.assertEqual(len(node.traffic_preview_pub.values),2)
        self.assertEqual(node.traffic_preview_pub.values[-1]['status'],'green')
        node.report()
        demand=node.diagnostics.values[-1]
        self.assertTrue(demand['process_on_receive'])
        self.assertTrue(demand['raw_subscription_active'])
        self.assertEqual(demand['raw_input_topic'],'/rescue/camera/road_raw')
        self.assertEqual(demand['source_topic'],'/image_right_raw')
        self.assertEqual(demand['process_fps_limit'],10.)
        self.assertEqual(demand['stamp'],node.now_ns/1e9)
        self.assertTrue(any(period==1. and callback==node.report for period,callback in node.timers))
        self.official_empty(node)

    def test_relay_normal_mode_preserves_pose_and_age_gates_and_timer_expiry(self):
        self.params.update(raw_input_topic='/rescue/camera/road_raw', process_on_receive=True)
        node=self.node()
        self.assertEqual(node.process_fps,10.)
        self.assertIsNone(node.raw_subscription)
        node.on_raw(self.image(node)); self.official_empty(node)
        node.on_mission(types.SimpleNamespace(data=json.dumps(dict(mode='patrol',camera_pose='traffic',
            gimbal_ready=True,traffic_active=True,stamp=10.))))
        node.on_raw(self.image(node,age=.6)); self.official_empty(node)
        node.on_raw(self.image(node))
        self.assertEqual(node.traffic_pub.values[-1]['status'],'unknown')
        self.assertFalse(node.traffic_preview_pub.values)
        self.assertEqual(node.traffic_pub.values[-1]['source_topic'],'/image_right_raw')
        node.mission_received=time.monotonic()-1
        node.process()
        self.assertIsNone(node.raw_subscription)
        node.now_ns+=100_000_000
        node.on_raw(self.image(node))
        self.assertEqual(len(node.traffic_pub.values),1)

    def test_unbounded_source_cannot_enable_receive_driven_processing(self):
        self.params['process_on_receive']=True
        with self.assertRaisesRegex(ValueError,'rate-bounded raw relay'):
            self.node(fixed=True)

    def dialogue_inspection(self, node, **changes):
        mission = dict(stamp=node.now_ns/1e9, mode='inspect', motion_allowed=False,
            traffic_active=False, auto_fire_active=False, pump_armed=False)
        mission.update(changes)
        node.on_mission(types.SimpleNamespace(data=json.dumps(mission)))
        node.on_dialogue(types.SimpleNamespace(data=json.dumps(dict(
            stamp=node.now_ns/1e9, busy=True, state='thinking'))))

    def test_dialogue_yields_preview_then_requires_a_new_capture(self):
        node = self.node(fixed=True)
        self.feed(node)
        old = self.image(node)
        self.dialogue_inspection(node)
        node.on_raw(old); node.process(); node.report()
        self.assertTrue(node.diagnostics.values[-1]['yielding_to_dialogue'])
        self.assertIsNone(node.raw_subscription)
        self.assertIsNone(node.latest)
        self.assertEqual(len(node.traffic_preview_pub.values), 1)
        node.now_ns += 100_000_000
        node.on_dialogue(types.SimpleNamespace(data=json.dumps(dict(
            stamp=node.now_ns/1e9, busy=False, state='complete'))))
        self.assertIsNotNone(node.raw_subscription)
        node.on_raw(old); node.process()
        self.assertEqual(len(node.traffic_preview_pub.values), 1)
        self.feed(node)
        self.assertEqual(len(node.traffic_preview_pub.values), 2)
        self.assertEqual(node.traffic_preview_pub.values[-1]['capture_ros_s'], 10.1)
        self.official_empty(node)

    def test_dialogue_heartbeat_failure_and_traffic_resume_preview(self):
        node = self.node(fixed=True)
        self.dialogue_inspection(node)
        self.assertIsNone(node.raw_subscription)
        node.dialogue_received = time.monotonic()-3.
        node.process()
        self.assertIsNotNone(node.raw_subscription)
        self.dialogue_inspection(node)
        self.assertIsNone(node.raw_subscription)
        node.on_mission(types.SimpleNamespace(data=json.dumps(dict(node.mission,
            traffic_active=True))))
        self.assertIsNotNone(node.raw_subscription)

    def test_actual_traffic_observer_never_yields_to_dialogue(self):
        node = self.node()
        self.dialogue_inspection(node, traffic_active=True, camera_pose='traffic',
            gimbal_ready=True)
        self.assertFalse(node.yielding_to_dialogue)
        self.assertIsNotNone(node.raw_subscription)
        self.feed(node, kind='green')
        self.assertEqual(node.traffic_pub.values[-1]['status'], 'green')
        self.assertFalse(node.traffic_preview_pub.values)


    def test_road_pose_never_subscribes_or_classifies_stripes(self):
        node=self.node()
        node.on_mission(types.SimpleNamespace(data=json.dumps(dict(mode='patrol',camera_pose='road',
            gimbal_ready=True,traffic_active=False,stamp=10.))))
        self.feed(node)
        self.assertIsNone(node.raw_subscription)
        self.official_empty(node)
        self.assertFalse(node.traffic_preview_pub.values)

    def test_one_decode_and_same_pixels_for_detector_and_jpeg(self):
        node=self.node(fixed=True);node.visualization_pub.subscribers=1
        with patch.object(self.module,'image_to_bgr',wraps=self.module.image_to_bgr) as decode, \
             patch.object(node,'detect',wraps=node.detect) as detect, \
             patch.object(self.module,'make_visualization',wraps=self.module.make_visualization) as encode:
            self.feed(node,kind='green')
            self.assertEqual(decode.call_count,1)
            self.assertIs(detect.call_args.args[0],encode.call_args.args[0])

    def test_preview_encoding_budget_does_not_slow_detection_budget(self):
        node=self.node(fixed=True);node.visualization_pub.subscribers=1
        self.assertEqual(sum(node.visualization_ready(20+i/10+1e-8) for i in range(100)),80)
        self.assertEqual(node.process_fps,10.)
        node.visualization_pub.subscribers=0
        self.assertFalse(node.visualization_ready(30.))
        self.assertIsNone(node.visualization_due)


if __name__=='__main__':unittest.main()
