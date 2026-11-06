"""Real mission callbacks with isolated fake ROS transport; no hardware opened."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision.task_interaction import TaskRouter,parse_task_text
from rescue_vision.traffic import TrafficController  # Load cv2 outside the temporary ROS module patch.
from rescue_vision.voice_verification import needs_independent_verification


class Message:
    STATUS=1;CROSSWALK=2;CROSSING_COMPLETE=3
    def __init__(self,**kw):
        self.header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=100,nanosec=0))
        self.__dict__.update(kw)


class Publisher:
    def __init__(self):self.values=[]
    def publish(self,value):self.values.append(value)


class Node:
    def __init__(self,name):self.now=100.;self.topics=[]
    def create_publisher(self,*args):return Publisher()
    def create_subscription(self,kind,topic,*args):self.topics.append(topic)
    def create_timer(self,*args):pass
    def get_clock(self):
        return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=round(self.now*1e9),
            to_msg=lambda:types.SimpleNamespace(sec=int(self.now),nanosec=round((self.now%1)*1e9))))
    def get_logger(self):return types.SimpleNamespace(warning=lambda _:None,error=lambda _:None)


class TrafficTaskTests(unittest.TestCase):
    def setUp(self):
        modules={name:types.ModuleType(name) for name in ('rclpy','rclpy.node','cv_bridge','std_msgs',
            'std_msgs.msg','rescue_interfaces','rescue_interfaces.msg','rescue_vision.node_common')}
        modules['rclpy.node'].Node=Node
        modules['cv_bridge'].CvBridge=lambda:None
        modules['std_msgs.msg'].String=Message
        for name in ('StereoPair','FpgaEvent','GimbalCommand','MotionPermission','ManeuverCommand'):
            setattr(modules['rescue_interfaces.msg'],name,Message)
        common=modules['rescue_vision.node_common'];common.IMAGE_QOS=object()
        params=dict(task_profiles_enabled=True,navigation_owner='fpga',initial_mode='inspect',gimbal_transport='x5',
            mount_config='mount',traffic_config='traffic')
        common.param=lambda node,name,default:params.get(name,default)
        common.read_json=lambda p:dict(controller=dict(stop_policy='delay',stop_delay_s=2.)) if p=='traffic' else dict(
            mounting_verified=False,pan_home_deg=90.,tilt_home_deg=90.,pan_sign=1.,tilt_sign=1.,
            pan_limits_deg=[0.,180.],tilt_limits_deg=[0.,180.],speed_deg_s=90.,extra_settle_s=.1)
        common.age=lambda node,stamp:node.now-stamp.sec-stamp.nanosec/1e9
        common.publish_json=lambda pub,value:pub.publish(json.loads(json.dumps(value)))
        common.spin=lambda _:None
        spec=importlib.util.spec_from_file_location('rescue_vision._traffic_task_test',ROOT/'src/rescue_vision/rescue_vision/mission_node.py')
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules):spec.loader.exec_module(self.module)
        self.node=self.module.Mission()
        self.clock=patch.object(self.module.time,'monotonic',side_effect=lambda:self.node.now)
        self.clock.start();self.addCleanup(self.clock.stop)

    def request(self,**kw):
        values=dict(mode='traffic',request_id='check-1',issued_at=self.node.now,ttl_s=2.)
        values.update(kw);self.node.request(Message(data=json.dumps(values)))

    def test_no_crosswalk_visual_subscription_or_callback_remains(self):
        self.assertNotIn('/road/observation',self.node.topics)
        self.assertIn('/traffic/observation',self.node.topics)
        self.assertFalse(hasattr(self.node,'road'))

    def test_extension_traffic_is_explicit_and_fixed_camera_can_be_verified(self):
        node=self.node;node.extension_enabled=True
        node.extension_state(Message(data=json.dumps(dict(stamp=100.,task='vision'))))
        event=Message(event_type=Message.CROSSWALK,event_id=12)
        node.event(event)
        self.assertIsNone(node.traffic.event_id)
        node.extension_state(Message(data=json.dumps(dict(stamp=100.,task='traffic',fixed_traffic_camera_verified=True))))
        self.assertIsNotNone(node.traffic.event_id)
        self.assertFalse(node.ready(100.),'Fixed camera still needs measured chassis stop')
        node.event(Message(event_type=Message.STATUS,vehicle_stopped=True))
        self.assertTrue(node.ready(100.))
        node.tick()
        self.assertEqual(node.state_pub.values[-1]['extension_task'],'traffic')
        self.assertFalse(node.state_pub.values[-1]['yolo_requested'])
        node.extension_state(Message(data=json.dumps(dict(stamp=100.,task='vision'))))
        node.tick()
        self.assertIsNone(node.traffic.event_id)
        self.assertTrue(node.state_pub.values[-1]['yolo_requested'])

    def test_extension_seek_keeps_yolo_and_uses_measured_stopped_feedback(self):
        node=self.node;node.extension_enabled=True;node.mode='navigate'
        node.extension_state(Message(data=json.dumps(dict(stamp=100.,task='semantic',active={'phase':'seeking'}))))
        node.event(Message(event_type=Message.STATUS,vehicle_stopped=False))
        node.tick()
        self.assertTrue(node.state_pub.values[-1]['yolo_requested'])
        self.assertFalse(node.state_pub.values[-1]['vehicle_stopped'])
        node.extension_state(Message(data=json.dumps(dict(stamp=100.,task='semantic',active={'phase':'stopping'}))))
        node.tick()
        self.assertFalse(node.state_pub.values[-1]['motion_allowed'])

    def test_exact_voice_route_stops_then_standby_observation_never_starts_driving(self):
        request=parse_task_text('识别红绿灯')
        request.update(command_id='check-1',source='text',issued_at=100.,ttl_s=2.)
        result=TaskRouter().route(request,100.,mission=dict(stamp=100.,mode='inspect'))
        self.assertIsNone(result['navigation'])
        self.assertEqual(result['actuator']['action'],'disarm')
        self.node.request(Message(data=json.dumps(result['mission'])))
        self.assertEqual(self.node.mode,'standby')
        self.assertEqual(self.node.traffic.state,'WAIT_STOP')
        self.assertFalse(self.node.permission_pub.values[-1].allow_motion)
        self.node.now=101.999;self.node.tick()
        self.assertEqual(self.node.traffic.state,'WAIT_STOP')
        self.node.now=102.;self.node.tick()
        self.assertEqual(self.node.traffic.state,'SCANNING')
        self.assertFalse(self.node.gimbal_pub.values,'Unverified mounting cannot command a gimbal')
        self.assertFalse(self.node.permission_pub.values[-1].allow_motion)

    def test_expired_duplicate_and_repeated_request_cannot_restart_timer(self):
        self.request(issued_at=90.);self.assertIsNone(self.node.traffic.event_id)
        self.request();event=self.node.traffic.event_id;start=self.node.traffic._start
        self.node.now+=.5;self.request();self.request(request_id='other')
        self.assertEqual(self.node.traffic.event_id,event);self.assertEqual(self.node.traffic._start,start)

    def test_preview_cannot_supply_formal_green(self):
        self.request()
        self.node.traffic_observation(Message(data=json.dumps(dict(capture_ros_s=100.,status='green',
            confidence=.9,observation_only=True,control_eligible=False))))
        self.assertIsNone(self.node.observation)

    def test_standby_does_not_command_road_pose_and_patrol_needs_chassis_feedback(self):
        self.node.request(Message(data='{"mode":"standby"}'));self.node.tick()
        self.assertFalse(self.node.gimbal_pub.values);self.assertFalse(self.node.permission_pub.values[-1].allow_motion)
        self.node.request(Message(data='{"mode":"patrol"}'));self.node.tick()
        self.assertFalse(self.node.permission_pub.values[-1].allow_motion)
        self.assertEqual(self.node.permission_pub.values[-1].reason,'fpga_status_unavailable')

    def test_removed_zebra_commands_rejected_and_conditional_light_text_read_only(self):
        for text in ('识别斑马线','开始识别斑马线','寻找斑马线'):
            with self.assertRaisesRegex(ValueError,'crosswalk_recognition_removed'):parse_task_text(text)
        toggle=parse_task_text('开启斑马线识别')
        self.assertEqual(toggle['action'],'set_zebra_detection')
        self.assertTrue(toggle['enabled'])
        for text in ('不要识别红绿灯','能否识别红绿灯','识别红绿灯然后前进'):
            self.assertEqual(parse_task_text(text)['action'],'dialogue')

    def test_traffic_voice_action_keeps_independent_asr_verification(self):
        self.assertTrue(needs_independent_verification('识别红绿灯',dict(confidence_available=False),{}))

    def test_green_requires_existing_drive_task_mounting_and_fresh_navigation(self):
        self.node.mode='navigate'
        self.node.mount['mounting_verified']=True
        self.request()
        self.assertEqual(self.node.mode,'navigate')
        self.node.ready=lambda now:True  # Isolated measured-pose input to arbitration.
        self.node.command=lambda *args:None
        def step(now,color=None):
            self.node.now=now
            self.node.navigation_proposal(Message(data=json.dumps(dict(stamp=now,owner='fpga',valid=True,
                ready_to_move=True,arrived=False,status_fresh=True,vehicle_stopped=True))))
            if color is not None:
                self.node.traffic_observation(Message(data=json.dumps(dict(capture_ros_s=now,frame_id=str(now),
                    status=color,confidence=.9,fixture_bbox=[40,40,80,160]))))
            self.node.tick()
        step(102.)
        step(102.31,'green')
        self.assertFalse(self.node.permission_pub.values[-1].allow_motion,'A single green is insufficient')
        for n in range(1,7):step(102.31+n*.1,'green')
        self.assertTrue(self.node.permission_pub.values[-1].allow_motion)
        self.node.nav_received=90.;self.node.tick()
        self.assertFalse(self.node.permission_pub.values[-1].allow_motion,'Stale navigation still revokes GO')
        step(103.02,'red')
        self.assertFalse(self.node.permission_pub.values[-1].allow_motion)


if __name__=='__main__':unittest.main()
