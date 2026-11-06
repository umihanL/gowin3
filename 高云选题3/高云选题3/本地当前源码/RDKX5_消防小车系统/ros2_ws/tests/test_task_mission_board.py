"""Task arbitration using real ROS types, domain 83, all hardware blocked.

Mission callbacks run synchronously with a controlled monotonic clock; original
capture timestamps remain from the actual ROS clock. No camera/actuator/UART
node or actual drive command receiver is started.
"""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


ROOT=Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST')=='1','Explicit isolated ROS test only')
class TaskMissionBoardTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'),'83')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'),'1')
        self.tmp=tempfile.TemporaryDirectory(prefix='rescue-task-mission-test-')
        self.addCleanup(self.tmp.cleanup)
        for name in ('_HardwarePWM','_PumpGPIO','_ExclusiveOwner'):
            guard=patch('rescue_vision.local_actuator_io.'+name,
                        side_effect=AssertionError('Hardware access forbidden: '+name))
            guard.start();self.addCleanup(guard.stop)
        self.now=100.
        timer=patch('rescue_vision.mission_node.time.monotonic',side_effect=lambda:self.now)
        timer.start();self.addCleanup(timer.stop)
        mount=json.loads((ROOT/'config/mount.template.json').read_text(encoding='utf-8-sig'))
        mount.update(mounting_verified=True,road_pose_verified=True,road_pan_deg=90.,road_tilt_deg=120.,
                     pan_home_deg=90.,tilt_home_deg=90.,speed_deg_s=180.,extra_settle_s=.1)
        path=Path(self.tmp.name)/'mount.json';path.write_text(json.dumps(mount),encoding='utf-8')
        import rclpy
        from rescue_vision.mission_node import Mission
        rclpy.init(args=['--ros-args','-p','task_profiles_enabled:=true','-p','gimbal_transport:=x5',
                        '-p','mount_config:='+str(path),'-p','traffic_config:='+str(ROOT/'config/traffic.json')])
        self.node=Mission()
        self.commands=[];self.permissions=[];self.states=[];self.traffic=[]
        class Capture:
            def __init__(self,output,is_json=False):self.output=output;self.is_json=is_json
            def publish(self,msg):self.output.append(json.loads(msg.data) if self.is_json else msg)
        self.node.gimbal_pub=Capture(self.commands)
        self.node.permission_pub=Capture(self.permissions)
        self.node.state_pub=Capture(self.states,True)
        self.node.traffic_pub=Capture(self.traffic,True)
        self.seq=0

    def tearDown(self):
        import rclpy
        self.node.destroy_node();rclpy.shutdown()

    def message(self,value):
        from std_msgs.msg import String
        return String(data=json.dumps(value))

    def status(self,stopped=False):
        from rescue_interfaces.msg import FpgaEvent
        msg=FpgaEvent();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.event_type=FpgaEvent.STATUS;msg.vehicle_stopped=stopped
        self.node.event(msg)

    def ack(self):
        self.node.link_state(self.message(dict(stamp=self.node.get_clock().now().nanoseconds/1e9,
            status_fresh=True,gimbal_settled_estimate=True,gimbal_acknowledged=True,
            target_deg=self.node.target)))

    def step(self,delta=.1,stopped=False,status=True,ack=True):
        self.now+=delta
        if status:self.status(stopped)
        if ack:self.ack()
        self.node.tick()

    def settle(self,stopped=False):
        self.now=max(self.now,self.node.move_end+.001)
        self.status(stopped);self.ack();self.node.tick()

    def start_patrol(self):
        self.node.request(self.message({'mode':'patrol'}))
        self.node.tick()
        self.assertFalse(self.permissions[-1].allow_motion,'Chassis feedback is still required')
        self.status(False);self.node.tick()
        self.assertTrue(self.permissions[-1].allow_motion)
        self.assertEqual(self.commands,[],'Removed road observation must not move the camera')

    def start_crossing(self):
        self.start_patrol()
        self.node.request(self.message(dict(mode='traffic',request_id='manual-light-1',
            issued_at=self.node.get_clock().now().nanoseconds/1e9,ttl_s=2.)))
        self.assertIsNotNone(self.node.traffic.event_id)
        self.assertFalse(self.permissions[-1].allow_motion,'Explicit light request immediately stops')
        count=len(self.commands)
        self.node.tick()
        self.assertEqual(len(self.commands),count,'Feedback profile waits for stopped chassis')
        self.step(stopped=True)
        self.assertEqual(self.commands[-1].owner,'traffic')
        self.assertEqual((self.commands[-1].pan_deg,self.commands[-1].tilt_deg),(90.,95.))
        self.assertFalse(self.permissions[-1].allow_motion)
        self.settle(stopped=True)

    def green(self,status=True):
        self.now+=.1
        if status:self.status(stopped=True)
        self.ack();self.seq+=1
        self.node.traffic_observation(self.message(dict(
            capture_ros_s=self.node.get_clock().now().nanoseconds/1e9,
            frame_id='traffic:'+str(self.seq),status='green',confidence=.9,fixture_bbox=[140,60,180,160])))
        self.node.tick()

    def test_patrol_needs_fresh_chassis_without_removed_camera_gate(self):
        self.start_patrol()
        self.step(.41)
        self.assertTrue(self.permissions[-1].allow_motion)
        self.node.fpga_time=self.now-1
        self.node.tick();self.assertFalse(self.permissions[-1].allow_motion,'Missing chassis must revoke motion')

    def test_explicit_traffic_stop_scan_stable_green_and_completion(self):
        from rescue_interfaces.msg import FpgaEvent
        self.start_crossing()
        for _ in range(10):self.green()
        self.assertEqual(self.node.traffic.state,'RELEASED')
        self.assertTrue(self.permissions[-1].allow_motion)
        event=int(self.node.traffic.event_id)
        msg=FpgaEvent();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.event_type=FpgaEvent.CROSSING_COMPLETE;msg.event_id=event+1
        self.node.event(msg);self.assertIsNotNone(self.node.traffic.event_id)
        msg.event_id=event;self.node.event(msg)
        self.assertIsNone(self.node.traffic.event_id)
        self.node.tick()
        self.assertEqual(self.node.camera_pose,'unconfigured')
        self.assertTrue(self.permissions[-1].allow_motion,'Fresh chassis patrol no longer needs zebra images')

    def test_released_green_never_outlives_chassis_status(self):
        self.start_crossing()
        for _ in range(10):self.green()
        self.assertTrue(self.permissions[-1].allow_motion)
        self.node.fpga_time=self.now-1
        self.green(status=False)
        self.assertFalse(self.permissions[-1].allow_motion,'Green does not override missing FPGA status')

    def test_observation_modes_immediately_hold_crossing_until_driving_resumes(self):
        self.start_crossing()
        for _ in range(10):self.green()
        self.assertTrue(self.permissions[-1].allow_motion)
        event=self.node.traffic.event_id
        for mode in ('inspect','fire','idle'):
            with self.subTest(mode=mode):
                previous=len(self.permissions)
                self.node.request(self.message({'mode':mode}))
                self.assertGreater(len(self.permissions),previous,'Mode request must immediately revoke old motion lease')
                self.assertFalse(self.permissions[-1].allow_motion)
                self.assertEqual(self.permissions[-1].reason,'crossing_pending_observation_hold')
                self.step(stopped=True)
                self.settle(stopped=True)
                for _ in range(10):self.green()
                self.assertFalse(self.permissions[-1].allow_motion,'Green cannot move an observing vehicle')
                self.assertEqual(self.states[-1]['reason'],'crossing_pending_observation_hold')
                self.assertFalse(self.states[-1]['yolo_requested'],'Pending crossing still owns observation')
                self.assertEqual(self.node.traffic.event_id,event,'Observation must not erase crossing requirement')
        self.node.request(self.message({'mode':'patrol'}))
        self.step(stopped=True);self.settle(stopped=True)
        for _ in range(10):self.green()
        self.assertTrue(self.permissions[-1].allow_motion,'Explicit return to driving can satisfy green crossing')
        self.assertEqual(self.node.traffic.event_id,event)

    def test_timeout_reset_holds_until_a_new_explicit_request(self):
        self.start_crossing()
        event=self.node.traffic.event_id
        self.now=self.node.traffic._start+self.node.traffic.cfg['event_timeout_s']+.1
        self.status(stopped=True);self.ack();self.node.tick()
        self.assertEqual(self.node.traffic.state,'TIMEOUT')
        self.node.request(self.message({'mode':'reset_traffic'}))
        self.assertIsNone(self.node.traffic.event_id)
        self.assertEqual(self.node.mode,'idle');self.assertTrue(self.node.operator_hold)
        self.assertFalse(self.permissions[-1].allow_motion)
        self.node.tick();self.assertIsNone(self.node.traffic.event_id)
        self.node.request(self.message(dict(mode='traffic',request_id='manual-light-2',
            issued_at=self.node.get_clock().now().nanoseconds/1e9,ttl_s=2.)))
        self.assertIsNotNone(self.node.traffic.event_id)
        self.assertNotEqual(self.node.traffic.event_id,event)
        self.assertFalse(self.permissions[-1].allow_motion)


if __name__=='__main__':unittest.main()
