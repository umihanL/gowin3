"""Real Mission/controller callbacks, fake ROS transport; never opens board I/O."""
import ast
import json
from pathlib import Path
import sys
import types
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision.auto_fire import AutoFireRound
from rescue_vision.auto_fire_gate import AutoArmFence
import test_traffic_task_control as traffic_fixture
Message=traffic_fixture.Message


class ScanRoundTests(unittest.TestCase):
    def start(self):
        c=AutoFireRound()
        self.emitted=c.request(dict(action='scan',command_id='scan-1',issued_at=100.,ttl_s=2.),100.)
        return c
    def evidence(self,now,**changes):
        mission=dict(stamp=now,mode='inspect',vehicle_stopped=True,gimbal_ready=True,
                     scan_hardware_ready=True,scan_wait_reason='scan_hardware_ready')
        mission.update(changes)
        return dict(mission=mission,targets=dict(capture_stamp=now,valid=True,registry_session='camera',
            targets=[dict(target_id=2,selectable=True,bbox_xyxy=[10,20,50,80])]))
    def test_scan_finds_fire_but_never_selects_or_arms(self):
        c=self.start();self.emitted+=c.tick(100.3,**self.evidence(100.3))
        self.emitted+=c.tick(100.5,**self.evidence(100.5))
        self.assertFalse(c.active);self.assertEqual(c.phase,'WAIT_SELECTION')
        self.assertEqual(c.observed_targets,[dict(target_id=2,registry_session='camera')])
        for t in range(101,110):self.emitted+=c.tick(t,**self.evidence(t))
        self.assertFalse(any(k=='select' or (k=='actuator' and v['action']=='arm_pump') for k,v in self.emitted))
        self.assertFalse(any(k=='mission' and v['mode']=='standby' for k,v in self.emitted))
    def test_scan_authority_cannot_arm_pump(self):
        f=AutoArmFence();f.request(dict(action='scan',command_id='scan-1',issued_at=100.,ttl_s=2.),100.)
        self.assertTrue(f.authorized('scan-1',100.1))
        self.assertFalse(f.check_arm(dict(auto_run_id='scan-1',issued_at=100.1,ttl_s=.5),100.1))
    def test_missing_hardware_waits_without_claiming_success(self):
        c=self.start();c.tick(100.3,**self.evidence(100.3,scan_hardware_ready=False,
            scan_wait_reason='scan_waiting_for_mount_verification'))
        self.assertTrue(c.active);self.assertEqual(c.scan_index,0)
        self.assertEqual(c.snapshot(100.3)['phase'],'WAIT_READY')
        self.assertEqual(c.reason,'scan_waiting_for_mount_verification')
    def test_scan_stops_on_feedback_or_vehicle_stop_loss_without_resume(self):
        for changes in [dict(scan_hardware_ready=False,scan_wait_reason='feedback_lost'),dict(vehicle_stopped=False)]:
            with self.subTest(changes=changes):
                c=self.start();c.tick(100.3,**self.evidence(100.3))
                out=c.tick(100.4,**self.evidence(100.4,**changes))
                self.assertFalse(c.active);self.assertTrue(c.reason.startswith('scan_interrupted:'))
                self.assertEqual(c.tick(100.5,**self.evidence(100.5)),[])
                self.assertEqual(dict(out)['actuator']['action'],'disarm')
    def test_manual_selection_releases_scan_without_overwriting_manual_mission(self):
        c=self.start()
        out=c.manual_selection(dict(command_id='select-2',target_id=2,issued_at=100.2,ttl_s=1.),100.2)
        self.assertFalse(c.active);self.assertEqual(c.reason,'manual_target_selected')
        self.assertNotIn('mission',dict(out));self.assertNotIn('select',dict(out))
    def test_no_fire_scan_ends_in_observation(self):
        c=self.start();out=[]
        for now in range(101,116):
            e=self.evidence(now);e['targets']['targets']=[];out+=c.tick(now,**e)
        self.assertFalse(c.active);self.assertEqual(c.reason,'no_selectable_fire_in_scan')
        self.assertFalse(any(k=='mission' and v['mode']=='standby' for k,v in out))


class NumberedArrivalTests(unittest.TestCase):
    setUp=traffic_fixture.TrafficTaskTests.setUp
    def mission_request(self,**kw):self.node.request(Message(data=json.dumps(kw)))
    def navigation(self,**kw):
        n=dict(stamp=self.node.now,owner='fpga',motion_kind='numbered_point',command_id='nav-1',
               arrived=True,valid=True,vehicle_stopped=True,status_fresh=True,estop=False,
               arrival_stamp=self.node.now,phase='arrived',ready_to_move=False,
               arrival_policy='manual' if self.node.arrival_policy=='manual' else 'scan')
        n.update(kw);self.node.navigation_state(Message(data=json.dumps(n)))
    def start(self,policy='scan'):
        self.node.extension_enabled=True
        self.mission_request(mode='navigate',command_id='nav-1',arrival_policy=policy)
    def test_extension1_arrival_keeps_yolo_without_auto_start(self):
        self.start('manual');self.navigation();self.node.tick()
        self.assertEqual(self.node.mode,'inspect');self.assertEqual(self.node.auto_pub.values,[])
        self.assertTrue(self.node.state_pub.values[-1]['yolo_requested'])
    def test_scan_arrival_once_even_if_duplicate_and_reenter_navigate(self):
        self.start();self.navigation();self.node.tick()
        self.assertEqual(len(self.node.auto_pub.values),1)
        self.assertEqual(self.node.auto_pub.values[0]['action'],'scan')
        self.assertEqual(self.node.scan_hold_run,'arrival:nav-1')
        self.navigation();self.node.tick();self.start();self.navigation();self.node.tick()
        self.assertEqual(len(self.node.auto_pub.values),1)
    def test_wrong_expired_failed_and_unconfirmed_arrival_have_no_effect(self):
        for changes in [dict(command_id='other'),dict(stamp=90.),dict(arrival_stamp=90.),
                dict(arrival_stamp=101.),dict(valid=False),dict(status_fresh=False),
                dict(vehicle_stopped=False),dict(estop=True),dict(phase='failed')]:
            with self.subTest(changes=changes):
                self.start();self.node.arrivals_seen.clear();self.node.navigation=None
                self.navigation(**changes);self.node.tick()
                self.assertEqual(self.node.mode,'navigate');self.assertFalse(self.node.auto_pub.values)
    def test_numbered_arrival_cannot_be_upgraded_to_automatic_spray(self):
        self.start();self.mission_request(arrival_policy='auto')
        self.navigation(arrival_policy='manual',task_profile='extension2');self.node.tick()
        self.assertEqual(self.node.auto_pub.values,[])
        self.assertEqual(self.node.arrival_policy,'manual')
    def test_numbered_arrival_rejects_auto_policy(self):
        self.start();self.navigation(arrival_policy='auto');self.node.tick()
        self.assertEqual(self.node.auto_pub.values,[]);self.assertEqual(self.node.mode,'navigate')
    def test_failed_command_cannot_later_replay_arrival(self):
        self.start();self.navigation(phase='failed',arrived=False,valid=False)
        self.navigation();self.node.tick();self.assertEqual(self.node.auto_pub.values,[])
    def test_scan_missing_hardware_never_commands_home(self):
        self.start();self.navigation();self.node.tick()
        self.node.auto_request(Message(data=json.dumps(self.node.auto_pub.values[0])))
        self.node.auto_state(Message(data=json.dumps(dict(stamp=100.,active=True,phase='SEARCH',
            scan_only=True,command_id='arrival:nav-1',pan_offset_deg=20.))))
        self.node.tick()
        self.assertEqual(self.node.gimbal_pub.values,[])
        state=self.node.state_pub.values[-1]
        self.assertTrue(state['scan_only']);self.assertFalse(state['scan_motion_allowed'])
        self.assertEqual(state['scan_wait_reason'],'scan_waiting_for_mount_verification')
    def test_scan_needs_live_measured_actuator_then_feedback_loss_revokes_motion(self):
        self.start();self.navigation();self.node.tick()
        self.node.mount['mounting_verified']=True
        self.node.auto_request(Message(data=json.dumps(self.node.auto_pub.values[0])))
        self.node.auto_state(Message(data=json.dumps(dict(stamp=100.,active=True,phase='SEARCH',
            scan_only=True,command_id='arrival:nav-1',pan_offset_deg=20.))))
        self.node.link_state(Message(data=json.dumps(dict(stamp=100.,enabled=True,dry_run=False,
            status_fresh=True,feedback_fresh=True,actual_angles_deg=[90.,90.]))))
        self.node.tick();self.assertTrue(self.node.scan_motion_allowed)
        sent=len(self.node.gimbal_pub.values);self.assertGreater(sent,0)
        self.node.link['feedback_fresh']=False;self.node.tick()
        self.assertFalse(self.node.scan_motion_allowed);self.assertEqual(len(self.node.gimbal_pub.values),sent)
    def test_stale_numbered_stop_is_not_replaced_by_unrelated_legacy_status(self):
        self.start();self.navigation();self.node.tick();self.node.now=100.4
        self.node.event(Message(event_type=Message.STATUS,vehicle_stopped=True))
        self.assertFalse(self.node.stopped(self.node.now))
    def test_actuator_boundary_blocks_old_scan_lease_immediately(self):
        source=ast.parse((ROOT/'src/rescue_vision/rescue_vision/local_actuator_node.py').read_text(encoding='utf-8'))
        cls=next(n for n in source.body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='owner_allowed')
        scope={};exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'<actual-owner-allowed>','exec'),scope)
        m=dict(mode='inspect',vehicle_stopped=True,scan_only=True,scan_motion_allowed=False)
        node=types.SimpleNamespace(mission=m,fresh_mission=lambda _:True)
        self.assertFalse(scope['owner_allowed'](node,100.,'mission_home'))
        m['scan_motion_allowed']=True;self.assertTrue(scope['owner_allowed'](node,100.,'mission_home'))
        m['vehicle_stopped']=False;self.assertFalse(scope['owner_allowed'](node,100.,'mission_home'))

if __name__=='__main__':unittest.main()
