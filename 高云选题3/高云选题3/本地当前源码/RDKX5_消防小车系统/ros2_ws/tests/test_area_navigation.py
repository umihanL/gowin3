"""Exercise actual core/protocol legs using explicit simulated FPGA replies."""
import json
import struct
import unittest
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision import ov5640_protocol as wire
from rescue_vision.extension_control import ExtensionControl
from rescue_vision.area_navigation import RESET_NAV, pose_reference
from rescue_vision.task_interaction import parse_task_text


class GeometryFixture:
    def readiness(self): return ''
    def set_region(self, value): self.region = value
    def observe_map(self, value, now): pass
    def observe_targets(self, value, now): pass
    def proposal(self, now, preferred): return {}


class Harness:
    def __init__(self):
        self.now = 100.
        self.c = ExtensionControl(True)
        c = self.c
        c.task = 'dialogue'; c.protocol_ready = True
        c.capabilities = dict(pump_b13=True, route_turn=True)
        c.numbered.load_catalog(json.loads((ROOT/'config/numbered_points.json').read_text(encoding='utf-8')))
        c.numbered.heading_supported = True
        self.catalog = json.loads((ROOT/'config/semantic_regions.json').read_text(encoding='utf-8'))
        self.signature = json.loads((ROOT/'config/fpga_map_catalog.json').read_text(encoding='utf-8'))['map_signature']
        c.area.load(self.catalog, self.signature, GeometryFixture())
        self.pose = {}
        self.feedback(session=False)

    def query(self, cmd, data):
        seq = self.c.send(cmd, now=self.now)
        self.c.receive(wire.Packet(seq, cmd|128, data), self.now+.001)

    def feedback(self, *, session=True, run=False, stopped=True, motion=0, point_state=None, task=None, pose=None):
        self.now += .05
        c, now = self.c, self.now
        c.actuator = dict(stamp=now, enabled=True, dry_run=False, status_fresh=True, feedback_fresh=True,
            reference_confirmed_this_start=True, pump_enabled=True, pump_ready=True, pump_on=False,
            pump_feedback_fresh=True, guard=dict(armed=False))
        c.mission_state = dict(stamp=now, mounting_verified=True, gimbal_transport='x5')
        c.auto_state = dict(stamp=now, active=False, phase='IDLE')
        c.observe_targets(dict(capture_stamp=now, capture_stamp_ns=int(now*1e9), sequence=int(now*100),
            frame_valid=True, valid=False, registry_session='test-camera', image_size_wh=[640,480], targets=[]),now)
        a = c.area.navigation.active or {}
        task = a.get('task_id', 0) if task is None else task
        self.query(wire.HEARTBEAT, bytes((0,motion,19|(8 if session else 0)|(4 if run else 0),task)))
        self.query(wire.POINT_STATUS, bytes((0,0,(4 if session else 0) if point_state is None else point_state,0)))
        self.query(wire.EXT_STATUS, bytes((10 if session else 0,8|(2 if stopped else 0),0,0)))
        if pose is not None:
            cell, heading = pose
            self.pose = dict(stamp=now, valid=True, x_mm=cell[0]*20+10,
                y_mm=3200-(cell[1]*20+10), heading_tenths=heading, mode=9,status=motion,task_id=task)
        c.area.observe_map(dict(stamp=now,supported=True,region_supported=True,map_signature=self.signature,
            pose=self.pose,start_reference=dict(stamp=now,valid=True,cell=[88,13],heading_deg=180.,generation=1)),now)

    def start(self, execute_fire=True):
        self.now += .01
        request = dict(parse_task_text('在A区域搜寻火源并执行灭火' if execute_fire else '去C区域搜索火焰'),
            command_id='region-test', source='web', issued_at=self.now, ttl_s=3.)
        self.c.request(request,self.now)

    def ack(self, result=0):
        self.now += .01
        a = self.c.area.navigation.active
        seq = a['sequence']; cmd = self.c.pending[seq][0]
        data = bytes((result,0,0,0))
        if not result and cmd == wire.SET_START_HEADING:
            data = bytes((0,))+wire.heading_payload(a['start_heading_deg'])[:2]+bytes((0,))
        self.c.receive(wire.Packet(seq,cmd|128,data), self.now)
        return cmd

    def drive_leg(self):
        n = self.c.area.navigation
        self.ack()
        self.feedback(session=False)
        n.tick(self.now+.002)
        assert n.active['phase'] == 'setting_start', n.active
        for _ in range(4): self.ack()
        assert n.active['phase'] == 'planning', n.active
        self.feedback(session=False,point_state=2)
        n.tick(self.now+.002)
        assert n.active['phase'] == 'starting',n.active
        self.ack()
        self.feedback(run=True,stopped=False,motion=2,pose=(n.active['start_cell'],n.active['start_heading_deg']*10))
        n.tick(self.now+.002)

    def arrive(self):
        n = self.c.area.navigation; a=n.active
        self.now += .01
        self.c.receive(wire.Packet(0,wire.DONE,bytes((a['task_id'],3,0,0))),self.now)
        self.feedback(motion=3,pose=(a['goal_cell'],900))
        n.tick(self.now+.002)


class AreaNavigationTests(unittest.TestCase):
    @staticmethod
    def axis_fault(h):
        h.c.actuator.update(error='tilt measured position outside calibrated limits/reference',
                            status_fresh=False, feedback_fresh=False,
                            pump_backend='fpga_uart', pump_pin='B13')

    def test_search_only_can_navigate_with_axis_fault_and_independent_b13_low(self):
        h = Harness(); self.axis_fault(h)
        self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False), '')
        self.assertNotEqual(h.c.area.readiness(h.now+.002, execute_fire=True), '')
        h.start(execute_fire=False)
        self.assertIs(h.c.area.navigation.active['execute_fire'], False)
        h.drive_leg()
        n = h.c.area.navigation
        h.c.receive(wire.Packet(0, wire.DONE, bytes((n.active['task_id'], 3, 0, 0))), h.now+.004)
        h.feedback(motion=3, pose=(n.active['goal_cell'], 900))
        self.axis_fault(h)
        h.c.area.tick(h.now+.002)
        self.assertEqual(n.active['phase'], 'arrived')
        self.assertEqual(h.c.area.search.phase, 'OBSERVING')
        # Full observation/next-leg handoff also retains the vehicle-only policy.
        for _ in range(33):
            h.feedback(motion=3, pose=(n.active['goal_cell'], 900)); self.axis_fault(h)
            h.c.area.tick(h.now+.002)
            if h.c.area.search.phase != 'OBSERVING':
                break
        self.assertEqual(h.c.area.search.phase, 'NAVIGATING')
        self.assertEqual(n.active['phase'], 'resetting')
        self.assertEqual(len(h.c.area.search.visited), 1)

    def test_axis_fault_navigation_exception_keeps_pump_and_estop_interlocks(self):
        for changes in (dict(pump_on=True), dict(pump_on=None), dict(pump_feedback_fresh=False),
                        dict(pump_backend='gpio'), dict(pump_pin='unknown'), dict(estop=True),
                        dict(guard=dict(armed=True)), dict(guard={}), dict(manual_pump_active=True),
                        dict(pump_requested_on=True)):
            with self.subTest(changes=changes):
                h = Harness(); self.axis_fault(h); h.c.actuator.update(changes)
                self.assertEqual(h.c.area.readiness(h.now+.002, execute_fire=False),
                                 'region_pump_off_and_disarmed_required')

    def test_observe_readiness_does_not_require_gimbal_or_geometry_but_fire_still_does(self):
        h = Harness()
        h.c.actuator.update(enabled=False, dry_run=True, feedback_fresh=False,
                            reference_confirmed_this_start=False, pump_enabled=False, pump_ready=False)
        h.c.mission_state.update(mounting_verified=False, gimbal_transport=None)
        h.c.area.approach = None
        state = h.c.state(h.now + .002)
        self.assertTrue(state['region_observe_ready'], state['region_observe_reason'])
        self.assertFalse(state['region_ready'])
        self.assertEqual(state['region_reason'], 'region_real_gimbal_reference_required')
        h.start(execute_fire=False)
        self.assertEqual(h.c.area.search.region_id, 'C')
        self.assertFalse(h.c.area.search.execute_fire)
        self.assertEqual(h.c.area.navigation.active['phase'], 'resetting')
        self.assertEqual(h.c.outbox[-1][4], RESET_NAV)

    def test_observe_still_requires_start_pump_off_idle_controller_and_priority_state(self):
        variants = (
            ('pump', lambda c: c.actuator.update(pump_feedback_fresh=False), 'pump_off'),
            ('start', lambda c: c.area.navigation.map['start_reference'].update(valid=False), 'mark_start'),
            ('priority', lambda c: c.mission_state.update(operator_hold=True), 'priority'),
            ('auto', lambda c: c.auto_state.update(active=True), 'auto_controller'),
            ('map', lambda c: c.area.navigation.map.update(map_signature='wrong-map'), 'map_identity'),
        )
        for name, change, expected in variants:
            with self.subTest(name=name):
                h = Harness()
                change(h.c)
                before = len(h.c.outbox)
                with self.assertRaisesRegex(ValueError, expected):
                    h.start(execute_fire=False)
                self.assertEqual(len(h.c.outbox), before)

    def test_observe_real_protocol_pause_requires_fresh_stopped_pose(self):
        h = Harness()
        h.c.area.approach = None
        h.start(execute_fire=False)
        h.drive_leg()
        n = h.c.area.navigation
        for _ in range(3):
            h.feedback(run=True, stopped=False, motion=2, pose=([88,13], 1800))
            h.c.observe_targets(dict(capture_stamp=h.now, frame_valid=True, registry_session='real-test',
                image_size_wh=[640,480], targets=[dict(target_id=5, selectable=True, confidence=.95,
                    bbox_xyxy=[100,100,200,200])]), h.now)
            h.c.area.tick(h.now + .002)
        self.assertEqual(n.active['phase'], 'pausing')
        self.assertEqual(h.c.area.search.phase, 'WAIT_STOP')
        h.ack()
        h.feedback(run=True, stopped=False, motion=2)
        h.c.area.tick(h.now + .002)
        self.assertEqual(h.c.area.search.phase, 'WAIT_STOP')
        h.feedback(motion=2, pose=([88,13], 1800))
        h.c.area.tick(h.now + .002)
        self.assertEqual(h.c.area.search.phase, 'OBSERVATION_DWELL')
        self.assertTrue(h.c.area.search.active)
        self.assertFalse(any(key == 'auto' for key, value in h.c.effects))
        paused_id = n.active['command_id']
        for _ in range(39):
            h.feedback(motion=2, pose=([88,13], 1800))
            h.c.area.tick(h.now + .002)
            self.assertEqual(h.c.area.search.phase, 'OBSERVATION_DWELL')
            self.assertEqual(n.active['phase'], 'paused')
        for _ in range(3):
            h.feedback(motion=2, pose=([88,13], 1800))
            h.c.area.tick(h.now + .002)
            if n.active['phase'] == 'resetting':
                break
        self.assertEqual(h.c.area.search.phase, 'NAVIGATING')
        self.assertTrue(h.c.area.search.active)
        self.assertEqual(n.active['phase'], 'resetting')
        self.assertNotEqual(n.active['command_id'], paused_id)
        self.assertEqual(n.active['start_cell'], [88,13])
        self.assertEqual(n.active['start_heading_deg'], 180.)
        self.assertEqual(len(h.c.area.search.snapshot(h.now)['remaining_waypoints']), 4)
        self.assertFalse(any(key == 'auto' or key == 'actuator' and value.get('action') in ('pump_on', 'arm')
                             for key, value in h.c.effects))
        self.assertFalse(n.state(h.now + .003)['failed'])
        self.assertFalse(n.state(h.now + .003)['arrived'])

    def test_region_request_cannot_silently_change_fire_authority_or_skip_voice_verification(self):
        for text, execute_fire in (('去C区域搜索火焰', True), ('在A区域搜寻火源并执行灭火', False)):
            h = Harness()
            request = dict(parse_task_text(text), execute_fire=execute_fire, command_id='changed-authority',
                           source='web', issued_at=h.now, ttl_s=3.)
            before = len(h.c.outbox)
            with self.assertRaisesRegex(ValueError, 'explicit_semantic_region_request_required'):
                h.c.request(request, h.now + .002)
            self.assertEqual(len(h.c.outbox), before)
        h = Harness()
        request = dict(parse_task_text('去C区域搜索火焰'), command_id='voice-region',
                       source='voice', issued_at=h.now, ttl_s=3.)
        with self.assertRaisesRegex(ValueError, 'independent_asr_required'):
            h.c.request(request, h.now + .002)
        request['voice_verification'] = {'verified': True}
        h.c.request(request, h.now + .003)
        self.assertTrue(h.c.area.search.active)

    def test_same_panel_position_new_confirmation_overrides_old_odometry(self):
        h=Harness();h.start();h.drive_leg();h.arrive()
        region=h.c.area
        original=region.navigation.map['start_reference']
        self.assertEqual(region.reference(h.now+.002)['source'],'fpga_encoder_imu_pose')
        original['generation']=2
        self.assertEqual(region.reference(h.now+.002)['cell'],[88,13])
        self.assertEqual(region.reference(h.now+.002)['source'],'fpga_touchscreen')

    def test_normal_finish_stop_is_not_reported_as_navigation_failure(self):
        h=Harness();h.start();h.drive_leg();h.arrive();n=h.c.area.navigation
        n.stop(h.now+.002,'no_fire_found');h.ack()
        h.feedback(motion=3,pose=(n.active['goal_cell'],900));n.tick(h.now+.002)
        self.assertEqual(n.active['phase'],'arrived')
        self.assertFalse(n.state(h.now+.003)['failed'])

    def test_goal_capture_tolerance_keeps_real_done_requirement(self):
        h=Harness();h.start();h.drive_leg();n=h.c.area.navigation;a=n.active
        h.now+=.01
        h.c.receive(wire.Packet(0,wire.DONE,bytes((a['task_id'],3,0,0))),h.now)
        goal=[a['goal_cell'][0]+3,a['goal_cell'][1]]
        h.feedback(motion=3,pose=(goal,900));n.tick(h.now+.002)
        self.assertTrue(n.state(h.now+.003)['arrived'])

    def test_all_four_waypoints_actual_correlated_arrival_then_no_fire_stop(self):
        h=Harness();h.start()
        ids=[]
        for _ in range(4):
            h.drive_leg();ids.append(h.c.area.navigation.active['task_id'])
            h.arrive()
            h.c.area.tick(h.now+.003)
            self.assertEqual(h.c.area.search.phase,'OBSERVING')
            for _ in range(33):
                h.feedback(motion=3,pose=(h.c.area.navigation.active['goal_cell'],900))
                h.c.area.tick(h.now+.003)
                if h.c.area.search.phase!='OBSERVING':break
        state=h.c.area.snapshot(h.now+.004)
        self.assertEqual(state['phase'],'COMPLETE')
        self.assertEqual(state['reason'],'no_fire_found')
        self.assertEqual(state['waypoints_visited'],4)
        self.assertEqual(len(set(ids)),4)
        self.assertFalse(any(k=='auto' and v.get('action')=='start' for k,v in h.c.effects))
        self.assertTrue(h.c.last_result['completed'])

    def test_ack_or_terminal_status_without_done_never_arrives(self):
        h=Harness();h.start();h.drive_leg()
        n=h.c.area.navigation
        h.feedback(motion=3,pose=(n.active['goal_cell'],900))
        n.tick(h.now+.002)
        self.assertFalse(n.state(h.now+.003)['arrived'])
        self.assertEqual(n.active['phase'],'running')

    def test_wrong_done_and_wrong_pose_cannot_advance(self):
        h=Harness();h.start();h.drive_leg();n=h.c.area.navigation
        h.c.receive(wire.Packet(0,wire.DONE,bytes((99,3,0,0))),h.now+.01)
        self.assertEqual(n.active['phase'],'running')
        h.c.receive(wire.Packet(0,wire.DONE,bytes((n.active['task_id'],3,0,0))),h.now+.012)
        h.feedback(motion=3,pose=([88,13],900));n.tick(h.now+.002)
        self.assertEqual(n.active['reason'],'area_terminal_pose_not_at_goal')
        self.assertFalse(n.state(h.now+.003)['arrived'])

    def test_pause_waits_for_wheel_stop_and_new_pose_before_replan(self):
        h=Harness();h.start();h.drive_leg();n=h.c.area.navigation
        n.pause(n.active['command_id'],h.now+.002);h.ack()
        h.feedback(motion=2,stopped=False);n.tick(h.now+.002)
        self.assertEqual(n.active['phase'],'pausing')
        h.feedback(motion=2,pose=([70,13],1500));n.tick(h.now+.002)
        self.assertEqual(n.active['phase'],'paused')
        ref=n.current_reference(h.now+.003)
        self.assertEqual((ref['cell'],ref['heading_deg']),([70,13],150.))
        n.start(dict(command_id='approach',goal_cell=[52,13],region_id='A',region_task_id='region-test'),h.now+.003)
        self.assertEqual((n.active['start_cell'],n.active['start_heading_deg']),([70,13],150.))
        self.assertEqual(h.c.outbox[-1][4],RESET_NAV)

    def test_preparation_without_pose_does_not_timeout_after_two_seconds(self):
        h=Harness();h.start();n=h.c.area.navigation
        h.ack();h.feedback(session=False);n.tick(h.now+.002)
        for _ in range(4):h.ack()
        for _ in range(80):
            h.feedback(session=False,point_state=1)
            h.c.area.tick(h.now+.002)
        self.assertEqual(n.active['phase'],'planning')
        self.assertTrue(h.c.area.search.active)

    def test_uncommissioned_geometry_and_nonoff_pump_block_before_upload(self):
        h=Harness();h.c.area.approach.readiness=lambda:'camera_to_vehicle_calibration_required'
        self.assertEqual(h.c.area.readiness(h.now+.002),'camera_to_vehicle_calibration_required')
        h.c.area.approach=GeometryFixture();h.c.actuator['pump_on']=True
        before=len(h.c.outbox)
        with self.assertRaisesRegex(ValueError,'pump_off'):
            h.start()
        self.assertEqual(len(h.c.outbox),before)

    def test_pose_reference_converts_world_y_without_claiming_absolute_fix(self):
        ref=pose_reference(dict(valid=True,stamp=100.,mode=9,status=3,task_id=7,
            x_mm=350,y_mm=2850,heading_tenths=1800),100.1,7)
        self.assertEqual((ref['cell'],ref['heading_deg']),([17,17],180.))
        self.assertFalse(ref['physical_position_verified'])


if __name__=='__main__':unittest.main()
