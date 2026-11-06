"""A retained regional route passes references without stopping or replanning.

UART and odometry are synthetic, including when replaying real RTL path CSVs.
Nothing here opens a physical serial port or operates the vehicle.
"""
import copy
import csv
import json
import math
import struct
import unittest
from pathlib import Path

from test_area_navigation import Harness, ROOT
from rescue_vision import ov5640_protocol as wire
from rescue_vision.area_navigation import RESET_NAV, SET_SPEED
from rescue_vision.retained_route import waypoint_references, project_progress, forward_proposal


def route(points, generation=42, stamp=100.):
    return dict(valid=True, sampled=False, generation=generation, stamp=stamp,
                total_count=len(points), sample_indices=list(range(len(points))), points_mm=points)


class ContinuousHarness(Harness):
    def __init__(self, speed=200, points=None, waypoints=None, start=(0,20), heading=0., goal=(100,20)):
        self.cruise = speed
        self.route_data = route(points or [[x,410] for x in range(10,2011,20)], generation=41)
        self.cursor = 0
        super().__init__()
        self.n = self.c.area.navigation
        self.sent = []
        self.n.start(dict(command_id='continuous-test', execute_fire=False, continuous_route=True,
            patrol_waypoints=waypoints or [dict(id='A'+str(i),cell=[i*20,20]) for i in range(1,5)],
            goal_cell=list(goal), start_reference=dict(valid=True,cell=list(start),heading_deg=heading)),self.now+.001)

    def feedback(self, **kwargs):
        super().feedback(**kwargs)
        if self.pose:
            self.pose.update(execution_index=self.cursor,route_generation=self.route_data['generation'])
        self.route_data['stamp'] = self.now
        self.c.area.navigation.map.update(continuous_route_supported=True,route=copy.deepcopy(self.route_data),pose=self.pose)

    def ack(self, result=0):
        self.now += .01
        a=self.n.active; seq=a['sequence']; command=self.c.pending[seq][0]
        self.sent.append(command)
        data=bytes((result,0,0,0))
        if not result:
            if command == RESET_NAV: data=bytes((0,0))+struct.pack('<H',self.cruise)
            elif command == SET_SPEED:
                speed=self.n.restore_speed_mm_s if a['phase']=='restoring_speed' else 60
                data=bytes((0,0))+struct.pack('<H',speed)
            elif command == wire.SET_START_HEADING:
                data=bytes((0,))+wire.heading_payload(a['start_heading_deg'])[:2]+bytes((0,))
        self.c.receive(wire.Packet(seq,command|128,data),self.now)
        return command

    def launch(self, complete=True):
        self.ack(); self.feedback(session=False); self.n.tick(self.now+.002)
        while self.n.active['phase']!='planning': self.ack()
        self.route_data.update(generation=42,sampled=not complete)
        self.feedback(session=False,point_state=2); self.n.tick(self.now+.002); self.n.tick(self.now+.003)
        if complete:
            self.ack()
            self.sample_position(self.route_data['points_mm'][0],self.n.active['start_heading_deg'],0)

    def sample_position(self, point, heading, index, *, run=True, motion=2):
        self.cursor=min(len(self.route_data['points_mm'])-1,index+3)
        self.now+=.15  # Together with Harness.feedback, use 5 Hz pose samples.
        self.feedback(run=run,stopped=not run,motion=motion,
                      pose=([int(point[0]//20),int(point[1]//20)],round(heading*10)))
        self.pose.update(x_mm=round(point[0]),y_mm=3200-round(point[1]))
        self.n.tick(self.now+.002)

    def sample(self, x, heading=0., run=True):
        self.sample_position([x*20+10,410],heading,x,run=run)

    def pause(self, x):
        self.n.pause('continuous-test',self.now+.003); self.ack(); self.sample(x,run=False)
        assert self.n.active['phase']=='paused',self.n.active['reason']

    def resume(self, x):
        self.n.resume('continuous-test',self.now+.003)
        assert self.n.resume_write_ready(self.n.active['sequence'],self.now+.004)
        self.ack(); self.sample(x)
        assert self.n.active['phase']=='running',self.n.active['reason']

    def done(self, point=None, heading=0.):
        self.now+=.01
        self.c.receive(wire.Packet(0,wire.DONE,bytes((self.n.active['task_id'],3,0,0))),self.now)
        self.sample_position(point or self.route_data['points_mm'][-1],heading,
                             len(self.route_data['points_mm'])-1,run=False,motion=3)


class ContinuousReferenceTests(unittest.TestCase):
    def test_four_references_pass_with_one_plan_no_stop_resume_or_speed_change(self):
        for speed in (100,200,1000):
            with self.subTest(speed=speed):
                h=ContinuousHarness(speed=speed); h.launch(); task=h.n.active['task_id']
                for x in range(1,101):
                    h.sample(x)
                    state=h.n.state(h.now+.003)
                    self.assertEqual(state['phase'],'running')
                    self.assertEqual(state['passed_waypoint_count'],min(4,x//20))
                    self.assertFalse(state['checkpoint_ready'])
                h.done()
                self.assertEqual(h.n.active['phase'],'arrived')
                self.assertEqual(h.n.active['task_id'],task)
                self.assertEqual(h.n.state(h.now+.003)['passed_waypoint_ids'],['A1','A2','A3','A4'])
                for command in (RESET_NAV,wire.PLAN,wire.START): self.assertEqual(h.sent.count(command),1)
                for command in (wire.STOP,wire.RESUME,SET_SPEED): self.assertNotIn(command,h.sent)

    def test_turning_heading_at_reference_does_not_require_straight_stop(self):
        h=ContinuousHarness(); h.launch(); h.sample(19,45.); h.sample(20,30.); h.sample(21,15.)
        self.assertEqual(h.n.active['phase'],'running')
        self.assertEqual(h.n.active['checkpoint_index'],1)
        self.assertNotIn(wire.STOP,h.sent)
        self.assertNotIn(SET_SPEED,h.sent)

    def test_two_fire_pauses_resume_same_route_and_finish_every_reference(self):
        h=ContinuousHarness(); h.launch(); task=h.n.active['task_id']
        for x in range(1,101):
            h.sample(x)
            if x in (15,55):
                before=h.n.active['checkpoint_index']; h.pause(x)
                for _ in range(10): h.sample(x,run=False)
                self.assertEqual(h.n.active['checkpoint_index'],before)
                h.resume(x)
        h.done()
        self.assertEqual(h.n.active['phase'],'arrived')
        self.assertEqual(h.n.active['task_id'],task)
        self.assertEqual(h.n.active['checkpoint_index'],4)
        self.assertEqual(h.sent.count(wire.STOP),2)
        self.assertEqual(h.sent.count(wire.RESUME),2)
        self.assertEqual(h.sent.count(wire.PLAN),1)

    def test_missing_cursor_does_not_fabricate_waypoint_passage(self):
        h=ContinuousHarness(); h.launch(); h.sample(19)
        h.feedback(run=True,stopped=False,motion=2,pose=([20,20],0))
        h.n.map['pose']['valid']=False; h.n.tick(h.now+.002)
        self.assertEqual(h.n.active['checkpoint_index'],0)
        self.assertEqual(h.n.active['phase'],'running')
        h.sample(20)
        self.assertEqual(h.n.active['checkpoint_index'],1)

    def test_repeated_pose_does_not_duplicate_passage(self):
        h=ContinuousHarness(); h.launch(); h.sample(20)
        for _ in range(5): h.n.tick(h.now+.003)
        self.assertEqual(h.n.active['passed_waypoint_ids'],['A1'])
        self.assertEqual(len(h.n.active['waypoint_passages']),1)

    def test_reference_cannot_be_manually_consumed_by_old_checkpoint_api(self):
        h=ContinuousHarness(); h.launch(); h.sample(15); h.pause(15)
        with self.assertRaisesRegex(ValueError,'measured_route_progress'):
            h.n.advance_checkpoint(h.now+.003)
        self.assertEqual(h.n.active['checkpoint_index'],0)

    def test_changed_route_pump_on_or_moved_paused_pose_cannot_resume(self):
        for change in ('route','pump','pose'):
            h=ContinuousHarness(); h.launch(); h.sample(15); h.pause(15)
            if change=='route': h.n.map['route']['generation']=99
            elif change=='pump': h.c.actuator['pump_on']=True
            else: h.sample(20,run=False)
            with self.assertRaisesRegex(ValueError,'same_route_stop_and_pump_off'):
                h.n.resume('continuous-test',h.now+.003)

    def test_download_waits_stationary_and_recovers_before_240_second_limit(self):
        h=ContinuousHarness(); h.launch(complete=False)
        for _ in range(300):
            h.now+=.4; h.feedback(session=False,point_state=2); h.n.tick(h.now+.002)
        self.assertEqual(h.n.active['phase'],'continuous_route_check')
        self.assertNotIn(wire.START,h.sent)
        h.route_data['sampled']=False; h.feedback(session=False,point_state=2); h.n.tick(h.now+.002)
        self.assertEqual(h.n.active['phase'],'starting')

    def test_incomplete_download_never_starts_after_timeout(self):
        h=ContinuousHarness(); h.launch(complete=False); h.now+=240.
        h.feedback(session=False,point_state=2); h.n.tick(h.now+.002)
        self.assertEqual(h.n.active['phase'],'stopping')
        self.assertNotIn(wire.START,h.sent)

    def test_explicit_fire_alignment_can_cross_reference_without_new_plan(self):
        h=ContinuousHarness(); h.launch(); h.sample(15); h.pause(15)
        proposal=h.n.alignment_proposal(h.now+.003,30.)
        self.assertTrue(proposal['valid'],proposal)
        self.assertGreater(proposal['goal_cell'][0],20)
        h.n.begin_retained_alignment(proposal,h.now+.004); h.ack(); h.ack(); h.sample(15)
        for x in range(16,proposal['goal_cell'][0]+1):
            h.sample(x)
            if h.n.active['phase']=='pausing': break
        self.assertEqual(h.n.active['checkpoint_index'],1)
        h.ack(); h.sample(x,run=False); h.ack()
        self.assertTrue(h.n.state(h.now+.003)['retained_alignment_finished'])
        self.assertEqual(h.n.restore_speed_mm_s,None)
        self.assertEqual(h.sent.count(wire.PLAN),1)


class ReferenceGeometryTests(unittest.TestCase):
    def test_corner_reference_is_not_shifted_or_rejected_for_clearance(self):
        points=[[x,410] for x in range(10,211,20)]+[[210,y] for y in range(430,611,20)]
        _,_,refs=waypoint_references(route(points),[dict(id='corner',cell=[10,20])])
        self.assertEqual(refs[0]['point_mm'],[210,410])
        self.assertTrue(refs[0]['reference_only'])
        self.assertNotIn('clearance_before_mm',refs[0])

    def test_lookahead_cursor_alone_does_not_claim_passage(self):
        points=[[x,410] for x in range(10,2011,20)]
        _,lengths,refs=waypoint_references(route(points),[dict(id='p',cell=[20,20])])
        _,progress,_=project_progress(points,lengths,[90,410],30)
        self.assertEqual(progress,80.)
        self.assertLess(progress,refs[0]['progress_mm'])

    def test_short_alignment_uses_real_route_tangent_and_preserves_exit(self):
        points=[[float(x),266.3] for x in range(1760,1000,-45)]
        proposal=forward_proposal(route(points),dict(valid=True,cell=[79,13],image_mm=[1590,270],heading_deg=180.),6,3,150.,100.,30.)
        self.assertTrue(proposal['valid'])
        short=[[x,410] for x in range(10,611,20)]
        near_exit=forward_proposal(route(short),dict(valid=True,cell=[22,20],heading_deg=0.),25,22,440.,100.,30.)
        self.assertFalse(near_exit['valid'])


class ActualRtlReferenceReplayTests(unittest.TestCase):
    def test_four_exported_rtl_routes_have_zero_intermediate_stops_without_fire(self):
        catalog=json.loads((ROOT/'config/semantic_regions.json').read_text(encoding='utf8'))
        orders={'A':['A2','A1','A4','A3'],'B':['B3','B1','B4','B2'],'C':['C4','C1','C3','C2'],'D':['D2','D1','D4','D3']}
        exits={'A':[96,13],'B':[79,13],'C':[96,147],'D':[79,147]}
        for region,order in orders.items():
            with self.subTest(region=region):
                source=ROOT.parent.parent/'ov5640/diagnostics/continuous_route_20261103'/f'{region.lower()}_route.csv'
                with source.open(encoding='utf-8-sig') as data:
                    rows=list(csv.DictReader(data))
                points=[[int(r['x_mm']),int(r['y_mm'])] for r in rows]
                by_id={p['id']:p for p in catalog['regions'][region]['waypoints']}
                h=ContinuousHarness(points=points,waypoints=[by_id[k] for k in order],start=(88,13),heading=180.,goal=exits[region])
                h.launch()
                for index,(p,q) in enumerate(zip(points,points[1:])):
                    distance=math.dist(p,q)
                    if not distance: continue
                    direction=math.degrees(math.atan2(p[1]-q[1],q[0]-p[0]))
                    steps=max(1,math.ceil(distance/20.))
                    for step in range(1,steps+1):
                        point=[round(p[k]+(q[k]-p[k])*step/steps) for k in (0,1)]
                        h.sample_position(point,direction,index+1)
                        self.assertEqual(h.n.active['phase'],'running',h.n.active['reason'])
                self.assertEqual(h.n.active['passed_waypoint_ids'],order)
                h.done(points[-1],direction)
                self.assertEqual(h.n.active['phase'],'arrived')
                for command in (wire.STOP,wire.RESUME,SET_SPEED): self.assertNotIn(command,h.sent)
                for command in (RESET_NAV,wire.PLAN,wire.START): self.assertEqual(h.sent.count(command),1)


if __name__=='__main__': unittest.main()
