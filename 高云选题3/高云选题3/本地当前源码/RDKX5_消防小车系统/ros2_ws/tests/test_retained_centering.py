"""Bidirectional fire centering over an unchanged paused FPGA route.

Only synthetic UART/status/odometry is used; these tests never open hardware.
"""
import copy
import hashlib
import json
import math
import struct
import unittest

from test_continuous_area_navigation import ContinuousHarness, route, ROOT
from rescue_vision import ov5640_protocol as wire
from rescue_vision.area_navigation import RESET_NAV, SET_SPEED
from rescue_vision.retained_route import (route_local_corridor, local_coordinates,
    grid_line_reason, _checked_free_grid, checked_route, retained_span_line_reason)


def free_grid(blocked=(), signature=1153302534, map_hash='synthetic-map', nodes=None):
    """An explicitly inflated synthetic road mask; no real motion is run."""
    cells=bytearray(176*160)
    for x,y in blocked: cells[y*176+x]=1
    runs=[]
    for y in range(160):
        x=0
        while x<176:
            if cells[y*176+x]: x+=1; continue
            start=x
            while x<176 and not cells[y*176+x]: x+=1
            runs.append([y,start,x-start])
    document=dict(schema_version=1,width_cells=176,height_cells=160,cell_cm=2,
        free_value=0,reference='nose_midpoint',planner_inflation_m=.095,
        map_signature=signature,map_sha256=map_hash,free_runs=runs,
        cells_sha256=hashlib.sha256(cells).hexdigest())
    if nodes is not None:
        document.update(directed_nodes=nodes,directed_nodes_sha256=
            hashlib.sha256(json.dumps(nodes,separators=(',',':')).encode('ascii')).hexdigest())
    return document


def checked_corridor(points=None, anchor=None, grid=None, **kwargs):
    points=points or [[x,410] for x in range(10,2011,20)]
    anchor=anchor or dict(image_mm=[610,410],heading_deg=0.)
    grid=grid or free_grid()
    return route_local_corridor(route(points),anchor,33,30,600.,
        traversability=grid,map_signature=1153302534,map_sha256='synthetic-map',**kwargs)


class CenteringHarness(ContinuousHarness):
    def __init__(self, pause_cell=30):
        super().__init__()
        self.c.capabilities['retained_jog']=True
        self.launch(); self.sample(pause_cell); self.pause(pause_cell)
        self.n.begin_centering(self.now+.003)
        self.saved_cursor=self.cursor
        self.origin=list(self.n.centering.session['anchor']['image_mm'])

    def position(self, offset=0., *, stopped=True, heading=0., tick=True):
        point=[self.origin[0]+offset,self.origin[1]]
        self.now+=.10
        self.feedback(run=False,stopped=stopped,motion=2,
                      pose=([int(point[0]//20),int(point[1]//20)],round(heading*10)))
        self.pose.update(x_mm=round(point[0]),y_mm=3200-round(point[1]))
        if tick: self.n.tick(self.now+.002)

    def begin_jog(self, direction='forward', distance=40, speed=40, acknowledge=True):
        proposal=self.n.centering_proposal(direction,self.now+.003,distance,speed)
        assert proposal['valid'],proposal
        self.n.begin_centering_jog(proposal,self.now+.004)
        seq=self.n.active['sequence']
        assert self.n.centering_write_ready(seq,self.now+.005)
        if acknowledge: self.ack()
        return seq

    def jog_status(self, phase=3, reason=0, accepted=None, flags=3):
        session=self.n.centering.session
        if session['query_sequence'] is None:
            self.now=max(self.now,session['next_poll'])
            self.n.tick(self.now+.002)
        seq=session['query_sequence']
        assert seq is not None, self.n.active
        accepted=session['jog']['sequence'] if accepted is None else accepted
        self.now+=.01
        self.c.receive(wire.Packet(seq,wire.RETAINED_JOG_STATUS|128,
                                  bytes((phase,reason,accepted,flags))),self.now)

    def complete(self, offset, phase=3, reason=0):
        self.position(offset,stopped=False)
        self.jog_status(phase,reason)
        self.position(offset,stopped=True)
        assert self.n.active['phase']=='paused',self.n.active


class RetainedProtocolTests(unittest.TestCase):
    def test_v1_disables_jog_v2_advertises_it(self):
        self.assertFalse(wire.capabilities(bytes((0,1,63,2)))['retained_jog'])
        self.assertTrue(wire.capabilities(bytes((0,2,127,2)))['retained_jog'])
        for data in (bytes((0,1,127,2)),bytes((0,2,255,2)),bytes((0,3,127,2))):
            with self.assertRaises(ValueError): wire.capabilities(data)

    def test_signed_payload_and_status_validation(self):
        self.assertEqual(wire.retained_jog_payload(-40,20),struct.pack('<hH',-40,20))
        for distance,speed in ((0,40),(101,40),(-101,40),(20,19),(20,61),(True,40)):
            with self.assertRaises(ValueError): wire.retained_jog_payload(distance,speed)
        status=wire.retained_jog_status(bytes((4,1,23,3)))
        self.assertTrue(status['stopped']); self.assertTrue(status['nav_valid'])
        self.assertEqual(status['accepted_sequence'],23)
        for data in (bytes((5,0,2,3)),bytes((3,1,2,3)),bytes((4,9,2,3)),bytes((1,0,2,8))):
            with self.assertRaises(ValueError): wire.retained_jog_status(data)


class LocalCorridorTests(unittest.TestCase):
    def test_straight_both_directions_and_map_headings(self):
        for heading in (0,90,-90,180):
            angle=math.radians(heading)
            points=[[1500+i*math.cos(angle),1500-i*math.sin(angle)] for i in range(-1000,1001,20)]
            anchor=dict(image_mm=[1500,1500],heading_deg=heading)
            bounds=route_local_corridor(route(points),anchor,53,50,1000.)
            self.assertAlmostEqual(bounds['backward_mm'],-580.)
            self.assertAlmostEqual(bounds['forward_mm'],580.)
            forward,lateral=local_coordinates(anchor,points[52])
            self.assertAlmostEqual(forward,40.); self.assertAlmostEqual(lateral,0.)

    def test_bend_and_end_limit_each_direction(self):
        points=[[x,410] for x in range(10,611,20)]+[[610,y] for y in range(430,811,20)]
        anchor=dict(image_mm=[410,410],heading_deg=0.)
        bounds=route_local_corridor(route(points),anchor,23,20,400.)
        self.assertEqual(bounds['backward_mm'],-380.)
        self.assertEqual(bounds['forward_mm'],180.)
        beginning=route_local_corridor(route(points),dict(image_mm=[10,410],heading_deg=0.),3,0,0.)
        self.assertEqual(beginning['backward_mm'],0.)

    def test_verified_actual_line_accepts_initial_offset_and_yaw(self):
        for point,heading in (([610,470],0.),([610,410],13.),([610,470],13.)):
            with self.subTest(point=point,heading=heading):
                bounds=checked_corridor(anchor=dict(image_mm=point,heading_deg=heading))
                self.assertTrue(bounds['geometry_verified'])
                self.assertGreaterEqual(bounds['forward_mm'],40.)
                self.assertLessEqual(bounds['backward_mm'],-40.)
                self.assertEqual(bounds['lateral_limit_mm'],40.)
                self.assertEqual(bounds['heading_limit_deg'],8.)

    def test_no_grid_keeps_previous_conservative_limits(self):
        points=[[x,410] for x in range(10,2011,20)]
        for point,heading in (([610,470],0.),([610,410],13.)):
            bounds=route_local_corridor(route(points),dict(image_mm=point,heading_deg=heading),33,30,600.)
            self.assertFalse(bounds['geometry_verified'])
            self.assertEqual(bounds['forward_mm'],0.)
            self.assertEqual(bounds['backward_mm'],0.)

    def test_actual_line_wall_blocks_despite_clear_route_centerline(self):
        grid=free_grid(blocked=[(36,23)])
        bounds=checked_corridor(anchor=dict(image_mm=[610,470],heading_deg=0.),grid=grid)
        self.assertGreaterEqual(bounds['forward_mm'],40.)
        self.assertLessEqual(bounds['forward_mm'],90.)
        self.assertEqual(bounds['forward_reason'],'centering_corridor_static_obstacle')
        self.assertGreater(checked_corridor(grid=grid)['forward_mm'],500.)

    def test_grid_identity_and_corruption_fail_without_fallback(self):
        changes=(('hash',lambda d:d.update(cells_sha256='wrong')),
                 ('map_identity',lambda d:d.update(map_signature=1)),
                 ('map_identity',lambda d:d.update(map_sha256='wrong')),
                 ('overlapping',lambda d:d['free_runs'].append([20,5,2])))
        for reason,change in changes:
            with self.subTest(reason=reason):
                grid=free_grid();change(grid)
                bounds=checked_corridor(grid=grid)
                self.assertEqual([bounds['backward_mm'],bounds['forward_mm']],[0.,0.])
                self.assertFalse(bounds['geometry_verified'])
                self.assertIn(reason,bounds['reason'])

    def test_verified_span_stops_before_turn_and_route_endpoint(self):
        points=[[x,410] for x in range(10,811,20)]+[[810,y] for y in range(430,811,20)]
        bounds=checked_corridor(points=points)
        self.assertGreaterEqual(bounds['forward_mm'],150.)
        self.assertLessEqual(bounds['forward_mm'],180.)
        self.assertEqual(bounds['route_span_indices'],[0,40])
        endpoint=checked_corridor(points=points[:41])
        self.assertEqual(endpoint['forward_mm'],bounds['forward_mm'])

    def test_changed_actual_line_cannot_cross_captured_span_even_in_free_grid(self):
        points=[[x,410] for x in range(10,811,20)]+[[810,y] for y in range(430,811,20)]
        points,lengths=checked_route(route(points))
        self.assertEqual(retained_span_line_reason(points,lengths,[0,40],[710,410],[830,410],1),
                         'centering_retained_straight_span_ended')
        self.assertEqual(retained_span_line_reason(points,lengths,[0,40],[710,410],[770,410],1),'')
        self.assertEqual(retained_span_line_reason(points,lengths,[0,40],[10,410],[70,410],1),'')

    def test_verified_graph_branch_clips_span_and_anchor_at_branch_refuses(self):
        nodes=[[int(810*25.6),int(410*25.6),0,[[1,1],[2,1]]],
               [int(830*25.6),int(410*25.6),0,[]],
               [int(810*25.6),int(430*25.6),90,[]]]
        grid=free_grid(nodes=nodes)
        bounds=checked_corridor(grid=grid)
        self.assertLessEqual(bounds['forward_mm'],180.)
        self.assertEqual(bounds['route_span_indices'][1],40)
        near=route_local_corridor(route([[x,410] for x in range(10,2011,20)]),
            dict(image_mm=[810,410],heading_deg=0.),43,40,800.,traversability=grid,
            map_signature=1153302534,map_sha256='synthetic-map')
        self.assertEqual(near['forward_mm'],0.)
        self.assertEqual(near['reason'],'centering_anchor_at_route_branch')

    def test_live_barrier_is_inflated_and_corner_touch_is_not_free(self):
        bounds=checked_corridor(barriers=dict(valid=True,segments_mm=[[850,300,850,600]]))
        self.assertLessEqual(bounds['forward_mm'],120.)
        self.assertEqual(bounds['forward_reason'],'centering_corridor_live_barrier')
        cells,_=_checked_free_grid(free_grid(blocked=[(1,0)]),1153302534,'synthetic-map')
        self.assertEqual(grid_line_reason(cells,[10,10],[30,30]),'centering_corridor_static_obstacle')
        self.assertEqual(grid_line_reason(cells,[10,10],[10,30]),'')

    def test_current_planner_grid_and_directed_graph_validate(self):
        grid=json.loads((ROOT/'config/region_traversability.json').read_text(encoding='utf-8'))
        catalog=json.loads((ROOT/'config/semantic_regions.json').read_text(encoding='utf-8'))
        cells,branches=_checked_free_grid(grid,1153302534,catalog['map_sha256'])
        self.assertEqual(len(cells),176*160)
        self.assertGreater(len(branches),0)


class RetainedCenteringTests(unittest.TestCase):
    def test_queued_written_and_acknowledged_are_distinct_evidence(self):
        h=CenteringHarness();seq=h.begin_jog(acknowledge=False)
        state=h.n.centering.state(h.now+.006)
        self.assertEqual(state['command_sequence'],seq)
        self.assertIsNone(state['written_at'])
        self.assertIsNone(state['accepted_sequence'])
        self.assertIsNone(state['acknowledged_sequence'])
        h.n.centering.mark_written((seq+1)&255,h.now+.007)
        self.assertIsNone(h.n.centering.state(h.now+.008)['written_at'])
        h.n.centering.mark_written(seq,h.now+.007)
        self.assertEqual(h.n.centering.state(h.now+.008)['written_at'],h.now+.007)
        self.assertIsNone(h.n.centering.state(h.now+.008)['acknowledged_sequence'])
        h.ack();state=h.n.centering.state(h.now+.003)
        self.assertEqual(state['acknowledged_sequence'],seq)
        self.assertEqual(state['accepted_sequence'],seq)
        self.assertEqual(state['ack_at'],h.now)

    def test_actual_pose_and_live_barrier_rechecked_at_uart_write(self):
        for change in ('heading','lateral','barrier'):
            with self.subTest(change=change):
                h=CenteringHarness()
                h.c.area.approach.traversability=free_grid(map_hash=h.c.area.catalog['map_sha256'])
                h.n.centering.reset();h.n.begin_centering(h.now+.003)
                self.assertTrue(h.n.centering.state(h.now+.003)['corridor']['geometry_verified'])
                seq=h.begin_jog(acknowledge=False)
                if change=='heading': h.position(0,heading=9,tick=False)
                elif change=='lateral':
                    h.position(0,tick=False);h.pose['y_mm']-=41
                else: h.n.map['barriers']=dict(valid=True,segments_mm=[[750,300,750,600]])
                self.assertFalse(h.n.centering_write_ready(seq,h.now+.006))

    def test_forward_reverse_returns_anchor_and_resumes_original_route_same_tick(self):
        h=CenteringHarness(); a=h.n.active
        identity=(a['task_id'],a['route_generation'],h.cursor,a['progress_mm'],a['checkpoint_index'])
        h.begin_jog('forward',100); h.complete(100)
        self.assertEqual(h.n.state(h.now+.003)['centering']['offset_mm'],100.)
        h.begin_jog('reverse',100); h.complete(0)
        proposal=h.n.centering_return_proposal(h.now+.003)
        self.assertTrue(proposal['at_anchor'])
        h.n.end_centering(h.now+.004)
        h.n.resume('continuous-test',h.now+.004)
        self.assertTrue(h.n.resume_write_ready(h.n.active['sequence'],h.now+.005))
        self.assertEqual(identity,(a['task_id'],a['route_generation'],h.cursor,a['progress_mm'],a['checkpoint_index']))
        self.assertEqual(h.sent.count(RESET_NAV),1); self.assertEqual(h.sent.count(wire.PLAN),1)
        self.assertEqual(h.sent.count(wire.START),1); self.assertNotIn(SET_SPEED,h.sent)
        self.assertNotIn(wire.MOVE,h.sent); self.assertNotIn(wire.VELOCITY,h.sent)

    def test_crossed_reference_during_local_jog_does_not_advance_route_progress(self):
        h=CenteringHarness(pause_cell=18)
        before=(h.n.active['progress_mm'],h.n.active['checkpoint_index'])
        h.begin_jog('forward',100); h.complete(100)
        self.assertEqual(before,(h.n.active['progress_mm'],h.n.active['checkpoint_index']))
        h.begin_jog('reverse',100); h.complete(0)
        self.assertEqual(before,(h.n.active['progress_mm'],h.n.active['checkpoint_index']))

    def test_active_session_blocks_resume_even_at_anchor_until_explicit_end(self):
        h=CenteringHarness()
        with self.assertRaisesRegex(ValueError,'anchor_restored'): h.n.resume('continuous-test',h.now+.003)
        h.begin_jog(); h.complete(40)
        with self.assertRaisesRegex(ValueError,'original_anchor'): h.n.end_centering(h.now+.003)
        with self.assertRaisesRegex(ValueError,'anchor_restored'): h.n.resume('continuous-test',h.now+.003)

    def test_multiple_flames_preserve_original_anchor_and_return_in_segments(self):
        h=CenteringHarness(); original=copy.deepcopy(h.n.centering.session['anchor'])
        for offset in (100,200,300):
            h.begin_jog('forward',100); h.complete(offset)
            h.n.begin_centering(h.now+.003)
            self.assertEqual(h.n.centering.session['anchor'],original)
        for offset in (200,100,0):
            proposal=h.n.centering_return_proposal(h.now+.003)
            self.assertEqual(proposal['distance_mm'],-100)
            h.n.begin_centering_jog(proposal,h.now+.004); h.ack(); h.complete(offset)
        self.assertTrue(h.n.state(h.now+.003)['centering']['return_ready'])
        self.assertEqual(h.n.centering.session['travelled_mm'],600.)

    def test_string_direction_is_explicit_not_default_forward(self):
        h=CenteringHarness()
        self.assertEqual(h.n.centering_proposal('reverse',h.now+.003)['distance_mm'],-40)
        for direction in ('left','',None,True,0):
            self.assertFalse(h.n.centering_proposal(direction,h.now+.003)['valid'])

    def test_firmware_pump_and_changed_proposal_rejected(self):
        h=CenteringHarness(); h.n.centering.reset(); h.c.capabilities['retained_jog']=False
        with self.assertRaisesRegex(ValueError,'firmware'): h.n.begin_centering(h.now+.003)
        h.c.capabilities['retained_jog']=True; h.n.begin_centering(h.now+.003)
        proposal=h.n.centering_proposal('forward',h.now+.003)
        proposal['session_id']='other'
        with self.assertRaisesRegex(ValueError,'proposal_changed'): h.n.begin_centering_jog(proposal,h.now+.004)
        h.c.actuator['pump_on']=True
        self.assertFalse(h.n.centering_proposal('forward',h.now+.003)['valid'])

    def test_old_status_and_stale_pose_do_not_complete_current_jog(self):
        h=CenteringHarness(); seq=h.begin_jog(); h.position(40,stopped=False)
        h.jog_status(accepted=(seq-1)&255)
        self.assertEqual(h.n.active['phase'],'centering_moving')
        h.position(40,stopped=False); h.jog_status()
        self.assertEqual(h.n.active['phase'],'centering_verifying')
        h.n.tick(h.now+.002)
        self.assertEqual(h.n.active['phase'],'centering_verifying')
        h.position(40)
        self.assertEqual(h.n.active['phase'],'paused')

    def test_early_stop_needs_matching_abort_and_measured_standstill(self):
        h=CenteringHarness(); h.begin_jog(); h.position(15,stopped=False)
        h.n.stop_centering_jog(h.now+.003); h.ack(); h.jog_status(4,1)
        self.assertEqual(h.n.active['phase'],'centering_verifying')
        h.position(15)
        self.assertEqual(h.n.active['phase'],'paused')
        self.assertEqual(h.n.centering.session['travelled_mm'],15.)

    def test_stop_before_3c_acceptance_requires_post_stop_status_and_unchanged_pose(self):
        h=CenteringHarness(); seq=h.begin_jog(acknowledge=False)
        h.n.stop_centering_jog(h.now+.005)
        self.assertFalse(h.n.centering_write_ready(seq,h.now+.006))
        h.ack(); h.position(0)
        h.jog_status(phase=0,accepted=(seq-1)&255)
        self.assertEqual(h.n.active['phase'],'centering_verifying')
        h.position(0)
        self.assertEqual(h.n.active['phase'],'paused')
        self.assertTrue(h.n.centering.session['jog']['cancelled_before_accept'])
        self.assertEqual(h.n.centering.session['travelled_mm'],0.)

    def test_preaccept_cancel_cannot_hide_actual_movement(self):
        h=CenteringHarness(); seq=h.begin_jog(acknowledge=False)
        h.n.stop_centering_jog(h.now+.005); h.ack(); h.position(10)
        h.jog_status(phase=0,accepted=(seq-1)&255); h.position(10)
        self.assertEqual(h.n.active['phase'],'centering_stopping')

    def test_changed_cursor_pump_heading_and_wrong_direction_fail_closed(self):
        for change in ('cursor','pump','heading','reverse'):
            with self.subTest(change=change):
                h=CenteringHarness(); h.begin_jog()
                if change=='cursor':
                    h.cursor+=1
                    for _ in range(11): h.position(0,stopped=False)
                elif change=='pump':
                    h.position(0,stopped=False,tick=False); h.c.actuator['pump_on']=True
                    h.n.tick(h.now+.002)
                elif change=='heading': h.position(0,stopped=False,heading=9)
                else: h.position(-11,stopped=False)
                self.assertEqual(h.n.active['phase'],'stopping')
                self.assertFalse(h.n.centering.active)

    def test_abort_reason_and_unmeasured_endpoint_cannot_be_success(self):
        for phase,reason,position in ((4,5,20),(3,0,20)):
            h=CenteringHarness(); h.begin_jog(); h.position(position,stopped=False)
            h.jog_status(phase,reason)
            if h.n.active['phase']!='stopping': h.position(position)
            self.assertEqual(h.n.active['phase'],'stopping')

    def test_budget_reserves_return_and_corridor_does_not_allow_overrun(self):
        h=CenteringHarness(); h.n.centering.session['travelled_mm']=2350.
        self.assertFalse(h.n.centering_proposal('forward',h.now+.003)['valid'])
        h.n.centering.session['travelled_mm']=0.
        for offset in (100,200,300,400,500):
            h.begin_jog('forward',100); h.complete(offset)
        proposal=h.n.centering_proposal('forward',h.now+.003,100)
        self.assertEqual(proposal['distance_mm'],80)
        h.n.begin_centering_jog(proposal,h.now+.004); h.ack(); h.complete(580)
        self.assertFalse(h.n.centering_proposal('forward',h.now+.003)['valid'])
        self.assertTrue(h.n.centering_return_proposal(h.now+.003)['valid'])

    def test_stop_ack_timeout_and_sequence_wrap_do_not_restart_old_jog(self):
        h=CenteringHarness(); seq=h.begin_jog(); h.complete(40)
        h.c.sequence=(seq-1)&255
        next_seq=h.begin_jog('reverse')
        self.assertNotEqual(next_seq,seq)
        h.n.stop_centering_jog(h.now+.003)
        for _ in range(15): h.position(40)
        self.assertEqual(h.n.active['phase'],'stopping')
        self.assertEqual(h.n.active['reason'],'centering_stop_ack_timeout')


if __name__=='__main__': unittest.main()
