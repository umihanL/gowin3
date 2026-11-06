"""One complete patrol passes waypoints and pauses only for confirmed fires."""
import json
from pathlib import Path
import unittest
from test_checkpoint_alignment import view
from test_region_search import actuator, geometry, CATALOG
from rescue_vision.region_search import RegionSearch
from rescue_vision.region_route import RegionRoutePlanner
ROOT = Path(__file__).resolve().parents[1]

class CountedPlanner(RegionRoutePlanner):
    def __init__(self):
        graph = json.loads((ROOT / 'config/region_traversability.json').read_text(encoding='utf8'))
        super().__init__(graph, CATALOG, graph['map_signature'])
        self.calls = 0
    def plan(self, *args):
        self.calls += 1
        return super().plan(*args)

class RetainedSearchHarness:
    def __init__(self, region='A', execute_fire=False):
        self.now, self.index = 100., 0
        self.planner = CountedPlanner()
        self.search = RegionSearch(CATALOG, self.planner, dict(enabled=True, forward_image_side='right'))
        self.history = self.search.start(dict(action='search_region', region_id=region,
            execute_fire=execute_fire, command_id='whole-tour', continuous_route=True), self.now,
            dict(valid=True, cell=[88, 13], heading_deg=180.))
        self.command = self.search.leg_id
    def tick(self, *, index=None, targets=(), phase='running', dt=.2, approach=None,
             nav_change=None, frame_change=None, auto=None, output=None, centering_proposal=None):
        self.now = round(self.now + dt, 8)
        s = self.search
        if index is not None: self.index = index
        nav = dict(stamp=self.now, command_id=self.command, phase=phase,
            stopped=phase in ('paused', 'arrived'), pose_valid=True,
            pose_cell=list(s.exit['cell'] if phase == 'arrived' else s.goal_cell),
            pose_heading_deg=0., arrived=phase == 'arrived', retained_route=True,
            checkpoint_ready=False, checkpoint_index=self.index,
            passed_waypoint_count=self.index,
            passed_waypoint_ids=[p['id'] for p in s.waypoints[:self.index]])
        nav.update(nav_change or {})
        document = view(self.now, targets)
        document.update(frame_change or {})
        proposal = geometry(self.now, **approach) if approach is not None else None
        effects = s.tick(self.now, nav, document, auto or {}, output or actuator(self.now), proposal, centering_proposal=centering_proposal)
        self.history += effects
        return effects
    def detect(self, target=1, center=.65):
        for _ in range(3): self.tick(targets=((target, center),))
        assert self.search.phase == 'WAIT_STOP', self.search.snapshot(self.now)

class BatchHarness(RetainedSearchHarness):
    """Synthetic side camera and signed local moves, no device access."""
    def __init__(self, execute_fire=False, targets=None):
        super().__init__(execute_fire=execute_fire)
        self.position_mm = 0
        self.phase = 'running'
        self.scene = targets or {1: .72, 2: .28}
        self.pending_distance = None
        self.auto_since = None
        self.centered_stamps = {}
        self.frame_number = 0
    def step(self, *, frame_change=None, nav_change=None, proposal_change=None, output=None):
        if self.phase == 'pausing':
            self.phase = 'paused'
        elif self.phase == 'centering_moving':
            self.position_mm += self.pending_distance
            self.pending_distance = None
            self.phase = 'paused'
        s = self.search
        targets = [(key, x-.002*self.position_mm) for key,x in self.scene.items()
                   if .03 <= x-.002*self.position_mm <= .97]
        proposal = None
        if s.phase == 'CENTER_SELECT' and s.centering.current:
            direction = s.centering.relation(self.now)
            if direction in ('forward','reverse'):
                error = abs(s.centering.observation(self.now)['center_x_norm']-.5)
                distance = (20 if error < .12 else 40)*(1 if direction=='forward' else -1)
                proposal = dict(stamp=round(self.now+.2,8),valid=True,distance_mm=distance,speed_mm_s=20)
        elif s.phase in ('CENTER_RETURN','CENTER_REACQUIRE'):
            distance = min(100,abs(self.position_mm))*(-1 if self.position_mm>0 else 1)
            proposal = dict(stamp=round(self.now+.2,8),valid=True,at_anchor=abs(self.position_mm)<=20,
                            distance_mm=distance,speed_mm_s=40)
        if proposal_change is not None:
            proposal = dict(stamp=round(self.now+.2,8),**proposal_change)
        auto = None
        if s.phase in ('EXTINGUISHING','WAIT_PUMP_OFF'):
            auto = dict(stamp=round(self.now+.2,8),command_id=s.run_id,active=False,
                phase='COMPLETE',reason='one_target_round_finished',burst_completed=True,
                saw_pump_on=True,confirmed_pump_off_at=round(self.now+.1,8))
        approach = dict(target=s.target[1],in_spray_range=True,
            fire_world_m=[.4+(s.target[1]-1)*.3,2.7,.35]) if s.execute_fire and s.target else None
        self.frame_number += 1
        document = dict(capture_stamp_ns=round((self.now+.2)*1e9),sequence=self.frame_number,coordinate_frame='left_rectified')
        document.update(frame_change or {})
        nav = dict(centering=dict(active=True,offset_mm=self.position_mm,
            return_ready=abs(self.position_mm)<=20 and self.phase=='paused'))
        nav.update(nav_change or {})
        effects = self.tick(targets=targets,phase=self.phase,centering_proposal=proposal,
            nav_change=nav,frame_change=document,approach=approach,auto=auto,output=output)
        for key,value in effects:
            if key == 'pause': self.phase = 'pausing'
            elif key == 'center_jog':
                assert self.phase == 'paused'
                self.pending_distance = value['distance_mm']; self.phase = 'centering_moving'
            elif key == 'center_stop':
                self.pending_distance = None; self.phase = 'paused'
            elif key == 'end_centering': assert abs(self.position_mm)<=20
            elif key == 'resume_retained': self.phase = 'running'
        if s.phase == 'CENTER_DWELL':
            self.centered_stamps.setdefault(s.target,[]).append((self.now,self.phase,s.centering.relation(self.now)))
        return effects
    def run_batch(self):
        for _ in range(220):
            self.step()
            if any(k=='resume_retained' for k,_ in self.history): return
            assert self.search.active, self.search.reason
        raise AssertionError(self.search.snapshot(self.now))

class RegionalRetainedTests(unittest.TestCase):
    def test_trigger_then_empty_scene_resumes_same_route_without_any_jog_or_done_claim(self):
        h=BatchHarness(targets={375:.65})
        for _ in range(8):
            h.step()
            if h.search.phase=='WAIT_STOP':break
        self.assertEqual(h.search.phase,'WAIT_STOP')
        trigger_time=h.now;h.scene={}
        for _ in range(20):
            h.step()
            if any(key=='resume_retained' for key,_ in h.history):break
        self.assertEqual(h.phase,'running')
        self.assertLess(h.now-trigger_time,4.)
        self.assertEqual(h.search.centering.snapshot(h.now)['completed_count'],0)
        self.assertFalse(any(key=='center_jog' for key,_ in h.history))
        self.assertEqual(sum(key=='navigate' for key,_ in h.history),1)
        self.assertEqual(sum(key=='resume_retained' for key,_ in h.history),1)
        self.assertIn(('camera-session',375),h.search.excluded)
        self.assertEqual(h.search.skipped_observations[-1]['reason'],'no_flame_visible_after_verified_stop')

    def test_same_view_id_churn_after_pause_keeps_seed_and_moves_to_center(self):
        h=BatchHarness(targets={375:.72})
        for _ in range(8):
            h.step()
            if h.search.phase=='WAIT_STOP':break
        for identifier in (383,384,386,386,386,386):
            h.scene={identifier:.72};h.step()
        h.run_batch()
        self.assertEqual(h.search.centering.snapshot(h.now)['completed_count'],1)
        self.assertEqual(len(h.search.centering.queue),1)
        row=h.search.centering.queue[0]
        self.assertEqual(row['original_target_id'],375)
        self.assertEqual(row['target_id'],386)
        self.assertTrue(any(key=='center_jog' and value['distance_mm']>0 for key,value in h.history))
        self.assertTrue(any(key=='center_jog' and value['distance_mm']<0 for key,value in h.history))
        self.assertIn(('camera-session',375),h.search.handled)
        self.assertIn(('camera-session',386),h.search.handled)
        self.assertEqual(sum(key=='resume_retained' for key,_ in h.history),1)

    def test_two_flames_with_brief_empty_frame_still_center_each_before_resume(self):
        h=BatchHarness(targets={1:.72,2:.28});injected=False
        for _ in range(220):
            if h.search.phase=='CENTER_COLLECT' and not injected:
                original=h.scene;h.scene={};h.step();h.scene=original;injected=True
            else:h.step()
            if any(key=='resume_retained' for key,_ in h.history):break
        self.assertTrue(injected)
        self.assertEqual(h.search.centering.snapshot(h.now)['completed_count'],2)
        self.assertEqual(len(h.search.centering.queue),2)
        self.assertEqual(h.phase,'running')
        self.assertTrue(any(key=='center_jog' and value['distance_mm']>0 for key,value in h.history))
        self.assertTrue(any(key=='center_jog' and value['distance_mm']<0 for key,value in h.history))

    def test_all_regions_pass_four_waypoints_without_midroute_stops(self):
        for region in 'ABCD':
            h=RetainedSearchHarness(region)
            for index in range(1,5): self.assertEqual(h.tick(index=index),[])
            self.assertEqual(h.search.phase,'EXITING')
            self.assertFalse(any(k in ('pause','stop','resume_retained') for k,_ in h.history))
            h.tick(phase='arrived')
            self.assertEqual(h.search.phase,'COMPLETE'); self.assertEqual(h.planner.calls,1)
    def test_all_left_and_right_flames_center_then_return_and_resume_one_route(self):
        h=BatchHarness();h.run_batch()
        s=h.search
        self.assertEqual(s.handled,{('camera-session',1),('camera-session',2)})
        self.assertEqual(s.centering.snapshot(h.now)['completed_count'],2)
        self.assertTrue(any(v['distance_mm']>0 for k,v in h.history if k=='center_jog'))
        self.assertTrue(any(v['distance_mm']<0 for k,v in h.history if k=='center_jog'))
        self.assertLessEqual(abs(h.position_mm),20)
        self.assertEqual(sum(k=='navigate' for k,_ in h.history),1)
        self.assertEqual(sum(k=='resume_retained' for k,_ in h.history),1)
        self.assertEqual(s.leg_id,h.command);self.assertEqual(s.visited,[])
        for identity,samples in h.centered_stamps.items():
            self.assertTrue(all(phase=='paused' and relation=='center' for _,phase,relation in samples))
            finished=next(row['finished_at'] for row in s.centering.queue if row['target_id']==identity[1])
            self.assertGreaterEqual(finished-samples[0][0]+1e-6,1.)
        self.assertFalse(any(k=='auto' and v.get('action')=='start' for k,v in h.history))
    def test_handled_flames_do_not_restart_batch_after_cooldown(self):
        h=BatchHarness();h.run_batch()
        for _ in range(30):h.step()
        self.assertEqual(sum(k=='pause' for k,_ in h.history),1)
        self.assertEqual(h.search.phase,'NAVIGATING')
    def test_four_visible_flames_are_each_observed_before_original_route_resume(self):
        h=BatchHarness(targets={1:.12,2:.38,3:.65,4:.88});h.run_batch()
        self.assertEqual(h.search.handled,{('camera-session',i) for i in range(1,5)})
        self.assertEqual(h.search.centering.snapshot(h.now)['completed_count'],4)
        self.assertEqual(h.search.skipped_observations,[])
        self.assertEqual(sum(k=='resume_retained' for k,_ in h.history),1)
        self.assertEqual(h.planner.calls,1)
        self.assertLessEqual(abs(h.position_mm),20)
        for row in h.search.centering.queue:
            samples=h.centered_stamps[(row['registry_session'],row['target_id'])]
            self.assertGreaterEqual(row['finished_at']-samples[0][0]+1e-6,1.)
    def test_offscreen_queued_flame_is_reacquired_from_anchor_then_centered(self):
        h=BatchHarness(targets={1:.1,2:.9});h.run_batch()
        self.assertEqual(h.search.handled,{('camera-session',1),('camera-session',2)})
        self.assertTrue(any(row.get('reacquire_attempted') for row in h.search.centering.queue))
        self.assertEqual(h.search.skipped_observations,[])
        self.assertLessEqual(abs(h.position_mm),20)
    def test_new_fire_after_cooldown_is_eligible(self):
        h=BatchHarness(targets={1:.5});h.run_batch()
        h.scene={3:.75}
        for _ in range(10):h.step()
        self.assertEqual(sum(k=='pause' for k,_ in h.history),1)
        for _ in range(10):h.step()
        self.assertEqual(sum(k=='pause' for k,_ in h.history),2)
    def test_reacquired_tracker_id_requires_fresh_anchor_evidence_and_is_retired(self):
        h=BatchHarness(targets={1:.1,2:.9})
        for _ in range(100):
            h.step()
            if h.search.phase=='CENTER_REACQUIRE': break
        self.assertEqual(h.search.phase,'CENTER_REACQUIRE')
        original=h.search.centering.identity
        h.scene[31]=h.scene.pop(original[1])
        h.run_batch()
        row=next(row for row in h.search.centering.queue if row['target_id']==31)
        proof=row['reassociation']
        self.assertEqual(row['status'],'completed')
        self.assertGreaterEqual(len(proof['capture_stamps']),3)
        self.assertTrue(all(stamp>proof['anchor_since'] for stamp in proof['capture_stamps']))
        self.assertFalse(proof['physical_identity_verified'])
        self.assertIn(original,h.search.handled)
        self.assertIn(('camera-session',31),h.search.handled)
        self.assertEqual(h.planner.calls,1)
        self.assertLessEqual(abs(h.position_mm),20)
    def test_ambiguous_reappearing_flames_are_not_falsely_completed(self):
        h=BatchHarness(targets={1:.1,2:.9})
        for _ in range(100):
            h.step()
            if h.search.phase=='CENTER_REACQUIRE': break
        original=h.search.centering.identity
        location=h.scene.pop(original[1])
        h.scene.update({31:location,32:location+.002})
        h.run_batch()
        row=next(row for row in h.search.centering.queue if row['target_id']==original[1])
        self.assertEqual(row['status'],'skipped')
        self.assertNotIn('reassociation',row)
        self.assertNotIn(('camera-session',31),h.search.handled)
        self.assertNotIn(('camera-session',32),h.search.handled)
        self.assertEqual(h.search.phase,'NAVIGATING')
    def test_fire_mode_completes_each_round_before_return(self):
        h=BatchHarness(execute_fire=True);h.run_batch()
        self.assertEqual(h.search.rounds,2)
        self.assertEqual(sum(k=='auto' and v.get('action')=='start' for k,v in h.history),2)
        self.assertLessEqual(abs(h.position_mm),20)
        self.assertEqual(h.search.phase,'NAVIGATING')
    def test_repeated_capture_does_not_complete_one_second_dwell(self):
        h=BatchHarness(targets={1:.5})
        while h.search.phase!='CENTER_DWELL':h.step()
        stamp=h.search.centering.capture
        for _ in range(3):h.step(frame_change=dict(capture_stamp=stamp))
        self.assertNotIn(('camera-session',1),h.search.handled)
        self.assertFalse(any(k=='resume_retained' for k,_ in h.history))
    def test_position_or_visual_center_loss_restarts_dwell(self):
        h=BatchHarness(targets={1:.5})
        while h.search.phase!='CENTER_DWELL':h.step()
        for _ in range(3):h.step()
        h.step(nav_change=dict(pose_valid=False))
        self.assertIsNone(h.search.centering.dwell_at)
        for _ in range(3):h.step()
        self.assertNotIn(('camera-session',1),h.search.handled)
    def test_target_lost_is_recorded_without_claiming_centered_or_extinguished(self):
        h=BatchHarness(targets={1:.8})
        while h.search.phase!='CENTER_SELECT':h.step()
        h.scene={}
        for _ in range(20):
            h.step()
            if h.search.phase=='NAVIGATING':break
        self.assertEqual(h.search.phase,'NAVIGATING')
        self.assertEqual(h.search.handled,set())
        self.assertEqual(h.search.rounds,0)
        self.assertEqual(h.search.skipped_observations[-1]['reason'],'fire_centering_target_lost')
    def test_unavailable_corridor_skips_target_and_restores_anchor(self):
        h=BatchHarness(targets={1:.8})
        while h.search.phase!='CENTER_SELECT':h.step()
        h.step()
        h.step(proposal_change=dict(valid=False,reason='centering_no_verified_reverse_corridor'))
        for _ in range(10):
            h.step()
            if h.search.phase=='NAVIGATING':break
        self.assertEqual(h.search.phase,'NAVIGATING')
        self.assertEqual(h.search.handled,set())
    def test_wrong_direction_proposal_does_not_move(self):
        h=BatchHarness(targets={1:.8})
        while h.search.phase!='CENTER_SELECT':h.step()
        h.step()
        effects=h.step(proposal_change=dict(valid=True,distance_mm=-40,speed_mm_s=40))
        self.assertFalse(any(k=='center_jog' for k,_ in effects))
    def test_centred_frame_while_vehicle_moving_cannot_count_dwell(self):
        h=BatchHarness(targets={1:.5})
        while h.search.phase!='CENTER_SELECT':h.step()
        h.phase='centering_verifying'
        for _ in range(6):h.step()
        self.assertIsNone(h.search.centering.dwell_at)
        self.assertEqual(h.search.handled,set())
    def test_low_confidence_is_not_a_pause_trigger(self):
        h=RetainedSearchHarness();low=view(h.now,((1,.65),))['targets'];low[0]['confidence']=.89
        for _ in range(5):h.tick(frame_change=dict(targets=low))
        self.assertFalse(any(k=='pause' for k,_ in h.history))
    def test_stale_pose_or_wrong_passage_identity_cannot_consume_waypoints(self):
        h=RetainedSearchHarness();h.tick(index=2,nav_change=dict(pose_valid=False))
        self.assertEqual(h.search.visited,[])
        h.tick(nav_change=dict(passed_waypoint_ids=['wrong','ids']))
        self.assertEqual(h.search.visited,[])
        h.tick();self.assertEqual(len(h.search.visited),2)
    def test_download_wait_does_not_consume_motion_timeout(self):
        h=RetainedSearchHarness()
        for _ in range(185):h.tick(phase='continuous_route_check',dt=1.)
        self.assertTrue(h.search.active)
    def test_marker_stop_does_not_start_flame_batch(self):
        h=RetainedSearchHarness()
        for _ in range(12):self.assertEqual(h.tick(phase='paused'),[])
        self.assertFalse(h.search.centering.active)
    def test_early_done_cannot_skip_remaining_waypoints(self):
        h=RetainedSearchHarness();h.tick(index=2,phase='arrived')
        self.assertEqual(h.search.phase,'FAILED')

if __name__=='__main__':unittest.main()
