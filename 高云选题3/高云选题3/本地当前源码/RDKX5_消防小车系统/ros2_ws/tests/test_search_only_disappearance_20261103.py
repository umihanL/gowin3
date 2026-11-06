"""No hardware: visual dropout cannot discard a search-only retained route."""
import copy
import unittest

from test_region_retained_search import BatchHarness, RetainedSearchHarness
from test_region_continuous_flow import ContinuousHarness
from rescue_vision import ov5640_protocol as wire
from rescue_vision.area_navigation import RESET_NAV
from rescue_vision.retained_route import local_coordinates


class SearchOnlyVisualRecoveryTests(unittest.TestCase):
    def stopped_batch(self, stage='WAIT_STOP'):
        h=BatchHarness(targets={375:.5})
        for _ in range(40):
            h.step()
            if h.search.phase==stage:return h
        self.fail(h.search.snapshot(h.now))

    def assert_resumed_without_completion(self,h):
        self.assertTrue(h.search.active,h.search.reason)
        self.assertEqual(h.phase,'running')
        self.assertEqual(sum(k=='navigate' for k,_ in h.history),1)
        self.assertEqual(sum(k=='resume_retained' for k,_ in h.history),1)
        self.assertEqual(h.search.centering.snapshot(h.now)['completed_count'],0)
        self.assertFalse(any(k=='auto' and v.get('action')=='start' for k,v in h.history))

    def test_fresh_empty_and_stale_stream_both_resume_original_route(self):
        for stage in ('WAIT_STOP','CENTER_DWELL'):
            for stale in (False,True):
                with self.subTest(stage=stage,stale=stale):
                    h=self.stopped_batch(stage);start=h.now;h.scene={}
                    for _ in range(30):
                        h.step(frame_change={'capture_stamp':start} if stale else None)
                        if any(k=='resume_retained' for k,_ in h.history):break
                    self.assert_resumed_without_completion(h)
                    self.assertLess(h.now-start,4.)
                    self.assertEqual(h.search.snapshot(h.now)['visual_coverage_incomplete'],stale)

    def test_capture_counter_restart_cannot_trap_collection(self):
        h=self.stopped_batch();h.step()
        for _ in range(25):
            h.step(frame_change={'sequence':0})
            if any(k=='resume_retained' for k,_ in h.history):break
        self.assert_resumed_without_completion(h)
        self.assertEqual(h.search.centering.abandoned_reason,'search_only_visual_unavailable_skip_observation')
        self.assertTrue(h.search.snapshot(h.now)['visual_coverage_incomplete'])

    def test_no_frame_from_start_keeps_original_navigation_and_reports_incomplete_coverage(self):
        h=RetainedSearchHarness()
        for index in range(1,5):
            h.tick(index=index,dt=1.,frame_change={'frame_valid':False})
        self.assertEqual(h.search.phase,'EXITING')
        self.assertTrue(h.search.visual_degraded)
        self.assertFalse(h.search.snapshot(h.now)['fire_stop_enabled'])
        self.assertFalse(any(k in ('pause','stop','resume_retained') for k,_ in h.history))
        h.tick(phase='arrived',frame_change={'frame_valid':False})
        self.assertEqual(h.search.phase,'COMPLETE')
        self.assertTrue(h.search.snapshot(h.now)['visual_coverage_incomplete'])

    def test_stream_recovery_enables_only_new_confirmed_observations(self):
        h=RetainedSearchHarness()
        h.tick(dt=2.2,frame_change={'frame_valid':False})
        h.tick(targets=((5,.5),))
        self.assertFalse(h.search.visual_degraded)
        self.assertIsNotNone(h.search.visual_recovered_at)
        self.assertNotEqual(h.search.phase,'WAIT_STOP')
        h.tick(targets=((5,.5),));h.tick(targets=((5,.5),))
        self.assertEqual(h.search.phase,'WAIT_STOP')

    def test_moving_batch_does_not_resume_until_measured_stop_and_anchor_return(self):
        h=RetainedSearchHarness();h.detect(center=.7)
        h.tick(phase='paused',nav_change={'centering':dict(active=True,offset_mm=0.,return_ready=True)})
        c=h.search.centering
        for _ in range(5):h.tick(phase='paused',targets=((1,.7),))
        self.assertEqual(h.search.phase,'CENTER_SELECT')
        if c.current is None:h.tick(phase='paused',targets=((1,.7),))
        h.tick(phase='paused',targets=((1,.7),),centering_proposal=dict(stamp=h.now+.2,valid=True,distance_mm=40,speed_mm_s=40))
        self.assertEqual(h.search.phase,'CENTER_MOVING')
        stale=h.now
        for _ in range(12):h.tick(phase='centering_moving',frame_change={'capture_stamp':stale})
        self.assertEqual(h.search.phase,'CENTER_STOP')
        self.assertEqual(sum(k=='center_stop' for k,_ in h.history),1)
        self.assertFalse(any(k=='resume_retained' for k,_ in h.history))
        for _ in range(3):h.tick(phase='centering_stopping',frame_change={'capture_stamp':stale})
        self.assertFalse(any(k=='resume_retained' for k,_ in h.history))
        h.tick(phase='paused',frame_change={'capture_stamp':stale})
        self.assertEqual(h.search.phase,'CENTER_RETURN')
        h.tick(phase='paused',frame_change={'capture_stamp':stale},
            centering_proposal=dict(stamp=h.now+.2,valid=True,at_anchor=False,distance_mm=-40,speed_mm_s=40))
        h.tick(phase='centering_moving',frame_change={'capture_stamp':stale})
        self.assertEqual(sum(k=='center_stop' for k,_ in h.history),1,'return jog must not be cancelled')
        self.assertFalse(any(k=='resume_retained' for k,_ in h.history))
        h.tick(phase='paused',frame_change={'capture_stamp':stale},
            centering_proposal=dict(stamp=h.now+.2,valid=True,at_anchor=True))
        self.assertEqual(sum(k=='resume_retained' for k,_ in h.history),1)

    def test_visual_recovery_never_overrides_navigation_failure_or_operator_cancel(self):
        h=self.stopped_batch();old=h.now
        h.tick(dt=2.2,phase='failed',frame_change={'capture_stamp':old})
        self.assertEqual(h.search.phase,'FAILED')
        self.assertEqual(h.search.reason,'region_navigation_failed')
        h=self.stopped_batch();h.search.cancel(h.now,'operator_stop')
        for _ in range(20):h.step(frame_change={'frame_valid':False})
        self.assertFalse(any(k=='resume_retained' for k,_ in h.history))
        self.assertEqual(h.search.phase,'CANCELLED')

    def test_extinguishing_task_keeps_existing_camera_loss_failure(self):
        h=RetainedSearchHarness(execute_fire=True)
        h.tick(frame_change={'frame_valid':False},dt=2.2)
        self.assertEqual(h.search.phase,'FAILED')
        self.assertEqual(h.search.reason,'region_yolo_feedback_missing')


class RealControllerVisualRecoveryTests(unittest.TestCase):
    def test_camera_loss_after_measured_jog_returns_anchor_before_exact_route_resume(self):
        h=ContinuousHarness();h.start();h.drive_mission();points=h.route['points_mm']
        found=False
        for index in range(3,len(points)-8):
            anchor=dict(image_mm=points[index],heading_deg=h.path_pose(index)[1])
            behind=local_coordinates(anchor,points[index-3])
            ahead=local_coordinates(anchor,points[index+6])
            if behind[0]<-100 and ahead[0]>100 and abs(behind[1])<10 and abs(ahead[1])<10:
                while h.path_cursor<index-2:h.advance()
                found=True;break
        self.assertTrue(found)
        h.targets=[(1,.7)]
        while h.search.phase!='WAIT_STOP':h.advance()
        h.verify_pause()
        saved=(h.nav.active['task_id'],h.nav.active['route_generation'],h.execution_index,
               h.nav.active['progress_mm'],h.nav.active['checkpoint_index'])
        for _ in range(40):
            if h.nav.active['phase']=='centering_starting':break
            h.feedback(tick=True)
        self.assertEqual(h.nav.active['phase'],'centering_starting')
        h.targets=[];h.complete_jog()
        frozen=copy.deepcopy(h.c.area.targets);original=h.c.observe_targets
        h.c.observe_targets=lambda document,now:original(copy.deepcopy(frozen),now)
        for _ in range(50):
            if h.nav.active['phase']=='resuming':break
            if h.nav.active['phase']=='centering_starting':h.complete_jog()
            else:h.feedback(tick=True)
        self.assertEqual(h.nav.active['phase'],'resuming',h.search.snapshot(h.now))
        self.assertEqual(saved,(h.nav.active['task_id'],h.nav.active['route_generation'],h.execution_index,
            h.nav.active['progress_mm'],h.nav.active['checkpoint_index']))
        self.assertEqual(h.nav.centering.session['jog_count'],2)
        self.assertAlmostEqual(local_coordinates(h.nav.centering.session['anchor'],h.image_position)[0],0,delta=1.)
        self.assertTrue(h.nav.resume_write_ready(h.nav.active['sequence'],h.now+.003))
        self.assertEqual(h.search.centering.snapshot(h.now)['completed_count'],0)
        self.assertTrue(h.search.visual_coverage_incomplete)

    def test_camera_stops_after_pause_restores_same_task_and_finishes_original_rom_route(self):
        h=ContinuousHarness();h.start();h.drive_mission();h.detect();h.verify_pause()
        saved=(h.nav.active['task_id'],h.nav.active['route_generation'],h.execution_index,
               h.nav.active['progress_mm'],h.nav.active['checkpoint_index'])
        frozen=copy.deepcopy(h.c.area.targets)
        for _ in range(30):
            h.feedback(tick=False)
            h.c.area.targets=copy.deepcopy(frozen)
            h.c.area.tick(h.now+.002)
            if h.nav.active['phase']=='resuming':break
            self.assertTrue(h.search.active,h.search.reason)
        self.assertEqual(h.nav.active['phase'],'resuming',h.search.snapshot(h.now))
        self.assertEqual(saved,(h.nav.active['task_id'],h.nav.active['route_generation'],h.execution_index,
            h.nav.active['progress_mm'],h.nav.active['checkpoint_index']))
        self.assertTrue(h.nav.resume_write_ready(h.nav.active['sequence'],h.now+.003))
        self.assertEqual(h.search.centering.snapshot(h.now)['completed_count'],0)
        self.assertTrue(h.search.visual_degraded)
        h.targets=[];h.resume();h.finish_exit()
        self.assertEqual(h.search.phase,'COMPLETE')
        self.assertEqual(len(h.search.visited),4)
        commands=[row[0] for row in h.sent_commands]
        for command in (wire.PLAN,RESET_NAV,wire.START,wire.RESUME):self.assertEqual(commands.count(command),1)
        self.assertNotIn(wire.RETAINED_JOG,commands)
        self.assertTrue(h.search.snapshot(h.now)['visual_coverage_incomplete'])


if __name__=='__main__':unittest.main()
