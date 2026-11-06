"""Observation evidence for all-sided per-flame stationary centering."""
import unittest
from test_checkpoint_alignment import view
from rescue_vision.flame_centering import FlameCentering


class CenteringEvidenceTests(unittest.TestCase):
    def ready(self, targets=((1,.5),), side='right'):
        c=FlameCentering(dict(forward_image_side=side)); c.begin(100.)
        for stamp in (100.2,100.4,100.6,100.8): c.observe(view(stamp,targets),stamp)
        self.assertTrue(c.freeze(100.8)); c.select()
        return c
    def test_all_three_sides_are_queued_and_reverse_is_distinct(self):
        c=self.ready(((1,.2),(2,.5),(3,.8)))
        self.assertEqual({row['target_id'] for row in c.queue},{1,2,3})
        self.assertEqual(c.identity[1],2)
        c.finish_target('completed','test',101.); c.select()
        self.assertEqual(c.relation(100.8),'reverse')
    def test_left_camera_installation_reverses_motion_mapping(self):
        c=self.ready(((1,.2),),side='left')
        self.assertEqual(c.relation(100.8),'forward')
    def test_low_camera_fps_waits_for_three_distinct_frames_before_freezing(self):
        c=FlameCentering();c.begin(100.)
        c.observe(view(100.2,((1,.5),)),100.2)
        c.observe(view(100.6,((1,.5),)),100.6)
        self.assertFalse(c.freeze(100.7))
        c.observe(view(101.,((1,.5),)),101.)
        self.assertTrue(c.freeze(101.))
    def test_frame_gap_resets_in_progress_dwell(self):
        c=self.ready()
        for stamp in (101.,101.2,101.4): c.observe(view(stamp,((1,.5),)),stamp)
        c.begin_dwell(101.4)
        c.observe(view(102.2,((1,.5),)),102.2)
        self.assertIsNone(c.dwell_at)
        self.assertFalse(c.dwell_complete(102.6))
    def test_elapsed_wall_time_does_not_replace_full_capture_interval(self):
        c=self.ready()
        for stamp in (101.,101.2,101.4): c.observe(view(stamp,((1,.5),)),stamp)
        c.begin_dwell(101.4)
        for stamp in (101.6,101.8,102.,102.2): c.observe(view(stamp,((1,.5),)),stamp)
        self.assertFalse(c.dwell_complete(102.5))
        c.observe(view(102.4,((1,.5),)),102.4)
        self.assertTrue(c.dwell_complete(102.5))
    def test_reused_or_invalid_boxes_do_not_enter_queue(self):
        c=FlameCentering();c.begin(100.)
        row=view(100.2,((1,.5),));row['targets'][0]['confidence']=.89
        for stamp in (100.2,100.4,100.6):
            row['capture_stamp']=stamp;c.observe(row,stamp)
        self.assertTrue(c.freeze(100.8));self.assertEqual(c.queue,[])

    def test_return_anchor_reassociation_requires_three_new_exposures(self):
        c=self.ready(((1,.2),)); c.current['reacquire_attempted']=True
        for stamp in (101.,101.2,101.4): c.observe(view(stamp,((9,.2),)),stamp,stationary=False)
        # Pre-return views cannot contribute, even if the new tracker ID is
        # already stable while the car is still reversing to its saved anchor.
        self.assertFalse(c.reassociate_at_anchor(101.4,101.4))
        for stamp in (101.6,101.8):
            c.observe(view(stamp,((9,.2),)),stamp,stationary=False)
            self.assertFalse(c.reassociate_at_anchor(stamp,101.4))
            self.assertFalse(c.reassociate_at_anchor(stamp,101.4))
        c.observe(view(102.,((9,.2),)),102.,stationary=False)
        self.assertTrue(c.reassociate_at_anchor(102.,101.4))
        self.assertEqual(c.identity,('camera-session',9))
        self.assertEqual(c.current['original_identity'],['camera-session',1])
        self.assertEqual(c.current['original_target_id'],1)
        self.assertEqual(c.current['reassociation']['capture_stamps'],[101.6,101.8,102.])
        self.assertFalse(c.current['reassociation']['physical_identity_verified'])
        self.assertFalse(c.current['reassociation']['extinguished_verified'])
        self.assertIsNone(c.dwell_at)

    def test_reassociation_cannot_steal_other_queued_or_finished_target(self):
        for finished in (False,True):
            c=self.ready(((1,.2),(2,.8))); c.current['reacquire_attempted']=True
            other=next(row for row in c.queue if row['target_id']==2)
            if finished: other['status']='completed'
            for stamp in (101.,101.2,101.4):
                c.observe(view(stamp,((2,.2),)),stamp)
                self.assertFalse(c.reassociate_at_anchor(stamp,100.9))
            self.assertEqual(c.identity,('camera-session',1))

    def test_reassociation_rejects_ambiguous_overlapping_tracks_and_registry_change(self):
        for targets,session in ((((8,.198),(9,.202)),'camera-session'),(((9,.2),),'new-camera')):
            c=self.ready(((1,.2),)); c.current['reacquire_attempted']=True
            for stamp in (101.,101.2,101.4,101.6):
                c.observe(view(stamp,targets,session=session),stamp)
                self.assertFalse(c.reassociate_at_anchor(stamp,100.9))
            self.assertEqual(c.identity,('camera-session',1))

    def test_reassociation_same_new_id_cannot_match_two_original_queue_boxes(self):
        c=self.ready(((1,.2),(2,.205)));c.current['reacquire_attempted']=True
        original=c.identity
        for stamp in (101.,101.2,101.4):
            c.observe(view(stamp,((9,.202),)),stamp)
            self.assertFalse(c.reassociate_at_anchor(stamp,100.9))
        self.assertEqual(c.identity,original)

    def test_reassociation_reset_on_gap_or_changed_stationary_interval(self):
        for change in ('gap','since'):
            c=self.ready(((1,.2),)); c.current['reacquire_attempted']=True
            for stamp in (101.,101.2):
                c.observe(view(stamp,((9,.2),)),stamp,stationary=False)
                self.assertFalse(c.reassociate_at_anchor(stamp,100.9))
            stamp,since=(102.,100.9) if change=='gap' else (101.4,101.3)
            c.observe(view(stamp,((9,.2),)),stamp,stationary=False)
            self.assertFalse(c.reassociate_at_anchor(stamp,since))
            self.assertEqual(c.identity,('camera-session',1))

    def test_reassociation_rejects_wrong_position_shape_or_frame(self):
        for change in ('position','size','frame'):
            c=self.ready(((1,.2),));c.current['reacquire_attempted']=True
            for stamp in (101.,101.2,101.4):
                document=view(stamp,((9,.7 if change=='position' else .2),))
                if change=='size': document['targets'][0]['bbox_xyxy']=[90,100,170,200]
                if change=='frame': document['coordinate_frame']='different-optical-frame'
                c.observe(document,stamp)
                self.assertFalse(c.reassociate_at_anchor(stamp,100.9))

    def test_republished_same_exposure_metadata_never_counts_three_times(self):
        c=self.ready(((1,.2),));c.current['reacquire_attempted']=True
        for stamp in (101.,101.2,101.4,101.6):
            document=view(stamp,((9,.2),));document.update(capture_stamp_ns=101000000000,sequence=20)
            c.observe(document,stamp)
            self.assertFalse(c.reassociate_at_anchor(stamp,100.9))
        self.assertEqual(c.identity,('camera-session',1))

    def test_observed_original_track_cannot_be_rebound_and_attempt_is_explicit(self):
        for still_visible in (True,False):
            c=self.ready(((1,.2),))
            if still_visible: c.current['reacquire_attempted']=True
            for stamp in (101.,101.2,101.4):
                c.observe(view(stamp,((1,.2),(9,.2)) if still_visible else ((9,.2),)),stamp)
                self.assertFalse(c.reassociate_at_anchor(stamp,100.9))

    def test_trigger_survives_braking_but_needs_three_post_stop_confident_frames(self):
        c=FlameCentering();c.begin(101.,seed=view(100.,((375,.65),)))
        self.assertEqual(c.queue[0]['status'],'candidate')
        self.assertTrue(c.queue[0]['seeded_from_trigger'])
        self.assertEqual(c.queue[0]['post_stop_stamps'],[])
        for stamp,identifier in ((101.2,383),(101.4,384)):
            c.observe(view(stamp,((identifier,.65),)),stamp)
        self.assertFalse(c.freeze(101.6))
        c.observe(view(101.6,((386,.65),)),101.6)
        self.assertTrue(c.freeze(101.61));self.assertEqual(len(c.queue),1)
        self.assertEqual(c.queue[0]['status'],'queued')
        self.assertEqual(c.queue[0]['target_id'],386)
        self.assertEqual(c.queue[0]['original_target_id'],375)

    def test_short_empty_frames_and_changing_tracker_ids_keep_one_batch_identity(self):
        c=FlameCentering();c.begin(100.,seed=view(100.,((1,.65),)))
        batch=c.queue[0]['batch_id']
        for stamp,targets in ((100.1,((2,.65),)),(100.2,()),(100.3,((3,.65),)),
                              (100.4,()),(100.5,((4,.65),)),(100.7,((4,.65),))):
            c.observe(view(stamp,targets),stamp)
        self.assertTrue(c.freeze(100.7));self.assertEqual(len(c.queue),1)
        self.assertEqual(c.queue[0]['batch_id'],batch)
        self.assertEqual(c.queue[0]['status'],'queued')
        self.assertEqual(c.queue[0]['target_id'],4)
        self.assertEqual({tuple(x) for x in c.queue[0]['observed_id_aliases']},
                         {('camera-session',i) for i in range(1,5)})

    def test_pure_empty_scene_closes_seed_as_skipped_without_false_completion(self):
        c=FlameCentering();c.begin(100.,seed=view(100.,((1,.65),)))
        for stamp in (100.2,100.4,100.6,100.8):c.observe(view(stamp),stamp)
        self.assertTrue(c.freeze(100.81))
        self.assertEqual(c.queue[0]['status'],'skipped')
        self.assertEqual(c.queue[0]['reason'],'no_flame_visible_after_verified_stop')
        self.assertEqual(c.snapshot(100.81)['completed_count'],0)
        self.assertIsNone(c.select())

    def test_stationary_id_change_rebinds_after_three_frames_without_return_motion(self):
        c=self.ready(((1,.65),));batch=c.current['batch_id']
        for stamp in (101.,101.2):
            c.observe(view(stamp,((9,.65),)),stamp,stationary=True,at_anchor=True)
            self.assertEqual(c.identity,('camera-session',1))
        c.observe(view(101.4,((9,.65),)),101.4,stationary=True,at_anchor=True)
        self.assertEqual(c.identity,('camera-session',9));self.assertEqual(c.current['batch_id'],batch)
        self.assertEqual(c.current['reassociation']['scope'],'unique_stationary_local_view')

    def test_moving_tracker_change_cannot_silently_switch_target(self):
        c=self.ready(((1,.65),))
        for stamp in (101.,101.2,101.4):
            c.observe(view(stamp,((9,.65),)),stamp,stationary=False,at_anchor=False)
        self.assertEqual(c.identity,('camera-session',1))
        self.assertEqual(c.relation(101.4),'missing')

    def test_late_candidates_have_capacity_and_intake_deadline(self):
        c=self.ready(((1,.5),));c.max_targets=2
        for stamp in (101.,101.2,101.4):
            c.observe(view(stamp,((1,.5),(2,.75),(3,.2))),stamp)
        self.assertEqual(len(c.queue),2);self.assertTrue(c.queue_truncated)
        self.assertEqual(c.queue[1]['status'],'queued')
        for stamp in (107.,107.2,107.4):c.observe(view(stamp,((4,.1),)),stamp)
        self.assertEqual(len(c.queue),2)
        self.assertNotIn(4,[row['target_id'] for row in c.queue])
        # Also prove the deadline when capacity is still available.
        c=self.ready(((1,.5),))
        for stamp in (107.,107.2,107.4):c.observe(view(stamp,((4,.1),)),stamp)
        self.assertEqual(len(c.queue),1)

    def test_visible_but_unstable_collection_waits_then_explicitly_times_out(self):
        c=FlameCentering();c.begin(100.)
        for i in range(1,12):
            stamp=100.+i*.2
            c.observe(view(stamp,((i,.05+i*.065),)),stamp)
            self.assertFalse(c.freeze(stamp))
        c.observe(view(102.41,((12,.9),)),102.41)
        self.assertTrue(c.freeze(102.41))
        self.assertEqual(c.collect_reason,'flame_batch_confirmation_timeout')
        self.assertTrue(all(row['status']=='skipped' for row in c.queue))
        self.assertEqual(c.snapshot(102.41)['completed_count'],0)

    def test_one_empty_exposure_breaks_continuous_center_dwell(self):
        c=self.ready()
        for stamp in (101.,101.2,101.4):c.observe(view(stamp,((1,.5),)),stamp)
        c.begin_dwell(101.4)
        c.observe(view(101.6),101.6)
        self.assertIsNone(c.dwell_at)
        for stamp in (101.8,102.,102.2,102.4):c.observe(view(stamp,((1,.5),)),stamp)
        self.assertFalse(c.dwell_complete(102.5))


if __name__=='__main__':unittest.main()
