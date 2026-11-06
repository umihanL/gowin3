import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime.observation import Policy, Detection, Observer
from runtime.run_observation import attach_ranges, telemetry_at, decide_observation


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.observer=Observer()
        self.seq=0

    def target(self,**kw):
        values=dict(class_id=1,confidence=.8,bbox=(306,210,334,270),
                    distance_m=.7,distance_source='test_fixture')
        values.update(kw)
        return Detection(**values)

    def frame(self,ds=None,**kw):
        values=dict(timestamp=self.seq*.1,frame_id=self.seq,width=640,height=480,
                    speed_mps=0.,safety_clear=True,at_observation_point=True)
        values.update(kw);self.seq+=1
        return self.observer.update([self.target()] if ds is None else ds,**values)

    def test_same_target_three_of_five_with_missed_frame(self):
        self.assertFalse(self.frame()['ready'])
        self.assertFalse(self.frame()['ready'])
        missed=self.frame([])
        self.assertFalse(missed['ready'])
        self.assertTrue(self.frame()['ready'])
        self.assertFalse(self.frame([])['ready'])

    def test_duplicate_frame_cannot_count_or_reauthorize(self):
        a=self.frame()
        for _ in range(10):
            r=self.frame(timestamp=a['timestamp'],frame_id=a['frame_id'])
            self.assertFalse(r['ready'])
        self.assertEqual(len(self.observer.history),1)

    def test_far_tiny_or_unmeasured_never_ready(self):
        for d,reason in [(self.target(distance_m=1.3),'too_far'),
                         (self.target(distance_m=.2),'too_near'),
                         (self.target(distance_m=None),'await_associated_range'),
                         (self.target(distance_source=None),'await_associated_range'),
                         (self.target(bbox=(317,222,323,258)),'insufficient_pixels')]:
            self.observer.reset()
            for _ in range(7):r=self.frame([d])
            self.assertFalse(r['ready']);self.assertEqual(r['reason'],reason)

    def test_duplicate_frame_state_changes_revoke_old_confirmation(self):
        for changed in [dict(safety_clear=False),dict(speed_mps=.1),
                        dict(at_observation_point=False),dict(pan_deg=20)]:
            self.observer.reset();self.seq=0
            self.frame();self.frame();ready=self.frame()
            self.assertTrue(ready['ready'])
            invalid=self.frame(timestamp=ready['timestamp'],frame_id=ready['frame_id'],**changed)
            self.assertFalse(invalid['ready'])
            self.assertEqual(len(self.observer.history),0)
            next_frame=self.frame()
            self.assertEqual(next_frame['votes'],1)
            self.assertFalse(next_frame['ready'])

    def test_inference_delay_rechecks_captured_frame_and_telemetry(self):
        for telemetry_stamp,delay,reason in [(10.,.06,'await_fresh_rig_telemetry'),
                                            (10.2,.26,'stale_source_frame')]:
            self.observer.reset();self.seq=0
            self.frame();self.frame();self.assertTrue(self.frame()['ready'])
            telemetry=dict(timestamp_unix_s=telemetry_stamp,frame_id=3,
                pan_deg=0,tilt_deg=0,speed_mps=0,safety_clear=True,at_observation_point=True)
            out=decide_observation(self.observer,[self.target()],telemetry,
                timestamp=.3,frame_id=3,width=640,height=480,
                decision_stamp=.3+delay,decision_wall=10.2+delay)
            self.assertFalse(out['ready']);self.assertEqual(out['reason'],reason)
            self.assertEqual(len(self.observer.history),0)
        self.observer.reset()
        for i in range(3):
            telemetry=dict(timestamp_unix_s=10+i*.1,frame_id=i,
                pan_deg=0,tilt_deg=0,speed_mps=0,safety_clear=True,at_observation_point=True)
            out=decide_observation(self.observer,[self.target()],telemetry,
                timestamp=i*.1,frame_id=i,width=640,height=480,
                decision_stamp=i*.1+.05,decision_wall=10+i*.1+.05)
        self.assertTrue(out['ready'])

    def test_unknown_speed_or_unsafe_stop_clears_votes(self):
        self.frame();self.frame()
        self.assertFalse(self.frame(speed_mps=None)['ready'])
        self.assertEqual(self.frame()['votes'],1)
        self.assertFalse(self.frame(safety_clear=False)['ready'])
        self.assertEqual(len(self.observer.history),0)

    def test_motion_or_time_gap_restarts_confirmation(self):
        self.frame();self.frame()
        self.assertEqual(self.frame(pan_deg=3)['votes'],1)
        self.frame(pan_deg=3)
        self.assertEqual(self.frame(timestamp=3,pan_deg=3)['votes'],1)

    def test_multiple_objects_and_class_switch_do_not_combine(self):
        self.frame();self.frame()
        changed=self.frame([self.target(class_id=2)])
        self.assertEqual(changed['votes'],1)
        self.frame([self.target(class_id=2)])
        other=self.frame([self.target(class_id=2,bbox=(340,210,368,270))])
        self.assertEqual(other['votes'],0)  # New object is also off-centre.
        self.assertFalse(other['ready'])

    def test_cumulative_gimbal_drift_resets(self):
        self.frame(pan_deg=0)
        self.frame(pan_deg=1.5)
        r=self.frame(pan_deg=3)
        self.assertEqual(r['votes'],1)

    def test_crop_upscaling_cannot_invent_detail(self):
        d=self.target(bbox=(317,222,323,258),detail_scale=2)
        self.assertEqual(self.frame([d])['state'],'scan')

    def test_alignment_requests_no_direct_vehicle_motion(self):
        r=self.frame([self.target(bbox=(450,200,480,265))])
        self.assertEqual(r['state'],'align')
        self.assertLess(r['pan_deg'],0)
        self.assertFalse(r['vehicle_motion_authorized'])
        self.assertFalse(r['ready'])

    def test_measurement_not_shared_across_people(self):
        ds=[self.target(distance_m=None),self.target(bbox=(380,210,408,270),distance_m=None)]
        attach_ranges(ds,[dict(bbox_xyxy=[306,210,334,270],distance_m=.6,source='range_sensor')])
        self.assertEqual(ds[0].distance_m,.6)
        self.assertIsNone(ds[1].distance_m)

    def test_stale_or_wrong_frame_telemetry_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'telemetry.json'
            path.write_text(json.dumps(dict(timestamp_unix_s=10,frame_id=3)),encoding='utf-8')
            self.assertTrue(telemetry_at(path,10.1,3))
            self.assertFalse(telemetry_at(path,10.5,3))
            self.assertFalse(telemetry_at(path,10.1,4))

    def test_configuration_rejects_bad_values(self):
        for kw in [dict(min_hits=6),dict(range_min_m=1.2),dict(confidence=float('nan')),
                   dict(class_priority=[1,1,2]),dict(min_height_px=0)]:
            with self.assertRaises(ValueError):Policy(**kw)


if __name__=='__main__':unittest.main()
