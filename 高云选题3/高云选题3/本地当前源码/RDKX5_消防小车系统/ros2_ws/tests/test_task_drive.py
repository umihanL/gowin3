import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'/'rescue_vision'))
from rescue_vision.task_drive import CrosswalkLatch,road_pose


class CrosswalkLatchTests(unittest.TestCase):
    def observe(self,latch,t,detected=True,valid=True):
        return latch.observe(t,detected,valid,t+.01)

    def test_three_distinct_spaced_frames_trigger_once(self):
        latch=CrosswalkLatch()
        self.assertFalse(self.observe(latch,1.))
        self.assertFalse(self.observe(latch,1.1))
        self.assertTrue(self.observe(latch,1.2))
        self.assertFalse(self.observe(latch,1.3))
        self.assertFalse(self.observe(latch,1.4))

    def test_duplicate_and_old_capture_cannot_confirm(self):
        latch=CrosswalkLatch()
        for _ in range(10):self.assertFalse(self.observe(latch,1.))
        self.assertFalse(self.observe(latch,.9))
        self.assertFalse(self.observe(latch,1.1))
        self.assertTrue(self.observe(latch,1.2))

    def test_false_invalid_and_large_gap_reset_evidence(self):
        for detected,valid,gap in ((False,True,.1),(True,False,.1),(True,True,.4)):
            latch=CrosswalkLatch()
            self.observe(latch,1.);self.observe(latch,1.1)
            self.assertFalse(self.observe(latch,1.1+gap,detected,valid))
            self.assertFalse(self.observe(latch,1.2+gap))

    def test_completion_cooldown_and_clear_frames_both_required(self):
        latch=CrosswalkLatch()
        for t in (1.,1.1,1.2):self.observe(latch,t)
        # It never rearms merely because a crossing leaves the image.
        for i in range(30):self.observe(latch,1.3+i*.1,False)
        self.assertFalse(latch.armed)
        latch.complete(5.)
        for i in range(10):self.observe(latch,5.1+i*.1,False)
        self.assertFalse(latch.armed)
        # Positive stripes break the clear-frame run even after cooldown.
        self.observe(latch,7.1,True)
        for i in range(4):self.observe(latch,7.2+i*.1,False)
        self.assertFalse(latch.armed)
        self.assertFalse(self.observe(latch,7.6,False))
        self.assertTrue(latch.armed)
        self.assertFalse(self.observe(latch,7.7))
        self.assertFalse(self.observe(latch,7.8))
        self.assertTrue(self.observe(latch,7.9))

    def test_stale_future_and_nan_are_not_evidence(self):
        for capture,now in ((1.,2.),(3.,2.),(float('nan'),2.)):
            latch=CrosswalkLatch()
            self.assertFalse(latch.observe(capture,True,True,now))
            self.assertIsNone(latch.last_capture)


class RoadPoseTests(unittest.TestCase):
    def test_requires_both_mount_and_road_calibration(self):
        mount=dict(mounting_verified=True,road_pose_verified=True,road_pan_deg=90.,road_tilt_deg=120.,
                   pan_limits_deg=[30.,150.],tilt_limits_deg=[40.,140.])
        self.assertEqual(road_pose(mount),(90.,120.))
        for change in ({'mounting_verified':False},{'road_pose_verified':False},{'road_tilt_deg':None},
                       {'road_tilt_deg':180.},{'road_tilt_deg':float('nan')},{'road_pan_deg':True}):
            self.assertIsNone(road_pose({**mount,**change}))


if __name__=='__main__':unittest.main()
