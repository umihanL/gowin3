"""Synthetic calibrated stereo fixture verifies metric units and rejection."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime.stereo_depth import CalibratedStereo


class StereoTests(unittest.TestCase):
    def calibration(self):
        return dict(calibrated=True,translation_unit='m',image_size=[640,480],
                    K_left=[[440,0,320],[0,440,240],[0,0,1]],D_left=[0]*5,
                    K_right=[[440,0,320],[0,440,240],[0,0,1]],D_right=[0]*5,
                    R=np.eye(3).tolist(),T=[-.08,0,0])

    def test_known_disparity_produces_metric_range_and_rejects_no_depth(self):
        with tempfile.TemporaryDirectory() as tmp:
            file=Path(tmp)/'synthetic_fixture_only.json'
            file.write_text(json.dumps(self.calibration()),encoding='utf-8')
            stereo=CalibratedStereo(file)
            rng=np.random.default_rng(7)
            left=rng.integers(0,255,(480,640,3),dtype=np.uint8)
            left=cv2.GaussianBlur(left,(3,3),0)
            right=np.zeros_like(left);right[:,:-40]=left[:,40:]
            _,_,depth=stereo.process(left,right)
            measured=stereo.detection_range(depth,(280,190,360,290))
            self.assertAlmostEqual(measured,.88,delta=.025) # 440*.08/40
            self.assertIsNone(stereo.detection_range(np.full_like(depth,np.nan),(280,190,360,290)))

    def test_assumed_camera_parameters_cannot_supply_range(self):
        with tempfile.TemporaryDirectory() as tmp:
            file=Path(tmp)/'not_calibrated.json';value=self.calibration();value['calibrated']=False
            file.write_text(json.dumps(value),encoding='utf-8')
            with self.assertRaises(ValueError):CalibratedStereo(file)


if __name__=='__main__':unittest.main()
