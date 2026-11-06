import pathlib
import sys
import unittest
import math
import numpy as np
import cv2
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.aim_geometry import RectifiedCameraCalibration,VariableRateServo
from rescue_vision.legacy_visual_servo import ServoConfig


class GeometryTests(unittest.TestCase):
    def test_rectified_rays_rotated_back_to_physical_camera(self):
        rotation=cv2.Rodrigues(np.array([.1,-.2,.07]))[0]
        k=np.array([[400.,0,320],[0,400,240],[0,0,1]])
        calib=RectifiedCameraCalibration(dict(camera_calibrated=True,calibration_id='synthetic',eye='left',
            image_size_wh=[640,480],camera_matrix=k.tolist(),distortion_coefficients=[0]*5),rotation)
        rays=np.array([[.2,.1,1],[0,0,1]]).T
        rect=k@rotation@rays;pixels=(rect[:2]/rect[2]).T
        result=calib.angle_error(pixels[0],pixels[1],(640,480))
        self.assertAlmostEqual(result['yaw_right_positive'],math.degrees(math.atan(.2)),places=6)
        self.assertAlmostEqual(result['pitch_up_positive'],math.degrees(math.atan2(-.1,math.hypot(.2,1))),places=6)
    def test_slow_fresh_updates_have_bounded_steps_but_long_gap_holds(self):
        p=VariableRateServo(ServoConfig(enabled=True,mounting_verified=True,pan_deg_per_px=.1,tilt_deg_per_px=.1))
        aim=dict(state='ALIGNING',image_locked=False,pixel_error=dict(x_right=40,y_down=0))
        p.update(aim,0);out=p.update(aim,.4)
        self.assertIsNotNone(out['command']);self.assertLessEqual(out['command']['pan_target_deg']-90,1.5)
        self.assertIsNone(p.update(aim,2.)['command'])

if __name__=='__main__':unittest.main(verbosity=2)
