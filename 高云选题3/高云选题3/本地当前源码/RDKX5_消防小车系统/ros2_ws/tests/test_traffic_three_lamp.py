"""Real cropped fixture regressions plus isolated negative lighting cases.

Derived images are correlated checks, not a field-accuracy evaluation.
"""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision.traffic import TrafficDetector, TrafficController


def fixture(state='off', panel=True):
    image=np.full((400,300,3),160,np.uint8)
    if panel:cv2.rectangle(image,(100,35),(200,345),(16,16,16),-1)
    for color,center,bgr in [('red',(150,85),(0,0,100)),('yellow',(150,185),(0,100,100)),('green',(150,285),(0,100,0))]:
        cv2.circle(image,center,32,bgr,-1)
        if state==color or state=='both' and color in ('red','green'):
            cv2.circle(image,center,21,tuple(255 if c else 0 for c in bgr),-1)
    return image


class ThreeLampTests(unittest.TestCase):
    def setUp(self):
        self.detector=TrafficDetector(json.loads((ROOT/'config/traffic.fpga_navigation.json').read_text()))
    def image(self,name):
        return cv2.imdecode(np.fromfile(ROOT/'tests/fixtures/traffic_three_lamp'/name,dtype=np.uint8),cv2.IMREAD_COLOR)
    def test_user_real_red_white_core_and_green_are_distinguished(self):
        for name,expected in [('red_on.png','red'),('green_on.png','green')]:
            image=self.image(name)
            result=self.detector.detect(image,capture_time=42.,frame_id='physical-left:42')
            self.assertEqual(result['status'],expected)
            self.assertEqual(result['capture_time'],42.)
            self.assertEqual(result['frame_id'],'physical-left:42')
            self.assertEqual(result['image_size'],[image.shape[1],image.shape[0]])
            self.assertEqual(result['fixture_bbox'],result['candidates'][0]['fixture_bbox'])
            x1,y1,x2,y2=result['fixture_bbox']
            self.assertTrue(0<=x1<x2<=image.shape[1] and 0<=y1<y2<=image.shape[0])
            for lamp in result['candidates'][0]['lamps']:
                self.assertTrue(x1<=lamp['center'][0]<=x2 and y1<=lamp['center'][1]<=y2)
    def test_actual_frame_embedding_and_rotations_keep_correct_label(self):
        for name,expected in [('red_on.png','red'),('green_on.png','green')]:
            for scale,angle in [(.6,0),(1.,0),(1.,-15),(1.,15)]:
                image=self.image(name);image=cv2.resize(image,None,fx=scale,fy=scale)
                canvas=np.full((544,640,3),160,np.uint8);h,w=image.shape[:2]
                canvas[272-h//2:272-h//2+h,320-w//2:320-w//2+w]=image
                rotated=cv2.warpAffine(canvas,cv2.getRotationMatrix2D((320,272),angle,1),(640,544),borderValue=(160,160,160))
                with self.subTest(name=name,scale=scale,angle=angle):self.assertEqual(self.detector.detect(rotated)['status'],expected)
    def test_unlit_and_yellow_and_both_never_grant_green(self):
        for mode,expected in [('off','unknown'),('yellow','unknown'),('red','red'),('green','green'),('both','conflict')]:
            with self.subTest(mode=mode):self.assertEqual(self.detector.detect(fixture(mode))['status'],expected)
    def test_no_panel_partial_board_or_reversed_colors_are_not_green(self):
        images=[fixture('green',False),np.full((400,300,3),(0,255,0),np.uint8)]
        image=fixture('green');image[:125]=160;images.append(image)
        image=fixture('green');image[53:118]=image[253:318];image[253:318]=(0,0,150);images.append(image)
        for image in images:self.assertNotEqual(self.detector.detect(image)['status'],'green')
    def test_green_glass_with_small_specular_highlight_is_not_active(self):
        image=fixture('off');cv2.circle(image,(150,285),5,(255,255,255),-1)
        self.assertNotEqual(self.detector.detect(image)['status'],'green')
    def test_two_distinct_green_fixtures_are_ambiguous(self):
        image=np.full((500,800,3),160,np.uint8)
        image[50:450,60:360]=fixture('green');image[50:450,440:740]=fixture('green')
        result=TrafficDetector({'fixture_layout':'three_lamp','roi_normalized':[0,0,1,1]}).detect(image)
        self.assertEqual(result['status'],'unknown');self.assertEqual(result['reason'],'three_lamp_multiple_fixtures')
    def test_configured_three_lamp_mode_never_uses_two_lamp_fallback(self):
        with patch('rescue_vision.traffic.detect_three_lamp_fixture',return_value=None):
            self.assertEqual(self.detector.detect(fixture('green'))['status'],'unknown')
    def test_uncertain_light_after_stable_green_revokes_permission(self):
        for status in ('unknown','conflict','yellow','red'):
            c=TrafficController();c.on_crosswalk('test',0);c.tick(0,vehicle_stopped=True,gimbal_ready=True)
            for n in range(7):
                t=.31+n*.1;out=c.tick(t,dict(status='green',confidence=.9,fixture_bbox=[1,1,50,120],capture_time=t,frame_id=n),True,True)
            self.assertTrue(out['move_allowed'])
            self.assertFalse(c.tick(1.02,dict(status=status,confidence=.9,fixture_bbox=[1,1,50,120],capture_time=1.02,frame_id=8),False,True)['move_allowed'])

if __name__=='__main__':unittest.main()
