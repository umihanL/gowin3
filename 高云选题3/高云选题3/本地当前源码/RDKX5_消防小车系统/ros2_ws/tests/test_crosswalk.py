import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'/'rescue_vision'))
from rescue_vision.crosswalk import CrosswalkDetector, observation_mode
from rescue_vision.traffic import TrafficDetector


def zebra(count=7, direction='vertical', light=235, road=25):
    image = np.full((360, 480), road, dtype=np.uint8)
    for i in range(count):
        if direction == 'vertical':
            cv2.rectangle(image, (70+i*47, 170), (88+i*47, 285), light, -1)
        else:
            cv2.rectangle(image, (100, 124+i*30), (370, 135+i*30), light, -1)
    return image


class CrosswalkTests(unittest.TestCase):
    def setUp(self):
        self.detector = CrosswalkDetector()

    def test_six_or_more_parallel_separated_stripes(self):
        for n in (6, 7, 8):
            result = self.detector.detect(zebra(n))
            self.assertTrue(result['detected'], result)
            self.assertEqual(result['stripe_count'], n)

    def test_horizontal_bars(self):
        self.assertTrue(self.detector.detect(zebra(direction='horizontal'))['detected'])

    def test_five_bars_are_insufficient(self):
        self.assertFalse(self.detector.detect(zebra(5))['detected'])

    def test_lane_boundaries_and_single_stopline(self):
        image = np.full((360, 480), 25, np.uint8)
        cv2.line(image, (50, 355), (170, 80), 255, 8)
        cv2.line(image, (430, 355), (300, 80), 255, 8)
        self.assertFalse(self.detector.detect(image)['detected'])
        cv2.line(image, (60, 280), (420, 280), 255, 15)
        self.assertFalse(self.detector.detect(image)['detected'])

    def test_uniform_dark_bright_and_low_contrast(self):
        for image in (np.full((360, 480), 250, np.uint8), np.zeros((360, 480), np.uint8),
                      zebra(light=190, road=175)):
            self.assertFalse(self.detector.detect(image)['detected'])

    def test_dim_and_slanted_crosswalk(self):
        image = zebra(light=135, road=20)
        self.assertTrue(self.detector.detect(image)['detected'])
        mat = cv2.getRotationMatrix2D((240, 220), 12, 1)
        image = cv2.warpAffine(zebra(), mat, (480, 360), borderValue=25)
        self.assertTrue(self.detector.detect(image)['detected'])

    def test_perspective_crosswalk(self):
        source = np.float32([[30,100], [450,100], [450,340], [30,340]])
        target = np.float32([[105,115], [380,130], [450,335], [35,335]])
        image = cv2.warpPerspective(zebra(), cv2.getPerspectiveTransform(source,target), (480,360), borderValue=25)
        self.assertTrue(self.detector.detect(image)['detected'])

    def test_upper_wall_windows_excluded(self):
        image = np.full((360,480),25,np.uint8)
        for x in range(50, 420, 45):
            cv2.rectangle(image, (x,10), (x+15,90),235,-1)
        self.assertFalse(self.detector.detect(image)['detected'])

    def test_non_parallel_clutter(self):
        image = np.full((360,480),25,np.uint8)
        for i in range(7):
            points = cv2.boxPoints(((65+i*52,220),(16,65),i*22)).astype(np.int32)
            cv2.fillConvexPoly(image,points,235)
        self.assertFalse(self.detector.detect(image)['detected'])

    def test_invalid_inputs_and_config(self):
        for image in (None, 'bad', np.ones((300,300),np.float32), np.zeros((4,4),np.uint8)):
            self.assertFalse(self.detector.detect(image)['valid'])
        with self.assertRaises(ValueError):
            CrosswalkDetector({'stripe_min_count':3})

    def test_circular_leds_need_known_black_fixture(self):
        image = np.full((360,480,3),185,np.uint8)
        cv2.rectangle(image,(220,90),(280,240),(10,10,10),-1)
        cv2.circle(image,(250,200),10,(0,255,0),-1)
        self.assertEqual(TrafficDetector().detect(image)['status'],'green')
        self.assertFalse(self.detector.detect(image)['detected'])
        cv2.circle(image,(250,130),10,(0,0,255),-1)
        self.assertEqual(TrafficDetector().detect(image)['status'],'conflict')


class ObservationGatingTests(unittest.TestCase):
    def mode(self, mission=None, **kw):
        base=dict(stamp=10.,mode='patrol',camera_pose='road',gimbal_ready=True,traffic_active=False)
        base.update(mission or {})
        times=dict(mission_age_s=.1,capture_ros_s=10.05,now_ros_s=10.10)
        times.update(kw)
        return observation_mode(base,**times)

    def test_only_current_ready_road_pose_runs_crosswalk(self):
        self.assertEqual(self.mode(),'road')
        self.assertEqual(self.mode({'mode':'navigate'}),'road')
        for change in ({'gimbal_ready':False},{'camera_pose':'fire'},{'mode':'inspect'},{'stamp':9.}):
            self.assertIsNone(self.mode(change))

    def test_traffic_preempts_road(self):
        self.assertEqual(self.mode({'traffic_active':True}),'traffic')
        self.assertIsNone(self.mode({'traffic_active':True,'gimbal_ready':False}))

    def test_stale_future_and_invalid_frames_fail_closed(self):
        for changes in ({'capture_ros_s':9.7},{'capture_ros_s':10.2},{'capture_ros_s':float('nan')},
                        {'mission_age_s':.6},{'mission_age_s':-.1}):
            self.assertIsNone(self.mode(**changes))


if __name__=='__main__':
    unittest.main()
