"""User's two real three-lens lamp photos plus explicit negative fixtures.

Image variants are regression checks, not additional independent accuracy data.
No camera, ROS publisher, UART, gimbal or vehicle control is opened by this test.
"""
import json
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.traffic import TrafficDetector, TrafficController

FIXTURES = Path(__file__).parent/'fixtures/traffic_three_lamp'


def photograph(color):
    return cv2.imdecode(np.fromfile(str(FIXTURES/(color+'_on.png')), np.uint8), cv2.IMREAD_COLOR)


def three_lamp_image(color=None, values=(120, 150, 100), panel=True, angle=25):
    image = np.full((360, 480, 3), 185, np.uint8)
    if panel:
        cv2.rectangle(image, (206, 70), (274, 290), (15, 15, 15), -1)
    colors = [(0, 0, values[0]), (0, values[1], values[1]), (0, values[2], 0)]
    if color == 'red':
        colors[0] = (0, 0, 255)
    elif color == 'green':
        colors[2] = (0, 255, 0)
    elif color == 'yellow':
        colors[1] = (0, 255, 255)
    for i, value in enumerate(colors):
        cv2.circle(image, (240, 110+70*i), 23, value, -1)
    if color == 'red_white':
        cv2.circle(image, (240, 110), 18, (255, 255, 255), -1)
    return cv2.warpAffine(image, cv2.getRotationMatrix2D((240, 180), angle, 1),
                          (480, 360), borderValue=(185, 185, 185))


class ThreeLampRegressionTests(unittest.TestCase):
    def setUp(self):
        self.detector = TrafficDetector(json.loads((ROOT/'config/traffic.fpga_navigation.json').read_text()))

    def test_real_fixture_scale_rotation_and_moderate_light_variants(self):
        for color in ('red', 'green'):
            original = photograph(color)
            for scale in (.35, .5, .75, 1., 1.5):
                with self.subTest(color=color, scale=scale):
                    resized = cv2.resize(original, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                    canvas = np.full((544, 640, 3), 185, np.uint8)
                    h, w = resized.shape[:2]
                    canvas[50:50+h, 210:210+w] = resized
                    self.assertEqual(self.detector.detect(canvas)['status'], color)
            for angle in (-15, 0, 15, 30):
                with self.subTest(color=color, angle=angle):
                    canvas = np.full((544, 640, 3), 185, np.uint8)
                    h, w = original.shape[:2]
                    canvas[130:130+h, 210:210+w] = original
                    rotated = cv2.warpAffine(canvas, cv2.getRotationMatrix2D((320, 272), angle, 1),
                                            (640, 544), borderValue=(185, 185, 185))
                    self.assertEqual(self.detector.detect(rotated)['status'], color)
            for gain in (.8, .9, 1.1):
                adjusted = np.clip(original.astype(float)*gain, 0, 255).astype(np.uint8)
                result = self.detector.detect(adjusted)
                # Strongly dimmed photos can lose emission evidence: conservative
                # unknown is valid. Never tune thresholds to force every variant.
                self.assertIn(result['status'], (color, 'unknown'))
                if result['status'] == 'unknown':
                    self.assertEqual(result['confidence'], 0.)

    def test_colored_caps_without_emission_cannot_be_green(self):
        for values in ((120, 150, 100), (230, 230, 230), (180, 170, 180), (90, 90, 90), (255, 255, 255)):
            with self.subTest(values=values):
                result = self.detector.detect(three_lamp_image(values=values))
                self.assertEqual(result['status'], 'unknown')
                self.assertEqual(result['reason'], 'three_lamp_brightness_ambiguous')
        self.assertEqual(self.detector.detect(three_lamp_image(color='yellow'))['status'], 'unknown')

    def test_red_or_uncertain_photographs_never_grant_crossing(self):
        controller = TrafficController({'settle_s': 0})
        self.assertTrue(controller.on_crosswalk('test-only', 0.))
        controller.tick(0., vehicle_stopped=True, gimbal_ready=True)
        red = photograph('red')
        dim_red = np.clip(red.astype(float)*.8, 0, 255).astype(np.uint8)
        off = three_lamp_image()
        for i, frame in enumerate([red, dim_red, off]*4):
            stamp = .1+i*.1
            result = self.detector.detect(frame, capture_time=stamp, frame_id=str(i))
            self.assertNotEqual(result['status'], 'green')
            output = controller.tick(stamp, result, vehicle_stopped=True, gimbal_ready=True)
            self.assertFalse(output['move_allowed'])


if __name__ == '__main__':
    unittest.main()
