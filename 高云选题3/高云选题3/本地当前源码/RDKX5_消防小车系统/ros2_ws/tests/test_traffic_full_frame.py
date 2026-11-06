"""Real supplied photos with full-frame coverage; derivatives are not new samples."""
import sys
from pathlib import Path
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.traffic import TrafficDetector


def photo(name):
    return cv2.imdecode(np.fromfile(ROOT/'tests/fixtures/traffic_three_lamp'/name,
                                  np.uint8), cv2.IMREAD_COLOR)


def embed(image, x, y):
    canvas = np.full((544, 640, 3), 160, np.uint8)
    h, w = image.shape[:2]
    canvas[y:y+h, x:x+w] = image
    return canvas


class TrafficFullFrameTests(unittest.TestCase):
    def setUp(self):
        self.detector = TrafficDetector({'fixture_layout': 'three_lamp',
                                        'roi_normalized': [0, 0, 1, 1]})

    def test_new_annotated_red_also_works_without_any_drawn_overlay(self):
        image = photo('red_on_annotated.png')
        # Pixel crop lies INSIDE the preexisting rectangle. No box line or
        # Chinese text remains; it is never used as evidence about lamp state.
        for crop in (image, image[75:254, 106:195]):
            result = self.detector.detect(crop)
            self.assertEqual(result['status'], 'red')
            self.assertGreaterEqual(result['photometry']['red']['white_core_fraction'], .35)
        # Original screenshot decoration with the actual lamp erased is not a
        # fixture. Recognition cannot derive a label from its red annotation.
        decoration = image.copy()
        decoration[74:254, 105:197] = 160
        self.assertEqual(self.detector.detect(decoration)['status'], 'unknown')

    def test_complete_edge_fixtures_at_four_scales(self):
        examples = [('red', photo('red_on.png')), ('green', photo('green_on.png')),
                    ('red', photo('red_on_annotated.png')[75:254, 106:195])]
        for color, source in examples:
            for scale in (.35, .5, .75, 1.):
                image = cv2.resize(source, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                h, w = image.shape[:2]
                for edge, x, y in [('left', 0, 150), ('right', 640-w, 150),
                                   ('top', 250, 0), ('bottom', 250, 544-h)]:
                    with self.subTest(color=color, scale=scale, edge=edge):
                        self.assertEqual(self.detector.detect(embed(image, x, y))['status'], color)

    def test_old_center_crop_loses_a_complete_edge_fixture(self):
        image = photo('red_on_annotated.png')[75:254, 106:195]
        canvas = embed(image, 0, 150)
        old = TrafficDetector({'fixture_layout': 'three_lamp',
                               'roi_normalized': [.1, .05, .9, .9]})
        self.assertEqual(old.detect(canvas)['status'], 'unknown')
        self.assertEqual(self.detector.detect(canvas)['status'], 'red')

    def test_clipped_fixture_is_not_accepted_as_green(self):
        source = photo('green_on.png')
        # Lose the top red lens, or the illuminated green lens: neither image
        # establishes all three positions. Full-frame coverage adds no fallback.
        for partial in (source[75:], source[:105]):
            result = self.detector.detect(embed(partial, 200, 150))
            self.assertEqual(result['status'], 'unknown')


if __name__ == '__main__':
    unittest.main()
