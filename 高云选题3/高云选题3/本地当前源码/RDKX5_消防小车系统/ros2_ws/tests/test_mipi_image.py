"""MIPI storage and queue behavior, independent of ROS and physical cameras."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.mipi_image import LatestStereoPairer, image_to_bgr, stamp_nanoseconds


class ImageDecodeTests(unittest.TestCase):
    def test_nv12_black_white_and_color_uv_order(self):
        # Limited-range BT.601 black, white, red and blue; UV must not be VU.
        for y, u, v, expected in ((16, 128, 128, (0, 0, 0)), (235, 128, 128, (255, 255, 255)),
                                  (82, 90, 240, (0, 0, 255)), (41, 240, 110, (255, 0, 0))):
            data = bytes([y] * 8 + [u, v, u, v])
            result = image_to_bgr(data, 4, 2, 4, 'nv12')
            self.assertEqual(result.shape, (2, 4, 3))
            np.testing.assert_allclose(result, np.tile(expected, (2, 4, 1)), atol=2)

    def test_nv12_removes_padding_from_both_planes(self):
        packed = np.array([[82] * 4, [82] * 4, [90, 240, 90, 240]], dtype=np.uint8)
        padded = np.full((3, 8), 9, dtype=np.uint8)
        padded[:, :4] = packed
        expected = cv2.cvtColor(packed, cv2.COLOR_YUV2BGR_NV12)
        np.testing.assert_array_equal(image_to_bgr(padded.tobytes(), 4, 2, 8, 'NV12'), expected)

    def test_rgb_bgr_and_mono_padding(self):
        rgb = bytes([255, 0, 0, 0, 0, 255, 99, 99])
        np.testing.assert_array_equal(image_to_bgr(rgb, 2, 1, 8, 'rgb8'), [[[0, 0, 255], [255, 0, 0]]])
        np.testing.assert_array_equal(image_to_bgr(rgb, 2, 1, 8, 'bgr8'), [[[255, 0, 0], [0, 0, 255]]])
        np.testing.assert_array_equal(image_to_bgr(bytes([23, 210, 99, 99]), 2, 1, 4, 'mono8'),
                                      [[[23, 23, 23], [210, 210, 210]]])

    def test_invalid_storage_is_rejected(self):
        for args in ((bytes(12), 4, 2, 3, 'nv12'), (bytes(12), 3, 2, 4, 'nv12'),
                     (bytes(12), 4, 3, 4, 'nv12'), (bytes(11), 4, 2, 4, 'nv12'),
                     (bytes(13), 4, 2, 4, 'nv12'), (bytes(12), 4, 2, 4, 'yuv420'),
                     (bytes(8), 4, 2, 4, 'bgr8')):
            with self.subTest(args=args[1:]), self.assertRaises(ValueError):
                image_to_bgr(*args)

    def test_real_driver_dimensions(self):
        image = image_to_bgr(bytes([128]) * 2088960, 1280, 1088, 1280, 'nv12')
        self.assertEqual(image.shape, (1088, 1280, 3))


class PairingTests(unittest.TestCase):
    epoch = 1_791_500_000_000_000_000

    def stamp(self, delta_ms):
        return self.epoch + round(delta_ms * 1e6)

    def add(self, pairer, eye, ms, name, now_ms=100):
        return pairer.push(eye, self.stamp(ms), name, self.stamp(now_ms))

    def test_timestamp_preserves_nanosecond_precision(self):
        value = stamp_nanoseconds(SimpleNamespace(sec=1_791_500_000, nanosec=123456789))
        self.assertEqual(value, self.epoch + 123456789)

    def test_newest_pair_skips_backlog_and_never_reuses(self):
        pairer = LatestStereoPairer()
        for ms in (10, 40, 70):
            self.add(pairer, 'left', ms, 'L' + str(ms))
            self.add(pairer, 'right', ms + 2, 'R' + str(ms))
        left, right = pairer.take_latest(self.stamp(100))
        self.assertEqual((left.payload, right.payload), ('L70', 'R70'))
        self.assertEqual(right.stamp_ns - left.stamp_ns, 2_000_000)
        self.assertEqual(pairer.counts['skipped'], 4)
        self.assertIsNone(pairer.take_latest(self.stamp(100)))

    def test_mismatched_eyes_wait_for_matching_frame(self):
        pairer = LatestStereoPairer(max_skew_s=.012)
        self.add(pairer, 'left', 20, 'Lold')
        self.add(pairer, 'right', 70, 'R70')
        self.assertIsNone(pairer.take_latest(self.stamp(100)))
        self.add(pairer, 'left', 71, 'L71')
        left, right = pairer.take_latest(self.stamp(100))
        self.assertEqual((left.payload, right.payload), ('L71', 'R70'))

    def test_stale_future_duplicates_and_out_of_order_rejected(self):
        pairer = LatestStereoPairer(max_age_s=.5)
        self.assertFalse(self.add(pairer, 'left', -501, 'stale', 0))
        self.assertFalse(self.add(pairer, 'left', 101, 'future', 100))
        self.assertTrue(self.add(pairer, 'left', 90, 'valid'))
        self.assertFalse(self.add(pairer, 'left', 90, 'duplicate'))
        self.assertFalse(self.add(pairer, 'left', 80, 'out of order'))
        self.assertTrue(self.add(pairer, 'right', 92, 'valid'))
        self.assertIsNone(pairer.take_latest(self.stamp(700)))
        self.assertEqual(pairer.counts['expired'], 2)

    def test_queue_is_bounded_if_other_eye_stops(self):
        pairer = LatestStereoPairer(queue_size=3)
        for ms in range(10):
            self.add(pairer, 'left', ms, ms)
        self.assertEqual(len(pairer.queues['left']), 3)
        self.assertEqual(pairer.counts['queue_overflow'], 7)

    def test_pair_skew_limit_and_same_stamp_not_sync_claim(self):
        pairer = LatestStereoPairer(max_skew_s=.012)
        self.add(pairer, 'left', 30, 'left')
        self.add(pairer, 'right', 43, 'outside')
        self.assertIsNone(pairer.take_latest(self.stamp(100)))
        self.add(pairer, 'left', 43, 'equal_stamp')
        left, right = pairer.take_latest(self.stamp(100))
        self.assertEqual(left.stamp_ns, right.stamp_ns)
        self.assertFalse(hasattr(pairer, 'synchronization_verified'))


if __name__ == '__main__':
    unittest.main()
