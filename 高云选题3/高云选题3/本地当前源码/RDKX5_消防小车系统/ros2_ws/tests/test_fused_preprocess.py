"""The fixed HD fast path must give the model byte-identical NV12 input."""
import os
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.legacy_detector import letterbox, nv12_tensor
from rescue_vision.native_preprocess import NativeNV12


def compatibility_images():
    for seed in range(16):
        yield f'random_{seed}', np.random.default_rng(seed).integers(0, 256, (1088, 1280, 3), dtype=np.uint8)
    for color in ((0, 0, 0), (255, 255, 255), (114, 114, 114),
                  (0, 0, 255), (0, 255, 0), (255, 0, 0),
                  (255, 255, 0), (0, 255, 255), (255, 0, 255), (1, 1, 1)):
        yield f'color_{color}', np.full((1088, 1280, 3), color, dtype=np.uint8)
    yy, xx = np.indices((1088, 1280))
    yield 'gradient', np.stack((xx % 256, yy % 256, (xx + yy) % 256), axis=-1).astype(np.uint8)
    yield 'checkerboard', np.repeat((((xx + yy) % 2) * 255).astype(np.uint8)[..., None], 3, axis=2)
    # Exercise every possible 2x2 uint8 sum (including both half-integer ties).
    sums = np.arange(544 * 640 * 3).reshape(544, 640, 3) % 1021
    image = np.empty((1088, 1280, 3), dtype=np.uint8)
    for index, (dy, dx) in enumerate(((0, 0), (0, 1), (1, 0), (1, 1))):
        image[dy::2, dx::2] = np.clip(sums - index * 255, 0, 255)
    yield 'all_rounding_sums', image
    yield 'negative_strides', image[::-1, ::-1]
    canvas = np.zeros((1088, 2560, 3), dtype=np.uint8)
    canvas[:, ::2] = image
    yield 'column_stride', canvas[:, ::2]


class FusedHDTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.native = NativeNV12(os.environ.get('RESCUE_NATIVE_LIBRARY'))
        except OSError as error:
            raise unittest.SkipTest(str(error))
        if not cls.native.hd_available:
            raise unittest.SkipTest('Native library lacks fused HD symbol')

    def test_every_model_input_byte_matches_original(self):
        for name, image in compatibility_images():
            with self.subTest(image=name):
                original = image.copy()
                padded, meta = letterbox(image)
                self.assertEqual(meta, dict(gain=.5, left=0, top=48, width=1280, height=1088))
                expected = nv12_tensor(padded)
                actual = self.native.prepare_hd(image)
                self.assertEqual(actual.shape, (614400,))
                np.testing.assert_array_equal(actual, expected)
                np.testing.assert_array_equal(image, original)

    def test_general_nv12_path_still_matches(self):
        image = np.random.default_rng(105).integers(0, 256, (34, 18, 3), dtype=np.uint8)
        np.testing.assert_array_equal(self.native(image), nv12_tensor(image))

    def test_parallel_workers_preserve_every_byte(self):
        if not self.native.hd_parallel_available:
            self.skipTest('Library was built without OpenMP')
        image = np.random.default_rng(208).integers(0, 256, (1088, 1280, 3), dtype=np.uint8)
        expected = nv12_tensor(letterbox(image)[0])
        for threads in (2, 4):
            with self.subTest(threads=threads):
                np.testing.assert_array_equal(self.native.prepare_hd(image, threads=threads), expected)
        for threads in (0, 9, -1, 1.5, True):
            with self.assertRaises(ValueError):
                self.native.prepare_hd(image, threads=threads)

    def test_rejects_wrong_dimensions_and_keeps_independent_outputs(self):
        for shape in ((640, 640, 3), (1088, 1282, 3)):
            with self.assertRaises(ValueError):
                self.native.prepare_hd(np.zeros(shape, dtype=np.uint8))
        image = np.zeros((1088, 1280, 3), dtype=np.uint8)
        first = self.native.prepare_hd(image)
        saved = first.copy()
        image[:] = 255
        second = self.native.prepare_hd(image)
        self.assertFalse(np.shares_memory(first, second))
        np.testing.assert_array_equal(first, saved)
        self.assertFalse(np.array_equal(first, second))


if __name__ == '__main__':
    unittest.main()
