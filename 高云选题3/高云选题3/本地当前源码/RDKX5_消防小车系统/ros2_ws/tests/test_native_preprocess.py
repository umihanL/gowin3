"""Pixel compatibility of optimized full-range NV12 and explicit native loading."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.legacy_detector import nv12_tensor
from rescue_vision.native_preprocess import NativeNV12, choose_preprocessor, nv12_numpy


class NV12Tests(unittest.TestCase):
    def test_numpy_pixel_exact_for_colors_random_and_noncontiguous(self):
        rng = np.random.default_rng(14)
        cases = [rng.integers(0,256,(640,640,3),dtype=np.uint8),
                 rng.integers(0,256,(1088,1280,3),dtype=np.uint8)[::2,::2]]
        for color in ((0,0,0),(255,255,255),(0,0,255),(0,255,0),(255,0,0),(114,114,114)):
            cases.append(np.full((16,16,3),color,dtype=np.uint8))
        for index, image in enumerate(cases):
            with self.subTest(case=index):
                np.testing.assert_array_equal(nv12_numpy(image), nv12_tensor(image))

    def test_top_left_chroma_sampling_not_average_or_uv_swap(self):
        image = np.array([[[0,0,255],[0,255,0]],[[255,0,0],[255,255,255]]],dtype=np.uint8)
        converted = nv12_numpy(image)
        np.testing.assert_array_equal(converted, nv12_tensor(image))
        self.assertEqual(converted[-2:].tolist(), [84,255])

    def test_bad_images_and_explicit_fallback(self):
        for image in (np.empty((3,4,3),np.uint8),np.empty((4,4,3),np.float32),
                      np.empty((4,4),np.uint8),np.empty((0,4,3),np.uint8)):
            with self.assertRaises(ValueError):
                nv12_numpy(image)
        operation, name = choose_preprocessor(Path(__file__).with_name('missing_native_library.so'))
        self.assertIs(operation, nv12_numpy)
        self.assertEqual(name, 'numpy_sparse_chroma')

    def test_native_pixel_exact_when_built(self):
        try:
            native = NativeNV12()
        except OSError:
            self.skipTest('Native library not built on this host')
        rng = np.random.default_rng(25)
        for shape in ((640,640,3),(18,34,3),(1088,1280,3)):
            image = rng.integers(0,256,shape,dtype=np.uint8)
            np.testing.assert_array_equal(native(image),nv12_tensor(image))


if __name__ == '__main__':
    unittest.main()
