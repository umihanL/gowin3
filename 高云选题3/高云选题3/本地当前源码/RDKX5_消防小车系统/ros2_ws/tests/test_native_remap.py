"""Byte-exact interpolation, buffer ownership, and fail-closed native loading."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
sys.path.insert(0, str(ROOT/'reports'))
from rescue_vision.native_remap import NativeRemap
from rescue_vision.lens_correction import LensCorrector
from test_lens_correction import fixture
from prototype_native_remap import verify


class RemapLoadingTests(unittest.TestCase):
    def test_only_missing_library_can_fallback(self):
        with patch('rescue_vision.native_remap.NativeRemap', side_effect=FileNotFoundError('missing')):
            correction = LensCorrector(fixture(), remap_backend='native_or_opencv')
            self.assertEqual(correction.remap_backend, 'opencv')
            self.assertEqual(correction.remap_fallback_reason, 'missing')
            with self.assertRaises(FileNotFoundError):
                LensCorrector(fixture(), remap_backend='native')
        for error in (RuntimeError('bad ABI'), OSError('broken dependency'), AttributeError('missing symbol')):
            with patch('rescue_vision.native_remap.NativeRemap', side_effect=error):
                with self.assertRaises(type(error)):
                    LensCorrector(fixture(), remap_backend='native_or_opencv')

    def test_bad_backend_and_threads_fail(self):
        with self.assertRaises(ValueError):
            LensCorrector(fixture(), remap_backend='guess')
        for threads in (0, 9, True, 1.5):
            with self.assertRaises(ValueError):
                NativeRemap(threads=threads)
            with self.assertRaises(ValueError):
                LensCorrector(fixture(), remap_threads=threads)


class NativeRemapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = Path(os.environ.get('RESCUE_NATIVE_REMAP_LIBRARY',
            ROOT/'src/rescue_vision/rescue_vision/_native_remap.so'))
        if not cls.library.exists():
            raise unittest.SkipTest('Build native remap before native numerical tests')
        cls.remap = NativeRemap(cls.library, threads=2)

    def test_every_fraction_border_and_real_calibration_is_byte_exact(self):
        instances = {n: NativeRemap(self.library, threads=n) for n in (1, 2, 4, 8)}
        count, _ = verify(lambda image, maps, threads=1, table=0: instances[threads](image, maps),
                          ROOT/'config/gs130w_intrinsics_20261010.json',
                          ROOT/'logs/fire_lens_20261010/raw_physical_left.png')
        self.assertGreaterEqual(count, 30)

    def test_strided_inputs_are_unmodified_and_outputs_are_owned(self):
        image = np.random.default_rng(1).integers(0, 256, (60, 80, 3), dtype=np.uint8)
        image = image[::2, ::2]
        yy, xx = np.mgrid[:30, :40]
        maps = cv2.convertMaps((xx+.5).astype('float32'), (yy+.25).astype('float32'), cv2.CV_16SC2)
        before = image.copy()
        image.setflags(write=False)
        for array in maps:
            array.setflags(write=False)
        first = self.remap(image, maps)
        saved = first.copy()
        second = self.remap(np.zeros_like(image), maps)
        np.testing.assert_array_equal(image, before)
        np.testing.assert_array_equal(first, saved)
        np.testing.assert_array_equal(first, cv2.remap(image, *maps, cv2.INTER_LINEAR))
        self.assertFalse(np.shares_memory(first, second))

    def test_lens_cache_intrinsics_and_gray_fallback(self):
        with patch('rescue_vision.native_remap.NativeRemap', return_value=self.remap):
            native = LensCorrector(fixture(), remap_backend='native')
        legacy = LensCorrector(fixture())
        image = np.random.default_rng(5).integers(0, 256, (256, 320, 3), dtype=np.uint8)
        actual, expected = native.correct(image), legacy.correct(image)
        np.testing.assert_array_equal(actual.image, expected.image)
        np.testing.assert_array_equal(actual.newK, expected.newK)
        self.assertEqual(actual.metadata['remap_backend'], 'native_fixed_bilinear')
        actual.metadata.pop('remap_backend'); expected.metadata.pop('remap_backend')
        self.assertEqual(actual.metadata, expected.metadata)
        native.correct(image)
        gray = native.correct(image[:, :, 0])
        self.assertEqual(native.maps_built, 1)
        self.assertEqual(gray.metadata['remap_backend'], 'opencv')
        np.testing.assert_array_equal(gray.image, legacy.correct(image[:, :, 0]).image)
        with patch.object(self.remap, 'convert', return_value=-2):
            with self.assertRaisesRegex(RuntimeError, 'failed'):
                native.correct(image)

    def test_invalid_shapes_and_short_native_buffers_fail_before_write(self):
        image = np.ones((2, 2, 3), dtype=np.uint8)
        xy = np.zeros((2, 2, 2), dtype=np.int16)
        fractions = np.zeros((2, 2), dtype=np.uint16)
        for bad_image in (image.astype('float32'), image[:, :, 0], image[:0]):
            with self.assertRaises(ValueError):
                self.remap(bad_image, (xy, fractions))
        with self.assertRaises(ValueError):
            self.remap(image, (xy.astype('float32'), fractions))
        output = np.full_like(image, 47)
        code = self.remap.convert(image.ctypes.data, 1, 2, 2, xy.ctypes.data, xy.nbytes,
            fractions.ctypes.data, fractions.nbytes, 2, 2, output.ctypes.data, output.nbytes, 1)
        self.assertEqual(code, -2)
        self.assertTrue((output == 47).all())
        code = self.remap.convert(image.ctypes.data, image.nbytes, 2, 2, xy.ctypes.data, xy.nbytes,
            fractions.ctypes.data, fractions.nbytes, 2, 2, image.ctypes.data, image.nbytes, 1)
        self.assertEqual(code, -3)


if __name__ == '__main__':
    unittest.main()
