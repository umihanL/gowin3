"""Exact native BGR fixed-map remap. No library/runtime error is silently ignored."""
import ctypes
from pathlib import Path
import numpy as np


class NativeRemap:
    def __init__(self, library_path=None, threads=8):
        if isinstance(threads, bool) or not isinstance(threads, int) or not 1 <= threads <= 8:
            raise ValueError('Remap threads must be an integer in 1..8')
        self.threads = threads
        self.path = Path(library_path) if library_path else Path(__file__).with_name('_native_remap.so')
        if not self.path.is_file():
            raise FileNotFoundError('Native remap library missing: ' + str(self.path))
        self.library = ctypes.CDLL(str(self.path.resolve()))
        version = self.library.rescue_remap_version
        version.argtypes, version.restype = [], ctypes.c_int
        if version() != 1:
            raise RuntimeError('Unsupported native remap ABI')
        self.convert = self.library.rescue_remap_bgr
        self.convert.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_int,
                                ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t,
                                ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        self.convert.restype = ctypes.c_int

    def __call__(self, image, maps):
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('Native remap requires HxWx3 uint8 BGR')
        if not isinstance(maps, (tuple, list)) or len(maps) != 2:
            raise ValueError('Native remap requires two fixed-point maps')
        xy, fractions = maps
        if (not isinstance(xy, np.ndarray) or not isinstance(fractions, np.ndarray) or
                xy.dtype != np.int16 or fractions.dtype != np.uint16 or
                fractions.ndim != 2 or xy.shape != (*fractions.shape, 2)):
            raise ValueError('Native remap requires CV_16SC2 and matching CV_16UC1 maps')
        height, width = image.shape[:2]
        out_height, out_width = fractions.shape
        if any(not 0 < value < 32767 for value in (height, width, out_height, out_width)):
            raise ValueError('Remap dimensions must be in 1..32766')
        image, xy, fractions = (np.require(a, requirements=('C', 'A')) for a in (image, xy, fractions))
        result = np.empty((out_height, out_width, 3), dtype=np.uint8)
        code = self.convert(image.ctypes.data, image.nbytes, width, height,
                            xy.ctypes.data, xy.nbytes, fractions.ctypes.data, fractions.nbytes,
                            out_width, out_height, result.ctypes.data, result.nbytes, self.threads)
        if code:
            raise RuntimeError('Native remap failed with code ' + str(code))
        return result
