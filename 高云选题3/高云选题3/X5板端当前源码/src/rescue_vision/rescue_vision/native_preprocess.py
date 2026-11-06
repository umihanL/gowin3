"""Optional native full-range NV12; pure NumPy fallback preserves model pixels.

Loading this module opens no camera, starts no thread and does not load a model.
The native library is built separately with tools/build_native_preprocess.sh.
"""
import ctypes
from pathlib import Path

import numpy as np


def _image(padded):
    image = np.asarray(padded)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError('Expected HxWx3 uint8 BGR')
    height, width = image.shape[:2]
    if not height or not width or height % 2 or width % 2:
        raise ValueError('NV12 requires positive even dimensions')
    return image


def nv12_numpy(padded):
    """Same float32 arithmetic as legacy, but chroma is computed only at [::2,::2]."""
    image = _image(padded)
    height, width = image.shape[:2]
    b, g, r = (image[..., index].astype(np.float32) for index in range(3))
    y = .299*r + .587*g + .114*b
    small_b, small_g, small_r = b[::2, ::2], g[::2, ::2], r[::2, ::2]
    u = -.169*small_r - .331*small_g + .5*small_b + 128
    v = .5*small_r - .419*small_g - .081*small_b + 128
    result = np.empty((height * 3 // 2, width), dtype=np.uint8)
    result[:height] = np.clip(y, 0, 255, out=y)
    result[height:, 0::2] = np.clip(u, 0, 255, out=u)
    result[height:, 1::2] = np.clip(v, 0, 255, out=v)
    return result.reshape(-1)


class NativeNV12:
    """Explicit native adapter. Construction reports a missing/incompatible library."""
    def __init__(self, library_path=None):
        self.path = Path(library_path) if library_path else Path(__file__).with_name('_native_preprocess.so')
        self.library = ctypes.CDLL(str(self.path.resolve()))
        version = self.library.rescue_nv12_version
        version.argtypes, version.restype = [], ctypes.c_int
        if version() != 1:
            raise RuntimeError('Unsupported native NV12 library version')
        self.convert = self.library.rescue_bgr_to_nv12
        self.convert.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t,
                                 ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t]
        self.convert.restype = ctypes.c_int
        self._prepare_hd = getattr(self.library, 'rescue_prepare_hd_nv12', None)
        if self._prepare_hd is not None:
            self._prepare_hd.argtypes = self.convert.argtypes
            self._prepare_hd.restype = ctypes.c_int
        self._prepare_hd_parallel = getattr(self.library, 'rescue_prepare_hd_nv12_parallel', None)
        if self._prepare_hd_parallel is not None:
            self._prepare_hd_parallel.argtypes = self.convert.argtypes + [ctypes.c_int]
            self._prepare_hd_parallel.restype = ctypes.c_int

    @property
    def hd_available(self):
        return self._prepare_hd is not None

    @property
    def hd_parallel_available(self):
        return self._prepare_hd_parallel is not None

    def prepare_hd(self, image, threads=1):
        """Exact HD letterbox + NV12, retaining full-resolution preview pixels.

        The fixed 2:1 model resize matches OpenCV INTER_LINEAR uint8 rounding.
        No input is modified, and each call owns its independent output buffer.
        Other image shapes must continue through the general letterbox path.
        Parallel preparation is opt-in and requires an OpenMP-enabled build;
        callers select a bounded worker count after whole-pipeline benchmarks.
        """
        image = _image(image)
        if image.shape != (1088, 1280, 3):
            raise ValueError('Fused HD preparation requires 1280x1088 BGR input')
        if self._prepare_hd is None:
            raise RuntimeError('Native library has no fused HD preparation symbol')
        if not isinstance(threads, int) or isinstance(threads, bool) or not 1 <= threads <= 8:
            raise ValueError('Fused preparation threads must be an integer in 1..8')
        if threads > 1 and self._prepare_hd_parallel is None:
            raise RuntimeError('Native library was built without parallel HD preparation')
        image = np.ascontiguousarray(image)
        result = np.empty(640 * 640 * 3 // 2, dtype=np.uint8)
        args = (image.ctypes.data, 1088, 1280, image.strides[0], result.ctypes.data, result.size)
        code = self._prepare_hd(*args) if threads == 1 else self._prepare_hd_parallel(*args, threads)
        if code:
            raise RuntimeError('Fused HD conversion failed with code ' + str(code))
        return result

    def __call__(self, padded):
        image = np.ascontiguousarray(_image(padded))
        height, width = image.shape[:2]
        result = np.empty(height * width * 3 // 2, dtype=np.uint8)
        code = self.convert(image.ctypes.data, height, width, image.strides[0],
                            result.ctypes.data, result.size)
        if code:
            raise RuntimeError('Native NV12 conversion failed with code ' + str(code))
        return result


def choose_preprocessor(library_path=None):
    """Return (callable, backend label); callers may log which route was selected."""
    try:
        return NativeNV12(library_path), 'native_float32'
    except (OSError, RuntimeError, AttributeError):
        return nv12_numpy, 'numpy_sparse_chroma'
