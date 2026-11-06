"""Synchronous hbDNN execution without holding Python's GIL during BPU waits.

This adapter accepts only this project's packed 640 NV12 / six float NHWC head
contract. It changes neither preprocessing nor model evaluation. Output arrays
own their Python storage and remain valid after the next call or after close().
An instance serializes forward/close; different instances may use separate BPU
contexts. No hardware access occurs when importing this module.
"""
import ctypes
from pathlib import Path
import threading

import numpy as np

OUTPUT_NAMES = ('cls_s8', 'box_s8', 'cls_s16', 'box_s16', 'cls_s32', 'box_s32')
OUTPUT_SHAPES = tuple((1, side, side, channels) for side in (80, 40, 20) for channels in (3, 64))
INPUT_BYTES = 640 * 640 * 3 // 2
OUTPUT_FLOATS = sum(int(np.prod(shape)) for shape in OUTPUT_SHAPES)


def _packed_input(value):
    array = np.asarray(value)
    if array.dtype != np.uint8 or array.size != INPUT_BYTES:
        raise ValueError('Expected 614400 packed uint8 NV12 bytes for 640x640 input')
    return np.ascontiguousarray(array).reshape(-1)


class NativeRuntime:
    """Persistent buffers and exact float-output inference, with explicit close."""
    backend = 'native_hbdnn'
    names = OUTPUT_NAMES

    def __init__(self, model, library_path=None):
        self._lock = threading.Lock()
        self._context = None
        self.path = Path(library_path) if library_path else Path(__file__).with_name('_native_runtime.so')
        self.library = ctypes.CDLL(str(self.path.resolve()))
        version = self.library.rescue_dnn_version
        version.argtypes, version.restype = [], ctypes.c_int
        if version() != 1:
            raise RuntimeError('Unsupported native hbDNN adapter version')
        self._create = self.library.rescue_dnn_create
        self._create.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_size_t]
        self._create.restype = ctypes.c_void_p
        self._forward = self.library.rescue_dnn_forward
        self._forward.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t,
                                 ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t]
        self._forward.restype = ctypes.c_int
        self._destroy = self.library.rescue_dnn_destroy
        self._destroy.argtypes, self._destroy.restype = [ctypes.c_void_p], ctypes.c_int
        error = ctypes.create_string_buffer(2048)
        self._context = self._create(str(Path(model).resolve()).encode('utf-8'), error, len(error))
        if not self._context:
            raise RuntimeError(error.value.decode('utf-8', errors='replace'))

    def forward(self, packed_nv12):
        packed = _packed_input(packed_nv12)
        result = np.empty(OUTPUT_FLOATS, dtype=np.float32)
        error = ctypes.create_string_buffer(2048)
        with self._lock:
            if not self._context:
                raise RuntimeError('Native hbDNN runtime is closed')
            code = self._forward(self._context, packed.ctypes.data, packed.size,
                                 result.ctypes.data, result.size, error, len(error))
            if code:
                raise RuntimeError(error.value.decode('utf-8', errors='replace'))
        outputs, offset = [], 0
        for shape in OUTPUT_SHAPES:
            count = int(np.prod(shape))
            outputs.append(result[offset:offset + count].reshape(shape))
            offset += count
        return outputs

    def close(self):
        with self._lock:
            if self._context:
                status = self._destroy(self._context)
                self._context = None
                if status:
                    raise RuntimeError('Native hbDNN task completion unconfirmed; DMA context quarantined. Restart process.')

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def __del__(self):
        if getattr(self, '_context', None):
            try:
                self.close()
            except Exception:
                # Explicit close reports errors; native quarantine also prints
                # a diagnostic. Never throw from garbage collection.
                pass


def try_native_runtime(model, library_path=None):
    """Return (runtime, reason). Caller decides/logs fallback to existing backend.

    Only an unavailable shared library allows fallback. An incompatible model,
    native ABI, initialization error or forward error always propagates, so a
    broken deployment is not silently hidden behind another backend.
    """
    try:
        return NativeRuntime(model, library_path), None
    except OSError as error:
        return None, str(error)
