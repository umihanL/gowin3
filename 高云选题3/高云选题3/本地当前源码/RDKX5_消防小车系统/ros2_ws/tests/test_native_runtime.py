"""CPU-only contract/lifetime tests; BPU equivalence uses benchmark_native_runtime."""
import ctypes
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.native_runtime import INPUT_BYTES, NativeRuntime, OUTPUT_SHAPES, try_native_runtime


class Function:
    def __init__(self, operation):
        self.operation = operation
    def __call__(self, *args):
        return self.operation(*args)


class FakeLibrary:
    def __init__(self):
        self.calls = 0
        self.destroyed = 0
        self.rescue_dnn_version = Function(lambda: 1)
        self.rescue_dnn_create = Function(lambda *_: 1)
        self.rescue_dnn_forward = Function(self.forward)
        self.rescue_dnn_destroy = Function(self.destroy)

    def forward(self, handle, source, input_bytes, destination, count, error, capacity):
        self.calls += 1
        array = np.ctypeslib.as_array((ctypes.c_float * count).from_address(destination))
        array[:] = self.calls
        return 0

    def destroy(self, *_):
        self.destroyed += 1
        return 0


class NativeRuntimeContractTest(unittest.TestCase):
    def setUp(self):
        self.library = FakeLibrary()
        self.patcher = patch('rescue_vision.native_runtime.ctypes.CDLL', return_value=self.library)
        self.patcher.start()
        self.runtime = NativeRuntime('dummy.bin')

    def tearDown(self):
        self.runtime.close()
        self.patcher.stop()

    def test_outputs_own_storage_across_forward_and_close(self):
        first = self.runtime.forward(np.zeros(INPUT_BYTES, dtype=np.uint8))
        second = self.runtime.forward(np.zeros(INPUT_BYTES, dtype=np.uint8))
        self.runtime.close()
        self.assertEqual(tuple(x.shape for x in first), OUTPUT_SHAPES)
        self.assertTrue(all(np.all(x == 1) and x.dtype == np.float32 for x in first))
        self.assertTrue(all(np.all(x == 2) for x in second))
        self.assertEqual(self.library.destroyed, 1)

    def test_reject_bad_inputs_before_native_call(self):
        for bad in (np.zeros(INPUT_BYTES - 1, dtype=np.uint8), np.zeros(INPUT_BYTES, dtype=np.float32)):
            with self.assertRaises(ValueError):
                self.runtime.forward(bad)
        self.assertEqual(self.library.calls, 0)

    def test_closed_context_is_not_used(self):
        self.runtime.close()
        with self.assertRaises(RuntimeError):
            self.runtime.forward(np.zeros(INPUT_BYTES, dtype=np.uint8))
        self.assertEqual(self.library.calls, 0)

    def test_initialization_fallback_is_reported(self):
        with patch('rescue_vision.native_runtime.ctypes.CDLL', side_effect=OSError('missing library')):
            runtime, reason = try_native_runtime('dummy.bin')
        self.assertIsNone(runtime)
        self.assertIn('missing library', reason)

    def test_forward_error_is_not_silently_fallback(self):
        self.library.rescue_dnn_forward.operation = lambda *_: -1
        with self.assertRaises(RuntimeError):
            self.runtime.forward(np.zeros(INPUT_BYTES, dtype=np.uint8))

    def test_quarantined_context_is_not_freed_twice(self):
        self.library.rescue_dnn_destroy.operation = lambda *_: -1
        with self.assertRaisesRegex(RuntimeError, 'quarantined'):
            self.runtime.close()
        self.runtime.close()
        with self.assertRaises(RuntimeError):
            self.runtime.forward(np.zeros(INPUT_BYTES, dtype=np.uint8))


if __name__ == '__main__':
    unittest.main()
