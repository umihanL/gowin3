#!/usr/bin/env python3
"""Offline pixel equivalence and CPU timing; no camera, BPU or ROS is opened."""
import argparse
import json
from pathlib import Path
import platform
import statistics
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.legacy_detector import nv12_tensor
from rescue_vision.native_preprocess import NativeNV12, nv12_numpy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--iterations', type=int, default=20)
    args = parser.parse_args()
    native = NativeNV12(args.library)
    rng = np.random.default_rng(20261010)
    cases = {f'random_{seed}': rng.integers(0, 256, (640, 640, 3), dtype=np.uint8) for seed in range(4)}
    for name, color in [('black', (0,0,0)), ('white', (255,255,255)), ('red', (0,0,255)),
                        ('green', (0,255,0)), ('blue', (255,0,0)), ('letterbox_gray', (114,114,114))]:
        cases[name] = np.full((640, 640, 3), color, dtype=np.uint8)
    yy, xx = np.mgrid[:640, :640]
    cases['gradient'] = np.stack((xx % 256, yy % 256, (xx + yy) % 256), axis=-1).astype(np.uint8)
    checker = np.zeros((640, 640, 3), dtype=np.uint8)
    checker[0::2, 0::2] = [0,0,255]
    checker[0::2, 1::2] = [0,255,0]
    checker[1::2, 0::2] = [255,0,0]
    checker[1::2, 1::2] = [255,255,255]
    cases['chroma_sampling_checker'] = checker
    report = {'platform': platform.platform(), 'python': platform.python_version(), 'numpy': np.__version__,
              'size_wh': [640,640], 'camera_opened': False, 'bpu_loaded': False,
              'scope': 'CPU preprocessing only; existing services continue running',
              'native_library': str(native.path), 'accuracy': [], 'timing': {}}
    for name, image in cases.items():
        reference = nv12_tensor(image)
        for backend, operation in [('numpy_sparse_chroma', nv12_numpy), ('native_float32', native)]:
            result = operation(image)
            diff = np.abs(result.astype(np.int16) - reference.astype(np.int16))
            report['accuracy'].append(dict(case=name, backend=backend,
                                           bytes=int(result.size), max_abs_error=int(diff.max()),
                                           changed_bytes=int(np.count_nonzero(diff)), mean_abs_error=float(diff.mean()),
                                           y_changed=int(np.count_nonzero(diff[:640*640])),
                                           uv_changed=int(np.count_nonzero(diff[640*640:]))))
    sample = cases['random_0']
    functions = [('legacy', nv12_tensor), ('numpy_sparse_chroma', nv12_numpy), ('native_float32', native)]
    measured = {name: [] for name, _ in functions}
    for _, operation in functions:
        operation(sample)
    # Interleave all backends to reduce drift from CPU scaling/other live services.
    for _ in range(args.iterations):
        for name, operation in functions:
            start = time.perf_counter()
            operation(sample)
            measured[name].append((time.perf_counter() - start) * 1000)
    for name, values in measured.items():
        report['timing'][name] = dict(iterations=len(values), median_ms=statistics.median(values),
                                     p10_ms=float(np.percentile(values,10)), p90_ms=float(np.percentile(values,90)),
                                     minimum_ms=min(values), maximum_ms=max(values))
    report['bit_exact'] = all(row['changed_bytes'] == 0 for row in report['accuracy'])
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'bit_exact': report['bit_exact'], 'timing': report['timing'],
                      'max_abs_error': max(row['max_abs_error'] for row in report['accuracy']),
                      'output': str(destination)}))


if __name__ == '__main__':
    main()
