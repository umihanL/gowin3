#!/usr/bin/env python3
"""Opt-in equivalence/latency benchmark; does not access cameras or load a model."""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'rescue_vision'))
sys.path.insert(0, str(ROOT / 'tests'))
from rescue_vision.native_preprocess import NativeNV12
from rescue_vision.legacy_detector import letterbox, nv12_tensor, read_image
from test_fused_preprocess import compatibility_images


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', default=None)
    parser.add_argument('--rounds', type=int, default=50)
    parser.add_argument('--threads', type=int, default=8)
    parser.add_argument('--workers', type=int, default=1)
    parser.add_argument('--real-image', action='append', default=[])
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    cv2.setNumThreads(args.threads)
    native = NativeNV12(args.library)
    verified = []
    images = list(compatibility_images())
    for path in args.real_image:
        image = read_image(path)
        if image.shape != (1088, 1280, 3):
            raise ValueError(f'Real image must retain original HD pixels: {path} {image.shape}')
        images.append((str(path), image))
    for name, image in images:
        expected = nv12_tensor(letterbox(image)[0])
        actual = native.prepare_hd(image, threads=args.workers)
        np.testing.assert_array_equal(actual, expected, err_msg=name)
        verified.append(name)
    # Interleave routes, avoiding timer/thermal ordering bias.
    old, new = [], []
    image = images[-1][1] if args.real_image else images[0][1]
    for index in range(args.rounds + 10):
        for label in (('old', 'new') if index % 2 else ('new', 'old')):
            started = time.perf_counter()
            result = native(letterbox(image)[0]) if label == 'old' else native.prepare_hd(image, threads=args.workers)
            elapsed = (time.perf_counter() - started) * 1000
            if index >= 10:
                (old if label == 'old' else new).append(elapsed)
    summarize = lambda values: dict(median_ms=float(np.median(values)),
                                    p90_ms=float(np.percentile(values, 90)),
                                    min_ms=float(min(values)), max_ms=float(max(values)))
    report = dict(opencv=cv2.__version__, threads=cv2.getNumThreads(), workers=args.workers,
                  verified_cases=verified, bytes_per_case=614400,
                  exact=True, rounds=args.rounds, separated=summarize(old),
                  fused=summarize(new))
    report['median_speedup'] = report['separated']['median_ms'] / report['fused']['median_ms']
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
