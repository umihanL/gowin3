#!/usr/bin/env python3
"""Board-only exact-output comparison against pyeasy, followed by timing.

Stop other detector processes before measuring; this script opens the BPU but
does not open a camera, modify a model or command the robot.
"""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import threading
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.legacy_detector import OUTPUT_NAMES, detections, letterbox, read_image
from rescue_vision.native_preprocess import choose_preprocessor
from rescue_vision.native_runtime import NativeRuntime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--library')
    parser.add_argument('--image', action='append', default=[])
    parser.add_argument('--iterations', type=int, default=20)
    args = parser.parse_args()
    report = {'model': args.model, 'model_sha256': hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
              'camera_opened': False, 'bpu_loaded': True, 'accuracy': [], 'timing': {}, 'gil_progress': {}}
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    native = None
    try:
        from hobot_dnn import pyeasy_dnn
        model = pyeasy_dnn.load(args.model)[0]
        names = [output.name for output in model.outputs]
        native = NativeRuntime(args.model, args.library)

        def reference(packed):
            outputs = model.forward(packed)
            by_name = {name: np.asarray(tensor.buffer) for name, tensor in zip(names, outputs)}
            return [by_name[name] for name in OUTPUT_NAMES]

        preprocess, report['preprocess_backend'] = choose_preprocessor()
        rng = np.random.default_rng(20261010)
        cases = {name: np.full((640, 640, 3), color, dtype=np.uint8)
                 for name, color in [('black', (0, 0, 0)), ('gray', (114, 114, 114)),
                                     ('white', (255, 255, 255)), ('red', (0, 0, 255))]}
        cases['random'] = rng.integers(0, 256, (640, 640, 3), dtype=np.uint8)
        yy, xx = np.mgrid[:640, :640]
        cases['gradient'] = np.stack((xx % 256, yy % 256, (xx + yy) % 256), axis=-1).astype(np.uint8)
        for index, path in enumerate(args.image):
            cases[f'image_{index}_{Path(path).name}'] = read_image(path)
        for case, image in cases.items():
            padded, meta = letterbox(image)
            packed = preprocess(padded)
            expected = reference(packed)
            actual = native.forward(packed)
            result = {'case': case, 'heads': [], 'detections_equal': {}}
            for name, a, b in zip(OUTPUT_NAMES, expected, actual):
                result['heads'].append({'name': name, 'shape': list(a.shape),
                    'float_values': a.size, 'bit_exact': bool(np.array_equal(a.view(np.uint8), b.view(np.uint8))),
                    'changed_values': int(np.count_nonzero(a != b)),
                    'max_abs_error': float(np.max(np.abs(a - b)))})
            for label, class_ids in [('all_classes', None), ('fire_only', (0,))]:
                result['detections_equal'][label] = detections(expected, meta, class_ids=class_ids) == detections(actual, meta, class_ids=class_ids)
            report['accuracy'].append(result)
        report['bit_exact'] = all(head['bit_exact'] for case in report['accuracy'] for head in case['heads'])
        report['detections_equal'] = all(all(case['detections_equal'].values()) for case in report['accuracy'])
        if not report['bit_exact'] or not report['detections_equal']:
            raise RuntimeError('Native/pyeasy output equivalence failed; do not deploy')
        saved = native.forward(packed)
        saved_bytes = [array.tobytes() for array in saved]
        native.forward(preprocess(cases['black']))
        report['outputs_survive_next_forward'] = saved_bytes == [array.tobytes() for array in saved]
        functions = [('pyeasy', reference), ('native_hbdnn', native.forward)]
        measured = {name: [] for name, _ in functions}
        for _ in range(args.iterations):
            for name, operation in functions:
                started = time.perf_counter()
                operation(packed)
                measured[name].append((time.perf_counter() - started) * 1000)
        for name, samples in measured.items():
            report['timing'][name] = {'iterations': len(samples), 'median_ms': statistics.median(samples),
                'p90_ms': float(np.percentile(samples, 90)), 'minimum_ms': min(samples), 'maximum_ms': max(samples)}
        # The heartbeat needs the GIL but sleeps between checks. Native waits
        # should permit regular progress; pyeasy blocks throughout its C call.
        for name, operation in functions:
            ticks, stop = [], threading.Event()
            def heartbeat():
                while not stop.wait(.001):
                    ticks.append(time.perf_counter())
            thread = threading.Thread(target=heartbeat)
            thread.start()
            intervals = []
            try:
                for _ in range(8):
                    time.sleep(.005)
                    start = time.perf_counter()
                    operation(packed)
                    end = time.perf_counter()
                    intervals.append((start, end))
            finally:
                stop.set()
                thread.join()
            # Discard 2 ms boundaries where a heartbeat might run after C return.
            within = [sum(start + .002 < tick < end - .002 for tick in ticks) for start, end in intervals]
            report['gil_progress'][name] = {'heartbeat_ticks_inside_each_forward': within,
                                           'total_inside_forward': sum(within)}
        native.close()
        report['outputs_survive_close'] = saved_bytes == [array.tobytes() for array in saved]
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        if native is not None:
            native.close()
        destination.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps({key: report[key] for key in ('bit_exact', 'detections_equal', 'timing', 'gil_progress', 'error') if key in report}))


if __name__ == '__main__':
    main()
