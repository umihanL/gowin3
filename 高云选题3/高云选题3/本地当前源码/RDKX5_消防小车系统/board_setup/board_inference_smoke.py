"""Actual X5 inference checks using packaged data; never opens physical devices.

Run from the board's activated application environment. This exercises BPU,
CvBridge, CPU OCR and offline ASR. It does not launch ROS nodes or access camera,
UART, microphone, speaker, GPIO or actuators. No historical report is overwritten.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import importlib.metadata
import json
import platform
from pathlib import Path
import sys
import time
import traceback
import wave


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def package_version(name):
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def timestamp():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def output_metadata(tensors):
    records = []
    for tensor in tensors:
        properties = tensor.properties
        record = {'name': tensor.name}
        for key in ('shape', 'layout', 'dtype', 'tensor_type', 'aligned_shape',
                    'scale_data', 'scale_axis'):
            try:
                value = getattr(properties, key)
            except (AttributeError, TypeError):
                continue
            # Scalars, enum objects and short vectors are useful diagnostics.
            record[key] = str(value)[:1000]
        records.append(record)
    return records


def bpu_check(root, result):
    import numpy as np
    from rescue_vision.legacy_detector import (
        BoardDetector, nv12_tensor, letterbox, decode_dense, detections,
        OUTPUT_NAMES, read_image,
    )
    contract = json.loads((root/'models/yolo/model_contract.json').read_text(encoding='utf-8'))
    model = root/'models/yolo/wallpaper_yolo11n_3class_bayese_640_nv12.bin'
    result.update(backend='hobot_dnn.pyeasy_dnn', model_sha256=sha256(model),
                  candidate_quality_gate_passed=False, accuracy_validation=False)
    if result['model_sha256'] != contract['bin_sha256']:
        raise ValueError('Model differs from shipped model contract')
    if tuple(contract['output_names']) != tuple(OUTPUT_NAMES):
        raise ValueError('Runtime decoder and shipped output names differ')
    started = time.perf_counter()
    detector = BoardDetector(model, backend='pyeasy')
    result['load_seconds'] = time.perf_counter() - started
    result['runtime_inputs'] = output_metadata(detector.model.inputs)
    result['runtime_outputs'] = output_metadata(detector.model.outputs)

    def validate(outputs):
        records = []
        for name, array in zip(OUTPUT_NAMES, outputs):
            expected = tuple(contract['output_shapes'][name])
            if tuple(array.shape) != expected:
                raise ValueError(f'{name}: actual shape {array.shape}, expected {expected}')
            if not np.issubdtype(array.dtype, np.floating):
                raise ValueError(f'{name}: integer outputs require quantization metadata')
            if not np.isfinite(array).all():
                raise ValueError(f'{name}: non-finite output')
            records.append({'name': name, 'shape': list(array.shape),
                            'dtype': str(array.dtype), 'all_finite': True,
                            'minimum': float(array.min()), 'maximum': float(array.max())})
        dense = decode_dense(outputs)
        if dense.shape != (8400, 7) or not np.isfinite(dense).all():
            raise ValueError('Decoded dense output violates 8400x7 finite contract')
        return records

    synthetic = np.full((640, 640, 3), 114, dtype=np.uint8)
    packed = nv12_tensor(synthetic)
    if packed.size != 614400 or packed.dtype != np.uint8 or not packed.flags.c_contiguous:
        raise ValueError('Packed input violates full-range NV12 640x640 byte contract')
    result['input_contract'] = {'format': 'BT.601 full-range NV12',
        'packed_shape': list(packed.shape), 'dtype': str(packed.dtype),
        'bytes': packed.nbytes, 'contiguous': bool(packed.flags.c_contiguous)}
    started = time.perf_counter()
    outputs = detector.forward(packed)
    result['synthetic_warmup_forward_seconds'] = time.perf_counter() - started
    result['synthetic_warmup_outputs'] = validate(outputs)
    result['warmup_calls'] = 1

    image_path = root/'reports/recorded_scene_image.png'
    image = read_image(image_path)
    result['image'] = {'path': str(image_path.relative_to(root)),
        'sha256': sha256(image_path), 'size_wh': [image.shape[1], image.shape[0]],
        'kind': 'packaged synthetic map render, not real camera capture'}
    started = time.perf_counter()
    padded, meta = letterbox(image)
    packed_image = nv12_tensor(padded)
    result['image_preprocessing_seconds_once'] = time.perf_counter() - started
    result['measured_calls'] = []
    for index in range(3):
        started = time.perf_counter()
        outputs = detector.forward(packed_image)
        forward_seconds = time.perf_counter() - started
        output_info = validate(outputs)
        started = time.perf_counter()
        items = detections(outputs, meta, conf=.25, iou=.7)
        postprocess_seconds = time.perf_counter() - started
        result['measured_calls'].append({'index': index + 1,
            'forward_seconds': forward_seconds,
            'postprocess_seconds': postprocess_seconds,
            'output_metadata': output_info, 'detections': items})
    result['timing_scope'] = ('Three sequential BPU forward calls after one synthetic warmup; '
        'preprocessing measured once, postprocessing measured separately; contract validation '
        'excluded from timings. These are not live-camera or whole-system FPS measurements.')


def bridge_check(root, result):
    import numpy as np
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rescue_vision.legacy_detector import read_image
    image = read_image(root/'reports/recorded_scene_image.png')
    bridge = CvBridge()
    started = time.perf_counter()
    msg = bridge.cv2_to_imgmsg(image, 'bgr8')
    restored = bridge.imgmsg_to_cv2(msg, 'bgr8')
    result.update(numpy=np.__version__, opencv=cv2.__version__, rclpy_imported=True,
                  round_trip_seconds=time.perf_counter() - started,
                  actual_bgr8_ros_message_round_trip=True,
                  ros_nodes_started=False)
    if not np.array_equal(image, restored):
        raise ValueError('CvBridge image round trip changed pixel values')


def ocr_check(root, result):
    import onnxruntime
    from rescue_vision.legacy_detector import read_image
    from rescue_vision.building_ocr import OCRReader, BuildingMatcher
    image = read_image(root/'reports/recorded_scene_image.png')
    result.update(onnxruntime=onnxruntime.__version__,
        rapidocr_onnxruntime=package_version('rapidocr-onnxruntime'),
        available_providers=onnxruntime.get_available_providers(),
        cpu_requested=True, threads=2, real_image_accuracy_validation=False)
    started = time.perf_counter()
    reader = OCRReader(root/'models/ocr', threads=2)
    result['load_and_model_hash_check_seconds'] = time.perf_counter() - started
    started = time.perf_counter()
    readings = reader.predict(image)
    result['inference_seconds'] = time.perf_counter() - started
    result['readings'] = readings
    result['building_match'] = BuildingMatcher(root/'config/buildings.json').match(readings)
    result['source'] = 'packaged synthetic map render, not a real photograph'
    # OCR may read an unexpected sign: report the actual output, not a fabricated success.
    result['functional_nonempty_readings'] = bool(readings)
    if not readings:
        raise ValueError('OCR executed but returned no text for the packaged sign image')


def voice_check(root, result):
    import vosk
    from rescue_vision.voice import IntentParser, asr_text
    result.update(vosk=package_version('vosk'),
        source='Microsoft Huihui synthesized PCM, not physical microphone',
        speaker_or_microphone_opened=False)
    started = time.perf_counter()
    model = vosk.Model(str(root/'models/voice/vosk-model-small-cn-0.22'))
    result['load_seconds'] = time.perf_counter() - started
    buildings = json.loads((root/'config/buildings.json').read_text(encoding='utf-8'))
    parser = IntentParser(buildings['buildings'])
    expected = [('navigate', 'C'), ('report', None), ('stop', None),
                ('resume', None), ('inspect', None), (None, None)]
    result['cases'] = []
    for index, (action, building_id) in enumerate(expected):
        path = root/'reports/voice_audio_huihui'/f'command_{index:02d}_16k.wav'
        with wave.open(str(path), 'rb') as stream:
            if (stream.getnchannels(), stream.getsampwidth(), stream.getframerate()) != (1, 2, 16000):
                raise ValueError('ASR file must contain mono 16-bit 16000 Hz PCM')
            pcm = stream.readframes(stream.getnframes())
        recognizer = vosk.KaldiRecognizer(model, 16000)
        recognizer.SetWords(True)
        parts = []
        started = time.perf_counter()
        for offset in range(0, len(pcm), 8000):
            if recognizer.AcceptWaveform(pcm[offset:offset + 8000]):
                parts.append(json.loads(recognizer.Result()))
        parts.append(json.loads(recognizer.FinalResult()))
        elapsed = time.perf_counter() - started
        decoded = {'text': ' '.join(p.get('text', '') for p in parts),
                   'result': [word for part in parts for word in part.get('result', [])]}
        record = {'wav': str(path.relative_to(root)), 'sha256': sha256(path),
                  'pcm_seconds': len(pcm) / 32000, 'decode_seconds': elapsed,
                  'decoded': decoded, 'expected_action': action,
                  'expected_building_id': building_id}
        try:
            text, confidence = asr_text(decoded, .65)
            command = parser.parse(text, time.time(), 'voice')
            record.update(command=command, confidence=confidence,
                passed=command['action'] == action and command.get('building_id') == building_id)
        except ValueError as exc:
            record.update(rejected_reason=str(exc), passed=action is None)
        result['cases'].append(record)
    result['passed_cases'] = sum(case['passed'] for case in result['cases'])
    result['total_cases'] = len(result['cases'])
    if result['passed_cases'] != result['total_cases']:
        raise ValueError('At least one packaged synthetic speech intent did not match')


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--workspace', type=Path, default=Path('/root/rescue/ros2_ws'))
    cli.add_argument('--report', type=Path)
    args = cli.parse_args()
    root = args.workspace.resolve()
    if not (root/'src/rescue_vision/rescue_vision').is_dir():
        cli.error('workspace does not contain rescue_vision source')
    sys.path.insert(0, str(root/'src/rescue_vision'))
    report = args.report or root/'reports/board_20261010/inference.json'
    if report.exists():
        report = report.with_name(report.stem + '_' + dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + report.suffix)
    report.parent.mkdir(parents=True, exist_ok=True)
    data = {'started_utc': timestamp(), 'platform': platform.platform(),
        'machine': platform.machine(), 'python': sys.version,
        'python_executable': sys.executable, 'workspace': str(root),
        'script_sha256': sha256(__file__),
        'scope': 'Actual model computation only; no physical I/O or vehicle control.',
        'physical_camera_opened': False, 'physical_uart_opened': False,
        'microphone_opened': False, 'speaker_opened': False,
        'bpu_accuracy_validated': False, 'whole_system_realtime_validated': False,
        'checks': {}}
    def save():
        report.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    save()
    for name, check in [('bpu', bpu_check), ('cv_bridge', bridge_check),
                        ('ocr', ocr_check), ('vosk', voice_check)]:
        record = {'started_utc': timestamp(), 'passed': False}
        data['checks'][name] = record
        started = time.perf_counter()
        print(f'Starting {name}', flush=True)
        try:
            check(root, record)
            record['passed'] = True
        except Exception as exc:
            record.update(error=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc())
        record['total_seconds'] = time.perf_counter() - started
        record['finished_utc'] = timestamp()
        save()
        print(json.dumps({'check': name, 'passed': record['passed'],
                          'error': record.get('error'), 'seconds': record['total_seconds']}), flush=True)
    data['finished_utc'] = timestamp()
    data['passed'] = all(record['passed'] for record in data['checks'].values())
    save()
    print('Report:', report, flush=True)
    raise SystemExit(0 if data['passed'] else 1)


if __name__ == '__main__':
    main()
