#!/usr/bin/env python3
"""File-only ASR comparison through production adapters; no microphone or ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import unicodedata
import wave

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.asr_backend import load_asr_backend, SAMPLE_RATE, MAX_AUDIO_S


def normalized_text(value):
    value = unicodedata.normalize('NFKC', value).lower()
    return ''.join(char for char in value if not char.isspace() and not unicodedata.category(char).startswith(('P', 'Z')))


def character_error_rate(reference, hypothesis):
    expected, actual = normalized_text(reference), normalized_text(hypothesis)
    previous = list(range(len(actual)+1))
    for row, expected_char in enumerate(expected, 1):
        current = [row]
        for col, actual_char in enumerate(actual, 1):
            current.append(min(current[-1]+1, previous[col]+1, previous[col-1]+(expected_char != actual_char)))
        previous = current
    return dict(distance=previous[-1], reference_characters=len(expected), hypothesis_characters=len(actual),
        cer=previous[-1]/len(expected) if expected else None,
        normalized_reference=expected, normalized_hypothesis=actual,
        normalization='NFKC, lowercase, remove punctuation and whitespace; no number-word conversion')


def read_pcm(path):
    with wave.open(str(path), 'rb') as audio:
        if (audio.getcomptype() != 'NONE' or audio.getnchannels() != 1 or
                audio.getsampwidth() != 2 or audio.getframerate() != SAMPLE_RATE):
            raise ValueError('Expected uncompressed mono PCM16 WAV at 16000 Hz; convert explicitly before comparing')
        frames = audio.getnframes()
        if frames <= 0 or frames > SAMPLE_RATE*MAX_AUDIO_S:
            raise ValueError('WAV must contain 0 < duration <= 30 seconds; no silent truncation')
        raw = audio.readframes(frames)
        if len(raw) != frames*2:
            raise ValueError('WAV data is truncated')
    return raw


def run_backend(name, raw, workspace, config, threads, reference=None, *, loader=load_asr_backend):
    actual = dict(config, asr_backend=name, asr_fallback_backend=None, asr_threads=threads)
    started = time.perf_counter()
    backend = loader(actual, workspace)
    load_s = time.perf_counter()-started
    recognizer = backend.create_recognizer(SAMPLE_RATE)
    recognizer.SetWords(True)
    started = time.perf_counter()
    parts = []
    for offset in range(0, len(raw), 3200):
        if recognizer.AcceptWaveform(raw[offset:offset+3200]):
            parts.append(json.loads(recognizer.Result()))
    parts.append(json.loads(recognizer.FinalResult()))
    decode_s = time.perf_counter()-started
    text = ' '.join(part.get('text', '').strip() for part in parts if part.get('text', '').strip())
    duration = len(raw)/(SAMPLE_RATE*2)
    result = dict(backend=name, engine=backend.engine, load_s=round(load_s, 6),
        inference_s=round(decode_s, 6), audio_s=duration, real_time_factor=decode_s/duration,
        text=text, confidence_available=backend.confidence_available, segment_results=parts,
        fallback_enabled=False, audio_uploaded=False, command_published=False)
    if reference is not None:
        result['single_sample_cer'] = character_error_rate(reference, text)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=ROOT)
    parser.add_argument('--wav', required=True, type=Path)
    parser.add_argument('--reference', help='Exact spoken reference; single-sample CER only')
    parser.add_argument('--threads', type=int, choices=(1, 2), default=2)
    parser.add_argument('--backend', choices=('sensevoice', 'vosk', 'both'), default='both')
    parser.add_argument('--config', type=Path, help='Optional voice config for model paths; fallback always disabled')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--sample-kind', choices=('public', 'synthetic', 'real_recording', 'unknown'), default='unknown')
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text(encoding='utf-8-sig')) if args.config else {}
    raw = read_pcm(args.wav)
    result = dict(wav=str(args.wav.resolve()), pcm_sha256=hashlib.sha256(raw).hexdigest(),
        sample_kind=args.sample_kind, microphone_opened=False, ros_used=False,
        benchmark_note='One file tests software decoding only. Synthetic/public examples do not establish real microphone accuracy or safe robot command recognition.',
        threads=args.threads, results=[])
    failed = False
    for name in ('sensevoice', 'vosk') if args.backend == 'both' else (args.backend,):
        try:
            row = run_backend(name, raw, args.workspace.resolve(), config, args.threads, args.reference)
        except Exception as exc:
            failed = True
            row = dict(backend=name, error=type(exc).__name__, detail=str(exc), fallback_enabled=False,
                command_published=False, audio_uploaded=False)
        result['results'].append(row)
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding='utf-8')
    print(text, end='')
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
