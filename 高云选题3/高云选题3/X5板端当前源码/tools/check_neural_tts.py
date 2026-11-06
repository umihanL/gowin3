#!/usr/bin/env python3
"""Synthesize an explicit text locally, optionally play it. Never record or issue commands."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rescue_vision'))
from rescue_vision.voice_tts_process import ProcessNeuralTTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--text', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--play', action='store_true')
    parser.add_argument('--repeat', type=int, choices=(1, 2), default=1)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding='utf-8-sig'))
    result = dict(backend=config.get('tts_backend', 'sherpa_vits'), microphone_recorded=False, hardware_commands_sent=False,
                  audible_confirmed=None, runs=[])
    backend = ProcessNeuralTTS(config, ROOT)
    try:
        for index in range(args.repeat):
            started = time.monotonic()
            wav, metadata = backend.synthesize(args.text)
            run = dict(metadata, total_s=time.monotonic()-started, bytes=len(wav), run=index+1)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_bytes(wav)
            if args.play:
                command = ['aplay', '-q']
                if config.get('tts_audio_device'):
                    command += ['-D', str(config['tts_audio_device'])]
                p = subprocess.run(command + [str(args.output)], capture_output=True,
                                   text=True, timeout=max(10., metadata['duration_s'] + 5.))
                run['playback_returncode'] = p.returncode
                run['playback_stderr'] = p.stderr[-500:]
                if p.returncode:
                    raise RuntimeError(p.stderr)
            result['runs'].append(run)
        result['model'] = backend.metadata
        result['success'] = True
    finally:
        backend.close()
        args.output.with_suffix('.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
