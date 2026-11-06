#!/usr/bin/env python3
"""Local audio diagnostics. Never imports ROS, publishes commands or enables voice.

Default: enumerate devices/config/model only. Recording, ASR and playback are
explicit CLI operations; captured audio remains in memory unless --save-wav is
given. Text recognized here is evidence, never an instruction to the robot.
"""
import argparse
from array import array
import datetime
import importlib.util
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import wave


RATE = 16000
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.audio_pcm import PCM16To16k


def command_result(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                errors='replace', timeout=5, check=False)
        return dict(command=command, returncode=result.returncode,
                    stdout=result.stdout.strip(), stderr=result.stderr.strip())
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(command=command, error=str(exc))


def dependency_present(name):
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def enumerate_devices():
    result = {'alsa': {}, 'sounddevice': [], 'sounddevice_available': False}
    for executable in ('arecord', 'aplay'):
        path = shutil.which(executable)
        result['alsa'][executable] = (command_result([path, '-l']) if path
                                      else {'error': 'command_not_installed'})
    cards = Path('/proc/asound/cards')
    if cards.is_file():
        result['alsa']['cards'] = cards.read_text(errors='replace').strip()
    try:
        import sounddevice as sd
        apis = sd.query_hostapis()
        result['sounddevice'] = [dict(index=index, name=device['name'],
            hostapi=apis[device['hostapi']]['name'],
            input_channels=device['max_input_channels'],
            output_channels=device['max_output_channels'],
            default_sample_rate=device['default_samplerate'])
            for index, device in enumerate(sd.query_devices())]
        result['sounddevice_available'] = True
    except Exception as exc:
        result['sounddevice_error'] = str(exc)
    return result


def select_input(devices, selector):
    """Resolve a device now, refusing ambiguous/default routing for microphone tests."""
    if selector is None or not str(selector).strip():
        raise ValueError('Specify --input-device using an unambiguous device name from inventory')
    requested = str(selector).strip()
    candidates = [device for device in devices if device['input_channels'] > 0]
    if requested.isdigit():
        matched = [device for device in candidates if device['index'] == int(requested)]
    else:
        matched = [device for device in candidates
                   if requested.casefold() in device['name'].casefold()]
    if len(matched) != 1:
        raise ValueError('Input selector must match exactly one capture device; matched: ' +
                         json.dumps(matched, ensure_ascii=False))
    return matched[0]


def stable_input_hint(device, devices):
    """USB model names often survive re-enumeration; ALSA hw card numbers do not."""
    fragment = re.sub(r'\s*\(hw:\d+,\d+\).*$', '', device['name']).strip()
    if fragment.casefold() in ('default', 'sysdefault', 'pulse', 'pipewire'):
        return None
    matches = [item for item in devices if item['input_channels'] > 0
               and fragment.casefold() in item['name'].casefold()]
    return fragment if len(matches) == 1 else None


def pcm_metrics(raw):
    if len(raw) % 2:
        raise ValueError('PCM16 must contain whole samples')
    samples = array('h')
    samples.frombytes(raw)
    if sys.byteorder != 'little':
        samples.byteswap()
    if not samples:
        return dict(samples=0, duration_seconds=0, rms_dbfs=None, peak_dbfs=None,
                    peak=0, clipped_fraction=0, zero_fraction=1, signal_present=False)
    count = len(samples)
    peak = max(abs(value) for value in samples)
    rms = math.sqrt(sum(value * value for value in samples) / count)
    return dict(samples=count, duration_seconds=count / RATE,
                rms_dbfs=round(20 * math.log10(rms / 32768), 2) if rms else None,
                peak_dbfs=round(20 * math.log10(peak / 32768), 2) if peak else None,
                peak=peak, clipped_fraction=sum(abs(value) >= 32760 for value in samples) / count,
                zero_fraction=sum(value == 0 for value in samples) / count,
                signal_present=peak > 0,
                note='Nonzero signal is not proof of intelligible speech; compare silence and speaking captures.')


def record_audio(index, seconds, capture_rate=RATE, start_delay=0.):
    import sounddevice as sd
    converter = PCM16To16k(capture_rate)
    sd.check_input_settings(device=index, channels=1, dtype='int16', samplerate=capture_rate)
    remaining = round(seconds * capture_rate)
    chunks = []
    overflow_count = 0
    print('READY: input format/filter prepared. Recording starts in %.1f seconds.' % start_delay,
          file=sys.stderr, flush=True)
    if start_delay:
        time.sleep(start_delay)
    with sd.RawInputStream(device=index, channels=1, dtype='int16',
                           samplerate=capture_rate, blocksize=capture_rate // 10) as stream:
        started_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
        start = time.monotonic()
        print('RECORDING NOW: %.1f seconds; started_utc=%s; no robot commands are sent.' % (seconds, started_utc),
              file=sys.stderr, flush=True)
        while remaining > 0:
            count = min(capture_rate // 10, remaining)
            chunk, overflowed = stream.read(count)
            chunks.append(converter.convert(bytes(chunk)))
            overflow_count += int(overflowed)
            remaining -= count
        elapsed = time.monotonic() - start
        ended_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
    print('RECORDING ENDED: ended_utc=%s; transcribing locally.' % ended_utc, file=sys.stderr, flush=True)
    return b''.join(chunks), overflow_count, dict(started_utc=started_utc, ended_utc=ended_utc,
                                                  capture_wall_seconds=elapsed, start_delay_seconds=start_delay)


def asr_check(raw, model_dir, threshold):
    import vosk
    if not (model_dir / 'am/final.mdl').is_file():
        raise ValueError('Vosk model missing: run tools/prepare_voice.py')
    vosk.SetLogLevel(-1)
    recognizer = vosk.KaldiRecognizer(vosk.Model(str(model_dir)), RATE)
    recognizer.SetWords(True)
    results = []
    for offset in range(0, len(raw), 3200):
        if recognizer.AcceptWaveform(raw[offset:offset + 3200]):
            results.append(json.loads(recognizer.Result()))
    results.append(json.loads(recognizer.FinalResult()))
    text = ' '.join(result.get('text', '').strip() for result in results).strip()
    words = [word for result in results for word in result.get('result', [])]
    confidence = [float(word.get('conf', 0)) for word in words]
    valid = bool(confidence) and all(math.isfinite(value) and 0 <= value <= 1 for value in confidence)
    mean = sum(confidence) / len(confidence) if valid else None
    accepted = bool(text and mean is not None and mean >= threshold)
    return dict(text=text, words=words, mean_confidence=mean, minimum_confidence=threshold,
                confidence_gate_passed=accepted, command_published=False,
                note='Only transcription and confidence are checked; command parsing is not invoked.')


def playback(text, config, device):
    executable = shutil.which(config.get('tts_executable', 'espeak-ng'))
    if not executable:
        raise ValueError('espeak-ng is not installed')
    command = [executable, '-v', config.get('tts_voice', 'cmn'),
               '-s', str(int(config.get('tts_words_per_minute', 145))), '--stdin']
    # Exactly the same direct espeak-ng backend as voice_node.py. ALSA aplay
    # card indices must not be blindly substituted for this backend's device.
    if device is not None:
        command[1:1] = ['-d', str(device)]
    result = subprocess.run(command, input=text, encoding='utf-8',
                            capture_output=True, timeout=30, check=False)
    return dict(command=command, returncode=result.returncode, stderr=result.stderr[-1200:],
                process_succeeded=result.returncode == 0, audible_confirmed=False,
                note='Process success does not prove the speaker was audible; confirm by listening.')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/voice.json')
    parser.add_argument('--input-device', help='Unique sounddevice name fragment, preferably USB model name; index allowed for one-off testing')
    parser.add_argument('--record-seconds', type=float, help='Explicitly capture 1–15 seconds of mono PCM16 at 16 kHz')
    parser.add_argument('--capture-rate', type=int, choices=(16000, 32000, 48000),
                        help='Actual hardware rate; 32/48 kHz are anti-alias filtered to 16 kHz for ASR')
    parser.add_argument('--start-delay', type=float, default=0.,
                        help='Wait 0–30 seconds AFTER printing READY and BEFORE opening microphone')
    parser.add_argument('--asr', action='store_true', help='Locally transcribe the explicit recording; never dispatch its text')
    parser.add_argument('--save-wav', type=Path, help='Opt-in recording file; existing files are not replaced')
    parser.add_argument('--play-text', help='Explicitly speak a short test phrase via the production espeak-ng backend')
    parser.add_argument('--tts-device', help='espeak-ng device override (not sounddevice or ALSA numeric index)')
    parser.add_argument('--report', type=Path, help='Write JSON inventory/results; contains recognized text if --asr is given')
    args = parser.parse_args(argv)
    if args.record_seconds is not None:
        if not math.isfinite(args.record_seconds) or not 1 <= args.record_seconds <= 15:
            parser.error('--record-seconds must be between 1 and 15')
    if (args.asr or args.save_wav) and args.record_seconds is None:
        parser.error('--asr and --save-wav require explicit --record-seconds')
    if not math.isfinite(args.start_delay) or not 0 <= args.start_delay <= 30:
        parser.error('--start-delay must be between 0 and 30 seconds')
    if args.start_delay and args.record_seconds is None:
        parser.error('--start-delay requires explicit --record-seconds')
    if args.play_text is not None and not 1 <= len(args.play_text.strip()) <= 120:
        parser.error('--play-text must contain 1–120 characters')
    if args.record_seconds is not None and args.play_text is not None:
        parser.error('Run recording and playback separately to avoid treating test speech as microphone evidence')
    if args.save_wav and args.save_wav.exists():
        parser.error('--save-wav target already exists; choose a new filename')
    return args


def main(argv=None):
    args = parse_args(argv)
    config = json.loads(args.config.read_text(encoding='utf-8-sig'))
    model_dir = Path(config.get('model_dir', 'models/voice/vosk-model-small-cn-0.22'))
    if not model_dir.is_absolute():
        model_dir = ROOT / model_dir
    report = dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        mode='diagnostic_only_no_ros', command_published=False, configuration_modified=False,
        config_path=str(args.config),
        configured={key: config.get(key) for key in ('microphone_enabled', 'tts_enabled',
            'listening_mode', 'audio_input_device', 'tts_audio_device', 'sample_rate_hz', 'capture_sample_rate_hz')},
        model=dict(path=str(model_dir), acoustic_model_present=(model_dir / 'am/final.mdl').is_file(),
                   vosk_installed=dependency_present('vosk')), inventory=enumerate_devices(),
        recording_requested=args.record_seconds is not None, playback_requested=args.play_text is not None)
    errors = []
    if args.record_seconds is not None:
        try:
            selector = args.input_device if args.input_device is not None else config.get('audio_input_device')
            selected = select_input(report['inventory']['sounddevice'], selector)
            report['selected_input'] = selected
            report['recommended_input_name'] = stable_input_hint(selected, report['inventory']['sounddevice'])
            capture_rate = args.capture_rate or config.get('capture_sample_rate_hz', RATE)
            raw, overflows, timing = record_audio(selected['index'], args.record_seconds, capture_rate, args.start_delay)
            report['capture'] = dict(**pcm_metrics(raw), **timing, overflow_count=overflows,
                                     usable_for_asr=overflows == 0, capture_sample_rate_hz=capture_rate,
                                     asr_sample_rate_hz=RATE, anti_alias_resampled=capture_rate != RATE)
            if args.save_wav:
                args.save_wav.parent.mkdir(parents=True, exist_ok=True)
                with args.save_wav.open('xb') as target, wave.open(target, 'wb') as output:
                    output.setnchannels(1)
                    output.setsampwidth(2)
                    output.setframerate(RATE)
                    output.writeframes(raw)
                report['saved_wav'] = str(args.save_wav.resolve())
            if overflows:
                errors.append('audio_overflow: do not accept this capture for ASR')
            elif args.asr:
                report['asr'] = asr_check(raw, model_dir, float(config.get('min_asr_confidence', .65)))
        except Exception as exc:
            errors.append('capture_or_asr_failed: ' + str(exc))
    if args.play_text is not None:
        try:
            report['playback'] = playback(args.play_text, config,
                args.tts_device if args.tts_device is not None else config.get('tts_audio_device'))
            if not report['playback']['process_succeeded']:
                errors.append('playback_process_failed')
        except Exception as exc:
            errors.append('playback_failed: ' + str(exc))
    report['errors'] = errors
    encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(encoded + '\n', encoding='utf-8')
    print(encoded)
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
