"""Cached offline neural TTS in a disposable process; no audio device or ROS access."""
import io
import math
import multiprocessing
import os
from pathlib import Path
import re
import threading
import time
import wave


MAX_SPOKEN_TEXT_CHARS = 900


def speech_chunks(text, max_chars=180):
    """Preserve a bounded full answer while keeping native synthesis pieces small."""
    if not isinstance(text, str) or not text.strip() or len(text) > MAX_SPOKEN_TEXT_CHARS:
        raise ValueError('speech_text_requires_1_to_900_characters')
    if type(max_chars) is not int or not 1 <= max_chars <= 350:
        raise ValueError('speech_chunk_limit_requires_1_to_350')
    chunks = []
    while len(text) > max_chars:
        boundary = max((text.rfind(mark, 0, max_chars) + 1
                        for mark in ('。', '！', '？', '!', '?', '；', ';', '\n')), default=0)
        end = boundary or max_chars
        chunks.append(text[:end])
        text = text[end:]
    if text:
        chunks.append(text)
    return chunks


def alsa_playback_status(device, proc_root='/proc/asound'):
    """Read-only presence check for an explicit ALSA hardware output.

    Do not open a device, substitute the microphone/onboard card, or claim
    arbitrary ALSA plugins are absent. An available PCM is not audible proof.
    """
    result = dict(tts_audio_device=device, tts_device_ready=None, reason='')
    if device is None:
        return result
    match = re.fullmatch(r'(?:plug)?hw:(?:CARD=)?([^,]+)(?:,(?:DEV=)?(\d+))?(?:,(?:SUBDEV=)?\d+)?', str(device))
    root = Path(proc_root)
    if match is None or not root.is_dir():
        return result
    card, pcm = match.group(1), match.group(2) or '0'
    for path in root.glob('card[0-9]*'):
        try:
            identity = (path / 'id').read_text().strip()
        except OSError:
            continue
        if identity != card and path.name != 'card' + card:
            continue
        if (path / ('pcm' + pcm + 'p') / 'info').is_file():
            return dict(result, tts_device_ready=True)
        return dict(result, tts_device_ready=False, reason='audio_output_pcm_missing')
    return dict(result, tts_device_ready=False, reason='audio_output_device_missing')


def audio_pcm16(samples, config):
    """Optional bounded peak normalization. Preserve raw level by default."""
    import numpy as np
    samples = np.asarray(samples, dtype=np.float32)
    if samples.size == 0 or not np.isfinite(samples).all():
        raise ValueError('Invalid neural speech waveform')
    target = float(config.get('tts_normalize_peak', 0.))
    maximum = float(config.get('tts_max_gain', 4.))
    if not math.isfinite(target) or not 0. <= target <= 1.:
        raise ValueError('tts_normalize_peak must be between 0 and 1')
    if not math.isfinite(maximum) or not 1. <= maximum <= 8.:
        raise ValueError('tts_max_gain must be between 1 and 8')
    peak = float(np.max(np.abs(samples)))
    # Do not boost silence or near-silent numerical residue (-80 dBFS).
    gain = min(maximum, target / peak) if target > 0. and peak > 1.e-4 else 1.
    output = np.clip(samples * gain, -1., 1.)
    return (output * 32767).astype('<i2').tobytes(), dict(
        audio_gain=gain, original_peak=peak, output_peak=float(np.max(np.abs(output))))


def model_paths(config, workspace):
    root = Path(config.get('tts_model_dir', 'models/voice/sherpa-onnx-vits-zh-ll'))
    if not root.is_absolute():
        root = Path(workspace) / root
    root = root.resolve()
    result = {}
    model_files = [('tokens', 'tokens.txt'), ('lexicon', 'lexicon.txt')]
    if config.get('tts_backend', 'sherpa_vits') == 'sherpa_matcha':
        model_files += [('acoustic_model', 'model-steps-3.onnx'), ('vocoder', 'vocos-22khz-univ.onnx')]
    else:
        model_files += [('model', 'model.onnx')]
    for key, default in model_files:
        name = config.get('tts_' + key, default)
        path = Path(name)
        path = path if path.is_absolute() else root / path
        if not path.is_file():
            raise FileNotFoundError('Missing TTS ' + key + ': ' + str(path))
        result[key] = str(path)
    rules = config.get('tts_rule_fsts', ['phone.fst', 'date.fst', 'number.fst'])
    if not isinstance(rules, list) or not all(isinstance(v, str) for v in rules):
        raise ValueError('tts_rule_fsts must be a list of file names')
    paths = [Path(v) if Path(v).is_absolute() else root / v for v in rules]
    if any(not p.is_file() for p in paths):
        raise FileNotFoundError('Missing TTS number/date normalization rule')
    result['rule_fsts'] = ','.join(str(p) for p in paths)
    return result


def _neural_worker(connection, config, workspace):
    # The native inference can be terminated without wedging the voice node.
    # Playback stays in the parent, so terminating synthesis cannot orphan aplay.
    try:
        if hasattr(os, 'nice'):
            os.nice(5)
        import numpy as np
        import sherpa_onnx
        paths = model_paths(config, workspace)
        threads = max(1, min(2, int(config.get('tts_threads', 2))))
        if config.get('tts_backend', 'sherpa_vits') == 'sherpa_matcha':
            network = {'matcha': sherpa_onnx.OfflineTtsMatchaModelConfig(
                acoustic_model=paths['acoustic_model'], vocoder=paths['vocoder'],
                tokens=paths['tokens'], lexicon=paths['lexicon'])}
        else:
            network = {'vits': sherpa_onnx.OfflineTtsVitsModelConfig(
                model=paths['model'], tokens=paths['tokens'], lexicon=paths['lexicon'])}
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(**network,
                num_threads=threads, provider='cpu', debug=False),
            rule_fsts=paths['rule_fsts'], max_num_sentences=1)
        if not tts_config.validate():
            raise ValueError('Invalid offline neural TTS configuration')
        tts = sherpa_onnx.OfflineTts(tts_config)
        sid = int(config.get('tts_speaker_id', 0))
        speed = float(config.get('tts_speed', 1.0))
        if not 0 <= sid < tts.num_speakers or not math.isfinite(speed) or not .7 <= speed <= 1.4:
            raise ValueError('Invalid TTS speaker ID or speed')
        connection.send(('ready', dict(sample_rate=tts.sample_rate, speakers=tts.num_speakers,
                                       speaker_id=sid, threads=threads)))
        while True:
            request, text = connection.recv()
            started = time.monotonic()
            audio = tts.generate(text, sid=sid, speed=speed)
            samples = np.asarray(audio.samples, dtype=np.float32)
            rate = int(audio.sample_rate)
            if rate <= 0 or samples.size == 0 or samples.size > rate * 120 or not np.isfinite(samples).all():
                raise RuntimeError('Invalid or excessively long TTS audio')
            pcm, volume = audio_pcm16(samples, config)
            wav = io.BytesIO()
            with wave.open(wav, 'wb') as output:
                output.setnchannels(1); output.setsampwidth(2); output.setframerate(rate)
                output.writeframes(pcm)
            connection.send(('audio', request, wav.getvalue(),
                             dict(synthesis_s=time.monotonic()-started, duration_s=samples.size/rate,
                                  sample_rate=rate, speaker_id=sid, **volume)))
    except (EOFError, BrokenPipeError):
        pass
    except BaseException as exc:
        try:
            connection.send(('error', type(exc).__name__ + ': ' + str(exc)))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        connection.close()


class ProcessNeuralTTS:
    """One sequential synthesizer. Timeout reclaims native code; next use reloads."""
    def __init__(self, config, workspace, worker_target=None):
        self.config = dict(config)
        self.workspace = str(workspace)
        self._worker_target = worker_target or _neural_worker
        self._ctx = multiprocessing.get_context('spawn')
        self._lifecycle = threading.RLock()
        self._operation = threading.Lock()
        self._process = None
        self._connection = None
        self._closed = threading.Event()
        self._request = 0
        self.metadata = {}

    def _drop(self):
        with self._lifecycle:
            process, connection = self._process, self._connection
            self._process = self._connection = None
            if process is not None:
                if process.is_alive():
                    process.terminate()
                process.join(timeout=.5)
                if process.is_alive():
                    process.kill(); process.join(timeout=.5)
            if connection is not None:
                connection.close()

    def close(self):
        self._closed.set()
        self._drop()

    def _receive(self, deadline):
        while not self._closed.is_set():
            connection, process = self._connection, self._process
            if connection is None or process is None:
                raise RuntimeError('TTS worker cancelled')
            if connection.poll(.05):
                message = connection.recv()
                if message[0] == 'error':
                    raise RuntimeError(message[1])
                return message
            if not process.is_alive():
                raise RuntimeError('TTS worker exited')
            if time.monotonic() >= deadline:
                raise TimeoutError('Neural TTS synthesis timed out')
        raise RuntimeError('TTS closed')

    def _prepare(self, deadline):
        if self._process is None:
            with self._lifecycle:
                if self._closed.is_set():
                    raise RuntimeError('TTS closed')
                parent, child = self._ctx.Pipe()
                self._connection = parent
                self._process = self._ctx.Process(target=self._worker_target,
                    args=(child, self.config, self.workspace), daemon=True)
                self._process.start(); child.close()
            ready = self._receive(deadline)
            if ready[0] != 'ready':
                raise RuntimeError('Invalid TTS startup response')
            self.metadata = ready[1]

    def prepare(self):
        """Load/cache the model only; never synthesize, access audio or issue commands."""
        with self._operation:
            if self._closed.is_set():
                raise RuntimeError('TTS closed')
            budget = max(1., min(90., float(self.config.get('tts_synthesis_timeout_s', 45.))))
            try:
                self._prepare(time.monotonic() + budget)
                return dict(self.metadata)
            except BaseException:
                self._drop()
                raise

    def synthesize(self, text):
        if not isinstance(text, str) or not text.strip() or len(text) > 350:
            raise ValueError('TTS expects 1 to 350 text characters')
        with self._operation:
            if self._closed.is_set():
                raise RuntimeError('TTS closed')
            budget = max(1., min(90., float(self.config.get('tts_synthesis_timeout_s', 45.))))
            deadline = time.monotonic() + budget
            try:
                self._prepare(deadline)
                self._request += 1
                self._connection.send((self._request, text))
                audio = self._receive(deadline)
                if audio[0] != 'audio' or audio[1] != self._request:
                    raise RuntimeError('Stale or invalid TTS audio response')
                return audio[2], audio[3]
            except BaseException:
                self._drop()
                raise
