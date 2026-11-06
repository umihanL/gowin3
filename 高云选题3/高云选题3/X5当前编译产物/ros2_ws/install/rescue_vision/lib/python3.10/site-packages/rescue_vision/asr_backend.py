"""Local ASR adapters; no ROS, microphone, network, downloads or command dispatch.

SenseVoice is an offline decoder: AcceptWaveform buffers actual mono PCM16 and
never claims a streaming endpoint. Its confidence is unavailable, not a guessed
number. The caller must route unsupported-confidence actions through independent command verification.
"""
import json
from pathlib import Path
import threading
import time


SAMPLE_RATE = 16000
MAX_AUDIO_S = 30


def _model_path(workspace, value, default):
    path = Path(value if value is not None else default).expanduser()
    return path if path.is_absolute() else Path(workspace) / path


def _threads(config):
    value = config.get('asr_threads', 2)
    if type(value) is not int or not 1 <= value <= 8:
        raise ValueError('asr_threads must be an integer in 1..8')
    return value


class SenseVoiceBackend:
    """Own the cached native model; each capture gets independent PCM storage."""
    asr_backend = 'sensevoice'
    engine = 'sensevoice-small-int8-sherpa-onnx'
    confidence_available = False
    fallback_from = None
    fallback_reason = None

    def __init__(self, model_dir, *, vad_model_path=None, num_threads=2,
                 native_recognizer=None, vad_factory=None):
        if type(num_threads) is not int or not 1 <= num_threads <= 8:
            raise ValueError('asr_threads must be an integer in 1..8')
        self.model_dir = Path(model_dir)
        self.num_threads = num_threads
        self.vad_model_path = Path(vad_model_path) if vad_model_path is not None else self.model_dir.parent / 'silero_vad.onnx'
        if vad_factory is None and not self.vad_model_path.is_file():
            raise FileNotFoundError('Mandatory Silero VAD model missing: ' + str(self.vad_model_path))
        if native_recognizer is None:
            model = self.model_dir / 'model.int8.onnx'
            tokens = self.model_dir / 'tokens.txt'
            if not model.is_file() or not tokens.is_file():
                raise FileNotFoundError('SenseVoice requires model.int8.onnx and tokens.txt in ' + str(self.model_dir))
            import sherpa_onnx
            native_recognizer = sherpa_onnx.OfflineRecognizer.from_sense_voice(
                model=str(model), tokens=str(tokens), language='zh', use_itn=True,
                num_threads=num_threads, provider='cpu')
        self.native_recognizer = native_recognizer
        self.decode_lock = threading.Lock()
        self.vad_factory = vad_factory or self._create_vad

    def _create_vad(self):
        import sherpa_onnx
        config = sherpa_onnx.VadModelConfig()
        config.silero_vad.model = str(self.vad_model_path)
        config.silero_vad.threshold = .5
        config.silero_vad.min_silence_duration = .5
        config.silero_vad.min_speech_duration = .2
        config.silero_vad.max_speech_duration = 30.
        config.silero_vad.window_size = 512
        config.sample_rate = SAMPLE_RATE
        config.num_threads = 1
        config.provider = 'cpu'
        return sherpa_onnx.VoiceActivityDetector(config, buffer_size_in_seconds=60)

    def create_recognizer(self, sample_rate=SAMPLE_RATE):
        return SenseVoiceRecognizer(self, sample_rate)


class SenseVoiceRecognizer:
    """Vosk-shaped session adapter around one bounded final offline decode."""
    def __init__(self, backend, sample_rate=SAMPLE_RATE):
        if type(sample_rate) is not int or sample_rate != SAMPLE_RATE:
            raise ValueError('SenseVoice adapter requires actual 16000 Hz mono PCM16')
        self.backend = backend
        self.sample_rate = sample_rate
        self._pcm = bytearray()
        self._result = None
        self._error = None
        self._finished = False
        self._lock = threading.Lock()

    def SetWords(self, enabled):
        # Compatibility only: SenseVoice does not expose Vosk word confidence.
        return None

    @property
    def pcm_bytes(self):
        """The exact full 16 kHz capture for an independent verifier, no gain/trim."""
        return bytes(self._pcm)

    def AcceptWaveform(self, raw):
        if not isinstance(raw, (bytes, bytearray, memoryview)):
            raise TypeError('ASR input must be PCM16 bytes')
        raw = bytes(raw)
        if len(raw) % 2:
            raise ValueError('PCM16 input must contain whole samples')
        with self._lock:
            if self._finished:
                raise RuntimeError('ASR capture already finalized')
            if len(self._pcm) + len(raw) > SAMPLE_RATE * MAX_AUDIO_S * 2:
                raise ValueError('ASR audio exceeds 30 seconds; refusing truncation')
            self._pcm.extend(raw)
        return False

    def _base_result(self, audio_s):
        return dict(text='', result=[], confidence=None, confidence_available=False,
            engine=self.backend.engine, asr_backend=self.backend.asr_backend,
            sample_rate_hz=SAMPLE_RATE, audio_s=audio_s, decode_s=0.,
            speech_verified=False, vad_speech_detected=False, speech_guard='silero_vad',
            speech_s=0., segments=[], decode_audio_s=0., vad_s=0.,
            audio_uploaded=False)

    def _speech_samples(self, samples):
        """Run mandatory neural VAD, then merge padded intervals without duplication."""
        import numpy as np
        vad = self.backend.vad_factory()
        intervals = []
        def drain():
            while not vad.empty():
                segment = vad.front
                start = max(0, min(len(samples), int(segment.start)))
                end = max(start, min(len(samples), int(segment.start) + len(segment.samples)))
                if end > start:
                    intervals.append((start, end))
                vad.pop()
        for start in range(0, len(samples), 512):
            chunk = samples[start:start+512]
            if len(chunk) < 512:
                chunk = np.pad(chunk, (0, 512-len(chunk)))
            vad.accept_waveform(chunk)
            drain()
        vad.flush()
        drain()
        def merge(values):
            merged = []
            for start, end in sorted(values):
                if merged and start <= merged[-1][1]:
                    merged[-1] = (merged[-1][0], max(end, merged[-1][1]))
                else:
                    merged.append((start, end))
            return merged
        speech = merge(intervals)
        segments = [dict(start_s=start/SAMPLE_RATE, end_s=end/SAMPLE_RATE) for start, end in speech]
        speech_s = sum(end-start for start, end in speech) / SAMPLE_RATE
        # Keep 150 ms around each speech interval so VAD boundaries do not cut
        # quiet initial/final phonemes. Preserve a short silence between gaps.
        padding = 2400
        padded = merge([(max(0, start-padding), min(len(samples), end+padding)) for start, end in speech])
        parts = []
        previous_end = None
        for start, end in padded:
            if previous_end is not None:
                parts.append(np.zeros(min(1600, start-previous_end), dtype=np.float32))
            parts.append(samples[start:end])
            previous_end = end
        trimmed = np.concatenate(parts) if parts else np.empty(0, dtype=np.float32)
        return trimmed, speech_s, segments

    def Result(self):
        # Offline decoding has no partial endpoint. Do not decode or publish a
        # provisional guess when a compatibility caller asks for Result().
        with self._lock:
            return self._result or json.dumps(self._base_result(len(self._pcm) / (SAMPLE_RATE * 2)), ensure_ascii=False)

    def FinalResult(self):
        with self._lock:
            if self._result is not None:
                return self._result
            if self._error is not None:
                raise self._error
            self._finished = True
            raw = bytes(self._pcm)
            result = self._base_result(len(raw) / (SAMPLE_RATE * 2))
            try:
                import numpy as np
                pcm = np.frombuffer(raw, dtype='<i2')
                if not pcm.size:
                    result['reason'] = 'no_audio'
                else:
                    samples = pcm.astype(np.float32) / np.float32(32768.)
                    vad_started = time.monotonic()
                    speech, speech_s, segments = self._speech_samples(samples)
                    result.update(vad_s=max(0., time.monotonic()-vad_started), speech_s=speech_s,
                        segments=segments, decode_audio_s=len(speech)/SAMPLE_RATE,
                        vad_speech_detected=bool(segments), speech_verified=bool(segments))
                    if not segments:
                        result['reason'] = 'no_speech'
                        self._result = json.dumps(result, ensure_ascii=False, allow_nan=False)
                        return self._result
                    started = time.monotonic()
                    with self.backend.decode_lock:
                        recognizer = self.backend.native_recognizer
                        stream = recognizer.create_stream()
                        stream.accept_waveform(SAMPLE_RATE, speech)
                        recognizer.decode_stream(stream)
                        text = stream.result.text
                    if not isinstance(text, str):
                        raise ValueError('SenseVoice returned a non-text result')
                    result.update(text=text.strip(), decode_s=max(0., time.monotonic() - started),
                                  reason='' if text.strip() else 'no_speech')
                self._result = json.dumps(result, ensure_ascii=False, allow_nan=False)
                return self._result
            except Exception as exc:
                # A compatibility retry of FinalResult must not start a second
                # native decode after failure or silently process missing PCM.
                self._error = exc
                raise


class VoskBackend:
    """Explicit legacy backend/fallback, retaining actual word confidence."""
    asr_backend = 'vosk'
    engine = 'vosk-small-cn-0.22'
    confidence_available = True
    fallback_from = None
    fallback_reason = None

    def __init__(self, model_dir):
        self.model_dir = Path(model_dir)
        if not (self.model_dir / 'am/final.mdl').is_file():
            raise FileNotFoundError('Vosk model missing: ' + str(self.model_dir))
        import vosk
        self._vosk = vosk
        self.model = vosk.Model(str(self.model_dir))

    def create_recognizer(self, sample_rate=SAMPLE_RATE):
        if type(sample_rate) is not int or sample_rate != SAMPLE_RATE:
            raise ValueError('Vosk Chinese backend requires actual 16000 Hz mono PCM16')
        return self._vosk.KaldiRecognizer(self.model, sample_rate)


def load_asr_backend(config, workspace):
    """Load once, cache at node level. Fallback requires explicit configuration."""
    name = config.get('asr_backend', 'vosk')
    fallback = config.get('asr_fallback_backend')
    if name not in ('sensevoice', 'vosk'):
        raise ValueError('asr_backend must be sensevoice or vosk')
    if fallback not in (None, '', 'vosk'):
        raise ValueError('Only an explicit vosk fallback is supported')
    threads = _threads(config)
    vosk_dir = _model_path(workspace, config.get('model_dir'), 'models/voice/vosk-model-small-cn-0.22')
    if name == 'vosk':
        return VoskBackend(vosk_dir)
    sensevoice_dir = _model_path(workspace, config.get('sensevoice_model_dir'),
                               'models/voice/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17')
    vad_path = _model_path(workspace, config.get('silero_vad_model_path'), 'models/voice/silero_vad.onnx')
    try:
        return SenseVoiceBackend(sensevoice_dir, vad_model_path=vad_path, num_threads=threads)
    except Exception as exc:
        if fallback != 'vosk':
            raise
        backend = VoskBackend(vosk_dir)
        backend.fallback_from = 'sensevoice'
        backend.fallback_reason = type(exc).__name__ + ': ' + str(exc)
        return backend
