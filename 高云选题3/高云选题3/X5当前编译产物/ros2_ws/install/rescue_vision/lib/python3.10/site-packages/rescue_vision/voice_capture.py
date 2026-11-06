"""Bounded streaming ASR decoupled from microphone deadlines (no ROS/audio I/O)."""
import json
import math
import queue
from threading import Event, Thread

from .voice import asr_text, normalize_voice_text, DIRECT_STOP_PHRASES


def combine_asr_results(results):
    """Keep final backend evidence, including empty finals and unavailable scores."""
    combined = {}
    for result in results:
        combined.update({key: value for key, value in result.items() if key not in ('text', 'result')})
    combined['text'] = ' '.join(r.get('text', '').strip() for r in results if r.get('text', '').strip())
    combined['result'] = [word for result in results for word in result.get('result', [])]
    for key in ('decode_s', 'audio_s', 'speech_s'):
        values = [r[key] for r in results if type(r.get(key)) in (float, int) and math.isfinite(r[key])]
        if values:
            combined[key] = sum(values)
    if any(r.get('confidence_available') is False for r in results):
        combined.update(confidence=None, confidence_available=False)
    return combined


def capture_duration(config, listening_mode):
    value = config.get('max_listen_seconds', 15.)
    if type(value) not in (float, int) or not math.isfinite(value):
        raise ValueError('invalid_max_listen_seconds')
    # PTT preserves a whole observation/question, including later corrections.
    return min(30., max(10., value)) if listening_mode == 'push_to_talk' else min(15., max(1., value))


class StreamingASR:
    """One decoder consumes bounded PCM; the capture owner alone controls the device."""
    def __init__(self, recognizer, listening_mode, minimum_confidence, max_blocks=320):
        self.recognizer = recognizer
        self.listening_mode = listening_mode
        self.minimum_confidence = minimum_confidence
        self.chunks = queue.Queue(maxsize=max_blocks)
        self.input_finished = Event()
        self.cancelled = Event()
        self.done = Event()
        self.endpoint = Event()
        self.started = Event()
        self.results = []
        self.emergency_result = None
        self.error = None
        self.thread = Thread(target=self._run, daemon=True)

    def start(self):
        if self.cancelled.is_set():
            self.done.set()
            return
        try:
            self.started.set()
            self.thread.start()
        except Exception:
            self.done.set()
            raise

    def submit(self, pcm):
        if pcm:
            self.chunks.put_nowait(pcm)

    def finish(self):
        self.input_finished.set()

    def cancel(self):
        self.cancelled.set()
        self.input_finished.set()
        # Native ASR can outlive a thread event. Production recognizers own an
        # isolated model worker and can terminate an unfinished call safely.
        # Completed sessions leave the cached model alive for the next capture.
        if not self.done.is_set():
            cancel_native = getattr(self.recognizer, 'Cancel', None)
            if cancel_native is not None:
                try:
                    cancel_native()
                except Exception as exc:
                    # Cleanup failure must not skip the microphone owner's
                    # finally block. An unfinished decoder still blocks reuse.
                    if self.error is None:
                        self.error = exc
                if self.started.is_set():
                    self.done.wait(1.)
        if not self.started.is_set():
            # A cancelled preparation has no native decoder in flight. Leaving
            # done unset here would permanently reserve the microphone owner.
            self.done.set()

    def combined(self):
        return combine_asr_results(self.results)

    def _endpoint_result(self, result):
        if not result.get('text', '').strip():
            return
        self.results.append(result)
        if self.listening_mode == 'continuous':
            self.endpoint.set()
            return
        if self.listening_mode != 'push_to_talk':
            return
        # Only an entire, confident, affirmative standalone stop can shorten
        # PTT. A prior phrase, qualifier, negation, or ordinary motion command
        # prevents this fast path. Full PTT still receives the shared parser.
        combined = self.combined()
        try:
            text, _ = asr_text(combined, self.minimum_confidence)
            if normalize_voice_text(text) in DIRECT_STOP_PHRASES:
                self.emergency_result = combined
                self.endpoint.set()
        except (ValueError, TypeError):
            pass

    def _run(self):
        try:
            while not self.cancelled.is_set():
                try:
                    pcm = self.chunks.get(timeout=.05)
                except queue.Empty:
                    if self.input_finished.is_set():
                        break
                    continue
                if self.recognizer.AcceptWaveform(pcm):
                    self._endpoint_result(json.loads(self.recognizer.Result()))
                    # A confident standalone emergency stop must not wait for
                    # queued trailing audio or FinalResult. Earlier endpoints
                    # were included in the exact whole-text check above.
                    if self.emergency_result is not None:
                        return
            if not self.cancelled.is_set():
                # Include trailing speech even after one or more endpoints.
                self.results.append(json.loads(self.recognizer.FinalResult()))
        except Exception as exc:
            self.error = exc
        finally:
            self.done.set()
