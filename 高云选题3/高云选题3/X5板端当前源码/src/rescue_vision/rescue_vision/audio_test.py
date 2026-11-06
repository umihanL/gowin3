"""Pure PCM measurements and request validation for non-command microphone tests."""
from collections import OrderedDict
import math
import re
from .voice_capture import combine_asr_results


class AudioTestRequestGate:
    def __init__(self):
        self.seen = OrderedDict()

    def accept(self, request, now):
        if not isinstance(request, dict) or type(request.get('enabled')) is not bool:
            raise ValueError('invalid_audio_test_request')
        identifier = request.get('command_id')
        if not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z0-9_:.-]{1,100}', identifier):
            raise ValueError('invalid_command_id')
        stamp, ttl = request.get('issued_at'), request.get('ttl_s')
        finite = lambda v: type(v) in (int, float) and math.isfinite(v)
        if (not finite(now) or not finite(stamp) or not finite(ttl) or not 0 < ttl <= 10
                or not 0 <= now - stamp <= ttl):
            raise ValueError('expired_or_future_audio_test_request')
        if identifier in self.seen:
            raise ValueError('duplicate_audio_test_request')
        self.seen[identifier] = now
        while len(self.seen) > 512:
            self.seen.popitem(last=False)
        return request['enabled'], identifier


class PCMLevelMeter:
    """RMS/peak from actual little-endian signed PCM16, without synthetic levels."""
    def __init__(self):
        self.count = 0
        self.sum_squares = 0.
        self.peak = 0
        self.clipped = 0

    def add(self, raw):
        if len(raw) % 2:
            raise ValueError('PCM16 input must contain whole samples')
        if not raw:
            return
        import numpy as np
        samples = np.frombuffer(raw, dtype='<i2').astype(np.float64)
        absolute = np.abs(samples)
        self.count += samples.size
        self.sum_squares += float(np.dot(samples, samples))
        self.peak = max(self.peak, int(absolute.max()))
        self.clipped += int(np.count_nonzero(absolute >= 32760))

    def snapshot(self, reset=False):
        def dbfs(value):
            return round(max(-96., 20 * math.log10(value / 32768)), 2) if value else -96.
        result = dict(level_dbfs=None, peak_dbfs=None, clipped_fraction=None, level_samples=self.count)
        if self.count:
            result.update(level_dbfs=dbfs(math.sqrt(self.sum_squares / self.count)),
                          peak_dbfs=dbfs(self.peak), clipped_fraction=self.clipped / self.count)
        if reset:
            self.__init__()
        return result


def diagnostic_transcript(results, minimum_confidence):
    """Expose uncertain transcripts as evidence, never as an executable command."""
    if (type(minimum_confidence) not in (int, float) or not math.isfinite(minimum_confidence)
            or not 0 <= minimum_confidence <= 1):
        raise ValueError('invalid_minimum_asr_confidence')
    combined = combine_asr_results(results)
    text = combined['text']
    metadata = {key: combined[key] for key in ('engine','asr_backend','confidence_available',
        'decode_s','audio_s','speech_s','vad_s','decode_audio_s','segments','speech_guard','speech_verified','vad_speech_detected')
        if key in combined}
    if combined.get('confidence_available') is False:
        verified = (combined.get('speech_verified') is True and combined.get('vad_speech_detected') is True
                    and combined.get('speech_guard') == 'silero_vad')
        return dict(metadata, text=text[:1000], confidence=None, confidence_gate_passed=False,
            transcription_accepted=bool(text and verified), minimum_confidence=minimum_confidence,
            reason='' if text and verified else 'no_speech' if not text else 'voice_speech_not_verified')
    words = [word for result in results for word in result.get('result', [])]
    values = [word.get('conf') for word in words]
    valid = bool(values) and all(type(value) in (int, float) and math.isfinite(value)
                                 and 0 <= value <= 1 for value in values)
    confidence = sum(values) / len(values) if valid else None
    passed = bool(text and confidence is not None and confidence >= minimum_confidence)
    reason = ('' if passed else 'no_speech' if not text else
              'invalid_asr_confidence' if not valid else 'low_asr_confidence')
    return dict(metadata, text=text[:1000], confidence=confidence, confidence_gate_passed=passed,
                transcription_accepted=passed,
                minimum_confidence=minimum_confidence, reason=reason)
