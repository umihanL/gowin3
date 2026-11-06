"""Streaming PCM16 anti-alias conversion for 48/32 kHz USB microphone input.

Vosk always receives actual 16 kHz mono PCM. Filter state and decimation phase
persist between chunks, avoiding false frequency/time scaling or chunk seams.
No devices, ROS or threads are opened by this module.
"""
class PCM16To16k:
    def __init__(self, capture_rate=16000):
        if type(capture_rate) is not int or capture_rate not in (16000, 32000, 48000):
            raise ValueError('capture_sample_rate_hz must be 16000, 32000 or 48000')
        self.capture_rate = capture_rate
        self.factor = capture_rate // 16000
        self.phase = 0
        self.delay_seconds = 0.
        if self.factor != 1:
            import numpy as np
            from scipy.signal import firwin
            # Preserve speech frequencies while removing content that would
            # alias above the 8 kHz output Nyquist limit. Kaiser FIR ~80 dB.
            self.coefficients = firwin(64 * self.factor + 1, 7200,
                                       fs=capture_rate, window=('kaiser', 8.6))
            self.state = np.zeros(len(self.coefficients) - 1)
            self.delay_seconds = (len(self.coefficients) - 1) / (2 * capture_rate)

    def convert(self, raw):
        if len(raw) % 2:
            raise ValueError('PCM16 input must contain whole samples')
        if self.factor == 1 or not raw:
            return bytes(raw)
        import numpy as np
        from scipy.signal import lfilter
        samples = np.frombuffer(raw, dtype='<i2').astype(np.float64)
        filtered, self.state = lfilter(self.coefficients, [1.], samples, zi=self.state)
        output = filtered[(-self.phase) % self.factor::self.factor]
        self.phase = (self.phase + len(samples)) % self.factor
        return np.clip(np.rint(output), -32768, 32767).astype('<i2').tobytes()
