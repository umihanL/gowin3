"""Resampling must preserve speech rate and reject out-of-band alias energy."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.audio_pcm import PCM16To16k


class AudioPCMTests(unittest.TestCase):
    def test_16k_is_bit_exact(self):
        raw = np.arange(-1000, 1000, dtype='<i2').tobytes()
        self.assertEqual(PCM16To16k().convert(raw), raw)

    def test_stream_chunks_equal_whole_input_without_seams(self):
        raw = np.random.default_rng(12).integers(-20000, 20000, 48000, dtype=np.int16).tobytes()
        whole = PCM16To16k(48000).convert(raw)
        converter = PCM16To16k(48000)
        chunks = b''.join(converter.convert(raw[offset:offset + 2194]) for offset in range(0, len(raw), 2194))
        self.assertEqual(chunks, whole)
        self.assertEqual(len(chunks), 16000 * 2)

    def test_one_khz_preserved_twelve_khz_alias_rejected(self):
        t = np.arange(48000) / 48000
        levels = []
        for frequency in (1000, 12000):
            raw = np.rint(10000 * np.sin(2 * np.pi * frequency * t)).astype('<i2').tobytes()
            output = np.frombuffer(PCM16To16k(48000).convert(raw), dtype='<i2').astype(float)[500:]
            levels.append(np.sqrt(np.mean(output ** 2)))
        self.assertGreater(levels[0], 7000)
        self.assertLess(levels[1], 2)

    def test_invalid_rate_and_partial_sample_refused(self):
        for rate in (44100, True, 0, 16000.):
            with self.subTest(rate=rate), self.assertRaises(ValueError): PCM16To16k(rate)
        with self.assertRaises(ValueError): PCM16To16k(48000).convert(b'\x01')


if __name__ == '__main__':
    unittest.main()
