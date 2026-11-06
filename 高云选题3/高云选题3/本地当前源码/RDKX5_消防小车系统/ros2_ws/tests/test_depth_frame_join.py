"""Real timestamp identities, bounded retention, and ambiguity rejection."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.depth_frame_join import DepthFrameJoin


class JoinTests(unittest.TestCase):
    def setUp(self):
        self.join = DepthFrameJoin()
        self.now = 1_700_000_000_000_000_000

    def test_exact_left_identity_not_latest_left(self):
        self.join.push('left', self.now, 'wanted', self.now)
        self.join.push('left', self.now + 33_000_000, 'different', self.now + 33_000_000)
        self.join.push('right', self.now + 1_000_000, 'right', self.now + 33_000_000)
        pair, reason = self.join.match(self.now, self.now + 60_000_000)
        self.assertEqual(pair, ('wanted', 'right', .001))
        self.assertEqual(reason, 'ok')
        self.assertEqual(self.join.match(self.now + 1, self.now + 60_000_000)[1], 'exact_left_frame_missing')

    def test_missing_skewed_and_ambiguous_right_rejected(self):
        self.join.push('left', self.now, 'left', self.now)
        self.assertEqual(self.join.match(self.now, self.now)[1], 'matching_right_frame_missing')
        self.join.push('right', self.now + 13_000_000, 'late', self.now + 13_000_000)
        self.assertEqual(self.join.match(self.now, self.now + 13_000_000)[1], 'matching_right_frame_missing')
        other = DepthFrameJoin()
        other.push('left', self.now, 'left', self.now)
        other.push('right', self.now, 'r1', self.now)
        other.push('right', self.now + 1_000_000, 'r2', self.now + 1_000_000)
        self.assertEqual(other.match(self.now, self.now + 1_000_000)[1], 'ambiguous_right_frame')

    def test_stale_and_future_expire_never_reused(self):
        self.join.push('left', self.now, 'left', self.now)
        self.join.push('right', self.now, 'right', self.now)
        self.assertEqual(self.join.match(self.now, self.now + 500_000_001)[1], 'stale_detection')
        self.assertTrue(all(not q for q in self.join.frames.values()))
        self.assertFalse(self.join.push('left', self.now + 100, 'future', self.now))

    def test_duplicates_out_of_order_and_capacity(self):
        join = DepthFrameJoin(capacity=2)
        for i in range(4):
            self.assertTrue(join.push('left', self.now + i, i, self.now + 10))
        self.assertEqual(list(join.frames['left'].values()), [2, 3])
        self.assertFalse(join.push('left', self.now + 3, 'duplicate', self.now + 10))
        self.assertFalse(join.push('left', self.now + 2, 'old', self.now + 10))

    def test_bad_parameters(self):
        for kwargs in [dict(max_age_s=.6), dict(max_skew_s=.1), dict(capacity=True), dict(capacity=33)]:
            with self.assertRaises(ValueError):
                DepthFrameJoin(**kwargs)


if __name__ == '__main__':
    unittest.main()
