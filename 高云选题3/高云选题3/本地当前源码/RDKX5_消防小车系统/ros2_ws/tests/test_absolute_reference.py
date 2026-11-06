"""Single-turn references survive runtime count resets without physical writes."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.absolute_reference import build_absolute_reference, restore_absolute_reference
from rescue_vision.pd42_protocol import single_turn_counts


def config():
    return dict(feedback_timeout_s=.6, feedback_limit_tolerance_deg=.2,
        pan=dict(protocol='zdt_xs', reference_angle_deg=0., direction=1, gear_ratio=1.,
                 min_angle_deg=-90., max_angle_deg=90.),
        tilt=dict(protocol='pd42', reference_angle_deg=0., direction=1, gear_ratio=1.,
                  min_angle_deg=0., max_angle_deg=60.))


def samples(pan_absolute=62000, pan_position=3000, tilt_position=2000):
    return [dict(started=100.+i*.05,
        pan=dict(absolute_counts=pan_absolute, position_counts=pan_position, speed_rpm=0),
        tilt=dict(absolute_counts=tilt_position % 51200, position_counts=tilt_position, speed_rpm=0))
        for i in range(3)]


class AbsoluteReferenceTests(unittest.TestCase):
    def test_zdt_reboot_zero_coordinate_preserves_old_mechanical_origin(self):
        servo = config(); original = samples()
        anchor = build_absolute_reference(servo, original, 'fixed-zero-1', now=100.11)
        now = samples(pan_absolute=1000, pan_position=0)
        before = deepcopy((servo, anchor, now))
        restored = restore_absolute_reference(servo, now, anchor, now=100.11)
        self.assertEqual(restored['pan']['reference_counts'], -4536)
        self.assertAlmostEqual(restored['pan']['actual_angle_deg'], 4536*360/65536)
        self.assertEqual((servo, anchor, now), before)
        self.assertEqual(anchor['reference_id'], 'fixed-zero-1')

    def test_pd42_lost_multiturn_count_and_encoder_wrap_preserve_joint_angle(self):
        servo = config()
        anchor = build_absolute_reference(servo, samples(tilt_position=3*51200+51000), 'fixed', now=100.11)
        restored = restore_absolute_reference(servo, samples(tilt_position=1000), anchor, now=100.11)
        self.assertEqual(restored['tilt']['reference_counts'], -200)
        self.assertEqual(restored['tilt']['actual_angle_deg'], 1200*360/51200)

    def test_negative_direction_uses_saved_origin_not_shortest_positive_motion(self):
        servo = config(); servo['pan']['direction'] = servo['tilt']['direction'] = -1
        anchor = build_absolute_reference(servo, samples(1000, 200, 1000), 'fixed', now=100.11)
        restored = restore_absolute_reference(servo, samples(65000, 0, 500), anchor, now=100.11)
        self.assertEqual(restored['pan']['reference_counts'], 1536)
        self.assertAlmostEqual(restored['pan']['actual_angle_deg'], 1536*360/65536)
        self.assertEqual(restored['tilt']['reference_counts'], 1000)
        self.assertEqual(restored['tilt']['actual_angle_deg'], 500*360/51200)

    def test_direct_negative_pan_angle_and_positive_tilt_restored(self):
        servo = config(); anchor = build_absolute_reference(servo, samples(), 'fixed', now=100.11)
        restored = restore_absolute_reference(servo, samples(62000-16384, -2, 2000+6400), anchor, now=100.11)
        self.assertEqual(restored['pan']['actual_angle_deg'], -90.)
        self.assertEqual(restored['tilt']['actual_angle_deg'], 45.)

    def test_reduction_gearing_multiturn_ambiguity_is_rejected(self):
        servo = config(); servo['pan']['gear_ratio'] = 2
        with self.assertRaisesRegex(ValueError, 'multiturn_range_ambiguous'):
            build_absolute_reference(servo, samples(), 'fixed', now=100.11)

    def test_fractional_gearing_and_nonzero_mechanical_reference(self):
        servo = config()
        servo['pan'].update(gear_ratio=.5, reference_angle_deg=30.)
        servo['tilt']['reference_angle_deg'] = 30.
        anchor = build_absolute_reference(servo, samples(), 'fixed', now=100.11)
        restored = restore_absolute_reference(servo,
            samples((62000+4096) % 65536, 7, 5200), anchor, now=100.11)
        self.assertEqual(restored['pan']['actual_angle_deg'], 75.)
        self.assertEqual(restored['tilt']['actual_angle_deg'], 52.5)
        self.assertEqual(restored['pan']['reference_counts'], 7-4096)

    def test_runtime_reference_int32_overflow_is_rejected(self):
        servo = config(); anchor = build_absolute_reference(servo, samples(), 'fixed', now=100.11)
        for absolute, position in ((62001, -(2**31)), (61999, 2**31-1)):
            with self.subTest(position=position), self.assertRaisesRegex(ValueError, 'counts_overflow'):
                restore_absolute_reference(servo, samples(absolute, position), anchor, now=100.11)

    def test_tolerance_cannot_pick_between_two_valid_encoder_wraps(self):
        servo = config(); servo['pan'].update(min_angle_deg=-179.9, max_angle_deg=179.9)
        anchor = build_absolute_reference(servo, samples(0), 'fixed', now=100.11)
        with self.assertRaisesRegex(ValueError, 'wrap_ambiguous'):
            restore_absolute_reference(servo, samples(32768), anchor, now=100.11)

    def test_outside_limits_is_not_clamped_to_zero_or_limit(self):
        servo = config(); anchor = build_absolute_reference(servo, samples(), 'fixed', now=100.11)
        with self.assertRaisesRegex(ValueError, 'outside_limits'):
            restore_absolute_reference(servo, samples(tilt_position=2000-400), anchor, now=100.11)

    def test_missing_or_failed_encoder_reply_never_uses_position_as_zdt_absolute(self):
        servo = config(); anchor = build_absolute_reference(servo, samples(), 'fixed', now=100.11)
        for key, value in (('absolute_counts', None), ('absolute_counts', True),
                           ('absolute_counts', 65536), ('position_counts', None),
                           ('error', 'UART timeout')):
            rows = samples(); rows[-1]['pan'][key] = value
            with self.subTest(key=key, value=value), self.assertRaisesRegex(ValueError, 'feedback_missing'):
                restore_absolute_reference(servo, rows, anchor, now=100.11)

    def test_stale_unordered_and_moving_samples_do_not_restore(self):
        servo = config(); anchor = build_absolute_reference(servo, samples(), 'fixed', now=100.11)
        with self.assertRaisesRegex(ValueError, 'stale'):
            restore_absolute_reference(servo, samples(), anchor, now=101.)
        rows = samples(); rows[-1]['started'] = rows[0]['started']
        with self.assertRaisesRegex(ValueError, 'unordered'):
            restore_absolute_reference(servo, rows, anchor, now=100.11)
        rows = samples(); rows[1]['tilt']['speed_rpm'] = 1
        with self.assertRaisesRegex(ValueError, 'not_stationary'):
            restore_absolute_reference(servo, rows, anchor, now=100.11)

    def test_installation_change_invalidates_saved_reference(self):
        servo = config(); anchor = build_absolute_reference(servo, samples(), 'fixed', now=100.11)
        for changes in (dict(direction=-1), dict(gear_ratio=1.1), dict(max_angle_deg=80),
                        dict(reference_angle_deg=1)):
            changed = deepcopy(servo); changed['pan'].update(changes)
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, 'installation_changed'):
                restore_absolute_reference(changed, samples(), anchor, now=100.11)

    def test_pd42_single_turn_source_is_signed_modulo_and_rejects_missing_readback(self):
        self.assertEqual(single_turn_counts(-100), 51100)
        self.assertEqual(single_turn_counts(51200+1234), 1234)
        for invalid in (None, True, 1.5, 2**31, -(2**31)-1):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                single_turn_counts(invalid)
        rows = samples(); rows[-1]['tilt']['absolute_counts'] += 1
        with self.assertRaisesRegex(ValueError, 'source_mismatch'):
            build_absolute_reference(config(), rows, 'fixed', now=100.11)

    def test_software_rezero_creates_new_anchor_without_mutating_old(self):
        servo = config(); first = build_absolute_reference(servo, samples(), 'first', now=100.11)
        second = build_absolute_reference(servo, samples(1000), 'second', now=100.11)
        self.assertEqual(first['axes']['pan']['zero_absolute_counts'], 62000)
        self.assertEqual(second['axes']['pan']['zero_absolute_counts'], 1000)
        self.assertNotEqual(first['reference_id'], second['reference_id'])


if __name__ == '__main__': unittest.main()
