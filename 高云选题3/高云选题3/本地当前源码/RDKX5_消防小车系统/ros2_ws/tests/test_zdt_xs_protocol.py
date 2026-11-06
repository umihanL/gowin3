"""Vendor vectors and unit distinctions; no imports open a serial port."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.zdt_xs_protocol import (
    Command, ProtocolError, DeviceError, read_request, enable_request,
    stop_request, position_request, velocity_request, position_degrees, error_degrees, speed_rpm,
    parse_response, normalize_options, verified_motion_parameters, validate_request,
    select_firmware_request, zero_current_position_request, READ_COMMANDS, CONTROL_COMMANDS,
)


# Exact read-config examples on manual pp102 and106.
X_CONFIG = bytes.fromhex('01 42 25 18 00 01 01 02 02 00 10 01 00 00 04 B0 0B 80 0B B8 03 E8 05 07 00 01 00 01 00 08 08 98 07 D0 00 08 6B')
EMM_CONFIG = bytes.fromhex('01 42 21 15 19 02 02 02 00 10 01 00 04 B0 0B 80 0F A0 05 07 01 00 01 01 00 08 08 98 07 D0 00 08 6B')
# Independent read-only capture from the connected second motor, 2026-10-27.
LIVE_VERSION = bytes.fromhex('01 1F 00 CE 23 14 6B')
LIVE_OPTIONS = bytes.fromhex('01 1A 00 06 6B')
LIVE_CONFIG = bytes.fromhex('01 42 21 15 19 02 02 02 00 10 01 00 04 B0 0B B8 13 88 05 07 01 00 01 01 00 08 08 98 07 D0 00 08 6B')


class ZDTProtocolTests(unittest.TestCase):
    def test_single_turn_absolute_encoder_read_is_unsigned_and_read_only(self):
        self.assertEqual(read_request(1, Command.READ_ENCODER), bytes.fromhex('01 31 6b'))
        self.assertEqual(validate_request(bytes.fromhex('01 31 6b')), (1, Command.READ_ENCODER))
        self.assertIn(Command.READ_ENCODER, READ_COMMANDS)
        self.assertNotIn(Command.READ_ENCODER, CONTROL_COMMANDS)
        for encoded, value in (('00 00', 0), ('12 34', 4660), ('ff ff', 65535), ('e2 6b', 57963)):
            with self.subTest(value=value):
                self.assertEqual(parse_response(bytes.fromhex('01 31 '+encoded+' 6b')).value, value)
        with self.assertRaises(DeviceError):
            parse_response(bytes.fromhex('01 31 e2 6b')).require_success()
        for malformed in ('01 31 00 6b', '01 31 00 01 02 6b', '01 31 ff ff 00'):
            with self.subTest(malformed=malformed), self.assertRaises(ProtocolError):
                parse_response(bytes.fromhex(malformed))

    def test_vendor_current_position_zero_is_explicit_control_not_a_read(self):
        packet = zero_current_position_request(1)
        self.assertEqual(packet.hex(' '), '01 0a 6d 6b')
        self.assertEqual(validate_request(packet), (1, Command.RESET_CURRENT_POSITION))
        self.assertIn(Command.RESET_CURRENT_POSITION, CONTROL_COMMANDS)
        self.assertNotIn(Command.RESET_CURRENT_POSITION, READ_COMMANDS)
        with self.assertRaises(ProtocolError): read_request(1, Command.RESET_CURRENT_POSITION)
        self.assertTrue(parse_response(bytes.fromhex('01 0a 02 6b')).ok)
        with self.assertRaises(DeviceError):
            parse_response(bytes.fromhex('01 0a e2 6b')).require_success()
        for frame in ('00 0a 6d 6b', '01 0a 6b', '01 0a 6c 6b', '01 0a 6d 00 6b'):
            with self.subTest(frame=frame), self.assertRaises(ProtocolError):
                validate_request(bytes.fromhex(frame))

    def test_vendor_firmware_selection_vectors_are_explicit_control_not_reads(self):
        for mode, code in (('x', 0), ('emm', 1), ('emm_turbo', 2)):
            with self.subTest(mode=mode):
                packet = select_firmware_request(1, mode)
                self.assertEqual(packet, bytes((1, 0xD5, 0x69, 0, code, 0x6B)))
                self.assertEqual(validate_request(packet), (1, Command.SELECT_FIRMWARE))
                self.assertEqual(select_firmware_request(1, mode, persist=True),
                                 bytes((1, 0xD5, 0x69, 1, code, 0x6B)))
        self.assertNotIn(Command.SELECT_FIRMWARE, READ_COMMANDS)
        self.assertIn(Command.SELECT_FIRMWARE, CONTROL_COMMANDS)
        with self.assertRaises(ProtocolError): read_request(1, Command.SELECT_FIRMWARE)
        self.assertTrue(parse_response(bytes.fromhex('01 d5 02 6b')).ok)
        for code in (0xE2, 0xEE):
            with self.assertRaises(DeviceError):
                parse_response(bytes((1, 0xD5, code, 0x6B))).require_success()
        with self.assertRaises(ProtocolError): parse_response(bytes.fromhex('01 d5 9f 6b'))

    def test_firmware_selection_rejects_ambiguous_arguments_and_malformed_frames(self):
        for mode in (None, 0, 1, True, 'Emm', 'turbo', '', [], {}):
            with self.subTest(mode=mode), self.assertRaises(ProtocolError):
                select_firmware_request(1, mode)
        for persist in (None, 0, 1, 'false', [], {}):
            with self.subTest(persist=persist), self.assertRaises(ProtocolError):
                select_firmware_request(1, 'emm', persist=persist)
        for frame in ('01 d5 6b', '01 d5 69 00 6b', '01 d5 68 00 01 6b',
                      '01 d5 69 02 01 6b', '01 d5 69 00 03 6b',
                      '01 d5 69 00 01 00 6b', '00 d5 69 00 01 6b'):
            with self.subTest(frame=frame), self.assertRaises(ProtocolError):
                validate_request(bytes.fromhex(frame))

    def test_vendor_emm_velocity_example_and_feedback_units(self):
        self.assertEqual(velocity_request(1,-1500,mode='emm',speed_div10=False).hex(' '),
                         '01 f6 01 05 dc 0a 00 6b')
        self.assertEqual(velocity_request(1,5,mode='emm',speed_div10=True).hex(' '),
                         '01 f6 00 00 32 0a 00 6b')
        self.assertTrue(parse_response(bytes.fromhex('01 f6 02 6b')).ok)

    def test_x_velocity_is_nine_bytes_acceleration_before_speed(self):
        frame=velocity_request(1,-5,mode='x',acceleration=10)
        self.assertEqual(frame.hex(' '),'01 f6 01 00 0a 00 32 00 6b')
        self.assertEqual(validate_request(frame),(1,Command.VELOCITY))

    def test_velocity_requires_known_mode_scale_and_canonical_layout(self):
        for kw in [dict(mode=None),dict(mode='emm'),dict(mode='emm',speed_div10=1)]:
            with self.assertRaises(ProtocolError):velocity_request(1,5,**kw)
        for rpm in [0,True,float('inf'),3001,.01]:
            with self.assertRaises(ProtocolError):velocity_request(1,rpm,mode='x')
        for frame in ['01 f6 00 00 05 0a 01 6b','01 f6 00 00 00 0a 00 6b',
                      '01 f6 00 00 05 6b','01 f6 02 00 05 0a 00 6b']:
            with self.assertRaises(ProtocolError):validate_request(bytes.fromhex(frame))

    def test_vendor_read_enable_stop_requests(self):
        self.assertEqual(read_request(1, Command.READ_VERSION).hex(' '), '01 1f 6b')
        self.assertEqual(read_request(1, Command.READ_CONFIG).hex(' '), '01 42 6c 6b')
        self.assertEqual(enable_request(1, True).hex(' '), '01 f3 ab 01 00 6b')
        self.assertEqual(enable_request(1, False).hex(' '), '01 f3 ab 00 00 6b')
        self.assertEqual(stop_request(1).hex(' '), '01 fe 98 00 6b')

    def test_vendor_emm_ten_revolution_vector(self):
        packet = position_request(1, -3600, 1500, mode='emm', motion_mode=0,
                                  microsteps=16, motor_step_deg=1.8,
                                  speed_div10=False, acceleration=0)
        self.assertEqual(packet.hex(' '), '01 fd 01 05 dc 00 00 00 7d 00 00 00 6b')
        self.assertEqual(validate_request(packet), (1, Command.POSITION_EMM))

    def test_emm_five_degrees_uses_pulses_not_feedback_counts(self):
        packet = position_request(1, 5, 5, mode='emm', motion_mode=1,
                                  microsteps=16, motor_step_deg=1.8,
                                  speed_div10=False, acceleration=10)
        self.assertEqual(packet.hex(' '), '01 fd 00 00 05 0a 00 00 00 2c 01 00 6b')

    def test_emm_half_degree_motor_and_speed_scale(self):
        packet = position_request(1, 9, 5, mode='emm', microsteps=16,
                                  motor_step_deg=.9, speed_div10=True)
        self.assertEqual(int.from_bytes(packet[3:5], 'big'), 50)
        self.assertEqual(int.from_bytes(packet[6:10], 'big'), 160)
        self.assertEqual(packet[-3], 2)

    def test_x_angle_scaling_does_not_change_rpm(self):
        for scaled, expected in [(False, 50), (True, 500)]:
            packet = position_request(1, -5, 5, mode='x', motion_mode=1, angle_div10=scaled)
            self.assertEqual(packet[:5], bytes.fromhex('01 fb 01 00 32'))
            self.assertEqual(int.from_bytes(packet[5:9], 'big'), expected)
            self.assertEqual(packet[-3:], bytes.fromhex('01 00 6b'))
            validate_request(packet)

    def test_unknown_or_missing_units_never_encode_motion(self):
        cases = [dict(mode=None), dict(mode='unknown'), dict(mode='emm'),
                 dict(mode='emm', microsteps=16, motor_step_deg=1.8),
                 dict(mode='x'), dict(mode='x', angle_div10=1),
                 dict(mode='emm', microsteps=0, motor_step_deg=1.8, speed_div10=False)]
        for kwargs in cases:
            with self.subTest(kwargs=kwargs), self.assertRaises(ProtocolError):
                position_request(1, 5, 5, **kwargs)

    def test_numeric_boundaries_reject_nan_bool_overflow(self):
        for angle, rpm in [(float('nan'), 5), (float('inf'), 5), (True, 5),
                           (5, 0), (5, True), (5, -1), (5, 3001), (1e15, 5)]:
            with self.subTest(angle=angle, rpm=rpm), self.assertRaises(ProtocolError):
                position_request(1, angle, rpm, mode='x', angle_div10=False)
        with self.assertRaises(ProtocolError):
            position_request(1, 5, .01, mode='x', angle_div10=False)

    def test_config_examples_identify_firmware_and_units(self):
        emm = parse_response(EMM_CONFIG).value
        x = parse_response(X_CONFIG).value
        self.assertEqual((emm['mode'], emm['microsteps'], emm['motor_step_deg']), ('emm', 16, 1.8))
        self.assertIsNone(emm['speed_div10'])
        self.assertEqual((x['mode'], x['microsteps'], x['angle_div10']), ('x', 16, False))
        self.assertIsNone(x['motor_step_deg'])
        for c in (emm, x):
            self.assertEqual((c['uart_mode'], c['baudrate'], c['ack_mode'], c['checksum_mode']), (2, 115200, 1, 0))
            self.assertAlmostEqual(c['position_window_deg'], .8)

    def test_zero_microstep_byte_means_256(self):
        raw = bytearray(EMM_CONFIG); raw[9] = 0
        self.assertEqual(parse_response(raw).value['microsteps'], 256)

    def test_invalid_config_declared_length_count_address_rejected(self):
        for index, value in [(2, 32), (3, 22), (4, 0x20), (18, 9), (20, 2), (22, 5)]:
            raw = bytearray(EMM_CONFIG); raw[index] = value
            with self.subTest(index=index), self.assertRaises(ProtocolError):
                parse_response(raw)

    def test_options_single_byte_decoded(self):
        o = parse_response(bytes.fromhex('01 1a 87 6b')).value
        self.assertEqual((o['mode'], o['motor_step_deg'], o['closed_loop'], o['scaled_input']), ('emm', .9, True, True))
        self.assertTrue(o['layout_verified'])

    def test_unknown_two_byte_option_order_never_authorizes_motion(self):
        for frame in ('01 1a 00 86 6b', '01 1a 86 00 6b'):
            o = parse_response(bytes.fromhex(frame)).value
            self.assertFalse(o['layout_verified'])
            self.assertIsNone(o['mode'])
            self.assertIsNone(o['scaled_input'])
            with self.assertRaises(ProtocolError):
                verified_motion_parameters(parse_response(EMM_CONFIG).value, o)

    def test_observed_device_profile_cross_validates_without_mutating_raw_read(self):
        raw = parse_response(LIVE_OPTIONS).value
        config = parse_response(LIVE_CONFIG).value
        normalized = normalize_options(parse_response(LIVE_VERSION).value, config, raw)
        self.assertFalse(raw['layout_verified']); self.assertIsNone(raw['mode'])
        self.assertTrue(normalized['layout_verified'])
        self.assertEqual((normalized['raw'], normalized['mode'], normalized['motor_step_deg']), (6, 'emm', 1.8))
        self.assertTrue(normalized['closed_loop']); self.assertFalse(normalized['scaled_input'])
        self.assertEqual(normalized['layout_profile'], 'zdt_xs_fw206_hw2314_emm_u16be')
        self.assertIn('page 39', normalized['verification_evidence']['register_reference'])
        self.assertFalse(verified_motion_parameters(config, normalized)['speed_div10'])

    def test_extended_options_unknown_firmware_or_hardware_remain_read_only(self):
        raw = parse_response(LIVE_OPTIONS).value
        version = parse_response(LIVE_VERSION).value
        for modified in [dict(version, firmware_raw=205), dict(version, firmware_raw=207),
                         dict(version, hardware_raw=0x0314), dict(version, hardware_raw=0x2414)]:
            n = normalize_options(modified, parse_response(LIVE_CONFIG).value, raw)
            self.assertFalse(n['layout_verified'])
            self.assertEqual(n['layout_verification_reason'], 'extended_options_unverified_device_version')
            self.assertIsNone(n['scaled_input'])

    def test_observed_0406_explains_unknown_bit_without_authorizing_motion(self):
        raw = parse_response(bytes.fromhex('01 1a 04 06 6b')).value
        config = parse_response(LIVE_CONFIG).value
        version = parse_response(LIVE_VERSION).value
        n = normalize_options(version, config, raw)
        self.assertEqual(n['raw_bytes_hex'], '04 06')
        self.assertEqual(n['raw_u16be'], 0x0406)
        self.assertEqual(n['documented_bits_raw'], 0x0006)
        self.assertEqual(n['undocumented_bits_raw'], 0x0400)
        self.assertEqual(n['undocumented_bit_indices'], [10])
        self.assertFalse(n['layout_verified'])
        self.assertIsNone(n['mode']); self.assertIsNone(n['scaled_input'])
        self.assertIsNone(n['raw'])
        self.assertNotIn('layout_profile', n)
        self.assertIn('bit(s) 10', n['layout_verification_detail'])
        self.assertNotIn('raw_u16be', raw)
        with self.assertRaisesRegex(ProtocolError, '0x0406.*bit\\(s\\) 10'):
            verified_motion_parameters(config, n)
        self.assertEqual(normalize_options(version, config, n), n)

    def test_diagnostic_bits_are_not_carried_into_an_unknown_device(self):
        config = parse_response(LIVE_CONFIG).value
        version = parse_response(LIVE_VERSION).value
        n = normalize_options(version, config,
                              parse_response(bytes.fromhex('01 1a 04 06 6b')).value)
        changed = normalize_options(dict(version, firmware_raw=207), config, n)
        self.assertFalse(changed['layout_verified'])
        self.assertNotIn('raw_u16be', changed)
        self.assertNotIn('undocumented_bit_indices', changed)
        self.assertNotIn('layout_verification_detail', changed)
        self.assertEqual(changed['layout_verification_reason'],
                         'extended_options_unverified_device_version')

    def test_extended_options_reversed_bytes_reserved_bits_and_conflicts_rejected(self):
        config = parse_response(LIVE_CONFIG).value
        version = parse_response(LIVE_VERSION).value
        cases = [('06 00', 'reserved_bits'), ('00 0e', 'reserved_bits'),
                 ('00 46', 'reserved_bits'), ('04 06', 'reserved_bits'),
                 ('00 04', 'firmware_mode_conflict'), ('00 07', 'motor_step_conflict'),
                 ('00 02', 'closed_loop_unverified_or_conflicting'), ('00 16', 'direction_unverified_or_conflicting')]
        for hex_data, reason in cases:
            raw = parse_response(bytes.fromhex('01 1a ' + hex_data + ' 6b')).value
            n = normalize_options(version, config, raw)
            with self.subTest(data=hex_data):
                self.assertFalse(n['layout_verified'])
                self.assertIn(reason, n['layout_verification_reason'])
                with self.assertRaises(ProtocolError): verified_motion_parameters(config, n)

    def test_extended_profile_does_not_assume_unknown_config_loop_or_direction(self):
        c = parse_response(LIVE_CONFIG).value
        v = parse_response(LIVE_VERSION).value
        o = parse_response(LIVE_OPTIONS).value
        for changes in [dict(mode='x'), dict(closed_loop=None), dict(positive_direction_ccw=None)]:
            self.assertFalse(normalize_options(v, dict(c, **changes), o)['layout_verified'])

    def test_extended_profile_scale_and_parameter_lock_decoded_only_after_cross_checks(self):
        o = parse_response(bytes.fromhex('01 1a 03 86 6b')).value
        c = parse_response(LIVE_CONFIG).value
        n = normalize_options(parse_response(LIVE_VERSION).value, c, o)
        self.assertTrue(n['layout_verified']); self.assertTrue(n['scaled_input'])
        self.assertEqual(n['parameter_lock_level'], 3)
        self.assertTrue(verified_motion_parameters(c, n)['speed_div10'])
        self.assertEqual(normalize_options(parse_response(LIVE_VERSION).value, c, n), n)

    def test_documented_legacy_options_do_not_require_extended_profile(self):
        raw = parse_response(bytes.fromhex('01 1a 06 6b')).value
        self.assertEqual(normalize_options({}, {}, raw), raw)

    def test_verified_config_parameters_and_mode_conflicts(self):
        config = parse_response(EMM_CONFIG).value
        options = parse_response(bytes.fromhex('01 1a 86 6b')).value
        p = verified_motion_parameters(config, options)
        self.assertEqual(p['mode'], 'emm')
        self.assertTrue(p['speed_div10'])
        for changed in [dict(options, mode='x'), dict(options, motor_step_deg=.9),
                        dict(options, scaled_input=None), dict(options, closed_loop=False)]:
            with self.assertRaises(ProtocolError):
                verified_motion_parameters(config, changed)

    def test_unacknowledged_or_non_uart_config_never_authorizes_motion(self):
        config = parse_response(X_CONFIG).value
        options = parse_response(bytes.fromhex('01 1a 04 6b')).value
        for changes in [dict(ack_mode=0), dict(ack_mode=2), dict(ack_mode=4),
                        dict(checksum_mode=1), dict(uart_mode=3), dict(angle_div10=True)]:
            with self.subTest(changes=changes), self.assertRaises(ProtocolError):
                verified_motion_parameters(dict(config, **changes), options)
        self.assertEqual(verified_motion_parameters(dict(config, ack_mode=3), options)['mode'], 'x')

    def test_signed_position_speed_and_unit_distinctions(self):
        raw = parse_response(bytes.fromhex('01 36 01 00 01 00 00 6b')).value
        self.assertEqual(raw, -65536)
        self.assertEqual(position_degrees(raw, 'emm'), -360.)
        self.assertEqual(position_degrees(16, 'x'), 1.6)
        self.assertEqual(error_degrees(16, 'x'), .16)
        speed = parse_response(bytes.fromhex('01 35 01 00 32 6b')).value
        self.assertEqual(speed_rpm(speed, 'emm'), -50)
        self.assertEqual(speed_rpm(speed, 'x'), -5)

    def test_both_target_reads_use_signed_position_feedback_units(self):
        for cmd in (Command.READ_TARGET_POSITION, Command.READ_REALTIME_TARGET_POSITION):
            with self.subTest(command=cmd):
                request = bytes((1, cmd, 0x6B))
                self.assertEqual(read_request(1, cmd), request)
                self.assertEqual(validate_request(request), (1, cmd))
                # 0x00010000 feedback counts is one revolution in Emm,
                # irrespective of the configured microstep command units.
                for sign, expected in ((0, 65536), (1, -65536)):
                    frame = bytes((1, cmd, sign, 0, 1, 0, 0, 0x6B))
                    value = parse_response(frame, expected_address=1, expected_command=cmd).value
                    self.assertEqual(value, expected)
                    self.assertEqual(position_degrees(value, 'emm'), 360 if sign == 0 else -360)
                value = parse_response(bytes((1, cmd, 1, 0, 0, 0, 16, 0x6B))).value
                self.assertEqual(position_degrees(value, 'x'), -1.6)
                for bad in (bytes((1, cmd, 2, 0, 0, 0, 1, 0x6B)),
                            bytes((1, cmd, 0, 0, 0, 1, 0x6B))):
                    with self.assertRaises(ProtocolError): parse_response(bad)

    def test_bad_sign_byte_rejected(self):
        with self.assertRaises(ProtocolError):
            parse_response(bytes.fromhex('01 36 02 00 00 00 01 6b'))

    def test_status_data_is_not_success_or_error_marker(self):
        status = parse_response(bytes.fromhex('01 3a 83 6b')).value
        self.assertTrue(status['enabled']); self.assertTrue(status['arrived'])
        self.assertFalse(status['stalled'])
        self.assertTrue(parse_response(bytes.fromhex('01 3a e2 6b')).ok)

    def test_version_and_voltage_are_vendor_units(self):
        v = parse_response(bytes.fromhex('01 1f 00 c8 03 14 6b')).value
        self.assertEqual((v['firmware'], v['hardware_type'], v['hardware_version_raw']), ('2.0.0', 3, 20))
        self.assertEqual(parse_response(bytes.fromhex('01 24 2e e0 6b')).value, 12.)

    def test_ack_arrival_and_error_are_distinct(self):
        self.assertTrue(parse_response(bytes.fromhex('01 fd 02 6b')).ok)
        arrival = parse_response(bytes.fromhex('01 fd 9f 6b'))
        self.assertTrue(arrival.is_arrival_event)
        self.assertFalse(arrival.ok)
        for frame in ['01 fd e2 6b', '01 fd ee 6b', '01 fb 12 6b', '01 42 ee 6b']:
            with self.assertRaises(DeviceError):
                parse_response(bytes.fromhex(frame)).require_success()

    def test_tail_inside_data_not_frame_boundary(self):
        self.assertEqual(parse_response(bytes.fromhex('01 36 00 00 6b 00 01 6b')).value, 0x6B0001)

    def test_response_mismatches_rejected(self):
        for frame, args in [('01 24 00 01 00', {}), ('01 24 00 01 6b', {'expected_address': 2}),
                            ('01 24 00 01 6b', {'expected_command': Command.READ_SPEED}),
                            ('01 24 00 01 6b 6b', {}), ('00 3a 01 6b', {})]:
            with self.subTest(frame=frame), self.assertRaises(ProtocolError):
                parse_response(bytes.fromhex(frame), **args)

    def test_dangerous_unsupported_requests_rejected(self):
        for frame in ['00 1f 6b', '01 0e 52 6b', '01 48 d1 01 6b',
                      '01 42 6b', '01 f3 ab 01 01 6b', '01 fe 98 01 6b',
                      '01 fd 00 00 05 00 00 00 00 2c 01 01 6b']:
            with self.subTest(frame=frame), self.assertRaises(ProtocolError):
                validate_request(bytes.fromhex(frame))
        for addr in (0, -1, True, 256):
            with self.assertRaises(ProtocolError):
                read_request(addr, Command.READ_POSITION)


if __name__ == '__main__':
    unittest.main(verbosity=2)
