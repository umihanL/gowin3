"""Offline PD42 tests against manual examples, corrupt frames and serial chunks."""
import math
import pathlib
import struct
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.pd42_protocol import (
    Command, RunState, DeviceError, ProtocolError, ResponseDecoder,
    counts_to_degrees, degrees_to_counts, read_request, position_request,
    enable_request, stop_request, clear_status_request, parse_response,
    response_size, response_frame_size, velocity_request,
)


def reply(command, data=b'', *, status=1, address=1):
    # Independent helper for boundary cases; manual golden cases are literals.
    packet = bytes([0xC5, address, int(command), status]) + data
    return packet + bytes([sum(packet) % 256, 0x5C])


class RequestTests(unittest.TestCase):
    def test_vendor_velocity_example_uses_big_endian_float_not_integer_rpm(self):
        self.assertEqual(velocity_request(1,600,100).hex(' '),'c5 01 f1 00 64 44 16 00 00 75 5c')
        ack=parse_response(bytes.fromhex('C5 01 F1 01 B8 5C'))
        self.assertTrue(ack.ok);self.assertIsNone(ack.value)
        self.assertEqual(velocity_request(1,-5,10).hex(' '),'c5 01 f1 01 0a 40 a0 00 00 a2 5c')

    def test_velocity_invalid_values_are_not_sent_as_stop_or_nan(self):
        for rpm in [0,.01,6001,-6001,True,float('nan'),float('inf')]:
            with self.subTest(rpm=rpm),self.assertRaises(ValueError):velocity_request(1,rpm)
        with self.assertRaises(ValueError):velocity_request(0,5)
        with self.assertRaises(ValueError):velocity_request(1,5,201)

    def test_manual_read_requests(self):
        for command, hex_frame in [
            (Command.READ_POSITION, 'C5 01 2A F0 5C'),
            (Command.READ_SPEED, 'C5 01 29 EF 5C'),
            (Command.READ_STATUS, 'C5 01 2C F2 5C'),
            (Command.READ_ENABLED, 'C5 01 2F F5 5C'),
            (Command.READ_ARRIVED, 'C5 01 30 F6 5C'),
            (Command.READ_SYSTEM, 'C5 01 31 F7 5C'),
        ]:
            with self.subTest(command=command):
                self.assertEqual(read_request(1, command), bytes.fromhex(hex_frame))

    def test_manual_absolute_golden_is_thirteen_bytes(self):
        frame = position_request(1, 512000, 2000, 100)
        self.assertEqual(frame, bytes.fromhex('C5 01 F2 00 64 07 D0 00 07 D0 00 CA 5C'))
        self.assertEqual(len(frame), 13)  # vendor C sample incorrectly emits 14

    def test_manual_relative_golden(self):
        self.assertEqual(position_request(1, 512000, 2000, 100, relative=True),
                         bytes.fromhex('C5 01 F3 00 64 07 D0 00 07 D0 00 CB 5C'))

    def test_negative_target_is_sign_magnitude_not_twos_complement(self):
        frame = position_request(2, -12800, 30, 10)
        self.assertEqual(frame, bytes.fromhex('C5 02 F2 01 0A 00 1E 00 00 32 00 14 5C'))

    def test_manual_enable_disable_stop_and_recovery_requests(self):
        self.assertEqual(enable_request(1, True), bytes.fromhex('C5 01 FA 00 C0 5C'))
        self.assertEqual(enable_request(1, False), bytes.fromhex('C5 01 FA 01 C1 5C'))
        self.assertEqual(stop_request(1), bytes.fromhex('C5 01 FC C2 5C'))
        self.assertEqual(clear_status_request(1), bytes.fromhex('C5 01 FB C1 5C'))

    def test_no_broadcast_or_silent_address_coercion(self):
        for address in (0, 256, -1, True, 1.0, '1'):
            with self.subTest(address=address), self.assertRaises(ValueError):
                read_request(address, Command.READ_POSITION)

    def test_read_builder_rejects_motion_and_unknown_commands(self):
        for command in (Command.ENABLE, Command.ABSOLUTE_POSITION, 0xF8, True, 32.0):
            with self.subTest(command=command), self.assertRaises(ValueError):
                read_request(1, command)

    def test_out_of_range_inputs_rejected_without_wrapping(self):
        for target, rpm, acc in [(2**31, 30, 10), (-2**31-1, 30, 10),
                                  (0, -1, 10), (0, 6001, 10),
                                  (0, 30, -1), (0, 30, 201),
                                  (1.1, 30, 10), (0, 1.2, 10), (0, 30, True)]:
            with self.subTest(target=target, rpm=rpm, acc=acc), self.assertRaises(ValueError):
                position_request(1, target, rpm, acc)
        with self.assertRaises(ValueError):
            enable_request(1, 1)
        with self.assertRaises(ValueError):
            position_request(1, 0, 30, 10, relative=1)

    def test_wire_limits_and_zero_have_no_unintended_direction_bit(self):
        self.assertEqual(position_request(1, 0, 0, 0)[3:11], bytes(8))
        self.assertEqual(position_request(255, -(2**31), 6000, 200)[3:11],
                         bytes.fromhex('01 C8 17 70 80 00 00 00'))


class FeedbackTests(unittest.TestCase):
    def test_manual_system_snapshot_contains_independent_retained_target(self):
        # PD42 V1.2 PDF p43 (section 4.3.18), literal manufacturer frame.
        frame = bytes.fromhex('C5 01 31 01 41 C0 00 00 00 0F 40 88 5D 39 3F ED EB 28 40 8D 27 10 '
                              '00 00 00 07 D0 00 00 07 D0 00 00 00 00 00 00 00 7D 02 00 01 00 00 D7 5C')
        result = parse_response(frame, 1, Command.READ_SYSTEM).value
        self.assertEqual(result['target_counts'], 512000)
        self.assertEqual(result['position_counts'], 512000)
        self.assertEqual(result['position_error_counts'], 0)
        self.assertEqual(result['pulse_count'], 32002)
        self.assertEqual(result['speed_rpm'], 0)
        self.assertTrue(result['enabled'])
        self.assertTrue(result['arrived'])
        self.assertFalse(result['stalled'])
        self.assertFalse(result['group_address'])
        self.assertEqual(response_size(Command.READ_SYSTEM), 46)

    def test_system_snapshot_signed_fields_and_malformed_flags(self):
        payload = struct.pack('>fhfffhiiiIBBBB', 24., 15, 4., 2., 4., -3,
                              -200, -180, -20, 32002, 1, 0, 0, 0)
        value = parse_response(reply(Command.READ_SYSTEM, payload)).value
        self.assertEqual((value['target_counts'], value['position_counts']), (-200, -180))
        self.assertEqual(value['position_error_counts'], -20)
        self.assertFalse(value['enabled'])
        for index in range(36, 40):
            corrupted = bytearray(payload); corrupted[index] = 2
            with self.assertRaises(ProtocolError):
                parse_response(reply(Command.READ_SYSTEM, corrupted))
        with self.assertRaises(ProtocolError):
            parse_response(reply(Command.READ_SYSTEM, payload[:-1]))

    def test_manual_position_and_reached_are_different_reply_types(self):
        ack = parse_response(bytes.fromhex('C5 01 F2 01 B9 5C'), 1, Command.ABSOLUTE_POSITION)
        self.assertTrue(ack.ok)
        self.assertIsNone(ack.value)  # an ACK never fabricates an arrived flag
        pos = parse_response(bytes.fromhex('C5 01 2A 01 00 07 D0 00 C8 5C'), 1, Command.READ_POSITION)
        self.assertEqual(pos.value, 512000)
        self.assertEqual(counts_to_degrees(pos.value), 3600)
        self.assertTrue(parse_response(bytes.fromhex('C5 01 30 01 01 F8 5C')).value)

    def test_signed_feedback_position_speed_error_and_current(self):
        for command, raw, expected in [
            (Command.READ_POSITION, bytes.fromhex('FF FF CE 00'), -12800),
            (Command.READ_POSITION_ERROR, bytes.fromhex('FF FF FF FF'), -1),
            (Command.READ_SPEED, bytes.fromhex('FF E2'), -30),
            (Command.READ_CURRENT, bytes.fromhex('FF 9C'), -100),
        ]:
            with self.subTest(command=command):
                self.assertEqual(parse_response(reply(command, raw)).value, expected)

    def test_feedback_int32_boundaries(self):
        for raw, expected in [('80 00 00 00', -(2**31)), ('7F FF FF FF', 2**31-1)]:
            self.assertEqual(parse_response(reply(Command.READ_POSITION, bytes.fromhex(raw))).value, expected)

    def test_enabled_uses_manual_field_definition_and_preserves_raw(self):
        enabled = parse_response(bytes.fromhex('C5 01 2F 01 00 F6 5C'))
        self.assertIs(enabled.value, True)
        self.assertEqual(enabled.data, b'\x00')
        self.assertIs(parse_response(reply(Command.READ_ENABLED, b'\x01')).value, False)
        self.assertIs(parse_response(bytes.fromhex('C5 01 FA 01 00 C1 5C')).value, True)

    def test_status_enum_and_fault_states(self):
        for state in RunState:
            self.assertEqual(parse_response(reply(Command.READ_STATUS, bytes([state]))).value, state)

    def test_manual_version_voltage_and_speed(self):
        result = parse_response(bytes.fromhex('C5 01 20 01 0A 0A FB 5C'))
        self.assertEqual(result.value['firmware_raw'], 10)
        self.assertEqual(result.value['firmware'], 1.0)
        self.assertAlmostEqual(parse_response(reply(Command.READ_VOLTAGE, bytes.fromhex('41 BF C7 C0'))).value, 23.97253, places=4)
        self.assertEqual(parse_response(bytes.fromhex('C5 01 29 01 00 1F 0F 5C')).value, 31)

    def test_fault_ack_never_becomes_feedback(self):
        for status in range(0xE1, 0xE7):
            for data in (b'', b'\x00\x00\x00\x00'):
                result = parse_response(reply(Command.READ_POSITION, data, status=status))
                self.assertFalse(result.ok)
                self.assertIsNone(result.value)
                with self.assertRaises(DeviceError) as caught:
                    result.require_success()
                self.assertEqual(caught.exception.response.status, status)

    def test_reject_corrupt_delimiters_checksum_address_and_command(self):
        good = bytes.fromhex('C5 01 2A 01 00 07 D0 00 C8 5C')
        for index in (0, 1, 2, 5, -2, -1):
            changed = bytearray(good)
            changed[index] ^= 0x10
            with self.subTest(index=index), self.assertRaises(ProtocolError):
                parse_response(changed)
        with self.assertRaises(ProtocolError):
            parse_response(good, expected_address=2)
        with self.assertRaises(ProtocolError):
            parse_response(good, expected_command=Command.READ_SPEED)
        with self.assertRaises(ProtocolError):
            parse_response(reply(Command.READ_SPEED, b'\x00\x01', address=0))

    def test_reject_extra_bytes_truncation_and_wrong_payload_size(self):
        good = bytes.fromhex('C5 01 2A 01 00 07 D0 00 C8 5C')
        for length in range(len(good)):
            with self.assertRaises(ProtocolError):
                parse_response(good[:length])
        for frame in [good + b'\x00', reply(Command.READ_POSITION, b'\x01'),
                      reply(Command.READ_POSITION, b'\x01', status=0xE4)]:
            with self.assertRaises(ProtocolError):
                parse_response(frame)

    def test_reject_unknown_status_invalid_flags_and_nonfinite_voltage(self):
        for frame in [reply(Command.READ_POSITION, bytes(4), status=0),
                      reply(Command.READ_ENABLED, b'\x02'),
                      reply(Command.READ_ARRIVED, b'\xFF'),
                      reply(Command.READ_STATUS, b'\x06'),
                      reply(0xF8), reply(Command.READ_VOLTAGE, struct.pack('>f', math.nan)),
                      reply(Command.READ_VOLTAGE, struct.pack('>f', -1))]:
            with self.subTest(frame=frame.hex()), self.assertRaises(ProtocolError):
                parse_response(frame)

    def test_reply_lengths(self):
        self.assertEqual(response_size(Command.READ_POSITION), 10)
        self.assertEqual(response_size(Command.READ_SPEED), 8)
        self.assertEqual(response_size(Command.READ_STATUS), 7)
        self.assertEqual(response_frame_size(Command.ABSOLUTE_POSITION), 6)


class StreamAndUnitsTests(unittest.TestCase):
    def test_fragmented_and_concatenated_frames_with_embedded_delimiters(self):
        # Embedded 5C must not truncate; embedded C5 must not restart the frame.
        first = reply(Command.READ_POSITION, bytes.fromhex('00 C5 5C 01'))
        second = reply(Command.READ_STATUS, b'\x02')
        for split in range(len(first) + len(second) + 1):
            decoder = ResponseDecoder()
            packet = first + second
            result = decoder.feed(packet[:split]) + decoder.feed(packet[split:])
            self.assertEqual([r.value for r in result], [0xC55C01, RunState.RUNNING])

    def test_bytewise_short_and_long_error_replies(self):
        for body in [b'', b'\x00\x00\x00\x00']:
            decoder = ResponseDecoder()
            frames = []
            wire = reply(Command.READ_POSITION, body, status=0xE4) + reply(Command.READ_STATUS, b'\x00')
            for byte in wire:
                frames.extend(decoder.feed(bytes([byte])))
            self.assertEqual(len(frames), 2)
            self.assertFalse(frames[0].ok)
            self.assertEqual(frames[1].value, RunState.STOPPED)

    def test_noise_invalid_length_and_checksum_resynchronize(self):
        good = reply(Command.READ_POSITION, bytes.fromhex('FF FF CE 00'))
        bad = bytearray(good)
        bad[-2] ^= 1
        decoder = ResponseDecoder()
        result = decoder.feed(b'noise\xC5\x01\xEE\x01' + bytes(bad) + good)
        self.assertEqual([r.value for r in result], [-12800])
        self.assertGreater(decoder.malformed_frames, 0)
        self.assertGreater(decoder.discarded_bytes, 0)

    def test_reset_discards_timeout_partial_reply(self):
        decoder = ResponseDecoder()
        decoder.feed(bytes.fromhex('C5 01 2A 01 00'))
        decoder.reset()
        self.assertEqual(decoder.discarded_bytes, 5)
        result = decoder.feed(reply(Command.READ_STATUS, b'\x00'))
        self.assertEqual(result[0].value, RunState.STOPPED)

    def test_degree_conversion_has_no_wrap_and_bounded_quantization(self):
        self.assertEqual(degrees_to_counts(90), 12800)
        self.assertEqual(degrees_to_counts(-90), -12800)
        self.assertEqual(degrees_to_counts(720), 102400)
        for degrees in [0, -180, 0.001, 12.345, -36.281, 3600]:
            recovered = counts_to_degrees(degrees_to_counts(degrees))
            self.assertLessEqual(abs(recovered-degrees), 180/51200 + 1e-9)
        for invalid in [math.nan, math.inf, -math.inf, True, '90', 1e20, 1e308, 10**1000]:
            with self.assertRaises(ValueError):
                degrees_to_counts(invalid)


if __name__ == '__main__':
    unittest.main(verbosity=2)
