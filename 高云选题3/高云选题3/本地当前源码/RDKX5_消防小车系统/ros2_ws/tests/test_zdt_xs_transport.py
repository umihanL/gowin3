"""Wire-level failures/fragmentation with fake serial only, never hardware."""
import pathlib
import sys
import threading
import time
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.zdt_xs_protocol import Command, ProtocolError, DeviceError, read_request, enable_request, velocity_request, select_firmware_request, zero_current_position_request
from rescue_vision.zdt_xs_transport import ZDTSerialTransport, ZDTTimeoutError, ZDTTransportBusyError


class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.timeout = kwargs['timeout']
        self.rx = bytearray()
        self.response = bytes.fromhex('01 36 00 00 01 00 00 6b')
        self.writes = []
        self.closed = False
        self.partial = False
        self.max_chunk = 256
        self.reset_calls = 0

    @property
    def in_waiting(self): return len(self.rx)

    def write(self, data):
        self.writes.append(data)
        self.rx.extend(self.response)
        return len(data) - 1 if self.partial else len(data)

    def flush(self): pass

    def reset_input_buffer(self):
        self.reset_calls += 1
        self.rx.clear()

    def read(self, size):
        if not self.rx:
            time.sleep(self.timeout)
            return b''
        size = min(size, self.max_chunk)
        chunk = bytes(self.rx[:size]); del self.rx[:size]
        return chunk

    def close(self): self.closed = True


class ZDTTransportTests(unittest.TestCase):
    def test_single_turn_encoder_fragmented_reply_and_timeout_never_reuse_old_value(self):
        link, uart = self.make_link(); uart.max_chunk = 1
        packet = read_request(1, Command.READ_ENCODER)
        uart.response = bytes.fromhex('01 31 e2 6b 6b')
        self.assertEqual(link.exchange(packet).value, 0xE26B)
        uart.response = b''
        with self.assertRaises(ZDTTimeoutError):
            link.exchange(packet)
        uart.response = bytes.fromhex('01 31 ee 6b')
        with self.assertRaises(DeviceError):
            link.exchange(packet)
        self.assertEqual(uart.writes, [packet]*3)

    def test_current_position_zero_ack_and_lost_ack_never_add_control_writes(self):
        link, uart = self.make_link(); uart.max_chunk = 1
        packet = zero_current_position_request(1)
        uart.response = bytes.fromhex('01 0a 02 6b')
        self.assertTrue(link.exchange(packet).ok)
        self.assertEqual(uart.writes, [packet])
        uart.response = b''
        with self.assertRaises(ZDTTimeoutError): link.exchange(packet)
        link.close()
        self.assertEqual(uart.writes, [packet, packet])

    def test_firmware_selection_has_four_byte_ack_and_is_never_automatically_retried(self):
        link, uart = self.make_link(); uart.max_chunk = 1
        packet = select_firmware_request(1, 'emm')
        uart.response = bytes.fromhex('01 d5 02 6b')
        self.assertTrue(link.exchange(packet).ok)
        self.assertEqual(uart.writes, [packet])
        uart.response = b''
        with self.assertRaises(ZDTTimeoutError): link.exchange(packet)
        self.assertEqual(uart.writes, [packet, packet])
        link.close()
        self.assertEqual(uart.writes, [packet, packet])

    def test_both_velocity_layouts_send_once_and_lost_ack_does_not_retry(self):
        for mode in ('emm','x'):
            link,uart=self.make_link();uart.response=bytes.fromhex('01 f6 02 6b')
            frame=velocity_request(1,-5,mode=mode,speed_div10=False)
            self.assertTrue(link.exchange(frame).ok)
            self.assertEqual(uart.writes,[frame])
            uart.response=b''
            with self.assertRaises(ZDTTimeoutError):link.exchange(frame)
            self.assertEqual(uart.writes,[frame,frame])

    def make_link(self, timeout=.03):
        link = ZDTSerialTransport('/dev/fakeZDT', timeout_s=timeout, serial_factory=FakeSerial)
        self.addCleanup(link.close)
        return link, link._serial

    def test_open_close_no_writes_8n1_no_flow_control(self):
        link, uart = self.make_link()
        self.assertEqual(uart.writes, [])
        self.assertEqual((uart.kwargs['baudrate'], uart.kwargs['bytesize'], uart.kwargs['parity'], uart.kwargs['stopbits']), (115200, 8, 'N', 1))
        for key in ['xonxoff', 'rtscts', 'dsrdtr']: self.assertFalse(uart.kwargs[key])
        self.assertTrue(uart.kwargs['exclusive'])
        link.close(); self.assertTrue(uart.closed); self.assertEqual(uart.writes, [])

    def test_fragmented_response_is_single_transaction(self):
        link, uart = self.make_link(); uart.max_chunk = 1
        request = read_request(1, Command.READ_POSITION)
        self.assertEqual(link.exchange(request).value, 65536)
        self.assertEqual(uart.writes, [request])

    def test_deferred_fragmented_response_does_not_need_uart_drain(self):
        link, uart = self.make_link()
        reply, uart.response, uart.max_chunk = uart.response, b'', 1
        uart.flush = lambda: self.fail('tcdrain must not delay request/reply reads')
        original_read = uart.read
        calls = []
        def deferred(size):
            calls.append(size)
            if len(calls) == 1:
                return b''
            if len(calls) == 2:
                uart.rx.extend(reply)
            return original_read(size)
        uart.read = deferred
        request = read_request(1, Command.READ_POSITION)
        self.assertEqual(link.exchange(request).value, 65536)
        self.assertEqual(uart.writes, [request])
        self.assertGreater(len(calls), len(reply))

    def test_target_reads_accept_fragmentation_and_do_not_stop_at_payload_6b(self):
        for cmd in (Command.READ_TARGET_POSITION, Command.READ_REALTIME_TARGET_POSITION):
            with self.subTest(command=cmd):
                link, uart = self.make_link(); uart.max_chunk = 1
                uart.response = bytes((1, cmd, 1, 0, 0x6B, 0, 1, 0x6B))
                request = read_request(1, cmd)
                self.assertEqual(link.exchange(request).value, -0x006B0001)
                self.assertEqual(uart.writes, [request])

    def test_target_read_rejection_is_not_position_data_or_motion(self):
        for cmd in (Command.READ_TARGET_POSITION, Command.READ_REALTIME_TARGET_POSITION):
            link, uart = self.make_link(); uart.max_chunk = 1
            uart.response = bytes((1, cmd, 0xEE, 0x6B))
            request = read_request(1, cmd)
            with self.assertRaises(DeviceError): link.exchange(request)
            self.assertEqual(uart.writes, [request])

    def test_config_fragmentation_and_tail_in_data(self):
        link, uart = self.make_link(); uart.max_chunk = 1
        uart.response = bytes.fromhex('01 42 21 15 19 02 02 02 00 10 01 00 04 b0 0b 80 0f a0 05 07 01 00 01 01 00 08 08 98 07 d0 00 6b 6b')
        self.assertEqual(link.exchange(read_request(1, Command.READ_CONFIG)).value['mode'], 'emm')

    def test_legacy_options_wait_for_complete_frame(self):
        link, uart = self.make_link(); uart.response = bytes.fromhex('01 1a 86 6b'); uart.max_chunk = 1
        value = link.exchange(read_request(1, Command.READ_OPTIONS)).value
        self.assertTrue(value['scaled_input']); self.assertTrue(value['layout_verified'])

    def test_new_options_remain_read_only(self):
        link, uart = self.make_link(); uart.response = bytes.fromhex('01 1a 00 86 6b'); uart.max_chunk = 1
        value = link.exchange(read_request(1, Command.READ_OPTIONS)).value
        self.assertFalse(value['layout_verified']); self.assertIsNone(value['mode'])

    def test_stale_buffer_cleared_before_request(self):
        link, uart = self.make_link(); uart.rx.extend(bytes.fromhex('01 fd 02 6b'))
        self.assertEqual(link.exchange(read_request(1, Command.READ_POSITION)).value, 65536)
        self.assertEqual(uart.reset_calls, 1)

    def test_arrival_event_preceding_read_is_recorded_with_time(self):
        link, uart = self.make_link(); uart.response = bytes.fromhex('01 fd 9f 6b') + uart.response
        before = time.perf_counter()
        self.assertEqual(link.exchange(read_request(1, Command.READ_POSITION)).value, 65536)
        self.assertEqual(len(link.events), 1)
        self.assertTrue(link.events[0].response.is_arrival_event)
        self.assertGreaterEqual(link.events[0].received_at_monotonic, before)
        self.assertLessEqual(link.events[0].received_at_monotonic, time.perf_counter())

    def test_arrival_event_never_substitutes_for_ack(self):
        link, uart = self.make_link(); uart.response = bytes.fromhex('01 fd 9f 6b')
        from rescue_vision.zdt_xs_protocol import position_request
        request = position_request(1, 5, 5, mode='emm', microsteps=16, motor_step_deg=1.8, speed_div10=False)
        with self.assertRaises(ZDTTimeoutError): link.exchange(request)
        self.assertEqual(uart.writes, [request]); self.assertEqual(len(link.events), 1)

    def test_control_error_is_device_error(self):
        link, uart = self.make_link(); uart.response = bytes.fromhex('01 f3 e2 6b')
        with self.assertRaises(DeviceError): link.exchange(enable_request(1, True))

    def test_short_config_error_is_device_error_not_layout_error(self):
        for command, response in [(Command.READ_CONFIG, '01 42 ee 6b'), (Command.READ_VERSION, '01 1f e2 6b')]:
            link, uart = self.make_link(); uart.response = bytes.fromhex(response); uart.max_chunk = 1
            with self.assertRaises(DeviceError): link.exchange(read_request(1, command))

    def test_no_response_times_out_without_retry_or_disable(self):
        link, uart = self.make_link(); uart.response = b''
        request = enable_request(1, True)
        with self.assertRaises(ZDTTimeoutError): link.exchange(request)
        link.close(); self.assertEqual(uart.writes, [request])

    def test_partial_write_never_retries(self):
        link, uart = self.make_link(); uart.partial = True
        uart.read = lambda size: self.fail('a partial write cannot consume buffered reply bytes')
        uart.flush = lambda: self.fail('a partial write must not wait for drain')
        with self.assertRaises(ProtocolError): link.exchange(read_request(1, Command.READ_POSITION))
        self.assertEqual(len(uart.writes), 1)

    def test_bad_checksum_address_command_or_trailing_data_rejected(self):
        for response in ['02 36 00 00 00 00 01 6b', '01 35 00 00 01 6b',
                         '01 36 00 00 00 00 01 5c', '01 36 00 00 00 00 01 6b 00',
                         '01 36 00 00 00 00 01 6b 01 36 00 00 00 00 01 6b']:
            link, uart = self.make_link(); uart.response = bytes.fromhex(response)
            with self.subTest(response=response), self.assertRaises(ProtocolError):
                link.exchange(read_request(1, Command.READ_POSITION))

    def test_unsupported_raw_command_never_written(self):
        link, uart = self.make_link()
        for raw in ['00 1f 6b', '01 0e 52 6b', '01 48 d1 6b', '01 42 6b']:
            with self.assertRaises(ProtocolError): link.exchange(bytes.fromhex(raw))
        self.assertEqual(uart.writes, [])

    def test_truncated_reply_is_not_success(self):
        link, uart = self.make_link(); uart.response = bytes.fromhex('01 36 00 00')
        with self.assertRaises(ProtocolError): link.exchange(read_request(1, Command.READ_POSITION))

    def test_busy_transaction_is_not_queued(self):
        link, uart = self.make_link(.07); uart.response = b''
        result = []
        def worker():
            try: link.exchange(read_request(1, Command.READ_POSITION))
            except ZDTTimeoutError: result.append('timeout')
        thread = threading.Thread(target=worker); thread.start()
        deadline = time.monotonic() + 1
        while not uart.writes and time.monotonic() < deadline: time.sleep(.001)
        with self.assertRaises(ZDTTransportBusyError): link.exchange(read_request(1, Command.READ_POSITION))
        thread.join(1)
        self.assertEqual(result, ['timeout']); self.assertEqual(len(uart.writes), 1)

    def test_closed_or_invalid_transport_never_opens_or_writes(self):
        link, uart = self.make_link(); link.close()
        with self.assertRaises(RuntimeError): link.exchange(read_request(1, Command.READ_POSITION))
        self.assertEqual(uart.writes, [])
        for args in [dict(port=''), dict(port=None), dict(port='fake', baudrate=True),
                     dict(port='fake', timeout_s=0), dict(port='fake', timeout_s=float('nan'))]:
            with self.assertRaises(ValueError): ZDTSerialTransport(**args, serial_factory=lambda **kw: self.fail('opened'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
