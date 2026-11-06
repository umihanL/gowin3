"""No UART access: exact transmitted bytes and transport failure semantics."""
import pathlib
import sys
import threading
import time
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
from rescue_vision.pd42_protocol import Command, DeviceError, ProtocolError, read_request, position_request, velocity_request
from rescue_vision.pd42_transport import PD42SerialTransport, PD42TimeoutError, PD42TransportBusyError


class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.timeout = kwargs['timeout']
        self.rx = bytearray()
        self.response = bytes.fromhex('C5 01 2A 01 00 07 D0 00 C8 5C')
        self.writes = []
        self.write_times = []
        self.closed = False
        self.partial = False
        self.max_chunk = 256
        self.reset_calls = 0

    @property
    def in_waiting(self):
        return len(self.rx)

    def write(self, data):
        self.writes.append(data)
        self.write_times.append(time.perf_counter())
        self.rx.extend(self.response)
        return len(data)-1 if self.partial else len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.reset_calls += 1
        self.rx.clear()

    def read(self, size):
        if not self.rx:
            time.sleep(self.timeout)
            return b''
        size = min(size, self.max_chunk)
        data = self.rx[:size]
        del self.rx[:size]
        return bytes(data)

    def close(self):
        self.closed = True


class TransportTests(unittest.TestCase):
    def test_native_velocity_frame_is_validated_and_written_once(self):
        link,uart=self.make_transport();uart.response=bytes.fromhex('C5 01 F1 01 B8 5C')
        request=velocity_request(1,-5,10)
        self.assertTrue(link.exchange(request).ok);self.assertEqual(uart.writes,[request])

    def test_native_velocity_bad_float_and_wrong_payload_never_written(self):
        link,uart=self.make_transport()
        for payload in ['00 0A 7F C0 00 00','00 0A 00 00 00 00','00 0A 00 05','02 0A 40 A0 00 00']:
            body=bytes.fromhex('C5 01 F1 '+payload)
            frame=body+bytes((sum(body)&255,0x5C))
            with self.subTest(payload=payload),self.assertRaises(ProtocolError):link.exchange(frame)
        self.assertEqual(uart.writes,[])

    def make_transport(self, timeout=0.01):
        link = PD42SerialTransport('/dev/fakePD42', timeout_s=timeout, serial_factory=FakeSerial)
        self.addCleanup(link.close)
        return link, link._serial

    def test_open_close_configures_port_but_never_writes_motion(self):
        link, uart = self.make_transport()
        self.assertEqual(uart.writes, [])
        self.assertEqual(uart.kwargs['bytesize'], 8)
        self.assertEqual(uart.kwargs['parity'], 'N')
        self.assertEqual(uart.kwargs['stopbits'], 1)
        self.assertTrue(uart.kwargs['exclusive'])
        self.assertEqual(uart.kwargs['baudrate'], 115200)
        link.close()
        self.assertTrue(uart.closed)
        self.assertEqual(uart.writes, [])

    def test_one_contiguous_write_parses_fragmented_reply(self):
        link, uart = self.make_transport()
        uart.max_chunk = 1
        request = read_request(1, Command.READ_POSITION)
        self.assertEqual(link.exchange(request).value, 512000)
        self.assertEqual(uart.writes, [request])

    def test_clears_stale_buffer_before_new_read(self):
        link, uart = self.make_transport()
        uart.rx.extend(bytes.fromhex('C5 01 2C 01 01 F4 5C'))
        self.assertEqual(link.exchange(read_request(1, Command.READ_POSITION)).value, 512000)
        self.assertEqual(uart.reset_calls, 1)

    def test_minimum_two_ms_between_complete_requests(self):
        link, uart = self.make_transport()
        request = read_request(1, Command.READ_POSITION)
        link.exchange(request)
        link.exchange(request)
        self.assertGreaterEqual(uart.write_times[1]-uart.write_times[0], 0.002)

    def test_timeout_motion_has_exactly_one_write_and_no_implicit_stop(self):
        link, uart = self.make_transport()
        uart.response = b''
        motion = position_request(1, 12800, 10, 10)
        with self.assertRaises(PD42TimeoutError):
            link.exchange(motion)
        self.assertEqual(uart.writes, [motion])
        link.close()
        self.assertEqual(uart.writes, [motion])

    def test_partial_write_no_retry(self):
        link, uart = self.make_transport()
        uart.partial = True
        with self.assertRaises(ProtocolError):
            link.exchange(read_request(1, Command.READ_POSITION))
        self.assertEqual(len(uart.writes), 1)

    def test_short_device_error_is_not_success_or_timeout(self):
        link, uart = self.make_transport()
        uart.response = bytes.fromhex('C5 01 2A E6 D6 5C')
        with self.assertRaises(DeviceError):
            link.exchange(read_request(1, Command.READ_POSITION))
        self.assertEqual(len(uart.writes), 1)

    def test_wrong_address_or_command_rejected(self):
        for response in [bytes.fromhex('C5 02 2A 01 00 07 D0 00 C9 5C'),
                         bytes.fromhex('C5 01 2C 01 01 F4 5C')]:
            link, uart = self.make_transport()
            uart.response = response
            with self.assertRaises(ProtocolError):
                link.exchange(read_request(1, Command.READ_POSITION))

    def test_invalid_checksum_reply_rejected(self):
        link, uart = self.make_transport()
        uart.response = bytes.fromhex('C5 01 2A 01 00 07 D0 00 C9 5C')
        with self.assertRaises(ProtocolError):
            link.exchange(read_request(1, Command.READ_POSITION))

    def test_invalid_or_dangerous_raw_request_never_written(self):
        link, uart = self.make_transport()
        for frame in [b'', bytes.fromhex('C5 01 F8 BE 5C'),  # reset reference not supported
                      bytes.fromhex('C5 00 2A EF 5C'),       # broadcast
                      bytes.fromhex('C5 01 2A 00 F0 5C'),   # extra payload
                      bytes.fromhex('C5 01 2A F1 5C')]:     # bad checksum
            with self.subTest(frame=frame.hex()), self.assertRaises(ProtocolError):
                link.exchange(frame)
        self.assertEqual(uart.writes, [])

    def test_no_concurrent_transaction_is_queued(self):
        link, uart = self.make_transport(timeout=0.05)
        uart.response = b''
        started = threading.Event()
        outcomes = []

        def exchange():
            started.set()
            try:
                link.exchange(read_request(1, Command.READ_POSITION))
            except PD42TimeoutError:
                outcomes.append('timeout')

        worker = threading.Thread(target=exchange)
        worker.start()
        self.assertTrue(started.wait(1))
        deadline = time.monotonic()+1
        while not uart.writes and time.monotonic() < deadline:
            time.sleep(0.001)
        with self.assertRaises(PD42TransportBusyError):
            link.exchange(read_request(1, Command.READ_POSITION))
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(outcomes, ['timeout'])
        self.assertEqual(len(uart.writes), 1)

    def test_closed_transport_never_writes(self):
        link, uart = self.make_transport()
        link.close()
        with self.assertRaises(RuntimeError):
            link.exchange(read_request(1, Command.READ_POSITION))
        self.assertEqual(uart.writes, [])

    def test_invalid_configuration_does_not_open_port(self):
        opened = []
        def factory(**kwargs):
            opened.append(kwargs)
        for args in [dict(port=''), dict(port=None), dict(port='fake', baudrate=True),
                     dict(port='fake', timeout_s=0), dict(port='fake', timeout_s=float('nan'))]:
            with self.assertRaises(ValueError):
                PD42SerialTransport(**args, serial_factory=factory)
        self.assertEqual(opened, [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
