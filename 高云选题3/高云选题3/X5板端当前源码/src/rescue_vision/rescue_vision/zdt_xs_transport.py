"""Bounded ZDT fixed-6B serial exchange; never retry uncertain commands.

An ACK is only receipt, not arrival. In Both reply mode unsolicited FB/FD 9F
events may precede a read; they are recorded separately, never used as its
response. Opening/closing never enables, disables, moves or writes settings.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

from .zdt_xs_protocol import (
    Command, DEFAULT_BAUDRATE, ProtocolError, RESPONSE_LENGTHS,
    parse_response, validate_request,
)


class ZDTTimeoutError(TimeoutError):
    """Command may have executed: caller must not resend it automatically."""


class ZDTTransportBusyError(RuntimeError):
    pass


@dataclass(frozen=True)
class ArrivalEvent:
    response: object
    received_at_monotonic: float


class ZDTSerialTransport:
    def __init__(self, port, baudrate=DEFAULT_BAUDRATE, timeout_s=0.2, *, serial_factory=None):
        if not isinstance(port, str) or not port.strip():
            raise ValueError('explicit nonempty serial port required')
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError('positive integer baudrate required')
        if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
                or not math.isfinite(timeout_s) or timeout_s <= 0):
            raise ValueError('positive finite timeout required')
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self.port = port
        self.timeout_s = float(timeout_s)
        self._transaction_lock = threading.Lock()
        self._closed = False
        self.events = deque(maxlen=16)
        self._serial = serial_factory(port=port, baudrate=baudrate, bytesize=8,
                                      parity='N', stopbits=1,
                                      timeout=min(self.timeout_s, .01),
                                      write_timeout=self.timeout_s,
                                      xonxoff=False, rtscts=False, dsrdtr=False,
                                      exclusive=True)

    @staticmethod
    def _length(buffer, *, quiet=False):
        if len(buffer) < 2:
            return None
        try:
            command = Command(buffer[1])
        except ValueError as exc:
            raise ProtocolError('unknown serial reply command') from exc
        if command == Command.READ_CONFIG:
            if len(buffer) < 3:
                return None
            if buffer[2] in (0xE2, 0xEE):
                if quiet and len(buffer) == 4:
                    return 4
                if len(buffer) <= 4:
                    return None
            if buffer[2] not in (33, 37):
                raise ProtocolError('unsupported driver configuration length')
            return buffer[2]
        if command == Command.READ_OPTIONS:
            # Table and new firmware text differ (one/two bytes). Wait for a
            # bounded idle interval before selecting either explicit layout.
            if quiet:
                if len(buffer) not in (4, 5):
                    raise ProtocolError('ambiguous options reply length')
                return len(buffer)
            if len(buffer) > 5:
                raise ProtocolError('unexpected bytes after options reply')
            return None
        length = RESPONSE_LENGTHS[command]
        if quiet and len(buffer) == 4 and length > 4 and buffer[2] in (0xE2, 0xEE) and buffer[3] == 0x6B:
            return 4
        return length

    def exchange(self, request):
        address, command = validate_request(request)
        request = bytes(request)
        if not self._transaction_lock.acquire(blocking=False):
            raise ZDTTransportBusyError('ZDT transaction already pending')
        try:
            if self._closed:
                raise RuntimeError('ZDT transport closed')
            self._serial.reset_input_buffer()
            written = self._serial.write(request)
            if written != len(request):
                raise ProtocolError('partial ZDT write; outcome unknown, not retried')
            # No tcdrain/flush is needed for this request/reply protocol. A
            # complete matched reply proves transmission; X5 UART drain can
            # otherwise block tens of milliseconds after that reply arrives.
            deadline = time.perf_counter() + self.timeout_s
            last_received = time.perf_counter()
            buffer = bytearray()
            result = None
            arrival_count = 0
            while True:
                now = time.perf_counter()
                quiet = bool(buffer) and (now - last_received >= min(.01, self.timeout_s / 2) or now >= deadline)
                while buffer:
                    if buffer[0] != address:
                        raise ProtocolError('ZDT response address mismatch')
                    length = self._length(buffer, quiet=quiet)
                    if length is None or len(buffer) < length:
                        break
                    frame = bytes(buffer[:length])
                    del buffer[:length]
                    response = parse_response(frame, expected_address=address)
                    if response.is_arrival_event:
                        self.events.append(ArrivalEvent(response, last_received))
                        arrival_count += 1
                        if arrival_count > 16:
                            raise ProtocolError('too many asynchronous arrival events')
                        continue
                    if response.command != command:
                        raise ProtocolError('ZDT response command mismatch')
                    if result is not None:
                        raise ProtocolError('multiple ZDT replies for one request')
                    result = response.require_success()
                if result is not None:
                    if buffer:
                        raise ProtocolError('trailing incomplete bytes after ZDT reply')
                    return result
                if now >= deadline:
                    if buffer:
                        raise ProtocolError('incomplete or ambiguous ZDT reply; not retried')
                    raise ZDTTimeoutError(f'ZDT address {address}, command 0x{command:02X} timeout; '
                                          'execution is unknown and command was not retried')
                self._serial.timeout = min(.01, deadline - now)
                waiting = int(getattr(self._serial, 'in_waiting', 0))
                chunk = self._serial.read(min(max(waiting, 1), 256))
                if chunk:
                    buffer.extend(chunk)
                    last_received = time.perf_counter()
                    if len(buffer) > 512:
                        raise ProtocolError('ZDT receive buffer overflow')
        finally:
            self._transaction_lock.release()

    def close(self):
        with self._transaction_lock:
            if not self._closed:
                self._serial.close()
                self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
