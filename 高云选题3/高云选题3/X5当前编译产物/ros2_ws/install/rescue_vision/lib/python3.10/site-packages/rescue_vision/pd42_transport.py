"""One-at-a-time PD42 native UART transactions (never automatic motion/retry)."""
from __future__ import annotations

import math
import struct
import threading
import time

from .pd42_protocol import (
    Command, HEAD, TAIL, DEFAULT_BAUDRATE, MIN_COMMAND_INTERVAL_S,
    ProtocolError, Response, ResponseDecoder, read_request, position_request,
    enable_request, stop_request, clear_status_request, velocity_request,
)


class PD42TimeoutError(TimeoutError):
    """The request may have executed; it MUST NOT be resent automatically."""


class PD42TransportBusyError(RuntimeError):
    """Another request owns this serial link; do not queue stale motion."""


def _validate_request(request: bytes) -> tuple[int, Command]:
    if not isinstance(request, (bytes, bytearray, memoryview)):
        raise ProtocolError("request must be bytes")
    request = bytes(request)
    if len(request) < 5 or request[0] != HEAD or request[-1] != TAIL:
        raise ProtocolError("invalid request frame delimiters/length")
    if (sum(request[:-2]) & 0xFF) != request[-2]:
        raise ProtocolError("invalid request checksum")
    address = request[1]
    payload = request[3:-2]
    try:
        command = Command(request[2])
        if command == Command.VELOCITY:
            if len(payload) != 6:
                raise ValueError('velocity payload must be exactly six bytes')
            direction, acceleration, rpm = struct.unpack('>BBf', payload)
            if direction not in (0, 1):
                raise ValueError('invalid direction')
            expected = velocity_request(address, -rpm if direction else rpm, acceleration)
        elif command in (Command.ABSOLUTE_POSITION, Command.RELATIVE_POSITION):
            if len(payload) != 8:
                raise ValueError("position payload must be exactly eight bytes")
            direction, acceleration, rpm, magnitude = struct.unpack('>BBHI', payload)
            if direction not in (0, 1):
                raise ValueError("invalid direction")
            target = -magnitude if direction else magnitude
            expected = position_request(address, target, rpm, acceleration,
                                        relative=command == Command.RELATIVE_POSITION)
        elif command == Command.ENABLE:
            if payload not in (b'\x00', b'\x01'):
                raise ValueError("invalid enable payload")
            expected = enable_request(address, payload == b'\x00')
        elif command == Command.STOP:
            expected = stop_request(address)
        elif command == Command.CLEAR_STATUS:
            expected = clear_status_request(address)
        else:
            expected = read_request(address, command)
        if request != expected:
            raise ValueError("unexpected request length or noncanonical payload")
    except (ValueError, struct.error) as exc:
        raise ProtocolError(str(exc)) from exc
    return address, command


class PD42SerialTransport:
    """Open one UART/USB serial port in 8N1 and send explicitly supplied frames.

    Construction and close only open/close the port: they never enable, home,
    clear faults or move a motor. A per-port lock rejects concurrent exchange
    calls. Run exchange in the existing actuator worker, not a ROS callback.

    A verified response only acknowledges the command/returns a sensor reading.
    It does not prove target arrival or a mechanically safe gimbal position.
    There is no transaction ID in the wire protocol: after an uncertain motion
    result, application code must enter a fault state and inspect actual motor
    feedback before any new motion. No request is retried here.
    """

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE,
                 timeout_s: float = 0.15, *, serial_factory=None):
        if not isinstance(port, str) or not port.strip():
            raise ValueError("an explicit nonempty serial port is required")
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError("baudrate must be a positive integer")
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
            raise ValueError("timeout_s must be a positive finite number")
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be a positive finite number")
        if serial_factory is None:
            # Import only on actual construction, so unit tests/ROS dry-run do
            # not require pyserial or touch hardware.
            import serial
            serial_factory = serial.Serial
        self.port = port
        self.timeout_s = float(timeout_s)
        self._transaction_lock = threading.Lock()
        self._last_tx_end = float('-inf')
        self._closed = False
        self._serial = serial_factory(
            port=port, baudrate=baudrate, bytesize=8, parity='N', stopbits=1,
            timeout=min(self.timeout_s, 0.02), write_timeout=self.timeout_s,
            xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True,
        )

    def exchange(self, request: bytes) -> Response:
        """Exactly one write, strict matched reply, bounded timeout, no resend.

        Returned Response always passed require_success(). A device-reported
        failure raises DeviceError; corrupt/wrong-address/old-command data raises
        ProtocolError. Timeout means execution is unknown, NOT 'did not move'.
        """
        address, command = _validate_request(request)
        request = bytes(request)
        if not self._transaction_lock.acquire(blocking=False):
            raise PD42TransportBusyError("PD42 serial transaction already pending")
        try:
            if self._closed:
                raise RuntimeError("PD42 serial transport is closed")
            # perf_counter is monotonic with sub-ms resolution on Windows as
            # well as Linux; Python 3.10's Windows monotonic may tick at 15 ms.
            delay = MIN_COMMAND_INTERVAL_S - (time.perf_counter() - self._last_tx_end)
            if delay > 0:
                time.sleep(delay)
            # Native PD42 replies only to requests. Discard old buffered replies
            # before a transaction; an old ACK must not confirm a new command.
            self._serial.reset_input_buffer()
            decoder = ResponseDecoder()
            try:
                written = self._serial.write(request)  # one contiguous write
                if written != len(request):
                    raise ProtocolError("partial PD42 write; outcome unknown, not retried")
                # A matched reply proves transmission. tcdrain()/flush() can
                # wait indefinitely in a disconnected Linux UART driver and
                # holds this transaction lock before the response deadline is
                # even started. The bounded write + reply timeout is sufficient
                # here, as in the ZDT transport; never resend an uncertain move.
            finally:
                self._last_tx_end = time.perf_counter()
            deadline = self._last_tx_end + self.timeout_s
            while True:
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    decoder.reset()
                    raise PD42TimeoutError(
                        f"PD42 address {address}, command 0x{command:02X} timeout; "
                        "execution is unknown and command was not retried"
                    )
                self._serial.timeout = min(remaining, 0.02)
                waiting = int(getattr(self._serial, 'in_waiting', 0))
                chunk = self._serial.read(min(max(waiting, 1), 256))
                if not chunk:
                    continue
                frames = decoder.feed(chunk)
                if decoder.malformed_frames or decoder.discarded_bytes:
                    raise ProtocolError("corrupt/unexpected bytes in PD42 transaction")
                if not frames:
                    continue
                if len(frames) != 1:
                    raise ProtocolError("multiple PD42 replies for one request")
                response = frames[0]
                if response.address != address or response.command != command:
                    raise ProtocolError("PD42 reply address/command does not match request")
                return response.require_success()
        finally:
            self._transaction_lock.release()

    def close(self) -> None:
        """Close without changing motor state; the owning controller stops it."""
        with self._transaction_lock:
            if not self._closed:
                self._serial.close()
                self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
