"""ALIENTEK PD42S1 V1.2 native serial protocol, with no hardware side effects.

Source: supplied user manual sections 4.1, 4.3.1/4/5/10-17 and 4.4.3/4/11-13.
This is NOT Modbus RTU. Send a whole request in one write, then leave at least
2 ms between complete requests. Do not broadcast on a shared motor bus.

Position units are 51200 counts per motor revolution, independent of the
external pulse-input microstep setting. A motor reading is not a calibrated
gimbal angle: installation zero, direction and gearing must be verified.
No function here homes, calibrates, resets, saves parameters, or opens a port.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct


HEAD = 0xC5
TAIL = 0x5C
SUCCESS = 0x01
COUNTS_PER_REVOLUTION = 51200
DEFAULT_BAUDRATE = 115200
MIN_COMMAND_INTERVAL_S = 0.002
MIN_POSITION_COUNTS = -(2**31)
MAX_POSITION_COUNTS = 2**31 - 1


def single_turn_counts(position_counts):
    """PD42 V1.2 p14: power loss discards turns, retains single-turn position.

    READ_POSITION (0x2A), also in READ_SYSTEM, uses 51200 counts/revolution.
    This is only a coordinate conversion of a successfully received sample;
    it neither reads hardware nor sends the separate F8 coordinate-clear.
    """
    if (type(position_counts) is not int or
            not MIN_POSITION_COUNTS <= position_counts <= MAX_POSITION_COUNTS):
        raise ValueError('PD42 position must be a successfully read signed int32')
    return position_counts % COUNTS_PER_REVOLUTION


class Command(IntEnum):
    READ_VERSION = 0x20
    READ_CURRENT = 0x23
    READ_VOLTAGE = 0x24
    READ_SPEED = 0x29
    READ_POSITION = 0x2A
    READ_POSITION_ERROR = 0x2B
    READ_STATUS = 0x2C
    READ_STALL = 0x2D
    READ_ENABLED = 0x2F
    READ_ARRIVED = 0x30
    READ_SYSTEM = 0x31
    VELOCITY = 0xF1
    ABSOLUTE_POSITION = 0xF2
    RELATIVE_POSITION = 0xF3
    ENABLE = 0xFA
    CLEAR_STATUS = 0xFB
    STOP = 0xFC


class RunState(IntEnum):
    STOPPED = 0
    COMPLETED = 1
    RUNNING = 2
    OVERLOAD = 3
    STALLED = 4
    UNDERVOLTAGE = 5


ERROR_CODES = {
    0xE1: "frame too short",
    0xE2: "invalid header",
    0xE3: "invalid footer",
    0xE4: "checksum mismatch",
    0xE5: "unsupported command",
    0xE6: "invalid command data",
}

# Counts exclude the first response payload byte (the operation status).
_RESPONSE_DATA_SIZES = {
    Command.READ_VERSION: 2,
    Command.READ_CURRENT: 2,
    Command.READ_VOLTAGE: 4,
    Command.READ_SPEED: 2,
    Command.READ_POSITION: 4,
    Command.READ_POSITION_ERROR: 4,
    Command.READ_STATUS: 1,
    Command.READ_STALL: 1,
    Command.READ_ENABLED: 1,
    Command.READ_ARRIVED: 1,
    Command.READ_SYSTEM: 40,
    Command.VELOCITY: 0,
    Command.ABSOLUTE_POSITION: 0,
    Command.RELATIVE_POSITION: 0,
    Command.ENABLE: 1,
    Command.CLEAR_STATUS: 0,
    Command.STOP: 0,
}
_READ_COMMANDS = frozenset(c for c in Command if c.value < 0xF0)


class ProtocolError(ValueError):
    """Malformed or unexpected serial data; do not use it as feedback."""


class DeviceError(ProtocolError):
    """A valid PD42 response explicitly rejected a command."""

    def __init__(self, response: "Response"):
        self.response = response
        super().__init__(
            f"PD42 address {response.address}, command 0x{response.command:02X}: "
            f"{ERROR_CODES.get(response.status, 'unknown status')} "
            f"(0x{response.status:02X})"
        )


@dataclass(frozen=True)
class Response:
    address: int
    command: Command
    status: int
    data: bytes
    value: int | float | bool | dict | None

    @property
    def ok(self) -> bool:
        return self.status == SUCCESS

    def require_success(self) -> "Response":
        if not self.ok:
            raise DeviceError(self)
        return self


def _integer(value: int, name: str, low: int, high: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if not low <= value <= high:
        raise ValueError(f"{name} must be in [{low}, {high}]")
    return value


def _address(address: int) -> int:
    # A broadcast has no response and can move both axes unintentionally.
    return _integer(address, "unicast address", 1, 255)


def _command(command: int | Command) -> Command:
    if isinstance(command, bool) or not isinstance(command, int):
        raise ValueError("command must be a supported Command")
    try:
        return Command(command)
    except (ValueError, TypeError) as exc:
        raise ValueError("unsupported PD42 command") from exc


def _request(address: int, command: Command, payload: bytes = b"") -> bytes:
    body = bytes((HEAD, _address(address), command)) + payload
    return body + bytes((sum(body) & 0xFF, TAIL))


def read_request(address: int, command: int | Command) -> bytes:
    command = _command(command)
    if command not in _READ_COMMANDS:
        raise ValueError("read_request accepts read-only commands")
    return _request(address, command)


def position_request(address: int, target_counts: int, speed_rpm: int,
                     acceleration: int, *, relative: bool = False) -> bytes:
    """Build one 13-byte F2/F3 request, using signed target/delta counts.

    Positive means protocol direction 0; negative means direction 1. For F2
    this describes the target's sign relative to the verified origin, not the
    direction from the current position. Reject outside signed feedback range
    instead of silently wrapping/clamping. Velocity is motor RPM, acceleration
    is the device's r/s^2 parameter. Zero acceleration starts immediately.

    The caller MUST impose calibrated axis travel/velocity limits and a fresh
    feedback gate. An acknowledgement means accepted, never target reached.
    F3 is non-idempotent: never retry after an uncertain acknowledgement.
    """
    _integer(target_counts, "target_counts", MIN_POSITION_COUNTS, MAX_POSITION_COUNTS)
    _integer(speed_rpm, "speed_rpm", 0, 6000)
    _integer(acceleration, "acceleration", 0, 200)
    if not isinstance(relative, bool):
        raise ValueError("relative must be bool")
    command = Command.RELATIVE_POSITION if relative else Command.ABSOLUTE_POSITION
    payload = struct.pack(">BBHI", int(target_counts < 0), acceleration,
                          speed_rpm, abs(target_counts))
    return _request(address, command, payload)


def enable_request(address: int, enabled: bool) -> bytes:
    """FA uses 0=enabled and 1=disabled. Disabling removes holding torque."""
    if not isinstance(enabled, bool):
        raise ValueError("enabled must be bool")
    return _request(address, Command.ENABLE, bytes((0 if enabled else 1,)))


def velocity_request(address: int, rpm: float, acceleration: int = 10) -> bytes:
    """Native F1: direction, acceleration, big-endian float32 RPM.

    Manual section 4.4.2 and supplied smd_speed_mode() agree on this layout.
    Signed RPM chooses protocol direction; zero is not a documented F1 stop.
    Caller must enforce its run lease and separately verify actual feedback.
    """
    if isinstance(rpm, bool) or not isinstance(rpm, (int, float)) or not math.isfinite(rpm):
        raise ValueError('rpm must be finite')
    if not .1 <= abs(rpm) <= 6000:
        raise ValueError('absolute rpm must be in [0.1, 6000]')
    _integer(acceleration, 'acceleration', 0, 200)
    return _request(address, Command.VELOCITY,
                    struct.pack('>BBf', int(rpm < 0), acceleration, abs(rpm)))


def stop_request(address: int) -> bytes:
    """FC latches braking; prolonged braking can overheat the motor.

    This is not a normal holding mode. Recovery requires an explicitly planned
    mechanical-safe action; never blindly clear status or auto-resume F2.
    """
    return _request(address, Command.STOP)


def clear_status_request(address: int) -> bytes:
    """Explicit operator recovery only: FB also clears disabled/stall state."""
    return _request(address, Command.CLEAR_STATUS)


def counts_to_degrees(counts: int) -> float:
    """Motor-shaft angle relative to the device origin, not gimbal angle."""
    _integer(counts, "counts", MIN_POSITION_COUNTS, MAX_POSITION_COUNTS)
    return counts * 360.0 / COUNTS_PER_REVOLUTION


def degrees_to_counts(degrees: float) -> int:
    """Quantize a motor-shaft angle to the nearest count, with no wrapping."""
    if isinstance(degrees, bool) or not isinstance(degrees, (int, float)):
        raise ValueError("degrees must be a finite number")
    try:
        degrees = float(degrees)
    except OverflowError as exc:
        raise ValueError("degrees outside representable range") from exc
    if not math.isfinite(degrees):
        raise ValueError("degrees must be finite")
    scaled = degrees * COUNTS_PER_REVOLUTION / 360.0
    if not math.isfinite(scaled):
        raise ValueError("converted counts outside representable range")
    counts = round(scaled)
    return _integer(counts, "converted counts", MIN_POSITION_COUNTS, MAX_POSITION_COUNTS)


def response_size(command: int | Command) -> int:
    """Exact successful reply size; short error replies can be six bytes."""
    return 6 + _RESPONSE_DATA_SIZES[_command(command)]


response_frame_size = response_size


def _decode(command: Command, data: bytes):
    if command == Command.READ_SYSTEM:
        # V1.2 section 4.3.18, PDF pp.43-44: these 40 bytes exclude
        # the operation-status byte. In particular, target is NOT error.
        values = struct.unpack('>fhfffhiiiIBBBB', data)
        if (any(not math.isfinite(v) for v in (values[0], *values[2:5])) or
                values[0] < 0 or any(flag not in (0, 1) for flag in values[10:])):
            raise ProtocolError('invalid PD42 system feedback')
        keys = ('voltage_v', 'current_ma', 'flux_mwb', 'resistance_ohm',
                'inductance_mh', 'speed_rpm', 'target_counts', 'position_counts',
                'position_error_counts', 'pulse_count')
        result = dict(zip(keys, values[:10]))
        result.update(enabled=values[10] == 0, arrived=bool(values[11]),
                      stalled=bool(values[12]), group_address=bool(values[13]))
        return result
    if command in (Command.READ_POSITION, Command.READ_POSITION_ERROR):
        return struct.unpack(">i", data)[0]
    if command in (Command.READ_SPEED, Command.READ_CURRENT):
        return struct.unpack(">h", data)[0]
    if command == Command.READ_VOLTAGE:
        voltage = struct.unpack(">f", data)[0]
        if not math.isfinite(voltage) or voltage < 0:
            raise ProtocolError("invalid voltage feedback")
        return voltage
    if command == Command.READ_STATUS:
        try:
            return RunState(data[0])
        except ValueError as exc:
            raise ProtocolError("unknown run-state byte") from exc
    if command in (Command.READ_ENABLED, Command.ENABLE, Command.READ_STALL,
                   Command.READ_ARRIVED):
        if data[0] not in (0, 1):
            raise ProtocolError("invalid boolean feedback byte")
        # XLSX READ_ENABLED labels conflict with PDF definition + STM32 code;
        # use 0=enabled, retain data for commissioning (see sources report).
        return data[0] == 0 if command in (Command.READ_ENABLED, Command.ENABLE) else bool(data[0])
    if command == Command.READ_VERSION:
        return {"firmware_raw": data[0], "hardware_raw": data[1],
                "firmware": data[0] / 10.0, "hardware": data[1] / 10.0}
    return None


def parse_response(frame: bytes, expected_address: int | None = None,
                   expected_command: int | Command | None = None) -> Response:
    """Parse one complete frame, rejecting corruption and mismatched replies.

    A valid negative reply has .ok=False and .value=None; call
    .require_success() before treating it as an ACK. Error data is not feedback.
    The protocol has no transaction ID: transport must serialize requests and
    must never turn stale ACKs into evidence of a newly completed movement.
    """
    if not isinstance(frame, (bytes, bytearray, memoryview)):
        raise ProtocolError("response must be bytes")
    frame = bytes(frame)
    if len(frame) < 6:
        raise ProtocolError("response too short")
    if frame[0] != HEAD or frame[-1] != TAIL:
        raise ProtocolError("invalid frame delimiters")
    if (sum(frame[:-2]) & 0xFF) != frame[-2]:
        raise ProtocolError("response checksum mismatch")
    try:
        address = _address(frame[1])
        command = _command(frame[2])
    except ValueError as exc:
        raise ProtocolError(str(exc)) from exc
    if expected_address is not None and address != _address(expected_address):
        raise ProtocolError("response address does not match request")
    if expected_command is not None and command != _command(expected_command):
        raise ProtocolError("response command does not match request")
    status = frame[3]
    if status != SUCCESS and status not in ERROR_CODES:
        raise ProtocolError("unknown operation status")
    data = frame[4:-2]
    data_size = _RESPONSE_DATA_SIZES[command]
    if status == SUCCESS:
        if len(data) != data_size:
            raise ProtocolError(f"expected {data_size} response data bytes, got {len(data)}")
        value = _decode(command, data)
    else:
        # Manufacturer lists status in each reply but does not specify whether
        # failed reads keep their full payload. Permit only short/full lengths;
        # never decode failure padding as a valid position or ready flag.
        if len(data) not in (0, data_size):
            raise ProtocolError("invalid error response length")
        value = None
    return Response(address, command, status, data, value)


class ResponseDecoder:
    """Incremental bounded decoder for arbitrary serial read chunk sizes.

    Uses function-specific lengths, never the first 0x5C byte: payloads may
    contain header/footer bytes. Call reset() after a transaction timeout before
    issuing a new request; it cannot identify stale frames (no wire sequence ID).
    """

    def __init__(self):
        self._buffer = bytearray()
        self.discarded_bytes = 0
        self.malformed_frames = 0

    def reset(self) -> None:
        self.discarded_bytes += len(self._buffer)
        self._buffer.clear()

    def feed(self, chunk: bytes) -> list[Response]:
        if not isinstance(chunk, (bytes, bytearray, memoryview)):
            raise TypeError("serial chunk must be bytes")
        self._buffer.extend(chunk)
        result = []
        while self._buffer:
            if self._buffer[0] != HEAD:
                try:
                    offset = self._buffer.index(HEAD)
                except ValueError:
                    offset = len(self._buffer)
                del self._buffer[:offset]
                self.discarded_bytes += offset
                continue
            if len(self._buffer) < 4:
                break
            try:
                length = response_size(self._buffer[2])
                _address(self._buffer[1])
            except ValueError:
                del self._buffer[0]
                self.discarded_bytes += 1
                self.malformed_frames += 1
                continue
            status = self._buffer[3]
            if status not in (SUCCESS, *ERROR_CODES):
                del self._buffer[0]
                self.discarded_bytes += 1
                self.malformed_frames += 1
                continue
            lengths = (6, length) if status != SUCCESS and length != 6 else (length,)
            parsed = None
            for size in lengths:
                if len(self._buffer) >= size:
                    try:
                        parsed = parse_response(self._buffer[:size])
                    except ProtocolError:
                        continue
                    del self._buffer[:size]
                    break
            if parsed is not None:
                result.append(parsed)
            elif len(self._buffer) < max(lengths):
                break
            else:
                del self._buffer[0]
                self.discarded_bytes += 1
                self.malformed_frames += 1
        return result
