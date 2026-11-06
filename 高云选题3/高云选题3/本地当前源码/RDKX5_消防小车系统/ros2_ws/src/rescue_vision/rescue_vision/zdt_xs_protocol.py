"""ZDT X42S/Y42 second-generation protocol, fixed 0x6B checksum only.

Source: supplied V1.0.5 manual, pages 40, 48, 50, 54, 57, 60, 67, 69-70,
71-74, 77-78, 99-106. The supplied Pico Python example confirms Emm frames
but omits the READ_CONFIG auxiliary byte; the manual takes precedence.

Emm command positions are microstep pulses, while its feedback uses 65536
counts/revolution. X command positions use 0.1 or 0.01 degree; X position
feedback ALWAYS uses 0.1 degree and position error uses 0.01 degree.
This module has no hardware side effects and never changes driver settings.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
import struct


TAIL = 0x6B
DEFAULT_BAUDRATE = 115200
SUCCESS = 0x02
ARRIVED_EVENT = 0x9F


class FirmwareMode(str, Enum):
    EMM = 'emm'
    X = 'x'


class Command(IntEnum):
    RESET_CURRENT_POSITION = 0x0A
    READ_OPTIONS = 0x1A
    READ_VERSION = 0x1F
    READ_VOLTAGE = 0x24
    READ_ENCODER = 0x31
    READ_TARGET_POSITION = 0x33
    READ_REALTIME_TARGET_POSITION = 0x34
    READ_SPEED = 0x35
    READ_POSITION = 0x36
    READ_POSITION_ERROR = 0x37
    READ_STATUS = 0x3A
    READ_STATE = 0x3A
    READ_CONFIG = 0x42
    SELECT_FIRMWARE = 0xD5
    ENABLE = 0xF3
    VELOCITY = 0xF6
    POSITION_X = 0xFB
    POSITION_EMM = 0xFD
    STOP = 0xFE


CONTROL_COMMANDS = frozenset(c for c in Command if c >= 0xF0 or c in
                             (Command.SELECT_FIRMWARE, Command.RESET_CURRENT_POSITION))
READ_COMMANDS = frozenset(c for c in Command if c not in CONTROL_COMMANDS)
RESPONSE_LENGTHS = {
    Command.READ_VERSION: 7, Command.READ_VOLTAGE: 5, Command.READ_ENCODER: 5,
    Command.READ_TARGET_POSITION: 8, Command.READ_REALTIME_TARGET_POSITION: 8,
    Command.READ_SPEED: 6, Command.READ_POSITION: 8,
    Command.READ_POSITION_ERROR: 8, Command.READ_STATUS: 4,
    Command.SELECT_FIRMWARE: 4, Command.RESET_CURRENT_POSITION: 4,
    Command.ENABLE: 4, Command.POSITION_X: 4,
    Command.POSITION_EMM: 4, Command.STOP: 4, Command.VELOCITY: 4,
}
ACK_MODES = ('None', 'Receive', 'Reached', 'Both', 'Other')
BAUDRATES = (9600, 19200, 25000, 38400, 57600, 115200, 256000, 512000, 921600)


class ProtocolError(ValueError):
    """Malformed, unsupported or ambiguous data; never motion permission."""


class DeviceError(ProtocolError):
    def __init__(self, response):
        self.response = response
        label = {0xE2: 'invalid parameters or driver fault',
                 0xEE: 'invalid command format',
                 0x12: 'zero/limit already active; no motion',
                 0x22: 'zero/limit already active; no motion'}.get(response.status, 'not an acknowledgement')
        super().__init__(f'ZDT address {response.address}, command 0x{response.command:02X}: '
                         f'{label} (0x{response.status:02X})')


@dataclass(frozen=True)
class Response:
    address: int
    command: Command
    data: bytes
    value: object
    status: int | None = None

    @property
    def ok(self):
        return self.status is None or self.status == SUCCESS

    @property
    def is_arrival_event(self):
        return self.command in (Command.POSITION_X, Command.POSITION_EMM) and self.status == ARRIVED_EVENT

    def require_success(self):
        if not self.ok:
            raise DeviceError(self)
        return self


def _integer(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ProtocolError(f'{name} must be an integer in {low}..{high}')
    return value


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProtocolError(f'{name} must be finite')
    return float(value)


def _mode(mode):
    try:
        return FirmwareMode(mode)
    except (ValueError, TypeError) as exc:
        raise ProtocolError('firmware mode must be explicitly identified as emm or x') from exc


def _address(address):
    return _integer(address, 1, 255, 'unicast address')


def _flag(flag, name):
    if not isinstance(flag, bool):
        raise ProtocolError(f'{name} must be an explicitly known boolean')
    return flag


def read_request(address, command):
    _address(address)
    try:
        command = Command(command)
    except (ValueError, TypeError) as exc:
        raise ProtocolError('unsupported read command') from exc
    if command not in READ_COMMANDS:
        raise ProtocolError('not a read command')
    return bytes((address, command, 0x6C, TAIL)) if command == Command.READ_CONFIG else bytes((address, command, TAIL))


def enable_request(address, enabled):
    return bytes((_address(address), Command.ENABLE, 0xAB, int(_flag(enabled, 'enabled')), 0, TAIL))


def select_firmware_request(address, mode, *, persist=False):
    """Encode vendor V1.0.5 section 5.6.6 (p78); no I/O or automatic use.

    00/01/02 selects X/Emm/Emm Turbo. Default store=00 leaves EEPROM alone.
    Caller must first verify a stopped motor and later re-read configuration
    and options; an ACK does not verify the new mode or authorize movement.
    This is not a fault reset, calibration, homing or enable instruction.
    """
    modes = {'x': 0, 'emm': 1, 'emm_turbo': 2}
    if not isinstance(mode, str) or mode not in modes:
        raise ProtocolError('firmware selection must be x, emm or emm_turbo')
    return bytes((_address(address), Command.SELECT_FIRMWARE, 0x69,
                  int(_flag(persist, 'persist')), modes[mode], TAIL))


def zero_current_position_request(address):
    """Explicit coordinate reset from vendor 5.2.3 p48, not physical homing.

    No automatic use. Caller must preserve the old coordinate and invalidate
    any installation reference. Read position and both retained targets after
    the ACK; this instruction alone does not prove that old targets cleared.
    It does not calibrate, clear protection, restore defaults or enable motion.
    """
    return bytes((_address(address), Command.RESET_CURRENT_POSITION, 0x6D, TAIL))


def stop_request(address):
    """Immediate stop, no disable, zero reset, synchronization or restart."""
    return bytes((_address(address), Command.STOP, 0x98, 0, TAIL))


def position_request(address, target_degrees, rpm, *, mode, motion_mode=2,
                     microsteps=None, motor_step_deg=None, speed_div10=None,
                     angle_div10=None, acceleration=100):
    """Encode a motor position, with explicitly measured/configured units.

    motion_mode: 0 previous command target, 1 coordinate zero, 2 current position.
    Emm acceleration is a 0..255 selector; X uses the direct speed-limited FB
    command (no acceleration field). Mechanical limits belong to the caller.
    Never equate an ACK with measured target arrival.
    """
    address = _address(address)
    mode = _mode(mode)
    motion_mode = _integer(motion_mode, 0, 2, 'motion mode')
    angle = _finite(target_degrees, 'target_degrees')
    rpm = _finite(rpm, 'rpm')
    if not 0 < rpm <= 3000:
        raise ProtocolError('position speed must be >0 and <=3000 RPM')
    direction = int(angle < 0)
    if mode == FirmwareMode.EMM:
        _integer(microsteps, 1, 256, 'microsteps')
        if isinstance(motor_step_deg, bool) or motor_step_deg not in (1.8, 0.9):
            raise ProtocolError('Emm motor step must be verified as 1.8 or 0.9 degrees')
        _flag(speed_div10, 'Emm speed_div10')
        acceleration = _integer(acceleration, 0, 255, 'Emm acceleration')
        velocity = int(math.floor(rpm * (10 if speed_div10 else 1) + 0.5))
        # The supplied standard FD table explicitly limits encoded velocity to 0x0BB8.
        _integer(velocity, 1, 3000, 'Emm encoded velocity')
        magnitude = int(math.floor(abs(angle) * microsteps / motor_step_deg + 0.5))
        _integer(magnitude, 0, 0xFFFFFFFF, 'Emm pulse magnitude')
        return bytes((address, Command.POSITION_EMM, direction)) + struct.pack('>HBI', velocity, acceleration, magnitude) + bytes((motion_mode, 0, TAIL))
    _flag(angle_div10, 'X angle_div10')
    velocity = int(math.floor(rpm * 10 + 0.5))
    _integer(velocity, 1, 30000, 'X encoded velocity')
    magnitude = int(math.floor(abs(angle) * (100 if angle_div10 else 10) + 0.5))
    _integer(magnitude, 0, 0xFFFFFFFF, 'X angle magnitude')
    return bytes((address, Command.POSITION_X, direction)) + struct.pack('>HI', velocity, magnitude) + bytes((motion_mode, 0, TAIL))


def position_degrees(raw, mode):
    _integer(raw, -0xFFFFFFFF, 0xFFFFFFFF, 'position feedback')
    return raw * 360.0 / 65536 if _mode(mode) == FirmwareMode.EMM else raw / 10.0


def velocity_request(address, rpm, *, mode, speed_div10=None, acceleration=10):
    """Manual 5.3.5/5.3.7: X and Emm share F6 but have different layouts.

    Emm: dir, speed u16, acceleration u8; X: dir, acceleration u16 RPM/s,
    speed u16 in 0.1 RPM. No synchronization buffering or persistent writes.
    """
    address = _address(address)
    mode = _mode(mode)
    rpm = _finite(rpm, 'rpm')
    if not 0 < abs(rpm) <= 3000:
        raise ProtocolError('absolute velocity must be >0 and <=3000 RPM')
    direction = int(rpm < 0)
    if mode == FirmwareMode.EMM:
        _flag(speed_div10, 'Emm speed_div10')
        acceleration = _integer(acceleration, 0, 255, 'Emm acceleration')
        velocity = int(math.floor(abs(rpm) * (10 if speed_div10 else 1) + .5))
        _integer(velocity, 1, 3000, 'Emm encoded velocity')
        data = struct.pack('>BHB', direction, velocity, acceleration)
    else:
        acceleration = _integer(acceleration, 0, 65535, 'X acceleration RPM/s')
        velocity = int(math.floor(abs(rpm) * 10 + .5))
        _integer(velocity, 1, 30000, 'X encoded velocity')
        data = struct.pack('>BHH', direction, acceleration, velocity)
    return bytes((address, Command.VELOCITY)) + data + bytes((0, TAIL))


def error_degrees(raw, mode):
    _integer(raw, -0xFFFFFFFF, 0xFFFFFFFF, 'position error feedback')
    return raw * 360.0 / 65536 if _mode(mode) == FirmwareMode.EMM else raw / 100.0


def speed_rpm(raw, mode):
    _integer(raw, -65535, 65535, 'speed feedback')
    return float(raw) if _mode(mode) == FirmwareMode.EMM else raw / 10.0


def _signed(data):
    if data[0] not in (0, 1):
        raise ProtocolError('invalid sign byte')
    return int.from_bytes(data[1:], 'big') * (-1 if data[0] else 1)


def _decode_config(frame):
    length, count = frame[2:4]
    if (length, count) == (33, 21):
        mode = 'emm'
        if frame[4] not in (0x19, 0x32):
            raise ProtocolError('unknown Emm motor type')
        step = 1.8 if frame[4] == 0x19 else 0.9
        micro, uart, baud, checksum, ack = frame[9], frame[6], frame[18], frame[21], frame[22]
        angle_scale = None
        pulse_mode, direction = frame[5], frame[8]
        closed_loop = True if pulse_mode == 2 else False if pulse_mode == 1 else None
        driver_address = frame[20]
        if driver_address != frame[0]:
            raise ProtocolError('configuration address conflicts with response address')
    elif (length, count) == (37, 24):
        mode, step, driver_address = 'x', None, frame[0]
        micro, uart, baud, checksum, ack = frame[10], frame[7], frame[22], frame[24], frame[25]
        if frame[26] not in (0, 1):
            raise ProtocolError('unknown X angle scaling')
        angle_scale = bool(frame[26])
        pulse_mode, direction = frame[6], frame[9]
        closed_loop = bool(frame[5]) if frame[5] in (0, 1) else None
    else:
        raise ProtocolError('unknown configuration layout; cannot determine motor units')
    if uart not in range(5) or baud not in range(len(BAUDRATES)) or checksum not in range(5) or ack not in range(5):
        raise ProtocolError('unknown driver configuration enum')
    return dict(mode=mode, microsteps=micro or 256, motor_step_deg=step,
                speed_div10=None, angle_div10=angle_scale, ack_mode=ack,
                ack_mode_name=ACK_MODES[ack], checksum_mode=checksum,
                uart_mode=uart, baudrate=BAUDRATES[baud], address=driver_address,
                pulse_mode=pulse_mode, closed_loop=closed_loop,
                positive_direction_ccw=bool(direction) if direction in (0, 1) else None,
                position_window_deg=int.from_bytes(frame[-3:-1], 'big') / 10.0)


def normalize_options(version, config, options):
    """Validate one explicitly observed native-u16 layout; no protocol guessing.

    Base parsing intentionally leaves all five-byte replies unknown. This
    compatibility profile is limited to firmware 2.0.6 / hardware raw 0x2314
    running Emm. Evidence: vendor Modbus manual V1.0.1 p39 defines the same
    0x1A quantity as a high-byte-first 16-bit register; native manual V1.0.5
    p77 defines bits 0..9; this device returned 01 1A 00 06 6B with an
    independent 33-byte config agreeing on firmware, step, closed-loop and
    direction. Native u16 byte order is therefore cross-validated here, NOT
    claimed to be explicitly specified by the native manual for all firmware.

    Unrecognized/conflicting input remains read-only with a reason and raw
    bytes. This does not enable a motor or bypass ACK, fault, zero or limit
    checks. Call again with fresh version/config/options before each action.

    Firmware 2.0.6 has also returned 04 06. Neither supplied manual nor the
    2026-09-28 release notes defines bit 10. Report that bit explicitly, but
    do not infer that it is harmless or authorize motion by masking it out.
    """
    if not all(isinstance(v, dict) for v in (version, config, options)):
        raise ProtocolError('version, configuration and options readbacks required')
    try:
        data = bytes.fromhex(options.get('raw_bytes_hex', ''))
    except (ValueError, TypeError) as exc:
        raise ProtocolError('invalid raw option bytes') from exc
    if len(data) == 1:
        return dict(options)  # Documented legacy layout remains unchanged.
    result = dict(options)
    result.update(raw=None, layout_verified=False, mode=None, motor_step_deg=None,
                  closed_loop=None, scaled_input=None, positive_direction_ccw=None,
                  parameter_lock_level=None)
    result.pop('layout_profile', None)
    result.pop('verification_evidence', None)
    for field in ('raw_u16be', 'documented_bits_raw', 'undocumented_bits_raw',
                  'undocumented_bit_indices', 'layout_verification_detail'):
        result.pop(field, None)

    def reject(reason):
        result['layout_verification_reason'] = reason
        return result

    if len(data) != 2:
        return reject('extended_options_unverified_length')
    if (version.get('firmware_raw'), version.get('hardware_raw')) != (206, 0x2314):
        return reject('extended_options_unverified_device_version')
    if config.get('mode') != 'emm':
        return reject('extended_options_unverified_config_mode')
    raw = int.from_bytes(data, 'big')
    # Defined bits: motor(0), firmware(1), loop(2), direction(4), key lock(5),
    # input scale(7), parameter lock level(8..9). Do not ignore reserved bits.
    undocumented = raw & ~0x3B7
    result.update(raw_u16be=raw, documented_bits_raw=raw & 0x3B7,
                  undocumented_bits_raw=undocumented,
                  undocumented_bit_indices=[bit for bit in range(16)
                                            if undocumented & (1 << bit)])
    if undocumented:
        result['layout_verification_detail'] = (
            'OPTIONS 0x%04X has undocumented bit(s) %s (mask 0x%04X); '
            'vendor native V1.0.5 p77 and Modbus V1.0.1 p39 define only '
            'bits 0,1,2,4,5,7,8,9. Motion remains unverified.' %
            (raw, ','.join(map(str, result['undocumented_bit_indices'])), undocumented))
        return reject('extended_options_reserved_bits_set')
    mode = 'emm' if raw & 2 else 'x'
    step = .9 if raw & 1 else 1.8
    closed_loop = bool(raw & 4)
    direction = bool(raw & 0x10)
    if mode != config.get('mode'):
        return reject('extended_options_firmware_mode_conflict')
    if step != config.get('motor_step_deg'):
        return reject('extended_options_motor_step_conflict')
    if not isinstance(config.get('closed_loop'), bool) or closed_loop != config['closed_loop']:
        return reject('extended_options_closed_loop_unverified_or_conflicting')
    if not isinstance(config.get('positive_direction_ccw'), bool) or direction != config['positive_direction_ccw']:
        return reject('extended_options_direction_unverified_or_conflicting')
    result.update(raw=raw, layout_verified=True, mode=mode, motor_step_deg=step,
                  closed_loop=closed_loop, scaled_input=bool(raw & 0x80),
                  positive_direction_ccw=direction, parameter_lock_level=(raw >> 8) & 3,
                  layout_profile='zdt_xs_fw206_hw2314_emm_u16be',
                  layout_verification_reason='', verification_evidence={
                      'firmware_raw': 206, 'hardware_raw': 0x2314,
                      'register_reference': 'Vendor Modbus V1.0.1 section 3.5.4, page 39: high byte then low byte',
                      'bit_reference': 'Vendor native V1.0.5 section 5.6.4, page 77: bits 0..9',
                      'observed_native_response': '01 1a 00 06 6b',
                      'cross_checks': ['firmware_mode', 'motor_step_deg', 'closed_loop', 'positive_direction_ccw'],
                      'scope': 'Observed firmware/hardware Emm profile only; not a universal native-u16 specification',
                  })
    return result


def verified_motion_parameters(config, options):
    """Combine two fresh driver reads; refuse ambiguous or non-ACK settings.

    Does not write configuration. Caller must revalidate after reconnection or
    any change at the driver. This says nothing about physical limits/zero.
    """
    if not isinstance(config, dict) or not isinstance(options, dict):
        raise ProtocolError('configuration and option readbacks are required')
    if options.get('layout_verified') is False:
        detail = options.get('layout_verification_detail') or options.get('layout_verification_reason')
        raise ProtocolError('option layout not verified; input scaling unknown' +
                            (': ' + str(detail) if detail else ''))
    mode = _mode(config.get('mode'))
    if options.get('mode') != mode.value:
        raise ProtocolError('firmware mode conflicts between config and options')
    if config.get('checksum_mode') != 0 or config.get('uart_mode') != 2:
        raise ProtocolError('fixed 6B UART mode required for motion')
    if config.get('ack_mode') not in (1, 3):
        raise ProtocolError('Receive or Both acknowledgement mode required; driver was not changed')
    if options.get('closed_loop') is not True:
        raise ProtocolError('closed-loop mode must be reported by driver')
    scaled = _flag(options.get('scaled_input'), 'input scaling')
    step = options.get('motor_step_deg')
    if mode == FirmwareMode.EMM and step != config.get('motor_step_deg'):
        raise ProtocolError('motor step conflicts between config and options')
    if mode == FirmwareMode.X and scaled != config.get('angle_div10'):
        raise ProtocolError('X angle scaling conflicts between config and options')
    return dict(mode=mode.value, microsteps=config['microsteps'], motor_step_deg=step,
                speed_div10=scaled if mode == FirmwareMode.EMM else None,
                angle_div10=scaled if mode == FirmwareMode.X else None)


def parse_response(frame, expected_address=None, expected_command=None):
    if not isinstance(frame, (bytes, bytearray, memoryview)):
        raise ProtocolError('response must be bytes')
    frame = bytes(frame)
    if len(frame) < 4 or frame[-1] != TAIL:
        raise ProtocolError('invalid response length/fixed checksum')
    address = _address(frame[0])
    try:
        command = Command(frame[1])
    except ValueError as exc:
        raise ProtocolError('unsupported response command') from exc
    if expected_address is not None and address != expected_address:
        raise ProtocolError('response address mismatch')
    if expected_command is not None and command != expected_command:
        raise ProtocolError('response command mismatch')
    data = frame[2:-1]
    if command in CONTROL_COMMANDS:
        if len(frame) != 4 or data[0] not in (SUCCESS, ARRIVED_EVENT, 0xE2, 0xEE, 0x12, 0x22):
            raise ProtocolError('invalid control response')
        if data[0] == ARRIVED_EVENT and command not in (Command.POSITION_X, Command.POSITION_EMM):
            raise ProtocolError('unexpected arrival event')
        return Response(address, command, data, None, data[0])
    # Reads carry data, not a leading success byte. Do not mistake status bits
    # 0xE2/0xEE in the four-byte READ_STATUS/OPTIONS reply for a device error.
    if command not in (Command.READ_STATUS, Command.READ_OPTIONS) and len(frame) == 4 and data[0] in (0xE2, 0xEE):
        return Response(address, command, data, None, data[0])
    if command == Command.READ_CONFIG:
        if len(frame) not in (33, 37) or frame[2] != len(frame):
            raise ProtocolError('configuration declared/actual length mismatch')
        value = _decode_config(frame)
    elif command == Command.READ_OPTIONS:
        # Manual p77 table has 8 bits; new text adds bits 8-9 without byte
        # order. Preserve a two-byte reply for diagnosis, never guess its units.
        if len(frame) not in (4, 5):
            raise ProtocolError('unsupported options length')
        if len(frame) == 5:
            value = dict(raw=None, raw_bytes_hex=data.hex(' '), layout_verified=False,
                         mode=None, motor_step_deg=None, closed_loop=None,
                         scaled_input=None, positive_direction_ccw=None,
                         parameter_lock_level=None)
            return Response(address, command, data, value)
        raw = int.from_bytes(data, 'big')
        value = dict(raw=raw, raw_bytes_hex=data.hex(' '), layout_verified=True,
                     mode='emm' if raw & 2 else 'x',
                     motor_step_deg=0.9 if raw & 1 else 1.8,
                     closed_loop=bool(raw & 4), scaled_input=bool(raw & 0x80),
                     positive_direction_ccw=bool(raw & 0x10),
                     parameter_lock_level=None)
    else:
        if len(frame) != RESPONSE_LENGTHS[command]:
            raise ProtocolError('response length mismatch')
        if command == Command.READ_VERSION:
            firmware = int.from_bytes(data[:2], 'big')
            hardware = int.from_bytes(data[2:4], 'big')
            value = dict(firmware_raw=firmware,
                         firmware=f'{firmware // 100}.{firmware // 10 % 10}.{firmware % 10}',
                         hardware_raw=hardware, hardware_series=(hardware >> 12) & 15,
                         hardware_type=(hardware >> 8) & 15, hardware_version_raw=hardware & 255)
        elif command == Command.READ_VOLTAGE:
            value = int.from_bytes(data, 'big') / 1000.0
        elif command == Command.READ_ENCODER:
            # V1.0.5 section 5.5.7 (PDF p69): calibrated single-turn absolute
            # encoder, uint16 0..65535. Unlike 0x36 this is not the boot origin.
            value = int.from_bytes(data, 'big')
        elif command in (Command.READ_POSITION, Command.READ_POSITION_ERROR, Command.READ_SPEED,
                         Command.READ_TARGET_POSITION, Command.READ_REALTIME_TARGET_POSITION):
            # Manual 5.5.9/10: both target reads use signed position feedback
            # units (Emm 65536/rev; X 0.1 deg), never command microstep pulses.
            value = _signed(data)
        elif command == Command.READ_STATUS:
            raw = data[0]
            value = dict(raw=raw, enabled=bool(raw & 1), arrived=bool(raw & 2),
                         stalled=bool(raw & 4), stall_protection=bool(raw & 8),
                         left_limit=bool(raw & 0x10), right_limit=bool(raw & 0x20),
                         power_cycle_marker=bool(raw & 0x80))
        else:
            raise ProtocolError('unsupported response')
    return Response(address, command, data, value)


def validate_request(request):
    """Reject broadcast, homing and unsupported writes; reset/D5 are explicit."""
    if not isinstance(request, (bytes, bytearray, memoryview)):
        raise ProtocolError('request must be bytes')
    request = bytes(request)
    if len(request) < 3 or request[-1] != TAIL:
        raise ProtocolError('invalid request length/fixed checksum')
    address = _address(request[0])
    try:
        command = Command(request[1])
    except ValueError as exc:
        raise ProtocolError('unsupported request') from exc
    if command in READ_COMMANDS:
        if request != read_request(address, command):
            raise ProtocolError('invalid read request')
    elif command == Command.RESET_CURRENT_POSITION:
        if request != zero_current_position_request(address):
            raise ProtocolError('invalid current-position reset request')
    elif command == Command.SELECT_FIRMWARE:
        if (len(request) != 6 or request[2] != 0x69
                or request[3] not in (0, 1) or request[4] not in (0, 1, 2)):
            raise ProtocolError('invalid firmware selection request')
        if request != select_firmware_request(
                address, ('x', 'emm', 'emm_turbo')[request[4]], persist=bool(request[3])):
            raise ProtocolError('noncanonical firmware selection request')
    elif command == Command.ENABLE:
        if len(request) != 6 or request[3] not in (0, 1) or request != enable_request(address, bool(request[3])):
            raise ProtocolError('invalid enable request')
    elif command == Command.STOP:
        if request != stop_request(address):
            raise ProtocolError('invalid stop request')
    elif command == Command.VELOCITY:
        # Emm is eight bytes; X is nine. No other F6 layout is accepted.
        if len(request) == 8:
            velocity = int.from_bytes(request[3:5], 'big')
            velocity_valid = 1 <= velocity <= 3000
        elif len(request) == 9:
            velocity = int.from_bytes(request[5:7], 'big')
            velocity_valid = 1 <= velocity <= 30000
        else:
            raise ProtocolError('invalid velocity request length')
        if request[2] not in (0, 1) or request[-2] != 0 or not velocity_valid:
            raise ProtocolError('invalid velocity request')
    else:
        expected_length = 13 if command == Command.POSITION_EMM else 12
        velocity = int.from_bytes(request[3:5], 'big')
        if (len(request) != expected_length or request[2] not in (0, 1)
                or request[-3] not in (0, 1, 2) or request[-2] != 0
                or not 1 <= velocity <= (3000 if command == Command.POSITION_EMM else 30000)):
            raise ProtocolError('invalid position request')
    return address, command
