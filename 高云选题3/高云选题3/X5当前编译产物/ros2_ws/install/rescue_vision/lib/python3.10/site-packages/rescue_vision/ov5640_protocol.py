"""The actual ov5640 host UART: fixed 10-byte frames, XOR, not the CRC proposal."""
import struct
from dataclasses import dataclass

HEARTBEAT, STOP = 0x01, 0x05
RESUME = 0x07
START, HOST_STATUS, SELECT_POINT, POINT_STATUS = 0x03, 0x06, 0x08, 0x09
MOVE, VELOCITY, TASK_MODE, LIGHT, EXT_STATUS, CAPABILITIES, ROUTE_TURN = range(0x10, 0x17)
ZEBRA_ENABLE = 0x17
PUMP_ON, PUMP_OFF, PUMP_STATUS = 0x18, 0x19, 0x1a
DONE, FAILED, ZEBRA = 0xf0, 0xf1, 0xf3
LINK_TIMEOUT = 0xf2
SET_START, WAYPOINT, SET_GOAL, PLAN = 0x20, 0x21, 0x22, 0x23
SET_START_HEADING = 0x24
RETAINED_JOG, RETAINED_JOG_STATUS = 0x3c, 0x3d


def heading_payload(degrees):
    """Map heading, CCW from right, in signed tenths of a degree."""
    import math
    if type(degrees) not in (int, float) or not math.isfinite(degrees) or not -180 <= degrees <= 180:
        raise ValueError('numbered_heading_requires_minus180_to180')
    return struct.pack('<hBB', round(degrees * 10), 0, 0)


@dataclass(frozen=True)
class Packet:
    sequence: int
    command: int
    data: bytes


def encode(sequence, command, data=b'\0\0\0\0'):
    if type(sequence) is not int or not 0 <= sequence <= 255:
        raise ValueError('invalid_sequence')
    if type(command) is not int or not 0 <= command <= 255 or len(data) != 4:
        raise ValueError('invalid_command_or_payload')
    body = bytes((1, sequence, command)) + bytes(data)
    checksum = 0
    for value in body:
        checksum ^= value
    return b'\xa5\x5a' + body + bytes((checksum,))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        self.buffer = self.buffer[-4096:]
        out = []
        while len(self.buffer) >= 2:
            pos = self.buffer.find(b'\xa5\x5a')
            if pos < 0:
                self.buffer = self.buffer[-1:] if self.buffer[-1] == 0xa5 else bytearray()
                break
            del self.buffer[:pos]
            if len(self.buffer) < 10:
                break
            raw = self.buffer[:10]
            checksum = 0
            for byte in raw[2:9]:
                checksum ^= byte
            if raw[2] != 1 or checksum != raw[9]:
                self.bad_frames += 1
                del self.buffer[0]
                continue
            out.append(Packet(raw[3], raw[4], bytes(raw[5:9])))
            del self.buffer[:10]
        return out


def move_payload(direction, distance_m, speed_m_s=.10):
    if direction not in ('forward', 'reverse'):
        raise ValueError('invalid_direction')
    if type(distance_m) not in (int, float) or not .05 <= distance_m <= 10.:
        raise ValueError('distance_requires_5_to_1000_cm')
    if type(speed_m_s) not in (int, float) or not .02 <= speed_m_s <= .2:
        raise ValueError('speed_requires_20_to_200_mm_s')
    return struct.pack('<hH', round(distance_m * 1000) * (1 if direction == 'forward' else -1),
                       round(speed_m_s * 1000))


def velocity_payload(speed_mm_s, steering_deg=0):
    if type(speed_mm_s) is not int or not -200 <= speed_mm_s <= 200:
        raise ValueError('invalid_velocity')
    if type(steering_deg) is not int or not -28 <= steering_deg <= 28:
        raise ValueError('invalid_steering')
    return struct.pack('<hbB', speed_mm_s, steering_deg, 0)


def route_payload(junction_count, turn, speed_mm_s=100):
    if type(junction_count) is not int or not 1 <= junction_count <= 8 or turn not in ('left', 'right'):
        raise ValueError('invalid_junction_request')
    if type(speed_mm_s) is not int or not 20 <= speed_mm_s <= 200:
        raise ValueError('invalid_route_speed')
    return struct.pack('<BBH', junction_count, 1 if turn == 'left' else 2, speed_mm_s)


def capabilities(data):
    if (len(data) != 4 or data[0] != 0 or data[1] not in (1, 2) or
            data[2] & ~(63 if data[1] == 1 else 127) or data[3] > 3):
        raise ValueError('unsupported_fpga_extension_firmware')
    return {key: bool(data[2] & (1 << i)) for i, key in
            enumerate(('distance', 'velocity', 'traffic', 'route_turn', 'zebra_switch', 'pump_b13', 'retained_jog'))}


def retained_jog_payload(distance_mm, speed_mm_s=40):
    if type(distance_mm) is not int or not 1 <= abs(distance_mm) <= 100:
        raise ValueError('retained_jog_distance_requires_signed_1_to_100_mm')
    if type(speed_mm_s) is not int or not 20 <= speed_mm_s <= 60:
        raise ValueError('retained_jog_speed_requires_20_to_60_mm_s')
    return struct.pack('<hH', distance_mm, speed_mm_s)


def retained_jog_status(data):
    # Revision 20261104 separates invalid pose / inertial health / route
    # generation / cursor (9..12); old firmware's combined reason 3 remains valid.
    if (len(data) != 4 or data[0] > 4 or data[1] > 12 or data[3] & ~7 or
            (data[0] == 3 and data[1] != 0)):
        raise ValueError('invalid_retained_jog_status')
    return dict(phase=data[0], reason_code=data[1], accepted_sequence=data[2],
                stopped=bool(data[3] & 1), nav_valid=bool(data[3] & 2), run=bool(data[3] & 4))


def pump_payload(duration_ms):
    if type(duration_ms) is not int or not 1 <= duration_ms <= 1000:
        raise ValueError('fpga_pump_duration_requires_1_to_1000_ms')
    return struct.pack('<HBB', duration_ms, 0, 0)


def pump_status(data):
    if len(data) != 4 or data[0] & ~63 or data[1] not in (0, 1, 2, 3, 4, 5, 6, 8):
        raise ValueError('invalid_fpga_pump_status')
    remaining = int.from_bytes(data[2:4], 'little')
    on, ready, lockout = bool(data[0] & 1), bool(data[0] & 16), bool(data[0] & 32)
    if (remaining > 1000 or (not on and remaining != 0) or
            (on and (ready or lockout)) or (ready and (lockout or data[0] & 14 != 14))):
        raise ValueError('invalid_fpga_pump_status')
    reasons = {0: 'fpga_pump_normal', 1: 'fpga_pump_pulse_complete',
               2: 'fpga_pump_link_lost', 3: 'fpga_pump_scope_lost',
               4: 'fpga_pump_vehicle_moving', 5: 'fpga_pump_stopped',
               6: 'fpga_pump_mode_or_motion_changed', 8: 'fpga_pump_reset'}
    return dict(pump_on=on, wheels_still=bool(data[0] & 2), link_ok=bool(data[0] & 4),
                scope=bool(data[0] & 8), pump_ready=ready, lockout=lockout,
                remaining_ms=remaining, reason=reasons[data[1]], reason_code=data[1])


def status(data):
    if len(data) != 4 or data[0] > 12 or data[1] & ~31:
        raise ValueError('invalid_extension_status')
    return dict(phase=data[0], enabled=bool(data[1] & 1), vehicle_stopped=bool(data[1] & 2),
                zebra_wait=bool(data[1] & 4), mode_selected=bool(data[1] & 8),
                zebra_detection_enabled=bool(data[1] & 16),
                remaining_mm=int.from_bytes(data[2:4], 'little'))


def point_payload(point, task):
    if type(point) is not int or not 1 <= point <= 9:
        raise ValueError('numbered_point_requires_1_to_9')
    if type(task) is not int or not 0 <= task <= 255:
        raise ValueError('invalid_point_task_id')
    return bytes((point, task, 0, 0))


def point_status(data):
    if len(data) != 4 or data[0] != 0 or data[1] > 9 or data[2] > 4 or data[3] > 7:
        raise ValueError('invalid_numbered_point_status')
    # Generic-cell A* paths use point=0, including the route-ready state.
    # A numbered owner still verifies its own selected point before START.
    return dict(selected_point=data[1], state=data[2], error=data[3])


def host_status(data):
    if len(data) != 4 or data[0] != 0 or data[2] & ~31:
        raise ValueError('invalid_host_navigation_status')
    return dict(motion_status=data[1] & 15, motion_phase=data[1] >> 4,
                selected=bool(data[2] & 1), scope=bool(data[2] & 2),
                run=bool(data[2] & 4), nav_session=bool(data[2] & 8),
                link_ok=bool(data[2] & 16), task_id=data[3])
