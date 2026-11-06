"""One empirical, bounded FD8 bench profile; never production motion approval.

The 2026-10-31 hardware evidence verified these exact configuration bytes and
FD mode0 +8/-8 pulses at encoded speed1, acceleration1. Unknown option bit10
remains unknown. No velocity, arbitrary angle, calibration or mode writes are
authorized by this profile. The controller checks live targets and encoders.
"""
CONFIG = bytes.fromhex('21 15 19 02 02 02 00 10 01 00 04 b0 0b b8 13 88 05 07 01 00 01 01 00 08 08 98 07 d0 00 08')
COUNT_DEG = 360.0 / 65536
STEP_DEG = .9
STEP_COUNTS = STEP_DEG / COUNT_DEG
STATIONARY_TOLERANCE_DEG = .15
PROFILE = 'zdt_vendor_fd8_bench'


def matches(version, config_bytes, options):
    return (version.get('firmware_raw') == 206 and version.get('hardware_raw') == 0x2314
            and config_bytes == CONFIG and options.get('raw_bytes_hex') == '04 26')


def frame(direction):
    if direction not in (-1, 1):
        raise ValueError('FD8 direction must be -1 or 1')
    return bytes.fromhex('01 fd %02x 00 01 01 00 00 00 08 00 00 6b' % int(direction < 0))


def settled(sample, expected):
    """Independent actual position, both command targets and following error."""
    return (sample['speed_rpm'] == 0 and abs(sample['position_error_deg']) <= STATIONARY_TOLERANCE_DEG
            and abs(sample['position_counts'] - sample['position_before_counts']) * COUNT_DEG <= .03
            and abs(sample['position_counts'] - expected) * COUNT_DEG <= STATIONARY_TOLERANCE_DEG
            and abs(sample['position_before_counts'] - expected) * COUNT_DEG <= STATIONARY_TOLERANCE_DEG
            and abs(sample['retained_target_counts'] - expected) <= 2
            and abs(sample['realtime_target_counts'] - expected) <= 2
            and abs(sample['retained_target_counts'] - sample['realtime_target_counts']) <= 1)
