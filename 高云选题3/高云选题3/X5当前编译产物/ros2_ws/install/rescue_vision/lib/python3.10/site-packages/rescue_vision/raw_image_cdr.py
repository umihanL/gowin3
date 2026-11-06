"""Bounded sensor_msgs/Image CDR reader with a read-only view of pixel storage.

Use with rclpy's public ``raw=True`` subscription option. It avoids conversion
through ROS Image C storage and Python array.array, whose generated converter
makes two additional copies. The DDS serialized bytes still have an owner;
this is not a claim of transport zero-copy or a loaned ROS message.
"""
from dataclasses import dataclass, field
import struct


@dataclass
class _Stamp:
    sec: int = 0
    nanosec: int = 0


@dataclass
class _Header:
    stamp: _Stamp = field(default_factory=_Stamp)
    frame_id: str = ''


@dataclass(frozen=True)
class ImageView:
    header: object
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data: memoryview


def decode_image_cdr(serialized, *, header_factory=None, max_data_bytes=64 * 1024 * 1024):
    """Read CDR v1 BE/LE Image; reject unsupported formats and malformed input.

    ``header_factory=std_msgs.msg.Header`` supplies a real ROS header for callers
    which subsequently assign it to a ROS output message. The default is useful
    for ROS-independent tests and profiling. Pixel data refers to ``serialized``
    (which must be immutable bytes), preserving ownership through memoryview.
    """
    if not isinstance(serialized, bytes):
        raise TypeError('Raw image must be immutable serialized bytes')
    if not isinstance(max_data_bytes, int) or max_data_bytes <= 0:
        raise ValueError('max_data_bytes must be a positive integer')
    if len(serialized) < 4 or len(serialized) > max_data_bytes + 8192:
        raise ValueError('CDR image storage size is outside configured bounds')
    # DDS encapsulation IDs 0x0000/0x0001 are CDR v1 BE/LE, not XCDR2 or PL_CDR.
    identifier, options = struct.unpack_from('>HH', serialized)
    if identifier not in (0, 1) or options != 0:
        raise ValueError('Only plain CDR v1 BE/LE with zero options is supported')
    endian = '<' if identifier == 1 else '>'
    offset = 4

    def integer(fmt, size):
        nonlocal offset
        # Alignment is relative to the body after the 4-byte encapsulation.
        offset += (-(offset - 4)) % size
        if offset + size > len(serialized):
            raise ValueError('Truncated CDR scalar')
        result = struct.unpack_from(endian + fmt, serialized, offset)[0]
        offset += size
        return result

    def string(limit):
        nonlocal offset
        length = integer('I', 4)
        if not 1 <= length <= limit or offset + length > len(serialized):
            raise ValueError('Invalid CDR string length')
        if serialized[offset + length - 1] != 0:
            raise ValueError('CDR string lacks NUL terminator')
        value = serialized[offset:offset + length - 1]
        offset += length
        if b'\0' in value:
            raise ValueError('CDR string contains embedded NUL')
        try:
            return value.decode('utf-8')
        except UnicodeDecodeError as exc:
            raise ValueError('CDR string is not UTF-8') from exc

    sec, nanosec = integer('i', 4), integer('I', 4)
    if sec < 0 or nanosec >= 1_000_000_000:
        raise ValueError('Invalid image timestamp')
    frame_id = string(4096)
    height, width = integer('I', 4), integer('I', 4)
    encoding = string(64)
    is_bigendian = integer('B', 1)
    step, length = integer('I', 4), integer('I', 4)
    if not 0 < width <= 8192 or not 0 < height <= 8192:
        raise ValueError('Image dimensions are outside supported bounds')
    kind = encoding.lower()
    if kind not in ('nv12', 'bgr8', 'rgb8', 'mono8'):
        raise ValueError('Unsupported image encoding: ' + encoding)
    if is_bigendian not in (0, 1):
        raise ValueError('Invalid is_bigendian value')
    channels = 3 if kind in ('bgr8', 'rgb8') else 1
    if not width * channels <= step <= max_data_bytes:
        raise ValueError('Image row stride is invalid')
    if kind == 'nv12' and (width % 2 or height % 2 or step % 2):
        raise ValueError('NV12 dimensions and stride must be even')
    rows = height * 3 // 2 if kind == 'nv12' else height
    if not 0 < length <= max_data_bytes or length != rows * step:
        raise ValueError('Pixel length does not match image geometry')
    if offset + length != len(serialized):
        raise ValueError('Truncated pixel data or unexpected trailing CDR bytes')
    header = (header_factory or _Header)()
    header.stamp.sec, header.stamp.nanosec = sec, nanosec
    header.frame_id = frame_id
    return ImageView(header, height, width, encoding, is_bigendian, step,
                     memoryview(serialized)[offset:offset + length])
