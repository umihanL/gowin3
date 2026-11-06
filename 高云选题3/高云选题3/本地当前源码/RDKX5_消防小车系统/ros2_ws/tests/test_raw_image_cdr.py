import copy
import importlib.util
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.raw_image_cdr import decode_image_cdr


def fixture(*, little=True, frame_id='左眼', width=4, height=2,
            encoding='nv12', step=4, pixels=None, sec=42, nanosec=789):
    """Independent CDR serializer covering the Image schema used by the parser."""
    body = bytearray()
    endian = '<' if little else '>'

    def scalar(fmt, value):
        alignment = struct.calcsize(fmt)
        body.extend(b'\0' * ((-len(body)) % alignment))
        body.extend(struct.pack(endian + fmt, value))

    def string(value):
        encoded = value.encode() + b'\0'
        scalar('I', len(encoded))
        body.extend(encoded)

    scalar('i', sec)
    scalar('I', nanosec)
    string(frame_id)
    scalar('I', height)
    scalar('I', width)
    string(encoding)
    scalar('B', 0)
    scalar('I', step)
    if pixels is None:
        pixels = bytes(range(step * height * (3 if encoding == 'nv12' else 2) // 2))
    scalar('I', len(pixels))
    body.extend(pixels)
    return struct.pack('>HH', int(little), 0) + bytes(body)


class RawImageCdrTests(unittest.TestCase):
    def test_endian_utf8_alignment_pixels_and_ownership(self):
        for little in (False, True):
            for frame_id in ('', 'a', 'ab', 'abc', 'abcd', '左眼镜头'):
                raw = fixture(little=little, frame_id=frame_id)
                image = decode_image_cdr(raw)
                self.assertEqual(image.header.frame_id, frame_id)
                self.assertEqual(image.header.stamp.sec, 42)
                self.assertEqual(image.header.stamp.nanosec, 789)
                self.assertEqual((image.width, image.height, image.step), (4, 2, 4))
                self.assertEqual(bytes(image.data), bytes(range(12)))
                self.assertIs(image.data.obj, raw)
                self.assertTrue(image.data.readonly)
                del raw
                self.assertEqual(image.data[11], 11)

    def test_other_encodings_and_stride_padding(self):
        for kind, step in [('bgr8', 16), ('rgb8', 16), ('mono8', 6), ('nv12', 6)]:
            image = decode_image_cdr(fixture(encoding=kind, step=step))
            self.assertEqual(image.step, step)
            self.assertEqual(len(image.data), step * (3 if kind == 'nv12' else 2))

    def test_header_factory_preserves_ros_assignment_contract(self):
        class Header:
            def __init__(self):
                class Stamp:
                    pass
                self.stamp = Stamp()
        image = decode_image_cdr(fixture(), header_factory=Header)
        self.assertIsInstance(image.header, Header)
        self.assertEqual(copy.deepcopy(image.header).stamp.sec, 42)

    def test_all_truncations_and_trailing_payload_rejected(self):
        raw = fixture()
        for end in range(len(raw)):
            with self.assertRaises(ValueError):
                decode_image_cdr(raw[:end])
        with self.assertRaises(ValueError):
            decode_image_cdr(raw + b'\0')

    def test_invalid_geometry_encapsulation_time_and_storage(self):
        for changes in [dict(width=3), dict(height=3), dict(step=3),
                        dict(width=0), dict(width=9000), dict(encoding='jpeg'),
                        dict(sec=-1), dict(nanosec=1_000_000_000),
                        dict(pixels=b'xx')]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                decode_image_cdr(fixture(**changes))
        raw = fixture()
        for prefix in [b'\0\3\0\0', b'\0\1\0\1']:
            with self.assertRaises(ValueError):
                decode_image_cdr(prefix + raw[4:])
        with self.assertRaises(TypeError):
            decode_image_cdr(bytearray(raw))
        with self.assertRaises(ValueError):
            decode_image_cdr(raw, max_data_bytes=8)

    @unittest.skipUnless(importlib.util.find_spec('rclpy'), 'requires ROS 2 serializer')
    def test_real_ros_serializer_exact_payload_and_header(self):
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Image
        from std_msgs.msg import Header
        for encoding, channels in [('nv12', 1), ('bgr8', 3), ('mono8', 1)]:
            original = Image()
            original.header.stamp.sec = 946685000
            original.header.stamp.nanosec = 123456789
            original.header.frame_id = 'physical_left/楼'
            original.width, original.height = 1280, 1088
            original.encoding = encoding
            original.step = 1280 * channels
            rows = 1632 if encoding == 'nv12' else 1088
            original.data = (bytes(range(256)) * (original.step * rows // 256))
            serialized = serialize_message(original)
            parsed = decode_image_cdr(serialized, header_factory=Header)
            self.assertEqual(parsed.header, original.header)
            self.assertEqual(parsed.encoding, original.encoding)
            self.assertEqual(parsed.data, memoryview(original.data))
            self.assertIs(parsed.data.obj, serialized)


if __name__ == '__main__':
    unittest.main()
