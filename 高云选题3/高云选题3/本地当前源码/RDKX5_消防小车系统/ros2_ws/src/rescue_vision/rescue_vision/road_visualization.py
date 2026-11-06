"""Same-exposure diagnostic image transport; never supplies control evidence.

The producer encodes only for an actual subscriber. The HTTP consumer keeps one
bounded frame and a short demand lease; /api/state never includes image bytes.
OpenCV is imported only by the producer, not by the dashboard service.
"""
import base64
import copy
import json
import math
import struct
import threading
import time

MAX_JPEG_BYTES = 512 * 1024
MAX_METADATA_BYTES = 64 * 1024
MAX_MESSAGE_BYTES = ((MAX_JPEG_BYTES + 2) // 3) * 4 + MAX_METADATA_BYTES + 1024
STALE_AFTER_S = 1.5
DEMAND_LEASE_S = 3.0


def compact(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False,
                      separators=(',', ':')).encode('utf-8')


def make_visualization(bgr, *, capture_ros_s, sequence, source_topic, results):
    """Build one independent 640-wide preview from this detection's raw pixels."""
    import cv2
    if bgr is None or len(bgr.shape) != 3 or bgr.shape[2] != 3:
        raise ValueError('invalid_road_preview_pixels')
    height, width = bgr.shape[:2]
    if width > 640:
        bgr = cv2.resize(bgr, (640, round(height * 640 / width)), interpolation=cv2.INTER_AREA)
    height, width = bgr.shape[:2]
    ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok or len(encoded) > MAX_JPEG_BYTES:
        raise ValueError('road_preview_jpeg_limit')
    frame = dict(token=str(sequence), sequence=str(sequence),
                 frame_id='physical-left:' + str(sequence), width=width, height=height,
                 capture_ros_s=capture_ros_s, source_topic=source_topic,
                 coordinate_frame='raw_physical_left', synthetic=False,
                 observation_only=True, control_eligible=False, detections=[],
                 road=copy.deepcopy(results.get('road')),
                 traffic=copy.deepcopy(results.get('traffic')))
    if len(compact(frame)) > MAX_METADATA_BYTES:
        raise ValueError('road_preview_metadata_limit')
    return dict(schema_version=1, synthetic=False, frame=frame,
                jpeg_base64=base64.b64encode(encoded.tobytes()).decode('ascii'))


def jpeg_dimensions(data):
    """Read JPEG SOF dimensions without decompressing a frame in the web process."""
    if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
        raise ValueError('invalid_road_jpeg')
    pos = 2
    while pos + 4 <= len(data):
        if data[pos] != 255:
            raise ValueError('invalid_road_jpeg_marker')
        while pos < len(data) and data[pos] == 255:
            pos += 1
        if pos >= len(data):
            break
        marker = data[pos]
        pos += 1
        if marker in (0xDA, 0xD9):
            break
        if marker in (0x01, *range(0xD0, 0xD8)):
            continue
        if pos + 2 > len(data):
            break
        size = int.from_bytes(data[pos:pos + 2], 'big')
        if size < 2 or pos + size > len(data):
            raise ValueError('truncated_road_jpeg')
        if marker in (0xC0, 0xC1, 0xC2):
            if size < 8:
                raise ValueError('invalid_road_jpeg_sof')
            return (int.from_bytes(data[pos + 5:pos + 7], 'big'),
                    int.from_bytes(data[pos + 3:pos + 5], 'big'))
        pos += size
    raise ValueError('road_jpeg_dimensions_missing')


class RoadVisualizationCache:
    def __init__(self, monotonic=time.monotonic, wall=time.time):
        self.monotonic, self.wall = monotonic, wall
        self.lock = threading.Lock()
        self.deadline = 0.0
        self.latest = None

    def wanted(self, enabled=True):
        with self.lock:
            if not enabled or self.monotonic() >= self.deadline:
                self.latest = None
                if not enabled:
                    self.deadline = 0.0
                return False
            return True

    def update(self, message):
        if not isinstance(message, str) or len(message) > MAX_MESSAGE_BYTES:
            return False
        try:
            value = json.loads(message)
            if (not isinstance(value, dict) or value.get('schema_version') != 1
                    or value.get('synthetic') is not False):
                return False
            frame = value['frame']
            if (not isinstance(frame, dict) or frame.get('synthetic') is not False
                    or frame.get('coordinate_frame') != 'raw_physical_left'
                    or frame.get('observation_only') is not True
                    or frame.get('control_eligible') is not False):
                return False
            token, capture = frame.get('token'), frame.get('capture_ros_s')
            if (not isinstance(token, str) or not token or len(token) > 100
                    or frame.get('sequence') != token
                    or frame.get('frame_id') != 'physical-left:' + token
                    or type(capture) not in (int, float) or not math.isfinite(capture)):
                return False
            width, height = frame.get('width'), frame.get('height')
            if type(width) is not int or type(height) is not int or not (1 <= width <= 640 and 1 <= height <= 1280):
                return False
            if len(compact(frame)) > MAX_METADATA_BYTES:
                return False
            for key in ('road', 'traffic'):
                result = frame.get(key)
                if result is None:
                    continue
                if (not isinstance(result, dict) or result.get('synthetic') is True
                        or result.get('sequence') != token
                        or result.get('frame_id') != frame['frame_id']
                        or result.get('capture_ros_s') != capture
                        or result.get('source_topic') != frame.get('source_topic')):
                    return False
                size = result.get('image_size')
                if not isinstance(size, list) or len(size) != 2 or any(type(v) is not int or not 1 <= v <= 8192 for v in size):
                    return False
            jpeg = base64.b64decode(value['jpeg_base64'], validate=True)
            if not jpeg or len(jpeg) > MAX_JPEG_BYTES or jpeg_dimensions(jpeg) != (width, height):
                return False
            now = self.monotonic()
            age = self.wall() - capture
            with self.lock:
                if now >= self.deadline or not 0 <= age <= STALE_AFTER_S:
                    return False
                if self.latest and capture <= self.latest[0]['capture_ros_s']:
                    return False
                self.latest = (frame, jpeg, now)
                return True
        except (ValueError, TypeError, KeyError, OverflowError):
            return False

    def packet(self, enabled=True):
        """A GET renews demand; stale/paused/waiting packets never contain pixels."""
        now = self.monotonic()
        with self.lock:
            if not enabled or now >= self.deadline:
                self.latest = None
            self.deadline = now + DEMAND_LEASE_S if enabled else 0.0
            frame, jpeg, age = None, b'', None
            status = 'waiting' if enabled else 'paused'
            if self.latest:
                candidate, data, received = self.latest
                age = self.wall() - candidate['capture_ros_s']
                if 0 <= age <= STALE_AFTER_S and 0 <= now - received <= STALE_AFTER_S:
                    frame, jpeg, status = candidate, data, 'live'
                else:
                    status = 'stale'
            reasons = {'waiting': 'waiting_for_road_frame', 'paused': 'image_stream_disabled',
                       'stale': 'road_frame_expired', 'live': 'same_capture_road_analysis'}
            metadata = dict(transport_version=1, synthetic=False, state=status, reason=reasons[status],
                            frame_age_s=age, stale_after_s=STALE_AFTER_S, frame=frame,
                            stream=dict(enabled=bool(enabled), active=frame is not None),
                            source='road_visualization', observation_only=True)
            header = compact(metadata)
            return struct.pack('!I', len(header)) + header + jpeg
