#!/usr/bin/env python3
"""Read-only single-eye YOLO preview. Image/boxes must belong to the exact frame.

No camera driver, inference engine, actuator or command publisher is started here.
ROS imports stay in main so pairing and stale-state checks run without ROS.
"""
import argparse
import base64
from collections import OrderedDict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import struct
import threading
import time
from urllib.parse import parse_qs, urlsplit


def header_key(message):
    stamp = message.header.stamp
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("Invalid ROS timestamp")
    return (sec, nanosec, str(message.header.frame_id))


def message_key(message):
    """Legacy StereoPair identity, including its real sequence field."""
    return (int(message.sequence),) + header_key(message)


def jpeg_dimensions(jpeg):
    """Read JPEG SOF dimensions without decoding pixels or trusting ROS metadata."""
    data = memoryview(jpeg).cast("B")
    if len(data) < 4 or bytes(data[:2]) != b"\xff\xd8":
        raise ValueError("Preview is not a JPEG")
    if len(data) > 32 * 1024 * 1024:
        raise ValueError("Preview JPEG exceeds size limit")
    sof = {0xc0, 0xc1, 0xc2, 0xc3, 0xc5, 0xc6, 0xc7,
           0xc9, 0xca, 0xcb, 0xcd, 0xce, 0xcf}
    offset = 2
    while offset < len(data):
        if data[offset] != 0xff:
            raise ValueError("Invalid JPEG marker")
        while offset < len(data) and data[offset] == 0xff:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in (0xd9, 0xda, 0x00):
            break
        if marker == 0x01 or 0xd0 <= marker <= 0xd8:
            continue
        if offset + 2 > len(data):
            raise ValueError("Truncated JPEG segment")
        length = (data[offset] << 8) | data[offset + 1]
        if length < 2 or offset + length > len(data):
            raise ValueError("Invalid JPEG segment length")
        if marker in sof:
            if length < 8:
                raise ValueError("Truncated JPEG dimensions")
            height = (data[offset + 3] << 8) | data[offset + 4]
            width = (data[offset + 5] << 8) | data[offset + 6]
            components = data[offset + 7]
            if not components or length < 8 + 3 * components:
                raise ValueError("Invalid JPEG component table")
            if not (0 < width <= 16384 and 0 < height <= 16384):
                raise ValueError("Invalid JPEG dimensions")
            return width, height
        offset += length
    raise ValueError("JPEG has no complete dimensions header")


def pack_frame(metadata, jpeg=b""):
    """One immutable response binds metadata and JPEG; no second image request."""
    encoded = json.dumps(metadata, ensure_ascii=False, allow_nan=False,
                         separators=(",", ":")).encode("utf-8")
    return struct.pack(">I", len(encoded)) + encoded + jpeg


class PreviewState:
    """Bounded exact-frame matching and a thread-safe latest completed result."""

    def __init__(self, queue_size=4, stale_s=3.0, image_format="compressed", lease_s=5.0):
        if not 1 <= queue_size <= 4 or stale_s <= 0:
            raise ValueError("queue_size must be 1..4 and stale_s positive")
        if image_format not in ("compressed", "stereo"):
            raise ValueError("image_format must be compressed or stereo")
        self.image_format = image_format
        self.queue_size, self.stale_s = queue_size, float(stale_s)
        self.lock = threading.Lock()
        self.frames, self.detections = OrderedDict(), OrderedDict()
        self.fire_registries = OrderedDict()
        self.firefighter_registries = OrderedDict()
        self.latest = None
        self.latest_jpeg = b""
        self.latest_at = None
        self.latest_stamp_ns = None
        self.paired_frames = 0
        self.arrivals = deque(maxlen=20)
        self.available = None
        self.status = "waiting_for_matching_frame_and_detections"
        self.performance = None
        self.performance_at = None
        if not 0 < lease_s <= 30:
            raise ValueError("lease_s must be positive and at most 30 seconds")
        self.lease_s = float(lease_s)
        self.stream_requested = True
        self.stream_active = True
        self.stream_generation = 0
        self.stream_min_stamp_ns = None
        self.lease_at = time.monotonic()

    def _transition_stream(self, active, wall_ns=None):
        if self.stream_active == active:
            return
        self.stream_active = active
        self.stream_generation += 1
        self.stream_min_stamp_ns = (time.time_ns() if wall_ns is None else wall_ns) if active else None
        self.frames.clear()
        self.detections.clear()
        self.fire_registries.clear()
        self.firefighter_registries.clear()
        self.latest = self.latest_at = self.latest_stamp_ns = None
        self.latest_jpeg = b""
        self.arrivals.clear()

    def set_stream(self, enabled, now=None, wall_ns=None):
        if type(enabled) is not bool:
            raise ValueError("enabled must be a boolean")
        now = time.monotonic() if now is None else now
        with self.lock:
            self.stream_requested = enabled
            self.lease_at = now if enabled else None
            self._transition_stream(enabled, wall_ns)

    def touch_stream(self, now=None, wall_ns=None):
        """Only actual binary-image requests renew the global preview lease."""
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.stream_requested:
                self.lease_at = now
                self._transition_stream(True, wall_ns)

    def stream_target(self, now=None):
        """Called by the ROS timer; HTTP handlers never touch ROS subscriptions."""
        now = time.monotonic() if now is None else now
        with self.lock:
            active = (self.stream_requested and self.lease_at is not None
                      and 0 <= now - self.lease_at <= self.lease_s)
            self._transition_stream(active)
            return self.stream_active, self.stream_generation

    def update_performance(self, performance):
        with self.lock:
            self.performance = performance
            self.performance_at = time.monotonic()

    def _prune(self, now):
        for queue in (self.frames, self.detections, self.fire_registries, self.firefighter_registries):
            for key in list(queue):
                if not 0 <= now - queue[key][0] <= self.stale_s:
                    del queue[key]

    def prune(self, now=None):
        with self.lock:
            self._prune(time.monotonic() if now is None else now)

    def _key(self, message):
        return header_key(message) if self.image_format == "compressed" else message_key(message)

    def _frame_size(self, message):
        if self.image_format == "compressed":
            if "jpeg" not in str(message.format).lower():
                raise ValueError("Compressed preview must use JPEG format")
            return jpeg_dimensions(message.data)
        return int(message.left.width), int(message.left.height)

    def _push(self, kind, message, now, generation=None):
        key = self._key(message)
        if kind == "frame":
            width, height = self._frame_size(message)
        else:
            width, height = int(message.width), int(message.height)
        if width <= 0 or height <= 0:
            return None
        with self.lock:
            if not self.stream_active or (generation is not None and generation != self.stream_generation):
                return None
            self._prune(now)
            sec, nanosec, _ = header_key(message)
            stamp_ns = sec * 1_000_000_000 + nanosec
            if self.stream_min_stamp_ns is not None and stamp_ns < self.stream_min_stamp_ns:
                return None
            if self.latest_stamp_ns is not None and stamp_ns <= self.latest_stamp_ns:
                return None
            own = self.frames if kind == "frame" else self.detections
            other = self.detections if kind == "frame" else self.frames
            own[key] = (now, width, height, message)
            own.move_to_end(key)
            while len(own) > self.queue_size:
                own.popitem(last=False)
            if key not in other:
                return None
            _, other_width, other_height, other_message = other.pop(key)
            own.pop(key)
            if (width, height) != (other_width, other_height):
                self.status = "frame_detection_size_mismatch"
                return None
            return (message, other_message) if kind == "frame" else (other_message, message)

    def push_frame(self, message, now=None, generation=None):
        return self._push("frame", message, time.monotonic() if now is None else now, generation)

    def push_detections(self, message, now=None, generation=None):
        return self._push("detections", message, time.monotonic() if now is None else now, generation)

    def push_fire_registry(self, document, now=None):
        return self._push_registry(document, now, "fire")

    def push_firefighter_registry(self, document, now=None):
        if not isinstance(document, dict) or document.get("target_class") != "firefighter":
            return False
        return self._push_registry(document, now, "firefighter")

    def _push_registry(self, document, now, target_class):
        """Join IDs by integer capture identity; never move old boxes onto new pixels."""
        if not isinstance(document, dict) or document.get("frame_valid") is not True:
            return False
        stamp, sequence = document.get("capture_stamp_ns"), document.get("sequence")
        session, coordinate = document.get("registry_session"), document.get("coordinate_frame")
        size, targets = document.get("image_size_wh"), document.get("targets")
        if (type(stamp) is not int or stamp < 0 or type(sequence) is not int or sequence < 0
                or not isinstance(session, str) or not session or len(session) > 256
                or not isinstance(coordinate, str) or not coordinate or len(coordinate) > 256
                or not isinstance(size, (list, tuple)) or len(size) != 2
                or any(type(v) is not int or not 0 < v <= 16384 for v in size)
                or not isinstance(targets, list)):
            return False
        sec, nanosec = divmod(stamp, 1_000_000_000)
        token = ":".join(map(str, (sequence, sec, nanosec, coordinate)))
        items, seen = [], set()
        for row in targets[:128]:
            if not isinstance(row, dict):
                continue
            target_id, box, confidence = row.get("target_id"), row.get("bbox_xyxy"), row.get("confidence")
            if (type(target_id) is not int or target_id <= 0 or target_id in seen
                    or not isinstance(box, (list, tuple)) or len(box) != 4
                    or any(type(v) not in (float, int) or not math.isfinite(v) for v in box)
                    or not 0 <= box[0] < box[2] <= size[0] or not 0 <= box[1] < box[3] <= size[1]
                    or type(confidence) not in (float, int) or not math.isfinite(confidence)
                    or not 0 <= confidence <= 1):
                continue
            seen.add(target_id)
            items.append(dict(target_id=target_id, bbox_xyxy=list(box), confidence=float(confidence),
                              selectable=row.get("selectable") is True))
        registry = dict(frame_token=token, registry_session=session, coordinate_frame=coordinate,
                        target_class=target_class,
                        image_size_wh=list(size), targets=items)
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.stream_active or (self.stream_min_stamp_ns is not None and stamp < self.stream_min_stamp_ns):
                return False
            self._prune(now)
            cache = self.fire_registries if target_class == "fire" else self.firefighter_registries
            field = "fire_registry" if target_class == "fire" else "firefighter_registry"
            cache[token] = (now, registry)
            cache.move_to_end(token)
            while len(cache) > 64:
                cache.popitem(last=False)
            if (self.latest is not None and self.latest["token"] == token
                    and list(size) == [self.latest["width"], self.latest["height"]]):
                # Older HTTP snapshots must remain immutable. This does not renew
                # image freshness, increment FPS, or decode/re-encode the JPEG.
                self.latest = dict(self.latest, **{field: registry})
        return True

    def commit(self, frame, detections, jpeg_bytes, now=None, generation=None):
        """JPEG passes through in compressed mode; never reuse previous boxes."""
        if self._key(frame) != self._key(detections):
            raise ValueError("Image and detection identity differ")
        size = self._frame_size(frame)
        if size != (int(detections.width), int(detections.height)):
            raise ValueError("Image and detection dimensions differ")
        if jpeg_dimensions(jpeg_bytes) != size:
            raise ValueError("Encoded JPEG dimensions differ from detection coordinates")
        if self.image_format == "compressed" and bytes(frame.data) != jpeg_bytes:
            raise ValueError("Compressed JPEG payload differs from matched frame")
        items = []
        for detection in detections.detections:
            box = [float(value) for value in detection.bbox_xyxy]
            confidence = float(detection.confidence)
            if len(box) != 4 or not all(math.isfinite(v) for v in box + [confidence]):
                continue
            depth = float(detection.depth_m)
            depth_valid = bool(detection.depth_valid) and math.isfinite(depth) and depth > 0
            items.append({"class_id": int(detection.class_id),
                          "class_name": str(detection.class_name),
                          "confidence": confidence, "bbox_xyxy": box,
                          "depth_valid": depth_valid, "depth_m": depth if depth_valid else 0.0})
        # CompressedImage has no sequence: only DetectionArray supplies it.
        key = message_key(detections)
        completed = {"token": ":".join(map(str, key)), "sequence": key[0],
                     "coordinate_frame": key[3],
                     "width": int(detections.width), "height": int(detections.height),
                     "detections": items}
        now = time.monotonic() if now is None else now
        stamp_ns = key[1] * 1_000_000_000 + key[2]
        with self.lock:
            if not self.stream_active or (generation is not None and generation != self.stream_generation):
                return False
            if self.stream_min_stamp_ns is not None and stamp_ns < self.stream_min_stamp_ns:
                return False
            if self.latest_stamp_ns is not None and stamp_ns <= self.latest_stamp_ns:
                return False
            self._prune(now)
            cached = self.fire_registries.get(completed["token"])
            if cached is not None and cached[1]["image_size_wh"] == list(size):
                completed["fire_registry"] = cached[1]
            cached = self.firefighter_registries.get(completed["token"])
            if cached is not None and cached[1]["image_size_wh"] == list(size):
                completed["firefighter_registry"] = cached[1]
            self.latest, self.latest_at, self.latest_stamp_ns = completed, now, stamp_ns
            self.latest_jpeg = bytes(jpeg_bytes)
            self.available, self.status = bool(detections.detector_available), str(detections.status)
            self.paired_frames += 1
            self.arrivals.append(now)
        return True

    def snapshot_packet(self, after="", now=None, wall_ns=None):
        """Atomically capture metadata plus matching bytes; no base64 work."""
        now = time.monotonic() if now is None else now
        wall_ns = time.time_ns() if wall_ns is None else wall_ns
        with self.lock:
            self._prune(now)
            update_age = None if self.latest_at is None else now - self.latest_at
            frame_age = None if self.latest_stamp_ns is None else (wall_ns - self.latest_stamp_ns) / 1e9
            live = (self.latest is not None and 0 <= update_age <= self.stale_s
                    and 0 <= frame_age <= self.stale_s)
            samples = [t for t in self.arrivals if 0 <= now - t <= self.stale_s]
            fps = ((len(samples) - 1) / (samples[-1] - samples[0])
                   if live and len(samples) >= 2 and samples[-1] > samples[0] else 0.0)
            new_frame = self.stream_active and self.latest is not None and self.latest["token"] != after
            metadata = {"state": ("paused" if not self.stream_active else
                                  "live" if live else ("stale" if self.latest else "waiting")),
                    "transport_version": 1, "stale_after_s": self.stale_s,
                    "stream": {"enabled": self.stream_requested, "active": self.stream_active,
                               "generation": self.stream_generation, "lease_s": self.lease_s},
                    "detector_available": self.available, "status": self.status,
                    "frame_age_s": frame_age, "update_age_s": update_age,
                    "fps": round(fps, 2), "paired_frames": self.paired_frames,
                    "performance": (self.performance if self.performance_at is not None
                        and 0 <= now - self.performance_at <= self.stale_s else None),
                    "frame": self.latest if new_frame else None,
                    "fire_registry_update": (self.latest.get("fire_registry")
                        if live and self.stream_active and not new_frame else None),
                    "firefighter_registry_update": (self.latest.get("firefighter_registry")
                        if live and self.stream_active and not new_frame else None)}
            return metadata, self.latest_jpeg if new_frame else b""

    def snapshot(self, after="", now=None, wall_ns=None):
        """Legacy JSON API: encode base64 only when explicitly requested."""
        metadata, jpeg = self.snapshot_packet(after, now, wall_ns)
        if metadata["frame"] is not None:
            metadata["frame"] = dict(metadata["frame"],
                                     jpeg_base64=base64.b64encode(jpeg).decode("ascii"))
        return metadata


def encode_frame(frame):
    import cv2
    from rescue_vision.mipi_image import image_to_bgr
    source = frame.left
    image = image_to_bgr(source.data, source.width, source.height, source.step, source.encoding)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 82])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return encoded.tobytes()


def handler_class(state, page):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        # Headers and body are separate writes. On Linux, Nagle plus delayed
        # ACK otherwise stalls small keep-alive responses for about 40–60 ms.
        # The browser polls serially, so even an unchanged-frame reply matters.
        disable_nagle_algorithm = True
        def log_message(self, *args):
            pass

        def do_GET(self):
            self._serve(False)

        def do_HEAD(self):
            self._serve(True)

        def do_POST(self):
            if urlsplit(self.path).path != "/api/stream":
                self.send_error(404)
                return
            if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
                self.close_connection = True
                self.send_error(415, "application/json required")
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 1024:
                    raise ValueError("invalid body size")
                payload = json.loads(self.rfile.read(length))
                state.set_stream(payload["enabled"])
            except (ValueError, TypeError, KeyError):
                self.close_connection = True
                self.send_error(400, "Expected enabled boolean")
                return
            metadata, _ = state.snapshot_packet()
            metadata["frame"] = None
            body = json.dumps(metadata, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _serve(self, head):
            parsed = urlsplit(self.path)
            if parsed.path in ("/", "/index.html", "/detection-preview.html"):
                body, content_type = page, "text/html; charset=utf-8"
            elif parsed.path == "/api/state":
                query = parse_qs(parsed.query)
                after = query.get("after", [""])[0]
                if query.get("images", ["1"])[0] == "0":
                    metadata, _ = state.snapshot_packet(after)
                    metadata["frame"] = None
                else:
                    metadata = state.snapshot(after)
                body = json.dumps(metadata, ensure_ascii=False, allow_nan=False).encode("utf-8")
                content_type = "application/json; charset=utf-8"
            elif parsed.path == "/api/frame":
                if not head:
                    state.touch_stream()
                after = parse_qs(parsed.query).get("after", [""])[0]
                metadata, jpeg = state.snapshot_packet(after)
                body = pack_frame(metadata, jpeg)
                content_type = "application/octet-stream"
            elif parsed.path == "/favicon.ico":
                self.send_response(204)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            if not head:
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8086)
    parser.add_argument("--image-format", choices=("compressed", "stereo"),
                        default="compressed", help="Default: already encoded single-eye JPEG")
    args = parser.parse_args()
    page_path = Path(__file__).resolve().parents[1] / "web" / "detection-preview.html"
    page = page_path.read_bytes()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rescue_interfaces.msg import StereoPair, DetectionArray
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    state = PreviewState(image_format=args.image_format)
    rclpy.init()

    class PreviewNode(Node):
        def __init__(self):
            super().__init__("detection_preview")
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(DetectionArray, "/perception/detections", self.detections, qos)
            self.create_subscription(String, "/diagnostics/perception", self.performance, 10)
            self.create_subscription(String, "/fire/targets", self.fire_targets, qos)
            self.create_subscription(String, "/firefighter/targets", self.firefighter_targets, qos)
            self.image_subscription = None
            self.image_generation = None
            self.image_qos = qos
            self.create_timer(0.2, self.sync_stream)
            self.create_timer(0.5, state.prune)
            self.last_warning = -float("inf")

        def sync_stream(self):
            active, generation = state.stream_target()
            if self.image_subscription is not None and (not active or generation != self.image_generation):
                self.destroy_subscription(self.image_subscription)
                self.image_subscription = None
            if active and self.image_subscription is None:
                self.image_generation = generation
                msg_type = CompressedImage if args.image_format == "compressed" else StereoPair
                topic = "/vision/preview/compressed" if args.image_format == "compressed" else "/vision/frame"
                self.image_subscription = self.create_subscription(
                    msg_type, topic, lambda message, epoch=generation: self.process("frame", message, epoch),
                    self.image_qos)

        def process(self, kind, message, generation=None):
            try:
                if generation is None:
                    active, generation = state.stream_target()
                    if not active:
                        return
                pair = (state.push_frame(message, generation=generation) if kind == "frame"
                        else state.push_detections(message, generation=generation))
                if pair:
                    frame, detections = pair
                    jpeg = bytes(frame.data) if args.image_format == "compressed" else encode_frame(frame)
                    state.commit(frame, detections, jpeg, generation=generation)
            except (ValueError, TypeError, RuntimeError) as exc:
                if time.monotonic() - self.last_warning > 5:
                    self.get_logger().warning("Preview frame rejected: " + str(exc))
                    self.last_warning = time.monotonic()

        def performance(self, message):
            try:
                payload = json.loads(message.data)
                if isinstance(payload, dict):
                    state.update_performance(payload)
            except (ValueError, TypeError):
                pass

        def detections(self, message):
            self.process("detections", message)

        def fire_targets(self, message):
            try:
                state.push_fire_registry(json.loads(message.data))
            except (ValueError, TypeError):
                pass

        def firefighter_targets(self, message):
            try:
                state.push_firefighter_registry(json.loads(message.data))
            except (ValueError, TypeError):
                pass

    node = PreviewNode()
    server = ThreadingHTTPServer((args.bind, args.port), handler_class(state, page))
    server.daemon_threads = True
    web_thread = threading.Thread(target=server.serve_forever, daemon=True)
    web_thread.start()
    node.get_logger().info(f"Read-only matched YOLO view: http://{args.bind}:{args.port}/")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
