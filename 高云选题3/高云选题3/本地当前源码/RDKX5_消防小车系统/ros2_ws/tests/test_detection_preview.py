"""No ROS/camera needed: prevents stale or mismatched detection overlays."""
import base64
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from detection_preview import PreviewState, handler_class, jpeg_dimensions, pack_frame


def jpeg_header(width=1280, height=1088, progressive=False):
    """Structural SOF fixture; pixel decoding is deliberately not exercised here."""
    return (b"\xff\xd8\xff\xe0\x00\x04ok\xff" + (b"\xc2" if progressive else b"\xc0")
            + struct.pack(">HBHHB", 11, 8, height, width, 1)
            + b"\x01\x11\x00\xff\xd9")


JPEG = jpeg_header()
# Complete 1x1 grayscale JPEG, independent of OpenCV/Pillow/ROS availability.
ONE_PIXEL = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0aHBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/wAALCAABAAEBAREA/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/9oACAEBAAA/AEr/2Q==")


def messages(sequence=1, second=100, nanosec=0, frame_id="left_camera_optical", boxes=True):
    header = NS(stamp=NS(sec=second, nanosec=nanosec), frame_id=frame_id)
    image = NS(header=header, sequence=sequence, left=NS(width=1280, height=1088))
    detection = NS(class_id=0, class_name="fire", confidence=.9,
                   bbox_xyxy=[20, 30, 80, 90], depth_valid=False, depth_m=0.)
    result = NS(header=header, sequence=sequence, width=1280, height=1088,
                detector_available=True, status="candidate; depth=uncalibrated",
                detections=[detection] if boxes else [])
    return image, result


def compressed_messages(**kwargs):
    source, result = messages(**kwargs)
    return NS(header=source.header, format="bgr8; jpeg compressed bgr8", data=JPEG), result


def registry(sequence=1, second=100, nanosec=0, **overrides):
    value = dict(frame_valid=True, capture_stamp_ns=second*1_000_000_000+nanosec,
                 sequence=sequence, registry_session="session-1", coordinate_frame="left_camera_optical",
                 image_size_wh=[1280, 1088], targets=[dict(target_id=7, confidence=.95,
                 bbox_xyxy=[20, 30, 80, 90], selectable=True)])
    value.update(overrides)
    return value


class FireRegistryPreviewTests(unittest.TestCase):
    def test_registry_before_pixels_matches_exact_nanosecond_sequence_and_frame(self):
        for delta in ({}, {"nanosec": 2}, {"sequence": 2}, {"coordinate_frame": "other"},
                      {"image_size_wh": [640, 544]}):
            state = PreviewState()
            state.push_fire_registry(registry(**dict({"second": 1_790_000_000, "nanosec": 1}, **delta)), now=0)
            frame, result = compressed_messages(second=1_790_000_000, nanosec=1)
            state.commit(frame, result, JPEG, now=.1)
            packet, jpeg = state.snapshot_packet(now=.2, wall_ns=1_790_000_000_200_000_000)
            self.assertEqual("fire_registry" in packet["frame"], not bool(delta), delta)
            self.assertEqual(jpeg, JPEG)
            if not delta:
                self.assertEqual(packet["frame"]["fire_registry"]["targets"][0]["target_id"], 7)

    def test_late_registry_is_metadata_only_and_does_not_refresh_fps_or_old_snapshot(self):
        state = PreviewState()
        frame, result = compressed_messages()
        state.commit(frame, result, JPEG, now=0)
        first, _ = state.snapshot_packet(now=.1, wall_ns=100_100_000_000)
        token = first["frame"]["token"]
        state.push_fire_registry(registry(), now=.2)
        update, jpeg = state.snapshot_packet(after=token, now=.3, wall_ns=100_300_000_000)
        self.assertIsNone(update["frame"])
        self.assertEqual(jpeg, b"")
        self.assertEqual(update["fire_registry_update"]["frame_token"], token)
        self.assertNotIn("fire_registry", first["frame"])
        self.assertEqual(state.latest_at, 0)
        self.assertEqual(state.paired_frames, 1)
        state.push_fire_registry(registry(), now=3.1)
        stale, _ = state.snapshot_packet(after=token, now=3.2, wall_ns=103_200_000_000)
        self.assertEqual(stale["state"], "stale")
        self.assertIsNone(stale["fire_registry_update"])

    def test_old_registry_cannot_replace_new_frame_geometry(self):
        state = PreviewState()
        frame, result = compressed_messages(sequence=2, second=101)
        state.commit(frame, result, JPEG, now=1)
        state.push_fire_registry(registry(), now=1.1)
        self.assertNotIn("fire_registry", state.latest)
        self.assertEqual(state.latest["sequence"], 2)

    def test_invalid_identity_and_boxes_are_not_accepted(self):
        state = PreviewState()
        for delta in ({"capture_stamp_ns": 100e9}, {"sequence": True}, {"frame_valid": False},
                      {"registry_session": ""}, {"coordinate_frame": ""}, {"image_size_wh": [1280, False]}):
            self.assertFalse(state.push_fire_registry(registry(**delta), now=0))
        bad = dict(target_id=8, confidence=.96, bbox_xyxy=[-1, 30, 80, 90], selectable=True)
        document = registry()
        document['targets'].append(bad)
        state.push_fire_registry(document, now=0)
        document['targets'][0]['bbox_xyxy'][0] = 900
        frame, result = compressed_messages()
        state.commit(frame, result, JPEG, now=.1)
        self.assertEqual(state.latest['fire_registry']['targets'][0]['bbox_xyxy'][0], 20)
        self.assertEqual(len(state.latest['fire_registry']['targets']), 1)

    def test_registry_cache_bounded_expires_and_clears_on_stream_restart(self):
        state = PreviewState()
        for sequence in range(100):
            state.push_fire_registry(registry(sequence=sequence), now=sequence/100)
        self.assertEqual(len(state.fire_registries), 64)
        state.prune(now=5)
        self.assertFalse(state.fire_registries)
        state.push_fire_registry(registry(), now=5)
        state.set_stream(False, now=5.1)
        self.assertFalse(state.fire_registries)
        self.assertFalse(state.push_fire_registry(registry(), now=5.2))


class PreviewTests(unittest.TestCase):
    def test_exact_identity_required_and_either_arrival_order(self):
        for delta in ({"sequence": 2}, {"nanosec": 1}, {"frame_id": "other"}):
            state = PreviewState(image_format="stereo")
            frame, result = messages()
            _, wrong = messages(**delta)
            self.assertIsNone(state.push_frame(frame, 0))
            self.assertIsNone(state.push_detections(wrong, .1))
            self.assertEqual(state.push_detections(result, .2), (frame, result))
        state = PreviewState(image_format="stereo")
        self.assertIsNone(state.push_detections(result, 0))
        self.assertEqual(state.push_frame(frame, .1), (frame, result))

    def test_size_mismatch_dropped(self):
        frame, result = messages()
        result.width = 640
        state = PreviewState(image_format="stereo")
        state.push_frame(frame, 0)
        self.assertIsNone(state.push_detections(result, .1))
        self.assertEqual(len(state.frames), 0)
        self.assertEqual(state.snapshot(now=.2, wall_ns=100_000_000_000)["state"], "waiting")

    def test_queue_bound_and_expiry(self):
        state = PreviewState(image_format="stereo")
        for sequence in range(10):
            state.push_frame(messages(sequence=sequence)[0], sequence * .1)
        self.assertEqual(len(state.frames), 4)
        self.assertIsNone(state.push_detections(messages(sequence=0)[1], 1))
        self.assertIsNone(state.push_detections(messages(sequence=9)[1], 5))
        self.assertEqual(len(state.frames), 0)
        state.prune(9)
        self.assertEqual(len(state.detections), 0)

    def test_empty_detection_replaces_previous_boxes_and_after_token(self):
        state = PreviewState(image_format="stereo")
        frame, result = messages()
        state.commit(frame, result, JPEG, 0)
        first = state.snapshot(now=.1, wall_ns=100_100_000_000)
        self.assertEqual(len(first["frame"]["detections"]), 1)
        frame, result = messages(sequence=2, second=101, boxes=False)
        state.commit(frame, result, JPEG, 1)
        latest = state.snapshot(after=first["frame"]["token"], now=1.1, wall_ns=101_100_000_000)
        self.assertEqual(latest["frame"]["detections"], [])
        self.assertEqual(latest["state"], "live")
        self.assertEqual(latest["fps"], 1.)
        unchanged = state.snapshot(after=latest["frame"]["token"], now=1.2, wall_ns=101_200_000_000)
        self.assertIsNone(unchanged["frame"])
        self.assertTrue(unchanged["detector_available"])

    def test_stale_capture_update_future_and_restart(self):
        state = PreviewState(image_format="stereo")
        self.assertEqual(state.snapshot(now=0, wall_ns=0)["state"], "waiting")
        frame, result = messages()
        state.commit(frame, result, JPEG, 0)
        for now, wall in ((.1, 104_000_000_000), (4, 101_000_000_000), (.1, 99_999_999_999)):
            self.assertEqual(state.snapshot(now=now, wall_ns=wall)["state"], "stale")
        self.assertFalse(state.commit(frame, result, JPEG, 1))
        frame, result = messages(sequence=1, second=105)
        self.assertTrue(state.commit(frame, result, JPEG, 2))
        self.assertEqual(state.paired_frames, 2)
        self.assertEqual(state.snapshot(now=2, wall_ns=105_000_000_000)["state"], "live")

    def test_stereo_commit_rejects_encoded_size_change(self):
        frame, result = messages()
        with self.assertRaisesRegex(ValueError, "Encoded JPEG dimensions"):
            PreviewState(image_format="stereo").commit(frame, result, jpeg_header(640, 544), 0)


class CompressedPreviewTests(unittest.TestCase):
    def test_no_sequence_is_invented_for_standard_compressed_image(self):
        frame, result = compressed_messages(sequence=219)
        self.assertFalse(hasattr(frame, "sequence"))
        state = PreviewState()
        self.assertIsNone(state.push_frame(frame, 0))
        self.assertEqual(state.push_detections(result, .1), (frame, result))
        state.commit(frame, result, frame.data, .2)
        data, jpeg = state.snapshot_packet(now=.3, wall_ns=100_300_000_000)
        self.assertEqual(data["frame"]["sequence"], 219)
        self.assertEqual(jpeg, JPEG)
        self.assertNotIn("jpeg_base64", data["frame"])

    def test_exact_header_in_either_arrival_order(self):
        for delta in ({"second": 101}, {"nanosec": 1}, {"frame_id": "other"}):
            state = PreviewState()
            frame, result = compressed_messages()
            wrong, _ = compressed_messages(**delta)
            state.push_detections(result, 0)
            self.assertIsNone(state.push_frame(wrong, .1))
            self.assertEqual(state.push_frame(frame, .2), (frame, result))

    def test_actual_jpeg_dimensions_required(self):
        state = PreviewState()
        frame, result = compressed_messages()
        frame.data = jpeg_header(640, 544)
        state.push_frame(frame, 0)
        self.assertIsNone(state.push_detections(result, .1))
        self.assertEqual(state.status, "frame_detection_size_mismatch")
        self.assertIsNone(state.latest)
        with self.assertRaisesRegex(ValueError, "dimensions"):
            state.commit(frame, result, frame.data, .2)

    def test_compressed_commit_rejects_other_image_payload_or_header(self):
        frame, result = compressed_messages()
        state = PreviewState()
        with self.assertRaisesRegex(ValueError, "payload differs"):
            state.commit(frame, result, jpeg_header(progressive=True), 0)
        _, wrong = compressed_messages(nanosec=1)
        with self.assertRaisesRegex(ValueError, "identity differ"):
            state.commit(frame, wrong, frame.data, 0)

    def test_compressed_cache_bounded_and_expired(self):
        state = PreviewState(queue_size=2)
        for second in range(100, 110):
            state.push_frame(compressed_messages(second=second)[0], (second - 100) * .1)
        self.assertEqual(len(state.frames), 2)
        self.assertIsNone(state.push_detections(compressed_messages(second=100)[1], 1))
        state.prune(5)
        self.assertFalse(state.frames)
        self.assertFalse(state.detections)

    def test_atomic_packet_retains_pair_when_a_new_result_arrives(self):
        state = PreviewState()
        frame, result = compressed_messages()
        state.commit(frame, result, frame.data, 0)
        first, first_jpeg = state.snapshot_packet(now=.1, wall_ns=100_100_000_000)
        second, empty = compressed_messages(sequence=2, second=101, boxes=False)
        second.data = jpeg_header(progressive=True)
        state.commit(second, empty, second.data, 1)
        wire = pack_frame(first, first_jpeg)
        length = struct.unpack(">I", wire[:4])[0]
        self.assertEqual(json.loads(wire[4:4 + length])["frame"]["sequence"], 1)
        self.assertEqual(wire[4 + length:], frame.data)
        current, current_jpeg = state.snapshot_packet(now=1.1, wall_ns=101_100_000_000)
        self.assertEqual(current["frame"]["detections"], [])
        self.assertEqual(current_jpeg, second.data)
        metadata, no_image = state.snapshot_packet(current["frame"]["token"], now=1.2,
                                                  wall_ns=101_200_000_000)
        self.assertIsNone(metadata["frame"])
        self.assertEqual(no_image, b"")

    def test_base64_is_only_generated_for_explicit_legacy_snapshot(self):
        state = PreviewState()
        frame, result = compressed_messages()
        state.commit(frame, result, frame.data, 0)
        with patch("detection_preview.base64.b64encode", side_effect=AssertionError("extra encode")):
            metadata, jpeg = state.snapshot_packet(now=.1, wall_ns=100_100_000_000)
            pack_frame(metadata, jpeg)
        old = state.snapshot(now=.1, wall_ns=100_100_000_000)
        self.assertEqual(base64.b64decode(old["frame"]["jpeg_base64"]), JPEG)
        self.assertNotIn("jpeg_base64", state.latest)

    def test_stale_source_cannot_be_hidden_by_recent_commit(self):
        state = PreviewState()
        frame, result = compressed_messages()
        state.commit(frame, result, frame.data, 10)
        for now, wall in ((10.1, 104_000_000_000), (14, 101_000_000_000), (10.1, 99_000_000_000)):
            metadata, _ = state.snapshot_packet(now=now, wall_ns=wall)
            self.assertEqual(metadata["state"], "stale")
        self.assertIsNone(state.push_frame(frame, 11))
        self.assertFalse(state.commit(frame, result, frame.data, 11))

    def test_invalid_formats_timestamps_and_dimensions(self):
        state = PreviewState()
        frame, result = compressed_messages()
        frame.format = "png"
        with self.assertRaises(ValueError):
            state.push_frame(frame, 0)
        frame, _ = compressed_messages(nanosec=1_000_000_000)
        with self.assertRaises(ValueError):
            state.push_frame(frame, 0)
        with self.assertRaises(ValueError):
            PreviewState(image_format="unknown")

    def test_jpeg_dimensions_from_complete_image_and_structural_headers(self):
        self.assertEqual(jpeg_dimensions(ONE_PIXEL), (1, 1))
        self.assertEqual(jpeg_dimensions(JPEG), (1280, 1088))
        self.assertEqual(jpeg_dimensions(jpeg_header(640, 544, True)), (640, 544))
        self.assertEqual(jpeg_dimensions(bytearray(JPEG)), (1280, 1088))
        for bad in (b"", b"not-jpeg", JPEG[:12], b"\xff\xd8\xff\xe0\x00\x01", jpeg_header(0, 1088)):
            with self.subTest(jpeg=bad), self.assertRaises(ValueError):
                jpeg_dimensions(bad)


class PreviewHTTPTests(unittest.TestCase):
    def setUp(self):
        self.state = PreviewState()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler_class(self.state, b"preview-page"))
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)

    def tearDown(self):
        self.client.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def publish(self):
        stamp = time.time_ns()
        frame, result = compressed_messages(second=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000)
        self.state.commit(frame, result, frame.data)

    def test_binary_endpoint_unchanged_poll_and_keepalive(self):
        self.publish()
        self.client.request("GET", "/api/frame")
        response = self.client.getresponse()
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Content-Type"), "application/octet-stream")
        self.assertEqual(response.getheader("Cache-Control"), "no-store")
        body = response.read()
        length = struct.unpack(">I", body[:4])[0]
        metadata = json.loads(body[4:4 + length])
        self.assertEqual(body[4 + length:], JPEG)
        self.assertEqual(metadata["state"], "live")
        socket = self.client.sock
        self.client.request("GET", "/api/frame?after=" + metadata["frame"]["token"])
        response = self.client.getresponse()
        body = response.read()
        length = struct.unpack(">I", body[:4])[0]
        self.assertIsNone(json.loads(body[4:4 + length])["frame"])
        self.assertEqual(len(body), 4 + length)
        self.assertIs(self.client.sock, socket)

    def test_accepted_keepalive_socket_disables_delayed_ack_stalls(self):
        options = []
        original = self.server.RequestHandlerClass

        class InspectHandler(original):
            def setup(self):
                super().setup()
                options.append(self.connection.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY))

        self.server.RequestHandlerClass = InspectHandler
        for _ in range(2):
            self.client.request("GET", "/api/frame?after=unchanged")
            response = self.client.getresponse()
            self.assertEqual(response.status, 200)
            body = response.read()
            size = struct.unpack(">I", body[:4])[0]
            self.assertEqual(len(body), 4 + size)
            self.assertIsNone(json.loads(body[4:])["frame"])
        self.assertEqual(options, [1])

    def test_legacy_endpoint_and_head(self):
        self.publish()
        self.client.request("GET", "/api/state")
        response = self.client.getresponse()
        data = json.loads(response.read())
        self.assertEqual(base64.b64decode(data["frame"]["jpeg_base64"]), JPEG)
        self.client.request("HEAD", "/api/frame")
        response = self.client.getresponse()
        self.assertGreater(int(response.getheader("Content-Length")), len(JPEG))
        self.assertEqual(response.read(), b"")

    def test_waiting_packet_and_page(self):
        self.client.request("GET", "/api/frame")
        response = self.client.getresponse()
        body = response.read()
        length = struct.unpack(">I", body[:4])[0]
        self.assertEqual(json.loads(body[4:])["state"], "waiting")
        self.assertEqual(len(body), 4 + length)
        self.client.request("GET", "/")
        self.assertEqual(self.client.getresponse().read(), b"preview-page")

    def test_global_switch_off_sends_no_images_and_poll_cannot_reenable(self):
        self.publish()
        self.client.request("POST", "/api/stream", body=json.dumps({"enabled": False}),
                            headers={"Content-Type": "application/json"})
        response = self.client.getresponse()
        self.assertEqual(response.status, 200)
        self.assertFalse(json.loads(response.read())["stream"]["enabled"])
        self.client.request("GET", "/api/state")
        data = json.loads(self.client.getresponse().read())
        self.assertIsNone(data["frame"])
        self.assertEqual(data["state"], "paused")
        self.client.request("GET", "/api/frame")
        body = self.client.getresponse().read()
        length = struct.unpack(">I", body[:4])[0]
        metadata = json.loads(body[4:])
        self.assertFalse(metadata["stream"]["enabled"])
        self.assertIsNone(metadata["frame"])
        self.assertEqual(len(body), 4 + length)
        self.client.request("POST", "/api/stream", body=json.dumps({"enabled": True}),
                            headers={"Content-Type": "application/json"})
        data = json.loads(self.client.getresponse().read())
        self.assertTrue(data["stream"]["enabled"])
        self.assertIsNone(data["frame"])

    def test_status_only_api_does_not_encode_or_renew_lease(self):
        self.publish()
        lease = self.state.lease_at
        with patch("detection_preview.base64.b64encode", side_effect=AssertionError("unexpected base64")):
            self.client.request("GET", "/api/state?images=0")
            data = json.loads(self.client.getresponse().read())
        self.assertIsNone(data["frame"])
        self.assertEqual(self.state.lease_at, lease)

    def test_switch_requires_boolean(self):
        self.client.request("POST", "/api/stream", body=json.dumps({"enabled": "false"}),
                            headers={"Content-Type": "application/json"})
        response = self.client.getresponse()
        self.assertEqual(response.status, 400)
        response.read()
        self.assertTrue(self.state.stream_requested)


class StreamLeaseTests(unittest.TestCase):
    def test_lease_expiry_clears_frames_and_requests_alone_resume(self):
        state = PreviewState(lease_s=5)
        state.touch_stream(now=0)
        frame, result = compressed_messages()
        state.commit(frame, result, frame.data, now=0)
        self.assertTrue(state.stream_target(now=4.9)[0])
        self.assertFalse(state.stream_target(now=5.1)[0])
        self.assertTrue(state.stream_requested)
        self.assertFalse(state.frames)
        self.assertIsNone(state.latest)
        self.assertFalse(state.latest_jpeg)
        state.snapshot(now=6, wall_ns=106_000_000_000)
        self.assertFalse(state.stream_active)
        state.touch_stream(now=6, wall_ns=106_000_000_000)
        self.assertTrue(state.stream_target(now=6)[0])
        self.assertIsNone(state.push_frame(frame, now=6))
        fresh, fresh_result = compressed_messages(second=107)
        state.push_detections(fresh_result, now=6.1)
        self.assertEqual(state.push_frame(fresh, now=6.2), (fresh, fresh_result))

    def test_off_on_rejects_old_callbacks_in_progress_and_old_capture(self):
        state = PreviewState()
        old_generation = state.stream_generation
        old, result = compressed_messages(second=100)
        state.push_frame(old, now=0, generation=old_generation)
        state.set_stream(False, now=.1)
        self.assertFalse(state.frames)
        state.touch_stream(now=.2)
        self.assertFalse(state.stream_active)
        state.set_stream(True, now=.3, wall_ns=101_000_000_000)
        fresh, result = compressed_messages(second=102)
        self.assertIsNone(state.push_frame(fresh, now=.4, generation=old_generation))
        self.assertFalse(state.commit(fresh, result, fresh.data, now=.4, generation=old_generation))
        self.assertIsNone(state.push_frame(old, now=.4, generation=state.stream_generation))
        self.assertIsNone(state.push_frame(fresh, now=.5, generation=state.stream_generation))
        self.assertEqual(state.push_detections(result, now=.6), (fresh, result))

    def test_disable_does_not_destroy_diagnostic_status(self):
        state = PreviewState()
        state.update_performance({"processing_fps": 19.5})
        now = time.monotonic()
        state.set_stream(False, now=now)
        metadata, jpeg = state.snapshot_packet(now=now + .1)
        self.assertEqual(metadata["performance"]["processing_fps"], 19.5)
        self.assertEqual(metadata["state"], "paused")
        self.assertEqual(jpeg, b"")


if __name__ == "__main__":
    unittest.main()
