"""Direct single-eye ROS observation with bounded geometry/inference/JPEG workers.

The camera driver still owns both sensors. This 2D observer does not claim stereo
depth. JPEG, boxes and effective K share the same full-resolution pixel space.
"""
import copy
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass

import cv2
from .node_common import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from .vision_task_gate import VisionTaskGate
from .dialogue_workload import yield_optional_observation
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Header, String
from rescue_interfaces.msg import Detection, DetectionArray

from .latest_slot import LatestSlot
from .lens_correction import LensCorrector
from .mipi_image import image_to_bgr, image_storage_view, stamp_nanoseconds
from .node_common import IMAGE_QOS, age, param, publish_json, spin
from .raw_image_cdr import decode_image_cdr
from .raw_image_relay import RawImageRelay


def rate(times):
    values = list(times)
    return ((len(values) - 1) / (values[-1] - values[0])
            if len(values) > 1 and values[-1] > values[0] else 0.0)


@dataclass(frozen=True)
class PreparedFrame:
    """One immutable handoff; header, pixels and geometry always travel together."""
    header: object
    sequence: int
    capture_size: tuple
    image: object
    lens_metadata: object
    started_at: float
    decoded_at: float
    geometry_at: float
    prepared_at: float
    detector_input: object
    # An idle observation has no pixels, but retains geometry established by a
    # real decoded/corrected frame. It must never be sent to inference or JPEG.
    pixel_size: tuple


class FastMipiPerception(Node):
    def __init__(self):
        super().__init__('fast_mipi_perception')
        cv2.setNumThreads(int(param(self, 'opencv_threads', 2)))
        self.max_fps = float(param(self, 'process_fps', 30.0))
        self.pipeline_enabled = bool(param(self, 'pipeline_enabled', True))
        self.raw_subscription = bool(param(self, 'raw_subscription', True))
        if not math.isfinite(self.max_fps) or not 0 < self.max_fps <= 30:
            raise ValueError('process_fps must be in (0,30]')
        self.observation_fps = float(param(self, 'observation_fps', 2.0))
        if not math.isfinite(self.observation_fps) or not 0 < self.observation_fps <= 30:
            raise ValueError('observation_fps must be in (0,30]')
        self.max_age = float(param(self, 'max_input_age_s', .5))
        self.output_size = (int(param(self, 'image_width', 1280)),
                            int(param(self, 'image_height', 1088)))
        if not all(0 < side <= limit for side, limit in zip(self.output_size, (1280, 1088))):
            raise ValueError('Output size must not exceed the real 1280x1088 sensor image')
        self.jpeg_quality = int(param(self, 'jpeg_quality', 90))
        if not 50 <= self.jpeg_quality <= 98:
            raise ValueError('jpeg_quality must be in [50,98]')
        self.input_rotation = float(param(self, 'input_rotation', 90.0))
        left_source = str(param(self, 'left_source_topic', '/image_left_raw'))
        right_source = str(param(self, 'right_source_topic', '/image_right_raw'))
        if bool(param(self, 'swap_eyes', True)):
            left_source, right_source = right_source, left_source
        self.source_topic = left_source
        self.lens = None
        if bool(param(self, 'lens_enabled', True)):
            self.lens = LensCorrector.from_json(param(self, 'lens_intrinsics', ''),
                role_to_source={'left': left_source, 'right': right_source},
                alpha=float(param(self, 'lens_alpha', 0.0)),
                remap_backend=str(param(self, 'remap_backend', 'native_or_opencv')),
                remap_threads=int(param(self, 'remap_threads', 4)))
        self.fire_only = bool(param(self, 'fire_only', False))
        self.classes = ['fire'] if self.fire_only else ['fire', 'firefighter', 'person']
        self.task_gate = VisionTaskGate(bool(param(self, 'mission_gating', False)))
        self.dialogue_fps = float(param(self, 'dialogue_process_fps', 6.0))
        if not math.isfinite(self.dialogue_fps) or not 4 <= self.dialogue_fps <= 30:
            raise ValueError('dialogue_process_fps must be in [4,30]')
        self.workload_mission = ({}, None)
        self.workload_dialogue = ({}, None)
        self.create_subscription(String, '/scene/dialogue_status', self.dialogue_state, 10)
        self.add_on_set_parameters_callback(self.set_task_parameter)
        self.create_subscription(String, '/mission/state', self.mission_state, 10)
        self.detector = None
        if bool(param(self, 'detector_enabled', True)):
            from .legacy_detector import BoardDetector
            self.detector = BoardDetector(param(self, 'model', ''), param(self, 'backend', 'native_or_pyeasy'),
                class_ids=(0,) if self.fire_only else None,
                fused_preprocess=bool(param(self, 'fused_preprocess', True)),
                preprocess_threads=int(param(self, 'preprocess_threads', 2)))
            self.get_logger().info('Inference backend: ' + self.detector.backend)
            if self.detector.runtime_fallback_reason:
                self.get_logger().warning('Native runtime unavailable: ' + self.detector.runtime_fallback_reason)
        self.images, self.prepared, self.previews = LatestSlot(), LatestSlot(), LatestSlot()
        self.stopping = threading.Event()
        self.stats_lock = threading.Lock()
        self.capture_times, self.processing_times, self.jpeg_times = (deque(maxlen=60) for _ in range(3))
        self.geometry_times = deque(maxlen=60)
        self.last_stamp_ns = -1
        self.last_input_at = None
        self.input_count = self.invalid_count = self.dropped_inputs = self.dropped_previews = 0
        self.dropped_prepared = self.expired_prepared = self.geometry_count = 0
        self.idle_geometry_count = 0
        self.idle_geometry = None
        self.jpeg_ms = 0.0
        self.jpeg_stages = {}
        self.jpeg_count = 0
        self.observation_count = 0
        self.last_observation_at = None
        self.performance = {}
        self.stage_errors = dict(capture='', geometry='', inference='', preview='')
        self.last_error = ''
        self.last_warning_at = {}
        self.detection_pub = self.create_publisher(DetectionArray, '/perception/detections', IMAGE_QOS)
        self.preview_pub = self.create_publisher(CompressedImage, '/vision/preview/compressed', IMAGE_QOS)
        self.observation_pub = self.create_publisher(Image, '/vision/observation/image', IMAGE_QOS)
        self.road_relay = RawImageRelay(self.source_topic, '/rescue/camera/road_raw')
        self.road_relay_pub = self.create_publisher(Image, self.road_relay.output_topic, IMAGE_QOS)
        self.road_relay_count = 0
        self.road_relay_error = ''
        self.road_relay_warning_at = None
        self.create_subscription(String, '/diagnostics/road', self.road_demand, 10)
        self.diagnostic_pub = self.create_publisher(String, '/diagnostics/perception', 10)
        self.camera_pub = self.create_publisher(String, '/diagnostics/camera', 10)
        self.geometry_pub = self.create_publisher(CameraInfo, '/vision/camera_info',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.published_geometry = None
        self.create_subscription(Image, self.source_topic,
            self.receive_raw if self.raw_subscription else self.receive,
            IMAGE_QOS, raw=self.raw_subscription)
        self.create_timer(1.0, self.report)
        self.workers = [threading.Thread(target=self.process_loop, name='vision-inference', daemon=True),
                        threading.Thread(target=self.preview_loop, name='vision-jpeg', daemon=True)]
        if self.pipeline_enabled:
            self.workers.insert(0, threading.Thread(target=self.geometry_loop,
                                                    name='vision-geometry', daemon=True))
        for worker in self.workers:
            worker.start()
        self.get_logger().info('Direct 2D observation of ' + self.source_topic +
            '; geometry pipeline=' + str(self.pipeline_enabled) +
            '; latest-frame workers; full-resolution JPEG; no metric depth')

    def receive_raw(self, serialized):
        try:
            accepted = self.receive(decode_image_cdr(serialized, header_factory=Header))
            if accepted:
                self.relay_road(serialized)
        except (ValueError, TypeError) as exc:
            self.invalid_count += 1
            self.warn(str(exc), stage='capture')

    def road_demand(self, message):
        try:
            value = json.loads(message.data)
        except (ValueError, TypeError):
            value = {}
        self.road_relay.update(value, self.get_clock().now().nanoseconds/1e9, time.monotonic())

    def relay_road(self, message):
        # The auxiliary observer must never take down the camera/YOLO pipeline
        # if its DDS publisher fails. Road state will expire independently.
        now = time.monotonic()
        try:
            if self.road_relay.due(now, self.road_relay_pub.get_subscription_count()):
                # rclpy accepts the original CDR bytes (or the original Image
                # in the reference non-raw path), without restamping/resizing.
                self.road_relay_pub.publish(message)
                self.road_relay_count += 1
                self.road_relay_error = ''
        except Exception as exc:
            self.road_relay_error = type(exc).__name__ + ': ' + str(exc)[:200]
            if self.road_relay_warning_at is None or now-self.road_relay_warning_at > 5:
                self.get_logger().warning('Road image relay failed: ' + self.road_relay_error)
                self.road_relay_warning_at = now

    def mission_state(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                value = {}
        except (ValueError, TypeError):
            value = {}
        received = time.monotonic()
        self.workload_mission = (value, received)
        self.task_gate.update(value, self.get_clock().now().nanoseconds/1e9, received)

    def dialogue_state(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                value = {}
        except (ValueError, TypeError):
            value = {}
        self.workload_dialogue = (value, time.monotonic())

    def effective_process_fps(self):
        # Tuples are replaced atomically; never mutate or restamp received facts.
        mission, mission_received = self.workload_mission
        dialogue, dialogue_received = self.workload_dialogue
        yielding = yield_optional_observation(mission, mission_received,
            dialogue, dialogue_received, self.get_clock().now().nanoseconds/1e9, time.monotonic())
        return min(self.max_fps, self.dialogue_fps) if yielding else self.max_fps

    def set_task_parameter(self, parameters):
        for parameter in parameters:
            if parameter.name == 'mission_gating' and type(parameter.value) is not bool:
                return SetParametersResult(successful=False, reason='mission_gating must be boolean')
        for parameter in parameters:
            if parameter.name == 'mission_gating':
                with self.task_gate.lock:
                    self.task_gate.enabled = parameter.value
        return SetParametersResult(successful=True)

    def yolo_allowed(self, header):
        capture = header.stamp.sec + header.stamp.nanosec/1e9
        return self.detector is not None and self.task_gate.allowed(capture, time.monotonic())

    def receive(self, message):
        try:
            stamp_ns = stamp_nanoseconds(message.header.stamp)
            if stamp_ns <= self.last_stamp_ns or not 0 <= age(self, message.header.stamp) <= self.max_age:
                raise ValueError('old/future/out-of-order raw image timestamp')
            if (message.width, message.height) != (1280, 1088):
                raise ValueError('Camera resolution differs from the calibrated 1280x1088 mode')
            image_storage_view(message.data, message.width, message.height, message.step, message.encoding)
            self.last_stamp_ns = stamp_ns
            self.last_input_at = time.monotonic()
            with self.stats_lock:
                self.capture_times.append(self.last_input_at)
                self.input_count += 1
            self.images.put(message)
            self.clear_error('capture')
            if not self.raw_subscription:
                self.relay_road(message)
            return True
        except (ValueError, TypeError) as exc:
            self.invalid_count += 1
            self.warn(str(exc), 'capture')
            return False

    def clear_error(self, stage):
        """Only a successful operation in the failing stage proves recovery."""
        with self.stats_lock:
            self.stage_errors[stage] = ''
            self.last_error = '; '.join(f'{name}: {error}' for name, error
                                       in self.stage_errors.items() if error)

    def warn(self, message, stage):
        now = time.monotonic()
        with self.stats_lock:
            self.stage_errors[stage] = message
            self.last_error = '; '.join(f'{name}: {error}' for name, error
                                       in self.stage_errors.items() if error)
            log_warning = now - self.last_warning_at.get(stage, 0.0) > 5
            if log_warning:
                self.last_warning_at[stage] = now
        if log_warning:
            self.get_logger().warning(f'{stage}: {message}')

    def geometry_loop(self):
        """Overlap CPU geometry with BPU work, retaining only the newest result.

        Cap preparation as well as inference: a slow detector cannot cause an
        unbounded queue or keep remapping at the full camera rate unnecessarily.
        Only this worker touches the lens map cache in pipelined mode.
        """
        version, next_allowed = 0, 0.0
        while not self.stopping.is_set():
            if self.stopping.wait(max(0.0, next_allowed - time.monotonic())):
                break
            selected = self.images.take_after(version)
            if selected is None:
                continue
            new_version, source = selected
            self.dropped_inputs += max(0, new_version - version - 1)
            version = new_version
            next_allowed = time.monotonic() + 1.0 / self.effective_process_fps()
            try:
                if not 0 <= age(self, source.header.stamp) <= self.max_age:
                    continue
                prepared = self.prepare(source, version)
                self.clear_error('geometry')
                if not self.stopping.is_set():
                    self.prepared.put(prepared)
            except Exception as exc:
                self.warn('Geometry frame rejected: ' + str(exc), 'geometry')

    def process_loop(self):
        version, next_allowed = 0, 0.0
        handoff = self.prepared if self.pipeline_enabled else self.images
        while not self.stopping.is_set():
            if self.stopping.wait(max(0.0, next_allowed - time.monotonic())):
                break
            selected = handoff.take_after(version)
            if selected is None:
                continue
            new_version, frame = selected
            skipped = max(0, new_version - version - 1)
            if self.pipeline_enabled:
                self.dropped_prepared += skipped
            else:
                self.dropped_inputs += skipped
            version = new_version
            next_allowed = time.monotonic() + 1.0 / self.effective_process_fps()
            try:
                if not 0 <= age(self, frame.header.stamp) <= self.max_age:
                    if self.pipeline_enabled:
                        self.expired_prepared += 1
                    continue
                if not self.pipeline_enabled:
                    try:
                        frame = self.prepare(frame, version)
                        self.clear_error('geometry')
                    except Exception as exc:
                        self.warn('Geometry frame rejected: ' + str(exc), 'geometry')
                        continue
                self.infer(frame)
                if not self.stopping.is_set():
                    self.clear_error('inference')
            except Exception as exc:
                # No fallback to incorrectly labelled pixels after a geometry
                # or runtime failure. Capture remains responsive.
                self.warn('Inference frame rejected: ' + str(exc), 'inference')

    def prepare(self, source, sequence):
        started = time.perf_counter()
        with self.stats_lock:
            geometry_failed = bool(self.stage_errors['geometry'])
        if (self.idle_geometry is not None and not geometry_failed
                and not self.yolo_allowed(source.header)
                and not self.preview_pub.get_subscription_count()):
            # Geometry cannot change at runtime: resolution, source eye,
            # calibration and output size are fixed startup parameters. Keep
            # disabled observations fresh without remapping unused pixels.
            # A previous geometry error still requires a real frame to recover.
            # OCR observation pixels are emitted only with active detection, so
            # an OCR subscriber alone does not invalidate this idle condition.
            pixel_size, lens_metadata = self.idle_geometry
            finished = time.perf_counter()
            prepared = PreparedFrame(copy.deepcopy(source.header), sequence,
                (source.width, source.height), None, lens_metadata,
                started, started, started, finished, None, pixel_size)
            with self.stats_lock:
                self.geometry_times.append(time.monotonic())
                self.geometry_count += 1
                self.idle_geometry_count += 1
            return prepared
        image = image_to_bgr(source.data, source.width, source.height, source.step, source.encoding)
        decoded_at = time.perf_counter()
        lens_metadata = None
        if self.lens:
            corrected = self.lens.correct(image, role='left', output_size=self.output_size,
                input_metadata={'rotation': self.input_rotation, 'source_topic': self.source_topic})
            image, lens_metadata = corrected.image, corrected.metadata
        elif image.shape[1::-1] != self.output_size:
            image = cv2.resize(image, self.output_size, interpolation=cv2.INTER_AREA)
        # Readers may overlap. Accidental in-place preprocessing must fail
        # instead of altering the pixels later encoded or sent to the detector.
        image.setflags(write=False)
        geometry_at = time.perf_counter()
        detector_input = (self.detector.prepare(image) if self.detector is not None
                          and hasattr(self.detector, 'prepare') and self.yolo_allowed(source.header) else None)
        prepared_at = time.perf_counter()
        prepared = PreparedFrame(copy.deepcopy(source.header), sequence,
            (source.width, source.height), image, lens_metadata,
            started, decoded_at, geometry_at, prepared_at, detector_input, image.shape[1::-1])
        self.idle_geometry = (prepared.pixel_size, lens_metadata)
        with self.stats_lock:
            self.geometry_times.append(time.monotonic())
            self.geometry_count += 1
        return prepared

    def process(self, source, sequence):
        """Sequential reference path uses the identical geometry and inference."""
        self.infer(self.prepare(source, sequence))

    def infer(self, frame):
        started = time.perf_counter()
        if self.stopping.is_set():
            return
        image = frame.image
        # Demand can return after prepare() chose an idle observation. Wait for
        # the next real frame; never infer/encode absent or invented pixels.
        enabled = image is not None and self.yolo_allowed(frame.header)
        if not enabled:
            items = []
        elif frame.detector_input is not None:
            items = self.detector.predict_prepared(frame.detector_input)
        else:
            items = self.detector.predict(image) if self.detector else []
        out = DetectionArray()
        out.header = copy.deepcopy(frame.header)
        out.header.frame_id = 'undistorted_left_optical' if self.lens else 'left_camera_optical'
        out.sequence = frame.sequence
        out.width, out.height = frame.pixel_size
        out.rectified = False
        # A mode change while BPU inference ran invalidates that observation.
        enabled = enabled and self.yolo_allowed(frame.header)
        if not enabled:
            items = []
        out.detector_available = enabled
        # Publish the actual corrected-image K. rectified=False means stereo
        # rectification is unverified; it must not make aiming use raw-camera K.
        if frame.lens_metadata is not None:
            matrix = tuple(frame.lens_metadata['camera_matrix'])
            identity = (out.header.frame_id, out.width, out.height, matrix)
            if self.published_geometry != identity:
                info = CameraInfo()
                info.header = copy.deepcopy(out.header)
                info.width, info.height = out.width, out.height
                info.distortion_model = 'plumb_bob'
                info.d = [0.0] * 5
                info.k = list(matrix)
                info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                info.p = [matrix[0], matrix[1], matrix[2], 0.,
                          matrix[3], matrix[4], matrix[5], 0.,
                          matrix[6], matrix[7], matrix[8], 0.]
                self.geometry_pub.publish(info)
                self.published_geometry = identity
        lens_state = 'undistorted_candidate' if self.lens else 'raw'
        out.status = ('candidate_model_board_accuracy_unverified' if enabled else 'detector_disabled_by_task_or_configuration')
        out.status += '; depth=uncalibrated; classes=' + ','.join(self.classes) + '; lens=' + lens_state
        for item in items:
            detection = Detection()
            detection.class_id, detection.class_name = item['class_id'], item['class_name']
            detection.confidence = float(item['confidence'])
            detection.bbox_xyxy = list(map(float, item['bbox_xyxy']))
            detection.depth_valid = False
            detection.depth_reason = 'uncalibrated'
            out.detections.append(detection)
        inferred_at = time.perf_counter()
        # Do not publish an old result after a long model/CPU interruption.
        if self.stopping.is_set():
            return
        if not 0 <= age(self, out.header.stamp) <= 1.0:
            raise ValueError('processing result exceeded one-second capture-age limit')
        self.detection_pub.publish(out)
        observation_ms = self.publish_observation(frame, out)
        if image is not None and self.preview_pub.get_subscription_count():
            # Numpy image is immutable from here until the JPEG worker finishes.
            self.previews.put((copy.deepcopy(out.header), image))
        ended = time.perf_counter()
        timing = self.detector.last_timings if enabled else {}
        with self.stats_lock:
            self.processing_times.append(time.monotonic())
            self.performance = dict(
                sequence=frame.sequence, stamp_sec=out.header.stamp.sec,
                pipeline_enabled=self.pipeline_enabled,
                raw_subscription=self.raw_subscription,
                enabled_classes=self.classes, detector_network_classes=3,
                detector_active=enabled, mission_gating=self.task_gate.enabled,
                pixel_processing=image is not None,
                lens=lens_state, lens_metadata=frame.lens_metadata,
                source_topic=self.source_topic,
                capture_resolution=list(frame.capture_size),
                preview_resolution=[out.width, out.height], output_resolution=[out.width, out.height],
                model_resolution=[640, 640], jpeg_quality=self.jpeg_quality,
                preprocess_backend=getattr(self.detector, 'preprocess_backend', 'disabled'),
                preprocess_path=timing.get('preprocess_path', 'disabled'),
                inference_backend=getattr(self.detector, 'backend', 'disabled'),
                input_frames_skipped=self.dropped_inputs, preview_frames_skipped=self.dropped_previews,
                prepared_frames_skipped=self.dropped_prepared,
                prepared_frames_expired=self.expired_prepared,
                image_age_s=age(self, frame.header.stamp),
                queue_wait_ms=round((started-frame.prepared_at)*1000, 2),
                processing_latency_ms=round((ended-frame.started_at)*1000, 2),
                stage_ms=dict(decode=round((frame.decoded_at-frame.started_at)*1000, 2),
                    lens=round((frame.geometry_at-frame.decoded_at)*1000, 2),
                    preprocess=round(timing.get('preprocess_s', 0)*1000, 2),
                    inference=round(timing.get('inference_s', 0)*1000, 2),
                    postprocess=round(timing.get('postprocess_s', 0)*1000, 2),
                    observation_publish=round(observation_ms, 2),
                    publish=round((ended-inferred_at)*1000, 2),
                    total=round(((frame.prepared_at-frame.started_at)+(ended-started))*1000, 2)))

    def publish_observation(self, frame, detections):
        """Occasional lossless OCR input from the exact inference pixel buffer.

        No pixel allocation/copy occurs without active detection, subscribers
        and a due deadline. CameraInfo and DetectionArray were published first.
        OCR joins by this original capture stamp and the same pixel-space ID.
        """
        now = time.monotonic()
        if (not detections.detector_available or frame.image is None
                or not self.observation_pub.get_subscription_count()
                or (self.last_observation_at is not None
                    and now - self.last_observation_at < 1.0 / self.observation_fps)):
            return 0.0
        started = time.perf_counter()
        if (self.stopping.is_set() or not self.yolo_allowed(frame.header)
                or not 0 <= age(self, frame.header.stamp) <= 1.0):
            return 0.0
        message = Image()
        message.header = copy.deepcopy(detections.header)
        message.width, message.height = detections.width, detections.height
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = message.width * 3
        # array.array owns this exact full-resolution copy. memoryview avoids
        # an additional temporary image.tobytes() allocation before ROS publish.
        message.data.frombytes(memoryview(frame.image).cast('B'))
        if (self.stopping.is_set() or not self.yolo_allowed(frame.header)
                or not 0 <= age(self, frame.header.stamp) <= 1.0
                or not self.observation_pub.get_subscription_count()):
            return (time.perf_counter() - started) * 1000
        self.observation_pub.publish(message)
        self.last_observation_at = time.monotonic()
        with self.stats_lock:
            self.observation_count += 1
        return (time.perf_counter() - started) * 1000

    def preview_loop(self):
        version = 0
        while not self.stopping.is_set():
            selected = self.previews.take_after(version)
            if selected is None:
                continue
            new_version, (header, image) = selected
            self.dropped_previews += max(0, new_version - version - 1)
            version = new_version
            try:
                if not 0 <= age(self, header.stamp) <= 1.0 or not self.preview_pub.get_subscription_count():
                    continue
                started = time.perf_counter()
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                encoded_at = time.perf_counter()
                if not ok:
                    raise ValueError('JPEG encoder failed')
                if self.stopping.is_set():
                    return
                if not 0 <= age(self, header.stamp) <= 1.0 or not self.preview_pub.get_subscription_count():
                    continue
                message = CompressedImage()
                message.header = header
                message.format = 'bgr8; jpeg compressed bgr8'
                message.data.frombytes(encoded)
                copied_at = time.perf_counter()
                self.preview_pub.publish(message)
                published_at = time.perf_counter()
                with self.stats_lock:
                    self.jpeg_ms = round((time.perf_counter() - started)*1000, 2)
                    self.jpeg_stages = dict(encode=round((encoded_at-started)*1000, 2),
                        message_copy=round((copied_at-encoded_at)*1000, 2),
                        publish=round((published_at-copied_at)*1000, 2),
                        stats_wait=round((time.perf_counter()-published_at)*1000, 2))
                    self.jpeg_count += 1
                    self.jpeg_times.append(time.monotonic())
                self.clear_error('preview')
            except Exception as exc:
                self.warn('Preview frame rejected: ' + str(exc), 'preview')

    def report(self):
        with self.stats_lock:
            now = time.monotonic()
            fresh_rate = lambda times: rate(times) if times and now-times[-1] < 2.0 else 0.0
            perf = copy.deepcopy(self.performance)
            if perf:
                effective_fps = self.effective_process_fps()
                perf.update(processing_fps=round(fresh_rate(self.processing_times), 2),
                    geometry_fps=round(fresh_rate(self.geometry_times), 2),
                    prepared_frames=self.geometry_count,
                    idle_geometry_frames=self.idle_geometry_count,
                    capture_fps=round(fresh_rate(self.capture_times), 2),
                    configured_process_fps=self.max_fps,
                    effective_process_fps=effective_fps,
                    dialogue_yielding=effective_fps < self.max_fps,
                    preview_encode_fps=round(fresh_rate(self.jpeg_times), 2),
                    preview_encoded_frames=self.jpeg_count,
                    preview_subscribers=self.preview_pub.get_subscription_count(),
                    observation_published_frames=self.observation_count,
                    observation_rate_limit=self.observation_fps,
                    observation_subscribers=self.observation_pub.get_subscription_count(),
                    road_relay_frames=self.road_relay_count,
                    road_relay_rate=self.road_relay.rate,
                    road_relay_error=self.road_relay_error,
                    jpeg_stage_ms=dict(self.jpeg_stages),
                    stage_errors={name: error for name, error in self.stage_errors.items() if error},
                    processing_last_age_s=now-self.processing_times[-1] if self.processing_times else None)
                perf['stage_ms']['jpeg'] = self.jpeg_ms
            input_age = None if self.last_input_at is None else time.monotonic() - self.last_input_at
            health = dict(ok=input_age is not None and input_age < 1.0 and not self.last_error,
                source='official_mipi_ros_topic', source_topic=self.source_topic,
                scope='physical_left_only', right_eye_checked=False,
                last_input_age_s=input_age, received=self.input_count, invalid=self.invalid_count,
                stage_errors={name: error for name, error in self.stage_errors.items() if error},
                reason=self.last_error or ('ok' if input_age is not None and input_age < 1.0
                                           else 'no_recent_input'), metric_depth_verified=False,
                synchronization_verified=False)
        if perf:
            publish_json(self.diagnostic_pub, perf)
        publish_json(self.camera_pub, health)

    def destroy_node(self):
        self.stopping.set()
        self.images.close()
        self.prepared.close()
        self.previews.close()
        for worker in self.workers:
            worker.join(timeout=5.0)
        if self.detector is not None and hasattr(self.detector, 'close'):
            if not any(worker.is_alive() for worker in self.workers):
                self.detector.close()
        return super().destroy_node()


def main():
    spin(FastMipiPerception)
