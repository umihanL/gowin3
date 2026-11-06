"""Low-rate PP-OCRv4 reusing the physical-left MIPI and existing YOLO stream."""
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
from .node_common import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from rescue_interfaces.msg import DetectionArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from .building_ocr import OCRReader
from .dialogue_workload import yield_optional_observation
from .fire_aim_geometry import CORRECTED_FRAME, corrected_camera
from .lens_correction import LensCorrector
from .mipi_building_ocr import (BuildingObservation, ExactCaptureJoin,
                              corrected_observation_image, observation_signature)
from .mipi_image import image_to_bgr, stamp_nanoseconds
from .node_common import IMAGE_QOS, param, read_json, publish_json, spin
from .raw_image_cdr import decode_image_cdr


class MipiBuildingOCR(Node):
    def __init__(self):
        super().__init__('mipi_building_ocr')
        self.enabled = bool(param(self, 'ocr_enabled', True))
        self.fixed_camera_preview = bool(param(self, 'fixed_camera_preview', False))
        self.require_settled_pose = bool(param(self, 'require_settled_pose', True)) and not self.fixed_camera_preview
        self.source_topic = str(param(self, 'source_topic', '/image_right_raw'))
        self.corrected_input = bool(param(self, 'corrected_input', False))
        self.input_topic = (str(param(self, 'corrected_topic', '/vision/observation/image'))
                            if self.corrected_input else self.source_topic)
        self.camera = None
        self.interval = float(param(self, 'ocr_interval_s', 1.))
        self.max_input_age = float(param(self, 'max_input_age_s', .5))
        self.observation = BuildingObservation(read_json(param(self, 'buildings_config', '')),
            hits=int(param(self, 'confirmation_hits', 3)), interval_s=self.interval,
            max_result_age_s=float(param(self, 'max_result_age_s', 3.)))
        self.size = (int(param(self, 'image_width', 1280)), int(param(self, 'image_height', 1088)))
        if not .1 <= self.max_input_age <= 1 or self.size != (1280, 1088):
            raise ValueError('OCR requires current 1280x1088 geometry and a bounded input age')
        self.rotation = float(param(self, 'input_rotation', 90.))
        self.lens = None
        if bool(param(self, 'lens_enabled', True)) and not self.corrected_input:
            # The native fixed-map kernel uses the same interpolation and
            # border rules as OpenCV. Keep one CPU worker to avoid competing
            # with the high-rate YOLO geometry and the OCR inference worker.
            self.lens = LensCorrector.from_json(param(self, 'lens_intrinsics', ''),
                role_to_source={'left': self.source_topic},
                alpha=float(param(self, 'lens_alpha', 0.)),
                remap_backend=str(param(self, 'ocr_remap_backend', 'native_or_opencv')),
                remap_threads=int(param(self, 'ocr_remap_threads', 1)))
        self.coordinate_frame = CORRECTED_FRAME if self.corrected_input or self.lens else 'left_camera_optical'
        cv2.setNumThreads(1)
        self.reader, self.reader_error = None, ''
        if self.enabled:
            try:
                self.reader = OCRReader(param(self, 'ocr_models', ''),
                    threads=int(param(self, 'ocr_threads', 1)),
                    analysis_max_side=int(param(self, 'ocr_analysis_max_side', 512)),
                    max_text_regions=int(param(self, 'ocr_max_text_regions', 6)),
                    use_angle_cls=bool(param(self, 'ocr_angle_classification', False)))
            except Exception as exc:
                self.reader_error = type(exc).__name__ + ': ' + str(exc)
                self.get_logger().error('OCR unavailable: ' + self.reader_error)
        self.join = ExactCaptureJoin()
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.pending = None
        self.mission, self.mission_received = {}, None
        self.dialogue, self.dialogue_received = {}, None
        self.yielding_to_dialogue = False
        self.signature = None
        self.generation = 0
        self.active_after = math.inf
        self.raw_subscription = None
        self.next_allowed = 0.
        self.last_output_capture = None
        self.last_invalid_reason = None
        self.last_processing_ms = None
        self.counts = dict(raw_frames=0, corrected_frames=0, submitted=0, completed=0, expired=0, invalid=0)
        self.output = self.create_publisher(String, '/scene/ocr', 10)
        self.diagnostics = self.create_publisher(String, '/diagnostics/building_ocr', 10)
        self.create_subscription(String, '/mission/state', self.on_mission, 10)
        self.create_subscription(String, '/scene/dialogue_status', self.on_dialogue, 10)
        self.create_subscription(DetectionArray, '/perception/detections', self.on_detection, IMAGE_QOS)
        if self.corrected_input:
            geometry_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                      reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(CameraInfo, '/vision/camera_info', self.on_camera_info, geometry_qos)
        self.create_timer(.1, self.tick)
        self.create_timer(1., self.report)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def invalidate(self, reason):
        if reason != self.last_invalid_reason:
            publish_json(self.output, self.observation.invalid(self.now(), reason, self.generation))
            self.last_invalid_reason = reason
        self.last_output_capture = None

    def refresh(self):
        now = self.now()
        receive_age = math.inf if self.mission_received is None else time.monotonic()-self.mission_received
        signature = (observation_signature(self.mission, now, receive_age,
                         require_settled_pose=self.require_settled_pose)
                     if self.reader is not None else None)
        self.yielding_to_dialogue = yield_optional_observation(
            self.mission, self.mission_received, self.dialogue, self.dialogue_received,
            now, time.monotonic())
        if self.yielding_to_dialogue:
            signature = None
        if self.corrected_input:
            signature = ((signature, self.camera.doc['calibration_id'])
                         if signature is not None and self.camera is not None else None)
        if signature != self.signature:
            self.signature = signature
            self.generation += 1
            self.active_after = now if signature is not None else math.inf
            self.join.clear()
            self.observation.reset()
            self.invalidate('task_or_pose_changed')
        self.refresh_image_subscription()
        return signature is not None

    def refresh_image_subscription(self):
        """Request full-resolution pixels only while acquiring the next sample.

        OCR consumes one exact capture at a time. Keeping a DDS reader during
        inference would force the detector to serialize every 4 MB corrected
        image while none can be used. Releasing it also lets the producer skip
        that work when no other observer needs the original pixels.
        """
        acquiring = (self.signature is not None and self.pending is None
                     and time.monotonic() >= self.next_allowed)
        if acquiring and self.raw_subscription is None:
            self.raw_subscription = self.create_subscription(Image, self.input_topic,
                self.on_raw, IMAGE_QOS, raw=True)
        elif not acquiring and self.raw_subscription is not None:
            self.destroy_subscription(self.raw_subscription)
            self.raw_subscription = None
            self.join.clear()

    def on_camera_info(self, msg):
        try:
            self.camera = corrected_camera(msg, CORRECTED_FRAME, self.size)
        except (ValueError, TypeError, AttributeError):
            self.camera = None
        # A changed geometry invalidates every cached or in-flight observation.
        self.refresh()

    def on_mission(self, msg):
        try:
            self.mission = json.loads(msg.data)
            self.mission_received = time.monotonic()
        except (TypeError, ValueError):
            self.mission, self.mission_received = {}, None
        self.refresh()

    def on_dialogue(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Dialogue status must be an object')
            self.dialogue, self.dialogue_received = value, time.monotonic()
        except (TypeError, ValueError):
            self.dialogue, self.dialogue_received = {}, None
        # Refresh removes the image reader and invalidates in-flight generations.
        # A running native OCR call finishes normally but cannot publish old facts.
        self.refresh()

    def on_raw(self, serialized):
        if self.signature is None or self.raw_subscription is None:
            return
        try:
            frame = decode_image_cdr(serialized)
            stamp = stamp_nanoseconds(frame.header.stamp)
            if (frame.width, frame.height) != self.size or not 0 <= self.now()-stamp/1e9 <= self.max_input_age:
                return
            if self.corrected_input:
                corrected_observation_image(frame, self.camera, self.size)
            self.join.push_raw(stamp, frame)
            self.counts['corrected_frames' if self.corrected_input else 'raw_frames'] += 1
        except (TypeError, ValueError):
            self.counts['invalid'] += 1

    def on_detection(self, msg):
        if (self.signature is None or self.raw_subscription is None
                or msg.header.frame_id != self.coordinate_frame):
            return
        stamp = stamp_nanoseconds(msg.header.stamp)
        if (msg.width, msg.height) != self.size:
            return
        self.join.push_detection(stamp, dict(capture_stamp=stamp/1e9, capture_stamp_ns=stamp,
            sequence=msg.sequence, coordinate_frame=msg.header.frame_id,
            detector_available=bool(msg.detector_available),
            detections=[dict(class_name=d.class_name, confidence=float(d.confidence),
                bbox=[float(value) for value in d.bbox_xyxy]) for d in msg.detections]))

    def recognize(self, frame):
        started = time.monotonic()
        if self.corrected_input:
            image = corrected_observation_image(frame, self.camera, self.size)
        else:
            image = image_to_bgr(frame.data, frame.width, frame.height, frame.step, frame.encoding)
        if self.lens is not None and not self.corrected_input:
            image = self.lens.correct(image, role='left', output_size=self.size,
                input_metadata={'rotation': self.rotation, 'source_topic': self.source_topic}).image
        return self.reader.predict(image), (time.monotonic()-started)*1000

    def tick(self):
        active = self.refresh()
        if self.pending is not None:
            future, stamp, detection, generation = self.pending
            if future.done():
                self.pending = None
                try:
                    readings, self.last_processing_ms = future.result()
                    result = self.observation.finish(readings, detection, stamp, self.now(), generation,
                        current_generation=self.generation, active=active, coordinate_frame=self.coordinate_frame)
                    result.update(source_topic=self.source_topic, physical_eye='left',
                        input_topic=self.input_topic, corrected_input=self.corrected_input,
                        processing_ms=round(self.last_processing_ms, 3),
                        fixed_camera_preview=self.fixed_camera_preview,
                        unverified_pose=not self.require_settled_pose,
                        observation_signature=self.signature,
                        ocr_workload=dict(self.reader.last_timings),
                        observation_pose_ready=self.require_settled_pose and self.mission.get('gimbal_ready') is True)
                    publish_json(self.output, result)
                    self.last_output_capture = result.get('capture_stamp')
                    self.last_invalid_reason = None if result['valid'] else result['status']
                    self.counts['completed'] += 1
                    self.counts['expired'] += int(result['status'] == 'ocr_result_expired')
                except Exception as exc:
                    self.observation.reset()
                    self.invalidate('ocr_processing_failed')
                    self.counts['invalid'] += 1
                    self.get_logger().warning('OCR failed: ' + str(exc))
                self.refresh_image_subscription()
        if (self.last_output_capture is not None
                and not 0 <= self.now()-self.last_output_capture <= self.observation.max_age):
            self.observation.reset()
            self.invalidate('ocr_evidence_expired')
        if not active:
            self.invalidate('yielding_to_dialogue' if self.yielding_to_dialogue else
                            'ocr_unavailable' if self.reader is None else
                            'corrected_camera_info_unavailable' if self.corrected_input and self.camera is None else
                            'task_or_pose_not_ready')
            return
        if self.pending is not None or time.monotonic() < self.next_allowed:
            return
        selected = self.join.take(self.now(), self.active_after, self.max_input_age)
        if selected is None:
            return
        stamp, frame, detection = selected
        self.next_allowed = time.monotonic()+self.interval
        self.pending = self.worker.submit(self.recognize, frame), stamp, detection, self.generation
        self.counts['submitted'] += 1
        self.refresh_image_subscription()

    def report(self):
        publish_json(self.diagnostics, dict(stamp=self.now(), enabled=self.enabled,
            reader_available=self.reader is not None, reader_error=self.reader_error,
            ocr_workload=dict(self.reader.last_timings) if self.reader is not None else None,
            active=self.signature is not None, generation=self.generation,
            yielding_to_dialogue=self.yielding_to_dialogue,
            source_topic=self.source_topic, coordinate_frame=self.coordinate_frame,
            input_topic=self.input_topic, corrected_input=self.corrected_input,
            camera_geometry_id=self.camera.doc['calibration_id'] if self.camera else None,
            remap_backend='upstream_lossless_bgr' if self.corrected_input else self.lens.remap_backend if self.lens else 'disabled',
            remap_fallback_reason=self.lens.remap_fallback_reason if self.lens else None,
            raw_subscription_active=self.raw_subscription is not None and not self.corrected_input,
            image_subscription_active=self.raw_subscription is not None, counts=self.counts,
            processing_ms=self.last_processing_ms, interval_s=self.interval,
            fixed_camera_preview=self.fixed_camera_preview, require_settled_pose=self.require_settled_pose,
            pending=self.pending is not None, max_result_age_s=self.observation.max_age,
            whole_view_objects_assigned_to_building=False))

    def destroy_node(self):
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    spin(MipiBuildingOCR)
