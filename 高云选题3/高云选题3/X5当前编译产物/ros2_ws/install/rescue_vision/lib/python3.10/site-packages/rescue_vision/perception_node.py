"""YOLO on the exact displayed coordinate space; optional calibrated stereo depth."""
import time
import copy
from collections import deque

import cv2
from cv_bridge import CvBridge
from .node_common import Node
from std_msgs.msg import String
from rescue_interfaces.msg import StereoPair, Detection, DetectionArray
from .node_common import IMAGE_QOS, param, read_json, age, publish_json, spin
from .mipi_image import image_to_bgr
from .stereo import StereoDepth, CalibrationError, FrameError


def decode_image(msg):
    return image_to_bgr(msg.data, msg.width, msg.height, msg.step, msg.encoding)


class Perception(Node):
    def __init__(self):
        super().__init__('perception')
        cv2.setNumThreads(int(param(self, 'opencv_threads', 2)))
        self.bridge = CvBridge()
        self.latest = None
        self.last_sequence = None
        self.completed = deque(maxlen=30)
        self.performance = {}
        self.skipped_stale = 0
        self.max_age = float(param(self, 'max_input_age_s', .5))
        path = param(self, 'stereo_calibration', '')
        self.stereo = StereoDepth(read_json(path) if path else {'calibrated': False})
        self.lens = None
        self.lens_metadata = None
        self.output_size = (int(param(self, 'image_width', 640)),
                            int(param(self, 'image_height', 544)))
        self.input_rotation = float(param(self, 'input_rotation', 90.0))
        self.lens_path = param(self, 'lens_intrinsics', '')
        if not bool(param(self, 'lens_enabled', True)):
            self.lens_path = ''
        if self.lens_path:
            if self.stereo.calibrated:
                raise ValueError('Use stereo rectification OR single-eye undistortion, not both')
            from .lens_correction import LensCorrector
            left_source = param(self, 'left_source_topic', '/image_left_raw')
            right_source = param(self, 'right_source_topic', '/image_right_raw')
            if bool(param(self, 'lens_swap_eyes', False)):
                left_source, right_source = right_source, left_source
            self.lens = LensCorrector.from_json(self.lens_path,
                role_to_source={'left': left_source, 'right': right_source},
                alpha=float(param(self, 'lens_alpha', 0.0)))
        self.fire_only = bool(param(self, 'fire_only', False))
        self.classes = ['fire'] if self.fire_only else ['fire', 'firefighter', 'person']
        self.detector = None
        self.model_status = 'detector_disabled'
        if param(self, 'detector_enabled', False):
            from .legacy_detector import BoardDetector
            self.detector = BoardDetector(param(self, 'model', ''), param(self, 'backend', 'pyeasy'),
                                          class_ids=(0,) if self.fire_only else None)
            self.model_status = 'candidate_model_board_accuracy_unverified'
        self.create_subscription(StereoPair, '/stereo/pair', self.receive, IMAGE_QOS)
        self.output = self.create_publisher(DetectionArray, '/perception/detections', IMAGE_QOS)
        self.frames = self.create_publisher(StereoPair, '/vision/frame', IMAGE_QOS)
        self.diagnostics = self.create_publisher(String, '/diagnostics/perception', 10)
        fps = float(param(self, 'process_fps', 5.0))
        self.process_on_receive = bool(param(self, 'process_on_receive', False))
        if not self.process_on_receive:
            self.create_timer(1 / fps, self.tick)
        self.create_timer(1.0, self.report)

    def receive(self, msg):
        self.latest = msg
        if self.process_on_receive:
            # The MIPI adapter limits publication rate; depth-one DDS skips old
            # frames. Avoid a second timer adding latency between these stages.
            self.tick()

    def report(self):
        if self.performance:
            publish_json(self.diagnostics, self.performance)

    def tick(self):
        pair = self.latest
        if pair is None or pair.sequence == self.last_sequence:
            return
        self.last_sequence = pair.sequence
        if not 0 <= age(self, pair.header.stamp) <= self.max_age:
            self.skipped_stale += 1
            return
        started = time.perf_counter()
        out = DetectionArray()
        out.header = copy.deepcopy(pair.header)
        out.sequence = pair.sequence
        left = decode_image(pair.left)
        decoded_at = time.perf_counter()
        depth, reason, lens_state = None, 'uncalibrated', 'raw'
        out.rectified = False
        right = None
        try:
            if self.stereo.calibrated:
                right = decode_image(pair.right)
                left, right = self.stereo.rectify(left, right)
                out.rectified = True
                lens_state = 'stereo_rectified'
                if pair.synchronization_verified:
                    depth = self.stereo.compute(left, right, pair.skew_s)
                    reason = 'ok'
                else:
                    reason = 'exposure_synchronization_unverified'
            elif self.lens:
                corrected = self.lens.correct(left, role='left', output_size=self.output_size,
                    already_rectified=False, input_metadata={'rotation': self.input_rotation})
                left = corrected.image
                self.lens_metadata = corrected.metadata
                lens_state = 'undistorted_candidate'
        except (CalibrationError, FrameError, ValueError, cv2.error) as exc:
            # Do not silently fall back to raw pixels under a corrected frame_id.
            self.get_logger().error('Geometry rejected: ' + str(exc))
            return
        geometry_at = time.perf_counter()
        out.header.frame_id = ('rectified_left_optical' if out.rectified else
                               'undistorted_left_optical' if self.lens else 'left_camera_optical')
        out.width, out.height = left.shape[1], left.shape[0]
        out.detector_available = self.detector is not None
        out.status = (self.model_status + '; depth=' + reason + '; classes=' + ','.join(self.classes)
                      + '; lens=' + lens_state)
        items = self.detector.predict(left) if self.detector else []
        for item in items:
            d = Detection()
            d.class_id, d.class_name = item['class_id'], item['class_name']
            d.confidence = float(item['confidence'])
            d.bbox_xyxy = list(map(float, item['bbox_xyxy']))
            sample = (self.stereo.sample_bbox(depth, d.bbox_xyxy) if depth is not None else
                      {'valid': False, 'reason': reason})
            d.depth_valid, d.depth_reason = sample['valid'], sample['reason']
            if d.depth_valid:
                d.position_m.x, d.position_m.y, d.position_m.z = map(float, sample['xyz_m'])
                d.depth_m = float(sample['depth_m'])
            out.detections.append(d)
        inferred_at = time.perf_counter()
        if self.frames.get_subscription_count():
            frame = StereoPair()
            frame.header = copy.deepcopy(out.header)
            frame.sequence = pair.sequence
            frame.left = self.bridge.cv2_to_imgmsg(left, 'bgr8')
            frame.left.header = copy.deepcopy(out.header)
            if right is not None:
                frame.right = self.bridge.cv2_to_imgmsg(right, 'bgr8')
                frame.right.header = copy.deepcopy(pair.right.header)
                frame.right.header.frame_id = 'rectified_right_optical'
            else:
                # Raw right eye retains its encoding, size and frame_id. This is
                # NOT an equal-sized, rectified stereo pair for depth matching.
                frame.right = pair.right
            frame.synchronization_verified = pair.synchronization_verified
            frame.skew_s = pair.skew_s
            self.frames.publish(frame)
        self.output.publish(out)
        ended = time.perf_counter()
        self.completed.append(ended)
        fps = ((len(self.completed) - 1) / (self.completed[-1] - self.completed[0])
               if len(self.completed) > 1 else 0.0)
        timings = self.detector.last_timings if self.detector else {}
        self.performance = dict(
            sequence=pair.sequence, stamp_sec=out.header.stamp.sec,
            enabled_classes=self.classes, detector_network_classes=3,
            lens=lens_state, lens_metadata=self.lens_metadata,
            output_resolution=[out.width, out.height], processing_fps=round(fps, 2),
            skipped_stale=self.skipped_stale,
            image_age_s=age(self, pair.header.stamp),
            stage_ms={
                'decode': round((decoded_at - started) * 1000, 2),
                'lens': round((geometry_at - decoded_at) * 1000, 2),
                'preprocess': round(timings.get('preprocess_s', 0) * 1000, 2),
                'inference': round(timings.get('inference_s', 0) * 1000, 2),
                'postprocess': round(timings.get('postprocess_s', 0) * 1000, 2),
                'publish': round((ended - inferred_at) * 1000, 2),
                'total': round((ended - started) * 1000, 2)})


def main():
    spin(Perception)
