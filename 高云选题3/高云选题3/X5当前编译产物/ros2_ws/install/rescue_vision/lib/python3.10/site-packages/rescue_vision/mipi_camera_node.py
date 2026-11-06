"""Subscribe to the existing official MIPI driver; never reopen the hardware."""
import copy
import math
import time

import cv2
from cv_bridge import CvBridge
from .node_common import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from rescue_interfaces.msg import StereoPair

from .mipi_image import LatestStereoPairer, image_to_bgr, image_storage_view, stamp_nanoseconds
from .node_common import IMAGE_QOS, param, publish_json, spin


class MipiCamera(Node):
    def __init__(self):
        super().__init__('mipi_camera')
        self.swap = bool(param(self, 'swap_eyes', False))
        self.verified = bool(param(self, 'synchronization_verified', False))
        if param(self, 'input_rectified', False):
            raise ValueError('This raw-image adapter requires GDC/rectification disabled; '
                             'already rectified input needs a separate calibrated depth integration')
        self.width = int(param(self, 'expected_width', 1280))
        self.height = int(param(self, 'expected_height', 1088))
        if self.width <= 0 or self.height <= 0:
            raise ValueError('Expected dimensions must be positive')
        fps = float(param(self, 'publish_fps', 5.0))
        if not math.isfinite(fps) or not 0 < fps <= 60:
            raise ValueError('publish_fps must be in (0, 60]')
        self.pairer = LatestStereoPairer(
            float(param(self, 'max_skew_s', .012)), float(param(self, 'max_age_s', .5)),
            int(param(self, 'queue_size', 8)))
        self.frame_id = param(self, 'frame_id', 'stereo_rig')
        self.bridge = CvBridge()
        self.preserve_encoding = bool(param(self, 'preserve_encoding', False))
        self.last_publish_ms = None
        self.sequence = 0
        self.info = {}
        self.subscriptions_owned = []
        self.last_success_ns = None
        self.last_error = 'waiting_for_both_mipi_topics'
        self.last_skew_s = None
        self.source_metadata = {}
        self.decode_failures = 0
        self.pub = self.create_publisher(StereoPair, '/stereo/pair', IMAGE_QOS)
        self.status = self.create_publisher(String, '/diagnostics/camera', 10)
        for eye in ('left', 'right'):
            topic = str(param(self, eye + '_topic', '/image_' + eye + '_raw'))
            if not topic:
                raise ValueError('Image topic must not be empty')
            self.subscriptions_owned.append(self.create_subscription(
                Image, topic, lambda msg, side=eye: self.receive(side, msg), IMAGE_QOS))
            info_topic = str(param(self, eye + '_info_topic', topic + '/camera_info'))
            if info_topic:
                self.subscriptions_owned.append(self.create_subscription(
                    CameraInfo, info_topic, lambda msg, side=eye: self.receive_info(side, msg), IMAGE_QOS))
        self.create_timer(1.0 / fps, self.tick)
        self.create_timer(1.0, self.diagnostics)
        self.get_logger().info('MIPI topic bridge: raw images only; software pairing does not verify exposure synchronization')

    def receive(self, eye, msg):
        now_ns = self.get_clock().now().nanoseconds
        try:
            stamp_ns = stamp_nanoseconds(msg.header.stamp)
            accepted = self.pairer.push(eye, stamp_ns, msg, now_ns)
            if not accepted:
                self.last_error = 'timestamp_age_or_order_rejected; check realtime timestamps and ROS clock'
            self.source_metadata[eye] = dict(width=msg.width, height=msg.height, step=msg.step,
                encoding=msg.encoding, data_bytes=len(msg.data), frame_id=msg.header.frame_id,
                stamp_ns=stamp_ns, age_s=(now_ns - stamp_ns) / 1e9)
        except (ValueError, TypeError) as exc:
            self.last_error = str(exc)

    def receive_info(self, eye, msg):
        # Informational only. Receiving an intrinsic matrix does not verify its
        # match to the rotation, resolution, eye order or stereo extrinsics.
        self.info[eye] = dict(width=msg.width, height=msg.height,
            frame_id=msg.header.frame_id, distortion_model=msg.distortion_model,
            has_intrinsics=bool(msg.k[0] > 0 and msg.k[4] > 0))

    def tick(self):
        selected = self.pairer.take_latest(self.get_clock().now().nanoseconds)
        if selected is None:
            return
        if self.swap:
            selected = selected[::-1]
        try:
            started = time.perf_counter()
            left, right = selected
            frames = []
            for eye, sample in zip(('left', 'right'), selected):
                source = sample.payload
                if (source.width, source.height) != (self.width, self.height):
                    raise ValueError(f'{eye} dimensions {(source.width, source.height)} differ '
                                     f'from configured {(self.width, self.height)}')
                if self.preserve_encoding:
                    image_storage_view(source.data, source.width, source.height, source.step, source.encoding)
                    frame = copy.copy(source)  # Keep NV12; do not expand both eyes to BGR.
                else:
                    bgr = image_to_bgr(source.data, source.width, source.height, source.step, source.encoding)
                    frame = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
                frame.header = copy.deepcopy(source.header)
                frame.header.frame_id = eye + '_camera_optical'
                frames.append(frame)
            now_ns = self.get_clock().now().nanoseconds
            if any(not 0 <= now_ns - sample.stamp_ns <= self.pairer.max_age_ns for sample in selected):
                raise ValueError('Frames became stale during conversion')
            msg = StereoPair()
            msg.header = copy.deepcopy(frames[0].header)
            msg.header.frame_id = self.frame_id
            msg.sequence = self.sequence
            msg.left, msg.right = frames
            msg.skew_s = (right.stamp_ns - left.stamp_ns) / 1e9
            msg.synchronization_verified = self.verified
            self.pub.publish(msg)
            self.last_publish_ms = (time.perf_counter() - started) * 1000
            self.sequence += 1
            self.last_success_ns = now_ns
            self.last_skew_s = msg.skew_s
            self.last_error = ''
        except (ValueError, TypeError, RuntimeError, cv2.error) as exc:
            self.decode_failures += 1
            self.last_error = str(exc)
            self.get_logger().warning('MIPI frame rejected: ' + str(exc))

    def diagnostics(self):
        now_ns = self.get_clock().now().nanoseconds
        age_s = None if self.last_success_ns is None else (now_ns - self.last_success_ns) / 1e9
        ok = age_s is not None and 0 <= age_s <= 1.0 and not self.last_error
        publish_json(self.status, dict(ok=ok, source='official_mipi_ros_topics',
            reason=self.last_error or ('ok' if ok else 'no_recent_valid_pair'),
            published=self.sequence, last_publication_age_s=age_s, measured_stamp_skew_s=self.last_skew_s,
            synchronization_verified=self.verified, input_rectified=False, swap_eyes=self.swap,
            preserve_encoding=self.preserve_encoding, last_publish_ms=self.last_publish_ms,
            metric_depth_verified=False, camera_info=self.info, source_metadata=self.source_metadata,
            pairing=dict(self.pairer.counts), decode_failures=self.decode_failures))


def main():
    spin(MipiCamera)
