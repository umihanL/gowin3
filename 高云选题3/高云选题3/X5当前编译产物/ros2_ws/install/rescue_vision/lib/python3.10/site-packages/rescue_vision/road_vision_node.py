"""Traffic-light observer reusing the existing MIPI driver and unchanged pixels.

No gimbal, motor or GPIO commands originate here. Capture timestamps are ROS
timestamps; consumers convert age to their own monotonic clock. Source pixels
and the separate full-resolution YOLO preview are never modified.
"""
import json
import math
import time
from collections import deque

import cv2
from .node_common import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .dialogue_workload import yield_optional_observation
from .mipi_image import image_to_bgr, stamp_nanoseconds
from .node_common import IMAGE_QOS, param, publish_json, read_json, spin
from .raw_image_cdr import decode_image_cdr
from .road_visualization import make_visualization
from .traffic import TrafficDetector


def observation_mode(mission, *, mission_age_s, capture_ros_s, now_ros_s,
                     frame_max_age_s=.30, mission_max_age_s=.50):
    """Formal light observations require fresh mission and settled traffic pose."""
    if not isinstance(mission, dict):
        return None
    values = (mission.get('stamp'), mission_age_s, capture_ros_s, now_ros_s)
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        return None
    if (0 <= mission_age_s <= mission_max_age_s
            and 0 <= now_ros_s-mission['stamp'] <= mission_max_age_s
            and 0 <= now_ros_s-capture_ros_s <= frame_max_age_s
            and mission.get('gimbal_ready') is True
            and mission.get('traffic_active') is True
            and mission.get('camera_pose') == 'traffic'):
        return 'traffic'
    return None


class RoadVision(Node):
    def __init__(self):
        super().__init__('road_vision')
        road_path = str(param(self, 'road_config', ''))
        traffic_path = str(param(self, 'traffic_config', ''))
        cfg = read_json(road_path) if road_path else {}
        self.traffic = TrafficDetector(read_json(traffic_path) if traffic_path else {})
        self.source_topic = str(param(self, 'source_topic', '/image_right_raw'))
        self.raw_input_topic = str(param(self, 'raw_input_topic', self.source_topic))
        self.process_on_receive = bool(param(self, 'process_on_receive', False))
        if not self.raw_input_topic or (self.process_on_receive and self.raw_input_topic == self.source_topic):
            raise ValueError('Receive-driven road processing requires a separate rate-bounded raw relay topic')
        self.fixed_camera_observation = bool(param(self, 'fixed_camera_observation', False))
        self.process_fps = float(cfg.get('process_fps', 10.0))
        self.max_age = float(cfg.get('frame_max_age_s', .30))
        self.traffic_width = int(cfg.get('traffic_width', 640))
        self.visualization_fps = float(cfg.get('visualization_fps', 8.0))
        if not (math.isfinite(self.process_fps) and 1 <= self.process_fps <= 15
                and math.isfinite(self.max_age) and .05 <= self.max_age <= .5
                and math.isfinite(self.visualization_fps) and 1 <= self.visualization_fps <= 15
                and 320 <= self.traffic_width <= 1280):
            raise ValueError('Invalid road processing rate, age or resolution')
        # Fixed observation is real evidence only. It shares the 10 Hz budget,
        # but never supplies formal control observations or pose authority.
        cv2.setNumThreads(1)
        self.latest = None
        self.mission = {}
        self.mission_received = None
        self.dialogue, self.dialogue_received = {}, None
        self.yielding_to_dialogue = False
        self.last_stamp = -1
        self.active = None
        self.active_after_ros = math.inf
        self.raw_subscription = None
        self.counts = dict(received=0, processed=0, skipped=0, invalid=0)
        self.processed_times = deque(maxlen=60)
        self.stage_ms = {}
        self.traffic_pub = self.create_publisher(String, '/traffic/observation', 10)
        self.traffic_preview_pub = self.create_publisher(String, '/traffic/preview', 10)
        self.visualization_pub = self.create_publisher(String, '/road/visualization', IMAGE_QOS)
        self.visualization_encoded = 0
        self.visualization_errors = 0
        self.visualization_due = None
        self.diagnostics = self.create_publisher(String, '/diagnostics/road', 10)
        self.create_subscription(String, '/mission/state', self.on_mission, 10)
        self.create_subscription(String, '/scene/dialogue_status', self.on_dialogue, 10)
        self.create_timer(1/self.process_fps, self.process)
        self.create_timer(1., self.report)
        self.get_logger().info('Traffic-only observer subscribes on demand to ' + self.raw_input_topic +
            ('; fixed-camera evidence on preview topics only, no control observations'
             if self.fixed_camera_observation else '; confirmed mission pose required'))

    def refresh_active(self):
        """Own the DDS reader only while a fresh, settled task needs its pixels.

        Both callbacks and the 10 Hz timer run in the default mutually-exclusive
        callback group. Removing a reader also clears queued data; a new traffic
        observation session requires a capture after the new pose.
        """
        now_ros = self.get_clock().now().nanoseconds/1e9
        mission_age = (math.inf if self.mission_received is None else
                       time.monotonic()-self.mission_received)
        mode = ('fixed_preview' if self.fixed_camera_observation else
                observation_mode(self.mission, mission_age_s=mission_age, capture_ros_s=now_ros,
                                 now_ros_s=now_ros, frame_max_age_s=self.max_age))
        if self.mission.get('extension_enabled') is True and self.mission.get('extension_task')=='traffic':
            formal=observation_mode(self.mission,mission_age_s=mission_age,capture_ros_s=now_ros,
                now_ros_s=now_ros,frame_max_age_s=self.max_age)
            mode=formal or 'fixed_preview'
        if (self.mission.get('extension_enabled') is True and
                (self.mission.get('extension_task')!='traffic' or mission_age>.5 or
                 not 0<=now_ros-self.mission.get('stamp',0)<=.5)):
            mode=None
        self.yielding_to_dialogue = bool(self.fixed_camera_observation and
            yield_optional_observation(self.mission, self.mission_received,
                self.dialogue, self.dialogue_received, now_ros, time.monotonic()))
        if self.yielding_to_dialogue:
            mode = None
        if mode != self.active:
            self.active = mode
            self.active_after_ros = now_ros if mode else math.inf
            self.latest = None
            self.processed_times.clear()
            self.visualization_due = None
        if mode is not None and self.raw_subscription is None:
            self.raw_subscription = self.create_subscription(
                Image, self.raw_input_topic, self.on_raw, IMAGE_QOS, raw=True)
        elif mode is None and self.raw_subscription is not None:
            self.destroy_subscription(self.raw_subscription)
            self.raw_subscription = None
            self.latest = None
        return mode

    def on_mission(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Mission state must be an object')
            self.mission = value
            self.mission_received = time.monotonic()
        except (ValueError, TypeError):
            self.mission = {}
            self.mission_received = None
        self.refresh_active()

    def on_raw(self, serialized):
        # Holding one immutable serialized image bounds memory and skips backlog.
        if self.active is not None:
            self.counts['received'] += 1
            if self.latest is not None:
                self.counts['skipped'] += 1
            self.latest = serialized
            if self.process_on_receive:
                # The existing camera reader relays original CDR bytes at this
                # observer's configured cadence. Consume them immediately: two
                # unrelated timers could otherwise add a full period of age
                # and invalidate a correct capture under the 300 ms deadline.
                # The timer remains active for mission/pose expiry even when no
                # images arrive. It sees no queued frame after this call.
                self.process()

    def on_dialogue(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Dialogue status must be an object')
            self.dialogue, self.dialogue_received = value, time.monotonic()
        except (TypeError, ValueError):
            self.dialogue, self.dialogue_received = {}, None
        self.refresh_active()

    @staticmethod
    def resized(image, width):
        if image.shape[1] <= width:
            return image
        return cv2.resize(image, (width, round(image.shape[0]*width/image.shape[1])), interpolation=cv2.INTER_AREA)

    def detect(self, bgr):
        """Use the same working pixels for classification and preview encoding."""
        result = self.traffic.detect(bgr)
        result.pop('capture_time', None)
        result['valid'] = True
        return result

    def visualization_ready(self, now):
        if self.visualization_pub.get_subscription_count() <= 0:
            self.visualization_due = None
            return False
        if self.visualization_due is not None and now < self.visualization_due:
            return False
        period = 1.0/self.visualization_fps
        self.visualization_due = (now+period if self.visualization_due is None
            or now-self.visualization_due >= period else self.visualization_due+period)
        return True

    def frame_mode(self, capture, now_ros, now_monotonic):
        if self.mission.get('extension_enabled') is True and (
                self.mission.get('extension_task')!='traffic' or self.mission_received is None or
                not 0<=now_monotonic-self.mission_received<=.5 or
                not 0<=now_ros-self.mission.get('stamp',0)<=.5):
            return None
        if self.mission.get('extension_enabled') is True and self.mission.get('extension_task')=='traffic':
            formal=observation_mode(self.mission,mission_age_s=now_monotonic-self.mission_received,
                capture_ros_s=capture,now_ros_s=now_ros,frame_max_age_s=self.max_age)
            return formal or ('fixed_preview' if 0<=now_ros-capture<=self.max_age else None)
        if self.fixed_camera_observation:
            # Preview deliberately does not claim a settled gimbal or stopped
            # chassis; only independently current captures may be displayed.
            age = now_ros-capture
            return ('fixed_preview' if math.isfinite(age) and 0 <= age <= self.max_age else None)
        return observation_mode(self.mission,
            mission_age_s=(math.inf if self.mission_received is None else now_monotonic-self.mission_received),
            capture_ros_s=capture, now_ros_s=now_ros, frame_max_age_s=self.max_age)

    def process(self):
        # This executes even when the camera is silent, so stale task state
        # removes the DDS reader instead of continuing to copy large raw frames.
        self.refresh_active()
        serialized, self.latest = self.latest, None
        if serialized is None or (not self.fixed_camera_observation and self.mission_received is None):
            return
        started = time.monotonic()
        try:
            msg = decode_image_cdr(serialized)
            stamp_ns = stamp_nanoseconds(msg.header.stamp)
            if stamp_ns <= self.last_stamp:
                return
            capture = stamp_ns/1e9
            now_ros = self.get_clock().now().nanoseconds/1e9
            mode = self.frame_mode(capture, now_ros, started)
            if mode is None or capture < self.active_after_ros:
                return
            self.last_stamp = stamp_ns
            decoded = time.monotonic()
            # Decode and resize once. Previously crosswalk, traffic and preview
            # repeated this work on the same full-resolution image.
            bgr = self.resized(image_to_bgr(msg.data, msg.width, msg.height, msg.step,
                msg.encoding), self.traffic_width)
            prepared = time.monotonic()
            result = self.detect(bgr)
            analyzed = time.monotonic()
            self.stage_ms = dict(cdr=round((decoded-started)*1000,3),
                decode_resize=round((prepared-decoded)*1000,3),
                traffic=round((analyzed-prepared)*1000,3))
            results = [('traffic', self.traffic_preview_pub if mode == 'fixed_preview'
                        else self.traffic_pub, result)]
            # Processing completion must still belong to the same fresh command.
            now_ros = self.get_clock().now().nanoseconds/1e9
            if self.frame_mode(capture, now_ros, time.monotonic()) != mode:
                return
            for analysis, publisher, result in results:
                result.update(stamp={'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                    capture_ros_s=capture, sequence=str(stamp_ns), frame_id='physical-left:'+str(stamp_ns),
                    source_topic=self.source_topic, raw_input_topic=self.raw_input_topic,
                    source_frame=msg.header.frame_id, physical_eye='left',
                    camera_pose='unverified_fixed' if mode == 'fixed_preview' else mode,
                    source_image_size=[msg.width, msg.height],
                    processing_ms=round((time.monotonic()-started)*1000, 3))
                if mode == 'fixed_preview':
                    result.update(observation_only=True, control_eligible=False,
                                  hardware_pose_verified=False, analysis=analysis)
                publish_json(publisher, result)
            self.counts['processed'] += 1
            self.processed_times.append(time.monotonic())
            # Independent display transport, created only after control/evidence
            # publication. The web service subscribes only during its GET lease.
            # Never paint these raw-coordinate boxes onto the undistorted YOLO feed.
            if self.visualization_ready(time.monotonic()):
                try:
                    encoding_started = time.monotonic()
                    visualization = make_visualization(bgr, capture_ros_s=capture,
                        sequence=str(stamp_ns), source_topic=self.source_topic,
                        results={name: value for name, _, value in results})
                    publish_json(self.visualization_pub, visualization)
                    self.visualization_encoded += 1
                    self.stage_ms['visualization'] = round((time.monotonic()-encoding_started)*1000,3)
                except (ValueError, TypeError, cv2.error):
                    self.visualization_errors += 1
        except (ValueError, TypeError, cv2.error) as exc:
            self.counts['invalid'] += 1
            self.get_logger().warning('Road image rejected: '+str(exc))

    def report(self):
        now = time.monotonic()
        recent = [t for t in self.processed_times if now-t <= 5.0]
        fps = ((len(recent)-1)/(recent[-1]-recent[0])
               if len(recent) >= 2 and recent[-1] > recent[0] and now-recent[-1] < 1.0 else 0.0)
        publish_json(self.diagnostics, {'stamp': self.get_clock().now().nanoseconds/1e9,
            'active_mode': self.active, 'source_topic': self.source_topic,
            'raw_input_topic': self.raw_input_topic, 'process_on_receive': self.process_on_receive,
            'raw_subscription_active': self.raw_subscription is not None,
            'counts': self.counts, 'process_fps_limit': self.process_fps,
            'processing_fps': round(fps,2), 'stage_ms': self.stage_ms,
            'analysis': 'traffic_only', 'crosswalk_enabled': False,
            'single_frame_evidence_only': True, 'requires_confirmed_traffic_pose': True,
            'fixed_camera_observation': self.fixed_camera_observation,
            'yielding_to_dialogue': self.yielding_to_dialogue,
            'preview_only': self.fixed_camera_observation,
            'visualization_subscribers': self.visualization_pub.get_subscription_count(),
            'visualization_encoded': self.visualization_encoded,
            'visualization_errors': self.visualization_errors,
            'visualization_fps_limit': self.visualization_fps,
            'preview_topics': ['/traffic/preview'] if self.fixed_camera_observation else []})


def main():
    spin(RoadVision)
