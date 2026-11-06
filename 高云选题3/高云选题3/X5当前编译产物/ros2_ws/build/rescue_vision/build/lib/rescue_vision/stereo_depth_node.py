"""Optional full-resolution depth sidecar for the existing fast YOLO pipeline.

Never starts a camera or reruns YOLO. Control output keeps its verified-depth
contract. Factory-parameter observation is an explicit, separate JSON output;
it cannot make a DetectionArray depth valid.
"""
import copy
import json
import math
import threading
import time
from collections import OrderedDict

import cv2
import numpy as np
from .node_common import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rescue_interfaces.msg import DetectionArray

from .node_common import IMAGE_QOS, param, read_json, publish_json, spin, age
from .raw_image_cdr import decode_image_cdr
from .mipi_image import stamp_nanoseconds, image_storage_view
from .depth_frame_join import DepthFrameJoin
from .stereo import StereoDepth
from .fire_aim_geometry import corrected_camera, CORRECTED_FRAME
from .stereo_detection_geometry import CorrectedDetectionGeometry
from .lens_correction import LensCorrector


VERIFICATIONS = ('physical_eye_mapping_verified', 'synchronization_verified', 'metric_depth_verified')


def readiness(config, stereo):
    if config.get('enabled') is not True:
        return 'disabled'
    if not stereo.calibrated:
        return 'uncalibrated'
    for key in VERIFICATIONS:
        if config.get(key) is not True:
            return key + '_required'
    if not isinstance(config.get('verification_id'), str) or not config['verification_id'].strip():
        return 'verification_id_required'
    return 'ready'


def gray_image(view):
    """Use full-resolution NV12 Y directly; this sidecar never changes YOLO pixels."""
    storage = image_storage_view(view.data, view.width, view.height, view.step, view.encoding)
    kind = view.encoding.lower()
    if kind in ('nv12', 'mono8'):
        return np.ascontiguousarray(storage[:view.height, :view.width])
    image = np.ascontiguousarray(storage).reshape(view.height, view.width, 3)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY if kind == 'bgr8' else cv2.COLOR_RGB2GRAY)


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth')
        self.config = read_json(param(self, 'runtime_config', ''))
        self.calibration = read_json(param(self, 'stereo_calibration', ''))
        self.stereo = StereoDepth(self.calibration, self.config.get('matcher', {}))
        self.gate = readiness(self.config, self.stereo)
        self.observation_only = self.config.get('observation_only') is True
        self.scene_probe_enabled = self.config.get('scene_probe_enabled', True) is True
        self.regional_compute = self.config.get('regional_compute', self.observation_only) is True
        self.scene_probe_size = self.config.get('scene_probe_size_px', 100)
        if (isinstance(self.scene_probe_size, bool) or not isinstance(self.scene_probe_size, int)
                or not 32 <= self.scene_probe_size <= 300):
            raise ValueError('Scene probe must span 32..300 native pixels')
        self.compute_allowed = (self.config.get('enabled') is True and self.stereo.calibrated
                                and (self.gate == 'ready' or self.observation_only))
        self.control_allowed = self.gate == 'ready' and not self.observation_only
        self.missing_verifications = [key for key in VERIFICATIONS if self.config.get(key) is not True]
        if not isinstance(self.config.get('verification_id'), str) or not self.config['verification_id'].strip():
            self.missing_verifications.append('verification_id')
        self.max_age = float(self.config.get('max_frame_age_s', .5))
        observation_age = self.config.get('observation_max_age_s', self.max_age)
        if (isinstance(observation_age, bool) or not isinstance(observation_age, (int, float))
                or not math.isfinite(observation_age) or not self.max_age <= observation_age <= 3.):
            raise ValueError('Observation display age must be between control frame age and 3 seconds')
        self.observation_max_age = float(observation_age) if self.observation_only else self.max_age
        fps = float(self.config.get('process_fps', 5.))
        threads = self.config.get('opencv_threads', 2)
        if (not math.isfinite(fps) or not 2.5 <= fps <= 10
                or isinstance(threads, bool) or not isinstance(threads, int) or not 1 <= threads <= 4):
            raise ValueError('Depth fps must be 2.5..10; OpenCV threads 1..4')
        self.period = 1. / fps
        cv2.setNumThreads(threads)
        self.join = DepthFrameJoin(self.max_age, float(self.config.get('max_pair_skew_s', .012)),
                                   self.config.get('queue_size', 16))
        self.lock = threading.Lock()
        self.publish_lock = threading.Lock()
        self.last_output_stamp = -1
        self.last_observation_stamp = -1
        self.pending = OrderedDict()
        self.camera = None
        self.geometry = None
        self.geometry_id = None
        self.mission, self.mission_received = {}, None
        self.extension, self.extension_received = {}, None
        self.last_detection_stamp = -1
        self.last_compute = 0.
        self.counts = dict(input=0, computed=0, published=0, valid_depth=0, superseded=0,
                           unpaired_dropped=0, budget_skipped=0, observations_published=0,
                           observations_delayed=0, observations_expired=0, observations_superseded=0)
        self.last_processing = {}
        self.compute_pixel_seconds = None
        self.last_pair_wait_ms = None
        self.raw_stats = {eye: dict(received=0, accepted=0, last_stamp_ns=None,
                                   last_received_monotonic=None, reason='no_frames_received')
                          for eye in ('left', 'right')}
        self.last_pair_skew_s = None
        self.last_status, self.last_detail = self.gate, {}
        self.stop_event = threading.Event()
        self.worker = None
        source = str(param(self, 'detections_topic', '/perception/detections'))
        dest = str(param(self, 'output_topic', '/perception/depth_detections'))
        if source == dest:
            raise ValueError('Depth input and output topics must differ')
        self.publisher = self.create_publisher(DetectionArray, dest, IMAGE_QOS)
        self.status_pub = self.create_publisher(String, '/stereo/depth_status', 10)
        self.observation_pub = (self.create_publisher(String, '/stereo/depth_observation', 10)
                                if self.observation_only else None)
        self.create_subscription(DetectionArray, source, self.receive, IMAGE_QOS)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraInfo, '/vision/camera_info', self.info, qos)
        self.create_subscription(String, '/mission/state', self.mode, 10)
        self.create_subscription(String, '/extension/state', self.extension_state, 10)
        self.create_timer(1., self.status)
        if self.compute_allowed:
            self.validate_lens(read_json(param(self, 'lens_intrinsics', '')),
                               float(param(self, 'lens_alpha', 0.)))
            raw_depth = self.config.get('raw_qos_depth', min(8, self.join.capacity))
            if isinstance(raw_depth, bool) or not isinstance(raw_depth, int) or not 1 <= raw_depth <= 32:
                raise ValueError('Raw stereo subscription depth must be 1..32')
            raw_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=raw_depth,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
            for eye in ('left', 'right'):
                topic = self.config['physical_' + eye + '_topic']
                self.create_subscription(Image, topic, lambda data, eye=eye: self.raw(eye, data),
                                         raw_qos, raw=True)
            self.worker = threading.Thread(target=self.work, name='metric-stereo', daemon=True)
            self.worker.start()
        self.get_logger().info('Stereo sidecar: ' + self.gate +
                               ('; observation only, control depth disabled' if self.observation_only else '') +
                               '; original YOLO/preview unchanged')

    def validate_lens(self, snapshot, alpha):
        roles = {eye: self.config['physical_' + eye + '_topic'] for eye in ('left', 'right')}
        if not all(isinstance(t, str) and t.startswith('/') for t in roles.values()):
            raise ValueError('Explicit absolute physical-eye topics are required')
        if self.calibration.get('source_topics') is not None and self.calibration['source_topics'] != roles:
            raise ValueError('Stereo calibration source order differs from runtime input topics')
        lens = LensCorrector(snapshot, role_to_source=roles, alpha=alpha)
        gdc = snapshot['driver']['parameters'].get('gdc_enable')
        raw_input = gdc in (False, 0) or (isinstance(gdc, str) and gdc.lower() == 'false')
        rotation = float(self.config['input_rotation_deg'])
        if not math.isfinite(rotation) or not math.isclose(lens.rotation, rotation, abs_tol=1e-8):
            raise ValueError('Stereo runtime and current preview rotations differ')
        if (self.calibration.get('input_rotation_deg') is not None and
                not math.isclose(float(self.calibration['input_rotation_deg']), rotation, abs_tol=1e-8)):
            raise ValueError('Stereo calibration and runtime input rotations differ')
        # Current mono correction must describe the same measured physical rays.
        # Padding a 5-coefficient model with zero rational coefficients is valid.
        for eye, calibrated_K, calibrated_D in (('left', self.stereo.K1, self.stereo.D1),
                                                ('right', self.stereo.K2, self.stereo.D2)):
            source = snapshot['sources'][roles[eye]]
            K, D = np.asarray(source['K']).reshape(3, 3), np.asarray(source['D']).ravel()
            d1 = calibrated_D.ravel()
            n = max(len(D), len(d1))
            if (not np.allclose(K, calibrated_K, rtol=0, atol=1e-6)
                    or not np.allclose(np.pad(D, (0, n-len(D))), np.pad(d1, (0, n-len(d1))), rtol=0, atol=1e-8)
                    or (source['width'], source['height']) != self.stereo.image_size
                    or not raw_input):
                raise ValueError('Measured stereo intrinsics must match the ' + eye + ' raw lens mapping')
        source = snapshot['sources'][roles['left']]
        K, D = np.asarray(source['K']).reshape(3, 3), np.asarray(source['D']).ravel()
        self.expected_K = cv2.getOptimalNewCameraMatrix(K, D, self.stereo.image_size, alpha,
                                                        self.stereo.image_size)[0]

    def info(self, msg):
        try:
            camera = corrected_camera(msg, CORRECTED_FRAME, (msg.width, msg.height))
            if self.compute_allowed and (camera.size != self.stereo.image_size
                    or not np.allclose(camera.K, self.expected_K, rtol=0, atol=1e-6)):
                raise ValueError('Live corrected CameraInfo differs from verified lens geometry')
            with self.lock:
                self.camera = camera
        except (ValueError, TypeError, AttributeError) as exc:
            with self.lock:
                self.camera = None
            self.last_status = 'camera_geometry_rejected: ' + str(exc)

    def mode(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Mission state is not an object')
        except (ValueError, TypeError):
            value = {}
        with self.lock:
            self.mission, self.mission_received = value, time.monotonic()

    def active(self):
        if self.observation_only:
            return (self.mission_received is not None and 0 <= time.monotonic()-self.mission_received <= .5
                    and self.mission.get('mode') in ('inspect', 'fire'))
        if self.config.get('require_fire_mode', True) is False:
            return True
        extension_at = getattr(self, 'extension_received', None)
        if (extension_at is not None and 0 <= time.monotonic()-extension_at <= .6
                and (self.extension.get('region_search') or {}).get('active') is True
                and self.extension.get('task') == 'semantic'):
            return True
        return (self.mission_received is not None and 0 <= time.monotonic() - self.mission_received <= .5
                and self.mission.get('mode') == 'fire' and self.mission.get('traffic_active', False) is False)

    def extension_state(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('invalid_extension_state')
        except (ValueError, TypeError):
            value = {}
        with self.lock:
            self.extension, self.extension_received = value, time.monotonic()

    def raw(self, eye, data):
        with self.lock:
            self.raw_stats[eye]['received'] += 1
            self.raw_stats[eye]['last_received_monotonic'] = time.monotonic()
        try:
            frame = decode_image_cdr(data)
            stamp = stamp_nanoseconds(frame.header.stamp)
            with self.lock:
                self.raw_stats[eye]['last_stamp_ns'] = stamp
                if not self.active():
                    self.raw_stats[eye]['reason'] = 'inactive'
                    return
            if (frame.width, frame.height) != self.stereo.image_size:
                raise ValueError('Raw image dimensions do not match measured stereo calibration')
            with self.lock:
                accepted = self.join.push(eye, stamp, frame, self.get_clock().now().nanoseconds)
                self.raw_stats[eye]['accepted'] += int(accepted)
                self.raw_stats[eye]['reason'] = 'accepted' if accepted else 'stale_or_nonmonotonic_frame'
        except (ValueError, TypeError) as exc:
            self.last_status = 'raw_frame_rejected: ' + str(exc)
            with self.lock:
                self.raw_stats[eye]['reason'] = self.last_status

    def receive(self, msg):
        self.counts['input'] += 1
        stamp = stamp_nanoseconds(msg.header.stamp)
        if stamp <= self.last_detection_stamp or not 0 <= age(self, msg.header.stamp) <= self.max_age:
            return
        self.last_detection_stamp = stamp
        if not self.compute_allowed:
            self.publish(msg, self.gate)
            return
        with self.lock:
            if not self.active():
                reason = 'inactive_outside_fire_mode'
            elif not msg.detector_available and not self.observation_only:
                reason = 'detector_unavailable'
            elif (not any(d.class_id == 0 and d.confidence >= .4 for d in msg.detections)
                  and not (self.observation_only and self.scene_probe_enabled)):
                reason = 'no_fire_target'
            else:
                reason = None
                self.pending[stamp] = (msg, time.monotonic())
                while len(self.pending) > self.join.capacity:
                    self.pending.popitem(last=False)
                    self.counts['superseded'] += 1
        if reason:
            self.publish(msg, reason)

    def publish(self, msg, reason, estimates=None, started=None, scene_probe=None, pair_skew_s=None):
        with self.publish_lock:
            stamp = stamp_nanoseconds(msg.header.stamp)
            if stamp <= self.last_output_stamp:
                return
            self._publish(msg, reason, estimates, started, scene_probe, pair_skew_s)

    def _publish(self, msg, reason, estimates=None, started=None, scene_probe=None, pair_skew_s=None):
        observing = getattr(self, 'observation_only', False)
        if not 0 <= age(self, msg.header.stamp) <= self.max_age:
            self.last_status = 'expired_before_depth_publication'
            if observing:
                # Only the diagnostic display has a separate age budget. No
                # DetectionArray is emitted for this expired control frame.
                self.publish_observation(msg, reason, estimates, started, scene_probe, pair_skew_s)
            return
        out = copy.deepcopy(msg)
        out.stereo_calibration_id = ''
        verified_compute = (self.gate == 'ready' and not observing and
                            reason == 'computed' and self.stereo.calibrated)
        for i, d in enumerate(out.detections):
            d.depth_valid = False
            d.depth_m = 0.
            d.position_m.x = d.position_m.y = d.position_m.z = 0.
            d.depth_reason = 'observation_only_unverified' if observing else reason
            if verified_compute and estimates is not None and i in estimates:
                estimate = estimates[i]
                d.depth_reason = estimate['reason']
                if estimate['valid']:
                    d.depth_valid = True
                    d.depth_m = float(estimate['depth_m'])
                    d.position_m.x, d.position_m.y, d.position_m.z = map(float, estimate['xyz_m'])
                    d.depth_reason = 'verified_stereo_wallpaper_surface'
        if verified_compute and any(d.depth_valid for d in out.detections):
            out.stereo_calibration_id = self.stereo.calibration_id
        # Do not claim the corrected-image boxes are stereo-rectified boxes.
        out.status += '; sidecar_depth=' + reason + '; stereo_calibration=' + str(self.stereo.calibration_id)
        self.publisher.publish(out)
        self.last_output_stamp = stamp_nanoseconds(out.header.stamp)
        self.counts['published'] += 1
        self.counts['valid_depth'] += sum(d.depth_valid for d in out.detections)
        self.last_status = reason
        self.last_detail = dict(sequence=out.sequence, capture_stamp_ns=stamp_nanoseconds(out.header.stamp),
            coordinate_frame=out.header.frame_id, valid_depth_count=sum(d.depth_valid for d in out.detections),
            age_s=age(self, out.header.stamp),
            compute_ms=None if started is None else (time.perf_counter()-started)*1000)
        if observing:
            self.publish_observation(msg, reason, estimates, started, scene_probe, pair_skew_s)

    def raw_snapshot(self):
        now_ns, now = self.get_clock().now().nanoseconds, time.monotonic()
        with self.lock:
            values = copy.deepcopy(self.raw_stats)
        for eye, value in values.items():
            stamp, received = value.pop('last_stamp_ns'), value.pop('last_received_monotonic')
            value.update(topic=self.config.get('physical_' + eye + '_topic'),
                         last_stamp_ns=stamp,
                         last_age_s=None if stamp is None else (now_ns-stamp)/1e9,
                         last_receive_age_s=None if received is None else now-received)
        return values

    @staticmethod
    def observation_estimate(estimate):
        return dict(estimate_valid=estimate.get('valid') is True,
                    **{key: estimate.get(key) for key in ('depth_m', 'range_m', 'xyz_m', 'reason',
                                                        'valid_ratio', 'samples', 'depth_mad_m')})

    def publish_observation(self, msg, reason, estimates=None, started=None, scene_probe=None, pair_skew_s=None):
        # Called under publish_lock, independently monotonic from the control
        # topic: an older delayed computation must never replace a newer view.
        stamp_ns = stamp_nanoseconds(msg.header.stamp)
        if stamp_ns <= self.last_observation_stamp:
            self.counts['observations_superseded'] += 1
            return
        source_age = age(self, msg.header.stamp)
        expired = not 0 <= source_age <= self.observation_max_age
        delayed = not expired and source_age > self.max_age
        if expired:
            estimates, scene_probe, reason = None, None, 'observation_expired'
        elif delayed and reason == 'computed':
            reason = 'computed_delayed_observation'
        targets = []
        for index, estimate in (estimates or {}).items():
            detection = msg.detections[index]
            targets.append(dict(detection_index=index, class_id=detection.class_id,
                                confidence=float(detection.confidence),
                                bbox_xyxy=[float(value) for value in detection.bbox_xyxy],
                                **self.observation_estimate(estimate)))
        probe = None
        if scene_probe is not None:
            probe = dict(kind='calibration_probe', label='中心场景测距（非火源）',
                         bbox_xyxy=scene_probe['bbox_xyxy'], **self.observation_estimate(scene_probe))
        publish_json(self.observation_pub, dict(
            observation_only=True, control_eligible=False,
            delayed_observation=delayed, observation_expired=expired, source_age_s=source_age,
            observation_max_age_s=self.observation_max_age, control_max_age_s=self.max_age,
            metric_depth_verified=self.config.get('metric_depth_verified') is True,
            missing_verifications=list(self.missing_verifications), status=reason,
            calibration_id=self.stereo.calibration_id,
            calibration_source=self.calibration.get('calibration_source'),
            disparity_sign=self.stereo.disparity_sign, source_sequence=msg.sequence,
            capture_stamp_ns=stamp_ns, capture_stamp=stamp_ns/1e9,
            coordinate_frame=msg.header.frame_id, image_size_wh=[msg.width, msg.height],
            pair_skew_s=pair_skew_s, raw_stats=self.raw_snapshot(),
            compute_ms=None if started is None else (time.perf_counter()-started)*1000,
            targets=targets, scene_probe=probe,
            processing=(dict(self.last_processing) if self.last_processing.get('source_sequence') == msg.sequence else None),
            range_convention='reference_camera_euclidean_m', depth_convention='reference_camera_z_m'))
        self.last_observation_stamp = stamp_ns
        self.counts['observations_published'] += 1
        self.counts['observations_delayed'] += int(delayed)
        self.counts['observations_expired'] += int(expired)
        self.last_status = reason
        self.last_detail = dict(sequence=msg.sequence, capture_stamp_ns=stamp_ns,
                               coordinate_frame=msg.header.frame_id, valid_depth_count=0,
                               age_s=source_age, delayed_observation=delayed,
                               compute_ms=None if started is None else (time.perf_counter()-started)*1000)

    def next_pair(self):
        """Select the newest exact exposure already buffered in both eyes.

        A newer detection awaiting its raw callbacks cannot evict an older
        paired candidate. Missing frames wait only while their exact timestamp
        can still arrive, within the existing capture-age bound.
        """
        with self.lock:
            if not self.pending:
                return None
            camera = self.camera
            if not self.active() or camera is None:
                _, (msg, _) = self.pending.popitem(last=True)
                self.counts['superseded'] += len(self.pending)
                self.pending.clear()
                return msg, camera, None, ('inactive_outside_fire_mode' if not self.active()
                                           else 'camera_geometry_unavailable')
            now_ns = self.get_clock().now().nanoseconds
            rejected = None
            for stamp, (msg, received) in reversed(tuple(self.pending.items())):
                if stamp <= self.last_output_stamp:
                    del self.pending[stamp]
                    self.counts['superseded'] += 1
                    continue
                pair, reason = self.join.match(stamp, now_ns)
                if pair is not None:
                    older = [value for value in self.pending if value <= stamp]
                    for value in older:
                        del self.pending[value]
                    self.counts['superseded'] += len(older)-1
                    self.last_pair_wait_ms = (time.monotonic()-received)*1000
                    return msg, camera, pair, None
                waiting = ((reason == 'exact_left_frame_missing' and self.join.last_stamp['left'] <= stamp)
                           or (reason == 'matching_right_frame_missing'
                               and self.join.last_stamp['right'] <= stamp+self.join.skew_ns))
                if waiting:
                    self.last_status = 'waiting_for_exact_stereo_pair'
                    continue
                del self.pending[stamp]
                self.counts['unpaired_dropped'] += 1
                if rejected is None:
                    rejected = (msg, camera, None, reason)
            return rejected

    @staticmethod
    def region_area(region):
        return (region[2]-region[0])*(region[3]-region[1])

    def estimate_regions(self, msg, camera, pair):
        """Compute native-resolution regions without changing quality gates."""
        requests = OrderedDict()
        if self.observation_only and self.scene_probe_enabled:
            width, height = camera.size
            half_w, half_h = min(self.scene_probe_size/2, width/2), min(self.scene_probe_size/2, height/2)
            requests['scene'] = [width/2-half_w, height/2-half_h, width/2+half_w, height/2+half_h]
        requests.update({i: list(d.bbox_xyxy) for i, d in enumerate(msg.detections)
                         if msg.detector_available and d.class_id == 0 and d.confidence >= .4})
        answers = {key: dict(valid=False, reason='not_computed') for key in requests}
        groups = []
        for key, box in requests.items():
            try:
                bounds = self.geometry.sampling_bounds(box)
                region = (self.stereo.processing_region(bounds) if self.regional_compute
                          else (0, 0, *self.stereo.image_size))
            except (ValueError, TypeError) as exc:
                answers[key] = dict(valid=False, reason=str(exc))
                continue
            # Merge nearby work only when it adds little to the earlier area.
            # The first group is the independent scene probe; a distant false
            # fire detection cannot expand it back to an entire image.
            for group in groups:
                old = group['region']
                union = (min(old[0], region[0]), min(old[1], region[1]),
                         max(old[2], region[2]), max(old[3], region[3]))
                if self.region_area(union) <= 1.25*group['base_area']:
                    group['region'] = union
                    group['keys'].append(key)
                    break
            else:
                groups.append(dict(region=region, keys=[key], base_area=self.region_area(region)))
        profile = dict(source_sequence=msg.sequence, source_age_at_start_s=age(self, msg.header.stamp),
                       pair_wait_ms=self.last_pair_wait_ms, regions=[], processed_pixels=0,
                       full_frame_pixels=self.stereo.image_size[0]*self.stereo.image_size[1],
                       budget_skipped=0, native_resolution=True)
        start = time.perf_counter()
        left, right, skew = pair
        gray_left, gray_right = gray_image(left), gray_image(right)
        profile['gray_ms'] = (time.perf_counter()-start)*1000
        for group in groups:
            region, keys = group['region'], group['keys']
            pixels = self.region_area(region)
            remaining = self.max_age-age(self, msg.header.stamp)
            predicted = (None if self.compute_pixel_seconds is None
                         else pixels*self.compute_pixel_seconds*1.15+.01)
            if remaining <= .015 or (profile['regions'] and predicted is not None and remaining <= predicted):
                for key in keys:
                    answers[key] = dict(valid=False, reason='processing_budget_exhausted')
                profile['budget_skipped'] += len(keys)
                self.counts['budget_skipped'] += len(keys)
                continue
            detail = dict(rectified_xyxy=list(region), pixels=pixels)
            region_start = time.perf_counter()
            try:
                rectified = self.stereo.rectify(gray_left, gray_right, region=region)
                rectified_at = time.perf_counter()
                result = self.stereo.compute(*rectified, skew, region=region)
                matched_at = time.perf_counter()
                for key in keys:
                    answers[key] = self.geometry.sample_bbox(result, requests[key])
                sampled_at = time.perf_counter()
                detail.update(rectify_ms=(rectified_at-region_start)*1000,
                              match_ms=(matched_at-rectified_at)*1000,
                              sample_ms=(sampled_at-matched_at)*1000)
                cost = (sampled_at-region_start)/pixels
                self.compute_pixel_seconds = (cost if self.compute_pixel_seconds is None else
                                              .7*self.compute_pixel_seconds+.3*cost)
                profile['processed_pixels'] += pixels
            except (ValueError, TypeError, cv2.error) as exc:
                detail['reason'] = 'depth_rejected: '+str(exc)
                for key in keys:
                    answers[key] = dict(valid=False, reason=detail['reason'])
            profile['regions'].append(detail)
        profile['total_ms'] = (time.perf_counter()-start)*1000
        self.last_processing = profile
        probe = dict(answers['scene'], bbox_xyxy=requests['scene']) if 'scene' in answers else None
        return {key: value for key, value in answers.items() if key != 'scene'}, probe

    def work(self):
        while not self.stop_event.wait(.01):
            if time.monotonic()-self.last_compute < self.period:
                continue
            candidate = self.next_pair()
            if candidate is None:
                continue
            msg, camera, pair, reason = candidate
            if pair is None:
                self.publish(msg, reason)
                continue
            started = time.perf_counter()
            self.last_compute = time.monotonic()
            skew = pair[2]
            self.last_pair_skew_s = skew
            try:
                if msg.header.frame_id != CORRECTED_FRAME or msg.rectified or (msg.width, msg.height) != camera.size:
                    raise ValueError('Detection coordinates are not the current monocular-corrected physical left frame')
                identity = camera.doc['calibration_id']
                if self.geometry_id != identity:
                    self.geometry = CorrectedDetectionGeometry(self.stereo, camera.K, camera.size)
                    self.geometry_id = identity
                estimates, scene_probe = self.estimate_regions(msg, camera, pair)
                self.counts['computed'] += 1
                with self.lock:
                    still_valid = self.active() and self.camera is not None and self.camera.doc['calibration_id'] == identity
                if still_valid:
                    self.publish(msg, 'computed', estimates, started, scene_probe, skew)
                else:
                    self.publish(msg, 'mode_or_geometry_changed_during_compute', started=started, pair_skew_s=skew)
            except (ValueError, TypeError, cv2.error) as exc:
                self.publish(msg, 'depth_rejected: '+str(exc), started=started, pair_skew_s=skew)

    def status(self):
        publish_json(self.status_pub, dict(gate=self.gate, status=self.last_status,
            calibration_id=self.stereo.calibration_id, verification_id=self.config.get('verification_id'),
            observation_only=self.observation_only, compute_allowed=self.compute_allowed,
            observation_max_age_s=self.observation_max_age, control_max_age_s=self.max_age,
            control_eligible=self.control_allowed, missing_verifications=list(self.missing_verifications),
            raw_stats=self.raw_snapshot(), pair_skew_s=self.last_pair_skew_s,
            pending_frames=len(self.pending), processing=dict(self.last_processing),
            disparity_sign=self.stereo.disparity_sign,
            raw_subscriptions=self.worker is not None, counts=dict(self.counts), latest=dict(self.last_detail),
            range_convention='physical_left_camera_euclidean_m', physical_hit_verified=False))

    def destroy_node(self):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=5.)
        return super().destroy_node()


def main():
    spin(StereoDepthNode)
