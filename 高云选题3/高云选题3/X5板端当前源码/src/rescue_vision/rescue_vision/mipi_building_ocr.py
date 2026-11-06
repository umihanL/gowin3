"""Pure helpers for task-gated, exact-capture building-sign OCR.

An OCR reading proves a visible sign, not ownership of every object in a view.
The matched YOLO snapshot is kept historical; its stamp is never refreshed.
"""
from collections import OrderedDict
import copy
import json
import math

from .building_ocr import BuildingTracker


def corrected_observation_image(frame, camera, size=(1280, 1088)):
    """View the exact lossless detector pixels; never remap a corrected image.

    The CameraInfo is validated with corrected_camera before reaching here.
    Packed BGR is an explicit producer contract, not a guessed image format.
    The immutable serialized message owns the pixel storage through the view.
    """
    from .fire_aim_geometry import CORRECTED_FRAME
    from .mipi_image import image_storage_view
    if (camera is None or not camera.ready or camera.size != tuple(size)
            or camera.doc.get('coordinate_frame') != CORRECTED_FRAME):
        raise ValueError('Validated corrected-image CameraInfo is required for OCR')
    if (frame.header.frame_id != CORRECTED_FRAME or frame.encoding != 'bgr8'
            or (frame.width, frame.height) != tuple(size) or frame.step != size[0]*3):
        raise ValueError('OCR requires original packed corrected BGR pixels and their exact frame')
    image = image_storage_view(frame.data, frame.width, frame.height,
                               frame.step, frame.encoding).reshape(size[1], size[0], 3)
    image.setflags(write=False)
    return image


def _fresh(stamp, now, ttl):
    return (type(stamp) in (int, float) and math.isfinite(stamp)
            and math.isfinite(now) and 0 <= now-stamp <= ttl)


def observation_signature(mission, now_ros, receive_age_s, *, require_settled_pose=True):
    """No OCR of the road, during a scan, or after task/pose permission expires."""
    if (not isinstance(mission, dict) or not _fresh(mission.get('stamp'), now_ros, .5)
            or not 0 <= receive_age_s <= .5 or mission.get('mode') not in ('inspect', 'fire')
            or mission.get('traffic_active') is not False
            or (require_settled_pose and mission.get('gimbal_ready') is not True)
            or mission.get('motion_allowed') is not False):
        return None
    # Changes invalidate in-flight OCR even if the new mode later returns to the old one.
    return json.dumps([mission.get(k) for k in
        ('mode', 'gimbal_target_deg', 'observation_pose_id', 'task_id')], sort_keys=True)


class ExactCaptureJoin:
    """Bounded raw-frame storage; select only images actually processed by YOLO."""
    def __init__(self, raw_limit=8, detection_limit=24):
        if not 1 <= raw_limit <= 16 or not 1 <= detection_limit <= 64:
            raise ValueError('capture cache limits are outside supported bounds')
        self.raw_limit, self.detection_limit = raw_limit, detection_limit
        self.raw, self.detections = OrderedDict(), OrderedDict()
        self.consumed = -1

    def clear(self):
        self.raw.clear()
        self.detections.clear()
        self.consumed = -1

    @staticmethod
    def _put(store, key, value, limit):
        if type(key) is not int or key <= 0:
            raise ValueError('capture timestamp must be positive integer nanoseconds')
        store[key] = value
        while len(store) > limit:
            store.popitem(last=False)

    def push_raw(self, stamp_ns, frame):
        self._put(self.raw, stamp_ns, frame, self.raw_limit)

    def push_detection(self, stamp_ns, detection):
        self._put(self.detections, stamp_ns, detection, self.detection_limit)

    def take(self, now_ros, after_ros, max_input_age=.5):
        candidates = [key for key in self.raw.keys() & self.detections.keys()
            if key > self.consumed and key/1e9 >= after_ros
            and _fresh(key/1e9, now_ros, max_input_age)
            and self.detections[key].get('detector_available') is True]
        if not candidates:
            return None
        key = max(candidates)
        self.consumed = key
        frame, detection = self.raw[key], self.detections[key]
        for store in (self.raw, self.detections):
            for old in [stamp for stamp in store if stamp <= key]:
                del store[old]
        return key, frame, copy.deepcopy(detection)


class BuildingObservation:
    """Multi-capture sign confirmation tied to a single unchanged camera pose."""
    def __init__(self, config, hits=3, interval_s=1., max_result_age_s=3.):
        if not .5 <= interval_s <= 5 or not .5 <= max_result_age_s <= 5:
            raise ValueError('invalid OCR interval or evidence age')
        self.config, self.hits = config, int(hits)
        if not 2 <= self.hits <= 5:
            raise ValueError('OCR requires 2..5 distinct captures')
        self.interval_s, self.max_age = float(interval_s), float(max_result_age_s)
        self.reset()

    def reset(self):
        self.tracker = BuildingTracker(self.config, hits=self.hits, window=5,
            max_span_s=max(4., self.interval_s*(self.hits+1)))

    @staticmethod
    def invalid(now, reason, generation):
        return dict(stamp=now, capture_stamp=None, valid=False, status=reason,
            generation=generation, sequence=None, coordinate_frame=None, readings=[],
            building=dict(confirmed=False, status=reason, building_id=None),
            matched_detection=None, same_capture_verified=False,
            object_building_association_verified=False)

    def finish(self, readings, detection, stamp_ns, now_ros, generation,
               *, current_generation, active, coordinate_frame):
        capture = stamp_ns/1e9
        if not active or generation != current_generation:
            self.reset()
            return self.invalid(now_ros, 'task_or_pose_changed', current_generation)
        if not _fresh(capture, now_ros, self.max_age):
            self.reset()
            return self.invalid(now_ros, 'ocr_result_expired', generation)
        if (detection.get('capture_stamp_ns') != stamp_ns
                or detection.get('coordinate_frame') != coordinate_frame
                or detection.get('detector_available') is not True):
            self.reset()
            return self.invalid(now_ros, 'capture_or_geometry_mismatch', generation)
        if not isinstance(readings, list) or len(readings) > 128:
            self.reset()
            return self.invalid(now_ros, 'invalid_ocr_readings', generation)
        building = self.tracker.update(readings, capture, str(stamp_ns))
        # Losing a sign or seeing conflicting signs starts a new confirmation run.
        if building['status'] in ('unmatched', 'ambiguous'):
            self.reset()
        return dict(stamp=capture, capture_stamp=capture, capture_stamp_ns=stamp_ns,
            processed_stamp=now_ros, valid=True, status=building['status'],
            generation=generation, sequence=detection['sequence'],
            coordinate_frame=coordinate_frame, readings=readings, building=building,
            matched_detection=copy.deepcopy(detection), same_capture_verified=True,
            object_building_association_verified=False,
            evidence_ttl_s=self.max_age,
            note='Confirmed sign only; full-view detections are not assigned to this building.')
