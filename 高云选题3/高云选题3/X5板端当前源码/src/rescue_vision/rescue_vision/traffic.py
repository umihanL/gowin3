"""Conservative LED traffic light recognition and crossing state machine.

No ROS, serial, servo, or GPIO operations are performed here. All timestamps
must be seconds from the same monotonic clock; frame IDs identify exposures.
"""
from __future__ import annotations

import math
from collections import deque

import cv2
import numpy as np

from .traffic_lamps import detect_three_lamp_fixture


DEFAULT_DETECTOR = {
    "fixture_layout": "two_lamp",
    "roi_normalized": [0.10, 0.05, 0.90, 0.90],
    "board_dark_value_max": 80,
    "board_min_area_px": 120,
    "board_max_roi_fraction": 0.65,
    "board_aspect_min": 0.28,
    "board_aspect_max": 0.85,
    "board_dark_fraction_min": 0.50,
    "led_saturation_min": 110,
    "led_value_min": 150,
    "led_min_area_px": 6,
    "led_area_fraction_min": 0.004,
    "led_area_fraction_max": 0.15,
    "led_circularity_min": 0.48,
    "red_hue_ranges": [[0, 12], [168, 179]],
    "green_hue_range": [38, 88],
    "red_position_normalized": [0.50, 0.27],
    "green_position_normalized": [0.50, 0.73],
    "position_tolerance": 0.22,
    "overexposed_white_fraction_max": 0.12,

}

DEFAULT_CONTROLLER = {
    "stop_policy": "feedback",
    "stop_delay_s": 2.0,
    "pan_min_deg": -60.0,
    "pan_max_deg": 60.0,
    "pan_step_deg": 20.0,
    "pan_initial_deg": 0.0,
    "tilt_deg": 5.0,
    "settle_s": 0.30,
    "scan_dwell_s": 0.45,
    "frame_max_age_s": 0.30,
    "lost_scan_after_s": 0.65,
    "green_min_frames": 5,
    "green_min_duration_s": 0.40,
    "green_max_gap_s": 0.22,
    "release_valid_s": 0.45,
    "event_timeout_s": 45.0,
    "fixture_center_gate": 0.45,
    "fixture_min_iou": 0.20,
    "confidence_min": 0.50,
}


class TrafficDetector:
    """Detect the explicitly configured two-lamp or red/yellow/green fixture.

    A colored patch alone is never a valid lamp. A supported observation needs
    a dark rectangular panel, compact bright saturated color, and its expected
    position on the panel. This is a commissioning baseline, not proof that a
    real LED fixture is recognized in all lighting conditions.
    """

    def __init__(self, config=None):
        config = config or {}
        self.cfg = {**DEFAULT_DETECTOR, **config.get("detector", config)}
        if self.cfg['fixture_layout'] not in ('two_lamp', 'three_lamp'):
            raise ValueError('fixture_layout must be two_lamp or three_lamp')
        roi = self.cfg["roi_normalized"]
        if len(roi) != 4 or not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1):
            raise ValueError("roi_normalized must be [x0,y0,x1,y1] within 0..1")

    def detect(self, bgr, *, capture_time=None, frame_id=None):
        result = {"status": "unknown", "confidence": 0.0, "fixture_bbox": None,
                  "center": None, "capture_time": capture_time, "frame_id": frame_id,
                  "reason": "no_supported_lamp", "candidates": []}
        if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            result["reason"] = "invalid_image"
            return result
        height, width = bgr.shape[:2]
        result["image_size"] = [width, height]
        x0, y0, x1, y1 = self.cfg["roi_normalized"]
        ox, oy = int(x0 * width), int(y0 * height)
        crop = bgr[oy:int(y1 * height), ox:int(x1 * width)]
        if crop.size == 0:
            return result
        if self.cfg['fixture_layout'] == 'three_lamp':
            # Unlit covers are colored too. The configured real fixture must
            # never fall back to the legacy two-lamp color threshold.
            found = detect_three_lamp_fixture(crop, self.cfg)
            if found is None:
                result['reason'] = 'no_supported_three_lamp_fixture'
                return result
            result.update(found)
            def offset(item):
                if item.get('fixture_bbox') is not None:
                    a,b,c,d = item['fixture_bbox']
                    item['fixture_bbox'] = [a+ox,b+oy,c+ox,d+oy]
                if item.get('center') is not None:
                    a,b = item['center']; item['center'] = [a+ox,b+oy]
            offset(result)
            for candidate in result.get('candidates', []):
                offset(candidate)
                for lamp in candidate.get('lamps', []):
                    offset(lamp)
            result.update(capture_time=capture_time, frame_id=frame_id, image_size=[width,height])
            return result
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        dark = (hsv[:, :, 2] <= self.cfg["board_dark_value_max"]).astype(np.uint8) * 255
        contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        recognized, glare = [], False
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            if (area < self.cfg["board_min_area_px"] or area > dark.size * self.cfg["board_max_roi_fraction"]
                    or not self.cfg["board_aspect_min"] <= w / h <= self.cfg["board_aspect_max"]
                    or x <= 0 or y <= 0 or x + w >= crop.shape[1] or y + h >= crop.shape[0]):
                continue
            if cv2.contourArea(contour) / area < 0.78:
                continue
            panel = hsv[y:y+h, x:x+w]
            if np.mean(panel[:, :, 2] <= self.cfg["board_dark_value_max"]) < self.cfg["board_dark_fraction_min"]:
                continue
            white = (panel[:, :, 2] >= 250) & (panel[:, :, 1] <= 35)
            if np.mean(white) > self.cfg["overexposed_white_fraction_max"]:
                glare = True
                continue
            lamps = []
            for color in ("red", "green"):
                hue_mask = np.zeros((h, w), dtype=bool)
                ranges = self.cfg["red_hue_ranges"] if color == "red" else [self.cfg["green_hue_range"]]
                for low, high in ranges:
                    hue_mask |= (panel[:, :, 0] >= low) & (panel[:, :, 0] <= high)
                mask = (hue_mask & (panel[:, :, 1] >= self.cfg["led_saturation_min"])
                        & (panel[:, :, 2] >= self.cfg["led_value_min"])).astype(np.uint8) * 255
                blobs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for blob in blobs:
                    blob_area = cv2.contourArea(blob)
                    perimeter = cv2.arcLength(blob, True)
                    if (blob_area < max(self.cfg["led_min_area_px"], area * self.cfg["led_area_fraction_min"])
                            or blob_area > area * self.cfg["led_area_fraction_max"] or perimeter <= 0
                            or 4 * math.pi * blob_area / perimeter**2 < self.cfg["led_circularity_min"]):
                        continue
                    moments = cv2.moments(blob)
                    if moments["m00"] <= 0:
                        continue
                    cx, cy = moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
                    ex, ey = self.cfg[color + "_position_normalized"]
                    position_error = math.hypot(cx / w - ex, cy / h - ey)
                    if position_error > self.cfg["position_tolerance"]:
                        continue
                    lamps.append({"color": color, "center": [ox+x+cx, oy+y+cy],
                                  "position_error": position_error})
            if lamps:
                recognized.append({"fixture_bbox": [ox+x, oy+y, ox+x+w, oy+y+h],
                                   "center": [ox+x+w/2, oy+y+h/2], "lamps": lamps})
        result["candidates"] = recognized
        colors = {lamp["color"] for board in recognized for lamp in board["lamps"]}
        if "red" in colors and "green" in colors:
            result.update(status="conflict", reason="red_and_green_visible")
        elif glare:
            result.update(status="overexposed", reason="panel_glare")
        elif len(recognized) > 1:
            result.update(status="unknown", reason="multiple_fixtures_in_roi")
        elif recognized:
            board = recognized[0]
            result.update(status=next(iter(colors)), confidence=0.9,
                          fixture_bbox=board["fixture_bbox"], center=board["center"],
                          reason="supported_panel_and_led")
        return result


class TrafficController:
    """FPGA crosswalk event -> stop -> bounded scan -> stable green grant.

    ``gimbal_ready`` must come from the integration layer. It can mean real
    feedback or acknowledged command plus a measured travel wait, but must not
    be generated just by assigning the desired angle to an 'actual' variable.
    ``move_allowed`` is permission for this crossing only, never a motor command.
    """

    def __init__(self, config=None):
        config = config or {}
        self.cfg = {**DEFAULT_CONTROLLER, **config.get("controller", config)}
        c = self.cfg
        if c['stop_policy'] not in ('feedback', 'delay') or not .5 <= float(c['stop_delay_s']) <= 10:
            raise ValueError('invalid traffic stop policy')
        self._stop_wait_started = None
        if not (c["pan_min_deg"] <= c["pan_initial_deg"] <= c["pan_max_deg"]
                and c["pan_step_deg"] > 0 and c["green_min_frames"] >= 2
                and c["settle_s"] >= 0 and c["scan_dwell_s"] > 0
                and c["frame_max_age_s"] > 0 and c["release_valid_s"] > 0):
            raise ValueError("invalid controller limits or timing")
        self.state = "IDLE"
        self.event_id = None
        self.pan = float(c["pan_initial_deg"])
        self.direction = -1
        self._seen_events = deque(maxlen=256)
        self._clear_tracking()
        self._start = self._command_time = 0.0
        self._ready_since = None
        self._last_frame = None
        self._frame_capture = None
        self._grant_until = 0.0
        self.reason = "no_crosswalk_event"

    def _clear_tracking(self):
        self._fixture = None
        self._last_seen = None
        self._green_times = []

    def on_crosswalk(self, event_id, now_s):
        event_id = str(event_id)
        if not event_id or event_id in self._seen_events or self.event_id is not None:
            return False
        if not math.isfinite(float(now_s)):
            raise ValueError("invalid clock")
        self._seen_events.append(event_id)
        self.event_id = event_id
        self.state, self.reason = "WAIT_STOP", "crosswalk_stop_requested"
        self._start = self._command_time = float(now_s)
        self._stop_wait_started = float(now_s)
        self.pan = float(self.cfg["pan_initial_deg"])
        self.direction = -1
        self._ready_since = None
        self._last_frame = self._frame_capture = None
        self._grant_until = 0.0
        self._clear_tracking()
        return True

    def on_crossing_complete(self, event_id):
        if str(event_id) != self.event_id or self.state != "RELEASED":
            return False
        self.state, self.reason = "IDLE", "crossing_complete"
        self.event_id = None
        self._grant_until = 0.0
        self._clear_tracking()
        return True

    def _output(self, now_s):
        allowed = self.state == "RELEASED" and now_s < self._grant_until
        delay_done = (self._stop_wait_started is not None and
                      now_s-self._stop_wait_started >= self.cfg['stop_delay_s'])
        return {"state": self.state, "move_allowed": allowed,
                "stop_policy": self.cfg['stop_policy'],
                "stop_wait_complete": bool(delay_done) if self.cfg['stop_policy']=='delay' else None,
                "stop_confirmation_basis": 'elapsed_time_only' if self.cfg['stop_policy']=='delay' else 'fpga_feedback',
                "traffic_active": self.event_id is not None,
                "desired_pan_deg": self.pan, "desired_tilt_deg": float(self.cfg["tilt_deg"]),
                "reason": self.reason, "crosswalk_id": self.event_id,
                "valid_until_s": self._grant_until if allowed else None,
                "green_confirmation_frames": len(self._green_times),
                "actual_pan_deg": None, "actual_tilt_deg": None}

    @staticmethod
    def _valid_bbox(bbox):
        return (isinstance(bbox, (list, tuple)) and len(bbox) == 4
                and all(isinstance(v, (float, int)) and math.isfinite(v) for v in bbox)
                and bbox[2] > bbox[0] and bbox[3] > bbox[1])

    def _same_fixture(self, bbox):
        if self._fixture is None:
            return False
        a, b = self._fixture, bbox
        inter = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
        aa, ba = (a[2]-a[0])*(a[3]-a[1]), (b[2]-b[0])*(b[3]-b[1])
        iou = inter / max(1e-9, aa + ba - inter)
        dist = math.hypot((a[0]+a[2]-b[0]-b[2])/2, (a[1]+a[3]-b[1]-b[3])/2)
        diag = math.hypot(a[2]-a[0], a[3]-a[1])
        return iou >= self.cfg["fixture_min_iou"] and dist <= diag * self.cfg["fixture_center_gate"]

    def _stop(self, state, reason):
        if self.state == 'RELEASED' and state != 'RELEASED' and self.cfg['stop_policy'] == 'delay':
            # A revoked green grant sends STOP again; wait before a new scan.
            self._stop_wait_started = None
        self.state, self.reason = state, reason
        self._green_times.clear()
        self._grant_until = 0.0

    def _scan_next(self, now_s):
        candidate = self.pan + self.direction * self.cfg["pan_step_deg"]
        if candidate < self.cfg["pan_min_deg"] or candidate > self.cfg["pan_max_deg"]:
            self.direction *= -1
            candidate = self.pan + self.direction * self.cfg["pan_step_deg"]
        self.pan = float(np.clip(candidate, self.cfg["pan_min_deg"], self.cfg["pan_max_deg"]))
        self._command_time = now_s
        self._ready_since = None
        self._clear_tracking()
        self._stop("SCANNING", "scan_next_pose")

    def tick(self, now_s, observation=None, vehicle_stopped=False, gimbal_ready=False):
        now_s = float(now_s)
        if not math.isfinite(now_s):
            raise ValueError("invalid clock")
        if self.event_id is None or self.state == "TIMEOUT":
            return self._output(now_s)
        if now_s - self._start >= self.cfg["event_timeout_s"]:
            self._stop("TIMEOUT", "crosswalk_timeout_requires_explicit_reset")
            return self._output(now_s)
        if self.cfg['stop_policy'] == 'delay':
            if self._stop_wait_started is None:
                self._stop_wait_started = now_s
            vehicle_stopped = now_s-self._stop_wait_started >= self.cfg['stop_delay_s']
        if self.state == "WAIT_STOP":
            if not vehicle_stopped:
                return self._output(now_s)
            self._stop("SCANNING", "stop_delay_elapsed_unmeasured" if self.cfg['stop_policy']=='delay' else "vehicle_stop_confirmed")
            self._command_time = now_s
            self._ready_since = None
        # While released the chassis may move. If permission is revoked, all
        # further scanning and green acquisition require a fresh stop report.
        if not vehicle_stopped and self.state != "RELEASED":
            self._stop("WAIT_STOP", "vehicle_not_stopped")
            return self._output(now_s)
        if not gimbal_ready:
            self._ready_since = None
            self._stop("SCANNING", "waiting_for_gimbal_ready")
            return self._output(now_s)
        if self._ready_since is None:
            self._ready_since = now_s
        settled_at = max(self._ready_since, self._command_time) + self.cfg["settle_s"]
        if now_s < settled_at:
            self._stop("SCANNING", "gimbal_settling")
            return self._output(now_s)

        new_observation = False
        if observation is not None:
            capture = observation.get("capture_time")
            frame_id = observation.get("frame_id")
            valid_time = isinstance(capture, (int, float)) and math.isfinite(capture)
            if (not valid_time or frame_id is None or capture > now_s + 0.01
                    or now_s - capture > self.cfg["frame_max_age_s"]):
                self._stop("SCANNING", "invalid_or_stale_frame")
                return self._output(now_s)
            if frame_id != self._last_frame and (self._frame_capture is None or capture > self._frame_capture):
                self._last_frame, self._frame_capture = frame_id, capture
                new_observation = capture >= settled_at
        if new_observation:
            status = observation.get("status", "unknown")
            bbox = observation.get("fixture_bbox")
            if status in ("conflict", "red"):
                self._stop("RED_HOLD", "red_light" if status == "red" else "red_green_conflict")
                if self._valid_bbox(bbox):
                    self._fixture, self._last_seen = list(bbox), now_s
            elif (status == "green" and self._valid_bbox(bbox)
                  and observation.get("confidence", 0) >= self.cfg["confidence_min"]):
                same = self._same_fixture(bbox)
                if not same or (self._green_times and capture-self._green_times[-1] > self.cfg["green_max_gap_s"]):
                    self._green_times.clear()
                    self._grant_until = 0.0
                self._fixture, self._last_seen = list(bbox), now_s
                self._green_times.append(float(capture))
                self._green_times = self._green_times[-64:]
                stable = (len(self._green_times) >= self.cfg["green_min_frames"]
                          and self._green_times[-1]-self._green_times[0] >= self.cfg["green_min_duration_s"])
                if stable:
                    self.state, self.reason = "RELEASED", "stable_green_same_fixture"
                    self._grant_until = min(now_s+self.cfg["release_valid_s"],
                                            capture+self.cfg["frame_max_age_s"])
                else:
                    self.state, self.reason = "WAIT_GREEN", "confirming_green_same_fixture"
            else:
                self._stop("WAIT_GREEN" if self._last_seen is not None else "SCANNING",
                           "overexposed" if status == "overexposed" else "unknown_lamp")

        if self.state == "RELEASED" and now_s >= self._grant_until:
            self._stop("WAIT_GREEN", "green_permission_expired")
        if self._frame_capture is not None and now_s-self._frame_capture > self.cfg["frame_max_age_s"]:
            self._stop("SCANNING", "camera_frame_timeout")
        target_lost = self._last_seen is None or now_s-self._last_seen >= self.cfg["lost_scan_after_s"]
        if target_lost and now_s >= settled_at+self.cfg["scan_dwell_s"]:
            if vehicle_stopped:
                self._scan_next(now_s)
            else:
                self._stop("WAIT_STOP", "target_lost_stop_before_scan")
        return self._output(now_s)

    def reset(self):
        """Explicit operator/mission reset, preserving deduplication history."""
        self.event_id = None
        self._clear_tracking()
        self._stop("IDLE", "explicit_reset")
