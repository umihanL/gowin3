"""Conservative single-frame zebra evidence; it never issues motion commands.

The caller must confirm several independent captures in a verified road-facing
pose. This detector supports parallel bars across or along the image; neither a
single stop line nor the two road boundaries establish a crossing.
"""
from __future__ import annotations

import math

import cv2
import numpy as np


DEFAULT_CROSSWALK = {
    "roi_normalized": [0.08, 0.28, 0.92, 0.98],
    "white_value_min": 110,
    "otsu_margin": 12,
    "contrast_min": 50,
    "dark_value_max": 150,
    "stripe_min_count": 6,
    "stripe_min_area_px": 22,
    "stripe_min_aspect": 1.6,
    "stripe_min_fill": 0.60,
    "parallel_tolerance_deg": 28,
    "stripe_length_ratio_max": 3.5,
    "spacing_cv_max": 0.60,
    "span_fraction_min": 0.22,
    "white_fraction_max": 0.72,
}


class CrosswalkDetector:
    def __init__(self, config=None):
        self.cfg = {**DEFAULT_CROSSWALK, **(config or {})}
        c = self.cfg
        roi = c["roi_normalized"]
        if len(roi) != 4 or not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1):
            raise ValueError("crosswalk ROI must be [x0,y0,x1,y1] within 0..1")
        if (isinstance(c["stripe_min_count"], bool) or int(c["stripe_min_count"]) != c["stripe_min_count"]
                or not 6 <= c["stripe_min_count"] <= 30):
            raise ValueError("at least six independent zebra bars are required")
        for key in ("contrast_min", "stripe_min_area_px", "stripe_min_aspect", "spacing_cv_max"):
            if not isinstance(c[key], (int, float)) or not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(key + " must be finite and positive")
        if not 0 < c["parallel_tolerance_deg"] < 45:
            raise ValueError("parallel tolerance must be within (0,45) degrees")

    def detect(self, image):
        result = dict(valid=False, detected=False, classification="none", confidence=0.0,
                      stripe_count=0, bbox=None, reason="invalid_image")
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            return result
        if image.ndim == 3 and image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif image.ndim == 2:
            gray = image
        else:
            return result
        height, width = gray.shape
        result.update(image_size=[width, height])
        if min(width, height) < 40:
            return result
        c = self.cfg
        x0, y0, x1, y1 = c["roi_normalized"]
        ox, oy = int(width*x0), int(height*y0)
        crop = gray[oy:int(height*y1), ox:int(width*x1)]
        smooth = cv2.GaussianBlur(crop, (3, 3), 0)
        threshold, _ = cv2.threshold(smooth, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        threshold = min(254, max(float(c["white_value_min"]), threshold + c["otsu_margin"]))
        mask = (smooth >= threshold).astype(np.uint8)*255
        result.update(valid=True, reason="insufficient_parallel_stripes", threshold=threshold)
        if np.mean(mask > 0) > c["white_fraction_max"]:
            result["reason"] = "mostly_bright_surface"
            return result
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            rect = cv2.minAreaRect(contour)
            (cx, cy), (rw, rh), _ = rect
            long, short = max(rw, rh), min(rw, rh)
            if (area < c["stripe_min_area_px"] or short < 2 or long/short < c["stripe_min_aspect"]
                    or area/max(1, rw*rh) < c["stripe_min_fill"]):
                continue
            corners = cv2.boxPoints(rect)
            edges = np.roll(corners, -1, axis=0)-corners
            axis = edges[np.argmax(np.linalg.norm(edges, axis=1))]/long
            candidates.append(dict(center=np.array([cx, cy]), axis=axis, length=long, width=short,
                                   box=cv2.boundingRect(contour), contour=contour))
        tolerance = math.cos(math.radians(c["parallel_tolerance_deg"]))
        best = None
        for seed in candidates:
            along = seed["axis"]
            across = np.array([-along[1], along[0]])
            group = [v for v in candidates if abs(float(v["axis"]@along)) >= tolerance
                     and abs(float((v["center"]-seed["center"])@along)) <= max(v["length"], seed["length"])*0.50]
            if len(group) < c["stripe_min_count"]:
                continue
            group.sort(key=lambda v: float(v["center"]@across))
            positions = np.array([v["center"]@across for v in group])
            gaps = np.diff(positions)
            lengths = [v["length"] for v in group]
            if (np.min(gaps) <= 2 or np.std(gaps)/np.mean(gaps) > c["spacing_cv_max"]
                    or max(lengths)/min(lengths) > c["stripe_length_ratio_max"]):
                continue
            # White stripes must have distinct dark gaps, not overlapping fragments.
            if any(gaps[i] <= .5*(group[i]["width"] + group[i+1]["width"]) + 1
                   for i in range(len(gaps))):
                continue
            x = min(v["box"][0] for v in group)
            y = min(v["box"][1] for v in group)
            right = max(v["box"][0]+v["box"][2] for v in group)
            bottom = max(v["box"][1]+v["box"][3] for v in group)
            if max(right-x, bottom-y) < min(crop.shape)*c["span_fraction_min"]:
                continue
            region = smooth[y:bottom, x:right]
            region_mask = mask[y:bottom, x:right] > 0
            bright, dark = region[region_mask], region[~region_mask]
            if bright.size < 20 or dark.size < 20:
                continue
            white_median, dark_median = float(np.median(bright)), float(np.median(dark))
            if dark_median > c["dark_value_max"] or white_median-dark_median < c["contrast_min"]:
                continue
            spacing_quality = 1-min(1, float(np.std(gaps)/np.mean(gaps)))
            confidence = min(.95, .60 + .20*spacing_quality + .02*(len(group)-c["stripe_min_count"]))
            if best is None or len(group) > best["stripe_count"]:
                best = dict(detected=True, classification="crosswalk", confidence=confidence,
                            stripe_count=len(group), bbox=[x+ox, y+oy, right+ox, bottom+oy],
                            reason="parallel_separated_bright_bars_on_dark_road",
                            contrast=white_median-dark_median, spacing_cv=float(np.std(gaps)/np.mean(gaps)))
        if best:
            result.update(best)
        return result


def observation_mode(mission, *, mission_age_s, capture_ros_s, now_ros_s,
                     frame_max_age_s=.30, mission_max_age_s=.50):
    """Pure gating used by the ROS adapter; receipt time alone is not freshness."""
    if not isinstance(mission, dict):
        return None
    stamp = mission.get("stamp")
    values = (stamp, mission_age_s, capture_ros_s, now_ros_s)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        return None
    if not (0 <= mission_age_s <= mission_max_age_s and 0 <= now_ros_s-stamp <= mission_max_age_s
            and 0 <= now_ros_s-capture_ros_s <= frame_max_age_s and mission.get("gimbal_ready") is True):
        return None
    if mission.get("traffic_active") is True:
        return "traffic"
    if mission.get("mode") in ("standby", "patrol", "navigate") and mission.get("camera_pose") == "road":
        return "road"
    return None
