"""Image-only recognition of a physical red/amber/green three-lamp fixture.

Coordinates are relative to the supplied image (the caller may crop an ROI).
Colored lenses establish fixture geometry, not permission to drive. An active
lamp additionally needs a broad bright core, a core-to-rim contrast and a clear
brightness separation from the other red/green lens. A clipped white core is
accepted only when its red/green surround and its fixture position agree.

These photometric gates are a commissioning heuristic, not a calibrated light
meter. Reflections, very dim lamps and heavily clipped/cropped fixtures may be
ambiguous. The caller must keep unknown/conflicting evidence from authorizing
travel and apply its independent temporal/vehicle gates.
"""
from __future__ import annotations

import math

import cv2
import numpy as np


DEFAULT_THREE_LAMP = {
    "three_lamp_saturation_min": 65,
    "three_lamp_lens_value_min": 35,
    "three_lamp_yellow_hue_range": [13, 38],
    "three_lamp_green_hue_range": [39, 90],
    "three_lamp_min_blob_area": 12,
    "three_lamp_max_candidates": 12,
    "three_lamp_tilt_max_deg": 65.0,
    "three_lamp_dark_value_max": 100,
    "three_lamp_dark_support_min": 0.25,
    "three_lamp_active_core_min": 220,
    "three_lamp_bright_value_min": 230,
    "three_lamp_bright_fraction_min": 0.45,
    "three_lamp_core_rim_difference_min": 22,
    "three_lamp_active_difference_min": 35,
    "three_lamp_active_ratio_min": 1.20,
    "three_lamp_color_fraction_min": 0.16,
}


def _blobs(mask, cfg, kind):
    candidate_limit = max(1, min(16, int(cfg["three_lamp_max_candidates"])))
    contours, _ = cv2.findContours(mask.astype(np.uint8) * 255,
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    values = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        area = float(cv2.contourArea(contour))
        if area < cfg["three_lamp_min_blob_area"]:
            break
        x, y, width, height = cv2.boundingRect(contour)
        perimeter = cv2.arcLength(contour, True)
        if (not 0.43 <= width / height <= 1.85 or perimeter <= 0
                or 4 * math.pi * area / perimeter ** 2 < 0.38
                or area > mask.size * 0.24):
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        values.append(dict(center=np.array([moments["m10"], moments["m01"]]) / moments["m00"],
                           radius=math.sqrt(area / math.pi), kind=kind))
        if len(values) >= candidate_limit:
            break
    return values


def _lens_stats(hsv, mask, lamp, cfg):
    cx, cy = lamp["center"]
    radius = lamp["radius"]
    height, width = hsv.shape[:2]
    x0, y0 = max(0, int(cx - radius * 1.35)), max(0, int(cy - radius * 1.35))
    x1, y1 = min(width, int(cx + radius * 1.35) + 1), min(height, int(cy + radius * 1.35) + 1)
    patch = hsv[y0:y1, x0:x1]
    yy, xx = np.ogrid[y0:y1, x0:x1]
    squared = (xx - cx) ** 2 + (yy - cy) ** 2
    core = squared <= (radius * 0.55) ** 2
    face = squared <= (radius * 1.35) ** 2
    rim = (squared >= (radius * 0.83) ** 2) & face
    if np.count_nonzero(core) < 6 or np.count_nonzero(rim) < 6:
        return None
    color = mask[y0:y1, x0:x1]
    # Retain dark colored parts of the lens in the rim measurement. White cores
    # need color in the wider face, so a white room light cannot stand in alone.
    colored_rim = rim & color
    rim_values = patch[:, :, 2][colored_rim if np.count_nonzero(colored_rim) >= 6 else rim]
    core_values = patch[:, :, 2][core]
    core_white = (patch[:, :, 2] >= 240) & (patch[:, :, 1] <= 65)
    return {
        "core_value": float(np.median(core_values)),
        "rim_value": float(np.median(rim_values)),
        "bright_fraction": float(np.mean(core_values >= cfg["three_lamp_bright_value_min"])),
        "white_core_fraction": float(np.mean(core_white[core])),
        "color_fraction": float(np.mean(color[face])),
        "core_color_fraction": float(np.mean(color[core])),
    }


def _dark_support(hsv, lamps, cfg):
    top, middle, bottom = lamps
    axis = bottom["center"] - top["center"]
    distance = float(np.linalg.norm(axis))
    if distance <= 0:
        return None
    axis /= distance
    normal = np.array([axis[1], -axis[0]])
    radius = max(l["radius"] for l in lamps)
    start = top["center"] - axis * radius * 1.1
    end = bottom["center"] + axis * radius * 1.1
    corners = np.array([start + normal * radius * 1.15, start - normal * radius * 1.15,
                        end - normal * radius * 1.15, end + normal * radius * 1.15])
    height, width = hsv.shape[:2]
    # A fixture clipped by the input image cannot establish all three lenses.
    for lamp in lamps:
        cx, cy = lamp["center"]
        if min(cx, cy, width - 1 - cx, height - 1 - cy) < lamp["radius"] * 0.65:
            return None
    x0, y0 = np.maximum(0, np.floor(corners.min(axis=0))).astype(int)
    x1, y1 = np.minimum([width, height], np.ceil(corners.max(axis=0)) + 1).astype(int)
    yy, xx = np.ogrid[y0:y1, x0:x1]
    dx, dy = xx - top["center"][0], yy - top["center"][1]
    along, side = dx * axis[0] + dy * axis[1], dx * normal[0] + dy * normal[1]
    support = ((along >= -radius * 1.1) & (along <= distance + radius * 1.1)
               & (np.abs(side) <= radius * 1.15))
    for lamp in lamps:
        cx, cy = lamp["center"]
        support &= (xx - cx) ** 2 + (yy - cy) ** 2 >= (lamp["radius"] * 0.92) ** 2
    if np.count_nonzero(support) < 12:
        return None
    fraction = float(np.mean(hsv[y0:y1, x0:x1, 2][support] <= cfg["three_lamp_dark_value_max"]))
    if fraction < cfg["three_lamp_dark_support_min"]:
        return None
    return [int(x0), int(y0), int(x1), int(y1)], fraction


def _active(stats, cfg):
    return (stats["color_fraction"] >= cfg["three_lamp_color_fraction_min"]
            and stats["core_value"] >= cfg["three_lamp_active_core_min"]
            and stats["bright_fraction"] >= cfg["three_lamp_bright_fraction_min"]
            and (stats["white_core_fraction"] >= 0.35 or
                 stats["core_value"] - stats["rim_value"] >= cfg["three_lamp_core_rim_difference_min"]))


def _classify(red, green, cfg):
    red_on, green_on = _active(red, cfg), _active(green, cfg)
    if red_on and green_on:
        return "conflict", "three_lamp_both_bright"
    for color, active, own, other in (("red", red_on, red, green), ("green", green_on, green, red)):
        if (active and own["core_value"] - other["core_value"] >= cfg["three_lamp_active_difference_min"]
                and own["core_value"] >= other["core_value"] * cfg["three_lamp_active_ratio_min"]):
            return color, "three_lamp_active_" + color
    return "unknown", "three_lamp_brightness_ambiguous"


def detect_three_lamp_fixture(bgr, cfg=None):
    """Return image-local fixture evidence, or None when no supported triplet exists.

    Never fall back from an identified but ambiguous three-lamp fixture to a
    color-only detector. Input is expected to be a bounded working image (640 px
    wide in RoadVision), so per-color candidate counts cap the geometric search.
    """
    cfg = {**DEFAULT_THREE_LAMP, **(cfg or {})}
    if (bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8
            or min(bgr.shape[:2]) < 8):
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    lens = (saturation >= cfg["three_lamp_saturation_min"]) & (value >= cfg["three_lamp_lens_value_min"])
    red = lens & ((hue <= 12) | (hue >= 168))
    low, high = cfg["three_lamp_yellow_hue_range"]
    yellow = lens & (hue >= low) & (hue <= high)
    low, high = cfg["three_lamp_green_hue_range"]
    green = lens & (hue >= low) & (hue <= high)
    white = (saturation <= 65) & (value >= 240)
    middle_lamps, bottom_lamps = _blobs(yellow, cfg, "yellow"), _blobs(green, cfg, "green")
    if not middle_lamps or not bottom_lamps:
        return None
    top_lamps = _blobs(red, cfg, "red") + _blobs(white, cfg, "white")
    if not top_lamps:
        return None
    fixtures = []
    for middle in middle_lamps:
        for bottom in bottom_lamps:
            step = bottom["center"] - middle["center"]
            spacing = float(np.linalg.norm(step))
            radius = (middle["radius"] + bottom["radius"]) / 2
            if (not 1.65 <= spacing / radius <= 3.6 or step[1] <= 0
                    or abs(math.degrees(math.atan2(step[0], step[1]))) > cfg["three_lamp_tilt_max_deg"]
                    or not 0.60 <= middle["radius"] / bottom["radius"] <= 1.65):
                continue
            prediction = middle["center"] - step
            candidates = sorted(top_lamps, key=lambda top: float(np.linalg.norm(top["center"] - prediction)))
            for top in candidates:
                error = float(np.linalg.norm(top["center"] - prediction))
                if error > spacing * 0.30 or not 0.55 <= top["radius"] / radius <= 1.6:
                    continue
                top_stats, bottom_stats = _lens_stats(hsv, red, top, cfg), _lens_stats(hsv, green, bottom, cfg)
                if (top_stats is None or bottom_stats is None
                        or top_stats["color_fraction"] < cfg["three_lamp_color_fraction_min"]
                        or bottom_stats["color_fraction"] < cfg["three_lamp_color_fraction_min"]):
                    continue
                support = _dark_support(hsv, (top, middle, bottom), cfg)
                if support is None:
                    continue
                bbox, dark_fraction = support
                center = ((top["center"] + bottom["center"]) / 2).tolist()
                # Red rim and its white core can yield overlapping proposals.
                if any(np.linalg.norm(np.array(previous["center"]) - center) < radius for previous in fixtures):
                    continue
                status, reason = _classify(top_stats, bottom_stats, cfg)
                fixtures.append(dict(status=status, reason=reason, fixture_bbox=bbox, center=center,
                    lamps=[dict(color=color, center=lamp["center"].tolist(), radius=float(lamp["radius"]))
                           for color, lamp in (("red", top), ("yellow", middle), ("green", bottom))],
                    photometry={"red": top_stats, "green": bottom_stats},
                    dark_support_fraction=dark_fraction, layout_error_ratio=error / spacing))
                break
    if not fixtures:
        return None
    if len(fixtures) > 1:
        statuses = {fixture["status"] for fixture in fixtures}
        conflict = "red" in statuses and "green" in statuses or "conflict" in statuses
        return dict(status="conflict" if conflict else "unknown", confidence=0.0,
                    reason="three_lamp_multiple_fixtures", fixture_bbox=None, center=None, candidates=fixtures)
    fixture = fixtures[0]
    return dict(status=fixture["status"], confidence=0.9 if fixture["status"] in ("red", "green") else 0.0,
                reason=fixture["reason"], fixture_bbox=fixture["fixture_bbox"], center=fixture["center"],
                candidates=fixtures, fixture_layout="three_lamp",
                photometry=fixture["photometry"])
