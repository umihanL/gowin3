"""Calibrated horizontal depth in rectified reference-eye optical XYZ.

X points right, Y down, Z forward. Metres are available only from a measured
calibration whose translation is explicitly expressed in metres.
"""
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


class CalibrationError(ValueError):
    pass


class FrameError(ValueError):
    pass


@dataclass
class StereoResult:
    disparity_px: np.ndarray
    valid_mask: np.ndarray
    xyz_m: np.ndarray
    gray_left: np.ndarray
    frame_id: str = "rectified_left_optical"
    reason: str = "ok"
    origin_xy: tuple = (0, 0)
    full_image_size: Optional[tuple] = None


DEFAULT_CONFIG = {
    "num_disparities": 96,
    "block_size": 5,
    "uniqueness_ratio": 10,
    "lr_max_error_px": 1.0,
    "texture_std_min": 3.0,
    "texture_window": 7,
    "max_stamp_delta_s": 0.020,
    "min_depth_m": 0.15,
    "max_depth_m": 5.0,
    "roi_inset_fraction": 0.15,
    "roi_min_samples": 16,
    "roi_min_valid_ratio": 0.15,
    "roi_max_relative_mad": 0.12,
}


class StereoDepth:
    """Keep one instance per camera pair; do not share it across worker threads.

    A template with calibrated=false constructs successfully so image detection
    can continue. rectify()/compute() then raise CalibrationError rather than
    inventing metric depth. compute() accepts *already rectified* frames only.
    """

    def __init__(self, calibration: dict, config: Optional[dict] = None):
        self.config = {**DEFAULT_CONFIG, **(config or {})}
        self.calibrated = calibration.get("calibrated") is True
        self.calibration_id = calibration.get("calibration_id")
        self.image_size = None
        self.disparity_sign = 1
        self._validate_config()
        if not self.calibrated:
            return
        if not isinstance(self.calibration_id, str) or not self.calibration_id.strip():
            raise CalibrationError("Measured calibration needs a nonempty calibration_id")
        if calibration.get("translation_unit") != "m":
            raise CalibrationError("T must be measured in metres: translation_unit='m'")
        if calibration.get("camera_model", "pinhole") != "pinhole":
            raise CalibrationError("Only the OpenCV pinhole distortion model is supported")
        try:
            width, height = calibration["image_size"]
            if (isinstance(width, bool) or isinstance(height, bool)
                    or int(width) != width or int(height) != height or min(width, height) < 32):
                raise ValueError("image_size must contain positive integer dimensions")
            self.image_size = (int(width), int(height))
            self.K1 = self._matrix(calibration["K1"], (3, 3), "K1")
            self.K2 = self._matrix(calibration["K2"], (3, 3), "K2")
            self.D1 = self._distortion(calibration["D1"], "D1")
            self.D2 = self._distortion(calibration["D2"], "D2")
            self.R = self._matrix(calibration["R"], (3, 3), "R")
            self.T = self._matrix(calibration["T"], (3,), "T").reshape(3, 1)
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationError(f"Invalid measured calibration: {exc}") from exc
        for name, K in (("K1", self.K1), ("K2", self.K2)):
            if (K[0, 0] <= 0 or K[1, 1] <= 0
                    or not np.allclose(K[2], [0, 0, 1], atol=1e-8)
                    or abs(K[0, 1]) > 1e-8 or abs(K[1, 0]) > 1e-8):
                raise CalibrationError(f"{name} is not a valid pinhole camera matrix")
        if (not np.allclose(self.R.T @ self.R, np.eye(3), atol=1e-4)
                or abs(np.linalg.det(self.R) - 1.0) > 1e-4):
            raise CalibrationError("R must be a proper orthonormal rotation")
        self.baseline_m = float(np.linalg.norm(self.T))
        if not 0.001 <= self.baseline_m <= 1.0:
            raise CalibrationError("Baseline outside 0.001..1 m; check T units and calibration")
        if abs(self.T[0, 0]) <= max(abs(self.T[1, 0]), abs(self.T[2, 0])):
            raise CalibrationError("This SGBM pipeline requires a horizontal stereo camera pair")
        if self.image_size[0] <= self.config["num_disparities"] + self.config["block_size"]:
            raise CalibrationError("Image width must exceed the disparity search range")
        try:
            (self.R1, self.R2, self.P1, self.P2, self.Q,
             self.roi1, self.roi2) = cv2.stereoRectify(
                self.K1, self.D1, self.K2, self.D2, self.image_size, self.R, self.T,
                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
            if (not np.isfinite(self.P2).all() or not np.isfinite(self.Q).all()
                    or abs(self.P2[0, 3]) <= 1e-12 or abs(self.Q[3, 2]) <= 1e-12
                    or not np.allclose(self.P2[1:, 3], 0, rtol=0, atol=1e-9)):
                raise CalibrationError("Rectification must describe a finite horizontal stereo baseline")
            self.disparity_sign = 1 if self.Q[3, 2] > 0 else -1
            if self.disparity_sign * self.P2[0, 3] >= 0:
                raise CalibrationError("Rectified projection and disparity-to-depth signs disagree")
            self.map1 = cv2.initUndistortRectifyMap(
                self.K1, self.D1, self.R1, self.P1, self.image_size, cv2.CV_32FC1)
            self.map2 = cv2.initUndistortRectifyMap(
                self.K2, self.D2, self.R2, self.P2, self.image_size, cv2.CV_32FC1)
        except cv2.error as exc:
            raise CalibrationError(f"OpenCV cannot rectify calibration: {exc}") from exc
        block = self.config["block_size"]
        common = dict(numDisparities=self.config["num_disparities"], blockSize=block,
                      P1=8 * block * block, P2=32 * block * block,
                      disp12MaxDiff=1, uniquenessRatio=self.config["uniqueness_ratio"],
                      speckleWindowSize=50, speckleRange=2,
                      mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        # The reference eye stays attached to its detections. Swapping the
        # physical baseline changes the valid disparity sign, not the XYZ axes.
        negative_min = -self.config["num_disparities"] + 1
        self.left_matcher = cv2.StereoSGBM_create(
            minDisparity=0 if self.disparity_sign > 0 else negative_min, **common)
        self.right_matcher = cv2.StereoSGBM_create(
            minDisparity=negative_min if self.disparity_sign > 0 else 0, **common)

    def _validate_config(self):
        c = self.config
        for key in ("num_disparities", "block_size", "texture_window", "roi_min_samples", "uniqueness_ratio"):
            if isinstance(c[key], bool) or not isinstance(c[key], (int, np.integer)):
                raise ValueError(f"{key} must be an integer")
        if c["num_disparities"] < 16 or c["num_disparities"] % 16:
            raise ValueError("num_disparities must be a positive multiple of 16")
        if c["block_size"] < 3 or c["block_size"] % 2 == 0:
            raise ValueError("block_size must be odd and >=3")
        if c["texture_window"] < 3 or c["texture_window"] % 2 == 0:
            raise ValueError("texture_window must be odd and >=3")
        for key in ("lr_max_error_px", "texture_std_min", "max_stamp_delta_s", "min_depth_m",
                    "max_depth_m", "roi_inset_fraction", "roi_min_valid_ratio", "roi_max_relative_mad"):
            if not np.isfinite(c[key]):
                raise ValueError(f"{key} must be finite")
        if not 0 < c["min_depth_m"] < c["max_depth_m"]:
            raise ValueError("Depth limits must satisfy 0 < min < max")
        if not (0 <= c["roi_inset_fraction"] < .5 and 0 < c["roi_min_valid_ratio"] <= 1):
            raise ValueError("Invalid ROI fractions")
        if (c["lr_max_error_px"] <= 0 or c["texture_std_min"] <= 0 or c["max_stamp_delta_s"] < 0
                or c["roi_min_samples"] < 1 or c["roi_max_relative_mad"] <= 0
                or c["uniqueness_ratio"] < 0):
            raise ValueError("Invalid stereo quality thresholds")

    @staticmethod
    def _matrix(value, shape, name):
        arr = np.asarray(value, dtype=np.float64)
        if arr.shape != shape or not np.isfinite(arr).all():
            raise ValueError(f"{name} must be finite with shape {shape}")
        return arr

    @staticmethod
    def _distortion(value, name):
        arr = np.asarray(value, dtype=np.float64)
        if arr.ndim != 1 or arr.size not in (4, 5, 8, 12, 14) or not np.isfinite(arr).all():
            raise ValueError(f"{name} must contain 4, 5, 8, 12 or 14 finite coefficients")
        return arr

    def _check_frame(self, frame, expected_size=None):
        if not self.calibrated:
            raise CalibrationError("No measured stereo calibration; metric depth is unavailable")
        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8:
            raise FrameError("Images must be uint8 numpy arrays")
        if frame.ndim not in (2, 3) or (frame.ndim == 3 and frame.shape[2] != 3):
            raise FrameError("Expected grayscale or BGR image")
        expected_size = self.image_size if expected_size is None else expected_size
        if frame.shape[:2][::-1] != expected_size:
            raise FrameError(f"Image resolution must match the declared full-resolution region {expected_size}")

    def _region(self, region):
        if region is None:
            return (0, 0, *self.image_size)
        if (not isinstance(region, (list, tuple)) or len(region) != 4 or
                any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, np.integer)) for v in region)):
            raise FrameError('Rectified processing region must contain four integer coordinates')
        x0, y0, x1, y1 = map(int, region)
        if not (0 <= x0 < x1 <= self.image_size[0] and 0 <= y0 < y1 <= self.image_size[1]):
            raise FrameError('Rectified processing region is outside the calibrated image')
        if x1-x0 <= self.config['num_disparities']+self.config['block_size'] or y1-y0 < self.config['block_size']:
            raise FrameError('Rectified processing region is smaller than matcher support')
        return x0, y0, x1, y1

    def processing_region(self, sample_bounds):
        """Keep native pixels plus both matchers' disparity and texture context."""
        x0, y0, x1, y1 = sample_bounds
        support = max(16, self.config['block_size']*2, self.config['texture_window'])
        horizontal = self.config['num_disparities'] + support
        return self._region((max(0, int(x0)-horizontal), max(0, int(y0)-support),
                             min(self.image_size[0], int(x1)+horizontal),
                             min(self.image_size[1], int(y1)+support)))

    def rectify(self, left, right, region=None):
        self._check_frame(left)
        self._check_frame(right)
        x0, y0, x1, y1 = self._region(region)
        # Only the output maps are cropped. They still sample the unchanged
        # full raw images, with their original subpixel lens correction.
        maps1 = tuple(value[y0:y1, x0:x1] for value in self.map1)
        maps2 = tuple(value[y0:y1, x0:x1] for value in self.map2)
        return (cv2.remap(left, *maps1, cv2.INTER_LINEAR),
                cv2.remap(right, *maps2, cv2.INTER_LINEAR))

    def compute(self, left_rect, right_rect, stamp_delta_s=0, region=None):
        if not self.calibrated:
            raise CalibrationError("No measured stereo calibration; metric depth is unavailable")
        x0, y0, x1, y1 = self._region(region)
        self._check_frame(left_rect, (x1-x0, y1-y0))
        self._check_frame(right_rect, (x1-x0, y1-y0))
        if not np.isfinite(stamp_delta_s) or abs(stamp_delta_s) > self.config["max_stamp_delta_s"]:
            raise FrameError("Stereo capture timestamps differ beyond the synchronization limit")
        left = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY) if left_rect.ndim == 3 else left_rect
        right = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY) if right_rect.ndim == 3 else right_rect
        disp = self.left_matcher.compute(left, right).astype(np.float32) / 16.0
        reverse = self.right_matcher.compute(right, left).astype(np.float32) / 16.0
        h, w = disp.shape
        yy, xx = np.indices((h, w))
        rx = np.rint(xx - disp).astype(np.int32)
        in_bounds = (rx >= 0) & (rx < w)
        sampled_reverse = reverse[yy, np.clip(rx, 0, w - 1)]
        signed_disp = self.disparity_sign * disp
        signed_reverse = self.disparity_sign * sampled_reverse
        valid = ((signed_disp > .1) & (signed_disp < self.config["num_disparities"] - 1)
                 & in_bounds & (signed_reverse < -.1)
                 & (signed_reverse > -self.config["num_disparities"] + 1)
                 & (np.abs(disp + sampled_reverse) <= self.config["lr_max_error_px"]))
        for roi, x_coordinates in ((self.roi1, xx), (self.roi2, rx)):
            x, y, rw, rh = roi
            valid &= ((x_coordinates+x0 >= x) & (x_coordinates+x0 < x+rw)
                      & (yy+y0 >= y) & (yy+y0 < y+rh))
        if (x0, y0, x1, y1) != (0, 0, *self.image_size):
            radius = max(self.config['block_size']//2, self.config['texture_window']//2)
            valid &= ((xx >= radius) & (xx < w-radius) & (rx >= radius) & (rx < w-radius)
                      & (yy >= radius) & (yy < h-radius))
        window = (self.config["texture_window"],) * 2
        for gray, coordinates in ((left, xx), (right, np.clip(rx, 0, w - 1))):
            f = gray.astype(np.float32)
            mean = cv2.boxFilter(f, -1, window, normalize=True)
            variance = np.maximum(cv2.boxFilter(f * f, -1, window, normalize=True) - mean * mean, 0)
            valid &= np.sqrt(variance)[yy, coordinates] >= self.config["texture_std_min"]
        # Q and the original signed disparity must remain paired; abs(disp)
        # would put an inverted-baseline reconstruction behind the camera.
        reprojection = self.Q.copy()
        reprojection[:, 3] += x0*self.Q[:, 0] + y0*self.Q[:, 1]
        xyz = cv2.reprojectImageTo3D(disp, reprojection)
        valid &= (np.isfinite(xyz).all(axis=2)
                  & (xyz[:, :, 2] >= self.config["min_depth_m"])
                  & (xyz[:, :, 2] <= self.config["max_depth_m"]))
        xyz[~valid] = np.nan
        return StereoResult(disp, valid, xyz, left,
                            reason="ok" if valid.any() else "no_reliable_disparity",
                            origin_xy=(x0, y0), full_image_size=self.image_size)

    def sample_bbox(self, result, bbox):
        """Robust facade/target-surface point from a box in rectified-left pixels.

        This is the visible surface depth, not proof that a water jet will hit.
        The ROI estimate cannot disambiguate a large background-dominated box.
        """
        answer = {"valid": False, "xyz_m": None, "depth_m": None,
                  "frame_id": "rectified_left_optical", "valid_ratio": 0.0,
                  "samples": 0, "reason": "invalid_bbox"}
        if not self.calibrated:
            answer["reason"] = "uncalibrated"
            return answer
        coords = np.asarray(bbox, dtype=float)
        if coords.shape != (4,) or not np.isfinite(coords).all():
            return answer
        x1, y1, x2, y2 = coords
        if x2 <= x1 or y2 <= y1:
            return answer
        local_h, local_w = result.valid_mask.shape
        w, h = getattr(result, 'full_image_size', None) or (local_w, local_h)
        offset_x, offset_y = getattr(result, 'origin_xy', (0, 0))
        # A partially clipped target is unsuitable for a precise aiming range.
        if x1 < 0 or y1 < 0 or x2 > w or y2 > h:
            answer["reason"] = "clipped_bbox"
            return answer
        inset = self.config["roi_inset_fraction"]
        dx, dy = (x2 - x1) * inset, (y2 - y1) * inset
        x1, x2 = int(np.ceil(x1 + dx)), int(np.floor(x2 - dx))
        y1, y2 = int(np.ceil(y1 + dy)), int(np.floor(y2 - dy))
        x1, x2, y1, y2 = x1-offset_x, x2-offset_x, y1-offset_y, y2-offset_y
        if x2 <= x1 or y2 <= y1:
            return answer
        if not (0 <= x1 < x2 <= local_w and 0 <= y1 < y2 <= local_h):
            answer['reason'] = 'roi_outside_computed_region'
            return answer
        disparity = result.disparity_px[y1:y2, x1:x2]
        mask = (result.valid_mask[y1:y2, x1:x2] & np.isfinite(disparity)
                & (self.disparity_sign * disparity > 0))
        points = result.xyz_m[y1:y2, x1:x2][mask]
        points = points[np.isfinite(points).all(axis=1)]
        answer["samples"] = len(points)
        answer["valid_ratio"] = float(len(points) / mask.size)
        if len(points) < self.config["roi_min_samples"] or answer["valid_ratio"] < self.config["roi_min_valid_ratio"]:
            answer["reason"] = "insufficient_reliable_depth"
            return answer
        depth = points[:, 2]
        median = float(np.median(depth))
        mad = float(np.median(np.abs(depth - median)))
        answer["depth_mad_m"] = mad
        if mad / median > self.config["roi_max_relative_mad"]:
            answer["reason"] = "mixed_or_unstable_depth"
            return answer
        keep = np.abs(depth - median) <= max(3.0 * 1.4826 * mad, .02 * median)
        points = points[keep]
        if len(points) < self.config["roi_min_samples"]:
            answer["reason"] = "insufficient_inlier_depth"
            return answer
        point = np.median(points, axis=0)
        answer.update(valid=True, xyz_m=point.astype(float).tolist(), depth_m=float(point[2]),
                      samples=len(points), reason="ok")
        return answer
