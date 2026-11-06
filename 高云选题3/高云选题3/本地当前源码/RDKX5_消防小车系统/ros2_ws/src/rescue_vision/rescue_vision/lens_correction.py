"""Candidate monocular lens correction from an explicit CameraInfo snapshot.

This does not perform stereo rectification, validate EEPROM eye identity, or
establish metric depth. The published driver's R/P are deliberately not used:
in this GS130W driver they include inter-camera extrinsics, not a validated
monocular rectification transform. Raw input dimensions/rotation must match.
"""
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class CorrectedImage:
    image: np.ndarray
    camera_matrix: np.ndarray
    metadata: dict

    @property
    def newK(self):
        return self.camera_matrix


def _size(value, name):
    if (not isinstance(value, (tuple, list)) or len(value) != 2 or
            any(isinstance(x, bool) or not isinstance(x, (int, np.integer)) or x <= 0
                for x in value)):
        raise ValueError(name + " must be (positive integer width, height)")
    return tuple(int(x) for x in value)


def _matrix(value, name):
    try:
        out = np.asarray(value, dtype=np.float64).reshape(3, 3)
    except (TypeError, ValueError) as exc:
        raise ValueError(name + " must contain nine numbers") from exc
    if (not np.isfinite(out).all() or out[0, 0] <= 0 or out[1, 1] <= 0 or
            not np.allclose(out[2], [0, 0, 1], atol=1e-9) or
            abs(out[0, 1]) > 1e-9 or abs(out[1, 0]) > 1e-9):
        raise ValueError(name + " must be a finite, positive-focal pinhole matrix")
    return out.copy()


def _flag(value, name):
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, str) and value.lower() in ("true", "false"):
        return value.lower() == "true"
    if isinstance(value, (int, np.integer)) and value in (0, 1):
        return bool(value)
    raise ValueError(name + " must be a boolean")


class LensCorrector:
    """Cache R=I rational-model maps for each explicitly selected source eye.

    Use ``from_json`` with the saved GS130W snapshot. ``role_to_source`` maps
    logical/physical roles to raw ROS topics; the implementation never guesses
    that a topic named left must be the physical left lens.
    """
    def __init__(self, snapshot, *, role_to_source=None, alpha=0.0,
                 remap_backend="opencv", remap_threads=8):
        if remap_backend not in ("opencv", "native", "native_or_opencv"):
            raise ValueError("Unknown remap backend")
        if isinstance(remap_threads, bool) or not isinstance(remap_threads, int) or not 1 <= remap_threads <= 8:
            raise ValueError("Remap threads must be an integer in 1..8")
        self._snapshot = deepcopy(snapshot)
        if not math.isfinite(float(alpha)) or not 0 <= float(alpha) <= 1:
            raise ValueError("alpha must be finite and in [0, 1]")
        self.alpha = float(alpha)
        self._roles = deepcopy(role_to_source if role_to_source is not None
                               else snapshot.get("role_to_source", {}))
        if not self._roles or not isinstance(self._roles, dict):
            raise ValueError("An explicit role_to_source mapping is required")
        if len(set(self._roles.values())) != len(self._roles):
            raise ValueError("Different eye roles cannot silently share one source")
        self._sources = deepcopy(snapshot.get("sources", {}))
        if any(topic not in self._sources for topic in self._roles.values()):
            raise ValueError("role_to_source references an unknown CameraInfo source")
        try:
            self.rotation = float(snapshot["provenance"]["calibration_rotation_degrees"])
            driver_rotation = float(snapshot["driver"]["parameters"]["rotation"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("Snapshot must establish calibration and image rotation") from exc
        if (not math.isfinite(self.rotation) or
                not math.isclose(self.rotation, driver_rotation, abs_tol=1e-8)):
            raise ValueError("Snapshot image rotation differs from calibration rotation")
        self._snapshot_gdc = _flag(snapshot["driver"]["parameters"].get("gdc_enable", False),
                                   "snapshot gdc_enable")
        self._intrinsics = {}
        for topic in set(self._roles.values()):
            source = self._sources[topic]
            size = _size((source.get("width"), source.get("height")), "source resolution")
            if source.get("distortion_model") not in ("plumb_bob", "rational_polynomial"):
                raise ValueError("Only the eight-coefficient OpenCV rational model is supported")
            K = _matrix(source.get("K"), "source K")
            try:
                D = np.asarray(source["D"], dtype=np.float64).reshape(-1)
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("source D must contain eight finite rational coefficients") from exc
            if D.size != 8 or not np.isfinite(D).all():
                raise ValueError("source D must contain eight finite rational coefficients")
            self._intrinsics[topic] = (size, K, D.copy())
        self._maps = {}
        self.maps_built = 0
        self._native_remap = None
        self.remap_backend = "opencv"
        self.remap_fallback_reason = None
        if remap_backend != "opencv":
            from .native_remap import NativeRemap
            try:
                self._native_remap = NativeRemap(threads=remap_threads)
                self.remap_backend = "native_fixed_bilinear"
            except FileNotFoundError as exc:
                if remap_backend == "native":
                    raise
                self.remap_fallback_reason = str(exc)
        self.last_remap_backend = None

    @classmethod
    def from_json(cls, path, **kwargs):
        return cls(json.loads(Path(path).read_text(encoding="utf-8-sig")), **kwargs)

    def _validate_metadata(self, metadata, source_topic):
        for key in ("rotation", "rotation_degrees", "cal_rotation", "calibration_rotation_degrees"):
            if key in metadata:
                try:
                    rotation = float(metadata[key])
                except (TypeError, ValueError) as exc:
                    raise ValueError(key + " must be a numeric angle") from exc
                if not math.isfinite(rotation) or not math.isclose(rotation, self.rotation, abs_tol=1e-8):
                    raise ValueError(key + " differs from the saved calibration rotation")
        if "source_topic" in metadata and metadata["source_topic"] != source_topic:
            raise ValueError("Input source topic does not match the requested eye role")

    def _get_maps(self, source_topic, output_size):
        key = (source_topic, output_size, self.alpha)
        if key not in self._maps:
            size, K, D = self._intrinsics[source_topic]
            newK, roi = cv2.getOptimalNewCameraMatrix(K, D, size, self.alpha, output_size)
            newK = _matrix(newK, "newK")
            # Fixed-point maps reduce the per-frame remap overhead. They are
            # made once, never recomputed from a guessed field of view.
            maps = cv2.initUndistortRectifyMap(K, D, np.eye(3), newK,
                                             output_size, cv2.CV_16SC2)
            self._maps[key] = (maps, newK, tuple(int(x) for x in roi))
            self.maps_built += 1
        return self._maps[key]

    def correct(self, image, role="left", *, output_size=None,
                already_rectified=False, rectified_camera_matrix=None,
                input_metadata=None):
        """Correct one uint8 BGR/gray raw frame, returning image/newK/metadata.

        ``output_size`` is (width, height); raw input must still have exactly
        the snapshot dimensions. Set ``already_rectified=True`` and supply
        that image's actual K to bypass correction safely. A ``rectified`` or
        ``lens_correction_applied`` flag in input metadata also triggers this
        bypass. Refusing unknown K avoids pretending the raw K is still valid.
        """
        if role not in self._roles:
            raise ValueError("Unknown camera role: " + str(role))
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or
                image.ndim not in (2, 3) or (image.ndim == 3 and image.shape[2] != 3)):
            raise ValueError("image must be a uint8 gray or three-channel BGR array")
        source_topic = self._roles[role]
        size, _, _ = self._intrinsics[source_topic]
        actual_size = (image.shape[1], image.shape[0])
        if actual_size != size:
            raise ValueError(f"Input resolution {actual_size} differs from calibration {size}")
        output_size = size if output_size is None else _size(output_size, "output_size")
        input_metadata = deepcopy(input_metadata or {})
        self._validate_metadata(input_metadata, source_topic)
        bypass = _flag(already_rectified, "already_rectified") or self._snapshot_gdc
        for key in ("rectified", "lens_correction_applied", "gdc_enable"):
            if key in input_metadata:
                bypass = _flag(input_metadata[key], key) or bypass
        if bypass:
            current_K = rectified_camera_matrix
            if current_K is None:
                current_K = input_metadata.get("camera_matrix")
            if current_K is None:
                raise ValueError("Already-rectified input requires its actual rectified camera matrix")
            newK = _matrix(current_K, "rectified camera matrix")
            newK[0, :] *= output_size[0] / size[0]
            newK[1, :] *= output_size[1] / size[1]
            corrected = image if output_size == size else cv2.resize(image, output_size)
            roi = (0, 0, output_size[0], output_size[1])
            actual_backend = "upstream_bypass" if output_size == size else "opencv_resize"
        else:
            maps, newK, roi = self._get_maps(source_topic, output_size)
            if self._native_remap is not None and image.ndim == 3:
                corrected = self._native_remap(image, maps)
                actual_backend = "native_fixed_bilinear"
            else:
                corrected = cv2.remap(image, maps[0], maps[1], interpolation=cv2.INTER_LINEAR,
                                      borderMode=cv2.BORDER_CONSTANT)
                actual_backend = "opencv"
        self.last_remap_backend = actual_backend
        metadata = {
            "remap_backend": actual_backend,
            "role": role, "source_topic": source_topic,
            "pixel_space": "upstream_rectified" if bypass else "undistorted_monocular",
            "correction_applied_this_call": not bypass,
            "lens_correction_applied": True,
            "stereo_rectification_verified": False,
            "metric_depth_verified": False,
            "calibration_status": "candidate_eeprom_eye_assignment_unverified",
            "rotation_degrees": self.rotation,
            "width": output_size[0], "height": output_size[1],
            "camera_matrix": newK.reshape(-1).tolist(),
            "distortion_model": "plumb_bob", "D": [0.0] * 5,
            "R_used": np.eye(3).reshape(-1).tolist() if not bypass else None,
            "alpha": self.alpha if not bypass else None,
            "valid_roi": list(roi),
            "input_metadata": input_metadata,
            "source_metadata": {
                "camera_info": deepcopy(self._sources[source_topic]),
                "driver": deepcopy(self._snapshot.get("driver", {})),
                "provenance": deepcopy(self._snapshot.get("provenance", {})),
                "snapshot_schema_version": self._snapshot.get("schema_version"),
                "captured_client_date": self._snapshot.get("captured_client_date"),
                "role_to_source": deepcopy(self._roles),
                "role_mapping_evidence": self._snapshot.get("role_mapping_evidence", ""),
            },
        }
        return CorrectedImage(corrected, newK.copy(), metadata)
