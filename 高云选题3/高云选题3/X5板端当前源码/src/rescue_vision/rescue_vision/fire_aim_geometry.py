"""Strict frame/range/landing contracts for fire proposals; no hardware I/O."""
import hashlib
import json
import math
import numpy as np
from .aim_geometry import RectifiedCameraCalibration
from .legacy_aiming import CameraCalibration, bbox_iou
from .legacy_landing_calibration import LandingPointTable

CORRECTED_FRAME = 'undistorted_left_optical'


def _matrix(value):
    matrix = np.asarray(value, dtype=float).reshape(3, 3)
    if (not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], (0, 0, 1), rtol=0, atol=1e-9)
            or not np.allclose((matrix[0, 1], matrix[1, 0]), 0, rtol=0, atol=1e-9)):
        raise ValueError('Camera matrix is not a finite pinhole matrix')
    return matrix


def corrected_camera(info, frame_id, size):
    """Latched CameraInfo is geometry, not a depth/extrinsics measurement."""
    if frame_id != CORRECTED_FRAME or info is None:
        raise ValueError('Corrected-image CameraInfo is required; raw intrinsics cannot be substituted')
    if info.header.frame_id != frame_id or (info.width, info.height) != tuple(size):
        raise ValueError('CameraInfo frame or resolution does not match detection coordinates')
    if not all(isinstance(v, (int, np.integer)) and not isinstance(v, bool) and v > 0 for v in size):
        raise ValueError('Invalid image size')
    matrix = _matrix(info.k)
    distortion = np.asarray(info.d, dtype=float)
    rotation = np.asarray(info.r, dtype=float).reshape(3, 3)
    projection = np.asarray(info.p, dtype=float).reshape(3, 4)
    if (not np.isfinite(distortion).all() or not np.allclose(distortion, 0, rtol=0, atol=1e-12)
            or distortion.size not in (0, 4, 5, 8, 12, 14)
            or not np.isfinite(rotation).all() or not np.allclose(rotation, np.eye(3), rtol=0, atol=1e-9)
            or not np.isfinite(projection).all()
            or not np.allclose(projection[:, :3], matrix, rtol=0, atol=1e-8)
            or not np.allclose(projection[:, 3], 0, rtol=0, atol=1e-9)):
        raise ValueError('Corrected CameraInfo must describe zero distortion and P=[K|0], R=I')
    if getattr(info, 'binning_x', 0) not in (0, 1) or getattr(info, 'binning_y', 0) not in (0, 1):
        raise ValueError('CameraInfo binning is not supported for full-frame aiming')
    roi = getattr(info, 'roi', None)
    if roi is not None and (roi.x_offset or roi.y_offset or roi.width not in (0, size[0])
                            or roi.height not in (0, size[1]) or roi.do_rectify):
        raise ValueError('CameraInfo ROI changes the detection coordinate frame')
    geometry = dict(coordinate_frame=frame_id, image_size_wh=list(size), camera_matrix=matrix.tolist())
    digest = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest()[:16]
    return CameraCalibration(dict(camera_calibrated=True, calibration_id='camera_info_' + digest,
        eye='left', distortion_coefficients=[0.] * 5, **geometry))


def select_camera(frame_id, size, rectified, info, stereo):
    if frame_id == CORRECTED_FRAME:
        if rectified:
            raise ValueError('Monocular corrected frame cannot claim stereo rectification')
        return corrected_camera(info, frame_id, size)
    expected = 'rectified_left_optical' if rectified else 'left_camera_optical'
    if frame_id != expected:
        raise ValueError('Unsupported physical-left coordinate frame')
    if not stereo.calibrated:
        return CameraCalibration({'camera_calibrated': False, 'coordinate_frame': frame_id})
    if tuple(size) != tuple(stereo.image_size):
        raise ValueError('Stereo calibration resolution mismatch')
    document = dict(camera_calibrated=True, calibration_id=stereo.calibration_id,
                    image_size_wh=list(size), eye='left', coordinate_frame=frame_id,
                    camera_matrix=(stereo.P1[:3, :3] if rectified else stereo.K1).tolist(),
                    distortion_coefficients=([0.] * 5 if rectified else stereo.D1.tolist()))
    return RectifiedCameraCalibration(document, stereo.R1) if rectified else CameraCalibration(document)


def load_landing(document):
    """Unfilled templates remain explicitly disabled; malformed calibrations fail."""
    if document.get('landing_calibrated') is not True:
        return None
    return LandingPointTable(document)


def matched_detection(eligible, box):
    """Only associate depth with the current tracked fire, never another flame."""
    if box is None:
        return eligible[0] if len(eligible) == 1 else None
    matches = [item for item in eligible if bbox_iou(item.bbox_xyxy, box) > .5]
    return matches[0] if len(matches) == 1 else None


def target_range(detection, *, fixed_range_m=0., fixed_verified=False):
    if detection is None:
        return None, 'unavailable'
    if detection.depth_valid:
        xyz = np.array([detection.position_m.x, detection.position_m.y, detection.position_m.z], dtype=float)
        if np.isfinite(xyz).all() and xyz[2] > 0:
            value = float(np.linalg.norm(xyz))
            if math.isfinite(value) and value > 0:
                return value, 'matched_stereo_euclidean'
        return None, 'invalid_depth'
    if fixed_verified is True and math.isfinite(fixed_range_m) and fixed_range_m > 0:
        return float(fixed_range_m), 'verified_fixed_observation_assumption'
    return None, 'unavailable'


def landing_reference(table, calibration, frame_id, size, distance_m):
    """Return only in-range measured pixels in exactly the detector's geometry."""
    if table is None or distance_m is None or not calibration.ready:
        raise ValueError('Measured landing calibration, camera geometry, and target range are required')
    document = table.doc
    if (document.get('eye') != 'left' or document.get('physical_eye', 'left') != 'left'
            or document.get('coordinate_frame') != frame_id
            or tuple(document.get('image_size_wh', ())) != tuple(size)
            or document.get('distance_reference') != 'left_camera_euclidean_m'):
        raise ValueError('Landing table coordinate frame, physical eye, or range convention mismatch')
    if not document.get('camera_nozzle_mount_id') or not document.get('pump_setting_id'):
        raise ValueError('Landing measurements must identify the rigid mount and pump setting')
    matrix = _matrix(document.get('camera_matrix'))
    if not np.allclose(matrix, calibration.K, rtol=0, atol=1e-5):
        raise ValueError('Landing camera matrix differs from current image geometry')
    # Identical K and dimensions do not identify raw lens geometry. A changed
    # distortion model changes measured landing pixels even with the same K.
    distortion = document.get('distortion_coefficients')
    if distortion is None and frame_id != 'left_camera_optical':
        distortion = [0.] * len(calibration.dist)
    distortion = np.asarray(distortion, dtype=float)
    if (distortion.shape != calibration.dist.shape or not np.isfinite(distortion).all() or
            not np.allclose(distortion, calibration.dist, rtol=0, atol=1e-10)):
        raise ValueError('Landing lens distortion differs from current image geometry')
    point = table.aim_pixel(distance_m, size)
    return point, str(document['calibration_id'])


def mission_fresh(mission, received_s, now_s, timeout_s=.5):
    return (received_s is not None and math.isfinite(now_s) and 0 <= now_s - received_s <= timeout_s
            and mission.get('mode') == 'fire' and mission.get('traffic_active', False) is False
            and mission.get('gimbal_ready') is True)


def pump_candidate(aim, *, valid, compensated, range_m, calibration_id):
    return bool(valid and aim.get('image_locked') is True and aim.get('state') == 'IMAGE_LOCKED'
                and compensated and range_m is not None and math.isfinite(range_m) and range_m > 0
                and calibration_id)
