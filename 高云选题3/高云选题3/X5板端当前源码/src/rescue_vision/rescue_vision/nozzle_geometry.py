"""Rigid camera/nozzle parallax compensation; no servo or pump I/O.

The nozzle and camera rotate together. A straight nozzle ray intersects the
sphere at the measured target range from the physical-left optical centre.
That intersection projects to the pixel the visual servo should align with.
This accounts for measured nozzle translation and boresight, not water drop,
spray, pressure, servo backlash, or motion of the camera about the gimbal pivot.
Refresh range during closed-loop movement; never add this reference to a water
landing table, which already incorporates mounting and water behaviour.
"""
from copy import deepcopy
import math
import numpy as np


CORRECTED_FRAME = 'undistorted_left_optical'


def _positive_range(value):
    if (not isinstance(value, (int, float, np.integer, np.floating))
            or isinstance(value, (bool, np.bool_)) or not math.isfinite(value) or value <= 0):
        raise ValueError('Target range must be a finite positive distance in metres')
    return float(value)


def _size(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or not all(isinstance(v, (int, np.integer)) and not isinstance(v, (bool, np.bool_))
                       and v > 0 for v in value)):
        raise ValueError('Image size must contain two positive integer dimensions')
    return tuple(value)


def _matrix(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], (0, 0, 1), rtol=0, atol=1e-9)
            or not np.allclose((matrix[0, 1], matrix[1, 0]), 0, rtol=0, atol=1e-9)):
        raise ValueError('Camera matrix must describe a finite, zero-skew pinhole camera')
    return matrix


def _angles(xyz):
    x, y, z = xyz
    return dict(yaw_right_positive=math.degrees(math.atan2(x, z)),
                pitch_up_positive=math.degrees(math.atan2(-y, math.hypot(x, z))))


class GeometryNozzleReference:
    """Measured rigid geometry in physical-left x-right/y-down/z-forward axes.

    An unfilled template is allowed but never produces a reference. Calibrated
    documents require matching camera geometry and an explicit measured range
    envelope. The rotation maps nozzle coordinates to camera coordinates; the
    nozzle's positive Z axis is its outgoing straight-jet direction.
    """

    def __init__(self, document):
        if not isinstance(document, dict):
            raise ValueError('Nozzle calibration must be a document')
        self.doc = deepcopy(document)
        document = self.doc
        self.ready = document.get('nozzle_calibrated') is True
        if not self.ready:
            return
        if (document.get('camera_model') != 'pinhole'
                or document.get('physical_eye') != 'left'
                or document.get('coordinate_frame') != CORRECTED_FRAME
                or document.get('translation_unit') != 'm'
                or document.get('distance_reference') != 'left_camera_euclidean_m'
                or document.get('nozzle_axis') != '+z'):
            raise ValueError('Nozzle calibration has an unsupported camera/frame/unit/axis convention')
        for name in ('calibration_id', 'camera_nozzle_mount_id'):
            if not isinstance(document.get(name), str) or not document[name].strip():
                raise ValueError('Measured nozzle calibration requires ' + name)
        self.calibration_id = document['calibration_id']
        self.mount_id = document['camera_nozzle_mount_id']
        self.size = _size(document.get('image_size_wh'))
        self.K = _matrix(document.get('camera_matrix'))
        distortion = np.asarray(document.get('distortion_coefficients'), dtype=float)
        if (distortion.ndim != 1 or distortion.size not in (0, 4, 5, 8, 12, 14)
                or not np.isfinite(distortion).all()
                or not np.allclose(distortion, 0, rtol=0, atol=1e-12)):
            raise ValueError('Nozzle reference uses only the zero-distortion corrected image')
        self.t = np.asarray(document.get('nozzle_origin_camera_m'), dtype=float)
        self.R = np.asarray(document.get('R_camera_from_nozzle'), dtype=float)
        if self.t.shape != (3,) or not np.isfinite(self.t).all():
            raise ValueError('Nozzle origin must be a finite XYZ vector in camera metres')
        if (self.R.shape != (3, 3) or not np.isfinite(self.R).all()
                or not np.allclose(self.R.T @ self.R, np.eye(3), rtol=0, atol=1e-7)
                or not math.isclose(float(np.linalg.det(self.R)), 1., rel_tol=0, abs_tol=1e-7)):
            raise ValueError('R_camera_from_nozzle must be a proper orthonormal rotation')
        # Normalization prevents small accepted rotation roundoff changing range.
        self.axis = self.R[:, 2] / np.linalg.norm(self.R[:, 2])
        envelope = document.get('valid_range_m')
        if not isinstance(envelope, (list, tuple)) or len(envelope) != 2:
            raise ValueError('Provide measured minimum and maximum supported ranges')
        self.range_min, self.range_max = map(_positive_range, envelope)
        if self.range_min >= self.range_max:
            raise ValueError('Supported range minimum must be smaller than maximum')
        if self.range_min <= float(np.linalg.norm(self.t)):
            raise ValueError('Supported targets must lie farther from the camera than the nozzle origin')

    def reference(self, calibration, size, distance_m=None, *, frame_id=CORRECTED_FRAME,
                  target_xyz_m=None):
        """Return an absolute reference pixel, never an additive pixel offset.

        Supply a Euclidean range, a same-frame target XYZ, or both. Supplying
        both is checked for consistency. XYZ from stereo rectification must be
        rotated back to physical-left camera coordinates before this call.
        The result permits visual alignment only; it cannot authorize pumping.
        """
        if not self.ready:
            raise ValueError('Measured nozzle mounting calibration is unavailable')
        if frame_id != CORRECTED_FRAME or _size(size) != self.size:
            raise ValueError('Nozzle calibration frame or image size does not match detection geometry')
        if not getattr(calibration, 'ready', False):
            raise ValueError('Calibrated current corrected camera is required')
        camera_doc = getattr(calibration, 'doc', {})
        if (camera_doc.get('coordinate_frame') != frame_id or camera_doc.get('eye') != 'left'
                or camera_doc.get('physical_eye', 'left') != 'left'
                or camera_doc.get('camera_model', 'pinhole') != 'pinhole'
                or not camera_doc.get('calibration_id')
                or _size(getattr(calibration, 'size', None)) != self.size):
            raise ValueError('Current camera does not describe the calibrated physical-left image')
        matrix = _matrix(getattr(calibration, 'K', None))
        distortion = np.asarray(getattr(calibration, 'dist', None), dtype=float)
        if (not np.allclose(matrix, self.K, rtol=0, atol=1e-5)
                or distortion.ndim != 1 or distortion.size not in (0, 4, 5, 8, 12, 14)
                or not np.isfinite(distortion).all()
                or not np.allclose(distortion, 0, rtol=0, atol=1e-12)):
            raise ValueError('Nozzle calibration and current corrected camera intrinsics differ')
        target = None
        if target_xyz_m is not None:
            target = np.asarray(target_xyz_m, dtype=float)
            if target.shape != (3,) or not np.isfinite(target).all() or target[2] <= 0:
                raise ValueError('Target XYZ must be finite and in front of the physical-left camera')
            xyz_range = float(np.linalg.norm(target))
            if distance_m is None:
                distance_m = xyz_range
            elif not math.isclose(_positive_range(distance_m), xyz_range, rel_tol=1e-5, abs_tol=1e-6):
                raise ValueError('Target XYZ and Euclidean range disagree')
        distance = _positive_range(distance_m)
        if not self.range_min <= distance <= self.range_max:
            raise ValueError('Target is outside the measured nozzle range envelope')
        # |t + s*a|² = d²; with |a|=1 and |t|<d, exactly one root is positive.
        along = float(np.dot(self.t, self.axis))
        discriminant = along * along + distance * distance - float(np.dot(self.t, self.t))
        if not math.isfinite(discriminant) or discriminant <= 0:
            raise ValueError('No forward nozzle-ray intersection exists at this range')
        travel = -along + math.sqrt(discriminant)
        point = self.t + travel * self.axis
        if not math.isfinite(travel) or travel <= 0 or not np.isfinite(point).all() or point[2] <= 0:
            raise ValueError('Nozzle-ray intersection is behind the camera or nozzle')
        projection = self.K @ point
        pixel = projection[:2] / projection[2]
        if not np.isfinite(pixel).all() or not (0 <= pixel[0] < self.size[0] and 0 <= pixel[1] < self.size[1]):
            raise ValueError('Compensated reference falls outside the current image')
        reference_angles = _angles(point)
        result = dict(aim_pixel=pixel.tolist(), calibration_id=self.calibration_id,
            camera_nozzle_mount_id=self.mount_id, camera_calibration_id=camera_doc['calibration_id'],
            coordinate_frame=frame_id, physical_eye='left', image_size_wh=list(self.size),
            target_range_m=distance, distance_reference='left_camera_euclidean_m',
            reference_xyz_m=point.tolist(), nozzle_travel_m=travel,
            reference_angle_deg=reference_angles, nozzle_axis_camera=self.axis.tolist(),
            compensation_kind='rigid_nozzle_geometry', gravity_compensated=False,
            physical_hit_verified=False, pump_eligible=False)
        if target is not None:
            target_angles = _angles(target)
            result['target_xyz_m'] = target.tolist()
            result['error_to_reference_deg'] = {
                key: target_angles[key] - reference_angles[key] for key in reference_angles}
        return result
