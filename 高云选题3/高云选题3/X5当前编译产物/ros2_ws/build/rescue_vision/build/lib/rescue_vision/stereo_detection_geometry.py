"""Sample calibrated stereo depth in the unchanged monocular YOLO image.

The preview is undistorted with R=I; stereo matching uses a separate R1 and P1.
These pixel coordinates are not interchangeable. The caller must validate the
current corrected CameraInfo before passing its K here. No raw-K fallback is
provided, and this module neither certifies synchronization nor operates I/O.
"""
import numpy as np

from .stereo import CalibrationError


CORRECTED_FRAME = 'undistorted_left_optical'
RECTIFIED_FRAME = 'rectified_left_optical'


def _pinhole(value, name):
    matrix = np.asarray(value, dtype=np.float64)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], (0, 0, 1), rtol=0, atol=1e-9)
            or not np.allclose((matrix[0, 1], matrix[1, 0]), 0, rtol=0, atol=1e-9)):
        raise ValueError(f'{name} must be a finite pinhole matrix')
    return matrix


class CorrectedDetectionGeometry:
    """Cache the exact projective overlap between the two left-eye image grids.

    ``sample_bbox`` returns XYZ in physical-left optical axes (right/down/forward)
    and Euclidean range in metres. Its XY lies on the current detection centre
    ray at the robust left-camera Z of the visible ROI. This assumes the printed
    flame is on a locally nearly frontoparallel facade. It is not segmentation,
    a measured flame root, a sloped-plane fit, or evidence of water impact.

    One instance corresponds to one validated CameraInfo/stereo calibration;
    recreate it if either geometry changes. Same-frame stereo pairing and an
    explicitly verified exposure synchronization remain the caller's contract.
    """

    def __init__(self, stereo, corrected_K, image_size, frame_id=CORRECTED_FRAME):
        if not stereo.calibrated:
            raise CalibrationError('Measured stereo calibration is required')
        if frame_id != CORRECTED_FRAME:
            raise ValueError('Expected physical-left monocular corrected frame')
        if (len(image_size) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v <= 0
                       for v in image_size)
                or tuple(image_size) != tuple(stereo.image_size)):
            raise ValueError('Corrected image size must match the measured stereo calibration')
        self.size = tuple(int(v) for v in image_size)
        self.K = _pinhole(corrected_K, 'Corrected CameraInfo K').copy()
        self.K_inv = np.linalg.inv(self.K)
        self.R1 = np.asarray(stereo.R1, dtype=np.float64).copy()
        if (self.R1.shape != (3, 3) or not np.isfinite(self.R1).all()
                or not np.allclose(self.R1.T @ self.R1, np.eye(3), rtol=0, atol=1e-5)
                or abs(np.linalg.det(self.R1) - 1) > 1e-5):
            raise ValueError('Stereo R1 must be a proper rotation')
        projection = np.asarray(stereo.P1, dtype=np.float64)
        if (projection.shape != (3, 4) or not np.isfinite(projection).all()
                or not np.allclose(projection[:, 3], 0, rtol=0, atol=1e-9)):
            raise ValueError('Physical-left rectified P1 must have zero translation')
        self.rectified_K = _pinhole(projection[:, :3], 'Rectified P1')
        self.config = dict(stereo.config)
        # Older geometry-only providers omit this field and use + disparity.
        self.disparity_sign = getattr(stereo, 'disparity_sign', 1)
        if (isinstance(self.disparity_sign, (bool, np.bool_)) or
                self.disparity_sign not in (-1, 1)):
            raise ValueError('Stereo disparity sign must be -1 or +1')
        self.calibration_id = stereo.calibration_id
        self.frame_id = frame_id
        self._to_rectified = self.rectified_K @ self.R1 @ self.K_inv
        self._rectified_to_corrected = self.K @ self.R1.T @ np.linalg.inv(self.rectified_K)
        self._full_projection = None

    def _project_region(self, bounds):
        x0, y0, x1, y1 = bounds
        xx = np.arange(x0, x1, dtype=np.float64)[None, :]
        yy = np.arange(y0, y1, dtype=np.float64)[:, None]
        matrix = self._rectified_to_corrected
        denominator = matrix[2, 0]*xx + matrix[2, 1]*yy + matrix[2, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = (matrix[0, 0]*xx + matrix[0, 1]*yy + matrix[0, 2])/denominator
            v = (matrix[1, 0]*xx + matrix[1, 1]*yy + matrix[1, 2])/denominator
        return u, v, (denominator > 1e-9) & np.isfinite(u) & np.isfinite(v)

    def _full_maps(self):
        # Compatibility for callers inspecting the full projection; normal
        # target sampling never allocates or scans these full-frame arrays.
        if self._full_projection is None:
            self._full_projection = self._project_region((0, 0, *self.size))
        return self._full_projection

    @property
    def _u(self):
        return self._full_maps()[0]

    @property
    def _v(self):
        return self._full_maps()[1]

    @property
    def _front(self):
        return self._full_maps()[2]

    def _roi_projection(self, box):
        x1, y1, x2, y2 = box
        inset = self.config['roi_inset_fraction']
        dx, dy = (x2-x1)*inset, (y2-y1)*inset
        inner = (x1+dx, y1+dy, x2-dx, y2-dy)
        ix1, iy1, ix2, iy2 = inner
        corners = np.array(((ix1, iy1, 1), (ix2, iy1, 1),
                            (ix2, iy2, 1), (ix1, iy2, 1))) @ self._to_rectified.T
        if np.any(corners[:, 2] <= 1e-9):
            raise ValueError('roi_outside_rectified_view')
        corners = corners[:, :2]/corners[:, 2:3]
        w, h = self.size
        if (not np.isfinite(corners).all() or np.any(corners < 0)
                or np.any(corners[:, 0] > w-1) or np.any(corners[:, 1] > h-1)):
            raise ValueError('roi_outside_rectified_view')
        low = np.floor(corners.min(axis=0)).astype(int)
        high = np.ceil(corners.max(axis=0)).astype(int)+1
        return (int(low[0]), int(low[1]), min(w, int(high[0])), min(h, int(high[1]))), inner

    def sampling_bounds(self, bbox):
        """Full rectified pixel bounds of the complete inset corrected ROI."""
        box = np.asarray(bbox, dtype=np.float64)
        if (box.shape != (4,) or not np.isfinite(box).all() or
                not (box[0] < box[2] and box[1] < box[3])):
            raise ValueError('invalid_bbox')
        if box[0] < 0 or box[1] < 0 or box[2] > self.size[0] or box[3] > self.size[1]:
            raise ValueError('clipped_bbox')
        return self._roi_projection(box)[0]

    def sample_bbox(self, result, bbox, target_pixel=None):
        """Sample only rectified pixels projecting inside the corrected ROI.

        Quality thresholds are identical to StereoDepth.sample_bbox: inset,
        sample count, valid ratio, median absolute depth deviation and robust
        inlier rejection. Negative/nonfinite/behind-camera points cannot pass.
        ``target_pixel`` optionally selects a point within the same bbox; the
        default is its centre. Neither missing nor invalid depth is invented.
        """
        answer = dict(valid=False, xyz_m=None, depth_m=None, range_m=None,
                      frame_id=self.frame_id, valid_ratio=0., samples=0,
                      reason='invalid_bbox', stereo_calibration_id=self.calibration_id,
                      target_method='bbox centre ray at robust facade Z',
                      surface_assumption='locally nearly frontoparallel printed facade')
        try:
            box = np.asarray(bbox, dtype=np.float64)
        except (ValueError, TypeError):
            return answer
        if box.shape != (4,) or not np.isfinite(box).all():
            return answer
        x1, y1, x2, y2 = box
        w, h = self.size
        if not (x1 < x2 and y1 < y2):
            return answer
        if x1 < 0 or y1 < 0 or x2 > w or y2 > h:
            answer['reason'] = 'clipped_bbox'
            return answer
        try:
            target = np.asarray(target_pixel if target_pixel is not None
                                else ((x1 + x2) / 2, (y1 + y2) / 2), dtype=np.float64)
        except (ValueError, TypeError):
            answer['reason'] = 'invalid_target_pixel'
            return answer
        if (target.shape != (2,) or not np.isfinite(target).all()
                or not (x1 <= target[0] < x2 and y1 <= target[1] < y2)):
            answer['reason'] = 'invalid_target_pixel'
            return answer
        answer['target_pixel'] = target.astype(float).tolist()
        if target_pixel is not None:
            answer['target_method'] = 'selected bbox ray at robust facade Z'
        if not self._valid_result(result):
            answer['reason'] = 'stereo_result_geometry_mismatch'
            return answer
        try:
            bounds, (ix1, iy1, ix2, iy2) = self._roi_projection(box)
        except ValueError as exc:
            answer['reason'] = str(exc)
            return answer
        bx0, by0, bx1, by1 = bounds
        ox, oy = getattr(result, 'origin_xy', (0, 0))
        rh, rw = result.valid_mask.shape
        if not (ox <= bx0 < bx1 <= ox+rw and oy <= by0 < by1 <= oy+rh):
            answer['reason'] = 'roi_outside_computed_region'
            return answer
        # Count every projected inset pixel, including invalid disparity. A
        # crop must contain this whole rectangle, never shrink the denominator.
        u, v, front = self._project_region(bounds)
        region = (front & (u >= ix1) & (u < ix2) & (v >= iy1) & (v < iy2))
        area = int(np.count_nonzero(region))
        if area == 0:
            answer['reason'] = 'insufficient_reliable_depth'
            return answer
        ys, xs = slice(by0-oy, by1-oy), slice(bx0-ox, bx1-ox)
        disparity = result.disparity_px[ys, xs]
        mask = (region & result.valid_mask[ys, xs] & np.isfinite(disparity)
                & (self.disparity_sign * disparity > 0))
        # Row-vector equivalent of physical_xyz = R1.T @ rectified_xyz.
        rectified = np.asarray(result.xyz_m[ys, xs][mask], dtype=np.float64)
        points = np.column_stack([rectified[:, 0]*self.R1[0, j]+rectified[:, 1]*self.R1[1, j]
                                  + rectified[:, 2]*self.R1[2, j] for j in range(3)])
        reliable = (np.isfinite(points).all(axis=1)
                    & (points[:, 2] >= self.config['min_depth_m'])
                    & (points[:, 2] <= self.config['max_depth_m']))
        points = points[reliable]
        # Recheck the real XYZ projection, not only its rectified array index.
        # Inconsistent XYZ/frame data cannot contaminate another target's ROI.
        pixel_x = self.K[0, 0]*points[:, 0]/points[:, 2]+self.K[0, 2]
        pixel_y = self.K[1, 1]*points[:, 1]/points[:, 2]+self.K[1, 2]
        inside = ((pixel_x >= ix1) & (pixel_x < ix2) & (pixel_y >= iy1) & (pixel_y < iy2))
        points = points[inside]
        answer['samples'] = len(points)
        answer['valid_ratio'] = float(len(points) / area)
        if (len(points) < self.config['roi_min_samples']
                or answer['valid_ratio'] < self.config['roi_min_valid_ratio']):
            answer['reason'] = 'insufficient_reliable_depth'
            return answer
        depth = points[:, 2]
        median = float(np.median(depth))
        mad = float(np.median(np.abs(depth - median)))
        answer['depth_mad_m'] = mad
        if mad / median > self.config['roi_max_relative_mad']:
            answer['reason'] = 'mixed_or_unstable_depth'
            return answer
        inliers = np.abs(depth - median) <= max(3.0 * 1.4826 * mad, .02 * median)
        if np.count_nonzero(inliers) < self.config['roi_min_samples']:
            answer['reason'] = 'insufficient_inlier_depth'
            return answer
        z = float(np.median(depth[inliers]))
        ray = self.K_inv @ np.array((target[0], target[1], 1.))
        point = ray * (z / ray[2])
        if not np.isfinite(point).all() or point[2] <= 0:
            answer['reason'] = 'invalid_target_ray'
            return answer
        answer.update(valid=True, xyz_m=point.tolist(), depth_m=z,
                      range_m=float(np.linalg.norm(point)),
                      samples=int(np.count_nonzero(inliers)), reason='ok')
        return answer

    def _valid_result(self, result):
        if result is None or not isinstance(getattr(result, 'valid_mask', None), np.ndarray):
            return False
        shape = result.valid_mask.shape
        origin = getattr(result, 'origin_xy', (0, 0))
        full_size = getattr(result, 'full_image_size', None)
        if (len(shape) != 2 or not isinstance(origin, (tuple, list)) or len(origin) != 2
                or any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, np.integer)) for v in origin)
                or not (0 <= origin[0] < origin[0]+shape[1] <= self.size[0]
                        and 0 <= origin[1] < origin[1]+shape[0] <= self.size[1])):
            return False
        if full_size is None:
            if tuple(origin) != (0, 0) or shape != self.size[::-1]:
                return False
        elif (not isinstance(full_size, (tuple, list)) or len(full_size) != 2
              or any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, np.integer)) for v in full_size)
              or tuple(full_size) != self.size):
            return False
        return bool(result is not None and getattr(result, 'frame_id', '') == RECTIFIED_FRAME
                    and isinstance(getattr(result, 'valid_mask', None), np.ndarray)
                    and result.valid_mask.shape == shape and result.valid_mask.dtype == bool
                    and isinstance(getattr(result, 'xyz_m', None), np.ndarray)
                    and result.xyz_m.shape == shape + (3,)
                    and np.issubdtype(result.xyz_m.dtype, np.floating)
                    and isinstance(getattr(result, 'disparity_px', None), np.ndarray)
                    and result.disparity_px.shape == shape
                    and np.issubdtype(result.disparity_px.dtype, np.floating))
