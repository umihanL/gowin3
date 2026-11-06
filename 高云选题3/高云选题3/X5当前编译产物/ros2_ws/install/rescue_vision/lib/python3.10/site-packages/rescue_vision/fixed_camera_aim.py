"""Fixed stereo camera -> offset pan/tilt nozzle geometry; no hardware I/O.

Base axes are x forward, y left, z up. Pan is +z; positive tilt raises
the nozzle about -y. The tilt pivot and nozzle exit need not intersect.
This models a straight nozzle ray, NOT gravity, pressure or water impact.
"""
import math
from itertools import product
from copy import deepcopy

import numpy as np


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(name + ' must be finite')
    return float(value)


def _array(value, shape, name):
    arr = np.asarray(value, dtype=float)
    if arr.shape != shape or not np.isfinite(arr).all():
        raise ValueError(name + ' has invalid shape or values')
    return arr


def _identifier(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ValueError(name + ' must identify an actual calibration')
    return value


def _rotations(pan, tilt):
    cp, sp, ct, st = math.cos(pan), math.sin(pan), math.cos(tilt), math.sin(tilt)
    return (np.array([[cp, -sp, 0.], [sp, cp, 0.], [0., 0., 1.]]),
            np.array([[ct, 0., -st], [0., 1., 0.], [st, 0., ct]]))


class FixedCameraAim:
    def __init__(self, document):
        self.doc = deepcopy(document)
        document = self.doc
        if document.get('mode') != 'fixed_camera_3d':
            raise ValueError('Fixed camera geometry requires explicit mode=fixed_camera_3d')
        self.water_aim_policy = document.get('water_aim_policy', 'measured_impact')
        if self.water_aim_policy not in ('measured_impact', 'straight_ray'):
            raise ValueError('Unknown fixed-camera water_aim_policy')
        self.ready = document.get('calibrated') is True
        self.reset()
        if not self.ready:
            return
        for key in ('calibration_id', 'stereo_calibration_id', 'actuator_reference_id'):
            _identifier(document.get(key), key)
        if (document.get('translation_unit') != 'm' or
                document.get('axis_model') != 'serial_pan_z_tilt_minus_y_with_offsets' or
                document.get('axis_reference_verified') is not True):
            raise ValueError('Measured metre geometry and verified serial-axis references required')
        if document.get('source_frame') not in ('undistorted_left_optical', 'rectified_left_optical', 'left_camera_optical'):
            raise ValueError('Explicit physical-left optical source frame required')
        size = document.get('image_size_wh')
        if not isinstance(size, list) or len(size) != 2 or any(type(x) is not int or x < 32 for x in size):
            raise ValueError('Measured image_size_wh required')
        self.R = _array(document.get('R_camera_to_gimbal'), (3, 3), 'R_camera_to_gimbal')
        if (not np.allclose(self.R.T @ self.R, np.eye(3), atol=1e-6, rtol=0) or
                abs(np.linalg.det(self.R)-1.) > 1e-6):
            raise ValueError('R_camera_to_gimbal must be orthonormal with determinant +1')
        self.t = _array(document.get('t_camera_origin_in_gimbal_m'), (3,), 'camera origin')
        self.tilt_offset = _array(document.get('tilt_origin_in_pan_m'), (3,), 'tilt pivot offset')
        self.nozzle_offset = _array(document.get('nozzle_origin_in_tilt_m'), (3,), 'nozzle exit offset')
        self.nozzle_direction = _array(document.get('nozzle_direction_in_tilt'), (3,), 'nozzle direction')
        if (np.linalg.norm(self.t) > 5 or max(np.linalg.norm(self.tilt_offset), np.linalg.norm(self.nozzle_offset)) > 1 or
                abs(np.linalg.norm(self.nozzle_direction)-1.) > 1e-6):
            raise ValueError('Check metre offsets and unit-length nozzle direction')
        self.zero, self.sign, limits = [], [], []
        for name in ('pan', 'tilt'):
            axis = document.get('axes', {}).get(name, {})
            zero = _number(axis.get('zero_angle_deg'), name + ' zero')
            direction = axis.get('positive_direction')
            bounds = _array(axis.get('limits_deg'), (2,), name + ' limits')
            if (type(direction) is not int or direction not in (-1, 1) or
                    not -180 <= bounds[0] <= zero <= bounds[1] <= 180 or bounds[0] >= bounds[1]):
                raise ValueError('Invalid measured axis reference/direction/limits')
            self.zero.append(zero); self.sign.append(direction); limits.append(bounds)
        self.zero, self.sign, self.limits = np.array(self.zero), np.array(self.sign), np.array(limits)
        self.range = _array(document.get('valid_range_m'), (2,), 'valid_range_m')
        if not .05 <= self.range[0] < self.range[1] <= 10:
            raise ValueError('Measured range must be between .05 and 10 metres')
        for key, lo, hi in [('max_speed_deg_s', .1, 30), ('max_step_deg', .01, 2),
                           ('angle_tolerance_deg', .01, 1), ('line_tolerance_m', .00001, .01),
                           ('min_forward_ray_m', .01, .5), ('stable_lock_s', .1, 2),
                           ('max_frame_age_s', .05, .5), ('max_encoder_age_s', .03, .4),
                           ('max_target_shift_m', .001, .1)]:
            value = _number(document.get(key), key)
            if not lo <= value <= hi:
                raise ValueError(key + ' outside supported range')
            setattr(self, key, value)
        if self.water_aim_policy == 'straight_ray' and (
                self.angle_tolerance_deg > .3 or self.line_tolerance_m > .002):
            raise ValueError('Straight-ray policy requires at most 0.3 degree and 2 mm tolerances')

    def reset(self):
        self.identity = self.capture_key = self.last_target = None
        self.last_time = self.stable_since = None

    def summary(self):
        return dict(aim_mode='fixed_camera_3d', camera_mount='fixed_to_vehicle',
                    water_aim_policy=self.water_aim_policy,
                    calibration_ready=self.ready, geometry_locked=False, image_locked=False,
                    geometry_calibration_id=self.doc.get('calibration_id'),
                    stereo_calibration_id=self.doc.get('stereo_calibration_id'),
                    actuator_reference_id=self.doc.get('actuator_reference_id'),
                    camera_nozzle_mount_id=self.doc.get('camera_nozzle_mount_id'),
                    pump_setting_id=self.doc.get('pump_setting_id'),
                    geometry_angle_tolerance_deg=self.doc.get('angle_tolerance_deg'),
                    geometry_line_tolerance_m=self.doc.get('line_tolerance_m'),
                    geometry_min_forward_ray_m=self.doc.get('min_forward_ray_m'),
                    water_impact_calibration_verified=False,
                    pump_candidate=False, pump_allowed=False, landing_compensated=False,
                    physical_hit_verified=False, range_is_assumption=False)

    def forward(self, joint_angles_deg):
        angles = _array(joint_angles_deg, (2,), 'joint angles')
        pan, tilt = np.radians((angles-self.zero)*self.sign)
        rp, rt = _rotations(pan, tilt)
        return rp @ (self.tilt_offset+rt @ self.nozzle_offset), rp @ rt @ self.nozzle_direction

    def inverse(self, target_gimbal_m, measured_deg):
        """Bounded two-angle solve with measured offsets, no endpoint guessing.

        Candidate angles must satisfy both mechanical limits and the forward
        ray. Failure yields no command, even if a clamped angle is nearby.
        """
        point = _array(target_gimbal_m, (3,), 'target point')
        measured = _array(measured_deg, (2,), 'actual joint angles')
        low, high = self.limits[:, 0], self.limits[:, 1]
        if np.any(measured < low) or np.any(measured > high):
            raise ValueError('fixed_camera_encoder_outside_limits')
        physical = np.degrees([math.atan2(point[1], point[0]), math.atan2(point[2], math.hypot(*point[:2]))])
        guess = np.clip(self.zero+physical*self.sign, low, high)
        seeds = [measured, guess, (low+high)/2,
                 np.array([low[0], low[1]]), np.array([low[0], high[1]]),
                 np.array([high[0], low[1]]), np.array([high[0], high[1]])]
        def residual(q):
            origin, direction = self.forward(q)
            return np.cross(direction, point-origin)
        accepted = []
        for seed in seeds:
            q = seed.copy()
            for _ in range(40):
                err = residual(q)
                if np.linalg.norm(err) < min(self.line_tolerance_m*.1, 1e-6):
                    break
                eps = .001
                jac = np.column_stack([(residual(q+np.eye(2)[i]*eps)-residual(q-np.eye(2)[i]*eps))/(2*eps)
                                       for i in range(2)])
                step = np.clip(np.linalg.lstsq(jac, -err, rcond=None)[0], -15., 15.)
                improved = False
                for scale in (1., .5, .25, .1):
                    candidate = np.clip(q+step*scale, low, high)
                    if np.linalg.norm(residual(candidate)) < np.linalg.norm(err)-1e-12:
                        q = candidate; improved = True; break
                if not improved:
                    break
            origin, direction = self.forward(q)
            along = float(np.dot(point-origin, direction))
            miss = float(np.linalg.norm(residual(q)))
            if miss <= self.line_tolerance_m and along >= self.min_forward_ray_m:
                accepted.append((float(np.linalg.norm(q-measured)), q, origin, direction, along, miss))
        if not accepted:
            raise ValueError('fixed_camera_target_unreachable')
        _, q, origin, direction, along, miss = min(accepted, key=lambda x:x[0])
        return dict(geometry_target_deg=q.tolist(), nozzle_origin_gimbal_m=origin.tolist(),
                    nozzle_direction_gimbal=direction.tolist(), nozzle_target_distance_m=along,
                    geometric_ray_error_m=miss)

    def ray_error(self, point, angles):
        """Measured/attainable nozzle ray error, not the inverse solver residual."""
        origin, direction = self.forward(angles)
        delta = np.asarray(point)-origin
        return float(np.linalg.norm(np.cross(direction, delta))), float(np.dot(direction, delta))

    def attainable_target(self, point, goal, encoder, angle_tolerance=None):
        """Check explicit actuator lattice without relaxing aiming accuracy.

        Neighbouring lattice points are evaluated against both original angle
        and ray tolerances. This is a resolution preflight, not proof that a
        motor has reached a point or that water will land there.
        """
        steps = encoder.get('axis_step_deg')
        if steps is None:
            return dict(actuator_resolution_checked=False, attainable_target_deg=goal.tolist())
        bounds = _array(encoder.get('axis_reachable_limits_deg'), (2, 2), 'reachable axis limits')
        if (not isinstance(steps, (list, tuple)) or len(steps) != 2 or
                np.any(bounds[:, 0] < self.limits[:, 0]-1e-8) or
                np.any(bounds[:, 1] > self.limits[:, 1]+1e-8) or
                np.any(bounds[:, 0] > bounds[:, 1])):
            raise ValueError('fixed_camera_invalid_actuator_resolution')
        candidates = []
        quantized = False
        for i, step in enumerate(steps):
            if step is None:
                candidates.append([float(goal[i])])
                continue
            step = _number(step, 'axis step')
            span = (bounds[i, 1]-bounds[i, 0])/step if step > 0 else -1
            if not 0 < step <= 10 or abs(span-round(span)) > 1e-6:
                raise ValueError('fixed_camera_invalid_actuator_resolution')
            quantized = True
            index = (goal[i]-bounds[i, 0])/step
            candidates.append(sorted({float(bounds[i, 0]+min(round(span), max(0, k))*step)
                                      for k in (math.floor(index), math.ceil(index))}))
        evaluated = []
        for candidate in product(*candidates):
            q = np.asarray(candidate)
            miss, along = self.ray_error(point, q)
            angular = float(np.max(np.abs(q-goal)))
            accepted = (angular <= (self.angle_tolerance_deg if angle_tolerance is None else angle_tolerance)+1e-8 and
                        miss <= self.line_tolerance_m and along >= self.min_forward_ray_m)
            evaluated.append((not accepted, miss, angular, q, along))
        failed, miss, angular, q, along = min(evaluated, key=lambda row: row[:3])
        return dict(actuator_resolution_checked=quantized, attainable_target_deg=q.tolist(),
                    attainable_ray_error_m=miss, attainable_angle_error_deg=angular,
                    attainable_precision_met=not failed, attainable_forward_ray_m=along)

    def propose(self, *, target, capture_s, sequence, frame_id, image_size, rectified,
                stereo_calibration_id, active_stereo_calibration_id, identity, encoder, now_s, landing=None):
        result = dict(self.summary(), valid=False, target_range_m=None, range_source='unavailable',
                      geometry_target_deg=None, target_position_camera_m=None, target_position_gimbal_m=None,
                      servo_plan=dict(command=None, hardware_written=False, reference_source='measured_encoder_fixed_camera'))
        def reject(reason):
            self.reset(); result['reason'] = reason; result['servo_plan']['reason'] = reason
            return result
        if not self.ready:
            return reject('fixed_camera_uncalibrated')
        if (not isinstance(identity, (tuple, list)) or len(identity) != 3 or
                not isinstance(identity[0], str) or not identity[0] or
                any(type(v) is not int or v <= 0 for v in identity[1:])):
            return reject('fixed_camera_selection_required')
        if (not isinstance(target, dict) or target.get('class_id') != 0 or
                target.get('depth_valid') is not True or target.get('depth_reason') != 'verified_stereo_wallpaper_surface'):
            return reject('fixed_camera_stereo_depth_required')
        if (stereo_calibration_id != self.doc['stereo_calibration_id'] or
                active_stereo_calibration_id != stereo_calibration_id):
            return reject('fixed_camera_stereo_calibration_mismatch')
        if (frame_id != self.doc['source_frame'] or list(image_size) != self.doc['image_size_wh'] or
                rectified is not (frame_id == 'rectified_left_optical')):
            return reject('fixed_camera_frame_mismatch')
        try:
            now_s, capture_s = _number(now_s, 'now'), _number(capture_s, 'capture')
            if not 0 <= now_s-capture_s <= self.max_frame_age_s or type(sequence) is not int or sequence < 0:
                return reject('fixed_camera_depth_stale')
            xyz = _array(target.get('position_m'), (3,), 'depth point')
            depth = _number(target.get('depth_m'), 'depth')
            confidence = _number(target.get('confidence'), 'confidence')
            distance = float(np.linalg.norm(xyz))
            if (not .4 <= confidence <= 1 or xyz[2] <= 0 or depth <= 0 or
                    abs(depth-xyz[2]) > max(.001, .01*depth) or not self.range[0] <= distance <= self.range[1]):
                return reject('fixed_camera_stereo_depth_required')
            if not isinstance(encoder, dict):
                return reject('fixed_camera_encoder_feedback_required')
            stamp = _number(encoder.get('stamp'), 'encoder stamp')
            delay = _number(encoder.get('feedback_age_s'), 'encoder age')
            if (encoder.get('dry_run') is not False or encoder.get('enabled') is not True or
                    encoder.get('feedback_fresh') is not True or encoder.get('status_fresh') is not True or
                    encoder.get('error') or encoder.get('estop') or
                    not 0 <= delay <= self.max_encoder_age_s or not 0 <= now_s-stamp <= self.max_encoder_age_s or
                    not 0 <= now_s-(stamp-delay) <= self.max_encoder_age_s):
                return reject('fixed_camera_encoder_feedback_required')
            if (encoder.get('reference_confirmed_this_start') is not True or
                    encoder.get('reference_id') != self.doc['actuator_reference_id']):
                return reject('fixed_camera_reference_mismatch')
            actual = _array(encoder.get('actual_angles_deg'), (2,), 'measured joints')
            if not np.array_equal(_array(encoder.get('axis_limits_deg'), (2, 2), 'actuator limits'), self.limits):
                return reject('fixed_camera_reference_mismatch')
            point = self.R @ xyz + self.t
            geometry = self.inverse(point, actual)
            nominal = np.asarray(geometry['geometry_target_deg'])
            goal, tracking_point = nominal, point
            angular_tolerance = self.angle_tolerance_deg
            compensation = None
            if self.water_aim_policy == 'measured_impact' and landing is not None and landing.ready:
                compensation = landing.correction(nominal, geometry['nozzle_target_distance_m'], self.doc, encoder)
                goal = np.asarray(compensation['corrected_target_deg'])
                if np.any(goal < self.limits[:,0]) or np.any(goal > self.limits[:,1]):
                    return reject('fixed_camera_compensated_target_outside_limits')
                angular_tolerance = min(angular_tolerance, compensation['water_impact_angle_tolerance_deg'])
                compensated_origin, compensated_direction = self.forward(goal)
                # The calibrated outgoing ray is intentionally offset from the
                # straight line to the fire. Check tracking of that ray, not a
                # fictitious gravity-free path through the target.
                tracking_point = compensated_origin+geometry['nozzle_target_distance_m']*compensated_direction
            resolution = self.attainable_target(tracking_point, goal, encoder, angular_tolerance)
        except (ValueError, TypeError, OverflowError, np.linalg.LinAlgError) as exc:
            return reject(str(exc) if str(exc).startswith('fixed_camera_') else 'fixed_camera_invalid_geometry_or_feedback: '+str(exc))
        result.update(**resolution)
        if resolution.get('attainable_precision_met') is False:
            result.update(**geometry, target_position_camera_m=xyz.tolist(),
                          target_position_gimbal_m=point.tolist(), target_range_m=distance)
            return reject('fixed_camera_actuator_resolution_insufficient')
        key = (capture_s, sequence)
        if self.identity == tuple(identity) and self.capture_key is not None and (
                capture_s <= self.capture_key[0] or sequence <= self.capture_key[1]):
            return reject('fixed_camera_depth_out_of_order')
        changed = (self.identity != tuple(identity) or self.last_target is None or
                   np.linalg.norm(point-self.last_target) > self.max_target_shift_m)
        dt = 0. if self.last_time is None or changed else now_s-self.last_time
        if changed:
            self.stable_since = None
        self.identity, self.capture_key, self.last_target, self.last_time = tuple(identity), key, point, now_s
        error = goal-actual
        actual_miss, actual_along = self.ray_error(point, actual)
        tracking_miss, tracking_along = self.ray_error(tracking_point, actual)
        stopped = encoder.get('motor_rpm') == [0, 0] and encoder.get('motor_enabled') == [True, True]
        if (np.max(np.abs(error)) <= angular_tolerance and stopped and
                tracking_miss <= self.line_tolerance_m and tracking_along >= self.min_forward_ray_m):
            if self.stable_since is None:
                self.stable_since = now_s
        else:
            self.stable_since = None
        locked = self.stable_since is not None and now_s-self.stable_since >= self.stable_lock_s
        result.update(valid=True, **geometry, target_position_camera_m=xyz.tolist(), target_position_gimbal_m=point.tolist(),
                      actual_ray_error_m=actual_miss, actual_forward_ray_m=actual_along,
                      actual_tracking_ray_error_m=tracking_miss, aim_target_deg=goal.tolist(),
                      ray_tracking_reference=('measured_water_compensated_nozzle_ray' if compensation else 'straight_target_ray'),
                      target_range_m=distance, range_source='matched_stereo_euclidean', geometry_locked=locked,
                      reason='fixed_camera_geometry_locked' if locked else 'fixed_camera_aligning')
        if compensation is not None:
            result.update(compensation)
            result['pump_candidate'] = bool(locked)
        elif self.water_aim_policy == 'straight_ray':
            # A geometric lock deliberately carries no measured-water claim.
            # The actuator independently checks the selected policy and active
            # geometry/mount/pump/reference identities before admitting a pulse.
            result.update(pump_candidate=bool(locked), geometric_alignment_only=True,
                          landing_calibration_id=None, landing_compensated=False,
                          water_impact_calibration_verified=False, physical_hit_verified=False)
        plan = result['servo_plan']
        plan.update(actual_angles_deg=actual.tolist(), geometric_angle_error_deg=error.tolist(),
                    encoder_stamp=stamp-delay, geometry_locked=locked)
        if not 0 < dt <= self.max_frame_age_s:
            result['reason'] = 'fixed_camera_acquiring'
        else:
            bound = min(self.max_step_deg, self.max_speed_deg_s*min(dt, .2))
            desired = np.asarray(resolution['attainable_target_deg'])
            command = actual+np.clip(desired-actual, -bound, bound)
            plan['command'] = dict(pan_target_deg=float(command[0]), tilt_target_deg=float(command[1]))
        plan['reason'] = result['reason']
        return result
