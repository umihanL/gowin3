"""Measured fixed-camera water-aim angle grid, independent of legacy pixels.

No ballistic model or guessed pressure is used. Interpolation is confined to
complete, independently checked pan/tilt/distance cells. A calibration describes
past measurements; it never proves that the current spray physically hit.
"""
from copy import deepcopy
from itertools import product
import math

import numpy as np

from .fixed_camera_aim import _array, _identifier, _number


KEYS = ('nominal_pan_deg', 'nominal_tilt_deg', 'nozzle_distance_m')
BINDINGS = ('geometry_calibration_id', 'stereo_calibration_id', 'actuator_reference_id',
            'pump_setting_id', 'camera_nozzle_mount_id')


class FixedCameraLanding:
    def __init__(self, document):
        self.doc = deepcopy(document)
        self.ready = self.doc.get('calibrated') is True
        if self.doc.get('mode') != 'fixed_camera_measured_angular_grid':
            raise ValueError('Explicit fixed-camera measured angular grid required')
        if not self.ready:
            return
        for key in ('calibration_id', 'measurement_session_id', *BINDINGS):
            _identifier(self.doc.get(key), key)
        if (self.doc.get('angle_convention') != 'actuator_joint_delta_deg' or
                self.doc.get('distance_reference') != 'nominal_nozzle_forward_ray_m' or
                self.doc.get('measurement_unit') != 'm'):
            raise ValueError('Explicit actuator angle and measured metre conventions required')
        self.acceptance_radius = _number(self.doc.get('acceptance_radius_m'), 'acceptance_radius_m')
        self.angle_tolerance = _number(self.doc.get('water_impact_angle_tolerance_deg'), 'water angle tolerance')
        if not .0001 <= self.acceptance_radius <= .1:
            raise ValueError('Measured acceptance radius outside supported range')
        if not .001 <= self.angle_tolerance <= .3:
            raise ValueError('Measured water-angle tolerance must be within .001..0.3 degrees')
        rows = self.doc.get('measurements')
        if not isinstance(rows, list) or not 8 <= len(rows) <= 4096:
            raise ValueError('Complete 3D grid with at least eight measured corners required')
        self.values, self.ids = {}, set()
        for row in rows:
            key = self._coordinate(row)
            sample_id = _identifier(row.get('measurement_id'), 'measurement_id')
            delta = _array(row.get('angle_correction_deg'), (2,), 'measured angular correction')
            if key in self.values or sample_id in self.ids or np.max(np.abs(delta)) > 20:
                raise ValueError('Duplicate measurements or excessive angular correction')
            self._impact(row)
            self.ids.add(sample_id)
            self.values[key] = delta
        self.axes = [sorted({key[i] for key in self.values}) for i in range(3)]
        if any(len(axis) < 2 for axis in self.axes) or len(self.values) != math.prod(map(len, self.axes)):
            raise ValueError('Measured angular grid must be complete in all three dimensions')
        validation = self.doc.get('independent_validation')
        if not isinstance(validation, list) or not validation:
            raise ValueError('Independent measured impact validation required in every interpolation cell')
        verified_cells = set()
        for row in validation:
            point = self._coordinate(row)
            sample_id = _identifier(row.get('measurement_id'), 'independent measurement_id')
            if sample_id in self.ids:
                raise ValueError('Validation measurements must be independent')
            self.ids.add(sample_id)
            self._impact(row)
            cell, weights = self._cell(point)
            if any(not 0 < weight < 1 for weight in weights):
                raise ValueError('Validation must be strictly inside its interpolation cell')
            correction = self._interpolate(cell, weights)
            commanded = _array(row.get('commanded_angles_deg'), (2,), 'validation commanded angles')
            expected = np.asarray(point[:2])+correction
            if not np.allclose(commanded, expected, atol=.001, rtol=0):
                raise ValueError('Validation did not exercise the interpolated angle command')
            verified_cells.add(cell)
        all_cells = set(product(*(range(len(axis)-1) for axis in self.axes)))
        if verified_cells != all_cells:
            raise ValueError('Every interpolation cell requires an independent measured validation')

    def _coordinate(self, row):
        if not isinstance(row, dict):
            raise ValueError('Measurement row must be an object')
        values = tuple(_number(row.get(key), key) for key in KEYS)
        if not all(-180 <= v <= 180 for v in values[:2]) or not .05 <= values[2] <= 10:
            raise ValueError('Measured grid coordinate outside supported units/range')
        return values

    def _impact(self, row):
        if row.get('measured') is not True:
            raise ValueError('Actual water measurements are required')
        error = _array(row.get('measured_impact_error_m'), (3,), 'measured impact error')
        angle_error = _array(row.get('measured_encoder_error_deg'), (2,), 'measured encoder error')
        if np.max(np.abs(angle_error)) > self.angle_tolerance:
            raise ValueError('Measured motor error exceeds validated angular tolerance')
        if np.linalg.norm(error) > self.acceptance_radius:
            raise ValueError('Measured water impact exceeds declared acceptance radius')

    def _cell(self, point):
        indices, weights = [], []
        for axis, value in zip(self.axes, point):
            if not axis[0] <= value <= axis[-1]:
                raise ValueError('fixed_camera_landing_outside_measured_grid')
            index = min(len(axis)-2, max(0, int(np.searchsorted(axis, value, side='right'))-1))
            indices.append(index)
            weights.append((value-axis[index])/(axis[index+1]-axis[index]))
        return tuple(indices), weights

    def _interpolate(self, cell, weights):
        value = np.zeros(2)
        for offsets in product((0, 1), repeat=3):
            key = tuple(self.axes[i][cell[i]+offsets[i]] for i in range(3))
            weight = math.prod(weights[i] if offsets[i] else 1-weights[i] for i in range(3))
            value += weight*self.values[key]
        return value

    def correction(self, nominal_angles, nozzle_distance, geometry, encoder):
        if not self.ready:
            raise ValueError('fixed_camera_water_impact_unmeasured')
        expected = dict(geometry_calibration_id=geometry['calibration_id'],
                        stereo_calibration_id=geometry['stereo_calibration_id'],
                        actuator_reference_id=geometry['actuator_reference_id'])
        if any(self.doc[key] != value for key, value in expected.items()):
            raise ValueError('fixed_camera_landing_geometry_reference_mismatch')
        runtime = dict(geometry_calibration_id=encoder.get('geometry_calibration_id'),
                       actuator_reference_id=encoder.get('reference_id'),
                       pump_setting_id=encoder.get('pump_setting_id'),
                       camera_nozzle_mount_id=encoder.get('camera_nozzle_mount_id'))
        if (any(self.doc[key] != value for key, value in runtime.items()) or
                encoder.get('fixed_camera_landing_calibration_id') != self.doc['calibration_id']):
            raise ValueError('fixed_camera_landing_active_profile_mismatch')
        angles = _array(nominal_angles, (2,), 'nominal angles')
        point = self._coordinate(dict(zip(KEYS, [*angles.tolist(), nozzle_distance])))
        cell, weights = self._cell(point)
        delta = self._interpolate(cell, weights)
        return dict(landing_calibration_id=self.doc['calibration_id'],
                    **{key:self.doc[key] for key in BINDINGS},
                    angle_correction_deg=delta.tolist(),
                    corrected_target_deg=(angles+delta).tolist(),
                    water_impact_calibration_verified=True,
                    water_impact_angle_tolerance_deg=self.angle_tolerance,
                    landing_compensated=True,
                    measured_acceptance_radius_m=self.acceptance_radius,
                    interpolation_cell=list(cell),
                    physical_hit_verified=False)
