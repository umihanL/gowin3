"""Import factory CameraInfo extrinsics without certifying physical depth.

The GS130W raw driver snapshot uses P = K [R | T], with each camera pose
expressed relative to the same factory reference. This is not the usual
rectified ROS CameraInfo projection convention. Reject other conventions.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


IMPORT_SCHEMA = 'gs130w_factory_stereo_import/v1'
FACTORY_SLOT_SOURCES = ('/image_left_raw', '/image_right_raw')


def _array(value, shape, name):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in 'iuf' or raw.size != math.prod(shape):
            raise ValueError('numeric array of the required size expected')
        if any(isinstance(v, (bool, np.bool_)) for v in np.asarray(value, dtype=object).flat):
            raise ValueError('booleans are not calibration numbers')
        result = np.asarray(value, dtype=np.float64).reshape(shape)
    except (TypeError, ValueError) as exc:
        raise ValueError(name + ': invalid numeric array') from exc
    if not np.all(np.isfinite(result)):
        raise ValueError(name + ': non-finite value')
    return result


def _rotation(value, name):
    result = _array(value, (3, 3), name)
    if (not np.allclose(result.T @ result, np.eye(3), atol=1e-4, rtol=0) or
            abs(float(np.linalg.det(result)) - 1.) > 1e-4):
        raise ValueError(name + ': proper orthonormal rotation required')
    return result


def _angle(value, name):
    if isinstance(value, bool):
        raise ValueError(name + ': explicit rotation in degrees required')
    try:
        angle = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(name + ': explicit rotation in degrees required') from exc
    if not math.isfinite(angle) or angle % 360 not in (0., 90., 180., 270.):
        raise ValueError(name + ': expected 0, 90, 180 or 270 degrees')
    return angle % 360


def _camera(source, name):
    if not isinstance(source, dict):
        raise ValueError(name + ': CameraInfo object required')
    size = (source.get('width'), source.get('height'))
    if any(type(v) is not int or v < 32 for v in size):
        raise ValueError(name + ': full single-eye dimensions required')
    if source.get('distortion_model') not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError(name + ': only pinhole/rational distortion is supported')
    distortion = source.get('D')
    if not isinstance(distortion, list) or len(distortion) not in (4, 5, 8, 12, 14):
        raise ValueError(name + ': unsupported pinhole distortion length')
    d = _array(distortion, (len(distortion),), name + '.D')
    k = _array(source.get('K'), (3, 3), name + '.K')
    if (min(k[0, 0], k[1, 1]) <= 0 or
            not np.allclose(k[2], [0, 0, 1], atol=1e-8, rtol=0) or
            abs(k[0, 1]) > 1e-8 or abs(k[1, 0]) > 1e-8):
        raise ValueError(name + ': invalid pinhole intrinsics')
    r = _rotation(source.get('R'), name + '.R')
    p = _array(source.get('P'), (3, 4), name + '.P')
    # In the raw factory format P's first block contains K times the camera
    # extrinsic R, not a new rectified K. The fourth column then gives K*T.
    residual = float(np.max(np.abs(p[:, :3] - k @ r)))
    if not np.allclose(p[:, :3], k @ r, atol=1e-5, rtol=1e-7):
        raise ValueError(name + ': P does not follow factory K[R|T] convention')
    t = np.linalg.solve(k, p[:, 3])
    if not np.all(np.isfinite(t)):
        raise ValueError(name + ': invalid factory translation')
    return dict(size=size, K=k, D=d, R=r, T=t, projection_residual=residual)


def import_factory_stereo(snapshot_path, *, left_source=None, right_source=None,
                          left_calibration_source=None, right_calibration_source=None,
                          translation_unit='mm', mapping_evidence=None):
    """Return factory parameters in the existing stereo JSON schema.

    left_source/right_source identify input image topics. Independent
    *_calibration_source arguments select the EEPROM parameter slots assigned
    to them. If omitted, parameter source names equal input topic names for
    backwards compatibility. Changing image-topic assignments alone does not
    invert the factory extrinsics. Exchanging parameter slots does compute the
    inverse relative transform; T is never flipped alone to satisfy a matcher.
    """
    path = Path(snapshot_path).resolve()
    raw = path.read_bytes()
    try:
        document = json.loads(raw.decode('utf-8-sig'))
    except (UnicodeError, ValueError) as exc:
        raise ValueError('Factory snapshot must be a UTF-8 JSON document') from exc
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('Expected version 1 GS130W CameraInfo snapshot')
    sources, original = document.get('sources'), document.get('provenance')
    driver = document.get('driver', {}).get('parameters') if isinstance(document.get('driver'), dict) else None
    if not all(isinstance(v, dict) for v in (sources, original, driver)):
        raise ValueError('Snapshot sources, provenance and driver parameters required')
    header = original.get('eeprom_header')
    if (not isinstance(header, dict) or header.get('flag') != 'SZYGSJKJ' or
            header.get('cal_type') != 0):
        raise ValueError('Expected GS130W SZYGSJKJ pinhole factory EEPROM provenance')
    if not original.get('source') or not original.get('driver_version'):
        raise ValueError('Factory source and driver version must be recorded')
    rotation = _angle(original.get('calibration_rotation_degrees'), 'calibration rotation')
    if any(_angle(driver.get(key), 'driver.' + key) != rotation
           for key in ('rotation', 'cal_rotation')):
        raise ValueError('Image rotation and calibration rotation differ; do not rotate K/D twice')
    if driver.get('gdc_enable') not in (False, 'False', 'false', 0, '0'):
        raise ValueError('Only raw non-GDC CameraInfo snapshots can be imported')
    if translation_unit not in ('mm', 'm'):
        raise ValueError('Factory translation unit must explicitly be mm or m')
    if (left_source is None) != (right_source is None):
        raise ValueError('Specify both source slots or neither')
    explicit_order = left_source is not None
    if not explicit_order:
        left_source, right_source = FACTORY_SLOT_SOURCES
    if (not isinstance(left_source, str) or not isinstance(right_source, str) or
            left_source == right_source or left_source not in sources or right_source not in sources):
        raise ValueError('Two distinct source topics present in the snapshot are required')
    if (left_calibration_source is None) != (right_calibration_source is None):
        raise ValueError('Specify both calibration parameter sources or neither')
    explicit_calibration_mapping = left_calibration_source is not None
    if not explicit_calibration_mapping:
        left_calibration_source, right_calibration_source = left_source, right_source
    calibration_sources = (left_calibration_source, right_calibration_source)
    if (any(not isinstance(name, str) or name not in sources for name in calibration_sources) or
            left_calibration_source == right_calibration_source):
        raise ValueError('Two distinct calibration parameter sources in the snapshot are required')
    left, right = (_camera(sources[name], name) for name in calibration_sources)
    if left['size'] != right['size']:
        raise ValueError('Both factory eyes must have the same full image dimensions')
    for topic, camera in ((left_source, left), (right_source, right)):
        if (sources[topic].get('width'), sources[topic].get('height')) != camera['size']:
            raise ValueError('Mapped image dimensions differ from the factory parameter dimensions')
    if any(len(camera['D']) != header.get('d_num') for camera in (left, right)):
        raise ValueError('EEPROM distortion count does not match the source CameraInfo')
    # X_i = R_i X_factory + t_i, hence X_right = Rrel X_left + trel.
    relative_r = right['R'] @ left['R'].T
    relative_t = right['T'] - relative_r @ left['T']
    _rotation(relative_r, 'relative R')
    scale = .001 if translation_unit == 'mm' else 1.
    relative_t = relative_t * scale
    baseline = float(np.linalg.norm(relative_t))
    if not .001 <= baseline <= 1.:
        raise ValueError('Factory baseline outside 0.001..1 m; check source translation units')
    try:
        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
            left['K'], left['D'], right['K'], right['D'], left['size'],
            relative_r, relative_t.reshape(3, 1), flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    except cv2.error as exc:
        raise ValueError('Factory matrices cannot be rectified: ' + str(exc)) from exc
    if not all(np.all(np.isfinite(v)) for v in (r1, r2, p1, p2, q)):
        raise ValueError('Factory rectification contains non-finite values')
    horizontal = abs(relative_t[0]) > max(abs(relative_t[1]), abs(relative_t[2]))
    compatible = bool(horizontal and p2[0, 3] < 0 and q[3, 2] > 0)
    source_hash = hashlib.sha256(raw).hexdigest()
    identity = json.dumps([IMPORT_SCHEMA, source_hash, left_source, right_source,
                           left_calibration_source, right_calibration_source,
                           translation_unit, rotation], separators=(',', ':'))
    calibration_id = 'gs130w_factory_' + hashlib.sha256(identity.encode()).hexdigest()[:24]
    evidence = None
    if mapping_evidence is not None:
        evidence_path = Path(mapping_evidence).resolve()
        evidence = {'path': str(evidence_path),
                    'sha256': hashlib.sha256(evidence_path.read_bytes()).hexdigest()}
    return {
        'schema': IMPORT_SCHEMA,
        'calibrated': True,
        'calibration_id': calibration_id,
        'calibration_source': 'factory_eeprom',
        'calibration_status': 'factory_parameters_imported_unverified',
        'camera_model': 'pinhole',
        'translation_unit': 'm',
        'image_size': list(left['size']),
        'K1': left['K'].tolist(), 'D1': left['D'].tolist(),
        'K2': right['K'].tolist(), 'D2': right['D'].tolist(),
        'R': relative_r.tolist(), 'T': relative_t.tolist(),
        'baseline_m': baseline,
        'input_rotation_deg': rotation,
        'source_topics': {'left': left_source, 'right': right_source},
        'calibration_source_topics': {'left': left_calibration_source, 'right': right_calibration_source},
        'input_topic_to_calibration_source': {
            left_source: left_calibration_source, right_source: right_calibration_source,
        },
        'source_order': ('explicit_input_to_factory_mapping' if explicit_calibration_mapping else
                         'explicit_source_slots' if explicit_order else 'factory_slot_order'),
        'physical_eye_mapping_verified': False,
        'eeprom_eye_assignment_verified': False,
        'synchronization_verified': False,
        'metric_depth_verified': False,
        'stereo_metric_calibration_verified': False,
        'positive_disparity_compatible': compatible,
        'matcher_compatibility_reason': ('horizontal_positive_disparity' if compatible else
                                         'source_order_requires_different_matcher_or_verified_eye_mapping'),
        'factory_rectification': {
            'method': 'cv2.stereoRectify', 'alpha': 0,
            'R1': r1.tolist(), 'R2': r2.tolist(),
            'P1': p1.tolist(), 'P2': p2.tolist(), 'Q': q.tolist(),
        },
        'provenance': {
            'imported_at_utc': datetime.now(timezone.utc).isoformat(),
            'source_snapshot': str(path), 'source_sha256': source_hash,
            'source_translation_unit': translation_unit, 'translation_scale_to_m': scale,
            'projection_convention': 'P=K[R|T]; R/T relative to common factory reference',
            'projection_max_abs_residual': {
                left_calibration_source: left['projection_residual'],
                right_calibration_source: right['projection_residual'],
            },
            'input_topic_to_calibration_source': {
                left_source: left_calibration_source, right_source: right_calibration_source,
            },
            'mapping_evidence': evidence,
            'factory_slot_sources': list(FACTORY_SLOT_SOURCES),
            'snapshot_role_to_source': document.get('role_to_source'),
            'snapshot_role_mapping_evidence': document.get('role_mapping_evidence'),
            'snapshot_provenance': original,
            'snapshot_driver_parameters': driver,
            'captured_client_date': document.get('captured_client_date'),
        },
        'notes': (
            'calibrated=true records imported factory parameters, not verified metric depth. '
            'Slot names left/right identify input order; physical eye identity is unverified. '
            'Input image topics and factory parameter sources are recorded separately. '
            'The snapshot physical-role mapping is preserved as evidence and is not applied implicitly. '
            'No image, K/D or translation sign is silently rotated/flipped. '
            'Keep automatic aiming/water verification separate from factory parameter availability.'
        ),
    }


def factory_lens_snapshot(snapshot_path, calibration):
    """Return a separate LensCorrector snapshot using the explicit topic map.

    Raw topic metadata is retained. K/D/R/P are copied from the selected
    EEPROM slots; R/P keep their common factory-reference convention.
    """
    path = Path(snapshot_path).resolve()
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != calibration['provenance']['source_sha256']:
        raise ValueError('Source snapshot changed after stereo import')
    original = json.loads(raw.decode('utf-8-sig'))
    result = deepcopy(original)
    for topic, parameter_source in calibration['input_topic_to_calibration_source'].items():
        parameters = original['sources'][parameter_source]
        for key in ('K', 'D', 'R', 'P', 'width', 'height', 'distortion_model'):
            result['sources'][topic][key] = deepcopy(parameters[key])
        result['sources'][topic]['factory_calibration_source'] = parameter_source
    result['role_to_source'] = deepcopy(calibration['source_topics'])
    result['role_mapping_evidence'] = (
        'Explicit input-topic to EEPROM-slot mapping; see factory_mapping.mapping_evidence. '
        'Mapping is not promoted to verified by this importer.'
    )
    result['provenance']['source'] = 'Factory EEPROM snapshot with explicit parameter-slot reassignment'
    for flag in ('eeprom_eye_assignment_verified', 'stereo_metric_calibration_verified',
                 'physical_eye_mapping_verified', 'metric_depth_verified', 'synchronization_verified'):
        result['provenance'][flag] = False
        if flag in result:
            result[flag] = False
    result['factory_mapping'] = {
        'calibration_id': calibration['calibration_id'],
        'source_snapshot': str(path),
        'source_sha256': calibration['provenance']['source_sha256'],
        'input_topic_to_calibration_source': deepcopy(calibration['input_topic_to_calibration_source']),
        'mapping_evidence': deepcopy(calibration['provenance']['mapping_evidence']),
        'original_provenance': deepcopy(original['provenance']),
        'note': 'Derived file; original snapshot is unchanged. R/P retain factory-slot extrinsics.',
    }
    return result
