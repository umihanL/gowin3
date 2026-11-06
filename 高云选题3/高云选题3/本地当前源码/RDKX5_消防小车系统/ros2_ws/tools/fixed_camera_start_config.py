"""Select fixed-camera geometry or measured water table; no physical I/O."""
import argparse
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.fixed_camera_aim import FixedCameraAim
from rescue_vision.fixed_camera_landing import FixedCameraLanding


INSTALLATION_OFFSETS = {
    't_camera_origin_in_gimbal_m': ('camera_relative_to_pan_vehicle_mm', 5.),
    'tilt_origin_in_pan_m': ('tilt_relative_to_pan_vehicle_mm', 1.),
    'nozzle_origin_in_tilt_m': ('nozzle_relative_to_tilt_vehicle_mm', 1.),
}


def _vector(value, name, maximum):
    if (not isinstance(value, list) or len(value) != 3 or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in value) or
            math.hypot(*value) > maximum):
        raise ValueError('fixed_camera_installation_invalid_vector:'+name)
    return value


def _installation_document(path):
    """Validate pending measurements without treating them as calibration."""
    doc = json.loads(path.read_text(encoding='utf-8-sig'))
    if not isinstance(doc, dict):
        raise ValueError('fixed_camera_config_requires_object')
    if (doc.get('mode') != 'fixed_camera_3d' or doc.get('calibrated') is not False or
            doc.get('axis_reference_verified') is not False or doc.get('translation_unit') != 'm' or
            doc.get('axis_model') != 'serial_pan_z_tilt_minus_y_with_offsets'):
        raise ValueError('fixed_camera_installation_requires_pending_metre_geometry')
    measured = doc.get('installation_measurements')
    if (not isinstance(measured, dict) or measured.get('unit') != 'mm' or
            measured.get('vehicle_frame') != 'x_forward_y_left_z_up' or
            doc.get('gimbal_base_frame') != 'x_vehicle_left_y_vehicle_rear_z_up' or
            measured.get('initial_camera_and_nozzle_heading') != 'vehicle_left' or
            not isinstance(measured.get('origin'), str) or not measured['origin'].strip() or
            doc.get('source_frame') not in ('undistorted_left_optical', 'rectified_left_optical', 'left_camera_optical')):
        raise ValueError('fixed_camera_installation_invalid_frame_metadata')
    identifier = doc.get('installation_measurement_id')
    if not isinstance(identifier, str) or not identifier.strip() or len(identifier) > 128:
        raise ValueError('fixed_camera_installation_measurement_id_required')
    transform = measured.get('Q_vehicle_to_gimbal')
    if not isinstance(transform, list) or len(transform) != 3:
        raise ValueError('fixed_camera_installation_invalid_vehicle_transform')
    transform = [_vector(row, 'Q_vehicle_to_gimbal', 1.) for row in transform]
    if transform != [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]:
        raise ValueError('fixed_camera_installation_invalid_vehicle_transform')
    for key, (raw_key, maximum) in INSTALLATION_OFFSETS.items():
        value = _vector(doc.get(key), key, maximum)
        raw = _vector(measured.get(raw_key), raw_key, maximum*1000)
        converted = [sum(row[j]*raw[j] for j in range(3))/1000 for row in transform]
        if any(not math.isclose(a, b, rel_tol=0., abs_tol=1e-9) for a, b in zip(value, converted)):
            raise ValueError('fixed_camera_installation_offset_frame_mismatch:'+key)
    return doc


def _warn_existing_geometry(document, installation, chosen):
    """A pending measurement never overwrites a selected calibrated profile."""
    try:
        measured = _installation_document(installation)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print('Fixed-camera installation comparison unavailable: '+str(exc)+
              '; selected configuration retained: '+str(chosen), file=sys.stderr)
        return
    different = [key for key in INSTALLATION_OFFSETS if any(
        not math.isclose(a, b, rel_tol=0., abs_tol=1e-9)
        for a, b in zip(document[key], measured[key]))]
    if different:
        print('Fixed-camera WARNING: selected calibrated geometry differs from current installation measurements ('+
              ', '.join(different)+'); measurements were NOT applied to '+str(chosen)+
              '. Review and recalibrate the active profile for the current installation.', file=sys.stderr)


def resolve(workspace, explicit=None, kind='aim'):
    validators = {'aim': FixedCameraAim, 'landing': FixedCameraLanding}
    if kind not in validators:
        raise ValueError('unknown_fixed_camera_configuration_kind')
    root = Path(workspace).resolve()
    local = root/f'config/fixed_camera_{kind}.local.json'
    installation = root/'config/fixed_camera_aim.installation.json'
    # lexists also catches a broken local symlink: its presence must produce
    # a visible error, never silently switch to an unrelated template.
    local_selected = explicit is not None or os.path.lexists(local)
    installation_selected = kind == 'aim' and not local_selected and os.path.lexists(installation)
    chosen = Path(explicit) if explicit is not None else (local if local_selected else
               installation if installation_selected else root/f'config/fixed_camera_{kind}.template.json')
    if not chosen.is_absolute():
        chosen = root/chosen
    chosen = chosen.resolve()
    if any(ch in str(chosen) for ch in ('\n', '\r')):
        raise ValueError('fixed_camera_config_invalid_path')
    document = (_installation_document(chosen) if installation_selected else
                json.loads(chosen.read_text(encoding='utf-8-sig')))
    if not isinstance(document, dict):
        raise ValueError('fixed_camera_config_requires_object')
    model = validators[kind](document)
    if local_selected and not model.ready:
        raise ValueError('fixed_camera_local_config_not_calibrated:'+str(chosen))
    if kind == 'aim' and local_selected and os.path.lexists(installation):
        _warn_existing_geometry(document, installation, chosen)
    return chosen, model.ready


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--config')
    parser.add_argument('--kind', choices=('aim', 'landing'), default='aim')
    args = parser.parse_args()
    try:
        path, calibrated = resolve(args.workspace, args.config, args.kind)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print('Fixed-camera startup configuration rejected: '+str(exc), file=sys.stderr)
        raise SystemExit(2)
    label = 'geometry' if args.kind == 'aim' else 'water-impact table'
    pending_installation = (args.kind == 'aim' and not calibrated and
                            path == (Path(args.workspace)/'config/fixed_camera_aim.installation.json').resolve())
    print('Fixed-camera '+label+' configuration: '+str(path)+
          (' (loaded; live calibration/reference checks remain active)' if calibrated else
           ' (installation dimensions loaded; optical, stereo and axis calibration pending; fixed-camera aiming remains unavailable)'
           if pending_installation else
           ' (unmeasured template; '+('fixed-camera aiming' if args.kind == 'aim' else 'water compensation')+
           ' remains unavailable)'), file=sys.stderr)
    print(path.as_posix())


if __name__ == '__main__':
    main()
