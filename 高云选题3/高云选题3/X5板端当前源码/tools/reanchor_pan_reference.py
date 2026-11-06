"""Record an operator-confirmed pan zero from stationary readback; keep tilt zero.

Only read packets are sent. This does not move, enable, stop, clear driver
coordinates or command the pump. Stop the owning task service first. The
expected reference ID and absolute encoder count bind the confirmation to
the observed pose, rather than subtracting an arbitrary displayed angle.
"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import time
import uuid

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE/'src/rescue_vision'))
from rescue_vision.absolute_reference import restore_absolute_reference
from rescue_vision.gimbal_reference_store import persist_reference
from rescue_vision.live_mixed_actuator_io import (
    capture_reason, installation_reason, read_sample, restored_reference_config)
from rescue_vision.local_actuator_io import _ExclusiveOwner
from rescue_vision.pd42_transport import PD42SerialTransport
from rescue_vision.zdt_xs_transport import ZDTSerialTransport


def corrected_reference(config, rows, boot_id, expected_reference_id,
                        expected_absolute_counts, *, expected_tilt_deg=30.):
    """Pure coordinate update; the caller supplied fresh stationary samples."""
    cfg = deepcopy(config)
    pump = deepcopy(cfg.get('pump', {'enabled': False}))
    cfg['pump'] = {'enabled': False}  # validator below owns only motor UARTs
    servo = cfg['servo']
    if (servo.get('reference_id') != expected_reference_id or
            not isinstance(expected_reference_id, str) or not expected_reference_id):
        raise ValueError('pan_reanchor_reference_changed_since_confirmation')
    if (type(expected_absolute_counts) is not int or
            not 0 <= expected_absolute_counts < 65536 or
            type(expected_tilt_deg) not in (int, float) or not math.isfinite(expected_tilt_deg)):
        raise ValueError('pan_reanchor_expected_pose_invalid')
    reason = installation_reason(cfg) or capture_reason(rows, servo)
    if reason:
        raise ValueError(reason)
    anchor = servo.get('absolute_reference')
    before = restore_absolute_reference(servo, rows, anchor)
    current = rows[-1]['pan']['absolute_counts']
    displacement = abs((current-expected_absolute_counts+32768) % 65536-32768)*360/65536
    if displacement > .5:
        raise ValueError('pan_reanchor_physical_pose_changed_since_confirmation')
    if abs(before['tilt']['actual_angle_deg']-expected_tilt_deg) > .5:
        raise ValueError('pan_reanchor_tilt_pose_changed_since_confirmation')
    old_tilt = deepcopy(anchor['axes']['tilt'])
    ref_id = 'pan-zero-'+uuid.uuid4().hex
    anchor = deepcopy(anchor)
    anchor['reference_id'] = ref_id
    anchor['axes']['pan']['zero_absolute_counts'] = current
    servo['reference_id'] = ref_id
    servo['absolute_reference'] = anchor
    restored, angles = restored_reference_config(cfg, rows, boot_id)
    if restored['servo']['absolute_reference']['axes']['tilt'] != old_tilt:
        raise ValueError('pan_reanchor_must_preserve_tilt_anchor')
    restored['pump'] = pump
    audit = dict(kind='operator_confirmed_pan_only_zero',
        previous_reference_id=expected_reference_id,
        previous_pan_zero_absolute_counts=config['servo']['absolute_reference']['axes']['pan']['zero_absolute_counts'],
        pan_zero_absolute_counts=current, tilt_anchor_preserved=True,
        expected_absolute_counts=expected_absolute_counts,
        previous_angles_deg=[before[name]['actual_angle_deg'] for name in ('pan', 'tilt')],
        corrected_angles_deg=angles, physical_heading='vehicle_left',
        hardware_motion_written=False, driver_coordinate_clear_written=False)
    restored.setdefault('commissioning', {})['pan_reference_correction'] = audit
    return dict(config=restored, reference_id=ref_id, boot_id=boot_id,
        baseline=deepcopy(rows), audit=audit, motion_commands_sent=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=WORKSPACE)
    parser.add_argument('--expected-reference-id', required=True)
    parser.add_argument('--expected-absolute-counts', type=int, required=True)
    parser.add_argument('--expected-tilt-deg', type=float, default=30.)
    parser.add_argument('--operator-pan-zero-confirmed', action='store_true')
    parser.add_argument('--supported-load', action='store_true')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if not args.operator_pan_zero_confirmed or not args.supported_load:
        parser.error('Confirm physical pan zero and supported stationary load first')
    root = args.workspace.resolve(strict=True)
    config_path = root/'config/actuator_io.mixed.local.json'
    control_path = root/'config/local_fire_control.local.json'
    config = json.loads(config_path.read_text(encoding='utf-8-sig'))
    controls = json.loads(control_path.read_text(encoding='utf-8-sig'))
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    owner, ports = _ExclusiveOwner(), {}
    try:
        servo = config['servo']
        for name, factory in (('pan', ZDTSerialTransport), ('tilt', PD42SerialTransport)):
            port = servo[name]['port']
            ports[port] = factory(port, servo['baudrate'], servo['timeout_s'])
        rows = []
        for index in range(3):
            rows.append(read_sample(servo, ports))
            if index < 2:
                time.sleep(.05)
        result = corrected_reference(config, rows, boot_id, args.expected_reference_id,
            args.expected_absolute_counts, expected_tilt_deg=args.expected_tilt_deg)
        if args.apply:
            persist_reference(result, controls)
        report = root/'reports/gimbal_reference'
        report.mkdir(parents=True, exist_ok=True)
        output = report/(result['reference_id']+'.json')
        output.write_text(json.dumps(dict(applied=args.apply, **result), ensure_ascii=False,
                                     indent=2, allow_nan=False)+'\n', encoding='utf-8')
        print(json.dumps(dict(applied=args.apply, reference_id=result['reference_id'],
            audit=result['audit'], report=str(output)), ensure_ascii=False))
    finally:
        # Do not release exclusive ownership until every descriptor is closed.
        failures = []
        for port in ports.values():
            try:
                port.close()
            except Exception as exc:
                failures.append(str(exc))
        if failures:
            raise RuntimeError('pan_reanchor_serial_close_failed: '+'; '.join(failures))
        owner.close()


if __name__ == '__main__':
    main()
