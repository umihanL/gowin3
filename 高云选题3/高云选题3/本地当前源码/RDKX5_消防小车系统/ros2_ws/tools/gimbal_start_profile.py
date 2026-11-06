"""Start real gimbal readback every boot, with a separately confirmed joint reference."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys


def resolve(profile_path, workspace, boot_id, *, saved_absolute=False):
    path = Path(profile_path)
    if not path.is_file():
        return [], 'no_current_boot_gimbal_profile'
    try:
        profile = json.loads(path.read_text(encoding='utf-8'))
        if profile.get('schema_version') != 1 or (not saved_absolute and profile.get('boot_id') != boot_id):
            raise ValueError('gimbal_reference_boot_mismatch')
        if saved_absolute and profile.get('absolute_encoder_reference_saved') is not True:
            raise ValueError('saved_absolute_encoder_reference_unavailable')
        if profile.get('operator_zero_confirmed') is not True:
            raise ValueError('gimbal_operator_zero_unconfirmed')
        root = Path(workspace).resolve()
        args = []
        for name in ('io_config', 'control_config'):
            item = profile[name]
            config_path = Path(item['path']).resolve()
            if not config_path.is_relative_to(root / 'config') or not config_path.name.endswith('.local.json'):
                raise ValueError('gimbal_profile_config_outside_workspace')
            if '\n' in str(config_path) or '\r' in str(config_path):
                raise ValueError('gimbal_profile_invalid_path')
            raw = config_path.read_bytes()
            if hashlib.sha256(raw).hexdigest() != item['sha256']:
                raise ValueError('gimbal_profile_config_changed')
            cfg = json.loads(raw)
            if name == 'io_config':
                servo = cfg.get('servo', {})
                if cfg.get('dry_run') is not False or servo.get('backend') != 'mixed_serial':
                    raise ValueError('gimbal_profile_not_real_mixed')
                if servo.get('reference_id') != profile.get('reference_id') or not profile.get('reference_id'):
                    raise ValueError('gimbal_profile_reference_mismatch')
                if not all(servo.get(key) is True for key in ('calibration_confirmed', 'load_supported_verified')):
                    raise ValueError('gimbal_profile_installation_unconfirmed')
                if saved_absolute:
                    anchor = servo.get('absolute_reference') or {}
                    if anchor.get('schema_version') != 1 or anchor.get('reference_id') != profile['reference_id']:
                        raise ValueError('saved_absolute_encoder_reference_mismatch')
            elif cfg.get('enabled') is not True:
                raise ValueError('gimbal_controller_disabled')
            args.append(name + ':=' + str(config_path))
        return args + ['actuator_dry_run:=false', 'pd42_reference_confirmed:=true'], ''
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return [], str(exc)


def resolve_live_start(profile_path, workspace, boot_id, runtime_dir):
    """Reuse a valid reference, otherwise prepare real, read-only commissioning.

    Saved encoder counts describe the previous reference, never this boot's
    pose. Keep installation data for an explicit UI confirmation while the
    live wrapper publishes raw feedback without commanding the hardware.
    """
    args, reason = resolve(profile_path, workspace, boot_id)
    if args:
        current_io=json.loads(Path(args[0].split(':=',1)[1]).read_text(encoding='utf-8-sig'))
        if not current_io.get('servo',{}).get('absolute_reference'):
            return args, ''
        # A driver can reboot while the X5 stays on. Re-read its absolute
        # encoder on every service start instead of trusting running counts.
    root = Path(workspace).resolve()
    config_dir = root / 'config'
    saved_args, saved_reason = resolve(config_dir / 'gimbal-absolute-profile.local.json',
                                      workspace, boot_id, saved_absolute=True)
    if args:
        saved_args,saved_reason=resolve(profile_path,workspace,boot_id,saved_absolute=True)
    local_io = config_dir / 'actuator_io.mixed.local.json'
    source = (Path(saved_args[0].split(':=', 1)[1]) if saved_args else
              local_io if local_io.is_file() else config_dir / 'actuator_io.mixed.template.json')
    io = json.loads(source.read_text(encoding='utf-8-sig'))
    if not isinstance(io, dict) or io.get('servo', {}).get('backend') != 'mixed_serial':
        raise ValueError('live gimbal startup requires a mixed_serial installation')
    io = deepcopy(io)
    servo = io['servo']
    # Local configuration supplies installation limits/signs. The template
    # carries no verified installation and therefore only permits readback.
    if not local_io.is_file():
        servo['pan']['port'] = '/dev/ttyS3'
        servo['tilt']['port'] = '/dev/ttyS1'
    io['dry_run'] = False
    servo['live_readback_until_reference'] = True
    servo['reference_confirmed_this_start'] = False
    # A persistent anchor is only a candidate for real absolute readback.
    # Never turn an old per-boot count into a confirmed current pose here.
    servo['absolute_reference_restore_allowed'] = bool(saved_args)
    servo['absolute_reference_restore_reason'] = '' if saved_args else saved_reason
    io['live_reference'] = {'workspace': str(root), 'profile_path': str(Path(profile_path).resolve())}
    control_source = config_dir / 'local_fire_control.local.json'
    if saved_args:
        control_source = Path(saved_args[1].split(':=', 1)[1])
    if not control_source.is_file():
        control_source = config_dir / 'local_fire_control.template.json'
    control = json.loads(control_source.read_text(encoding='utf-8-sig'))
    if not isinstance(control, dict):
        raise ValueError('live gimbal controller configuration must be an object')
    # Reading/restoring motor references must not erase an explicitly enabled
    # finite FPGA pump channel on every service restart. The actuator still
    # starts DISARMED and independently requires fresh B13, stop, aim and water
    # calibration evidence before accepting any ON request. Templates and the
    # legacy GPIO backend retain their commissioning-only output policy.
    pump = io.setdefault('pump', {})
    keep_fpga_output = (source.name.endswith('.local.json') and
        control_source.name.endswith('.local.json') and
        pump.get('backend') == 'fpga_uart')
    if keep_fpga_output:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rescue_vision'))
        from rescue_vision.fpga_pump_io import validate_pump
        io['pump'] = validate_pump(pump)
        if type(control.get('pump_enabled')) is not bool:
            raise ValueError('local FPGA pump controller enable must be boolean')
        if control['pump_enabled'] and not pump['enabled']:
            raise ValueError('local FPGA pump controller requires pump.enabled=true')
        control['enabled'] = True
    else:
        pump['enabled'] = False
        control.update(enabled=True, pump_enabled=False)
    runtime = Path(runtime_dir)
    runtime.mkdir(parents=True, exist_ok=True)
    generated = []
    for name, doc in (('io', io), ('control', control)):
        path = runtime / ('gimbal-live-' + name + '.json')
        pending = path.with_suffix('.json.tmp')
        pending.write_text(json.dumps(doc, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        pending.replace(path)
        generated.append(('io_config' if name == 'io' else 'control_config') + ':=' + path.resolve().as_posix())
    return generated + ['actuator_dry_run:=false', 'pd42_reference_confirmed:=false'], (
        'saved_absolute_reference_waiting_for_live_readback' if saved_args else reason)


def apply_water_aim_policy(values, fixed_camera_config, runtime_dir):
    """Copy only the selected aiming policy into runtime actuator settings.

    Motor installation files remain byte-identical to the saved reference
    manifest. A water policy change is not a new encoder zero or permission
    to bypass the reference, geometry, stationary or output checks.
    """
    selected = Path(fixed_camera_config).resolve()
    if any(ch in str(selected) for ch in ('\n', '\r')):
        raise ValueError('fixed_camera_config_invalid_path')
    document = json.loads(selected.read_text(encoding='utf-8-sig'))
    if not isinstance(document, dict) or document.get('mode') != 'fixed_camera_3d':
        raise ValueError('fixed_camera_water_policy_requires_fixed_camera_3d')
    policy = document.get('water_aim_policy', 'measured_impact')
    if policy not in ('measured_impact', 'straight_ray'):
        raise ValueError('fixed_camera_water_policy_invalid')
    if len(values) != 4 or not values[0].startswith('io_config:='):
        raise ValueError('gimbal_runtime_policy_requires_resolved_profile')
    source = Path(values[0].split(':=', 1)[1])
    io = json.loads(source.read_text(encoding='utf-8-sig'))
    io.setdefault('pump', {})['water_aim_policy'] = policy
    directory = Path(runtime_dir).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    output = directory / 'gimbal-live-io.json'
    pending = output.with_suffix('.json.tmp')
    pending.write_text(json.dumps(io, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                       encoding='utf-8')
    pending.replace(output)
    return ['io_config:=' + output.as_posix()] + values[1:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default='/run/rescue/gimbal-profile.json')
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--runtime-dir', default='/run/rescue')
    parser.add_argument('--fixed-camera-config',
                        help='Geometry selected by fixed_camera_start_config; shares its water policy only')
    args = parser.parse_args()
    try:
        boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        boot_id = ''
    try:
        values, reason = resolve_live_start(args.profile, args.workspace, boot_id, args.runtime_dir)
        if args.fixed_camera_config:
            values = apply_water_aim_policy(values, args.fixed_camera_config, args.runtime_dir)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, 'Live gimbal startup configuration rejected: ' + str(exc) + '\n')
    if reason:
        print('Gimbal real UART readback enabled; reference status: ' + reason, file=sys.stderr)
    print('\n'.join(values), end='\n' if values else '')


if __name__ == '__main__':
    main()
