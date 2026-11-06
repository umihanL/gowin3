"""Share confirmed mechanical limits with Mission/FireAim; no hardware I/O."""
import argparse
import json
import math
from pathlib import Path


def generate(io_path, control_path, workspace, runtime_dir):
    root = Path(workspace).resolve()
    io = json.loads(Path(io_path).read_text(encoding='utf-8-sig'))
    control = json.loads(Path(control_path).read_text(encoding='utf-8-sig'))
    servo = io.get('servo', {})
    mount = json.loads((root/'config/mount.template.json').read_text(encoding='utf-8-sig'))
    planner = json.loads((root/'config/servo.template.json').read_text(encoding='utf-8-sig'))
    confirmed = (io.get('dry_run') is False and control.get('enabled') is True
                 and servo.get('backend') == 'mixed_serial'
                 and servo.get('calibration_confirmed') is True
                 and servo.get('load_supported_verified') is True
                 and all(servo.get(axis, {}).get('motion_enabled') is True
                         for axis in ('pan', 'tilt')))
    if confirmed:
        bounds = []
        for axis in ('pan', 'tilt'):
            values = [servo[axis].get('min_angle_deg'), servo[axis].get('max_angle_deg')]
            if (any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
                    or not -180 <= values[0] < values[1] <= 180):
                raise ValueError('Invalid confirmed mechanical limits: '+axis)
            bounds.append(values)
        # These are established task/standby coordinates, not raw encoder zero.
        if not (bounds[0][0] <= 0 <= bounds[0][1] and bounds[1][0] <= 30 <= bounds[1][1]):
            raise ValueError('Confirmed installation does not contain standby 0/30')
        speed = control.get('max_speed_deg_s')
        settle = control.get('settle_s')
        if (type(speed) not in (int, float) or not math.isfinite(speed) or not 0 < speed <= 90
                or type(settle) not in (int, float) or not math.isfinite(settle) or not .2 <= settle <= 5):
            raise ValueError('Invalid controller speed or settling interval')
        mount.update(mounting_verified=True, pan_home_deg=0., tilt_home_deg=30.,
                     pan_limits_deg=bounds[0], tilt_limits_deg=bounds[1],
                     speed_deg_s=speed, extra_settle_s=settle,
                     comment='Mechanical installation from the active mixed serial configuration. '
                             'Live encoder reference remains required; this is not optical/depth/water calibration.')
        planner.update(enabled=True, mounting_verified=True,
                       pan_home_deg=0., tilt_home_deg=30., pan_limits_deg=bounds[0],
                       tilt_limits_deg=bounds[1], max_speed_deg_s=speed)
    else:
        mount['mounting_verified'] = False
        planner.update(enabled=False, mounting_verified=False)
    directory = Path(runtime_dir).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    result = []
    for argument, name, value in (('mount_config', 'gimbal-task-mount.json', mount),
                                  ('servo_config', 'gimbal-task-servo.json', planner)):
        output = directory/name
        temporary = output.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        temporary.replace(output)
        result.append(argument+':='+output.as_posix())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--io-config', required=True)
    parser.add_argument('--control-config', required=True)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--runtime-dir', default='/run/rescue')
    args = parser.parse_args()
    try:
        values = generate(args.io_config, args.control_config, args.workspace, args.runtime_dir)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.exit(2, 'Mechanical task configuration rejected: '+str(exc)+'\n')
    print('\n'.join(values))


if __name__ == '__main__':
    main()
