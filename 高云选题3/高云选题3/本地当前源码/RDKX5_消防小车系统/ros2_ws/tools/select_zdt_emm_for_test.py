#!/usr/bin/env python3
"""Select ordinary Emm temporarily, using vendor manual V1.0.5 p78.

Fixed M2, unloaded only. No EEPROM save, automatic retry, home or calibration.
Default dry run opens no UART. Read both retained targets before changing mode.
"""
import argparse
import datetime
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision import zdt_xs_protocol as wire
from rescue_vision.zdt_xs_transport import ZDTSerialTransport


def select_ordinary_emm(link, report):
    switched = False
    origin = None

    def exchange(request):
        entry = {'request_hex': request.hex(' ')}
        report.setdefault('transactions', []).append(entry)
        result = link.exchange(request)
        result.require_success()
        entry.update(response_data_hex=result.data.hex(' '), decoded=result.value)
        return result

    def read(command):
        return exchange(wire.read_request(1, command))

    def snapshot(label):
        version = read(wire.Command.READ_VERSION).value
        config = read(wire.Command.READ_CONFIG)
        options = read(wire.Command.READ_OPTIONS)
        state = read(wire.Command.READ_STATE).value
        values = {}
        for key, command in (
            ('position', wire.Command.READ_POSITION),
            ('final_target', wire.Command.READ_TARGET_POSITION),
            ('realtime_target', wire.Command.READ_REALTIME_TARGET_POSITION),
        ):
            values[key] = wire.position_degrees(read(command).value, 'emm')
        values['error'] = wire.error_degrees(read(wire.Command.READ_POSITION_ERROR).value, 'emm')
        values['speed'] = wire.speed_rpm(read(wire.Command.READ_SPEED).value, 'emm')
        item = dict(label=label, version=version, config=config.value,
                    config_hex=config.data.hex(' '), options_hex=options.data.hex(' '),
                    state=state, values=values)
        report.setdefault('samples', []).append(item)
        if (version.get('firmware_raw'), version.get('hardware_raw')) != (206, 0x2314):
            raise RuntimeError('unexpected_device')
        c = config.value
        if (c.get('mode'), c.get('microsteps'), c.get('motor_step_deg'),
                c.get('closed_loop'), c.get('address'), c.get('baudrate')) != ('emm', 16, 1.8, True, 1, 115200):
            raise RuntimeError('unexpected_configuration')
        if (c.get('checksum_mode'), c.get('uart_mode'), c.get('ack_mode'),
                c.get('positive_direction_ccw')) != (0, 2, 1, False):
            raise RuntimeError('unexpected_transport_or_direction_configuration')
        if state.get('enabled') is not True or state.get('raw', 0) & 0x7c:
            raise RuntimeError('not_enabled_or_fault_limit_unknown_state')
        if values['speed'] != 0 or abs(values['error']) > .2:
            raise RuntimeError('not_stationary')
        if any(abs(values[k] - values['position']) > .2 for k in ('final_target', 'realtime_target')):
            raise RuntimeError('unmatched_retained_target')
        if origin is not None and abs(values['position'] - origin) > .2:
            raise RuntimeError('position_changed_during_selection')
        raw = int.from_bytes(options.data, 'big')
        # Permitted to change mode only for this explicitly observed device.
        # This is NOT a motion profile or a definition of undocumented bit10.
        if len(options.data) != 2 or raw not in (0x0006, 0x0026, 0x0406, 0x0426):
            raise RuntimeError('unexpected_option_bits')
        return item, config, options

    try:
        first, config_before, options_before = snapshot('before')
        origin = first['values']['position']
        expected_config = config_before.data
        frame = wire.select_firmware_request(1, 'emm', persist=False)
        report['selection_frame_hex'] = frame.hex(' ')
        switched = True  # Even an uncertain write may have taken effect.
        exchange(frame)
        for i in range(3):
            time.sleep(.1)
            after, config, options = snapshot('after_%d' % (i + 1))
            if config.data != expected_config:
                raise RuntimeError('documented_configuration_changed')
            normalized = wire.normalize_options(after['version'], config.value, options.value)
            wire.verified_motion_parameters(config.value, normalized)
            report['normalized_options'] = normalized
        report.update(completed=True, options_before=options_before.data.hex(' '),
                      options_after=options.data.hex(' '), persistent_configuration_written=False,
                      final_enabled=True, note='Ordinary Emm selected in RAM only. No motion command sent.')
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        if switched:
            try:
                exchange(wire.enable_request(1, False))
                report['fault_disable_ack'] = True
                report['fault_state'] = read(wire.Command.READ_STATE).value
            except Exception as cleanup:
                report['cleanup_error'] = str(cleanup)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--unloaded', action='store_true')
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.execute and not args.unloaded:
        parser.error('--unloaded required')
    report = dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  completed=False, error=None, motion_commands_sent=0,
                  persistent_configuration_written=False,
                  manual_reference='V1.0.5 section 5.6.6 page78: D5 69 store00 type01')
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report), encoding='utf-8')
    try:
        if args.execute:
            with ZDTSerialTransport('/dev/ttyS3', baudrate=115200, timeout_s=.2) as link:
                select_ordinary_emm(link, report)
        else:
            report['note'] = 'Dry run: no UART opened; no settings changed.'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return int(report['error'] is not None)


if __name__ == '__main__':
    raise SystemExit(main())
