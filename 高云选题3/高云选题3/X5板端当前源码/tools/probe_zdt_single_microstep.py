#!/usr/bin/env python3
"""Explicit unloaded, single-microstep commissioning of the observed M2.

Vendor V1.0.5 p57: FD mode2, Emm1.8deg/16 -> 0.1125deg per pulse.
This does not define bit10 or change production normalize_options rules.
No continuous speed, EEPROM write, homing, calibration or command retry.
"""
import argparse
import datetime
import json
import time
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision import zdt_xs_protocol as wire
from rescue_vision.zdt_xs_transport import ZDTSerialTransport

CONFIG = bytes.fromhex('21 15 19 02 02 02 00 10 01 00 04 b0 0b b8 13 88 05 07 01 00 01 01 00 08 08 98 07 d0 00 08')
STEP = 1.8 / 16


def run_probe(link, report):
    action_attempted = False
    origin = None

    def send(frame):
        row = dict(request_hex=frame.hex(' '), time_monotonic=time.monotonic())
        report.setdefault('transactions', []).append(row)
        response = link.exchange(frame)
        response.require_success()
        row.update(data_hex=response.data.hex(' '), decoded=response.value)
        return response

    def read(command):
        return send(wire.read_request(1, command))

    def configuration():
        version = read(wire.Command.READ_VERSION).value
        config = read(wire.Command.READ_CONFIG)
        options = read(wire.Command.READ_OPTIONS)
        if (version.get('firmware_raw'), version.get('hardware_raw')) != (206, 0x2314):
            raise RuntimeError('unexpected_device')
        if config.data != CONFIG or options.data not in (b'\x04\x06', b'\x04\x26', b'\x00\x06', b'\x00\x26'):
            raise RuntimeError('not_the_explicitly_observed_configuration')
        report.setdefault('configuration_samples', []).append(dict(
            version=version, config_hex=config.data.hex(' '), options_hex=options.data.hex(' '),
            unverified_extra_options=int.from_bytes(options.data, 'big') & ~0x3b7))
        return options.data

    def snapshot(label, *, require_enabled=True):
        s = dict(label=label, time_monotonic=time.monotonic())
        report.setdefault('samples', []).append(s)
        s['position_counts'] = read(wire.Command.READ_POSITION).value
        s['position'] = wire.position_degrees(s['position_counts'], 'emm')
        if origin is not None and abs(s['position'] - origin) > .45:
            raise RuntimeError('software_excursion_threshold_exceeded')
        s['speed'] = wire.speed_rpm(read(wire.Command.READ_SPEED).value, 'emm')
        s['state'] = read(wire.Command.READ_STATE).value
        if s['state']['raw'] & 0x7c or (require_enabled and not s['state']['enabled']):
            raise RuntimeError('fault_limit_unknown_state_or_disabled')
        for key, cmd in (('target', wire.Command.READ_TARGET_POSITION),
                         ('realtime_target', wire.Command.READ_REALTIME_TARGET_POSITION)):
            s[key + '_counts'] = read(cmd).value
            s[key] = wire.position_degrees(s[key + '_counts'], 'emm')
        s['error'] = wire.error_degrees(read(wire.Command.READ_POSITION_ERROR).value, 'emm')
        if abs(s['error']) > .45:
            raise RuntimeError('following_error_exceeded')
        return s

    def stationary(s, tolerance=.1):
        return (s['speed'] == 0 and abs(s['error']) <= tolerance and
                all(abs(s[k] - s['position']) <= tolerance for k in ('target', 'realtime_target')))

    try:
        initial_options = configuration()
        first = snapshot('initial_readonly', require_enabled=False)
        if not stationary(first):
            raise RuntimeError('initial_motion_or_stale_target')
        origin = first['position']
        # D5 can re-enable this firmware and is never repeated in a motion probe.
        # A manual turn while disabled leaves old targets: refuse before enable.
        if not first['state']['enabled']:
            action_attempted = True
            send(wire.enable_request(1, True))
        time.sleep(.2)
        baseline = []
        for i in range(3):
            s = snapshot('enabled_baseline_%d' % i)
            if not stationary(s, .055):
                raise RuntimeError('not_stationary_after_enable')
            baseline.append(s['position'])
            time.sleep(.06)
        if max(baseline) - min(baseline) > .03:
            raise RuntimeError('baseline_noise_too_large_for_single_step')
        origin = baseline[-1]
        report['baseline_position_deg'] = origin
        report['baseline_span_deg'] = max(baseline) - min(baseline)
        for index, direction in enumerate((1, -1), 1):
            if configuration() != initial_options:
                raise RuntimeError('options_changed_during_probe')
            before = snapshot('before_step_%d' % index)
            if not stationary(before, .055):
                raise RuntimeError('not_stationary_before_step')
            desired = direction * STEP
            expected = before['position'] + desired
            frame = wire.position_request(1, desired, 1, mode='emm', motion_mode=2,
                microsteps=16, motor_step_deg=1.8, speed_div10=False, acceleration=1)
            segment = dict(index=index, signed_pulses=direction, expected_delta_deg=desired,
                           before=before, completed=False)
            report.setdefault('segments', []).append(segment)
            report['motion_commands_attempted'] += 1
            action_attempted = True
            send(frame)
            # Emm speed feedback is integer RPM: sub-RPM settling can read 0.
            # Use encoder stability too, and allow settling without adding travel.
            deadline = time.monotonic() + 6.
            stable = 0
            stable_positions = []
            while time.monotonic() < deadline:
                s = snapshot('step_%d' % index)
                delta = s['position'] - before['position']
                # The acceptance tolerance must be narrower than one microstep:
                # a stationary motor must never pass this probe.
                arrived = (stationary(s, .025) and .05 <= direction * delta <= .18 and
                           abs(delta - desired) <= .055 and
                           all(abs(s[k] - expected) <= .055 for k in ('target', 'realtime_target')))
                stable = stable + 1 if arrived else 0
                stable_positions = (stable_positions + [s['position']])[-3:] if arrived else []
                if stable >= 3 and max(stable_positions) - min(stable_positions) <= .015:
                    segment.update(completed=True, measured_delta_deg=delta,
                                   after=s, stable_samples=stable,
                                   encoder_stable_span_deg=max(stable_positions) - min(stable_positions))
                    break
                time.sleep(.03)
            if not segment['completed']:
                raise RuntimeError('single_microstep_displacement_not_confirmed')
        residual = s['position'] - origin
        if abs(residual) > .06:
            raise RuntimeError('return_residual_exceeded')
        report.update(completed=True, final_enabled=s['state']['enabled'],
                      final_speed_rpm=s['speed'], return_residual_deg=residual,
                      note='Only this bounded single-microstep experiment passed. Bit10 remains unnamed; production compatibility unchanged.')
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        if action_attempted:
            try:
                send(wire.enable_request(1, False))
                report['fault_disable_ack'] = True
                report['fault_state'] = read(wire.Command.READ_STATE).value
            except Exception as cleanup:
                report['cleanup_error'] = str(cleanup)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--unloaded', action='store_true')
    parser.add_argument('--clearance-deg', type=float)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.execute and (not args.unloaded or args.clearance_deg is None or not 2 <= args.clearance_deg < float('inf')):
        parser.error('Explicit unloaded and at least +/-2deg clearance required')
    report = dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        completed=False, error=None, motion_commands_attempted=0, persistent_configuration_written=False,
        software_threshold_is_not_a_mechanical_limit=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report), encoding='utf-8')
    try:
        if args.execute:
            with ZDTSerialTransport('/dev/ttyS3', baudrate=115200, timeout_s=.12) as link:
                run_probe(link, report)
        else:
            report['note'] = 'Offline plan only; no UART opened. Fixed +1 then -1 microstep.'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return int(report['error'] is not None)


if __name__ == '__main__':
    raise SystemExit(main())
