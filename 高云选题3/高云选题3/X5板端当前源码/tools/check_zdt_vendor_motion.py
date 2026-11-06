#!/usr/bin/env python3
"""Opt-in, bounded M2 acceptance following the vendor Emm FD example.

Exactly +8/-8 pulses (0.9 degrees), mode 0 relative to the previous target,
1 RPM, acceleration selector 1. No firmware/config/calibration/zero writes.
Only the previously observed FW206/HW2314 configuration is accepted here.
This experiment does not certify unknown option bit10 for production motion.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision import zdt_xs_protocol as wire
from rescue_vision.zdt_xs_transport import ZDTSerialTransport

CONFIG = bytes.fromhex('21 15 19 02 02 02 00 10 01 00 04 b0 0b b8 13 88 05 07 01 00 01 01 00 08 08 98 07 d0 00 08')
OPTIONS = (b'\x00\x06', b'\x00\x26', b'\x04\x06', b'\x04\x26')
COUNT_DEG = 360.0 / 65536
DELTA_DEG = 8 * 1.8 / 16
FRAMES = (bytes.fromhex('01 fd 00 00 01 01 00 00 00 08 00 00 6b'),
          bytes.fromhex('01 fd 01 00 01 01 00 00 00 08 00 00 6b'))


class VendorMotionCheck:
    def __init__(self, link, report, clock=time.monotonic, sleep=time.sleep):
        self.link, self.report, self.clock, self.sleep = link, report, clock, sleep
        self.started = clock()
        self.origin = None
        self.initial_target = None
        self.options = None
        self.initial_enabled = None
        self.action_attempted = False
        report.update(completed=False, transactions=[], samples=[], segments=[],
                      motion_commands_attempted=0, persistent_configuration_written=False,
                      production_compatibility_changed=False,
                      software_monitor_is_not_mechanical_limit=True)

    def send(self, frame, deadline=None):
        now = self.clock()
        if deadline is not None and now >= deadline:
            raise RuntimeError('segment_deadline_expired')
        self.link.timeout_s = .15 if deadline is None else min(.15, deadline-now)
        row = dict(t_s=now-self.started, request_hex=frame.hex(' '))
        self.report['transactions'].append(row)
        if frame[1] in (wire.Command.ENABLE, wire.Command.POSITION_EMM):
            self.action_attempted = True  # even a failed write can have executed
        if frame[1] == wire.Command.POSITION_EMM:
            if frame not in FRAMES:
                raise RuntimeError('unexpected_motion_frame')
            self.report['motion_commands_attempted'] += 1
        try:
            response = self.link.exchange(frame)
            response.require_success()
            row.update(data_hex=response.data.hex(' '), decoded=response.value)
            if deadline is not None and self.clock() > deadline:
                raise RuntimeError('segment_deadline_expired')
            return response
        except (Exception, KeyboardInterrupt) as exc:
            row['error'] = type(exc).__name__ + ': ' + str(exc)
            raise

    def read(self, command, deadline=None):
        return self.send(wire.read_request(1, command), deadline)

    def configuration(self):
        version = self.read(wire.Command.READ_VERSION).value
        config = self.read(wire.Command.READ_CONFIG)
        options = self.read(wire.Command.READ_OPTIONS)
        if (version.get('firmware_raw'), version.get('hardware_raw')) != (206, 0x2314):
            raise RuntimeError('unexpected_device')
        if config.data != CONFIG or options.data not in OPTIONS:
            raise RuntimeError('not_the_explicitly_observed_configuration')
        if self.options is not None and options.data != self.options:
            raise RuntimeError('configuration_changed_during_check')
        self.options = options.data
        self.report.setdefault('configurations', []).append(dict(version=version,
            config_hex=config.data.hex(' '), options_hex=options.data.hex(' '),
            unknown_bit10=bool(int.from_bytes(options.data, 'big') & 0x400)))

    def sample(self, stage, deadline, require_enabled=True):
        start = self.clock()
        row = dict(stage=stage, t_s=start-self.started, atomic=False)
        self.report['samples'].append(row)
        def read_position():
            p = self.read(wire.Command.READ_POSITION, deadline).value
            if type(p) is not int:
                raise RuntimeError('invalid_position')
            if self.origin is not None and abs(p-self.origin)*COUNT_DEG > 1.3:
                raise RuntimeError('excursion_exceeds_1_3_degrees')
            return p
        row['position_before_counts'] = read_position()
        row['speed_rpm'] = wire.speed_rpm(self.read(wire.Command.READ_SPEED, deadline).value, 'emm')
        row['state'] = self.read(wire.Command.READ_STATE, deadline).value
        if row['state']['raw'] & 0x7c or (require_enabled and not row['state']['enabled']):
            raise RuntimeError('fault_limit_unknown_state_or_disabled')
        row['target_counts'] = self.read(wire.Command.READ_TARGET_POSITION, deadline).value
        row['realtime_target_counts'] = self.read(wire.Command.READ_REALTIME_TARGET_POSITION, deadline).value
        row['error_deg'] = wire.error_degrees(self.read(wire.Command.READ_POSITION_ERROR, deadline).value, 'emm')
        if abs(row['error_deg']) > 1.2:
            raise RuntimeError('following_error_exceeds_1_2_degrees')
        row['position_counts'] = read_position()
        row['sample_duration_s'] = self.clock()-start
        if row['sample_duration_s'] > .6:
            raise RuntimeError('feedback_snapshot_too_slow')
        row['position_deg'] = row['position_counts']*COUNT_DEG
        row['target_residual_deg'] = (row['position_counts']-row['target_counts'])*COUNT_DEG
        row['position_span_deg'] = abs(row['position_counts']-row['position_before_counts'])*COUNT_DEG
        return row

    @staticmethod
    def settled(row, expected_target):
        return (row['speed_rpm'] == 0 and abs(row['error_deg']) <= .10 and
                row['position_span_deg'] <= .03 and
                abs(row['position_counts']-expected_target)*COUNT_DEG <= .10 and
                abs(row['position_before_counts']-expected_target)*COUNT_DEG <= .10 and
                abs(row['target_counts']-expected_target) <= 2 and
                abs(row['realtime_target_counts']-expected_target) <= 2 and
                abs(row['target_counts']-row['realtime_target_counts']) <= 1)

    def stable_read(self, stage, expected, deadline, require_enabled=True):
        rows = [self.sample(stage, deadline, require_enabled) for _ in range(3)]
        if not all(self.settled(s, expected) for s in rows):
            raise RuntimeError('baseline_unsettled_or_retained_target_mismatch')
        if (max(s['position_counts'] for s in rows)-min(s['position_counts'] for s in rows))*COUNT_DEG > .03:
            raise RuntimeError('baseline_encoder_not_stable')
        if len({s['target_counts'] for s in rows}) != 1 or len({s['realtime_target_counts'] for s in rows}) != 1:
            raise RuntimeError('baseline_target_not_stable')
        return rows[-1]

    def run(self):
        self.configuration()
        deadline = self.clock()+6.
        first = self.sample('initial', deadline, require_enabled=False)
        self.origin = first['position_counts']
        self.initial_target = first['target_counts']
        self.initial_enabled = first['state']['enabled']
        self.report.update(initial_position_counts=self.origin, initial_target_counts=self.initial_target,
                           initially_enabled=self.initial_enabled)
        self.stable_read('initial', self.initial_target, deadline, require_enabled=False)
        if not self.initial_enabled:
            self.send(wire.enable_request(1, True), deadline)
        baseline = self.stable_read('enabled_baseline', self.initial_target, self.clock()+6.)
        if abs(baseline['position_counts']-self.origin)*COUNT_DEG > .15:
            raise RuntimeError('unexpected_enable_displacement')
        for index, (direction, frame) in enumerate(zip((1, -1), FRAMES), 1):
            self.configuration()
            previous = self.initial_target if index == 1 else self.report['segments'][0]['expected_target_counts']
            before = self.stable_read('before_%d' % index, previous, self.clock()+6.)
            # mode0 uses the preceding commanded target, NOT the measured position.
            expected = self.initial_target + (DELTA_DEG/COUNT_DEG if index == 1 else 0)
            segment = dict(index=index, direction=direction, pulses=8,
                before_position_counts=before['position_counts'], before_target_counts=before['target_counts'],
                expected_target_counts=expected, completed=False)
            self.report['segments'].append(segment)
            deadline = self.clock()+6.
            self.send(frame, deadline)
            stable = []
            while True:
                s = self.sample('segment_%d' % index, deadline)
                signed_delta = direction*(s['position_counts']-before['position_counts'])*COUNT_DEG
                ok = self.settled(s, expected) and .65 <= signed_delta <= 1.15
                if index == 2:
                    ok = ok and abs(s['position_counts']-self.origin)*COUNT_DEG <= .10
                stable = (stable+[s])[-3:] if ok else []
                if len(stable) == 3 and (max(x['position_counts'] for x in stable)-min(x['position_counts'] for x in stable))*COUNT_DEG <= .03:
                    segment.update(completed=True, measured_delta_deg=direction*signed_delta,
                        final_position_counts=s['position_counts'], final_target_counts=s['target_counts'],
                        target_residual_deg=s['target_residual_deg'], stable_samples=3)
                    break
                self.sleep(.03)
        self.report['return_residual_deg'] = (s['position_counts']-self.origin)*COUNT_DEG
        self.report['restored_target_error_counts'] = s['target_counts']-self.initial_target
        if not self.initial_enabled:
            self.disable('success_restore_disabled')
        self.report.update(completed=True, final_enabled=self.initial_enabled,
            conclusion='Bounded example motion only; unknown bit10 and production gimbal calibration remain unverified.')

    def disable(self, reason):
        row = self.report.setdefault('cleanup', {})
        row['reason'] = reason
        self.send(wire.enable_request(1, False))
        row['disable_ack'] = True
        state = self.read(wire.Command.READ_STATE).value
        row['state'] = state
        if state['enabled']:
            raise RuntimeError('disable_feedback_not_confirmed')
        row['disabled_readback'] = True

    def execute(self):
        try:
            self.run()
        except (Exception, KeyboardInterrupt) as exc:
            self.report['error'] = type(exc).__name__ + ': ' + str(exc)
            if self.action_attempted:
                try:
                    self.disable('fault_disable_no_return_or_retry')
                except (Exception, KeyboardInterrupt) as cleanup:
                    self.report['cleanup_error'] = str(cleanup)
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--unloaded', action='store_true')
    parser.add_argument('--clearance-deg', type=float)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.execute and (not args.unloaded or args.clearance_deg is None or
                         not math.isfinite(args.clearance_deg) or args.clearance_deg < 2.):
        parser.error('Requires confirmed unloaded installation and >=2 degrees clearance on BOTH sides')
    report = dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        axis='M2 ZDT horizontal, physical PIN3 TX/PIN5 RX', port='/dev/ttyS3',
        plan='Vendor FD mode0 +8/-8 pulses, 1 RPM, accel1, sync0; return to original target',
        hardware_requested=args.execute, completed=False, error=None)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    old_term = None
    try:
        if args.execute:
            def terminate(_sig, _frame):
                raise KeyboardInterrupt('termination_requested')
            old_term = signal.signal(signal.SIGTERM, terminate)
            with ZDTSerialTransport('/dev/ttyS3', baudrate=115200, timeout_s=.15) as link:
                VendorMotionCheck(link, report).execute()
        else:
            report['note'] = 'Offline plan only; no UART opened. Explicit diagnostic, not production authorization.'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        if old_term is not None:
            signal.signal(signal.SIGTERM, old_term)
        encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
        args.report.write_text(encoded+'\n', encoding='utf-8')
        print(encoded)
    return int(report['error'] is not None)


if __name__ == '__main__':
    raise SystemExit(main())
