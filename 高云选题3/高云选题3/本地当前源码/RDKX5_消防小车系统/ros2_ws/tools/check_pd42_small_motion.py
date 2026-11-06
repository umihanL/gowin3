#!/usr/bin/env python3
"""Opt-in M1 PD42 unloaded +/-1 degree encoder-verified acceptance.

Defaults to an offline plan: no UART and no absolute targets are fabricated.
Execute only with the live motor worker disconnected and this UART exclusively owned.
No enable, home, clear-status, calibration, zero reset or command retry occurs.
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.pd42_protocol import (Command, RunState, MIN_POSITION_COUNTS,
    MAX_POSITION_COUNTS, counts_to_degrees, degrees_to_counts, read_request,
    position_request, enable_request)
from rescue_vision.pd42_transport import PD42SerialTransport

PORT, ADDRESS, BAUDRATE = '/dev/ttyS1', 1, 115200
STEP_COUNTS = degrees_to_counts(1.)
ERROR_DEG, EXCURSION_DEG = .2, 1.7
STAGE_TIMEOUT_S, TRANSACTION_TIMEOUT_S = 4., .12


class AcceptanceFault(RuntimeError):
    pass


def targets_from_origin(origin):
    if type(origin) is not int or not MIN_POSITION_COUNTS <= origin <= MAX_POSITION_COUNTS:
        raise AcceptanceFault('invalid_initial_position')
    targets = [origin + STEP_COUNTS, origin - STEP_COUNTS, origin]
    if any(not MIN_POSITION_COUNTS <= p <= MAX_POSITION_COUNTS for p in targets):
        raise AcceptanceFault('absolute_target_outside_signed_feedback_range')
    return targets


def plan():
    return dict(port=PORT, address=ADDRESS, baudrate=BAUDRATE,
        motor='M1 PD42 tilt axis (PIN8 TX/PIN10 RX); unloaded acceptance only',
        relative_targets_deg=[1., -1., 0.], absolute_targets_counts=None,
        speed_rpm=1, acceleration=1, segment_timeout_s=STAGE_TIMEOUT_S,
        required_consecutive_samples=3, max_position_error_deg=ERROR_DEG,
        max_excursion_from_start_deg=EXCURSION_DEG,
        initial_position='Read from encoder only after explicit execution gates pass',
        start_condition='Already enabled, no stall/fault, zero measured speed; never auto-enable',
        success_action='Leave enabled and close UART; original position verified by encoder',
        fault_action='One FA disable removes drive torque on the explicitly unloaded motor; no FC heat-braking, return or retry')


class SmallMotionCheck:
    def __init__(self, transport, report, clock=time.monotonic, sleep=time.sleep):
        self.io, self.report, self.clock, self.sleep = transport, report, clock, sleep
        self.origin = None
        self.started = clock()
        report.update(transactions=[], samples=[], segments=[], motion_commands_sent=0,
                      enable_commands_sent=0, disable_commands_sent=0)

    def exchange(self, request, *, deadline=None, purpose='feedback'):
        now = self.clock()
        if deadline is not None and now >= deadline:
            raise AcceptanceFault('segment_deadline_expired')
        self.io.timeout_s = min(TRANSACTION_TIMEOUT_S, deadline-now) if deadline is not None else TRANSACTION_TIMEOUT_S
        record = dict(t_s=now-self.started, purpose=purpose, request_hex=request.hex(' '),
                      command='0x%02X' % request[2])
        self.report['transactions'].append(record)
        if request[2] == Command.ABSOLUTE_POSITION:
            self.report['motion_commands_sent'] += 1  # includes uncertain transmission
        if request[2] == Command.ENABLE:
            self.report['disable_commands_sent' if request[3] == 1 else 'enable_commands_sent'] += 1
        try:
            response = self.io.exchange(request)
            record.update(response_success=response.ok, raw_data_hex=response.data.hex(' '),
                          decoded=response.value, finished_t_s=self.clock()-self.started)
            response.require_success()
            if deadline is not None and self.clock() > deadline:
                raise AcceptanceFault('segment_deadline_expired')
            return response.value
        except (Exception, KeyboardInterrupt) as exc:
            record.update(error=type(exc).__name__+': '+str(exc), finished_t_s=self.clock()-self.started)
            raise

    def read(self, command, deadline=None):
        return self.exchange(read_request(ADDRESS, command), deadline=deadline)

    def sample(self, deadline, stage):
        sample = dict(t_s=self.clock()-self.started, stage=stage)
        self.report['samples'].append(sample)
        position = self.read(Command.READ_POSITION, deadline)
        if type(position) is not int:
            raise AcceptanceFault('invalid_position_feedback')
        sample.update(position_counts=position, position_deg=counts_to_degrees(position))
        if self.origin is not None:
            sample['excursion_deg'] = counts_to_degrees(position-self.origin)
            if abs(sample['excursion_deg']) > EXCURSION_DEG:
                raise AcceptanceFault('excursion_exceeds_1_7_degrees')
        sample['speed_rpm'] = self.read(Command.READ_SPEED, deadline)
        sample['stalled'] = self.read(Command.READ_STALL, deadline)
        if sample['stalled'] is not False:
            raise AcceptanceFault('stall_or_invalid_stall_feedback')
        sample['enabled'] = self.read(Command.READ_ENABLED, deadline)
        if sample['enabled'] is not True:
            raise AcceptanceFault('motor_not_enabled_no_auto_enable')
        sample['run_state'] = int(self.read(Command.READ_STATUS, deadline))
        if sample['run_state'] not in (RunState.STOPPED, RunState.COMPLETED, RunState.RUNNING):
            raise AcceptanceFault('driver_run_state_fault')
        if type(sample['speed_rpm']) is not int:
            raise AcceptanceFault('invalid_speed_feedback')
        return sample

    def run(self):
        self.report['version'] = self.read(Command.READ_VERSION)
        initial_deadline = self.clock()+STAGE_TIMEOUT_S
        first = self.sample(initial_deadline, 'initial')
        if first['speed_rpm'] != 0 or first['run_state'] == RunState.RUNNING:
            raise AcceptanceFault('initial_motor_not_stopped')
        self.origin = first['position_counts']
        targets = targets_from_origin(self.origin)
        self.report.update(initial_position_counts=self.origin,
                           initial_position_deg=counts_to_degrees(self.origin),
                           absolute_targets_counts=targets)
        # Confirm a stationary baseline without issuing any actuator command.
        for _ in range(2):
            self.sleep(.04)
            observed = self.sample(initial_deadline, 'initial')
            if (observed['speed_rpm'] != 0 or observed['run_state'] == RunState.RUNNING or
                    abs(counts_to_degrees(observed['position_counts']-self.origin)) > ERROR_DEG):
                raise AcceptanceFault('initial_position_not_stable')
        for index, target in enumerate(targets, 1):
            started = self.clock()
            deadline = started+STAGE_TIMEOUT_S
            segment = dict(index=index, target_counts=target,
                relative_target_deg=counts_to_degrees(target-self.origin), started_t_s=started-self.started,
                completed=False, stable_samples=0)
            self.report['segments'].append(segment)
            self.exchange(position_request(ADDRESS, target, 1, 1, relative=False),
                          deadline=deadline, purpose='segment_%d_absolute_target' % index)
            while True:
                observed = self.sample(deadline, 'segment_%d' % index)
                error = counts_to_degrees(observed['position_counts']-target)
                observed['target_error_deg'] = error
                arrived = (abs(error) <= ERROR_DEG and observed['speed_rpm'] == 0 and
                           observed['run_state'] != RunState.RUNNING)
                segment['stable_samples'] = segment['stable_samples']+1 if arrived else 0
                if segment['stable_samples'] >= 3:
                    segment.update(completed=True, elapsed_s=self.clock()-started,
                                   final_position_counts=observed['position_counts'], final_error_deg=error)
                    break
                remaining = deadline-self.clock()
                if remaining <= 0:
                    raise AcceptanceFault('segment_deadline_expired')
                self.sleep(min(.04, remaining))
        self.report.update(completed=True, final_enabled_verified=True,
                           final_position_counts=self.report['segments'][-1]['final_position_counts'])

    def fault_stop_disable(self):
        # Unloaded-only: disable directly; a failed subsequent UART transaction
        # must not leave FC heat-braking latched. Never clear or resume afterward.
        cleanup = self.report['fault_cleanup'] = dict(disable_ack=False, disabled_readback=False)
        try:
            self.exchange(enable_request(ADDRESS, False), purpose='fault_disable')
            cleanup['disable_ack'] = True
        except (Exception, KeyboardInterrupt) as exc:
            cleanup['disable_error'] = type(exc).__name__+': '+str(exc)
        try:
            cleanup['disabled_readback'] = self.read(Command.READ_ENABLED) is False
        except (Exception, KeyboardInterrupt) as exc:
            cleanup['readback_error'] = type(exc).__name__+': '+str(exc)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--unloaded', action='store_true')
    parser.add_argument('--clearance-deg', type=float, help='Confirmed free travel on EACH side, at least 2 degrees')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    if args.execute and (not args.unloaded or args.clearance_deg is None or
                         not math.isfinite(args.clearance_deg) or args.clearance_deg < 2.):
        parser.error('Execution needs --execute --unloaded --clearance-deg 2 (or greater on BOTH sides)')
    stamp = datetime.datetime.now(datetime.timezone.utc)
    report_path = args.report or ROOT/'reports/pd42_small_motion'/(
        stamp.strftime('%Y%m%dT%H%M%S_%fZ')+'_'+str(os.getpid())+'.json')
    report = dict(created_utc=stamp.isoformat(), hardware_access_requested=args.execute,
        plan=plan(), completed=False, error=None, configuration_modified=False,
        homing_or_zero_reset=False, clear_status_sent=False, report_path=str(report_path))
    # Check report storage before any hardware access.
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    connection, check = None, None
    old_term = None
    try:
        if args.execute:
            def terminate(_signum, _frame):
                raise KeyboardInterrupt('termination_requested')
            old_term = signal.signal(signal.SIGTERM, terminate)
            connection = PD42SerialTransport(PORT, baudrate=BAUDRATE, timeout_s=TRANSACTION_TIMEOUT_S)
            check = SmallMotionCheck(connection, report)
            check.run()
        else:
            report['note'] = 'Offline plan only; no UART opened. Absolute targets remain unknown until encoder readback.'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
        if check is not None and report.get('motion_commands_sent', 0) > 0:
            check.fault_stop_disable()
        else:
            report['fault_cleanup'] = dict(skipped=True, reason='no_motion_issued_by_this_check')
    finally:
        if connection is not None:
            try:
                connection.close()
                report['uart_closed'] = True
            except Exception as exc:
                report['uart_close_error'] = str(exc)
        if old_term is not None:
            signal.signal(signal.SIGTERM, old_term)
        encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
        report_path.write_text(encoded+'\n', encoding='utf-8')
        print(encoded)
    return 1 if report['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
