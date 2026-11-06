"""PD42S1 two-axis adapter. Serial work stays off the ROS executor thread.

Startup reads only. Motion requires an explicit leased target, measured position,
installation references confirmed for this start, and opt-in enable-on-command.
No homing, zeroing, calibration, fault clearing or automatic reconnection occurs.
Expiry uses FA disable, never the driver's heat-producing latched FC brake.
The tilt load must be independently supported before using this stop policy.
"""
from collections import deque
from copy import deepcopy
import math
import re
import threading
import time

from .pd42_protocol import (Command, RunState, COUNTS_PER_REVOLUTION,
                            read_request, position_request, enable_request)
from .pd42_transport import PD42SerialTransport
from .gimbal_limits import feedback_limit_tolerance, require_measured_limits


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(name + ' must be finite')
    return float(value)


def validate(config):
    cfg = deepcopy(config)
    if type(cfg.get('dry_run', True)) is not bool:
        raise ValueError('dry_run must be boolean')
    cfg.setdefault('dry_run', True)
    s = cfg.get('servo', {})
    if s.get('backend') != 'pd42_serial' or s.get('transport') not in ('ttl', 'rs485', 'usb_serial'):
        raise ValueError('Select pd42_serial with ttl, rs485 or usb_serial transport')
    s['feedback_limit_tolerance_deg'] = feedback_limit_tolerance(s)
    if s.get('stop_policy') != 'disable':
        raise ValueError('This adapter supports only FA disable; no automatic FC/FB brake recovery')
    for flag in ('enable_on_command', 'reference_confirmed_this_start', 'load_supported_verified', 'calibration_confirmed'):
        if type(s.get(flag)) is not bool:
            raise ValueError(flag + ' must be an explicit boolean')
    if not cfg['dry_run'] and not all(s[flag] for flag in (
            'reference_confirmed_this_start', 'load_supported_verified', 'calibration_confirmed')):
        raise ValueError('Verify reference this start, mechanical limits and supported tilt load before real I/O')
    for key, lo, hi in (('poll_hz', 2, 20), ('timeout_s', .01, .3),
                        ('feedback_timeout_s', .1, 1.), ('position_tolerance_deg', .01, 2.)):
        s[key] = number(s.get(key), key)
        if not lo <= s[key] <= hi:
            raise ValueError(key + ' outside supported interval')
    if type(s.get('baudrate')) is not int or s['baudrate'] not in (9600,19200,38400,57600,115200,230400,460800):
        raise ValueError('Explicit supported baudrate required')
    addresses = []
    for name in ('pan', 'tilt'):
        a = s.get(name, {})
        if type(a.get('address')) is not int or not 1 <= a['address'] <= 247:
            raise ValueError(name + ' address must be unicast 1..247')
        port = a.get('port')
        if port is not None and (not isinstance(port, str) or not re.fullmatch(r'/dev/(?:tty[A-Za-z0-9]+|serial/(?:by-id|by-path)/[A-Za-z0-9_.:+-]+)', port)):
            raise ValueError(name + ' must name an explicit Linux serial device')
        if not cfg['dry_run'] and not port:
            raise ValueError(name + ' serial port is not configured')
        for key in ('min_angle_deg', 'max_angle_deg', 'reference_angle_deg', 'gear_ratio'):
            a[key] = number(a.get(key), name + '.' + key)
        if not -180 <= a['min_angle_deg'] < a['max_angle_deg'] <= 180:
            raise ValueError(name + ' mechanical limits must be within -180..180 degrees')
        if not a['min_angle_deg'] <= a['reference_angle_deg'] <= a['max_angle_deg'] or not 0 < a['gear_ratio'] <= 200:
            raise ValueError(name + ' invalid reference/gear ratio')
        if type(a.get('direction')) is not int or a['direction'] not in (-1, 1):
            raise ValueError(name + ' direction must be +1 or -1')
        counts = a.get('reference_counts')
        if counts is not None and (type(counts) is not int or not -(2**31) <= counts < 2**31):
            raise ValueError(name + ' reference_counts must be signed int32')
        if not cfg['dry_run'] and counts is None:
            raise ValueError(name + ' calibrated reference_counts required')
        if type(a.get('acceleration')) is not int or not 0 <= a['acceleration'] <= 200:
            raise ValueError(name + ' acceleration must be an integer in 0..200')
        if port:
            for previous_port, previous_address in addresses:
                if port == previous_port and (s['transport'] != 'rs485' or a['address'] == previous_address):
                    raise ValueError('TTL/USB axes need independent ports; RS485 needs unique addresses')
            addresses.append((port, a['address']))
        s[name] = a
    pump = cfg.setdefault('pump', {'enabled': False})
    if type(pump.get('enabled')) is not bool:
        raise ValueError('pump.enabled must be boolean')
    if pump['enabled'] and not cfg['dry_run']:
        if not isinstance(pump.get('gpio_chip'), str) or not re.fullmatch(r'/dev/gpiochip\d+', pump['gpio_chip']):
            raise ValueError('Explicit pump GPIO chip required')
        if type(pump.get('line_offset')) is not int or pump['line_offset'] < 0 or type(pump.get('active_low')) is not bool:
            raise ValueError('Explicit pump line/polarity required')
    cfg['servo'] = s
    return cfg


def angle_to_counts(angle, axis):
    angle = number(angle, 'angle')
    if not axis['min_angle_deg'] <= angle <= axis['max_angle_deg']:
        raise ValueError('Angle exceeds measured mechanical limits')
    if axis['reference_counts'] is None:
        raise ValueError('Axis reference has not been measured')
    counts = round(axis['reference_counts'] + (angle-axis['reference_angle_deg']) *
                   axis['direction'] * axis['gear_ratio'] * COUNTS_PER_REVOLUTION / 360.)
    if not -(2**31) <= counts < 2**31:
        raise ValueError('Position exceeds driver int32 range')
    return counts


def counts_to_angle(counts, axis):
    if axis['reference_counts'] is None:
        return None
    return axis['reference_angle_deg'] + (counts-axis['reference_counts']) * 360. / (
        COUNTS_PER_REVOLUTION * axis['gear_ratio'] * axis['direction'])


class PD42ActuatorIO:
    requires_measured_feedback = True
    backend = 'pd42_serial'

    def _validate_config(self, config):
        return validate(config)

    def _make_transport(self, axis):
        return PD42SerialTransport(axis['port'], self.servo['baudrate'], self.servo['timeout_s'])

    def _position_packet(self, axis, angle, rpm):
        return position_request(axis['address'], angle_to_counts(angle, axis), rpm, axis['acceleration'])

    def _enable_axis(self, axis, enabled):
        self._exchange(axis, enable_request(axis['address'], enabled))

    def _pre_enable_axis(self, name, axis, measured):
        """Read retained target before energizing; never rebase or clear it.

        PD42 exposes the target through READ_SYSTEM (0x31), not 0x2B's
        position error. Two snapshots must show a stationary disabled axis
        already near that target. This runs only for disabled -> enabled.
        """
        first = self._exchange(axis, read_request(axis['address'], Command.READ_SYSTEM))
        status = self._exchange(axis, read_request(axis['address'], Command.READ_STATUS))
        last = self._exchange(axis, read_request(axis['address'], Command.READ_SYSTEM))
        if status not in (RunState.STOPPED, RunState.COMPLETED):
            raise RuntimeError(name + ' must be stopped before enable')
        factor = 360. / (COUNTS_PER_REVOLUTION * axis['gear_ratio'])
        tolerance = min(.1, self.servo['position_tolerance_deg'])
        for row in (first, last):
            if (not isinstance(row, dict) or row.get('enabled') is not False or
                    row.get('stalled') is not False or row.get('group_address') is not False or
                    row.get('speed_rpm') != 0 or
                    type(row.get('position_counts')) is not int or type(row.get('target_counts')) is not int):
                raise RuntimeError(name + ' retained target / disabled feedback is unverified')
            if abs(row['target_counts']-row['position_counts'])*factor > tolerance:
                raise RuntimeError(name + ' retained target differs from actual position; enable refused, no automatic rebase')
        span = max(first['position_counts'], last['position_counts'], measured['counts']) - min(
            first['position_counts'], last['position_counts'], measured['counts'])
        if first['target_counts'] != last['target_counts'] or span*factor > min(.03, tolerance):
            raise RuntimeError(name + ' moved or target changed during pre-enable readback')

    def __init__(self, config):
        self.config = self._validate_config(config)
        self.servo = self.config['servo']
        self.dry_run = self.config['dry_run']
        self.commands = deque(maxlen=256)
        self._mutex = threading.RLock()
        self._quit = threading.Event()
        self._thread = self._owner = self._pump = None
        self._ports = {}
        self._goal = None
        self._holding = None
        self._generation = 0
        self._accepted_generation = None
        self._accepted_at = None
        self._feedback = None
        self._received = None
        self._active = False
        self._fault = ''
        self._reference_handoff = False
        self.closed = False
        self.commands.append({'operation': 'initialize', 'backend': self.backend, 'hardware_written': False})
        if self.dry_run:
            return
        from .local_actuator_io import _ExclusiveOwner, _PumpGPIO
        try:
            self._owner = _ExclusiveOwner()
            if self.config['pump']['enabled']:
                self._pump = _PumpGPIO(self.config['pump'])
            for name in ('pan', 'tilt'):
                port = self.servo[name]['port']
                if port not in self._ports:
                    self._ports[port] = self._make_transport(self.servo[name])
            self._thread = threading.Thread(target=self._run, name=self.backend+'-gimbal-io', daemon=True)
            self._thread.start()
        except Exception:
            self.close()
            raise

    def _exchange(self, axis, packet):
        value = self._ports[axis['port']].exchange(packet).require_success().value
        if packet[2] == Command.ENABLE and value is not (packet[3] == 0):
            raise RuntimeError('Driver enable/disable reply does not match requested state')
        return value

    def set_gimbal(self, pan_deg, tilt_deg, *, speed_deg_s=20., lease_s=.3):
        speed = number(speed_deg_s, 'speed_deg_s')
        lease = number(lease_s, 'lease_s')
        if not 0 < speed <= 90 or not 0 < lease <= .5:
            raise ValueError('Expected speed (0,90] deg/s and lease (0,0.5] seconds')
        targets, packets = [], []
        for name, requested in zip(('pan', 'tilt'), (pan_deg, tilt_deg)):
            a = self.servo[name]
            angle = number(requested, name)
            if not a['min_angle_deg'] <= angle <= a['max_angle_deg']:
                raise ValueError(name + ' angle exceeds measured limits')
            if a.get('motion_enabled', True) is not True and abs(angle-a['reference_angle_deg']) > .01:
                raise ValueError(name + ' physical direction not verified; keep the reference angle')
            targets.append(angle)
            rpm = math.floor(speed * a['gear_ratio'] / 6.)
            if not 1 <= rpm <= 6000:
                raise ValueError('Requested speed cannot be represented safely as integer motor RPM')
            if a['reference_counts'] is not None:
                packets.append(self._position_packet(a, angle, rpm))
        with self._mutex:
            if self.closed or self._fault or self._reference_handoff:
                raise RuntimeError(self._fault or 'Actuator is closed')
            self._holding = None
            key = (tuple(targets), speed)
            if self._goal is None or self._goal['key'] != key:
                self._generation += 1
                self._accepted_generation = None
                # The serial worker may send the new target before the next
                # ROS pump-guard tick. Remove water before allowing that move.
                if self._pump is not None:
                    try:
                        self._pump.set(False)
                    except Exception as exc:
                        self._goal = None
                        self._fault = 'Pump OFF before retarget unconfirmed: ' + str(exc)
                        raise RuntimeError(self._fault) from exc
                self.commands.append({'operation': 'gimbal', 'target_deg': targets,
                                      'speed_deg_s': speed, 'hardware_written': False})
            self._goal = dict(key=key, target=targets, packets=packets, generation=self._generation,
                              deadline=time.monotonic()+lease)

    def _read_axes(self):
        return [self._read_axis(name, self.servo[name]) for name in ('pan', 'tilt')]

    def _read_axis(self, name, a):
        values = {cmd: self._exchange(a, read_request(a['address'], cmd)) for cmd in (
            Command.READ_POSITION, Command.READ_SPEED, Command.READ_STATUS, Command.READ_ENABLED)}
        status = values[Command.READ_STATUS]
        if status not in (RunState.STOPPED, RunState.COMPLETED, RunState.RUNNING):
            raise RuntimeError(name + ' driver fault: ' + str(status))
        angle = counts_to_angle(values[Command.READ_POSITION], a)
        require_measured_limits(name, values[Command.READ_POSITION], angle, a,
                                self.servo['feedback_limit_tolerance_deg'])
        return dict(angle=angle, counts=values[Command.READ_POSITION],
                    rpm=values[Command.READ_SPEED], enabled=values[Command.READ_ENABLED], status=int(status))

    def feedback(self):
        with self._mutex:
            delay = None if self._received is None else time.monotonic()-self._received
            fresh = bool(not self.closed and not self.dry_run and not self._fault and delay is not None and
                         0 <= delay <= self.servo['feedback_timeout_s'])
            axes = self._feedback or []
            goal = self._goal
            accepted = bool(fresh and goal and self._active and
                            self._accepted_generation == goal['generation'] and
                            time.monotonic() < goal['deadline'])
            settled = bool(accepted and self._accepted_at is not None and self._received > self._accepted_at
                and all(a['enabled'] and a['rpm'] == 0 and
                a['status'] in (0, 1) and abs(a['angle']-goal['target'][i]) <= self.servo['position_tolerance_deg']
                for i, a in enumerate(axes)))
            holding = self._holding
            holding_measured = bool(fresh and holding and len(axes) == 2 and
                all(a['enabled'] and a['rpm'] == 0 and a['status'] in (0, 1) and
                    abs(a['angle']-holding['target'][i]) <= self.servo['position_tolerance_deg']
                    for i, a in enumerate(axes)))
            return dict(backend=self.backend, feedback_required=True, feedback_fresh=fresh,
                        feedback_age_s=delay, actual_angles_deg=[a['angle'] for a in axes] if fresh else None,
                        motor_counts=[a['counts'] for a in axes] if fresh else None,
                        motor_enabled=[a['enabled'] for a in axes] if fresh else None,
                        motor_rpm=[a['rpm'] for a in axes] if fresh else None,
                        command_accepted=accepted, settled_measured=settled,
                        holding_position=holding is not None, holding_measured=holding_measured,
                        feedback_limit_tolerance_deg=self.servo['feedback_limit_tolerance_deg'],
                        hold_target_deg=list(holding['target']) if holding else None,
                        hardware_written=not self.dry_run and self._active,
                        reference_confirmed_this_start=self.servo['reference_confirmed_this_start'],
                        error=self._fault, target_deg=list(goal['target']) if goal else None)

    def _disable_axes(self):
        failures = []
        with self._mutex:
            self._holding = None
            self._accepted_generation = None
            if self._pump is not None:
                try:
                    self._pump.set(False)
                except Exception as exc:
                    failures.append('pump OFF: ' + str(exc))
        axis_failed = False
        for name in ('pan', 'tilt'):
            a = self.servo[name]
            try:
                self._enable_axis(a, False)
            except Exception as exc:
                axis_failed = True
                failures.append(name + ': ' + str(exc))
        with self._mutex:
            self._active = axis_failed  # A missing reply cannot prove de-energization.
        if failures:
            error = 'Disable could not be confirmed: ' + '; '.join(failures)
            with self._mutex:
                self._fault = error
            raise RuntimeError(error)

    def _motion_gate(self, goal, captured):
        # A lease renewed by ROS cannot make old serial measurements fresh.
        with self._mutex:
            if self._reference_handoff:
                return False
            if time.monotonic() - captured > self.servo['feedback_timeout_s']:
                raise RuntimeError('Serial position feedback expired before motion write')
            valid = bool(not self._quit.is_set() and self._goal is not None and
                         self._goal['generation'] == goal['generation'] and
                         time.monotonic() < self._goal['deadline'])
            holding_completed_goal = bool(self._holding and
                self._holding['generation'] == goal['generation'] and not self._quit.is_set())
        if not valid and self._active and not holding_completed_goal:
            self._disable_axes()
        return valid

    def hold_gimbal(self):
        """Finish one measured move without releasing torque or writing motion.

        This is not a stop or a lease extension: only the current, acknowledged,
        measured-arrived generation can become an idle hold. New goals replace
        it; explicit disable, worker faults and close still release both axes.
        """
        with self._mutex:
            if self.closed or self._fault or self._quit.is_set():
                raise RuntimeError(self._fault or 'Actuator is closed')
            if not self.feedback().get('settled_measured') or self._goal is None:
                raise RuntimeError('Holding requires the current measured-arrived gimbal target')
            if self._pump is not None:
                self._pump.set(False)
            self._holding = dict(target=list(self._goal['target']), generation=self._goal['generation'])
            self._goal = None
            self._generation += 1
            self._accepted_generation = None
            self.commands.append({'operation': 'hold_gimbal',
                                  'target_deg': list(self._holding['target']), 'hardware_written': False})

    def _check_holding(self, axes, captured):
        """Monitor existing torque holding; never enable, rebase or resend."""
        with self._mutex:
            holding = deepcopy(self._holding)
        if holding is None:
            return False
        if time.monotonic()-captured > self.servo['feedback_timeout_s']:
            raise RuntimeError('Serial position feedback expired while holding')
        if len(axes) != 2 or any(not axis['enabled'] or
                abs(axis['angle']-holding['target'][i]) > self.servo['position_tolerance_deg']
                for i, axis in enumerate(axes)):
            raise RuntimeError('Gimbal holding enable or measured target lost; no automatic recovery')
        return True

    def _cycle(self):
        # All serial transactions run on one owner thread. Stop invalidates queued work.
        captured = time.monotonic()
        axes = self._read_axes()
        with self._mutex:
            if self._reference_handoff:
                return  # Explicit read-only transfer preserves existing torque.
            self._feedback, self._received = axes, captured
            goal = deepcopy(self._goal)
        if goal is None and self._check_holding(axes, captured):
            return
        if goal is None or time.monotonic() >= goal['deadline']:
            if self._active:
                self._disable_axes()
            return
        if not self._motion_gate(goal, captured):
            return
        if goal['generation'] == self._accepted_generation:
            return
        # Preflight ALL disabled axes before either axis can move. Otherwise
        # discovering an old tilt target after writing pan yields a partial move.
        for index, name in enumerate(('pan', 'tilt')):
            if not axes[index]['enabled']:
                if not self.servo['enable_on_command']:
                    raise RuntimeError(name + ' driver is disabled; explicit enable-on-command is not configured')
                self._pre_enable_axis(name, self.servo[name], axes[index])
                if not self._motion_gate(goal, captured):
                    return
        for index, name in enumerate(('pan', 'tilt')):
            a = self.servo[name]
            if not self._motion_gate(goal, captured):
                return
            # Serial responses may block; never hold the ROS-facing state mutex here.
            if not axes[index]['enabled']:
                self._active = True  # An enable can energize even if the reply is lost.
                self._enable_axis(a, True)
            if not self._motion_gate(goal, captured):
                return
            self._active = True
            self._exchange(a, goal['packets'][index])
        if not self._motion_gate(goal, captured):
            return
        with self._mutex:
            if self._goal is not None and self._goal['generation'] == goal['generation']:
                self._accepted_generation = goal['generation']
                self._accepted_at = time.monotonic()

    def _poll_interval_s(self):
        """Minimum cycle interval; serial transaction time is never hidden."""
        return 1. / self.servo['poll_hz']

    def _run(self):
        try:
            while not self._quit.is_set():
                start = time.monotonic()
                self._cycle()
                self._quit.wait(max(0., self._poll_interval_s()-(time.monotonic()-start)))
        except Exception as exc:
            with self._mutex:
                self._fault = str(exc)
                self._goal = None
                self._holding = None
            if self._pump is not None:
                try:
                    self._pump.set(False)
                except Exception:
                    pass
        finally:
            from .gimbal_reference_policy import recoverable_reference_fault
            keep_torque = self._reference_handoff and (not self._fault or recoverable_reference_fault(self._fault))
            if self._active and not keep_torque:
                try:
                    self._disable_axes()
                except Exception as exc:
                    with self._mutex:
                        self._fault = (self._fault + '; ' + str(exc)).strip('; ')

    def set_pump(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('Pump state must be boolean')
        with self._mutex:
            if self.closed:
                raise RuntimeError('Actuator is closed')
            if enabled and (not self.config['pump']['enabled'] or not self.feedback()['settled_measured']):
                raise RuntimeError('Pump requires configured output and measured settled PD42 axes')
            if self._pump is not None:
                self._pump.set(enabled)
            self.commands.append({'operation': 'pump', 'enabled': enabled})

    def reference_handoff_reason(self):
        """Check an idle handoff; final raw validation belongs to the new owner."""
        from .gimbal_reference_policy import recoverable_reference_fault
        with self._mutex:
            if self.closed or self.dry_run or self._reference_handoff:
                return 'actuator_not_available_for_reference'
            if self.config['pump']['enabled']:
                return 'local_pump_not_supported_during_reference'
            if self._fault:
                return '' if recoverable_reference_fault(self._fault) else self._fault
            if self._goal is not None or (self._active and self._holding is None):
                return 'another_gimbal_owner_active'
            feedback = self.feedback()
            if (not feedback.get('feedback_fresh') or len(self._feedback or []) != 2 or
                    any(axis['rpm'] != 0 or axis['status'] not in (0, 1)
                        for axis in self._feedback)):
                return 'both_axes_must_be_measured_stationary'
            return ''

    def detach_for_reference(self):
        """Release UART ownership without enabling, disabling, moving or zeroing.

        Used only for explicit software-reference capture or read-only recovery
        of a coordinate-limit fault. The next owner must validate raw targets.
        """
        with self._mutex:
            reason = self.reference_handoff_reason()
            if reason:
                raise RuntimeError(reason)
            faulted = bool(self._fault)
            if not faulted:
                self._reference_handoff = True
                self._quit.set()
        # A fault may have been published before the old worker's finally block.
        # Let its fault stop finish before taking ownership; recovery must never
        # suppress a stop for a target that faulted while it was moving.
        if self._thread is not None:
            self._thread.join(timeout=3.)
            if self._thread.is_alive():
                raise RuntimeError('Reference handoff worker busy; ownership retained')
        from .gimbal_reference_policy import recoverable_reference_fault
        if self._fault and not recoverable_reference_fault(self._fault):
            raise RuntimeError(self._fault)
        if faulted:
            self._reference_handoff = True
            self._quit.set()
        for port, resource in list(self._ports.items()):
            resource.close()
            del self._ports[port]
        if self._owner is not None:
            self._owner.close()
            self._owner = None
        self.closed = True

    def disable_gimbal(self):
        with self._mutex:
            self._goal = None
            self._holding = None
            self._generation += 1
            self._accepted_generation = None
            self.commands.append({'operation': 'disable_gimbal'})
            if self._pump is not None:
                try:
                    self._pump.set(False)
                except Exception as exc:
                    self._fault = 'Pump OFF unconfirmed: ' + str(exc)
                    raise RuntimeError(self._fault) from exc

    def close(self):
        if self.closed:
            return
        errors = []
        try:
            self.disable_gimbal()
        except Exception as exc:
            errors.append(str(exc))
        self._quit.set()
        if self._pump is not None:
            try:
                self._pump.set(False)
            except Exception as exc:
                errors.append(str(exc))
        if self._thread is not None:
            self._thread.join(timeout=3.)
            if self._thread.is_alive():
                raise RuntimeError('Serial worker did not stop; ownership retained')
        for port, resource in list(self._ports.items()):
            try:
                resource.close()
                del self._ports[port]
            except Exception as exc:
                errors.append(str(exc))
        if self._pump is not None:
            try:
                self._pump.close()
                self._pump = None
            except Exception as exc:
                errors.append(str(exc))
        # A failed descriptor close must remain retryable. Marking closed and
        # releasing the global owner here previously stranded a live UART and
        # made subsequent reconnect attempts skip cleanup permanently.
        if not self._ports and self._pump is None and self._owner is not None:
            try:
                self._owner.close()
                self._owner = None
            except Exception as exc:
                errors.append(str(exc))
        self.closed = not self._ports and self._pump is None and self._owner is None
        if errors:
            raise RuntimeError('Actuator shutdown unconfirmed: ' + '; '.join(errors))
