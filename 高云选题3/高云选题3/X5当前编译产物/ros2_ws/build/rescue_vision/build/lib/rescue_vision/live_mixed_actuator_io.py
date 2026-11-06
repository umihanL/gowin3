"""Keep real mixed UART readback online before an explicit mechanical reference.

An absent per-boot reference never creates fake joint angles or opens motion.
The pending worker only uses read commands, including during reconnect and
shutdown. Explicit reference capture changes software coordinates, not driver
zero, enable state, calibration or retained targets. An external PD42 F8 zero
reset or mechanical coupling change invalidates a saved absolute origin and
requires explicit reference capture again.
"""
from collections import deque
from copy import deepcopy
from pathlib import Path
import math
import threading
import time
import uuid

from . import pd42_protocol as pd42, zdt_xs_protocol as zdt
from . import zdt_vendor_bench as vendor
from . import zdt_fd8_control as target_control
from .mixed_actuator_io import MixedActuatorIO, validate as validate_mixed
from .pd42_transport import PD42SerialTransport, PD42TimeoutError
from .zdt_xs_transport import ZDTSerialTransport, ZDTTimeoutError
from .gimbal_reference_policy import recoverable_reference_fault
from .absolute_reference import build_absolute_reference, restore_absolute_reference


def pending_config(config):
    """Validate serial/mechanical structure without accepting an old reference."""
    cfg = deepcopy(config)
    if cfg.get('dry_run') is not False or cfg.get('servo', {}).get('live_readback_until_reference') is not True:
        raise ValueError('Live reference readback requires explicit real mixed configuration')
    if cfg['servo'].get('reference_confirmed_this_start') is not False:
        raise ValueError('Pending readback cannot replace an already confirmed reference')
    if cfg.get('pump', {}).get('enabled') is not False:
        raise ValueError('Pending reference must not own an enabled local pump')
    # Historical targets/snapshots must not prevent UART inspection after a
    # reboot. They also must not be accepted as current mechanical coordinates.
    cfg['dry_run'] = True
    cfg['servo']['reference_id'] = None
    for name in ('pan', 'tilt'):
        axis = cfg['servo'][name]
        if not axis.get('port'):
            raise ValueError(name+' needs an explicit UART')
        axis['reference_counts'] = None
        axis.pop('zdt_fd8_snapshot', None)
        axis.pop('zdt_verified_snapshot', None)
    cfg = validate_mixed(cfg)
    cfg['dry_run'] = False
    return cfg


def read_sample(servo, ports, *, observed_hazard=None):
    """One bounded read-only sample; its age begins before the first read."""
    pan, tilt = servo['pan'], servo['tilt']
    started = time.monotonic()
    pread = lambda command: ports[pan['port']].exchange(zdt.read_request(pan['address'], command)).require_success().value
    tread = lambda command: ports[tilt['port']].exchange(pd42.read_request(tilt['address'], command)).require_success().value
    version = pread(zdt.Command.READ_VERSION)
    config = ports[pan['port']].exchange(zdt.read_request(pan['address'], zdt.Command.READ_CONFIG)).require_success()
    options = pread(zdt.Command.READ_OPTIONS)
    p = dict(position_before_counts=pread(zdt.Command.READ_POSITION),
             retained_target_counts=pread(zdt.Command.READ_TARGET_POSITION),
             realtime_target_counts=pread(zdt.Command.READ_REALTIME_TARGET_POSITION),
             position_error_deg=zdt.error_degrees(pread(zdt.Command.READ_POSITION_ERROR), 'emm'),
             speed_rpm=zdt.speed_rpm(pread(zdt.Command.READ_SPEED), 'emm'))
    if observed_hazard and p['speed_rpm'] != 0:
        observed_hazard('gimbal_raw_motion_present')
    p['state'] = pread(zdt.Command.READ_STATE)
    if observed_hazard and p['state']['raw'] & ~3:
        observed_hazard('gimbal_driver_fault_present')
    p['position_counts'] = pread(zdt.Command.READ_POSITION)
    if observed_hazard and p['state']['enabled'] and not vendor.settled(p,p['retained_target_counts']):
        observed_hazard('gimbal_retained_motion_unverified')
    p['absolute_counts'] = pread(zdt.Command.READ_ENCODER)
    t = tread(pd42.Command.READ_SYSTEM)
    if observed_hazard:
        if t['speed_rpm'] != 0:observed_hazard('gimbal_raw_motion_present')
        elif t['stalled'] or t['group_address']:observed_hazard('gimbal_driver_fault_present')
        elif t['enabled'] and abs(t['target_counts']-t['position_counts'])*360/51200 > .1:
            observed_hazard('gimbal_retained_motion_unverified')
    t['status'] = int(tread(pd42.Command.READ_STATUS))
    t['absolute_counts'] = pd42.single_turn_counts(t['position_counts'])
    return dict(started=started, pan=p, tilt=t, version=version,
                config_data_hex=config.data.hex(' '), options=options)


def capture_reason(rows, servo, *, now=None):
    """Reject moving, stale, faulty or unaligned drivers without writing them."""
    now = time.monotonic() if now is None else now
    if len(rows) < 3:
        return 'waiting_for_three_stationary_readbacks'
    if not 0 <= now-rows[-1]['started'] <= servo['feedback_timeout_s']:
        return 'raw_encoder_feedback_stale'
    first = rows[0]
    for row in rows:
        if not vendor.matches(row['version'], bytes.fromhex(row['config_data_hex']), row['options']):
            return 'zdt_live_configuration_not_verified_fd8'
        if (row['version'] != first['version'] or row['config_data_hex'] != first['config_data_hex']
                or row['options'] != first['options']):
            return 'driver_configuration_changed_during_capture'
        p, t = row['pan'], row['tilt']
        if (t['speed_rpm'] != 0 or t['stalled'] or t['group_address'] or t['status'] not in (0, 1)):
            return 'pd42_not_stationary_or_driver_fault'
        if abs(t['target_counts']-t['position_counts'])*360/51200 > .1:
            return 'pd42_retained_target_differs_no_automatic_rebase'
        state = p['state']
        if type(state.get('raw')) is not int or state['raw'] & ~3:
            return 'zdt_driver_fault_limit_or_reset'
        if not vendor.settled(p, p['retained_target_counts']):
            return 'zdt_not_stationary_or_retained_targets_differ'
        if (p['retained_target_counts'] != first['pan']['retained_target_counts'] or
                p['realtime_target_counts'] != first['pan']['realtime_target_counts'] or
                t['target_counts'] != first['tilt']['target_counts']):
            return 'retained_targets_changed_during_capture'
    for name, factor in (('pan', 360/65536), ('tilt', 360/51200)):
        counts = [row[name]['position_counts'] for row in rows]
        if (max(counts)-min(counts))*factor > .03:
            return name+'_moved_during_reference_capture'
    return ''


def fixed_nozzle_stationary_reason(rows, servo, *, now=None):
    """Read-only proof for a finite manual pulse after a latched motion fault.

    This does not restore motion permission or redefine encoder coordinates.
    Disabled axes may retain an old target; enabled axes must be aligned so a
    pending drive correction cannot begin after the operator starts the pump.
    """
    now = time.monotonic() if now is None else now
    if len(rows) < 3:
        return 'waiting_for_three_stationary_readbacks'
    stamps = [row.get('started') for row in rows]
    if (any(type(value) not in (int, float) or not math.isfinite(value) for value in stamps) or
            any(a >= b for a, b in zip(stamps, stamps[1:]))):
        return 'waiting_for_three_stationary_readbacks'
    if (not 0 <= now-stamps[-1] <= servo['feedback_timeout_s'] or
            not 0 < stamps[-1]-stamps[0] <= 1.):
        return 'raw_encoder_feedback_stale'
    enabled = None
    for row in rows:
        p, t = row['pan'], row['tilt']
        if any(type(value) is not int for value in (
                p.get('position_counts'), p.get('position_before_counts'),
                p.get('retained_target_counts'), p.get('realtime_target_counts'),
                t.get('target_counts'), t.get('position_counts'))):
            return 'gimbal_raw_position_invalid'
        if any(type(value) not in (int, float) or not math.isfinite(value)
               for value in (p.get('speed_rpm'), t.get('speed_rpm'), p.get('position_error_deg'))):
            return 'gimbal_raw_speed_invalid'
        if p['speed_rpm'] != 0 or t['speed_rpm'] != 0:
            return 'gimbal_raw_motion_present'
        state = p['state']
        if (type(state.get('raw')) is not int or state['raw'] & ~3 or
                t.get('stalled') is not False or t.get('group_address') is not False or
                type(t.get('status')) is not int or t['status'] not in (0, 1)):
            return 'gimbal_driver_fault_present'
        current_enabled = [state.get('enabled'), t.get('enabled')]
        if any(type(value) is not bool for value in current_enabled):
            return 'gimbal_enable_state_unknown'
        if enabled is not None and current_enabled != enabled:
            return 'gimbal_enable_state_changed'
        enabled = current_enabled
        if enabled[0] and not vendor.settled(p, p['retained_target_counts']):
            return 'zdt_enabled_retained_target_not_settled'
        if enabled[1] and abs(t['target_counts']-t['position_counts'])*360/51200 > .1:
            return 'pd42_enabled_retained_target_not_settled'
        if abs(p['position_counts']-p['position_before_counts'])*vendor.COUNT_DEG > .03:
            return 'pan_moved_during_stationary_readback'
    for name, factor in (('pan', 360/65536), ('tilt', 360/51200)):
        counts = [row[name]['position_counts'] for row in rows]
        if any(type(value) is not int for value in counts) or (max(counts)-min(counts))*factor > .03:
            return name+'_moved_during_stationary_readback'
    return ''


def verified_tilt_direction(config):
    commissioning = config.get('commissioning', {})
    if commissioning.get('tilt_physical_direction_verified') is not True:
        raise ValueError('Tilt physical direction must already be verified')
    nested = commissioning.get('tilt_direction_confirmation', {}).get('positive_motor_direction')
    legacy = commissioning.get('tilt_positive_direction')
    direction = nested if nested is not None else legacy
    if (direction not in ('up', 'down') or
            (nested is not None and legacy is not None and nested != legacy)):
        raise ValueError('Verified installation has no consistent tilt direction record')
    if config['servo']['tilt']['direction'] != (1 if direction == 'up' else -1):
        raise ValueError('Tilt direction sign differs from verified installation')
    return direction


def installation_reason(config):
    servo = config['servo']
    if not all(servo.get(key) is True for key in ('calibration_confirmed', 'load_supported_verified')):
        return 'installed_limits_gearing_or_supported_load_unconfirmed'
    try:
        verified_tilt_direction(config)
    except ValueError:
        return 'tilt_physical_direction_unconfirmed_or_inconsistent'
    if any(servo[name]['reference_angle_deg'] != 0 for name in ('pan', 'tilt')):
        return 'mechanical_zero_placement_not_configured_as_pan0_tilt0'
    return ''


def reference_config(config, rows, boot_id, reference_id):
    cfg = deepcopy(config)
    servo = cfg['servo']
    if not all(servo.get(key) is True for key in ('calibration_confirmed', 'load_supported_verified')):
        raise ValueError('Installed limits, gearing and supported load must already be confirmed')
    verified_tilt_direction(cfg)
    if not isinstance(boot_id, str) or not boot_id.strip():
        raise ValueError('Current X5 boot identity unavailable')
    for name in ('pan', 'tilt'):
        if servo[name]['reference_angle_deg'] != 0:
            raise ValueError('The confirmed placement is mechanical pan 0 / tilt 0')
        servo[name]['reference_counts'] = rows[-1][name]['position_counts']
    servo.update(reference_confirmed_this_start=True, reference_id=reference_id)
    servo['absolute_reference'] = build_absolute_reference(servo, rows, reference_id)
    servo['live_readback_until_reference'] = False
    servo['pan']['zdt_verified_snapshot'] = None
    last = rows[-1]
    servo['pan']['zdt_fd8_snapshot'] = dict(profile=target_control.PROFILE,
        version=last['version'], config_data_hex=last['config_data_hex'],
        options_raw_hex=last['options']['raw_bytes_hex'],
        reference_position_counts=last['pan']['position_counts'],
        startup_encoder_offset_counts=(last['pan']['position_counts']-last['pan']['absolute_counts']) % 65536,
        retained_target_counts=last['pan']['retained_target_counts'],
        realtime_target_counts=last['pan']['realtime_target_counts'])
    cfg.setdefault('commissioning', {}).update(boot_id=boot_id,
        reference='operator confirmed horizontal-centre and level nozzle in console')
    return validate_mixed(cfg)


def restored_reference_config(config, rows, boot_id):
    """Rebuild driver coordinates around a saved physical single-turn origin."""
    cfg = deepcopy(config)
    reason = installation_reason(cfg)
    if reason:
        raise ValueError(reason)
    if not isinstance(boot_id, str) or not boot_id.strip():
        raise ValueError('Current X5 boot identity unavailable')
    servo = cfg['servo']
    anchor = servo.get('absolute_reference')
    restored = restore_absolute_reference(servo, rows, anchor)
    reference_id = anchor['reference_id']
    for name in ('pan', 'tilt'):
        servo[name]['reference_counts'] = restored[name]['reference_counts']
    servo.update(reference_confirmed_this_start=True, reference_id=reference_id,
                 live_readback_until_reference=False)
    last = rows[-1]
    servo['pan']['zdt_verified_snapshot'] = None
    servo['pan']['zdt_fd8_snapshot'] = dict(profile=target_control.PROFILE,
        version=last['version'], config_data_hex=last['config_data_hex'],
        options_raw_hex=last['options']['raw_bytes_hex'],
        reference_position_counts=servo['pan']['reference_counts'],
        retained_target_counts=last['pan']['retained_target_counts'],
        realtime_target_counts=last['pan']['realtime_target_counts'],
        startup_position_counts=last['pan']['position_counts'],
        startup_encoder_offset_counts=(last['pan']['position_counts']-last['pan']['absolute_counts']) % 65536,
        startup_reference_id=reference_id, startup_boot_id=boot_id)
    cfg.setdefault('commissioning', {}).update(boot_id=boot_id,
        reference='saved mechanical origin restored from stationary absolute encoder readback')
    return validate_mixed(cfg), [restored[name]['actual_angle_deg'] for name in ('pan', 'tilt')]


class LiveMixedActuatorIO:
    requires_measured_feedback = True
    backend = 'mixed_serial'

    def __init__(self, config, *, start_worker=True):
        confirmed = config.get('servo', {}).get('reference_confirmed_this_start') is True
        self.config = validate_mixed(config) if confirmed else pending_config(config)
        if self.config['dry_run'] or self.config['pump']['enabled']:
            raise ValueError('Live reference owner requires real I/O and no local pump')
        self.servo = self.config['servo']
        self.dry_run = False
        self.closed = False
        self.commands = deque(maxlen=256)
        self._mutex = threading.RLock()
        self._transition_lock = threading.Lock()
        self._cancel = threading.Event()
        self._quit = threading.Event()
        self._thread = self._owner = self._delegate = None
        self._ports = {}
        self._rows = deque(maxlen=3)
        self._read_error = ''
        self._read_error_kind = 'none'
        # Losing a sample must never erase previously observed motion/faults.
        # This bounded latch is independent from the three-sample deque.
        self._fixed_nozzle_readonly_origin = not confirmed
        self._fixed_nozzle_hazard = ''
        self._capturing = False
        self._recovery_thread = None
        self._recovery_error = ''
        self._recovery_diagnostic = None
        self._next_recovery_at = 0.
        self._next_restore_at = 0.
        self._restore_attempted = confirmed
        self._restored_result = None
        self._restored_reference_id = None
        self._restore_pending_accept = False
        self._automatic_restore_allowed = self.servo.get('absolute_reference_restore_allowed') is True
        self._restore_state = ('saved' if confirmed and self.servo.get('absolute_reference') else
            'waiting' if self._automatic_restore_allowed and self.servo.get('absolute_reference') else
            'not_authorized' if self.servo.get('absolute_reference') else 'not_saved')
        self._restore_error = ('' if confirmed or self._automatic_restore_allowed else
            self.servo.get('absolute_reference_restore_reason', ''))
        self._start_worker_enabled = start_worker
        if confirmed:
            self._delegate = MixedActuatorIO(self.config)
            self.commands = self._delegate.commands
        elif start_worker:
            self._start_worker()

    def _open(self):
        if self._owner is None:
            from .local_actuator_io import _ExclusiveOwner
            self._owner = _ExclusiveOwner()
        for name, factory in (('pan', ZDTSerialTransport), ('tilt', PD42SerialTransport)):
            axis = self.servo[name]
            if axis['port'] not in self._ports:
                self._ports[axis['port']] = factory(axis['port'], self.servo['baudrate'], self.servo['timeout_s'])

    def _release(self):
        errors = []
        for port, resource in list(self._ports.items()):
            try:
                resource.close()
                del self._ports[port]
            except Exception as exc:
                errors.append(str(exc))
        # Retain the global lock if any UART failed to close. Another task
        # must not take ownership while a previous descriptor may still exist.
        if errors:
            raise RuntimeError('Readback ownership release failed: '+'; '.join(errors))
        if self._owner is not None:
            self._owner.close()
            self._owner = None

    def _start_worker(self):
        self._quit.clear()
        self._thread = threading.Thread(target=self._run, name='mixed-live-reference-readback', daemon=True)
        self._thread.start()

    def _sample(self):
        self._open()
        row = read_sample(self.servo, self._ports, observed_hazard=self._remember_nozzle_hazard)
        with self._mutex:
            self._rows.append(row)
            self._read_error = ''
            self._read_error_kind = 'none'
            p, t = row['pan'], row['tilt']
            if (p['speed_rpm'] != 0 or t['speed_rpm'] != 0 or
                    abs(p['position_counts']-p['position_before_counts'])*vendor.COUNT_DEG > .03):
                self._fixed_nozzle_hazard = 'gimbal_raw_motion_present'
            elif (p['state']['raw'] & ~3 or t['stalled'] or t['group_address'] or t['status'] not in (0, 1)):
                self._fixed_nozzle_hazard = 'gimbal_driver_fault_present'
            elif ((p['state']['enabled'] and not vendor.settled(p,p['retained_target_counts'])) or
                    (t['enabled'] and abs(t['target_counts']-t['position_counts'])*360/51200 > .1)):
                self._fixed_nozzle_hazard = 'gimbal_retained_motion_unverified'
            elif not fixed_nozzle_stationary_reason(list(self._rows),self.servo):
                self._fixed_nozzle_hazard = ''
        return row

    def _remember_nozzle_hazard(self, reason):
        # A later axis timeout cannot hide motion already returned by the
        # earlier axis in this same incomplete read-only transaction.
        with self._mutex:
            self._fixed_nozzle_hazard = reason

    def _run(self):
        while not self._quit.is_set():
            started = time.monotonic()
            delay = 1/self.servo['poll_hz']
            try:
                self._sample()
                self._attempt_absolute_restore()
            except Exception as exc:
                with self._mutex:
                    self._read_error = str(exc)
                    self._read_error_kind = 'timeout' if isinstance(exc,(PD42TimeoutError,ZDTTimeoutError)) else 'fault'
                    if self._read_error_kind != 'timeout':
                        self._fixed_nozzle_hazard = 'gimbal_readback_fault_present'
                    self._rows.clear()
                try: self._release()
                except Exception as cleanup:
                    with self._mutex:
                        self._read_error += '; '+str(cleanup)
                        self._read_error_kind = 'fault'
                        self._fixed_nozzle_hazard = 'gimbal_readback_ownership_unverified'
                delay = 1.
            self._quit.wait(max(0., delay-(time.monotonic()-started)))

    def feedback(self):
        with self._mutex:
            if self._delegate is not None:
                result, handoff_reason = self._delegate.feedback_with_reference_status()
                error = result.get('error', '')
                if (error and not self._capturing and not self.closed and
                        time.monotonic() >= self._next_recovery_at and
                        (self._recovery_thread is None or not self._recovery_thread.is_alive())):
                    self._next_recovery_at = time.monotonic()+1.
                    self._recovery_thread = threading.Thread(target=self._recover_readback,
                        args=(error,), name='mixed-reference-fault-readback', daemon=True)
                    self._recovery_thread.start()
                reason = ('absolute_reference_restore_waiting_for_persistence' if self._restore_pending_accept else
                    'reference_capture_in_progress' if self._capturing else
                    self._read_error or installation_reason(self.config) or handoff_reason)
                ready = not reason and not error and result.get('feedback_fresh') is True
                result.update(serial_connected=result.get('feedback_fresh') is True,
                    fixed_nozzle_readonly_eligible=False,
                    fixed_nozzle_readonly_reason='fixed_nozzle_motion_retirement_unverified',
                    raw_feedback_fresh=result.get('feedback_fresh') is True,
                    raw_feedback_age_s=result.get('feedback_age_s'),
                    reference_capture_supported=True, reference_capture_ready=ready,
                    reference_capture_reason=reason or 'ready_for_operator_zero_confirmation',
                    reference_capture_active=self._capturing, readback_error=self._read_error,
                    reference_recovery_permitted=bool(ready and self._recovery_error),
                    reference_recovery_error=self._recovery_error,
                    absolute_reference_saved=bool(self.servo.get('absolute_reference')),
                    absolute_reference_restore_state=self._restore_state,
                    absolute_reference_restore_reason=self._restore_error)
                if self._restore_pending_accept:
                    result.update(reference_confirmed_this_start=False, feedback_fresh=False,
                        command_accepted=False, settled_measured=False)
                return result
            rows = list(self._rows)
            row = rows[-1] if rows else None
            delay = time.monotonic()-row['started'] if row else None
            fresh = bool(not self.closed and not self._read_error and row and
                         0 <= delay <= self.servo['feedback_timeout_s'])
            reason = ('reference_capture_in_progress' if self._capturing else
                      self._read_error or installation_reason(self.config) or capture_reason(rows, self.servo))
            stationary_reason = self._read_error or ('raw_encoder_feedback_stale' if not fresh else
                                 fixed_nozzle_stationary_reason(rows, self.servo))
            readonly_reason = ('fixed_nozzle_motion_retirement_unverified' if
                not self._fixed_nozzle_readonly_origin or self._delegate is not None or
                self.closed or self._capturing or self._restore_pending_accept or
                self._recovery_error or (self._recovery_thread is not None and self._recovery_thread.is_alive())
                else self._fixed_nozzle_hazard or
                ('fixed_nozzle_readback_not_pure_timeout' if self._read_error_kind not in ('none','timeout') else ''))
            diagnostic_angles = None
            if fresh and self.servo.get('absolute_reference'):
                try:
                    mapped = restore_absolute_reference(self.servo, rows, self.servo['absolute_reference'])
                    diagnostic_angles = [mapped[name]['actual_angle_deg'] for name in ('pan', 'tilt')]
                except (ValueError, TypeError, KeyError):
                    pass
            return dict(backend=self.backend, dry_run=False, ownership='production_serial_readback',
                feedback_required=True, feedback_fresh=False, feedback_age_s=delay,
                raw_feedback_fresh=fresh, raw_feedback_age_s=delay, serial_connected=fresh,
                fixed_nozzle_stationary_confirmed=not stationary_reason and not self._capturing,
                fixed_nozzle_stationary_reason=stationary_reason or ('reference_capture_in_progress' if self._capturing else ''),
                readback_error_kind=self._read_error_kind,
                fixed_nozzle_readonly_eligible=not readonly_reason,
                fixed_nozzle_readonly_reason=readonly_reason,
                fixed_nozzle_known_hazard=self._fixed_nozzle_hazard,
                actual_angles_deg=None, command_accepted=False, settled_measured=False,
                diagnostic_angles_deg=diagnostic_angles,
                diagnostic_feedback_fresh=bool(fresh and diagnostic_angles is not None),
                holding_position=False, holding_measured=False, hold_target_deg=None,
                target_deg=None, requested_target_deg=None, hardware_written=False,
                reference_confirmed_this_start=False, commissioning_required=True,
                axis_protocols=['zdt_xs', 'pd42'], readback_error=self._read_error, error='',
                motor_counts=[row['pan']['position_counts'], row['tilt']['position_counts']] if fresh else None,
                motor_rpm=[row['pan']['speed_rpm'], row['tilt']['speed_rpm']] if fresh else None,
                motor_enabled=[row['pan']['state']['enabled'], row['tilt']['enabled']] if fresh else None,
                feedback_limit_tolerance_deg=self.servo['feedback_limit_tolerance_deg'],
                reference_capture_ready=fresh and not reason and not self.closed,
                reference_capture_reason=reason or 'ready_for_operator_zero_confirmation',
                reference_capture_active=self._capturing, reference_capture_supported=True,
                reference_recovery_permitted=bool(fresh and not reason and self._recovery_error),
                reference_recovery_error=self._recovery_error,
                pan_feedback_diagnostic=deepcopy(self._recovery_diagnostic),
                absolute_reference_saved=bool(self.servo.get('absolute_reference')),
                absolute_reference_restore_state=self._restore_state,
                absolute_reference_restore_reason=self._restore_error)

    snapshot = feedback

    def _recover_readback(self, error):
        """Every interrupted owner returns to reads; never resume its target."""
        with self._transition_lock:
            if (not error or self.closed or self._delegate is None or
                    self._delegate.feedback().get('error') != error):
                return
            try:
                diagnostic = self._delegate.feedback().get('pan_feedback_diagnostic')
                if recoverable_reference_fault(error):
                    self._delegate.detach_for_reference()
                else:
                    # Wait for the stopped worker and release its descriptors.
                    # No fault-clear, encoder-zero or motor target is written.
                    self._delegate.close()
                with self._mutex:
                    self._delegate = None
                    self._rows.clear()
                    self._read_error = ''
                    self._recovery_error = error
                    self._recovery_diagnostic = deepcopy(diagnostic)
                    self._restore_attempted = True
                    self._restore_state = 'operator_recovery_required'
                if self._start_worker_enabled:
                    self._start_worker()
            except Exception as exc:
                with self._mutex:
                    self._read_error = 'Reference readback handoff failed: '+str(exc)
                    # A transient descriptor-close failure must not permanently
                    # disable the handoff path. Retry cleanup at a bounded rate;
                    # the interrupted target is never queued or resent.
                    self._next_recovery_at = time.monotonic()+1.

    def restart_readback(self):
        """Explicit operator reconnect, restoring only the saved mechanical origin.

        The old worker must relinquish both UARTs before a new reader starts.
        A later restore validates actual stationary samples, driver alarms,
        retained targets and the saved absolute encoder anchor. There is no
        old-goal queue, calibration write, automatic enable or target replay.
        """
        with self._transition_lock:
            if self.closed or self._capturing:
                raise RuntimeError('gimbal_recovery_owner_busy')
            self._quit.set()
            if self._thread is not None:
                self._thread.join(timeout=3.)
                if self._thread.is_alive():
                    raise RuntimeError('Readback worker still busy; ownership retained')
            if self._delegate is not None:
                self._delegate.close()
                self._delegate = None
            else:
                self._release()
            with self._mutex:
                self._rows.clear()
                self._read_error = ''
                self._cancel.clear()
                self._restore_pending_accept = False
                self._restored_result = None
                self._restored_reference_id = None
                self._restore_attempted = False
                self._next_restore_at = 0.
                self._automatic_restore_allowed = bool(self.servo.get('absolute_reference'))
                self._restore_state = 'waiting' if self._automatic_restore_allowed else 'not_saved'
                self._restore_error = ''
            if self._start_worker_enabled:
                self._start_worker()
        return dict(reconnected=True, motion_commands_sent=False)

    def _attempt_absolute_restore(self):
        if (self._restore_attempted or time.monotonic() < self._next_restore_at or
                not self._automatic_restore_allowed or not self.servo.get('absolute_reference') or
                not self._transition_lock.acquire(blocking=False)):
            return
        try:
            if self.closed or self._capturing or self._delegate is not None:
                return
            with self._mutex:
                rows = deepcopy(list(self._rows))
            reason = capture_reason(rows, self.servo)
            if reason:
                self._restore_error = reason
                return
            self._restore_attempted = True
            self._capturing = True
            boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            cfg, angles = restored_reference_config(self.config, rows, boot_id)
            if self.closed or self._cancel.is_set():
                raise RuntimeError('absolute_reference_restore_cancelled')
            self._release()
            delegate = MixedActuatorIO(cfg)
            if self.closed or self._cancel.is_set():
                delegate.close()
                raise RuntimeError('absolute_reference_restore_cancelled')
            with self._mutex:
                self._delegate = delegate
                self._fixed_nozzle_readonly_origin = False
                self.config, self.servo = delegate.config, delegate.config['servo']
                self.commands = delegate.commands
                self._restored_reference_id = self.servo['reference_id']
                self._restore_pending_accept = True
                self._restore_state = 'awaiting_persistence'
                self._restore_error = ''
                self._restored_result = dict(config=deepcopy(self.config),
                    reference_id=self.servo['reference_id'], boot_id=boot_id, baseline=rows,
                    restored_angles_deg=angles, restore_kind='absolute_encoder', motion_commands_sent=False)
                self._quit.set()
        except Exception as exc:
            with self._mutex:
                self._restore_state = 'failed'
                self._restore_error = str(exc)
                # Serial disconnect/open errors and a delayed stationary sample
                # do not invalidate the saved mechanical origin. Retry read-only
                # restoration, not a motor command or a new zero capture.
                transient = isinstance(exc, (OSError, TimeoutError)) or str(exc) in (
                    'absolute_reference_feedback_stale_or_unordered',
                    'Local actuators already have an owner in this process')
                if transient and not self.closed and not self._cancel.is_set():
                    self._restore_attempted = False
                    self._next_restore_at = time.monotonic()+1.
        finally:
            self._capturing = False
            self._transition_lock.release()

    def take_restored_reference(self):
        with self._mutex:
            result = self._restored_result
            self._restored_result = None
            return deepcopy(result)

    def finish_reference_restore(self, reference_id, success, reason=''):
        with self._mutex:
            if not self._restore_pending_accept or reference_id != self._restored_reference_id:
                raise ValueError('absolute_reference_restore_identity_mismatch')
            if success is True:
                self._restore_pending_accept = False
                self._restore_state = 'restored'
                self._restore_error = ''
            else:
                self._restore_state = 'failed'
                self._restore_error = reason or 'absolute_reference_profile_save_failed'

    def commit_reference(self, *, operator_zero_confirmed, supported_load,
                         stationary_confirmed, tilt_positive_direction=None, cancel_event=None):
        if any(flag is not True for flag in (operator_zero_confirmed, supported_load, stationary_confirmed)):
            raise ValueError('Explicit zero placement, supported load and stationary confirmation required')
        if not self._transition_lock.acquire(blocking=False):
            raise RuntimeError('Reference capture already in progress')
        def cancelled():
            if self.closed or self._cancel.is_set() or (cancel_event is not None and cancel_event.is_set()):
                raise RuntimeError('Reference capture cancelled; no zero or motion written')
        try:
            if self._restore_pending_accept:
                raise RuntimeError('absolute_reference_restore_waiting_for_persistence')
            direction = verified_tilt_direction(self.config)
            if tilt_positive_direction is not None and tilt_positive_direction != direction:
                raise ValueError('Requested tilt direction differs from verified installation')
            self._cancel.clear()
            cancelled()
            with self._mutex: self._capturing = True
            self._restore_attempted = True
            if self._delegate is not None:
                self._delegate.detach_for_reference()
                with self._mutex:
                    self._delegate = None
                    self._rows.clear()
            self._quit.set()
            if self._thread is not None:
                self._thread.join(timeout=3.)
                if self._thread.is_alive():
                    raise RuntimeError('Readback worker still busy; ownership retained')
            cancelled()
            rows = []
            for index in range(3):
                cancelled()
                rows.append(self._sample())
                if index < 2:
                    time.sleep(.05)
            cancelled()
            reason = capture_reason(rows, self.servo)
            if reason:
                raise ValueError(reason)
            boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            ref_id = 'manual-zero-'+uuid.uuid4().hex
            cfg = reference_config(self.config, rows, boot_id, ref_id)
            cancelled()
            self._release()
            cancelled()
            delegate = MixedActuatorIO(cfg)
            try:
                cancelled()
            except Exception:
                delegate.close()
                raise
            try:
                with self._mutex:
                    cancelled()
                    self._delegate = delegate
                    self._fixed_nozzle_readonly_origin = False
                    self.config = delegate.config
                    self.servo = self.config['servo']
                    self.commands = delegate.commands
                    self._restore_state = 'saved'
                    self._restore_error = ''
            except Exception:
                delegate.close()
                raise
            return dict(config=deepcopy(self.config), reference_id=ref_id,
                        boot_id=boot_id, baseline=deepcopy(rows), motion_commands_sent=False,
                        recovered_error=self._recovery_error)
        finally:
            with self._mutex: self._capturing = False
            if self._delegate is None and not self.closed and self._start_worker_enabled:
                if self._thread is None or not self._thread.is_alive():
                    self._start_worker()
            self._transition_lock.release()

    def set_gimbal(self, *args, **kwargs):
        with self._mutex:
            if self._restore_pending_accept:
                raise RuntimeError('absolute_reference_restore_waiting_for_persistence')
            if self.closed or self._capturing:
                raise RuntimeError('Reference capture or shutdown in progress')
            if self._delegate is None:
                raise RuntimeError('Mechanical zero is not confirmed; real encoder readback remains online')
            return self._delegate.set_gimbal(*args, **kwargs)

    def hold_gimbal(self):
        with self._mutex:
            if self._restore_pending_accept or self._capturing or self.closed:
                raise RuntimeError('Reference capture or persistence in progress')
            if self._delegate is None:
                raise RuntimeError('No referenced movement to hold')
            return self._delegate.hold_gimbal()

    def set_pump(self, enabled):
        with self._mutex:
            if enabled and (self._restore_pending_accept or self._capturing or self.closed):
                raise RuntimeError('Reference capture or persistence in progress')
            if enabled and self._delegate is None:
                raise RuntimeError('Pump unavailable before a mechanical reference')
            if self._delegate is not None:
                return self._delegate.set_pump(enabled)

    def disable_gimbal(self):
        with self._mutex:
            self._cancel.set()
            delegate = self._delegate
        if delegate is not None:
            return delegate.disable_gimbal()
        # There was no motion or enable command to cancel in read-only mode.

    def close(self):
        self._cancel.set()
        with self._transition_lock:
            if self.closed:
                return
            self.closed = True
            self._quit.set()
            if self._thread is not None:
                self._thread.join(timeout=3.)
                if self._thread.is_alive():
                    raise RuntimeError('Readback worker still busy; ownership retained')
            if self._delegate is not None:
                self._delegate.close()
            else:
                self._release()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
