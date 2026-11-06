"""ZDT_XS pan + PD42 tilt, using the existing leased serial owner worker.

Motor settings and encoder reference are read, never changed. ZDT input units
must match a verified commissioning snapshot; unsupported option bits remain
blocking. A stop releases torque only with the supported-load confirmation.
"""
from copy import deepcopy
import math
import time

from .pd42_actuator_io import PD42ActuatorIO, validate as validate_pd42, number, counts_to_angle
from . import pd42_protocol as pd42
from . import zdt_xs_protocol as zdt
from .zdt_xs_transport import ZDTSerialTransport
from . import zdt_fd8_control as fd8
from .gimbal_limits import require_measured_limits


def verified_snapshot(axis, baudrate):
    snap = axis.get('zdt_verified_snapshot')
    if not isinstance(snap, dict) or set(snap) != {'version', 'config', 'options'}:
        raise ValueError('ZDT needs a fresh verified version/config/options commissioning snapshot')
    version, cfg, options = snap['version'], snap['config'], snap['options']
    normalized = zdt.normalize_options(version, cfg, options)
    params = zdt.verified_motion_parameters(cfg, normalized)
    if cfg.get('baudrate') != baudrate or cfg.get('address') != axis['address']:
        raise ValueError('ZDT snapshot UART address/baudrate differ from configured connection')
    return dict(version=version, config=cfg, options=normalized), params


def validate(config):
    cfg = deepcopy(config)
    if cfg.get('servo', {}).get('backend') != 'mixed_serial':
        raise ValueError('Select mixed_serial backend')
    cfg['servo']['backend'] = 'pd42_serial'
    cfg = validate_pd42(cfg)
    cfg['servo']['backend'] = 'mixed_serial'
    servo = cfg['servo']
    for name, protocol in (('pan', 'zdt_xs'), ('tilt', 'pd42')):
        if servo[name].get('protocol') != protocol:
            raise ValueError(name + ' protocol must be ' + protocol)
    if servo['transport'] not in ('ttl', 'usb_serial'):
        raise ValueError('Mixed motors require independent UARTs; no shared mixed-protocol bus')
    if servo['pan'].get('port') and servo['pan']['port'] == servo['tilt'].get('port'):
        raise ValueError('Mixed motors require independent UARTs')
    axis = servo['pan']
    if axis.get('zdt_fd8_snapshot') is not None:
        if axis.get('zdt_verified_snapshot') is not None:
            raise ValueError('Select FD8-only or general verified ZDT control, never both')
        fd8.validate_snapshot(axis, servo['baudrate'], servo['position_tolerance_deg'],
                              reference_id=servo.get('reference_id'),
                              boot_id=cfg.get('commissioning', {}).get('boot_id'),
                              feedback_limit_tolerance_deg=servo['feedback_limit_tolerance_deg'])
        if (servo.get('absolute_reference') is not None and
                type(axis['zdt_fd8_snapshot'].get('startup_encoder_offset_counts')) is not int):
            raise ValueError('Saved absolute origin requires current encoder/position offset readback')
    elif axis.get('zdt_verified_snapshot') is not None:
        verified_snapshot(axis, servo['baudrate'])
    elif not cfg['dry_run']:
        raise ValueError('ZDT firmware/options/scaling snapshot is not commissioned')
    # Pump commissioning is separate; this backend must not implicitly arm it.
    return cfg


def joint_to_motor_degrees(angle, axis, mode):
    angle = number(angle, 'joint angle')
    if not axis['min_angle_deg'] <= angle <= axis['max_angle_deg'] or axis['reference_counts'] is None:
        raise ValueError('ZDT joint limits/reference unavailable')
    return (zdt.position_degrees(axis['reference_counts'], mode) +
            (angle-axis['reference_angle_deg']) * axis['direction'] * axis['gear_ratio'])


def motor_to_joint_angle(counts, axis, mode):
    return (axis['reference_angle_deg'] +
            (zdt.position_degrees(counts, mode)-zdt.position_degrees(axis['reference_counts'], mode)) /
            (axis['direction']*axis['gear_ratio']))


class MixedActuatorIO(PD42ActuatorIO):
    backend = 'mixed_serial'

    def _validate_config(self, config):
        cfg = validate(config)
        axis = cfg['servo']['pan']
        snap = axis.get('zdt_fd8_snapshot')
        self._fd8 = fd8.FD8Stepper(axis, snap,
            feedback_limit_tolerance_deg=cfg['servo']['feedback_limit_tolerance_deg'],
            position_tolerance_deg=cfg['servo']['position_tolerance_deg']) if snap is not None else None
        self._fd8_tilt_generation = None
        self._fd8_requested_pan = None
        self._fd8_requested_target = None
        return cfg

    def set_gimbal(self, pan_deg, tilt_deg, *, speed_deg_s=20., lease_s=.3):
        if self._fd8 is not None:
            axis = self.servo['pan']
            requested = number(pan_deg, 'pan')
            if not axis['min_angle_deg'] <= requested <= axis['max_angle_deg']:
                raise ValueError('pan angle exceeds measured limits')
            pan_deg = fd8.quantized_target(requested, axis, axis['zdt_fd8_snapshot'],
                                           self.servo['position_tolerance_deg'])
        if self._fd8 is not None:
            with self._mutex:
                super().set_gimbal(pan_deg, tilt_deg, speed_deg_s=speed_deg_s, lease_s=lease_s)
                self._fd8_requested_pan = requested
                self._fd8_requested_target = [requested, float(tilt_deg)]
        else:
            super().set_gimbal(pan_deg, tilt_deg, speed_deg_s=speed_deg_s, lease_s=lease_s)

    def _make_transport(self, axis):
        if axis['protocol'] == 'zdt_xs':
            return ZDTSerialTransport(axis['port'], self.servo['baudrate'], self.servo['timeout_s'])
        return super()._make_transport(axis)

    def _exchange(self, axis, packet):
        if axis['protocol'] == 'zdt_xs':
            return self._ports[axis['port']].exchange(packet).require_success().value
        return super()._exchange(axis, packet)

    def _zdt_read(self, axis, command):
        return self._exchange(axis, zdt.read_request(axis['address'], command))

    def _read_axis(self, name, axis):
        if axis['protocol'] != 'zdt_xs':
            # 0x31 returns position, speed and enable from one driver snapshot.
            # Keep the separate run-state read: it also reports undervoltage
            # and overload, which cannot be inferred from an arrived bit.
            values = self._exchange(axis, pd42.read_request(axis['address'], pd42.Command.READ_SYSTEM))
            status = self._exchange(axis, pd42.read_request(axis['address'], pd42.Command.READ_STATUS))
            if (status not in (pd42.RunState.STOPPED, pd42.RunState.COMPLETED, pd42.RunState.RUNNING)
                    or values['stalled'] or values['group_address']):
                raise RuntimeError(name + ' driver fault: ' + str(status))
            angle = counts_to_angle(values['position_counts'], axis)
            require_measured_limits(name, values['position_counts'], angle, axis,
                                    self.servo['feedback_limit_tolerance_deg'])
            return dict(angle=angle, counts=values['position_counts'], rpm=values['speed_rpm'],
                        enabled=values['enabled'], status=int(status))
        if self._fd8 is not None:
            return self._read_fd8_axis(axis)
        expected, params = verified_snapshot(axis, self.servo['baudrate'])
        version = self._zdt_read(axis, zdt.Command.READ_VERSION)
        cfg = self._zdt_read(axis, zdt.Command.READ_CONFIG)
        options = zdt.normalize_options(version, cfg, self._zdt_read(axis, zdt.Command.READ_OPTIONS))
        # Settings can change through the driver's menu while the node runs.
        actual = dict(version=version, config=cfg, options=options)
        if actual != expected:
            raise RuntimeError('ZDT configuration changed/unverified; stop and recommission reference')
        counts = self._zdt_read(axis, zdt.Command.READ_POSITION)
        rpm = zdt.speed_rpm(self._zdt_read(axis, zdt.Command.READ_SPEED), params['mode'])
        state = self._zdt_read(axis, zdt.Command.READ_STATE)
        if any(state.get(key) for key in ('stalled', 'stall_protection', 'overtemperature',
                                         'overcurrent', 'left_limit', 'right_limit', 'power_cycle_marker')):
            raise RuntimeError('ZDT driver fault or physical limit active')
        angle = motor_to_joint_angle(counts, axis, params['mode'])
        require_measured_limits('ZDT', counts, angle, axis, self.servo['feedback_limit_tolerance_deg'])
        return dict(angle=angle, counts=counts, rpm=rpm, enabled=state['enabled'],
                    status=2 if rpm else 1 if state['arrived'] else 0)

    def _position_packet(self, axis, angle, rpm):
        if axis['protocol'] != 'zdt_xs':
            return super()._position_packet(axis, angle, rpm)
        if self._fd8 is not None:
            # Direction depends on live retained target. No generic position
            # packet may be constructed or queued for the bounded FD8 path.
            return None
        _, params = verified_snapshot(axis, self.servo['baudrate'])
        target = joint_to_motor_degrees(angle, axis, params['mode'])
        quantum = (params['motor_step_deg']/params['microsteps'] if params['mode'] == 'emm'
                   else .01 if params['angle_div10'] else .1)
        quantized = math.copysign(math.floor(abs(target)/quantum+.5)*quantum, target)
        actual_joint = angle+(quantized-target)/(axis['direction']*axis['gear_ratio'])
        if (abs(actual_joint-angle) > self.servo['position_tolerance_deg'] or
                not axis['min_angle_deg'] <= actual_joint <= axis['max_angle_deg']):
            raise ValueError('ZDT position quantization exceeds limits/tolerance')
        return zdt.position_request(axis['address'], target, rpm, **params,
                                    motion_mode=1, acceleration=axis['acceleration'])

    def _enable_axis(self, axis, enabled):
        if axis['protocol'] != 'zdt_xs':
            return super()._enable_axis(axis, enabled)
        # Disabling is independent from configuration validity, so it remains
        # possible after a readback fault. An ACK alone is not proof of state.
        self._exchange(axis, zdt.enable_request(axis['address'], enabled))
        state = self._zdt_read(axis, zdt.Command.READ_STATE)
        if state['enabled'] is not enabled:
            raise RuntimeError('ZDT enable/disable was not confirmed by measured state')

    def _pre_enable_axis(self, name, axis, measured):
        if axis['protocol'] != 'zdt_xs':
            return super()._pre_enable_axis(name, axis, measured)
        if self._fd8 is not None:
            if self._fd8.pending is not None or self._fd8.stable < 3:
                raise RuntimeError('FD8 requires three stable reference samples before enable')
            fresh = self._read_fd8_axis(axis)
            from .zdt_vendor_bench import settled
            if fresh['enabled'] or not settled(fresh['fd8_sample'], self._fd8.expected):
                raise RuntimeError('FD8 retained target differs before enable; no automatic rebase')
            return
        _, params = verified_snapshot(axis, self.servo['baudrate'])
        before = self._zdt_read(axis, zdt.Command.READ_POSITION)
        target = self._zdt_read(axis, zdt.Command.READ_TARGET_POSITION)
        realtime = self._zdt_read(axis, zdt.Command.READ_REALTIME_TARGET_POSITION)
        speed = self._zdt_read(axis, zdt.Command.READ_SPEED)
        state = self._zdt_read(axis, zdt.Command.READ_STATE)
        after = self._zdt_read(axis, zdt.Command.READ_POSITION)
        if (any(type(value) is not int for value in (before, target, realtime, after)) or
                not isinstance(state, dict) or state.get('enabled') is not False or
                type(state.get('raw')) is not int or state['raw'] & ~2 or
                zdt.speed_rpm(speed, params['mode']) != 0):
            raise RuntimeError(name + ' ZDT retained target / disabled feedback is unverified')
        def delta(a, b):
            return abs(zdt.position_degrees(a-b, params['mode'])) / axis['gear_ratio']
        tolerance = min(.1, self.servo['position_tolerance_deg'])
        if any(delta(goal, pos) > tolerance for goal in (target, realtime) for pos in (before, after)):
            raise RuntimeError(name + ' retained target differs from actual position; enable refused, no automatic rebase')
        if delta(target, realtime) > min(.03, tolerance):
            raise RuntimeError(name + ' ZDT retained and realtime targets disagree')
        span = max(before, after, measured['counts']) - min(before, after, measured['counts'])
        if delta(span, 0) > min(.03, tolerance):
            raise RuntimeError(name + ' moved during pre-enable readback')

    def _poll_interval_s(self):
        # A complete target needs three independent settled observations.
        # During a live goal sample at up to 20 Hz for prompt arrival/STOP
        # handling, then return to the idle cadence. The target is sent once.
        interval = super()._poll_interval_s()
        if self._fd8 is not None:
            with self._mutex:
                goal = self._goal
                if (goal is not None and time.monotonic() < goal['deadline'] and
                        self._accepted_generation != goal['generation']):
                    return min(interval, .05)
        return interval

    def feedback(self):
        result = super().feedback()
        result.update(axis_protocols=[self.servo[name]['protocol'] for name in ('pan', 'tilt')],
                      ownership='production_serial_owner' if not self.dry_run else 'dry_run_no_uart',
                      commissioning_required=self.dry_run or not self.servo['reference_confirmed_this_start'],
                      requested_target_deg=result.get('target_deg'))
        if self._fd8 is not None:
            with self._mutex:
                result.update(pan_motion_profile=fd8.PROFILE, pan_step_deg=fd8.PULSE_DEG,
                              pan_command_mode='complete_remaining_target',
                              pan_target_packet_hardware_verified=False,
                              feedback_poll_target_hz=1. / self._poll_interval_s(),
                              pan_encoded_speed=1, pan_arbitrary_position_protocol_verified=False,
                              pan_undocumented_options_raw='04 26', pan_undocumented_bit_indices=[10],
                              pan_requested_deg=self._fd8_requested_pan,
                              requested_target_deg=(list(self._fd8_requested_target)
                                  if self._goal and self._fd8_requested_target else None),
                              pan_steps_written=self._fd8.steps,
                              pan_step_pending=self._fd8.pending is not None,
                              pan_stable_samples=self._fd8.stable,
                              pan_feedback_diagnostic=deepcopy(self._fd8.diagnostic()))
                if self._fd8.pending is not None or self._fd8.stable < 3:
                    result['settled_measured'] = False
                    result['holding_measured'] = False
        return result

    def feedback_with_reference_status(self):
        """One coherent read-only snapshot for the live owner's status view.

        The ordinary reference_handoff_reason/detach_for_reference path still
        performs its independent final check. This display/admission preview
        uses the same policy without recursively constructing FD8 diagnostics
        twice. Holding the mutex keeps raw axes and feedback in one version.
        """
        from .gimbal_reference_policy import recoverable_reference_fault
        with self._mutex:
            result=self.feedback()
            if self.closed or self.dry_run or self._reference_handoff:
                reason='actuator_not_available_for_reference'
            elif self.config['pump']['enabled']:
                reason='local_pump_not_supported_during_reference'
            elif self._fault:
                reason='' if recoverable_reference_fault(self._fault) else self._fault
            elif self._goal is not None or (self._active and self._holding is None):
                reason='another_gimbal_owner_active'
            elif (not result.get('feedback_fresh') or len(self._feedback or [])!=2 or
                    any(axis['rpm']!=0 or axis['status'] not in (0,1) for axis in self._feedback)):
                reason='both_axes_must_be_measured_stationary'
            else:
                reason=''
            return result,reason

    def _verify_fd8_configuration(self, axis):
        version = self._zdt_read(axis, zdt.Command.READ_VERSION)
        reply = self._ports[axis['port']].exchange(zdt.read_request(axis['address'], zdt.Command.READ_CONFIG))
        reply.require_success()
        options = self._zdt_read(axis, zdt.Command.READ_OPTIONS)
        from .zdt_vendor_bench import matches
        if not matches(version, reply.data, options):
            raise RuntimeError('FD8 exact live firmware/config/options changed; recommission required')

    def _read_fd8_axis(self, axis):
        self._verify_fd8_configuration(axis)
        return self._read_fd8_dynamic_axis(axis)

    def _read_fd8_dynamic_axis(self, axis):
        # Metadata is checked separately on every cycle. The position age
        # starts BEFORE the first encoder query, never at read completion.
        started = time.monotonic()
        before = self._zdt_read(axis, zdt.Command.READ_POSITION)
        retained = self._zdt_read(axis, zdt.Command.READ_TARGET_POSITION)
        realtime = self._zdt_read(axis, zdt.Command.READ_REALTIME_TARGET_POSITION)
        error = self._zdt_read(axis, zdt.Command.READ_POSITION_ERROR)
        speed = self._zdt_read(axis, zdt.Command.READ_SPEED)
        state = self._zdt_read(axis, zdt.Command.READ_STATE)
        encoder_offset = axis['zdt_fd8_snapshot'].get('startup_encoder_offset_counts')
        absolute = (self._zdt_read(axis, zdt.Command.READ_ENCODER)
                    if encoder_offset is not None else None)
        after = self._zdt_read(axis, zdt.Command.READ_POSITION)
        if time.monotonic()-started > self.servo['feedback_timeout_s']:
            raise RuntimeError('FD8 feedback transaction exceeded real-time budget')
        if absolute is not None:
            # 0x31 is a single-turn encoder; 0x36 is the running coordinate.
            # Their startup offset is fixed, never learned again during motion.
            # Bracket the absolute read by actual position reads so ordinary
            # serial latency while moving cannot look like a sign mismatch.
            counts_tolerance = self.servo['feedback_limit_tolerance_deg']/fd8.COUNT_DEG
            low = min(before, after)-counts_tolerance
            high = max(before, after)+counts_tolerance
            mapped = absolute+encoder_offset
            first = math.ceil((low-mapped)/65536)
            last = math.floor((high-mapped)/65536)
            if first != last:
                raise RuntimeError('FD8 absolute encoder/runtime position mismatch: '
                    f'absolute_counts={absolute}, position_before_counts={before}, '
                    f'position_counts={after}, startup_encoder_offset_counts={encoder_offset}')
        if type(state.get('raw')) is not int or state['raw'] & ~3:
            raise RuntimeError('FD8 driver fault/limit/reset state active')
        if self._fd8.pending is not None and state.get('enabled') is not True:
            raise RuntimeError('FD8 enable lost during a step')
        sample = dict(position_before_counts=before, position_counts=after,
                      retained_target_counts=retained, realtime_target_counts=realtime,
                      position_error_deg=zdt.error_degrees(error, 'emm'),
                      speed_rpm=zdt.speed_rpm(speed, 'emm'), enabled=state['enabled'],
                      arrived=state['arrived'])
        if absolute is not None:
            sample.update(absolute_counts=absolute, startup_encoder_offset_counts=encoder_offset)
        angle = require_measured_limits('FD8', after, fd8.angle_for_counts(after, axis), axis,
                                       self.servo['feedback_limit_tolerance_deg'])
        return dict(angle=angle, counts=after, rpm=sample['speed_rpm'],
                    enabled=state['enabled'], status=2 if speed else 1 if state['arrived'] else 0,
                    fd8_sample=sample)

    def _cycle(self):
        if self._fd8 is None:
            return super()._cycle()
        # Check lease BEFORE a potentially slow serial sample, then again before
        # each write. An old pending relative move is never replayed after stop.
        with self._mutex:
            if self._reference_handoff:
                return
            initial = deepcopy(self._goal)
            holding = self._holding is not None
        if (initial is None or time.monotonic() >= initial['deadline']) and self._active and not holding:
            self._disable_axes()
            if self._fd8.pending is not None:
                raise RuntimeError('FD8 step interrupted by lost lease; recommission before retry')
        if self._fd8.pending is not None and time.monotonic() >= self._fd8.pending['deadline']:
            raise RuntimeError('FD8 measured step arrival timeout; no resend')
        # Firmware/config/options are still checked before every cycle and
        # every possible motion write. Their latency is not encoder age.
        self._verify_fd8_configuration(self.servo['pan'])
        captured = time.monotonic()
        axes = [self._read_fd8_dynamic_axis(self.servo['pan']),
                self._read_axis('tilt', self.servo['tilt'])]
        if time.monotonic()-captured > self.servo['feedback_timeout_s']:
            raise RuntimeError('Mixed FD8 feedback exceeded real-time budget')
        with self._mutex:
            if self._reference_handoff:
                return
        self._fd8.observe(axes[0]['fd8_sample'], time.monotonic())
        with self._mutex:
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
        if self._fd8.pending is None and self._fd8.stable < 3:
            return
        # Every disabled axis is preflighted before either motor receives writes.
        for i, name in enumerate(('pan', 'tilt')):
            if not axes[i]['enabled']:
                if not self.servo['enable_on_command']:
                    raise RuntimeError(name+' disabled; enable-on-command is not configured')
                self._pre_enable_axis(name, self.servo[name], axes[i])
                if not self._motion_gate(goal, captured):
                    return
        self._active = True
        enabled_pan = False
        for i, name in enumerate(('pan', 'tilt')):
            if not self._motion_gate(goal, captured):
                return
            if not axes[i]['enabled']:
                self._enable_axis(self.servo[name], True)
                enabled_pan = enabled_pan or name == 'pan'
        if not self._motion_gate(goal, captured):
            return
        if enabled_pan:
            # Recheck position, both targets and following error after enable.
            # The enable ACK/state must not substitute for stable feedback.
            self._fd8.stable = 0
            self._fd8.stable_counts = []
            return
        if self._fd8_tilt_generation != goal['generation']:
            self._exchange(self.servo['tilt'], goal['packets'][1])
            self._fd8_tilt_generation = goal['generation']
        if not self._motion_gate(goal, captured):
            return
        step = self._fd8.plan_step(goal['target'][0], self.servo['position_tolerance_deg'])
        if step is not None:
            packet = self._fd8.mark_written(*step, time.monotonic())
            self._exchange(self.servo['pan'], packet)
            if not self._motion_gate(goal, captured):
                return
        current = fd8.angle_for_counts(self._fd8.expected, self.servo['pan'])
        done = (self._fd8.pending is None and self._fd8.stable >= 3 and
                abs(current-goal['target'][0]) <= self.servo['position_tolerance_deg'])
        if done:
            with self._mutex:
                if (self._goal is not None and self._goal['generation'] == goal['generation'] and
                        self._accepted_generation != goal['generation']):
                    self._accepted_generation = goal['generation']
                    self._accepted_at = time.monotonic()
