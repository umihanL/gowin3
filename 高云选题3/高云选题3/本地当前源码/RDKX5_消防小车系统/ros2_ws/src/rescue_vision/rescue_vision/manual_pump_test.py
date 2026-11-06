"""An operator-confirmed finite B13 test, independent of automatic fire aiming.

No actuator I/O is performed here. The caller owns the existing FPGA publisher,
and binds the actual transport command ids before supplying readback. Requests
and ACKs are never treated as evidence that an output has turned on or off.
"""
import math
from collections import deque


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class ManualPumpTest:
    def __init__(self, enabled=False, max_duration_ms=1000):
        if (type(enabled) is not bool or type(max_duration_ms) is not int or
                not 1 <= max_duration_ms <= 1000 or (enabled and max_duration_ms < 100)):
            raise ValueError('manual_pump_invalid_configuration')
        self.enabled, self.max_duration_ms = enabled, max_duration_ms
        self.phase, self.reason = 'idle', 'not_requested'
        self.request_id = self.last_request_id = self.last_request_action = None
        self.last_result, self.last_request_reason = 'none', 'not_requested'
        self.request_stamp = None
        self.seen = deque(maxlen=512)
        self.cancel_before = -math.inf
        self.cooldown_until = 0.
        self.duration_ms = 400
        self.started_at = 0.
        self.on_id = self.off_id = None
        self.on_stamp = self.off_stamp = None
        self.saw_pump_on = False
        self.confirmed_pump_off = False
        self.confirmed_pump_off_stamp = None
        self.stop_request_id = None
        self.stop_pending = False
        self.stop_confirmed = False
        self.stop_confirmed_request_id = self.stop_confirmed_stamp = None
        self.interruption_reason = ''
        self.off_unconfirmed = False
        self.finish_result = None
        self.finish_reason = None
        self.preparation_off_id = self.preparation_off_stamp = None
        self.preparation_deadline = None
        self.preparation_deadline_mono = None
        self.fixed_nozzle_confirmed = False

    @property
    def active(self):
        return self.phase in ('preparing_off', 'waiting_on', 'spraying', 'waiting_off')

    @property
    def blocking(self):
        return self.active or self.off_unconfirmed

    @staticmethod
    def context_reason(context):
        if context.get('estop'):
            return 'manual_pump_fault_or_estop'
        # Missing FPGA telemetry is not positive evidence of vehicle motion.
        # Surface the failed link first, rather than interpreting the mission's
        # conservative vehicle_stopped=False fallback as "the car is moving".
        if context.get('fpga_recovering') is True:
            return 'manual_pump_fpga_reconnecting'
        if context.get('fpga_feedback_fresh') is False:
            return 'manual_pump_fpga_feedback_unavailable'
        if context.get('fpga_physical_host_ready') is False:
            return 'manual_pump_physical_control_page_required'
        if context.get('fpga_link_ok') is False:
            return 'manual_pump_fpga_link_required'
        if context.get('fault'):
            return 'manual_pump_fault_or_estop'
        if context.get('gimbal_known_moving') is True:
            return 'manual_pump_gimbal_moving'
        if context.get('mission_fresh') is not True:
            return 'manual_pump_mission_stale'
        if context.get('vehicle_stopped') is not True:
            return 'manual_pump_vehicle_not_stopped'
        if context.get('idle_mode') is not True:
            return 'manual_pump_idle_observation_required'
        if context.get('extension_idle') is not True:
            return 'manual_pump_fpga_idle_required'
        if context.get('other_owner'):
            return 'manual_pump_other_actuator_active'
        return ''

    def readiness(self, now, feedback, context):
        if not self.enabled:
            return False, 'manual_pump_disabled'
        if self.blocking:
            return False, 'manual_pump_waiting_off' if self.off_unconfirmed else 'manual_pump_running'
        reason = self.context_reason(context)
        if reason:
            return False, reason
        if now < self.cooldown_until:
            return False, 'manual_pump_cooldown'
        if feedback.get('pump_feedback_fresh') is not True or feedback.get('pump_on') is not False:
            return False, 'manual_pump_fresh_off_required'
        if feedback.get('pump_ready') is not True:
            # STOP deliberately latches B13 OFF. Only a NEW operator request
            # may prepare another pulse by sending its own explicit OFF.
            # Link, page, motion or unknown faults are never cleared here.
            stopped_lock = (feedback.get('pump_lockout') is True and
                feedback.get('pump_reason_code') == 5 and
                all(feedback.get(k) is True for k in
                    ('pump_scope', 'pump_link_ok', 'pump_wheels_still')))
            if not stopped_lock:
                return False, 'manual_pump_fpga_not_ready'
        return True, 'ready'

    def request(self, value, *, wall_now, now, feedback, context):
        self.last_request_id = value.get('command_id')
        self.last_request_action = value.get('action')
        self.request_stamp = value.get('issued_at')
        if value.get('action') == 'stop':
            self.cancel_before = max(self.cancel_before, wall_now)
            # A new STOP must obtain its own LOW readback. Historical LOW from
            # the previous pulse/STOP cannot acknowledge this request.
            self.stop_request_id = self.last_request_id
            self.stop_pending = True
            self.stop_confirmed = self.confirmed_pump_off = False
            self.stop_confirmed_request_id = self.stop_confirmed_stamp = None
            self.confirmed_pump_off_stamp = None
            self.off_id = self.off_stamp = None
            if not self.enabled and not self.blocking:
                self.stop_pending = False
                self.phase, self.reason = 'cancelled', 'manual_pump_disabled_no_active_test'
                self.last_result, self.last_request_reason = 'cancelled', self.reason
                return 'off'
            self.last_result = 'accepted'
            self.last_request_reason = 'manual_pump_operator_stop'
            self.cancel(now, 'manual_pump_operator_stop', failed=False, force=True)
            return 'off'
        try:
            cid, stamp, ttl = (value.get(k) for k in ('command_id', 'issued_at', 'ttl_s'))
            if 'fixed_nozzle_confirmed' in value and type(value['fixed_nozzle_confirmed']) is not bool:
                raise ValueError('manual_pump_fixed_nozzle_confirmation_must_be_boolean')
            if value.get('action') != 'start' or value.get('operator_confirmed') is not True:
                raise ValueError('manual_pump_operator_confirmation_required')
            if not isinstance(cid, str) or not 1 <= len(cid) <= 100 or cid in self.seen:
                raise ValueError('manual_pump_unique_request_required')
            self.seen.append(cid)
            if (not finite(stamp) or not finite(ttl) or not 0 < ttl <= 2 or
                    not 0 <= wall_now-stamp <= ttl or stamp <= self.cancel_before):
                raise ValueError('manual_pump_stale_request')
            duration = value.get('duration_ms', 400)
            if type(duration) is not int or not 100 <= duration <= self.max_duration_ms:
                raise ValueError('manual_pump_duration_outside_limits')
            ready, reason = self.readiness(now, feedback, context)
            if not ready:
                raise ValueError(reason)
            self.request_id = cid
            self.fixed_nozzle_confirmed = value.get('fixed_nozzle_confirmed') is True
            self.duration_ms, self.started_at = duration, now
            self.saw_pump_on = self.confirmed_pump_off = False
            self.confirmed_pump_off_stamp = None
            self.stop_request_id = None
            self.stop_pending = self.stop_confirmed = False
            self.stop_confirmed_request_id = self.stop_confirmed_stamp = None
            self.interruption_reason = ''
            self.on_id = self.off_id = self.on_stamp = self.off_stamp = None
            self.preparation_off_id = self.preparation_off_stamp = None
            self.preparation_deadline = min(stamp+ttl, wall_now+.8)
            self.preparation_deadline_mono = now+self.preparation_deadline-wall_now
            self.off_unconfirmed = True
            self.finish_result = self.finish_reason = None
            self.phase, self.reason = 'preparing_off', 'manual_pump_preparing_off'
            self.last_result, self.last_request_reason = 'accepted', self.reason
            return 'off'
        except (ValueError, TypeError) as exc:
            self.last_result, self.last_request_reason = 'rejected', str(exc)
            raise

    def submitted(self, command_id, stamp, *, enabled):
        if not isinstance(command_id, str) or not finite(stamp):
            raise ValueError('manual_pump_transport_identity_required')
        if enabled:
            if self.phase != 'waiting_on':
                raise ValueError('manual_pump_on_authority_expired')
            self.on_id, self.on_stamp = command_id, stamp
        elif self.phase == 'preparing_off':
            self.preparation_off_id, self.preparation_off_stamp = command_id, stamp
        else:
            if command_id != self.off_id or stamp != self.off_stamp:
                self.confirmed_pump_off = self.stop_confirmed = False
                self.confirmed_pump_off_stamp = None
                self.stop_confirmed_request_id = self.stop_confirmed_stamp = None
            self.off_id, self.off_stamp = command_id, stamp

    def cancel(self, now, reason, *, failed=True, force=False):
        if not self.blocking and not force:
            return None
        self.preparation_deadline = None
        self.preparation_deadline_mono = None
        self.fixed_nozzle_confirmed = False
        if failed and not self.interruption_reason:
            self.interruption_reason = reason
        if self.stop_pending:
            # A motor fault still denies a new pulse, but cannot turn an
            # explicit OFF acknowledgement into a failed motion command.
            self.finish_result, self.finish_reason = 'cancelled', 'manual_pump_operator_stop'
        elif self.finish_result != 'failed':
            self.finish_result, self.finish_reason = ('failed' if failed else 'cancelled'), reason
        self.phase, self.reason, self.off_unconfirmed = 'waiting_off', self.finish_reason, True
        self.cooldown_until = max(self.cooldown_until, now+1.)
        return 'off' if self.off_id is None or force else None

    def _finish(self, now, feedback_stamp):
        self.fixed_nozzle_confirmed = False
        self.off_unconfirmed = False
        self.confirmed_pump_off = True
        self.confirmed_pump_off_stamp = feedback_stamp
        self.cooldown_until = max(self.cooldown_until, now+1.)
        if self.stop_pending:
            self.stop_pending = False
            self.stop_confirmed = True
            self.stop_confirmed_request_id = self.stop_request_id
            self.stop_confirmed_stamp = feedback_stamp
            self.phase, self.reason = 'cancelled', 'manual_pump_operator_stop_confirmed'
        else:
            self.phase = self.finish_result or ('completed' if self.saw_pump_on else 'failed')
            self.reason = self.finish_reason or ('manual_pump_high_then_low_confirmed'
                if self.saw_pump_on else 'manual_pump_high_not_observed_no_retry')
        # A rejected overlapping request retains its own receipt; run identity
        # and terminal phase separately describe the original request.
        if self.last_result != 'rejected':
            self.last_result, self.last_request_reason = self.phase, self.reason

    def update(self, *, now, wall_now, feedback, context):
        if not self.blocking:
            return None
        recent = feedback.get('pump_feedback_fresh') is True
        stamp = feedback.get('pump_feedback_stamp')
        cid = feedback.get('pump_feedback_command_id')
        reason = self.context_reason(context)
        if reason and not self.interruption_reason:
            self.interruption_reason = reason
        if self.phase == 'preparing_off':
            if reason:
                return self.cancel(now, reason)
            if (self.preparation_deadline is None or wall_now >= self.preparation_deadline or
                    self.preparation_deadline_mono is None or now >= self.preparation_deadline_mono):
                return self.cancel(now, 'manual_pump_prepare_timeout')
            if self.preparation_off_id is None:
                return self.cancel(now, 'manual_pump_prepare_not_submitted')
            if recent and finite(stamp) and stamp >= self.preparation_off_stamp:
                if cid != self.preparation_off_id:
                    return self.cancel(now, 'manual_pump_prepare_interrupted')
                if feedback.get('pump_on') is not False:
                    return self.cancel(now, 'manual_pump_prepare_interrupted')
                if feedback.get('pump_ready') is True:
                    # The requested duration starts only after this fresh OFF
                    # generation has been read back. This is the sole ON edge.
                    self.preparation_deadline = None
                    self.preparation_deadline_mono = None
                    self.started_at = now
                    self.phase, self.reason = 'waiting_on', 'manual_pump_waiting_high_readback'
                    return 'on'
            return None
        if (self.off_id and recent and feedback.get('pump_on') is False and
                finite(stamp) and stamp >= self.off_stamp and cid == self.off_id):
            self._finish(now, stamp)
            return None
        if self.off_id is not None:
            # Once OFF was sent, only its matched readback releases ownership.
            # Persistent faults must neither resend it nor bypass the timeout.
            if wall_now-self.off_stamp > .8:
                self.phase, self.reason = 'failed', 'manual_pump_off_unconfirmed'
                self.finish_result, self.finish_reason = self.phase, self.reason
                if self.last_result != 'rejected':
                    self.last_result, self.last_request_reason = 'failed', self.reason
            return None  # Keep the output reservation until fresh matching LOW.
        if reason:
            return self.cancel(now, reason)
        if self.on_id is None:
            return self.cancel(now, 'manual_pump_on_not_submitted')
        if recent and finite(stamp) and stamp >= self.on_stamp and cid == self.on_id:
            if feedback.get('pump_on') is True:
                self.saw_pump_on = True
                self.phase, self.reason = 'spraying', 'manual_pump_high_observed'
            elif self.saw_pump_on:
                self.phase, self.reason = 'waiting_off', 'manual_pump_confirming_off'
                return 'off'
        if not recent and feedback.get('pump_readback_pending') is not True:
            return self.cancel(now, 'manual_pump_feedback_lost')
        if now-self.started_at >= self.duration_ms/1000.:
            self.phase, self.reason = 'waiting_off', 'manual_pump_duration_finished'
            return 'off'
        return None

    def status(self, now, feedback, context):
        ready, reason = self.readiness(now, feedback, context)
        return dict(active=self.active, blocking=self.blocking, enabled=self.enabled,
            phase=self.phase, reason=self.reason, can_start=ready, ready_reason=reason,
            request_id=self.request_id, last_request_id=self.last_request_id,
            last_request_action=self.last_request_action, request_stamp=self.request_stamp,
            last_result=self.last_result, last_request_reason=self.last_request_reason,
            duration_ms=self.duration_ms, max_duration_ms=self.max_duration_ms,
            cooldown_remaining_s=max(0., self.cooldown_until-now),
            pump_on=feedback.get('pump_on'), actual_pump_on=feedback.get('pump_on'), feedback_fresh=feedback.get('pump_feedback_fresh') is True,
            saw_pump_on=self.saw_pump_on, confirmed_pump_off=self.confirmed_pump_off,
            confirmed_pump_off_stamp=self.confirmed_pump_off_stamp,
            off_command_id=self.off_id, stop_request_id=self.stop_request_id,
            stop_pending=self.stop_pending, stop_confirmed=self.stop_confirmed,
            stop_confirmed_request_id=self.stop_confirmed_request_id,
            stop_confirmed_stamp=self.stop_confirmed_stamp,
            interruption_reason=self.interruption_reason,
            context_reason=self.context_reason(context), actuator_error=context.get('fault') or '',
            estop=context.get('estop'),
            fpga_recovering=context.get('fpga_recovering', False),
            fpga_feedback_fresh=context.get('fpga_feedback_fresh'),
            fpga_error=context.get('fpga_error') or '',
            gimbal_reference_fault_ignored=context.get('gimbal_reference_fault_ignored', False),
            gimbal_fault_ignored_for_manual_pump=context.get('gimbal_fault_ignored_for_manual_pump', False),
            gimbal_fault_diagnostic=context.get('gimbal_fault_diagnostic') or '',
            gimbal_stationary_confirmed=context.get('gimbal_stationary_confirmed', False),
            gimbal_stationary_reason=context.get('gimbal_stationary_reason') or '',
            gimbal_recovery_history_retired=context.get('gimbal_recovery_history_retired', False),
            gimbal_recovery_history=context.get('gimbal_recovery_history') or '',
            fpga_physical_host_ready=context.get('fpga_physical_host_ready'),
            fpga_link_ok=context.get('fpga_link_ok'),
            gimbal_known_moving=context.get('gimbal_known_moving', False),
            fixed_nozzle_confirmed=self.fixed_nozzle_confirmed,
            gimbal_communication_exempt=context.get('gimbal_communication_exempt',False),
            fixed_nozzle_motion_reason=context.get('fixed_nozzle_motion_reason',''),
            preparation_active=self.phase == 'preparing_off',
            preparation_remaining_s=(max(0., self.preparation_deadline_mono-now)
                if self.preparation_deadline_mono is not None else 0.),
            preparation_off_command_id=self.preparation_off_id,
            source='manual_fixed_nozzle_test', calibration_required=False,
            water_flow_verified=False, physical_hit_verified=False)
