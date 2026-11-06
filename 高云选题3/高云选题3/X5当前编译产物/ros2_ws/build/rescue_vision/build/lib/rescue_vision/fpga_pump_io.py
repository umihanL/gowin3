"""FPGA B13 finite-pulse requests and readback, with no UART/GPIO ownership.

Only extension_task opens the FPGA UART. A submitted request is never used as
output feedback, and a B13 output level never proves water flow or impact.
"""
from copy import deepcopy
import math
import time
import uuid


def validate_pump(config):
    value = deepcopy(config)
    if value.get('backend') != 'fpga_uart' or value.get('pin') != 'B13':
        raise ValueError('FPGA pump requires backend=fpga_uart and pin=B13')
    if type(value.get('enabled')) is not bool or value.get('active_high') is not True:
        raise ValueError('FPGA pump enabled must be boolean and active_high must be true')
    maximum = value.get('max_duration_ms', 1000)
    timeout = value.get('feedback_timeout_s', .6)
    if type(maximum) is not int or not 1 <= maximum <= 1000:
        raise ValueError('FPGA pump max_duration_ms must be 1..1000')
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not .1 <= timeout <= .6:
        raise ValueError('FPGA pump feedback_timeout_s must be 0.1..0.6')
    if any(value.get(key) is not None for key in ('gpio_chip', 'line_offset', 'active_low')):
        raise ValueError('FPGA pump configuration must not assign X5 GPIO')
    value.update(max_duration_ms=maximum, feedback_timeout_s=float(timeout))
    return value


class FPGAPumpTransport:
    """Single finite ON request per edge; OFF is always allowed and prioritized."""
    def __init__(self, config, emit=None, *, dry_run=True, wall_clock=time.time):
        self.config = validate_pump(config)
        self.emit = emit
        self.dry_run = dry_run
        self.wall_clock = wall_clock
        self.requested_on = False
        self._last_enabled = None
        self._last_sent = -math.inf
        self.last_command_id = None
        self._feedback = {}
        self._last_state_stamp = -math.inf
        self.closed = False
        if not dry_run and self.config['enabled'] and not callable(emit):
            raise ValueError('Real FPGA pump requires the ROS request publisher')

    def observe(self, value):
        if not isinstance(value, dict):
            self._feedback = {}
            return
        # Control-generation changes intentionally clear feedback_stamp. Keep
        # their pending/ACK metadata, while never treating it as readback.
        stamp = value.get('stamp')
        if type(stamp) not in (int, float) or not math.isfinite(stamp):
            self._feedback = {}
            return
        if stamp < self._last_state_stamp:
            return
        self._last_state_stamp = stamp
        self._feedback = deepcopy(value)

    def snapshot(self):
        value = self._feedback
        stamp = value.get('feedback_stamp', value.get('stamp'))
        delay = self.wall_clock() - stamp if type(stamp) in (int, float) else math.inf
        # Output permission and readback provenance are independent. An
        # uncommissioned gimbal must not hide genuine FPGA B13 feedback from
        # the vehicle's pump-OFF interlock. A local simulated request never
        # contributes to this value; only verified, fresh UART readback does.
        fresh = (value.get('feedback_fresh') is True and
                 value.get('real_uart') is True and value.get('protocol_ready') is True and
                 value.get('pin') == 'B13' and value.get('active_high') is True and
                 0 <= delay <= self.config['feedback_timeout_s'] and
                 type(value.get('pump_on')) is bool and not value.get('error'))
        actual = value.get('pump_on') if fresh else None
        ready = bool(not self.dry_run and self.config['enabled'] and
                     fresh and actual is False and value.get('pump_ready') is True)
        pending_until = value.get('pending_until')
        published = value.get('stamp')
        result = value.get('last_result') or {}
        if not isinstance(result, dict):
            result = dict(accepted=False, command_id=self.last_command_id)
        explicit_failure = (result.get('command_id') == self.last_command_id and
            (result.get('accepted') is False or 'timeout' in str(result.get('reason', ''))))
        pending = bool(not self.dry_run and value.get('pending_readback') is True and
            value.get('pending_command_id') == self.last_command_id and
            value.get('protocol_ready') is True and value.get('real_uart') is True and
            value.get('pin') == 'B13' and value.get('active_high') is True and
            not value.get('error') and not explicit_failure and
            all(value.get(key) is not False for key in ('link_ok', 'scope', 'wheels_still')) and
            type(pending_until) in (int, float) and math.isfinite(pending_until) and
            type(published) in (int, float) and math.isfinite(published) and
            0 <= self.wall_clock()-published <= .3 and
            self._last_sent <= self.wall_clock() < min(self._last_sent+.3, pending_until))
        return dict(pump_backend='fpga_uart', pump_pin='B13', pump_active_high=True,
                    pump_on=actual, pump_requested_on=self.requested_on,
                    pump_feedback_fresh=bool(fresh), pump_ready=ready,
                    pump_readback_pending=pending,
                    pump_feedback_stamp=stamp, pump_remaining_ms=value.get('remaining_ms') if fresh else None,
                    pump_scope=value.get('scope') if fresh else None,
                    pump_link_ok=value.get('link_ok') if fresh else None,
                    pump_wheels_still=value.get('wheels_still') if fresh else None,
                    pump_lockout=value.get('lockout') if fresh else None,
                    pump_reason_code=value.get('reason_code') if fresh else None,
                    pump_feedback_command_id=value.get('feedback_command_id') if fresh else None,
                    pump_last_command_id=self.last_command_id,
                    pump_reason=(value.get('reason') if fresh else 'fpga_pump_awaiting_readback'
                                 if pending else 'fpga_pump_feedback_unavailable'),
                    pump_feedback_meaning='FPGA output-level readback only; water flow is not measured')

    def set(self, enabled, *, duration_ms=400, force=False):
        if type(enabled) is not bool:
            raise ValueError('Pump state must be boolean')
        if self.closed and enabled:
            raise RuntimeError('FPGA pump adapter is closed')
        if enabled:
            if not self.config['enabled']:
                raise RuntimeError('FPGA pump output is disabled in configuration')
            if type(duration_ms) is not int or not 1 <= duration_ms <= self.config['max_duration_ms']:
                raise ValueError('FPGA pump duration exceeds configured finite pulse limit')
            if self.requested_on:
                return  # Never renew or retry an ON pulse.
            if not self.dry_run and not self.snapshot()['pump_ready']:
                raise RuntimeError('fpga_pump_fresh_off_ready_required')
        now = self.wall_clock()
        # A fresh, unexpected HIGH while idle warrants another OFF. Do not flood
        # the shared UART from every 20 ms gimbal timer or duplicate shutdown.
        unexpected_on = self.snapshot()['pump_on'] is True and now-self._last_sent >= .1
        if not enabled and self._last_enabled is False and not force and not unexpected_on:
            self.requested_on = False
            return
        command_id = 'pump-' + uuid.uuid4().hex
        request = dict(action='set_pump', enabled=enabled, duration_ms=duration_ms if enabled else 0,
                       command_id=command_id, issued_at=now)
        if not self.dry_run and callable(self.emit):
            self.emit(request)
        self.requested_on = enabled
        self._last_enabled = enabled
        self._last_sent = now
        self.last_command_id = command_id

    def close(self):
        if self.closed:
            return
        self.set(False, force=True)
        self.closed = True


class FPGAPumpIO:
    """Keep the existing gimbal adapter, route only pump commands over ROS."""
    pump_backend = 'fpga_uart'

    def __init__(self, gimbal, pump, pump_config):
        self.gimbal = gimbal
        self.fpga_pump = pump
        self.config = deepcopy(gimbal.config)
        self.config['pump'] = validate_pump(pump_config)

    def __getattr__(self, name):
        return getattr(self.gimbal, name)

    def commit_reference(self, **kwargs):
        result = self.gimbal.commit_reference(**kwargs)
        # The gimbal backend owns no GPIO or pump. Preserve the independently
        # configured FPGA channel while replacing software angle references.
        pump_config = deepcopy(self.config['pump'])
        self.config = deepcopy(self.gimbal.config)
        self.config['pump'] = pump_config
        result['config'] = deepcopy(self.config)
        return result

    def take_restored_reference(self):
        take = getattr(self.gimbal, 'take_restored_reference', None)
        result = take() if callable(take) else None
        if result is not None:
            pump_config = deepcopy(self.config['pump'])
            self.config = deepcopy(self.gimbal.config)
            self.config['pump'] = pump_config
            result['config'] = deepcopy(self.config)
        return result

    def set_pump(self, enabled, *, duration_ms=400):
        if self.gimbal.closed:
            raise RuntimeError('Actuator adapter is closed')
        if enabled and not self.dry_run and getattr(self.gimbal, 'requires_measured_feedback', False):
            if self.gimbal.feedback().get('settled_measured') is not True:
                raise RuntimeError('Pump requires measured settled gimbal axes')
        self.fpga_pump.set(enabled, duration_ms=duration_ms)

    def pump_feedback(self):
        return self.fpga_pump.snapshot()

    def disable_gimbal(self):
        try:
            self.fpga_pump.set(False)
        finally:
            self.gimbal.disable_gimbal()

    def hold_gimbal(self):
        self.fpga_pump.set(False)
        self.gimbal.hold_gimbal()

    def close(self):
        try:
            self.fpga_pump.close()
        finally:
            self.gimbal.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
