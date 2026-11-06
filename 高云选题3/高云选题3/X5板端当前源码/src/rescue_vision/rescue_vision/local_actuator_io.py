"""Explicit X5 hardware PWM + MOS GPIO adapter; default construction is dry-run.

No GPIO probing, software PWM, neutral-position movement, or automatic hardware
fallback is performed. Physical servo calibration and pin mux are prerequisites.
Sysfs PWM can remain active after a killed process: use independent pump power
interlocks/watchdogs for a real vehicle; close() is best-effort software shutdown.
"""
from copy import deepcopy
from collections import deque
import importlib
import math
import os
from pathlib import Path
import re
import threading
import time

_OWNER_LOCK = '/run/lock/rescue-local-actuators.lock'
_PROCESS_LOCK = threading.Lock()
_SERVO_PWM_CONTROLLER = '34170000.pwm'


def _number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(field + ' must be a finite number')
    return float(value)


def _integer(value, field):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(field + ' must be a nonnegative integer')
    return value


def _verify_pwm_controller(axis):
    resolved = Path(axis['chip']).resolve(strict=True)
    if axis['controller'] not in resolved.parts:
        raise ValueError('PWM chip does not resolve to verified servo controller '
                         +axis['controller']+': '+str(resolved))


def _validate(config):
    if not isinstance(config, dict):
        raise ValueError('Actuator configuration must be an object')
    cfg = deepcopy(config)
    dry_run = cfg.setdefault('dry_run', True)
    if not isinstance(dry_run, bool):
        raise ValueError('dry_run must be boolean')
    servo = cfg.get('servo', {})
    if not isinstance(servo, dict):
        raise ValueError('servo configuration must be an object')
    if servo.get('backend') != 'pwm_sysfs':
        raise ValueError('servo.backend must be pwm_sysfs (hardware PWM)')
    frequency = _number(servo.get('frequency_hz'), 'servo.frequency_hz')
    if frequency != 50:
        raise ValueError('This servo adapter requires an explicitly configured 50 Hz period')
    if not dry_run and servo.get('calibration_confirmed') is not True:
        raise ValueError('Verify mechanical limits/pulse calibration before setting calibration_confirmed=true')
    identities = []
    for name in ('pan', 'tilt'):
        axis = servo.get(name, {})
        if not isinstance(axis, dict):
            raise ValueError(name + ' configuration must be an object')
        for key in ('min_angle_deg', 'max_angle_deg', 'min_pulse_us', 'max_pulse_us'):
            axis[key] = _number(axis.get(key), name+'.'+key)
        if not -180 <= axis['min_angle_deg'] < axis['max_angle_deg'] <= 180:
            raise ValueError(name + ' angle interval must lie within [-180,180]')
        if not 400 <= axis['min_pulse_us'] < axis['max_pulse_us'] <= 2600:
            raise ValueError(name + ' pulse interval must lie within 400..2600 us')
        if not isinstance(axis.get('invert'), bool):
            raise ValueError(name + '.invert must be explicit boolean')
        if axis.get('controller') != _SERVO_PWM_CONTROLLER:
            raise ValueError(name + '.controller must be verified servo controller '+_SERVO_PWM_CONTROLLER)
        if not dry_run or axis.get('chip') is not None or axis.get('channel') is not None:
            if not isinstance(axis.get('chip'), str) or not re.fullmatch(r'/sys/class/pwm/pwmchip\d+', axis['chip']):
                raise ValueError(name + '.chip must name an explicit /sys/class/pwm/pwmchipN')
            _integer(axis.get('channel'), name+'.channel')
            identities.append((axis['chip'], axis['channel']))
        servo[name] = axis
    if len(identities) == 2 and identities[0] == identities[1]:
        raise ValueError('Pan and tilt must use independent PWM channels')
    pump = cfg.get('pump', {})
    if not isinstance(pump, dict):
        raise ValueError('pump configuration must be an object')
    enabled = pump.setdefault('enabled', False)
    if not isinstance(enabled, bool):
        raise ValueError('pump.enabled must be boolean')
    if enabled and (not dry_run or any(pump.get(key) is not None for key in ('gpio_chip', 'line_offset', 'active_low'))):
        if not isinstance(pump.get('gpio_chip'), str) or not re.fullmatch(r'/dev/gpiochip\d+', pump['gpio_chip']):
            raise ValueError('pump.gpio_chip must name an explicit /dev/gpiochipN')
        _integer(pump.get('line_offset'), 'pump.line_offset')
        if not isinstance(pump.get('active_low'), bool):
            raise ValueError('pump.active_low must be an explicit boolean')
    if not dry_run:
        for name in ('pan', 'tilt'):
            _verify_pwm_controller(servo[name])
    cfg['servo'], cfg['pump'] = servo, pump
    return cfg


def _write(path, value):
    Path(path).write_text(str(value), encoding='ascii')


class _ExclusiveOwner:
    def __init__(self):
        if not _PROCESS_LOCK.acquire(blocking=False):
            raise RuntimeError('Local actuators already have an owner in this process')
        self.fd = None
        try:
            self.fcntl = importlib.import_module('fcntl')
            self.fd = os.open(_OWNER_LOCK, os.O_CREAT | os.O_RDWR | getattr(os, 'O_CLOEXEC', 0), 0o600)
            self.fcntl.flock(self.fd, self.fcntl.LOCK_EX | self.fcntl.LOCK_NB)
        except Exception:
            if self.fd is not None:
                os.close(self.fd)
            _PROCESS_LOCK.release()
            raise

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
            _PROCESS_LOCK.release()


class _PumpGPIO:
    def __init__(self, config):
        self.gpio = importlib.import_module('gpiod')
        self.line = self.chip = self.request = None
        self.offset = config['line_offset']
        try:
            if callable(getattr(self.gpio, 'request_lines', None)):
                self.value = self.gpio.line.Value
                settings = self.gpio.LineSettings(direction=self.gpio.line.Direction.OUTPUT,
                    active_low=config['active_low'], output_value=self.value.INACTIVE)
                self.request = self.gpio.request_lines(config['gpio_chip'], consumer='rescue-pump',
                    config={self.offset: settings})
            else:
                self.chip = self.gpio.Chip(config['gpio_chip'])
                self.line = self.chip.get_line(self.offset)
                flags = self.gpio.LINE_REQ_FLAG_ACTIVE_LOW if config['active_low'] else 0
                self.line.request(consumer='rescue-pump', type=self.gpio.LINE_REQ_DIR_OUT,
                                  flags=flags, default_vals=[0])
        except Exception:
            if self.chip is not None:
                self.chip.close()
            raise

    def set(self, enabled):
        if self.request is not None:
            self.request.set_value(self.offset, self.value.ACTIVE if enabled else self.value.INACTIVE)
        else:
            self.line.set_value(int(enabled))

    def close(self):
        errors = []
        try:
            self.set(False)
        except Exception as exc:
            errors.append(exc)
        for resource in (self.request, self.line):
            if resource is not None:
                try:
                    resource.release()
                except Exception as exc:
                    errors.append(exc)
        if self.chip is not None:
            try:
                self.chip.close()
            except Exception as exc:
                errors.append(exc)
        self.request = self.line = self.chip = None
        if errors:
            raise RuntimeError('Pump shutdown could not be confirmed: '+repr(errors))


class _HardwarePWM:
    def __init__(self, axis, frequency_hz):
        self.chip = Path(axis['chip'])
        self.channel = axis['channel']
        self.path = self.chip / ('pwm'+str(self.channel))
        self.exported = False
        self.enabled = False
        if self.path.exists():
            raise RuntimeError('PWM channel is already exported; refusing to take it over: '+str(self.path))
        if self.channel >= int((self.chip/'npwm').read_text(encoding='ascii').strip()):
            raise ValueError('PWM channel exceeds chip npwm')
        try:
            _write(self.chip/'export', self.channel)
            self.exported = True
            deadline = time.monotonic()+.5
            while not self.path.exists():
                if time.monotonic() >= deadline:
                    raise TimeoutError('Exported PWM attributes did not appear')
                time.sleep(.005)  # Wait for sysfs creation only; never generate PWM in software.
            _write(self.path/'enable', 0)
            _write(self.path/'duty_cycle', 0)
            _write(self.path/'period', round(1_000_000_000/frequency_hz))
            _write(self.path/'polarity', 'normal')
        except Exception as exc:
            try:
                self.close()
            except Exception as cleanup:
                raise RuntimeError(f'PWM initialization failed: {exc}; cleanup: {cleanup}') from exc
            raise

    def set(self, pulse_us):
        _write(self.path/'duty_cycle', round(pulse_us*1000))
        if not self.enabled:
            _write(self.path/'enable', 1)
            self.enabled = True

    def disable(self):
        _write(self.path/'enable', 0)
        self.enabled = False
        _write(self.path/'duty_cycle', 0)

    def close(self):
        if not self.exported:
            return
        errors = []
        for path, value in ((self.path/'enable', 0), (self.path/'duty_cycle', 0),
                            (self.chip/'unexport', self.channel)):
            try:
                _write(path, value)
            except Exception as exc:
                errors.append(exc)
        self.exported = self.enabled = False
        if errors:
            raise RuntimeError('PWM shutdown could not be confirmed: '+repr(errors))


class LocalActuatorIO:
    """set_gimbal uses commanded angles only; it is not servo position feedback."""
    def __init__(self, config):
        self.config = _validate(config)
        self.dry_run = self.config['dry_run']
        self.commands = deque(maxlen=256)
        self.closed = False
        self._owner = self._pump = None
        self._axes = {}
        self._mutex = threading.RLock()
        if self.dry_run:
            self.commands.append({'operation': 'initialize', 'pump': False, 'pwm_enabled': False})
            return
        try:
            self._owner = _ExclusiveOwner()
            if self.config['pump']['enabled']:
                self._pump = _PumpGPIO(self.config['pump'])  # Atomic OFF precedes any PWM initialization.
            for name in ('pan', 'tilt'):
                self._axes[name] = _HardwarePWM(self.config['servo'][name], 50)
        except Exception as exc:
            try:
                self.close()
            except Exception as cleanup:
                raise RuntimeError(f'Actuator initialization failed: {exc}; cleanup: {cleanup}') from exc
            raise

    def _ensure_open(self):
        if self.closed:
            raise RuntimeError('Actuator adapter is closed')

    def set_gimbal(self, pan_deg, tilt_deg):
        with self._mutex:
            self._ensure_open()
            pulses = {}
            angles = {'pan': pan_deg, 'tilt': tilt_deg}
            for name, requested in angles.items():
                angle = _number(requested, name+'_deg')
                axis = self.config['servo'][name]
                if not axis['min_angle_deg'] <= angle <= axis['max_angle_deg']:
                    raise ValueError(name + ' angle exceeds calibrated mechanical limits')
                ratio = (angle-axis['min_angle_deg'])/(axis['max_angle_deg']-axis['min_angle_deg'])
                if axis['invert']:
                    ratio = 1-ratio
                pulses[name] = axis['min_pulse_us']+ratio*(axis['max_pulse_us']-axis['min_pulse_us'])
            try:
                if not self.dry_run:
                    for name in ('pan', 'tilt'):
                        self._axes[name].set(pulses[name])
            except Exception as exc:
                self._abort(exc)
            self.commands.append({'operation': 'gimbal', 'pan_deg': float(pan_deg),
                                  'tilt_deg': float(tilt_deg), 'pulse_us': pulses})

    def set_pump(self, enabled):
        with self._mutex:
            self._ensure_open()
            if not isinstance(enabled, bool):
                raise ValueError('Pump state must be boolean')
            if enabled and not self.config['pump']['enabled']:
                raise RuntimeError('Pump is disabled in configuration; an ON command is not permitted')
            try:
                if not self.dry_run and self._pump is not None:
                    self._pump.set(enabled)
            except Exception as exc:
                self._abort(exc)
            self.commands.append({'operation': 'pump', 'enabled': enabled})

    def disable_gimbal(self):
        """Stop both hardware PWM outputs while retaining channel ownership."""
        with self._mutex:
            self._ensure_open()
            try:
                if not self.dry_run:
                    for name in ('pan', 'tilt'):
                        self._axes[name].disable()
            except Exception as exc:
                self._abort(exc)
            self.commands.append({'operation': 'disable_gimbal', 'pwm_enabled': False})

    def _abort(self, cause):
        try:
            self.close()
        except Exception as cleanup:
            raise RuntimeError(f'Actuator command failed: {cause}; shutdown: {cleanup}') from cause
        raise RuntimeError('Actuator command failed; adapter closed: '+str(cause)) from cause

    def close(self):
        with self._mutex:
            if self.closed:
                return
            self.closed = True
            errors = []
            # Attempt every shutdown even when another output fails to turn off.
            for resource in [self._pump, *self._axes.values(), self._owner]:
                if resource is not None:
                    try:
                        resource.close()
                    except Exception as exc:
                        errors.append(exc)
            self._pump = self._owner = None
            self._axes.clear()
            self.commands.append({'operation': 'close', 'pump': False, 'pwm_enabled': False,
                                  'shutdown_confirmed': not errors})
            if errors:
                raise RuntimeError('Actuator shutdown could not be confirmed: '+repr(errors))

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def create_actuator_io(config, *, fpga_pump=None):
    if config.get('pump', {}).get('backend') == 'fpga_uart':
        from .fpga_pump_io import FPGAPumpIO, FPGAPumpTransport, validate_pump
        pump = validate_pump(config['pump'])
        transport = fpga_pump or FPGAPumpTransport(pump, dry_run=config.get('dry_run', True))
        # Never let the underlying local gimbal adapter request X5 pump GPIO.
        local = deepcopy(config)
        local['pump'] = dict(enabled=False)
        return FPGAPumpIO(create_actuator_io(local), transport, pump)
    if config.get('servo', {}).get('backend') == 'mixed_serial':
        if config.get('dry_run') is False and (config['servo'].get('live_readback_until_reference') is True or
                (config['servo'].get('reference_confirmed_this_start') is True and
                 config.get('pump', {}).get('enabled') is False)):
            from .live_mixed_actuator_io import LiveMixedActuatorIO
            return LiveMixedActuatorIO(config)
        from .mixed_actuator_io import MixedActuatorIO
        return MixedActuatorIO(config)
    if config.get('servo', {}).get('backend') == 'pd42_serial':
        from .pd42_actuator_io import PD42ActuatorIO
        return PD42ActuatorIO(config)
    return LocalActuatorIO(config)
