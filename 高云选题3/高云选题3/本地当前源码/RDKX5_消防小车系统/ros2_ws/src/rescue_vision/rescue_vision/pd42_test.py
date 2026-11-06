"""One motor's bounded commissioning; callers may run independent instances.

This is a bench tool, not the two-axis gimbal controller. Connecting only reads;
enable, each 5-degree move and leased low-speed run require user actions. No homing, device
zero write, persistent configuration, fault clear or automatic motion retry.
All angles are motor-shaft displacement from this connection's stationary
encoder reading. They are not calibrated pan/tilt angles.

Run all methods on one worker. In particular, the ROS executor must never wait
for a serial transaction. Disable drops holding torque, which is suitable
only for the user's explicitly confirmed unloaded test motor.
"""
from collections import deque
import math
import time

from .pd42_protocol import (Command, RunState, read_request, position_request, velocity_request,
                            enable_request, degrees_to_counts, counts_to_degrees)
from .pd42_transport import PD42SerialTransport


ACTIONS = ('connect', 'read', 'enable', 'step_positive', 'step_negative',
           'return_origin', 'run_positive', 'run_negative', 'keepalive', 'stop', 'disconnect')
MOTOR_PROFILES = {
    'motor1': {'port': '/dev/ttyS1', 'tx_pin': 8, 'rx_pin': 10, 'protocol': 'pd42'},
    'motor2': {'port': '/dev/ttyS3', 'tx_pin': 3, 'rx_pin': 5, 'protocol': 'zdt_xs'},
}
# Kept as the original default for offline tools importing this constant.
PORT = MOTOR_PROFILES['motor1']['port']
BAUDRATE = 115200
ADDRESS = 1
STEP_DEG = 5.0
LIMIT_DEG = 15.0
SPEED_RPM = 5
ACCELERATION = 10
FEEDBACK_MAX_AGE_S = .7
MOVE_TIMEOUT_S = 3.0
ENABLE_LEASE_S = 30.0
IDLE_DISCONNECT_S = 120.0
CONTINUOUS_LEASE_S = 5.0
CONTINUOUS_START_TIMEOUT_S = 3.0


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_request(payload, now_s):
    """Reject unknown fields, stale/future requests and unsupported actions."""
    required = {'action', 'request_id', 'issued_at', 'ttl_s'}
    if isinstance(payload, dict) and payload.get('action') == 'keepalive':
        required.add('run_id')
    if not isinstance(payload, dict) or set(payload) not in (required, required | {'motor_id'}):
        raise ValueError('invalid_motor_test_request')
    if payload['action'] not in ACTIONS:
        raise ValueError('unsupported_motor_test_action')
    motor_id = payload.get('motor_id', 'motor1')
    if not isinstance(motor_id, str) or motor_id not in MOTOR_PROFILES:
        raise ValueError('invalid_motor_test_motor_id')
    request_id = payload['request_id']
    if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
        raise ValueError('invalid_motor_test_request_id')
    if payload['action'] == 'keepalive':
        run_id = payload['run_id']
        if not isinstance(run_id, str) or not 1 <= len(run_id) <= 128:
            raise ValueError('invalid_motor_test_run_id')
    issued, ttl = payload['issued_at'], payload['ttl_s']
    if not finite(issued) or not finite(ttl) or not 0 < ttl <= 3.0:
        raise ValueError('invalid_motor_test_request_time')
    if not finite(now_s) or not -.05 <= now_s-issued <= ttl:
        raise ValueError('expired_motor_test_request')
    return dict(payload, motor_id=motor_id)


def bench_guard(evidence, now_s):
    """Require fresh inactive production owners, especially the UART owner."""
    if not isinstance(evidence, dict) or not finite(now_s):
        return 'motor_test_invalid_interlock_state'
    for name in ('mission', 'actuators', 'fpga_link'):
        item = evidence.get(name, {})
        if not isinstance(item, dict):
            return name + '_state_stale'
        stamp = item.get('stamp')
        if item.get('synthetic') is True or not finite(stamp) or not -.05 <= now_s-stamp <= 1.0:
            return name + '_state_stale'
    m, a, f = (evidence[name] for name in ('mission', 'actuators', 'fpga_link'))
    if m.get('mode') not in ('inspect', 'standby', 'idle', 'scene'):
        return 'motor_test_requires_idle_mission'
    if m.get('traffic_active') or m.get('auto_fire_active') or m.get('emergency_stop') or m.get('pump_armed'):
        return 'motor_test_mission_busy_or_estop'
    pump_guard = a.get('guard', {})
    if not isinstance(pump_guard, dict):
        return 'motor_test_invalid_interlock_state'
    if (a.get('dry_run') is not True or a.get('enabled') is not False
            or a.get('pump_on') or a.get('pump_enabled') or a.get('estop')
            or a.get('manual_pump_active') or a.get('pump_requested_on')
            or a.get('hardware_written') or pump_guard.get('armed')):
        return 'motor_test_production_actuator_active'
    if f.get('dry_run') is not True:
        return 'motor_test_fpga_uart_active'
    return ''


class BenchReject(ValueError):
    pass


class PD42Bench:
    """Synchronous state machine; the owning worker serializes every method."""
    def __init__(self, transport_factory=PD42SerialTransport, clock=time.monotonic, *, zdt_factory=None, motor_id=None):
        if motor_id is not None and motor_id not in MOTOR_PROFILES:
            raise ValueError('invalid bound motor_id')
        self.bound_motor_id = motor_id
        self.factory, self.clock = transport_factory, clock
        self.zdt_factory = zdt_factory
        self.zdt_config = {}
        self.zdt_options = {}
        self.zdt_parameters = {}
        self.zdt_motion_reason = 'zdt_configuration_not_read'
        self.zdt_config_bytes = b''
        self.zdt_limited = False
        self.zdt_origin_target = None
        self.zdt_offset = 0
        self.zdt_pending_offset = None
        self.zdt_arrival_samples = []
        self.zdt_baseline_reason = ''
        self.motor_id = motor_id or 'motor1'
        self.link = None
        self.feedback = {}
        self.feedback_at = None
        self.reference_counts = None
        self.target_counts = None
        self.move_deadline = None
        self.enable_deadline = None
        self.last_user_at = None
        self.seen = deque(maxlen=512)
        self.last_request_id = None
        self.last_action = None
        self.last_motor_id = None
        self.last_result = None
        self.reason = 'motor_test_disconnected'
        self.error = ''
        self.fault = False
        self.control_reason = 'mission_state_stale'
        self.maybe_enabled = False
        self.move_request_id = None
        self.move_action = None
        self.guard_probe = None
        self.last_keepalive_result = None
        self.last_keepalive_reason = ''
        self._clear_run()

    def _clear_run(self):
        self.run_id = None
        self.run_action = None
        self.run_direction = 0
        self.run_deadline = None
        self.run_start_deadline = None
        self.run_confirmed = False

    def _late_guard(self):
        # In ROS the callback executor may receive an emergency stop while the
        # worker is reading feedback. Recheck current ownership after those
        # reads, immediately before enable/position writes.
        if self.guard_probe is not None:
            self.control_reason = self.guard_probe()
            if self.control_reason:
                raise BenchReject(self.control_reason)

    def fresh(self):
        return self.feedback_at is not None and 0 <= self.clock()-self.feedback_at <= FEEDBACK_MAX_AGE_S

    def _reply(self, result, reason, error=''):
        self.last_result, self.reason, self.error = result, reason, error

    def reject(self, payload, reason):
        self.last_request_id = payload.get('request_id') if isinstance(payload, dict) else None
        self.last_action = payload.get('action') if isinstance(payload, dict) else None
        self.last_motor_id = payload.get('motor_id', 'motor1') if isinstance(payload, dict) else None
        self._reply('rejected', reason)

    def _read(self, command):
        if self.motor_id == 'motor2':
            from . import zdt_xs_protocol as zdt
            mapped = {Command.READ_VERSION: zdt.Command.READ_VERSION,
                      Command.READ_VOLTAGE: zdt.Command.READ_VOLTAGE}
            return self.link.exchange(zdt.read_request(ADDRESS, mapped[command])).value
        return self.link.exchange(read_request(ADDRESS, command)).value

    def _zdt_read(self, command):
        from .zdt_xs_protocol import read_request as request
        return self.link.exchange(request(ADDRESS, command)).value

    def _zdt_setup(self):
        from . import zdt_xs_protocol as zdt
        from . import zdt_vendor_bench as vendor
        config_reply = self.link.exchange(zdt.read_request(ADDRESS, zdt.Command.READ_CONFIG))
        self.zdt_config = config_reply.value
        self.zdt_config_bytes = config_reply.data
        self.zdt_options = zdt.normalize_options(self.feedback['version'], self.zdt_config,
                                               self._zdt_read(zdt.Command.READ_OPTIONS))
        self.zdt_limited = vendor.matches(self.feedback['version'], self.zdt_config_bytes, self.zdt_options)
        c, o = self.zdt_config, self.zdt_options
        if o.get('layout_verified') is not True:
            self.zdt_motion_reason = ('zdt_undocumented_options' if o.get('undocumented_bits_raw')
                                      else 'zdt_options_layout_unverified')
            self.feedback['configuration_detail'] = self.zdt_options.get('layout_verification_detail') or self.zdt_options.get('layout_verification_reason',
                'Unverified options layout; raw readback retained without guessing byte order.')
            return
        if c['mode'] != o['mode']:
            raise RuntimeError('zdt_firmware_mode_mismatch')
        self.zdt_motion_reason = ''
        if c['uart_mode'] != 2 or c['checksum_mode'] != 0 or c['baudrate'] != BAUDRATE:
            self.zdt_motion_reason = 'zdt_serial_configuration_mismatch'
        elif c['ack_mode'] not in (1, 3):
            self.zdt_motion_reason = 'zdt_command_ack_required'
        elif not o['closed_loop']:
            self.zdt_motion_reason = 'zdt_closed_loop_required'
        elif not isinstance(o.get('scaled_input'), bool):
            self.zdt_motion_reason = 'zdt_input_scale_unknown'
        elif c['mode'] == 'emm' and (not c.get('microsteps') or not o.get('motor_step_deg')):
            self.zdt_motion_reason = 'zdt_position_scale_unknown'
        if not self.zdt_motion_reason:
            try:
                self.zdt_parameters = zdt.verified_motion_parameters(c, o)
            except zdt.ProtocolError as exc:
                self.zdt_motion_reason = 'zdt_motion_parameters_unverified'
                self.feedback['configuration_detail'] = str(exc)

    def _to_degrees(self, counts):
        if self.motor_id == 'motor2':
            from .zdt_xs_protocol import position_degrees
            return position_degrees(counts, self.zdt_config['mode'])
        return counts_to_degrees(counts)

    def _verify_zdt_settings(self):
        """A changed menu must not reinterpret the captured encoder origin."""
        from . import zdt_xs_protocol as zdt
        version = self._zdt_read(zdt.Command.READ_VERSION)
        config_reply = self.link.exchange(zdt.read_request(ADDRESS, zdt.Command.READ_CONFIG))
        config = config_reply.value
        options = zdt.normalize_options(version, config, self._zdt_read(zdt.Command.READ_OPTIONS))
        if (version != self.feedback['version'] or config != self.zdt_config or options != self.zdt_options
                or config_reply.data != self.zdt_config_bytes):
            self.zdt_motion_reason = 'zdt_configuration_changed_reconnect'
            self.zdt_limited = False
            raise BenchReject(self.zdt_motion_reason)

    def _zdt_stable_baseline(self, *, require_enabled=False):
        """Do not enable against a stale retained target, or stack relative moves."""
        from . import zdt_vendor_bench as vendor
        rows = []
        for _ in range(3):
            self._poll()
            row = dict(self.feedback)
            if require_enabled and row.get('enabled') is not True:
                self.zdt_baseline_reason = 'motor_test_enabled_feedback_required'
                raise BenchReject(self.zdt_baseline_reason)
            expected = (row['retained_target_counts'] if self.zdt_origin_target is None else
                        self.zdt_origin_target + self.zdt_offset * vendor.STEP_COUNTS)
            if not self._stationary() or not vendor.settled(row, expected):
                self.zdt_baseline_reason = 'zdt_retained_target_mismatch'
                raise BenchReject(self.zdt_baseline_reason)
            rows.append(row)
        if (max(r['position_counts'] for r in rows)-min(r['position_counts'] for r in rows))*vendor.COUNT_DEG > .03:
            self.zdt_baseline_reason = 'zdt_retained_target_mismatch'
            raise BenchReject(self.zdt_baseline_reason)
        if len({r['retained_target_counts'] for r in rows}) != 1:
            self.zdt_baseline_reason = 'zdt_retained_target_mismatch'
            raise BenchReject(self.zdt_baseline_reason)
        self.zdt_baseline_reason = ''
        if self.zdt_origin_target is None:
            self.zdt_origin_target = rows[-1]['retained_target_counts']

    def _zdt_limited_move(self, action):
        from . import zdt_vendor_bench as vendor
        desired = (0 if action == 'return_origin' else
                   self.zdt_offset + (1 if action == 'step_positive' else -1))
        if desired not in (-1, 0, 1):
            raise BenchReject('zdt_limited_position_boundary')
        if desired == self.zdt_offset:
            self._reply('completed', 'motor_test_measured_arrival')
            return
        direction = desired - self.zdt_offset
        if direction not in (-1, 1):
            raise BenchReject('zdt_limited_position_boundary')
        self._late_guard()
        self.zdt_pending_offset = desired
        self.zdt_arrival_samples = []
        self.target_counts = self.zdt_origin_target + desired * vendor.STEP_COUNTS
        self.move_request_id, self.move_action = self.last_request_id, action
        # Fixed vendor bytes, no guessed RPM scale or absolute-target conversion.
        self.link.exchange(vendor.frame(direction))
        self.move_deadline = self.clock() + 6.
        self.enable_deadline = self.clock() + ENABLE_LEASE_S
        self._reply('accepted', 'motor_test_waiting_measured_arrival')

    def _to_counts(self, degrees):
        if self.motor_id == 'motor2':
            scale = 65536. / 360. if self.zdt_config['mode'] == 'emm' else 10.
            return round(degrees * scale)
        return degrees_to_counts(degrees)

    def _set_enabled(self, enabled):
        if self.motor_id == 'motor2':
            from .zdt_xs_protocol import enable_request as request
        else:
            request = enable_request
        self.link.exchange(request(ADDRESS, enabled))

    def _position_packet(self, target):
        if self.motor_id != 'motor2':
            return position_request(ADDRESS, target, SPEED_RPM, ACCELERATION)
        from .zdt_xs_protocol import position_request as request
        c, o = self.zdt_config, self.zdt_options
        angle = self._to_degrees(target)
        quantum = (o['motor_step_deg'] / c['microsteps'] if c['mode'] == 'emm'
                   else .01 if o['scaled_input'] else .1)
        quantized = math.copysign(math.floor(abs(angle) / quantum + .5) * quantum, angle)
        if abs(quantized - angle) > .3:
            raise BenchReject('zdt_step_resolution_too_coarse')
        if abs(quantized - self._to_degrees(self.reference_counts)) > LIMIT_DEG + 1e-8:
            raise BenchReject('motor_test_15_degree_limit')
        return request(ADDRESS, angle, SPEED_RPM, **self.zdt_parameters, motion_mode=1,
                       acceleration=ACCELERATION if c['mode'] == 'emm' else 100)

    def _velocity_packet(self, direction):
        if self.motor_id != 'motor2':
            return velocity_request(ADDRESS, direction * SPEED_RPM, ACCELERATION)
        from .zdt_xs_protocol import velocity_request as request
        return request(ADDRESS, direction * SPEED_RPM, mode=self.zdt_parameters['mode'],
                       speed_div10=self.zdt_parameters['speed_div10'], acceleration=ACCELERATION)

    def _check_run_feedback(self):
        """Signed vendor speed feedback, no automatic direction inversion."""
        if not self.run_id:
            return
        if not self.fresh() or self.feedback.get('enabled') is not True:
            raise RuntimeError('motor_test_continuous_feedback_lost')
        if self.feedback.get('stalled') or self.feedback.get('run_state') in (3, 4, 5):
            raise RuntimeError('motor_test_driver_fault')
        speed = self.feedback.get('speed_rpm')
        if not finite(speed):
            raise RuntimeError('motor_test_continuous_invalid_speed')
        if speed * self.run_direction < 0:
            raise RuntimeError('motor_test_continuous_direction_mismatch')
        if abs(speed) > SPEED_RPM * 2:
            raise RuntimeError('motor_test_continuous_overspeed')
        if speed == 0:
            if self.run_confirmed or self.clock() >= self.run_start_deadline:
                raise RuntimeError('motor_test_continuous_speed_not_confirmed')
            return
        if not self.run_confirmed:
            self.run_confirmed = True
            self.last_request_id, self.last_action = self.run_id, self.run_action
            self.last_motor_id = self.motor_id
            self._reply('completed', 'motor_test_continuous_running')

    def _expire_run(self):
        self._disable()
        self._reply('completed', 'motor_test_continuous_lease_expired')

    def _poll(self):
        if self.motor_id == 'motor2':
            from . import zdt_xs_protocol as zdt
            started = self.clock()
            position = self._zdt_read(zdt.Command.READ_POSITION)
            speed = zdt.speed_rpm(self._zdt_read(zdt.Command.READ_SPEED), self.zdt_config['mode'])
            state = self._zdt_read(zdt.Command.READ_STATE)
            fault = any(state.get(k) for k in ('stalled', 'stall_protection', 'overtemperature', 'overcurrent'))
            values = dict(position_counts=position, speed_rpm=speed, enabled=state['enabled'],
                          stalled=fault, arrived=state['arrived'], raw_state=state,
                          run_state=4 if fault else 2 if speed != 0 else 1 if state['arrived'] else 0)
            if self.zdt_limited:
                from . import zdt_vendor_bench as vendor
                values.update(position_before_counts=position,
                    retained_target_counts=self._zdt_read(zdt.Command.READ_TARGET_POSITION),
                    realtime_target_counts=self._zdt_read(zdt.Command.READ_REALTIME_TARGET_POSITION),
                    position_error_deg=zdt.error_degrees(self._zdt_read(zdt.Command.READ_POSITION_ERROR), 'emm'),
                    position_counts=self._zdt_read(zdt.Command.READ_POSITION))
                if self.clock()-started > .6:
                    raise RuntimeError('zdt_limited_feedback_too_slow')
                if state['raw'] & 0x7c:
                    raise RuntimeError('motor_test_driver_fault')
                if self.target_counts is not None and state['enabled'] is not True:
                    raise RuntimeError('motor_test_enabled_feedback_required')
                if self.reference_counts is not None and any(
                        abs(values[k]-self.reference_counts)*vendor.COUNT_DEG > 1.3
                        for k in ('position_before_counts', 'position_counts')):
                    raise RuntimeError('zdt_limited_excursion')
                if self.target_counts is not None and abs(values['position_error_deg']) > 1.2:
                    raise RuntimeError('zdt_limited_following_error')
                if abs(speed) > 2:
                    raise RuntimeError('zdt_limited_overspeed')
                if self.target_counts is None:
                    expected = (values['retained_target_counts'] if self.zdt_origin_target is None else
                                self.zdt_origin_target + self.zdt_offset * vendor.STEP_COUNTS)
                    self.zdt_baseline_reason = '' if vendor.settled(values, expected) else 'zdt_retained_target_mismatch'
            if values['enabled']:
                self.maybe_enabled = True
            self.feedback.update(values)
            self.feedback_at = self.clock()
            return values
        values = {
            'position_counts': self._read(Command.READ_POSITION),
            'speed_rpm': self._read(Command.READ_SPEED),
            'run_state': int(self._read(Command.READ_STATUS)),
            'enabled': self._read(Command.READ_ENABLED),
            'stalled': self._read(Command.READ_STALL),
        }
        if values['enabled']:
            self.maybe_enabled = True
        self.feedback.update(values)
        self.feedback_at = self.clock()
        return values

    def _stationary(self):
        return (self.fresh() and self.feedback.get('speed_rpm') == 0
                and self.feedback.get('run_state') in (int(RunState.STOPPED), int(RunState.COMPLETED))
                and self.feedback.get('stalled') is False)

    def _disable(self):
        """One disable request, no automatic retries; preserve uncertainty."""
        self._set_enabled(False)
        self.maybe_enabled = False
        self.enable_deadline = None
        self.target_counts = self.move_deadline = None
        self.move_request_id = self.move_action = None
        self.zdt_pending_offset = None
        self.zdt_arrival_samples = []
        self._clear_run()
        self._poll()
        if self.feedback.get('enabled') is not False:
            raise RuntimeError('motor_disable_not_confirmed')

    def _fail(self, exc):
        original = str(exc)
        self.fault = True
        self.feedback_at = None
        # A position/enable write may have executed despite a lost ACK. Send a
        # different, idempotent stop action once; never repeat the original move.
        if self.link and (self.maybe_enabled or self.target_counts is not None):
            try:
                self._disable()
            except Exception as stop_exc:
                original += '; disable_unconfirmed: ' + str(stop_exc)
        self.target_counts = self.move_deadline = self.enable_deadline = None
        self.move_request_id = self.move_action = None
        self._clear_run()
        self._reply('error', 'motor_test_fault_reconnect_required', original)

    def _close(self, disable=True):
        failure = None
        if self.link:
            if disable and (self.maybe_enabled or self.feedback.get('enabled') or self.target_counts is not None):
                try:
                    self._disable()
                except Exception as exc:
                    failure = str(exc)
            try:
                self.link.close()
            finally:
                self.link = None
        self.reference_counts = self.target_counts = self.move_deadline = None
        self.enable_deadline = self.feedback_at = None
        self.move_request_id = self.move_action = None
        self.feedback = {}
        self.zdt_origin_target = None
        self.zdt_offset = 0
        self.zdt_pending_offset = None
        self.zdt_arrival_samples = []
        self.zdt_baseline_reason = ''
        self.maybe_enabled = False
        self.zdt_limited = False
        self._clear_run()
        if failure:
            raise RuntimeError('disconnect_disable_unconfirmed: ' + failure)

    def handle(self, payload, now_s, evidence):
        """Heartbeats renew ownership without overwriting operator receipts."""
        is_keepalive = isinstance(payload, dict) and payload.get('action') == 'keepalive'
        names = ('last_request_id', 'last_action', 'last_motor_id', 'last_result', 'reason', 'error')
        previous = {name: getattr(self, name) for name in names} if is_keepalive else None
        self._handle(payload, now_s, evidence)
        if previous is not None:
            self.last_keepalive_result, self.last_keepalive_reason = self.last_result, self.reason
            # A real watchdog/fault stop remains visible. Routine heartbeat
            # success/rejection must not replace the user's last operation.
            restore = names if self.last_result == 'rejected' or self.reason == 'motor_test_continuous_keepalive' else names[:3]
            for name in restore:
                setattr(self, name, previous[name])

    def _handle(self, payload, now_s, evidence):
        """Validate once again immediately before physical work begins."""
        try:
            request = validate_request(payload, now_s)
        except ValueError as exc:
            self.reject(payload, str(exc)); return
        if request['request_id'] in self.seen:
            self.reject(request, 'duplicate_motor_test_request'); return
        self.seen.append(request['request_id'])
        self.last_request_id, self.last_action = request['request_id'], request['action']
        self.last_motor_id = request['motor_id']
        action = request['action']
        if self.bound_motor_id is not None and request['motor_id'] != self.bound_motor_id:
            self._reply('rejected', 'motor_test_wrong_bound_motor'); return
        if self.link and request['motor_id'] != self.motor_id:
            # In particular, a stale Stop from the other motor's page must not
            # disable whichever shaft currently owns the UART worker.
            self._reply('rejected', 'motor_test_different_motor_connected'); return
        if not self.link and action != 'connect' and request['motor_id'] != self.motor_id:
            self._reply('rejected', 'motor_test_not_connected'); return
        self.last_user_at = self.clock()
        self.control_reason = bench_guard(evidence, now_s)
        if action not in ('stop', 'disconnect') and self.control_reason:
            self._reply('rejected', self.control_reason); return
        try:
            if action == 'disconnect':
                self._close(); self.fault = False
                self._reply('completed', 'motor_test_disconnected'); return
            if action == 'connect':
                if self.link:
                    raise BenchReject('motor_test_already_connected')
                self.motor_id = request['motor_id']
                profile = MOTOR_PROFILES[self.motor_id]
                if self.motor_id == 'motor2':
                    from .zdt_xs_transport import ZDTSerialTransport
                    factory = self.zdt_factory or ZDTSerialTransport
                    self.zdt_config, self.zdt_options, self.zdt_parameters = {}, {}, {}
                    self.zdt_motion_reason = 'zdt_configuration_not_read'
                    self.zdt_limited = False
                else:
                    factory = self.factory
                self.link = factory(profile['port'], baudrate=BAUDRATE, timeout_s=.12 if self.motor_id == 'motor2' else .06)
                self.feedback['version'] = self._read(Command.READ_VERSION)
                self.feedback['voltage_v'] = self._read(Command.READ_VOLTAGE)
                if self.motor_id == 'motor2':
                    self._zdt_setup()
                self._poll()
                if not self._stationary():
                    # Connect is strictly read-only, even if another controller
                    # left this motor moving. Do not capture a moving reference.
                    self._close(disable=False)
                    raise BenchReject('motor_test_connect_requires_stopped_motor')
                self.reference_counts = self.feedback['position_counts']
                if self.zdt_limited:
                    # A mismatch stays connected and visibly read-only; no reset
                    # or enable is used to make old and actual targets agree.
                    try:
                        self._zdt_stable_baseline()
                    except BenchReject:
                        pass
                self.enable_deadline = None
                self.fault = False
                self._reply('completed', 'motor_test_connected_read_only'); return
            if not self.link:
                raise BenchReject('motor_test_not_connected')
            if action == 'stop':
                self._disable()
                self._reply('completed', 'motor_test_disabled_no_holding_torque'); return
            if self.fault:
                raise BenchReject('motor_test_fault_reconnect_required')
            if action == 'read':
                self._poll(); self.feedback['voltage_v'] = self._read(Command.READ_VOLTAGE)
                if self.run_id:
                    if self.clock() >= self.run_deadline:
                        self._expire_run(); return
                    self._check_run_feedback()
                self._reply('completed', 'motor_test_feedback_read'); return
            if action == 'keepalive':
                if self.run_id is None or request['run_id'] != self.run_id:
                    raise BenchReject('motor_test_run_id_mismatch')
                if self.clock() >= self.run_deadline:
                    self._expire_run(); return
                self._check_run_feedback()
                self._late_guard()
                self.run_deadline = self.clock() + CONTINUOUS_LEASE_S
                self.enable_deadline = self.run_deadline
                self._reply('completed', 'motor_test_continuous_keepalive'); return
            if self.run_id is not None:
                raise BenchReject('motor_test_continuous_active_stop_first')
            if self.target_counts is not None:
                raise BenchReject('motor_test_move_in_progress')
            if self.motor_id == 'motor2' and self.zdt_motion_reason and not self.zdt_limited:
                raise BenchReject(self.zdt_motion_reason)
            if self.motor_id == 'motor2':
                self._verify_zdt_settings()
            self._poll()
            if not self._stationary():
                raise BenchReject('motor_test_requires_stopped_feedback')
            if self.zdt_limited:
                if action in ('run_positive', 'run_negative'):
                    raise BenchReject('zdt_limited_position_only')
                self._zdt_stable_baseline()
            if action == 'enable':
                self._late_guard()
                self.maybe_enabled = True  # Outcome may be unknown if ACK fails.
                self._set_enabled(True)
                self._poll()
                if self.feedback.get('enabled') is not True:
                    raise RuntimeError('motor_enable_not_confirmed')
                if self.zdt_limited:
                    try:
                        self._zdt_stable_baseline(require_enabled=True)
                    except BenchReject as exc:
                        # After enable, any changed target is a control fault.
                        raise RuntimeError(str(exc)) from exc
                self.enable_deadline = self.clock() + ENABLE_LEASE_S
                self._reply('completed', 'motor_test_enabled_30s'); return
            if self.enable_deadline is None or self.clock() >= self.enable_deadline:
                raise BenchReject('motor_test_explicit_enable_required')
            if self.feedback.get('enabled') is not True or self.reference_counts is None:
                raise BenchReject('motor_test_enabled_feedback_required')
            if self.zdt_limited:
                self._zdt_limited_move(action)
                return
            if action in ('run_positive', 'run_negative'):
                direction = 1 if action == 'run_positive' else -1
                packet = self._velocity_packet(direction)
                self._late_guard()
                self.run_id = request['request_id']
                self.run_action = action
                self.run_direction = direction
                self.run_deadline = self.clock() + CONTINUOUS_LEASE_S
                self.run_start_deadline = self.clock() + CONTINUOUS_START_TIMEOUT_S
                self.run_confirmed = False
                self.link.exchange(packet)
                self.enable_deadline = self.run_deadline
                self._reply('accepted', 'motor_test_waiting_measured_speed'); return
            # Steps start from measured shaft angle and become absolute targets
            # in the selected vendor's units; uncertain moves are never resent.
            current = self.feedback['position_counts']
            delta = self._to_counts(STEP_DEG) * (1 if action == 'step_positive' else -1)
            target = self.reference_counts if action == 'return_origin' else current + delta
            limit = self._to_counts(LIMIT_DEG)
            if abs(current-self.reference_counts) > limit + self._to_counts(.3):
                raise BenchReject('motor_test_reference_range_reconnect_required')
            if abs(target-self.reference_counts) > limit:
                raise BenchReject('motor_test_15_degree_limit')
            packet = self._position_packet(target)
            self._late_guard()
            self.target_counts = target
            self.move_request_id, self.move_action = self.last_request_id, action
            self.link.exchange(packet)
            self.move_deadline = self.clock() + MOVE_TIMEOUT_S
            self.enable_deadline = self.clock() + ENABLE_LEASE_S
            self._reply('accepted', 'motor_test_waiting_measured_arrival')
        except BenchReject as exc:
            self._reply('rejected', str(exc))
        except Exception as exc:
            # A failed connect remains a read-only failure: close without any
            # enable/disable request, even when a read was only partially done.
            if action == 'connect':
                try: self._close(disable=False)
                except Exception: pass
                self.fault = True
                self._reply('error', 'motor_test_connect_failed', str(exc))
            else:
                self._fail(exc)

    def tick(self, now_s, evidence):
        self.control_reason = bench_guard(evidence, now_s)
        if not self.link:
            return
        try:
            if self.control_reason:
                self._close()
                self._reply('error', 'motor_test_interlock_lost', self.control_reason)
                return
            if self.fault:
                return
            if self.run_id is not None and self.clock() >= self.run_deadline:
                self._expire_run(); return
            if self.enable_deadline is not None and self.clock() >= self.enable_deadline:
                self._disable()
                self._reply('completed', 'motor_test_enable_lease_expired'); return
            if self.last_user_at is not None and self.clock()-self.last_user_at >= IDLE_DISCONNECT_S:
                self._close()
                self._reply('completed', 'motor_test_idle_disconnected'); return
            if self.zdt_limited and self.target_counts is not None and self.clock() >= self.move_deadline:
                raise RuntimeError('motor_test_arrival_timeout')
            self._poll()
            if self.zdt_limited and self.target_counts is not None and self.clock() >= self.move_deadline:
                raise RuntimeError('motor_test_arrival_timeout')
            if self.feedback.get('stalled') or self.feedback.get('run_state') in (3, 4, 5):
                raise RuntimeError('motor_test_driver_fault')
            if self.run_id is not None:
                if self.clock() >= self.run_deadline:
                    self._expire_run(); return
                self._check_run_feedback()
                return
            if self.target_counts is not None:
                if self.zdt_limited:
                    from . import zdt_vendor_bench as vendor
                    arrived = (self._stationary() and self.feedback.get('enabled') is True
                               and self.feedback.get('arrived') is True
                               and vendor.settled(self.feedback, self.target_counts))
                    self.zdt_arrival_samples = ((self.zdt_arrival_samples + [self.feedback['position_counts']])[-3:]
                                                if arrived else [])
                    arrived = (len(self.zdt_arrival_samples) == 3 and
                               (max(self.zdt_arrival_samples)-min(self.zdt_arrival_samples))*vendor.COUNT_DEG <= .03)
                else:
                    arrived = (self._stationary() and self.feedback.get('enabled') is True
                        and (self.motor_id != 'motor2' or self.feedback.get('arrived') is True)
                        and abs(self.feedback['position_counts']-self.target_counts) <= self._to_counts(.3))
                if arrived:
                    if self.zdt_limited:
                        self.zdt_offset = self.zdt_pending_offset
                        self.zdt_pending_offset = None
                        self.zdt_arrival_samples = []
                    self.last_request_id, self.last_action = self.move_request_id, self.move_action
                    self.last_motor_id = self.motor_id
                    self.target_counts = self.move_deadline = None
                    self.move_request_id = self.move_action = None
                    self._reply('completed', 'motor_test_measured_arrival')
                elif self.clock() >= self.move_deadline:
                    raise RuntimeError('motor_test_arrival_timeout')
        except Exception as exc:
            self._fail(exc)

    def snapshot(self, now_s):
        fresh = bool(self.link and self.fresh())
        counts = self.feedback.get('position_counts') if fresh else None
        enabled = self.feedback.get('enabled') if fresh else None
        idle = self.target_counts is None and self.run_id is None
        config_reason = self.zdt_motion_reason if self.motor_id == 'motor2' else ''
        limited = self.motor_id == 'motor2' and self.zdt_limited
        position_ready = ((limited and not self.zdt_baseline_reason and self.zdt_origin_target is not None)
                          or not config_reason)
        ready = bool(self.link and fresh and not self.control_reason and position_ready and not self.fault and idle and self._stationary())
        profile = MOTOR_PROFILES[self.motor_id]
        return dict(stamp=now_s, busy=not idle, connected=self.link is not None,
            motor_id=self.motor_id, port=profile['port'], tx_pin=profile['tx_pin'], rx_pin=profile['rx_pin'],
            protocol=profile['protocol'], firmware_mode=self.zdt_config.get('mode') if self.motor_id == 'motor2' else None,
            motion_configuration_valid=not bool(config_reason), motion_configuration_reason=config_reason,
            motion_configuration_detail=self.feedback.get('configuration_detail', ''),
            motion_profile='zdt_vendor_fd8_bench' if limited else 'verified' if not config_reason else 'unverified',
            limited_position_only=limited, position_test_ready=bool(position_ready),
            position_test_reason=self.zdt_baseline_reason if limited else config_reason,
            driver_configuration=dict(self.zdt_config) if self.motor_id == 'motor2' else None,
            driver_options=dict(self.zdt_options) if self.motor_id == 'motor2' else None,
            baudrate=BAUDRATE, address=ADDRESS, enabled=enabled,
            position_counts=counts,
            retained_target_counts=self.feedback.get('retained_target_counts') if fresh else None,
            realtime_target_counts=self.feedback.get('realtime_target_counts') if fresh else None,
            position_error_deg=self.feedback.get('position_error_deg') if fresh else None,
            position_deg=(self._to_degrees(counts-self.reference_counts)
                          if counts is not None and self.reference_counts is not None else None),
            speed_rpm=self.feedback.get('speed_rpm') if fresh else None,
            run_state=self.feedback.get('run_state') if fresh else None,
            voltage_v=self.feedback.get('voltage_v') if fresh else None,
            firmware=self.feedback.get('version') if fresh else None,
            feedback_fresh=fresh, reference_counts=self.reference_counts,
            feedback_age_s=(self.clock()-self.feedback_at if self.feedback_at is not None else None),
            target_deg=(self._to_degrees(round(self.target_counts)-self.reference_counts)
                        if self.target_counts is not None and self.reference_counts is not None else None),
            last_request_id=self.last_request_id, last_action=self.last_action,
            last_motor_id=self.last_motor_id,
            last_result=self.last_result, reason=self.reason, error=self.error,
            control_ready=not bool(self.control_reason), control_reason=self.control_reason,
            can_enable=ready and (enabled is False or self.enable_deadline is None),
            can_move=ready and enabled is True and self.enable_deadline is not None
                and self.clock() < self.enable_deadline,
            can_run=ready and not limited and enabled is True and self.enable_deadline is not None
                and self.clock() < self.enable_deadline,
            continuous_running=bool(self.run_id and self.run_confirmed and fresh
                and self.run_deadline is not None and self.clock() < self.run_deadline
                and enabled is True and finite(self.feedback.get('speed_rpm'))
                and self.feedback['speed_rpm'] * self.run_direction > 0
                and abs(self.feedback['speed_rpm']) <= SPEED_RPM * 2),
            continuous_pending=bool(self.run_id and not self.run_confirmed),
            continuous_direction=self.run_direction,
            run_id=self.run_id,
            keepalive_remaining_s=max(0., self.run_deadline-self.clock()) if self.run_deadline is not None else 0.,
            keepalive_timeout_s=CONTINUOUS_LEASE_S,
            last_keepalive_result=self.last_keepalive_result,
            last_keepalive_reason=self.last_keepalive_reason,
            enable_remaining_s=max(0., self.enable_deadline-self.clock()) if self.enable_deadline is not None else 0.,
            step_deg=.9 if limited else STEP_DEG, limit_deg=.9 if limited else LIMIT_DEG,
            test_speed_rpm=None if limited else SPEED_RPM, test_speed_raw=1 if limited else None,
            synthetic=False, bench_only=True, unloaded_motor_only=True)

    def close(self):
        try:
            self._close()
        except Exception as exc:
            self._reply('error', 'motor_test_shutdown_disable_unconfirmed', str(exc))
