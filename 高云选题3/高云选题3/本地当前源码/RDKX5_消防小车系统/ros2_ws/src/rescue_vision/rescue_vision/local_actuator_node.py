"""X5 gimbal owner, with GPIO or finite-pulse FPGA pump control.

The node starts disarmed. FPGA B13 pulses expire independently of this process;
the legacy GPIO backend still needs an independently timed cut-off. Target
angles and time-based settling alone are not position or water-flow feedback.
"""
import json
import math
import threading
import time
from collections import OrderedDict

from .node_common import Node
from std_msgs.msg import Bool, String
from rescue_interfaces.msg import GimbalCommand
from .node_common import age, param, publish_json, read_json, spin
from .local_actuator_io import create_actuator_io
from .local_actuator_guard import PumpGuard
from .auto_fire_gate import AutoArmFence
from .gimbal_self_test import GimbalSelfTest, feedback_target_matches, measured_target_matches
from .gimbal_manual import GimbalManual
from .fpga_pump_io import FPGAPumpTransport
from .fixed_camera_pump_gate import fixed_camera_water_lock
from .manual_pump_test import ManualPumpTest
from .gimbal_reference_store import persist_reference
from .gimbal_reference_policy import recoverable_reference_fault


class LocalActuator(Node):
    def __init__(self):
        super().__init__('local_actuator')
        self.cfg = read_json(param(self, 'control_config', ''))
        io_cfg = read_json(param(self, 'io_config', ''))
        io_cfg['dry_run'] = bool(param(self, 'dry_run', True)) or io_cfg.get('dry_run', True)
        if io_cfg.get('servo', {}).get('backend') in ('pd42_serial', 'mixed_serial'):
            # Reconfirm the installed reference on each launch; never trust a
            # previously saved flag after a single-turn driver's power cycle.
            io_cfg['servo']['reference_confirmed_this_start'] = bool(param(self, 'pd42_reference_confirmed', False))
        self.dry_run = io_cfg['dry_run']
        self.fpga_pump = None
        self.manual_pump_transport = None
        manual_pump_enabled = param(self, 'manual_pump_test_enabled', False)
        if type(manual_pump_enabled) is not bool:
            raise ValueError('manual_pump_test_enabled must be boolean')
        if manual_pump_enabled and io_cfg.get('pump', {}).get('backend') != 'fpga_uart':
            raise ValueError('Manual pump test requires finite FPGA B13 output')
        if io_cfg.get('pump', {}).get('backend') == 'fpga_uart':
            pump_pub = self.create_publisher(String, '/fpga/pump/request', 10)
            self.fpga_pump = FPGAPumpTransport(io_cfg['pump'],
                lambda value: publish_json(pump_pub, value), dry_run=self.dry_run,
                wall_clock=lambda: self.get_clock().now().nanoseconds/1e9)
            # Explicit manual commissioning permission is separate from the
            # automatic transport's dry-run and calibrated firing permission.
            manual_cfg = dict(io_cfg['pump'], enabled=manual_pump_enabled)
            self.manual_pump_transport = FPGAPumpTransport(manual_cfg,
                lambda value: publish_json(pump_pub, value), dry_run=not manual_pump_enabled,
                wall_clock=lambda: self.get_clock().now().nanoseconds/1e9)
            # Each message is a complete output snapshot. Consume the latest
            # query result instead of a backlog that has already expired.
            self.create_subscription(String, '/fpga/pump/state', self.pump_state, 1)
        self.pump_manual = ManualPumpTest(manual_pump_enabled,
            io_cfg.get('pump', {}).get('max_duration_ms', 1000))
        self.enabled = self.cfg.get('enabled') is True
        self.pump_enabled = self.cfg.get('pump_enabled') is True
        self.require_selection = bool(param(self, 'require_target_selection', False))
        self.armed_target = None
        self.armed_auto_run = None
        self.auto_fence = AutoArmFence()
        self.seen_arm_commands = OrderedDict()
        if not self.dry_run and not self.enabled:
            raise ValueError('Keep dry_run=true while the controller is disabled')
        if self.pump_enabled and io_cfg.get('pump', {}).get('enabled') is not True:
            raise ValueError('Pump controller requires pump.enabled=true in the I/O configuration')
        self.guard = PumpGuard(self.cfg.get('pump_guard', {}))
        self.speed_limit = float(self.cfg.get('max_speed_deg_s', 20.))
        self.settle_s = float(self.cfg.get('settle_s', .4))
        if not math.isfinite(self.speed_limit) or not 0 < self.speed_limit <= 90:
            raise ValueError('max_speed_deg_s must be finite in (0,90]')
        if not math.isfinite(self.settle_s) or not .2 <= self.settle_s <= 5:
            raise ValueError('settle_s must be in [0.2,5]')
        # Legacy GPIO requires an external cutoff. The FPGA backend instead
        # admits only a finite pulse with its protocol/ready readback verified.
        if (not self.dry_run and self.pump_enabled and self.fpga_pump is None and
                self.cfg.get('external_pump_timeout_verified') is not True):
            raise ValueError('Verify an independent timed pump cut-off before enabling real pump control')
        self.io = (create_actuator_io(io_cfg, fpga_pump=self.fpga_pump) if self.fpga_pump is not None
                   else create_actuator_io(io_cfg))
        self.measured_backend = bool(getattr(self.io, 'requires_measured_feedback', False))
        self.self_test = GimbalSelfTest(self.io.config.get('servo', {}))
        self.manual = GimbalManual(self.io.config.get('servo', {}))
        self.io_feedback = {}
        self.measured_settled_target = None
        self.mission = {}; self.mission_at = None
        self.extension = {}; self.extension_at = None
        self.motor_bench = {}
        self.aim = {}; self.aim_at = None
        self.requested = self.commanded = None
        self.command_deadline = 0.; self.command_owner = None
        self.command_speed = self.speed_limit
        self.settled_after = math.inf
        self.last_tick = time.monotonic(); self.last_publish = 0.
        self.pwm_active = False; self.pump_on = None if self.fpga_pump is not None else False
        self.estop = False; self.fault = ''; self.last_request = 'startup_disarmed'
        self.reference_job = None
        self.reference_seen = OrderedDict()
        self.reference_cancel_reason = ''
        self.recovery_job = None
        self.recovery_phase, self.recovery_reason = 'idle', 'not_requested'
        self.recovery_seen = OrderedDict()
        self.output = self.create_publisher(String, '/actuators/state', 10)
        self.self_test_output = self.create_publisher(String, '/gimbal/self_test/state', 10)
        self.create_subscription(String, '/gimbal/self_test/request', self.self_test_request, 10)
        self.manual_output = self.create_publisher(String, '/gimbal/manual/state', 10)
        self.pump_manual_output = self.create_publisher(String, '/pump/manual/state', 10)
        self.create_subscription(String, '/pump/manual/request', self.pump_manual_request, 10)
        self.create_subscription(String, '/extension/state', self.extension_state, 10)
        self.create_subscription(String, '/motor_test/state', self.motor_bench_state, 10)
        self.create_subscription(String, '/gimbal/manual/request', self.manual_request, 10)
        self.create_subscription(GimbalCommand, '/control/gimbal', self.gimbal, 10)
        self.create_subscription(String, '/mission/state', self.mission_state, 10)
        self.create_subscription(String, '/fire/proposal', self.fire_proposal, 10)
        self.create_subscription(String, '/actuators/request', self.request, 10)
        self.create_subscription(String, '/fire/auto/request', self.auto_request, 10)
        self.create_subscription(Bool, '/control/emergency_stop', self.emergency, 10)
        self.create_timer(.02, self.tick)
        self.get_logger().info('X5 actuator adapter: dry_run=' + str(self.dry_run) +
            '; ' + ('serial encoder feedback required' if self.measured_backend else 'PWM, actual joint angles unavailable'))

    def parse(self, msg):
        value = json.loads(msg.data)
        if not isinstance(value, dict):
            raise ValueError('Expected JSON object')
        return value

    def pump_state(self, msg):
        if self.fpga_pump is not None:
            try:
                value = self.parse(msg)
                self.fpga_pump.observe(value)
                self.manual_pump_transport.observe(value)
            except (ValueError, TypeError):
                self.fpga_pump.observe(None)
                self.manual_pump_transport.observe(None)
            self.pump_on = self.fpga_pump.snapshot()['pump_on']

    def mission_state(self, msg):
        try:
            value = self.parse(msg)
            stamp = value.get('stamp')
            delay = self.get_clock().now().nanoseconds/1e9-stamp if isinstance(stamp, (int, float)) else math.inf
            if not 0 <= delay <= .5:
                raise ValueError('Stale mission state')
            self.mission = value; self.mission_at = time.monotonic()
            if value.get('emergency_stop') is True:
                self.auto_fence.manual_disarm(self.get_clock().now().nanoseconds/1e9)
                self.stop_outputs('fpga_emergency_stop')
        except (ValueError, TypeError):
            self.mission = {}; self.mission_at = None

    def fire_proposal(self, msg):
        try:
            self.aim = self.parse(msg); self.aim_at = time.monotonic()
        except (ValueError, TypeError):
            self.aim = {}; self.aim_at = None

    def fresh_mission(self, now):
        return self.mission_at is not None and 0 <= now-self.mission_at <= .5

    def extension_state(self, msg):
        try:
            value = self.parse(msg)
            stamp = value.get('stamp')
            if type(stamp) not in (int, float) or not 0 <= self.get_clock().now().nanoseconds/1e9-stamp <= .6:
                raise ValueError('stale_extension')
            self.extension, self.extension_at = value, time.monotonic()
        except (TypeError, ValueError):
            self.extension, self.extension_at = {}, None

    def motor_bench_state(self, msg):
        try:self.motor_bench = self.parse(msg)
        except (TypeError, ValueError):self.motor_bench = {'active': True}

    def pump_manual_context(self, now, feedback=None, *, fixed_nozzle_confirmed=None):
        e, m = self.extension, self.mission
        bench = self.motor_bench
        motors = bench.get('motors', {})
        if isinstance(motors, dict): motors = motors.values()
        if not isinstance(motors, (list, tuple, type({}.values()))): motors = []
        bench_busy = bench.get('active') or any(isinstance(v, dict) and
            (v.get('busy') or v.get('active') or v.get('continuous_active') or v.get('continuous_running') or
             v.get('run_id') or v.get('moving')) for v in motors)
        fpga_fresh = bool(self.extension_at is not None and 0 <= now-self.extension_at <= .6 and
            e.get('real_uart') is True and e.get('status_fresh') is True)
        motion_busy = bool(self.self_test.active or self.manual.active or bench_busy or self.pwm_active or
            self.requested is not None or self.command_owner is not None or
            self.recovery_job is not None or self.reference_job is not None)
        # A fixed-nozzle pulse does not use calibrated gimbal coordinates.
        # Exempt only a known reference/limit mismatch after raw encoder
        # samples prove both axes stationary and no motion owner remains.
        # Driver/communication errors and unknown pending motion stay faults.
        reference_only_fault = False
        stationary_motion_fault = False
        stationary_reason = ''
        if feedback is None:
            feedback = self.io.feedback() if self.measured_backend else {}
        speeds = feedback.get('motor_rpm')
        known_gimbal_moving = bool(feedback.get('raw_feedback_fresh') is True and
            (feedback.get('pan_step_pending') or (isinstance(speeds, (list, tuple)) and
            any(type(v) in (int, float) and math.isfinite(v) and v != 0 for v in speeds))))
        if self.measured_backend and not motion_busy and recoverable_reference_fault(self.fault):
            speeds = feedback.get('motor_rpm')
            reference_only_fault = bool(feedback.get('raw_feedback_fresh') is True and
                not feedback.get('readback_error') and
                (not feedback.get('error') or feedback.get('error') == self.fault) and
                feedback.get('target_deg') is None and feedback.get('requested_target_deg') is None and
                not feedback.get('pan_step_pending') and
                isinstance(speeds, (list, tuple)) and len(speeds) == 2 and
                all(type(v) in (int, float) and math.isfinite(v) and v == 0 for v in speeds))
        # FD8's software speed/following-error stop remains a motion fault.
        # Its read-only recovery may independently prove a fixed nozzle is now
        # stationary. This grants only one operator-requested B13 pulse, never
        # an enable, target replay, fault clear or automatic aiming permission.
        if self.fault == 'FD8 overspeed or excessive following error':
            stationary_reason = feedback.get('fixed_nozzle_stationary_reason') or ''
            stationary_motion_fault = bool(not motion_busy and
                feedback.get('reference_recovery_error') == self.fault and
                feedback.get('fixed_nozzle_stationary_confirmed') is True and
                feedback.get('raw_feedback_fresh') is True and
                not feedback.get('readback_error') and not feedback.get('error') and
                feedback.get('target_deg') is None and feedback.get('requested_target_deg') is None and
                not feedback.get('pan_step_pending'))
        ignored_fault = reference_only_fault or stationary_motion_fault
        # The readback owner retains the original recovery error for diagnosis,
        # even after a saved reference has been restored successfully. Retire
        # it from THIS manual-pulse gate only while current encoder evidence
        # proves a healthy, stationary owner with no unfinished motion. A new
        # reference does not create a holding target, so no hold is required;
        # an existing hold must still be independently measured in position.
        recovery_history = feedback.get('reference_recovery_error') or ''
        recovery_history_retired = bool(self.measured_backend and recovery_history and
            not motion_busy and not self.fault and not feedback.get('error') and
            not feedback.get('readback_error') and not feedback.get('fixed_nozzle_known_hazard') and
            feedback.get('feedback_fresh') is True and feedback.get('raw_feedback_fresh') is True and
            feedback.get('reference_confirmed_this_start') is True and
            not feedback.get('pan_step_pending') and
            feedback.get('target_deg') is None and feedback.get('requested_target_deg') is None and
            not feedback.get('reference_capture_active') and
            (feedback.get('holding_position') is not True or feedback.get('holding_measured') is True) and
            isinstance(speeds, (list, tuple)) and len(speeds) == 2 and
            all(type(v) in (int, float) and math.isfinite(v) and v == 0 for v in speeds))
        recovery_fault = '' if recovery_history_retired else recovery_history
        if fixed_nozzle_confirmed is None:
            fixed_nozzle_confirmed = bool(self.pump_manual.active and self.pump_manual.fixed_nozzle_confirmed)
        # Only the read-only owner can attest that it has no unretired motion.
        # A timed-out moving delegate, historical fault or pending handoff is
        # never eligible, even when this click confirms a fixed nozzle.
        communication_exempt = bool(fixed_nozzle_confirmed is True and not motion_busy and
            not known_gimbal_moving and not self.fault and not feedback.get('error') and
            not recovery_fault and
            feedback.get('ownership') == 'production_serial_readback' and
            feedback.get('fixed_nozzle_readonly_eligible') is True and
            feedback.get('hardware_written') is False and
            feedback.get('target_deg') is None and feedback.get('requested_target_deg') is None and
            not feedback.get('pan_step_pending') and not feedback.get('holding_position') and
            not feedback.get('reference_capture_active') and not getattr(self.io,'closed',True) and
            feedback.get('readback_error_kind') in ('none','timeout') and
            (feedback.get('readback_error') or feedback.get('raw_feedback_fresh') is not True))
        return dict(mission_fresh=self.fresh_mission(now),
            vehicle_stopped=m.get('vehicle_stopped') is True,
            idle_mode=m.get('mode') in ('idle', 'standby', 'inspect', 'fire') and
                not m.get('traffic_active') and not m.get('auto_fire_active'),
            extension_idle=fpga_fresh and e.get('vehicle_stopped') is True and e.get('busy') is False,
            fpga_recovering=e.get('uart_recovering') is True,
            fpga_feedback_fresh=fpga_fresh, fpga_error=e.get('error') or '',
            fpga_physical_host_ready=bool(fpga_fresh and
                e.get('fpga_host', {}).get('fresh') is True and
                e.get('fpga_host', {}).get('selected') is True and
                e.get('fpga_host', {}).get('scope') is True),
            fpga_link_ok=bool(fpga_fresh and e.get('fpga_host', {}).get('link_ok') is True),
            gimbal_known_moving=known_gimbal_moving,
            gimbal_communication_exempt=communication_exempt,
            fixed_nozzle_motion_reason=feedback.get('fixed_nozzle_readonly_reason',''),
            gimbal_reference_fault_ignored=reference_only_fault,
            gimbal_fault_ignored_for_manual_pump=ignored_fault,
            gimbal_fault_diagnostic=self.fault,
            gimbal_stationary_confirmed=stationary_motion_fault,
            gimbal_stationary_reason=stationary_reason,
            gimbal_recovery_history_retired=recovery_history_retired,
            gimbal_recovery_history=recovery_history,
            estop=bool(self.estop or m.get('emergency_stop')),
            fault='' if ignored_fault else (self.fault or feedback.get('error') or
                recovery_fault or feedback.get('fixed_nozzle_known_hazard') or
                ('' if communication_exempt else feedback.get('readback_error') or
                 ('gimbal_feedback_unavailable' if self.measured_backend and
                  feedback.get('raw_feedback_fresh') is not True else ''))),
            other_owner=bool(motion_busy or self.guard.armed or
                (self.fpga_pump is not None and self.fpga_pump.requested_on)))

    def pump_manual_request(self, msg):
        accepted_request = False
        try:
            data = self.parse(msg)
            if data.get('action') != 'stop' and self.recovery_job is not None:
                raise ValueError('gimbal_recovery_running')
            if data.get('action') != 'stop' and self.reference_job is not None:
                raise ValueError('gimbal_reference_capture_running')
            now, wall = time.monotonic(), self.get_clock().now().nanoseconds/1e9
            feedback = self.manual_pump_transport.snapshot() if self.manual_pump_transport else {}
            event = self.pump_manual.request(data, wall_now=wall, now=now,
                feedback=feedback, context=self.pump_manual_context(now,
                    fixed_nozzle_confirmed=data.get('fixed_nozzle_confirmed') is True))
            accepted_request = True
            if data.get('action') == 'stop':
                self.auto_fence.manual_disarm(wall)
                self.guard.disarm('operator_manual_pump_stop')
                self.armed_target = self.armed_auto_run = None
            if data.get('action') == 'stop' and not self.pump_manual.enabled:
                self.write_pump(False, force=True)
                self.publish_status(now, self.guard.snapshot(), force=True)
                return
            if data.get('action') == 'start' and event in ('off', 'on'):
                self.auto_fence.manual_disarm(wall)
                self.guard.disarm('manual_pump_owns_output')
                self.armed_target = self.armed_auto_run = None
            self.write_manual_pump(event == 'on', force=event == 'off')
            self.last_request = 'manual_pump_' + str(data.get('action'))
        except (ValueError, TypeError, RuntimeError) as exc:
            self.last_request = 'manual_pump_rejected: ' + str(exc)
            # A failed ON submission must still reserve output until fresh OFF.
            if accepted_request and self.pump_manual.blocking and self.pump_manual.on_id is None:
                self.pump_manual.cancel(time.monotonic(), str(exc))
                self.write_manual_pump(False, force=True)
        self.publish_status(time.monotonic(), self.guard.snapshot(), force=True)

    def write_manual_pump(self, enabled, *, force=False):
        transport = self.manual_pump_transport
        if transport is None:
            if enabled: raise RuntimeError('manual_pump_fpga_backend_required')
            return
        transport.set(enabled, duration_ms=self.pump_manual.duration_ms, force=force)
        self.pump_manual.submitted(transport.last_command_id, transport._last_sent, enabled=enabled)
        self.pump_on = transport.snapshot()['pump_on']

    def tick_manual_pump(self, now):
        feedback = self.manual_pump_transport.snapshot() if self.manual_pump_transport else {}
        event = self.pump_manual.update(now=now, wall_now=self.get_clock().now().nanoseconds/1e9,
            feedback=feedback, context=self.pump_manual_context(now))
        if event == 'off':self.write_manual_pump(False, force=True)
        elif event == 'on':self.write_manual_pump(True)
        self.publish_status(now, self.guard.snapshot())

    def owner_allowed(self, now, owner, feedback=None):
        m = self.mission
        if owner == 'manual':
            return self.manual.active and self.manual.readiness(
                self.io.feedback() if feedback is None else feedback, self.manual_context(now), own=True)[0]
        if owner == 'self_test':
            return self.self_test.active and self.self_test.readiness(
                self.io.feedback() if feedback is None else feedback, self.self_test_context(now), own=True)[0]
        if not self.fresh_mission(now) or m.get('operator_hold') or m.get('emergency_stop'):
            return False
        if owner == 'mission_home' and m.get('fixed_camera_observation') is True:
            return False  # A fixed camera cannot gain visibility by sweeping the nozzle.
        if owner == 'mission_home' and m.get('scan_only') is True and m.get('scan_motion_allowed') is not True:
            return False
        if owner == 'traffic':
            return m.get('traffic_active') is True
        if owner == 'road':
            return (not m.get('traffic_active') and m.get('mode') in ('standby','patrol','navigate')
                    and m.get('camera_pose') == 'road' and m.get('mounting_verified') is True)
        return (not m.get('traffic_active') and m.get('vehicle_stopped') is True and
                ((owner == 'fire' and m.get('mode') == 'fire') or
                 (owner == 'mission_home' and m.get('mode') in ('fire', 'inspect'))))

    def gimbal(self, msg):
        now = time.monotonic()
        if self.reference_job is not None or self.self_test.active or self.manual.active or self.pump_manual.blocking:
            return  # Self-test exclusively owns both axes until stop/completion.
        if self.measured_backend and self.io.feedback().get('reference_confirmed_this_start') is not True:
            return  # Live UART readback alone does not define mechanical angles.
        delay = age(self, msg.header.stamp)
        if not self.enabled or self.estop or self.fault or self.recovery_job is not None or not self.owner_allowed(now, msg.owner):
            return
        if not 0 <= delay <= .25 or not 1 <= msg.lease_ms <= 500:
            return
        lease = msg.lease_ms / 1000 - delay
        target = (float(msg.pan_deg), float(msg.tilt_deg))
        speed = float(msg.speed_deg_s)
        if lease <= 0 or not all(math.isfinite(x) for x in (*target, speed)) or not 0 < speed <= 90:
            return
        if target != self.requested:
            self.settled_after = math.inf
        self.requested = target
        self.command_owner = msg.owner
        self.command_speed = min(speed, self.speed_limit)
        self.command_deadline = now + lease

    def self_test_context(self, now):
        return dict(enabled=self.enabled, dry_run=self.dry_run, estop=self.estop or self.mission.get('emergency_stop'),
                    fault=self.fault, mission_fresh=self.fresh_mission(now),
                    vehicle_stopped=self.mission.get('vehicle_stopped') is True,
                    idle_mode=self.mission.get('mode') in ('idle', 'standby', 'inspect') and
                              not self.mission.get('traffic_active') and not self.mission.get('auto_fire_active'),
                    pump_on=self.pump_blocks_gimbal(), pump_armed=self.guard.armed,
                    active_owner=self.command_owner if self.pwm_active and now < self.command_deadline else None)

    def pump_blocks_gimbal(self):
        # A disabled automatic pump with no local pulse does not require an
        # unrelated FPGA connection merely to adjust the nozzle. A reported
        # ON, pending ON or explicit manual pulse always blocks both axes.
        return (self.pump_on is True or self.guard.armed or self.pump_manual.blocking or
                (self.fpga_pump is not None and self.fpga_pump.requested_on) or
                (self.manual_pump_transport is not None and self.manual_pump_transport.requested_on) or
                (self.pump_enabled and self.pump_on is not False))

    def manual_vehicle_policy(self, now):
        """Independent hand adjustment does not invent FPGA stop telemetry.

        Unknown vehicle telemetry is permitted only in a fresh idle/inspection
        mission, with no active navigation/traffic/automatic work. Positive
        evidence of movement always denies manual admission. Automatic aiming
        and self-test continue to use their measured-stop contexts unchanged.
        """
        mission = self.mission
        extension = self.extension
        extension_current = self.extension_at is not None and 0 <= now-self.extension_at <= .6
        telemetry_current = bool(extension_current and extension.get('real_uart') is True and
                                 extension.get('status_fresh') is True and
                                 type(extension.get('vehicle_stopped')) is bool)
        known_moving = bool((telemetry_current and extension['vehicle_stopped'] is False) or
                           (self.fresh_mission(now) and mission.get('vehicle_moving') is True))
        known_stopped = bool(self.fresh_mission(now) and mission.get('vehicle_stopped') is True)
        base = dict(independent_manual_adjustment=True, manual_vehicle_feedback_required=False,
                    manual_vehicle_feedback_known=bool(telemetry_current or known_stopped),
                    manual_vehicle_known_moving=known_moving)
        if not self.fresh_mission(now):
            return dict(base, ready=False, reason='mission_status_stale')
        if known_moving:
            return dict(base, ready=False, reason='manual_vehicle_known_moving')
        if (mission.get('mode') not in ('idle', 'standby', 'inspect') or
                mission.get('traffic_active') or mission.get('auto_fire_active') or
                mission.get('motion_allowed') is True):
            return dict(base, ready=False, reason='end_automatic_task_before_manual_adjustment')
        if extension_current and (extension.get('busy') or extension.get('active') or
                extension.get('mode_transition') or extension.get('task') == 'traffic' or
                (isinstance(extension.get('region_search'), dict) and extension['region_search'].get('active'))):
            return dict(base, ready=False, reason='end_automatic_task_before_manual_adjustment')
        return dict(base, ready=True, reason='ready')

    def manual_context(self, now):
        context = self.self_test_context(now)
        policy = self.manual_vehicle_policy(now)
        context.update(policy)
        context['manual_vehicle_policy_ready'] = policy['ready']
        context['manual_vehicle_policy_reason'] = policy['reason']
        return context

    def reference_readiness(self, now, feedback, *, own=False):
        if self.reference_job is not None and not own:
            return False, 'gimbal_reference_capture_running'
        vehicle_policy = self.manual_vehicle_policy(now)
        recovery_error = (self.reference_job.get('recovery_error', '') if own and self.reference_job
                          else feedback.get('reference_recovery_error', ''))
        recoverable = bool(recoverable_reference_fault(self.fault) and self.fault == recovery_error and
            (own or feedback.get('reference_recovery_permitted') is True))
        checks = (
            (not self.dry_run and self.enabled and self.measured_backend, 'real_actuator_not_enabled'),
            (own or feedback.get('reference_capture_supported') is True,
             'gimbal_reference_capture_not_configured'),
            (not self.estop and (not self.fault or recoverable) and not self.mission.get('emergency_stop'), 'actuator_fault_or_estop'),
            (vehicle_policy['ready'], vehicle_policy['reason']),
            (not self.pump_blocks_gimbal(), 'pump_must_be_disarmed'),
            (not self.manual.active and not self.self_test.active and not self.pwm_active and
             self.requested is None and self.command_owner is None, 'another_gimbal_owner_active'),
        )
        for ok, reason in checks:
            if not ok:
                return False, reason
        if not own and feedback.get('reference_capture_ready') is not True:
            return False, feedback.get('reference_capture_reason') or 'raw_encoder_feedback_stale'
        if not own:
            counts, speeds = feedback.get('motor_counts'), feedback.get('motor_rpm')
            if (feedback.get('raw_feedback_fresh') is not True or
                    not isinstance(counts, (list, tuple)) or len(counts) != 2 or
                    not all(type(v) is int for v in counts) or
                    not isinstance(speeds, (list, tuple)) or len(speeds) != 2 or
                    not all(type(v) in (int, float) and v == 0 for v in speeds)):
                return False, 'both_axes_must_be_measured_stationary'
        return True, 'ready'

    def request_reference(self, data):
        rid, stamp, ttl = (data.get(k) for k in ('request_id', 'issued_at', 'ttl_s'))
        wall = self.get_clock().now().nanoseconds/1e9
        if not isinstance(rid, str) or not 1 <= len(rid) <= 100 or rid in self.reference_seen or rid in self.manual.seen:
            raise ValueError('Unique request_id required')
        if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
                type(ttl) not in (int, float) or not math.isfinite(ttl) or
                not 0 < ttl <= 3 or not 0 <= wall-stamp <= ttl):
            raise ValueError('Fresh issued_at/ttl_s required')
        self.reference_seen[rid] = wall
        while len(self.reference_seen) > 128:
            self.reference_seen.popitem(last=False)
        if any(data.get(flag) is not True for flag in
               ('operator_zero_confirmed', 'supported_load', 'stationary_confirmed')):
            raise ValueError('gimbal_reference_explicit_pose_support_stationary_required')
        feedback = self.io.feedback()
        ready, reason = self.reference_readiness(time.monotonic(), feedback)
        if not ready:
            raise ValueError(reason)
        cancel = threading.Event()
        done = threading.Event()
        job = dict(request_id=rid, cancel=cancel, done=done, result=None, error=None,
                   deadline=time.monotonic()+10., recovery_error=(
                       feedback.get('reference_recovery_error', '') if self.fault else ''))
        self.reference_job = job
        self.reference_cancel_reason = ''
        self.manual.phase = 'referencing'
        self.manual.reason = self.manual.last_request_reason = 'gimbal_reference_capture_running'
        self.manual.last_request_id = rid
        self.manual.last_result = 'accepted'
        def run():
            try:
                job['result'] = self.io.commit_reference(operator_zero_confirmed=True,
                    supported_load=True, stationary_confirmed=True, cancel_event=cancel)
            except Exception as exc:
                job['error'] = str(exc)
            finally:
                done.set()
        job['thread'] = threading.Thread(target=run, name='gimbal-reference-capture', daemon=True)
        job['thread'].start()
        self.last_request = 'gimbal_reference_capture_started'

    def cancel_reference(self, reason):
        if self.reference_job is not None:
            self.reference_job['cancel'].set()
            self.reference_cancel_reason = reason

    def tick_reference(self, now):
        job = self.reference_job
        if job is None:
            return False
        ready, reason = self.reference_readiness(now, self.io.feedback(), own=True)
        if not ready:
            self.cancel_reference(reason)
        if now >= job['deadline']:
            self.cancel_reference('gimbal_reference_capture_timeout')
        if not job['done'].is_set():
            return True
        self.reference_job = None
        if job['cancel'].is_set():
            # A completed read-only capture may race with STOP. Keep all motion
            # blocked until an explicit later commissioning attempt/restart.
            if job['result'] is not None:
                self.fault = 'reference_capture_cancelled_after_capture_restart_required'
            self.manual.phase = 'cancelled'
            self.manual.reason = self.manual.last_request_reason = self.reference_cancel_reason or 'operator_stop'
            self.manual.last_result = 'cancelled'
            self.last_request = 'gimbal_reference_cancelled'
            return True
        if job['error']:
            self.manual.phase = 'failed'
            self.manual.reason = self.manual.last_request_reason = job['error']
            self.manual.last_result = 'error'
            self.last_request = 'gimbal_reference_failed: ' + job['error']
            return True
        try:
            persist_reference(job['result'], self.cfg)
            if (self.fault and self.fault == job.get('recovery_error') and
                    self.fault == job['result'].get('recovered_error')):
                self.fault = ''
            seen = self.manual.seen
            self.self_test = GimbalSelfTest(self.io.config['servo'])
            self.manual = GimbalManual(self.io.config['servo'])
            self.manual.seen = seen
            self.manual.seen.append(job['request_id'])
            self.manual.phase = 'completed'
            self.manual.reason = self.manual.last_request_reason = 'gimbal_reference_recorded_no_motion'
            self.manual.last_request_id = job['request_id']
            self.manual.last_result = 'completed'
            self.last_request = 'gimbal_reference_recorded_no_motion'
        except Exception as exc:
            self.fault = 'reference_profile_save_failed: ' + str(exc)
            self.manual.phase = 'failed'
            self.manual.reason = self.manual.last_request_reason = self.fault
            self.manual.last_result = 'error'
        return True

    def consume_restored_reference(self):
        """Persist a read-only absolute-origin restore before permitting targets."""
        take = getattr(self.io, 'take_restored_reference', None)
        result = take() if callable(take) else None
        if result is None:
            return False
        try:
            if (result.get('restore_kind') != 'absolute_encoder' or
                    result.get('motion_commands_sent') is not False or self.reference_job is not None or
                    self.manual.active or self.self_test.active or self.pwm_active or self.requested is not None):
                raise ValueError('absolute_reference_restore_context_changed')
            persist_reference(result, self.cfg)
            seen = self.manual.seen
            self.self_test = GimbalSelfTest(self.io.config['servo'])
            self.manual = GimbalManual(self.io.config['servo'])
            self.manual.seen = seen
            self.manual.phase = 'completed'
            self.manual.reason = self.manual.last_request_reason = 'absolute_reference_restored_no_motion'
            self.manual.last_result = 'completed'
            self.io.finish_reference_restore(result['reference_id'], True)
            self.last_request = 'absolute_reference_restored_no_motion'
        except Exception as exc:
            self.fault = 'absolute_reference_restore_save_failed: '+str(exc)
            self.io.finish_reference_restore(result['reference_id'], False, self.fault)
            self.last_request = self.fault
        return True

    def self_test_request(self, msg):
        try:
            data = self.parse(msg)
            if data.get('action') != 'stop' and self.recovery_job is not None:
                raise ValueError('gimbal_recovery_running')
            if data.get('action') != 'stop' and self.reference_job is not None:
                raise ValueError('gimbal_reference_capture_running')
            if data.get('action') != 'stop' and self.pump_manual.blocking:
                raise ValueError('manual_pump_running')
            if data.get('action') != 'stop' and self.manual.active:
                raise ValueError('manual_gimbal_already_running')
            self.self_test.request(data, wall_now=self.get_clock().now().nanoseconds/1e9,
                now=time.monotonic(), feedback=self.io.feedback() if self.measured_backend else {},
                context=self.self_test_context(time.monotonic()))
            if data.get('action') == 'stop':
                self.requested = None
                self.command_deadline = 0.
                self.stop_outputs('self_test_stop')
            else:
                self.guard.disarm('self_test'); self.write_pump(False)
                self.armed_target = self.armed_auto_run = None
            self.last_request = 'self_test_' + str(data.get('action'))
        except (ValueError, TypeError) as exc:
            self.last_request = 'self_test_rejected: ' + str(exc)

    def manual_request(self, msg):
        try:
            data = self.parse(msg)
            if data.get('action') == 'recover':
                self.manual.last_request_id = data.get('request_id')
                self.request_recovery(data)
                return
            if data.get('action') != 'stop' and self.recovery_job is not None:
                raise ValueError('gimbal_recovery_running')
            if data.get('action') == 'confirm_reference':
                self.manual.last_request_id = data.get('request_id')
                self.request_reference(data)
                return
            if data.get('action') != 'stop' and self.reference_job is not None:
                raise ValueError('gimbal_reference_capture_running')
            if data.get('action') != 'stop' and self.pump_manual.blocking:
                raise ValueError('manual_pump_running')
            if data.get('action') != 'stop' and self.self_test.active:
                raise ValueError('gimbal_self_test_active')
            self.manual.request(data, wall_now=self.get_clock().now().nanoseconds/1e9,
                now=time.monotonic(), feedback=self.io.feedback() if self.measured_backend else {},
                context=self.manual_context(time.monotonic()))
            if data.get('action') == 'stop':
                if self.recovery_job is not None:
                    self.recovery_job['cancelled'] = True
                self.requested = None; self.command_deadline = 0.
                self.stop_outputs('manual_gimbal_stop')
            else:
                self.guard.disarm('manual_gimbal'); self.write_pump(False)
                self.armed_target = self.armed_auto_run = None
            self.last_request = 'manual_gimbal_' + str(data.get('action'))
        except (ValueError, TypeError) as exc:
            self.last_request = 'manual_gimbal_rejected: ' + str(exc)
            self.manual.last_result = 'rejected'
            self.manual.last_request_reason = str(exc)

    def recovery_readiness(self, now):
        supported = callable(getattr(self.io, 'restart_readback', None))
        if not supported or self.dry_run or not self.enabled:
            return False, 'gimbal_recovery_not_supported'
        if self.recovery_job is not None:
            return False, 'gimbal_recovery_running'
        if self.reference_job is not None:
            return False, 'gimbal_reference_capture_running'
        if self.pump_manual.blocking:
            return False, 'manual_pump_running'
        if self.estop or self.mission.get('emergency_stop'):
            return False, 'emergency_stop_active'
        policy = self.manual_vehicle_policy(now)
        if not policy['ready']:
            return False, policy['reason']
        motors = self.motor_bench.get('motors', {})
        motors = list(motors.values()) if isinstance(motors, dict) else motors
        if self.motor_bench.get('active') or any(isinstance(m, dict) and any(m.get(k) for k in
                ('active', 'busy', 'continuous_active', 'continuous_running', 'moving', 'run_id'))
                for m in (motors if isinstance(motors, (list, tuple)) else [])):
            return False, 'motor_bench_active'
        return True, 'ready'

    def request_recovery(self, data):
        rid, stamp, ttl = (data.get(k) for k in ('request_id', 'issued_at', 'ttl_s'))
        wall, now = self.get_clock().now().nanoseconds/1e9, time.monotonic()
        if not isinstance(rid, str) or not 1 <= len(rid) <= 100 or rid in self.recovery_seen:
            raise ValueError('Unique request_id required')
        if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
                type(ttl) not in (int, float) or not math.isfinite(ttl) or
                not 0 < ttl <= 3 or not 0 <= wall-stamp <= ttl):
            raise ValueError('Fresh issued_at/ttl_s required')
        ready, reason = self.recovery_readiness(now)
        if not ready:
            raise ValueError(reason)
        self.recovery_seen[rid] = wall
        while len(self.recovery_seen) > 128:
            self.recovery_seen.popitem(last=False)
        self.auto_fence.manual_disarm(wall)
        # Forget requests first, even when a disconnected serial stop raises.
        # The new worker performs reads only until fresh checks pass.
        stop_error = ''
        try:
            self.stop_outputs('operator_gimbal_recovery', force_pump=True)
        except Exception as exc:
            stop_error = str(exc)
        self.requested = self.commanded = None
        self.command_deadline = 0.
        self.command_owner = None
        self.pwm_active = False
        self.guard.disarm('operator_gimbal_recovery')
        self.armed_target = self.armed_auto_run = None
        job = dict(request_id=rid, done=threading.Event(), error=None,
                   deadline=now+15., previous_fault=self.fault, stop_error=stop_error)
        self.recovery_job = job
        self.recovery_phase, self.recovery_reason = 'reconnecting', 'gimbal_reconnecting_readback'
        self.manual.phase = 'recovering'
        self.manual.reason = self.manual.last_request_reason = self.recovery_reason
        self.manual.last_request_id, self.manual.last_result = rid, 'accepted'
        def run():
            try:
                self.io.restart_readback()
            except Exception as exc:
                job['error'] = str(exc)
            finally:
                job['done'].set()
        job['thread'] = threading.Thread(target=run, name='gimbal-operator-recovery', daemon=True)
        job['thread'].start()
        self.last_request = 'gimbal_recovery_started'

    def tick_recovery(self, now):
        job = self.recovery_job
        if job is None:
            return
        feedback = self.io.feedback()
        reason = job['error'] or feedback.get('readback_error') or feedback.get('error')
        if job['done'].is_set() and not job['error']:
            self.recovery_phase = 'checking'
            self.consume_restored_reference()
            feedback = self.io.feedback()
            # Recovery only reopens readback and restores the saved reference.
            # Use the same independent-manual pump policy as admission: a
            # disabled pump does not require unrelated FPGA telemetry, while
            # known/pending water or an enabled unknown output still blocks.
            pump_off = not self.pump_blocks_gimbal()
            rpm = feedback.get('motor_rpm')
            stationary = isinstance(rpm, (list, tuple)) and len(rpm) == 2 and all(v == 0 for v in rpm)
            policy = self.manual_vehicle_policy(now)
            reason = (feedback.get('error') or feedback.get('readback_error') or
                      ('gimbal_recovery_cancelled' if job.get('cancelled') else '') or
                      (policy['reason'] if not policy['ready'] else '') or
                      ('gimbal_recovery_waiting_pump_off' if not pump_off else '') or
                      ('gimbal_recovery_waiting_stationary_readback' if not stationary else '') or
                      (feedback.get('absolute_reference_restore_reason') if not feedback.get('feedback_fresh') else '') or
                      ('gimbal_recovery_waiting_saved_reference' if not feedback.get('reference_confirmed_this_start') else '') or
                      ('raw_encoder_feedback_stale' if not feedback.get('feedback_fresh') else ''))
            if not reason and not self.estop and not self.mission.get('emergency_stop'):
                self.fault = ''
                self.recovery_job = None
                self.recovery_phase, self.recovery_reason = 'completed', 'gimbal_recovered_no_motion'
                self.manual.phase = 'completed'
                self.manual.last_request_id = job['request_id']
                self.manual.reason = self.manual.last_request_reason = self.recovery_reason
                self.manual.last_result = 'completed'
                self.last_request = self.recovery_reason
                return
        self.recovery_reason = reason or 'gimbal_reconnecting_readback'
        interrupted = now >= job['deadline'] or self.estop or self.mission.get('emergency_stop')
        if interrupted:
            job['cancelled'] = True
        if job['done'].is_set() and (job['error'] or job.get('cancelled')):
            self.recovery_job = None
            self.recovery_phase = 'waiting'
            self.manual.phase = 'failed'
            self.manual.last_request_id = job['request_id']
            self.manual.last_result = 'error'
            self.manual.reason = self.manual.last_request_reason = self.recovery_reason
            self.last_request = 'gimbal_recovery_waiting: '+self.recovery_reason

    def gates(self, now, feedback=None):
        m, a = self.mission, self.aim
        aim_detail = a.get('aim')
        if not isinstance(aim_detail, dict):
            aim_detail = {}
        mission_ok = (self.enabled and self.fresh_mission(now) and m.get('mode') == 'fire'
                      and m.get('vehicle_stopped') is True and not m.get('traffic_active')
                      and not m.get('operator_hold') and not m.get('emergency_stop'))
        command_ok = (self.requested is not None and self.commanded == self.requested
                      and self.command_owner == 'fire' and self.pwm_active
                      and now < self.command_deadline and now >= self.settled_after)
        if self.measured_backend:
            if feedback is None:
                feedback = self.io.feedback()
            command_ok = (command_ok and feedback.get('feedback_fresh') is True and
                          measured_target_matches(feedback, self.requested,
                              self.io.config['servo']['position_tolerance_deg']))
        else:
            feedback = {}
        if self.fpga_pump is not None:
            pump = self.fpga_pump.snapshot()
            # UART ON/OFF invalidates the old readback generation. Give only
            # this already-authorized session a bounded wait for its matching
            # status query. This is never a new-arm or navigation permission.
            command_ok = command_ok and (pump['pump_feedback_fresh'] or
                (self.guard.armed and pump['pump_readback_pending']))
        stamp = a.get('capture_stamp')
        capture_age = (self.get_clock().now().nanoseconds/1e9 - stamp
                       if isinstance(stamp, (int, float)) and math.isfinite(stamp) else math.inf)
        fixed_mode = (a.get('aim_mode') == 'fixed_camera_3d' or
                      a.get('camera_mount') == 'fixed_to_vehicle' or a.get('geometry_locked') is True)
        if fixed_mode:
            lock_ok = self.measured_backend and fixed_camera_water_lock(
                a, aim_detail, self.io.config, feedback, self.requested)
        else:
            lock_ok = (a.get('coordinate_frame') == 'undistorted_left_optical' and
                       aim_detail.get('coordinate_frame') == 'undistorted_left_optical' and
                       aim_detail.get('image_size_wh') == [1280, 1088] and
                       bool(aim_detail.get('camera_calibration_id')) and a.get('image_locked') is True)
        aim_ok = (self.aim_at is not None and 0 <= now-self.aim_at <= .4 and
                  0 <= capture_age <= .4 and a.get('valid') is True and
                  now-capture_age >= self.settled_after and
                  lock_ok and a.get('pump_candidate') is True and
                  (fixed_mode or (a.get('landing_compensated') is True and bool(a.get('landing_calibration_id')))) and
                  a.get('servo_reference_target_deg') == (list(self.requested) if self.requested else None))
        if self.require_selection:
            identity = self.target_identity(a)
            aim_ok = (aim_ok and identity is not None and self.target_identity(aim_detail) == identity
                      and (self.armed_target is None or identity == self.armed_target))
        if self.armed_auto_run is not None:
            aim_ok = aim_ok and self.auto_fence.authorized(
                self.armed_auto_run, self.get_clock().now().nanoseconds/1e9)
        return bool(mission_ok), bool(command_ok), bool(aim_ok)

    def auto_request(self, msg):
        try:
            data = self.parse(msg)
            if data.get('action') in ('start', 'scan') and self.reference_job is not None:
                raise ValueError('gimbal_reference_capture_running')
            if data.get('action') in ('start', 'scan') and self.pump_manual.blocking:
                raise ValueError('manual_pump_running')
            self.auto_fence.request(data, self.get_clock().now().nanoseconds/1e9)
            if data.get('action') == 'cancel' and self.armed_auto_run is not None:
                self.guard.disarm('automatic_authority_cancelled')
                self.write_pump(False)
                self.armed_target = self.armed_auto_run = None
        except (ValueError, TypeError) as exc:
            self.get_logger().warning('Auto authority rejected: '+str(exc))

    @staticmethod
    def target_identity(value):
        target, generation, session = (value.get(k) for k in ('target_id','selection_generation','registry_session'))
        if (type(target) is not int or target < 1 or type(generation) is not int or generation < 1
                or not isinstance(session,str) or not session):
            return None
        return target,generation,session

    def request(self, msg):
        try:
            data = self.parse(msg); action = data.get('action'); now = time.monotonic()
            if action == 'disarm':
                # A delayed automatic completion must not cancel a newer manual
                # arm. Manual disarm has no run id and always requests OFF.
                if 'auto_run_id' in data:
                    if data.get('retire_auto_run') is True:
                        self.auto_fence.retire(data['auto_run_id'])
                    if data['auto_run_id'] != self.armed_auto_run:
                        return
                else:
                    self.auto_fence.manual_disarm(self.get_clock().now().nanoseconds/1e9,
                                                  data.get('preserve_auto_run_id'))
                    if self.pump_manual.blocking:
                        self.pump_manual.cancel(now, 'operator_disarm', failed=False, force=True)
                        self.write_manual_pump(False, force=True)
                self.guard.disarm('operator_disarm'); self.write_pump(False, force=True)
                self.armed_target = self.armed_auto_run = None
            elif action == 'arm_pump':
                if self.reference_job is not None:
                    raise ValueError('gimbal_reference_capture_running')
                if self.pump_manual.blocking:
                    raise ValueError('manual_pump_running')
                if ('auto_run_id' in data and not self.auto_fence.check_arm(
                        data, self.get_clock().now().nanoseconds/1e9)):
                    raise ValueError('Automatic pump request has no active explicit run authority')
                if self.require_selection:
                    cid=data.get('command_id')
                    if not isinstance(cid,str) or not 1<=len(cid)<=100 or cid in self.seen_arm_commands:
                        raise ValueError('Pump request ID is missing or already consumed')
                    self.seen_arm_commands[cid]=now
                    while len(self.seen_arm_commands)>512:self.seen_arm_commands.popitem(last=False)
                    stamp=data.get('issued_at');ttl=data.get('ttl_s',2.)
                    delay=self.get_clock().now().nanoseconds/1e9-stamp if isinstance(stamp,(int,float)) else math.inf
                    identity=self.target_identity(data)
                    if (identity is None or identity!=self.target_identity(self.aim) or not 0<ttl<=5
                            or not 0<=delay<=ttl):
                        raise ValueError('Fresh explicit pump request must identify the currently locked fire')
                if (not self.pump_enabled or self.estop or self.fault or not all(self.gates(now))):
                    raise ValueError('Pump requires enabled configuration, stopped fire mission, settled gimbal and calibrated image lock')
                if self.fpga_pump is not None and not self.fpga_pump.snapshot()['pump_ready']:
                    raise ValueError('fpga_pump_fresh_off_ready_required')
                result = self.guard.arm(now, data.get('lease_s', 15.), max_bursts=data.get('max_bursts'))
                if self.require_selection and result['arm_accepted']:self.armed_target=identity
                if result['arm_accepted']:self.armed_auto_run=data.get('auto_run_id')
                self.write_pump(bool(result['pump_on']))
                if not result['arm_accepted']:
                    self.last_request = 'arm_pump not renewed: ' + result['reason']
                    return
            elif action == 'reset_estop':
                if not self.fresh_mission(now) or not self.mission.get('operator_hold'):
                    raise ValueError('Set mission idle before resetting emergency stop')
                # Releasing the operator stop never clears the interrupted
                # I/O state. It permits the separate explicit recovery action
                # instead of leaving estop/fault checks in a restart-only loop.
                self.estop = False
            else:
                raise ValueError('Expected action disarm / arm_pump / reset_estop')
            self.last_request = str(action)
        except (ValueError, TypeError) as exc:
            if not self.guard.pump_on and not self.pump_manual.blocking:
                try: self.write_pump(False)
                except Exception as output_error: self.fault = str(output_error)
            self.last_request = 'rejected: ' + str(exc)
            self.get_logger().warning(self.last_request)

    def write_pump(self, enabled, *, force=False):
        if self.pump_manual.blocking:
            if enabled:
                raise ValueError('manual_pump_running')
            return  # The manual owner writes and tracks its one OFF generation.
        if self.fpga_pump is not None:
            remaining = ((self.guard.burst_deadline or time.monotonic()) - time.monotonic())
            if enabled and remaining <= 0:
                enabled = False  # Never round an already expired pulse up to 1 ms.
            duration = max(1, min(self.fpga_pump.config['max_duration_ms'], math.floor(remaining*1000)))
            self.fpga_pump.set(bool(enabled), duration_ms=duration, force=force)
            self.pump_on = self.fpga_pump.snapshot()['pump_on']
            return
        if enabled != self.pump_on:
            self.io.set_pump(bool(enabled)); self.pump_on = bool(enabled)

    def finish_hold(self, reason):
        """Normal measured completion keeps installed axes at their target."""
        self.guard.disarm(reason)
        self.armed_target = self.armed_auto_run = None
        self.write_pump(False)
        self.io.hold_gimbal()
        self.requested = None
        self.command_deadline = 0.
        self.command_owner = None
        self.pwm_active = False
        self.commanded = None
        self.settled_after = math.inf
        self.measured_settled_target = None

    def stop_outputs(self, reason, *, force_pump=False):
        self.cancel_reference(reason)
        manual_owned = self.pump_manual.blocking
        if manual_owned:
            self.pump_manual.cancel(time.monotonic(), reason, failed=True, force=True)
            self.write_manual_pump(False, force=True)
        if self.manual.active:
            self.manual.cancel(reason)
        if self.self_test.active:
            self.self_test.cancel(reason)
        self.requested = None
        self.command_deadline = 0.
        self.command_owner = None
        self.guard.disarm(reason)
        self.armed_target=self.armed_auto_run=None
        try:
            if self.fpga_pump is not None and not manual_owned:
                self.fpga_pump.set(False, force=force_pump)
                self.pump_on = self.fpga_pump.snapshot()['pump_on']
            elif not manual_owned:
                self.io.set_pump(False)
                self.pump_on = False
        finally:
            # FPGAPumpIO.disable_gimbal also emits automatic OFF. During a
            # manual reservation use its underlying gimbal so that the OFF
            # readback is not stolen by a second transport generation.
            (getattr(self.io, 'gimbal', self.io) if manual_owned else self.io).disable_gimbal()
            self.pwm_active = False
            self.commanded = None
            self.settled_after = math.inf
            self.measured_settled_target = None

    def emergency(self, msg):
        if msg.data:
            self.estop = True
            self.auto_fence.manual_disarm(self.get_clock().now().nanoseconds/1e9)
            try: self.stop_outputs('emergency_stop', force_pump=True)
            except Exception as exc: self.fault = str(exc)

    def tick(self):
        now = time.monotonic(); dt = max(0., min(.1, now-self.last_tick)); self.last_tick = now
        if self.recovery_job is not None:
            self.tick_recovery(now)
            self.publish_status(now, self.guard.snapshot())
            return
        if self.consume_restored_reference():
            self.publish_status(now, self.guard.snapshot())
            return
        if self.reference_job is not None:
            self.tick_reference(now)
            self.publish_status(now, self.guard.snapshot())
            return
        if self.pump_manual.blocking:
            try:self.tick_manual_pump(now)
            except Exception as exc:
                self.fault = str(exc)
                self.pump_manual.cancel(now, 'manual_pump_io_fault', force=True)
                try:self.write_manual_pump(False, force=True)
                except Exception:pass
                self.publish_status(now, self.guard.snapshot())
            return  # The manual pulse exclusively owns B13, never the axes.
        try:
            gate_feedback = None
            if self.fpga_pump is not None:
                self.pump_on = self.fpga_pump.snapshot()['pump_on']
            if self.measured_backend:
                self.io_feedback = self.io.feedback()
                # Reuse only within this tick until an output operation.
                # The idle path needs no second copy of the FD8 diagnostics.
                gate_feedback = self.io_feedback
                if self.io_feedback.get('error'):
                    raise RuntimeError(self.io_feedback['error'])
            if self.self_test.active:
                target = self.self_test.tick(now, self.io_feedback, self.self_test_context(now))
                if target is None:
                    self.requested = None; self.command_deadline = 0.
                    gate_feedback = None
                    if self.self_test.phase == 'completed':
                        self.finish_hold('self_test_finished')
                    else:
                        self.stop_outputs('self_test_finished')
                else:
                    self.requested, self.command_owner = target, 'self_test'
                    self.command_speed = min(self.speed_limit, self.self_test.speed)
                    self.command_deadline = now + .3
            if self.manual.active:
                target = self.manual.tick(now, self.io_feedback, self.manual_context(now))
                if target is None:
                    self.requested = None; self.command_deadline = 0.
                    gate_feedback = None
                    if self.manual.phase == 'completed':
                        self.finish_hold('manual_gimbal_finished')
                    else:
                        self.stop_outputs('manual_gimbal_finished')
                else:
                    self.requested, self.command_owner = target, 'manual'
                    self.command_speed = min(self.speed_limit, self.manual.speed)
                    self.command_deadline = now + .3
            allowed = (self.enabled and not self.estop and not self.fault and
                       self.requested is not None and now < self.command_deadline and
                       self.owner_allowed(now, self.command_owner,
                           feedback=gate_feedback))
            if not allowed:
                if self.pwm_active or self.pump_on is True or (
                        self.fpga_pump is not None and self.fpga_pump.requested_on):
                    gate_feedback = None
                    self.stop_outputs('command_expired_or_mission_unavailable')
            elif self.measured_backend:
                # The driver ramps in hardware. Renew a latest-only, bounded
                # lease; the serial worker never blocks this ROS executor.
                self.io.set_gimbal(*self.requested, speed_deg_s=self.command_speed,
                                  lease_s=min(.5, self.command_deadline-now))
                self.commanded = self.requested
                self.pwm_active = True  # Legacy field means gimbal-output request active.
                self.io_feedback = self.io.feedback()
                # The lease/target write above is a feedback boundary. Reuse
                # only its new snapshot for the immediately following gate;
                # no feedback is cached across timer ticks or output changes.
                gate_feedback = self.io_feedback
                if measured_target_matches(self.io_feedback, self.requested,
                        self.io.config['servo']['position_tolerance_deg']):
                    if self.measured_settled_target != self.requested:
                        self.measured_settled_target = self.requested
                        self.settled_after = now + self.settle_s
                else:
                    self.measured_settled_target = None
                    self.settled_after = math.inf
            else:
                if self.commanded is None:
                    # First PWM target is explicitly requested by Mission.
                    # Unknown physical startup angle cannot be speed-controlled.
                    next_target = self.requested
                else:
                    step = self.command_speed * dt
                    next_target = tuple(a + max(-step, min(step, b-a))
                                        for a, b in zip(self.commanded, self.requested))
                if not self.pwm_active or next_target != self.commanded:
                    self.io.set_gimbal(*next_target)
                    self.commanded = next_target; self.pwm_active = True
                    self.settled_after = now + self.settle_s
            mission_ok, command_ok, aim_ok = self.gates(now, feedback=gate_feedback)
            guard = self.guard.update(now, mission_ok=mission_ok, command_ok=command_ok,
                                      aim_ok=aim_ok, estop=self.estop or bool(self.fault))
            self.write_pump(bool(guard['pump_on']) and self.pump_enabled)
            if not guard.get('armed',False):self.armed_target=self.armed_auto_run=None
        except Exception as exc:
            self.fault = str(exc)
            # A previous successful reconnect is historical, not the current
            # connection state. Surface a later UART/driver fault immediately
            # instead of leaving the console labelled "recovered".
            if self.measured_backend:
                self.recovery_phase = 'required'
                self.recovery_reason = self.fault
            if self.self_test.active:
                self.self_test.cancel(self.fault, failed=True)
            if self.manual.active:
                self.manual.cancel(self.fault, failed=True)
            try: self.stop_outputs('latched_io_fault')
            except Exception: pass
            guard = dict(self.guard.snapshot(), pump_on=False, state='RECOVERY_REQUIRED', reason=self.fault)
        self.publish_status(now, guard)

    def publish_status(self, now, guard, *, force=False):
        if force or now-self.last_publish >= .1:
            self.last_publish = now
            # One encoder snapshot for all status publications in this tick.
            # This avoids inconsistent target/age fields if the UART worker
            # completes a transaction between successive feedback() copies.
            feedback = self.io.feedback() if self.measured_backend else {}
            manual_state = self.manual.status(feedback, self.manual_context(now))
            can_recover, recovery_ready_reason = self.recovery_readiness(now)
            recovery_state = dict(recovery_supported=callable(getattr(self.io, 'restart_readback', None)),
                recovery_phase=self.recovery_phase, recovery_reason=self.recovery_reason,
                recovery_active=self.recovery_job is not None, can_recover=can_recover,
                recovery_ready_reason=recovery_ready_reason,
                diagnostic_feedback_fresh=feedback.get('diagnostic_feedback_fresh', False),
                diagnostic_angles_deg=feedback.get('diagnostic_angles_deg'))
            manual_state.update(recovery_state)
            capture_ready, capture_reason = self.reference_readiness(now, feedback)
            manual_state.update(reference_capture_ready=capture_ready,
                reference_capture_reason=capture_reason,
                reference_capture_supported=feedback.get('reference_capture_supported', False),
                reference_recovery_permitted=feedback.get('reference_recovery_permitted', False),
                reference_recovery_error=feedback.get('reference_recovery_error', ''),
                absolute_reference_saved=feedback.get('absolute_reference_saved', False),
                absolute_reference_restore_state=feedback.get('absolute_reference_restore_state', ''),
                absolute_reference_restore_reason=feedback.get('absolute_reference_restore_reason', ''))
            if self.reference_job is not None:
                manual_state.update(can_start=False, ready_reason='gimbal_reference_capture_running')
            if self.recovery_job is not None:
                manual_state.update(can_start=False, ready_reason='gimbal_recovery_running')
            pump_manual_feedback = self.manual_pump_transport.snapshot() if self.manual_pump_transport else {}
            fixed_context=self.pump_manual_context(now,feedback=feedback,fixed_nozzle_confirmed=True)
            fixed_ready,fixed_reason=self.pump_manual.readiness(now,pump_manual_feedback,fixed_context)
            fixed_ready=bool(fixed_ready and fixed_context['gimbal_communication_exempt'])
            if not fixed_ready and fixed_reason=='ready':
                fixed_reason=fixed_context.get('fixed_nozzle_motion_reason') or 'fixed_nozzle_exemption_not_required'
            publish_json(self.pump_manual_output, dict(stamp=self.get_clock().now().nanoseconds/1e9,
                **self.pump_manual.status(now, pump_manual_feedback, self.pump_manual_context(now, feedback=feedback)),
                fixed_nozzle_readiness=fixed_ready,fixed_nozzle_ready_reason=fixed_reason))
            publish_json(self.manual_output, dict(stamp=self.get_clock().now().nanoseconds/1e9,
                **manual_state))
            accepted = (not self.fault and not self.estop and self.pwm_active and
                        now < self.command_deadline and self.commanded == self.requested)
            if self.measured_backend:
                accepted = bool(accepted and feedback.get('feedback_fresh') and
                                feedback.get('command_accepted') and
                                feedback_target_matches(feedback, self.requested,
                                    self.io.config['servo']['position_tolerance_deg']))
            settled = accepted and now >= self.settled_after
            if self.measured_backend:
                settled = settled and feedback.get('settled_measured') is True
            pump_feedback = self.fpga_pump.snapshot() if self.fpga_pump is not None else {}
            if self.fpga_pump is not None:
                self.pump_on = pump_feedback['pump_on']
            publish_json(self.output, dict(stamp=self.get_clock().now().nanoseconds/1e9,
                dry_run=self.dry_run, enabled=self.enabled,
                status_fresh=not bool(self.fault) and (not self.measured_backend or feedback.get('feedback_fresh') is True),
                gimbal_acknowledged=accepted, target_deg=list(self.requested) if self.requested else None,
                commanded_deg=list(self.commanded) if self.commanded else None,
                actual_angles_deg=feedback.get('actual_angles_deg'), gimbal_settled_estimate=settled,
                gimbal_settled_measured=bool(self.measured_backend and settled),
                backend=getattr(self.io, 'backend', 'pwm_sysfs'),
                axis_protocols=feedback.get('axis_protocols'),
                gimbal_owner='self_test' if self.self_test.active else self.command_owner,
                feedback_required=self.measured_backend, feedback_fresh=feedback.get('feedback_fresh', False),
                raw_feedback_fresh=feedback.get('raw_feedback_fresh', False),
                raw_feedback_age_s=feedback.get('raw_feedback_age_s'),
                serial_connected=feedback.get('serial_connected', False),
                readback_error=feedback.get('readback_error', ''),
                reference_confirmed_this_start=feedback.get('reference_confirmed_this_start', False),
                reference_capture_supported=feedback.get('reference_capture_supported', False),
                reference_capture_ready=capture_ready, reference_capture_reason=capture_reason,
                reference_recovery_permitted=feedback.get('reference_recovery_permitted', False),
                reference_recovery_error=feedback.get('reference_recovery_error', ''),
                absolute_reference_saved=feedback.get('absolute_reference_saved', False),
                absolute_reference_restore_state=feedback.get('absolute_reference_restore_state', ''),
                absolute_reference_restore_reason=feedback.get('absolute_reference_restore_reason', ''),
                reference_id=self.io.config.get('servo', {}).get('reference_id'),
                axis_limits_deg=[[self.io.config['servo'][name]['min_angle_deg'],
                                  self.io.config['servo'][name]['max_angle_deg']]
                                 for name in ('pan', 'tilt')] if self.measured_backend else None,
                axis_reachable_limits_deg=manual_state.get('axis_reachable_limits_deg') if self.measured_backend else None,
                axis_step_deg=manual_state.get('axis_step_deg') if self.measured_backend else None,
                motion_axes=manual_state.get('motion_axes') if self.measured_backend else None,
                feedback_age_s=feedback.get('feedback_age_s'), motor_counts=feedback.get('motor_counts'),
                feedback_limit_tolerance_deg=feedback.get('feedback_limit_tolerance_deg'),
                motor_rpm=feedback.get('motor_rpm'), motor_enabled=feedback.get('motor_enabled'),
                gimbal_holding=feedback.get('holding_position', False),
                gimbal_holding_measured=feedback.get('holding_measured', False),
                gimbal_hold_target_deg=feedback.get('hold_target_deg'),
                pan_feedback_diagnostic=feedback.get('pan_feedback_diagnostic'),
                hardware_written=feedback.get('hardware_written', False) if self.measured_backend else not self.dry_run and self.pwm_active,
                pump_on=self.pump_on, pump_enabled=self.pump_enabled, guard=guard,
                manual_pump_active=self.pump_manual.blocking,
                manual_pump_phase=self.pump_manual.phase,
                manual_pump_output_enabled=self.pump_manual.enabled,
                pump_setting_id=self.io.config.get('pump', {}).get('setting_id'),
                water_aim_policy=self.io.config.get('pump', {}).get('water_aim_policy', 'measured_impact'),
                fixed_camera_landing_calibration_id=self.io.config.get('pump', {}).get('fixed_camera_landing_calibration_id'),
                geometry_calibration_id=self.io.config.get('pump', {}).get('geometry_calibration_id'),
                camera_nozzle_mount_id=self.io.config.get('pump', {}).get('camera_nozzle_mount_id'),
                pump_backend=pump_feedback.get('pump_backend', 'gpio'),
                pump_pin=pump_feedback.get('pump_pin'),
                pump_requested_on=pump_feedback.get('pump_requested_on', self.pump_on),
                pump_feedback_fresh=pump_feedback.get('pump_feedback_fresh', not self.dry_run),
                pump_readback_pending=pump_feedback.get('pump_readback_pending', False),
                pump_ready=pump_feedback.get('pump_ready', self.pump_enabled and not self.dry_run),
                pump_feedback_stamp=pump_feedback.get('pump_feedback_stamp'),
                pump_remaining_ms=pump_feedback.get('pump_remaining_ms'),
                pump_reason=pump_feedback.get('pump_reason'),
                pump_feedback_meaning=pump_feedback.get('pump_feedback_meaning', 'Local GPIO command; water flow is not measured'),
                last_request=self.last_request, estop=self.estop, error=self.fault,
                armed_target=list(self.armed_target) if self.armed_target else None,
                armed_auto_run_id=self.armed_auto_run,
                auto_authorized_run_id=(self.auto_fence.active_run if self.auto_fence.authorized(
                    self.auto_fence.active_run,self.get_clock().now().nanoseconds/1e9) else None),
                meaning=('Measured encoder angles; installation reference and gearing must be verified'
                         if self.measured_backend else 'PWM command accepted; settling is estimated, not measured joint feedback'),
                **recovery_state))
            publish_json(self.self_test_output, dict(stamp=self.get_clock().now().nanoseconds/1e9,
                **self.self_test.status(feedback, self.self_test_context(now))))

    def destroy_node(self):
        self.cancel_reference('shutdown')
        if self.reference_job is not None:
            self.reference_job['thread'].join(timeout=3.)
        if self.recovery_job is not None:
            self.recovery_job['thread'].join(timeout=3.)
        errors = []
        # ROS publication may already be unavailable. A failed pump OFF
        # publication must not skip the independent UART worker shutdown.
        actions = []
        if self.manual_pump_transport is not None:
            actions.append(('manual pump', self.manual_pump_transport.close))
        actions.extend((('pump guard', lambda: self.guard.disarm('shutdown')),
                        ('actuator I/O', self.io.close)))
        for name, close in actions:
            try:
                close()
            except Exception as exc:
                errors.append(name + ': ' + str(exc))
        if errors:
            self.get_logger().error('Actuator shutdown could not be confirmed: ' + '; '.join(errors))
        return super().destroy_node()


def main():
    spin(LocalActuator)
