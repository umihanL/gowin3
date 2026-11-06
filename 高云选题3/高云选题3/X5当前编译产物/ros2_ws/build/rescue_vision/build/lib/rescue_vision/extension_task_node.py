"""Exclusive owner of the physical FPGA UART for the ov5640 extension profile."""
import copy
import json
import math
import select
import time
from pathlib import Path
from .node_common import Node
from std_msgs.msg import String
from rescue_interfaces.msg import FpgaEvent
from .node_common import param, publish_json, spin, read_json
from .extension_control import ExtensionControl, fresh
from .route_fire_patrol import navigation_off_disarmed
from .ov5640_protocol import Parser, EXT_STATUS, HEARTBEAT, POINT_STATUS, CAPABILITIES, STOP
from .fpga_map_transport import (MapTelemetry, COMMANDS as MAP_COMMANDS, POSE_COMMANDS,
                                 POSE_XY, POSE_ANGLE, ROUTE_CURSOR)
from .fpga_pump_bridge import FPGAPumpBridge, COMMANDS as PUMP_COMMANDS
from .ov5640_protocol import PUMP_ON, PUMP_OFF, PUMP_STATUS, RESUME, HOST_STATUS
from .ov5640_protocol import RETAINED_JOG, RETAINED_JOG_STATUS


class ExtensionTask(Node):
    def __init__(self):
        super().__init__('extension_task')
        config_path = str(param(self, 'extension_config', ''))
        cfg = read_json(config_path) if config_path else {}
        self.serial = None
        self.error = None
        port = cfg.get('port', '')
        enabled = cfg.get('enabled') is True
        self.core = ExtensionControl(real_uart=False, side_camera_verified=cfg.get('side_camera_verified') is True,
            screen_x_per_forward_sign=cfg.get('screen_x_per_forward_sign',0),
            fixed_traffic_camera_verified=cfg.get('fixed_traffic_camera_verified') is True)
        self.core.route_catalog = read_json(str(param(self, 'route_config', '')))
        point_path = str(param(self, 'numbered_points_config',
            str(Path(config_path).parent / 'numbered_points.json') if config_path else ''))
        if point_path:
            self.core.numbered.load_catalog(read_json(point_path))
        config_dir = Path(config_path).parent
        region_path = config_dir / 'semantic_regions.json'
        if region_path.is_file():
            from .region_approach import RegionApproach
            region_catalog = read_json(str(region_path))
            map_catalog = read_json(str(config_dir / 'fpga_map_catalog.json'))
            approach_config_path = config_dir / 'region_approach.local.json'
            if not approach_config_path.is_file():
                approach_config_path = config_dir / 'region_approach.json'
            approach_cfg = read_json(str(approach_config_path))
            approach_cfg['traversability'] = read_json(str(config_dir / 'region_traversability.json'))
            approach = RegionApproach(approach_cfg, map_catalog, region_catalog, self.core.route_catalog)
            alignment_path = config_dir / 'checkpoint_alignment.local.json'
            if not alignment_path.is_file():
                alignment_path = config_dir / 'checkpoint_alignment.json'
            alignment_cfg = read_json(str(alignment_path)) if alignment_path.is_file() else None
            self.core.area.load(region_catalog, map_catalog.get('map_signature'), approach,
                                alignment_config=alignment_cfg)
        self.actuator = {}
        self.parser = Parser()
        self.tx_queue = []
        self.in_flight = None
        self.received_at = None
        self.state_pub = self.create_publisher(String, '/extension/state', 10)
        self.navigation_pub = self.create_publisher(String, '/navigation/state', 10)
        self.navigation_proposal_pub = self.create_publisher(String, '/navigation/proposal', 10)
        self.map_telemetry = MapTelemetry()
        self._map_queued_request = None
        self.map_pub = self.create_publisher(String, '/fpga/map_state', 10)
        self.pump_pub = self.create_publisher(String, '/fpga/pump/state', 10)
        self.pump_bridge = FPGAPumpBridge(self.core)
        self.event_pub = self.create_publisher(FpgaEvent, '/fpga/events', 10)
        self.outputs = {key: self.create_publisher(String, topic, 10) for key, topic in (
            ('mission', '/mission/request'), ('interaction', '/interaction/command'),
            ('auto', '/fire/auto/request'), ('actuator', '/actuators/request'))}
        self.create_subscription(String, '/extension/request', self.request, 10)
        self.create_subscription(String, '/fire/targets', self.targets, 10)
        self.create_subscription(String, '/firefighter/targets', self.firefighter_targets, 10)
        self.create_subscription(String, '/traffic/state', self.traffic, 10)
        self.create_subscription(String, '/actuators/state', self.actuators, 10)
        self.create_subscription(String, '/fire/auto/state', self.auto_state, 10)
        self.create_subscription(String, '/mission/state', self.mission_state, 10)
        from rescue_interfaces.msg import DetectionArray
        from .node_common import IMAGE_QOS
        self.create_subscription(DetectionArray, '/perception/depth_detections', self.depth, IMAGE_QOS)
        self.create_subscription(String, '/fpga/pump/request', self.pump_request, 10)
        # This graph contains only loaded configuration; reconnect never copies
        # a live navigation, centering, pump or reference state machine.
        self._uart_core_template = copy.deepcopy(self.core)
        self._uart_port = port
        self._uart_enabled = bool(enabled and isinstance(port, str) and port and port not in ('/dev/ttyS1', '/dev/ttyS3'))
        self._uart_closed = False
        self._uart_retry_at = 0.
        self._uart_retry_delay = .5
        self._uart_recovering = False
        self._uart_accept_after = -math.inf
        if enabled:
            if not self._uart_enabled:
                self.error = 'assign_separate_fpga_uart_motor_ports_are_reserved'
            else:
                self._try_uart_open(self.now())
        self.create_timer(.02, self.tick)
        self.last_publish = -1.

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def pump(self):
        if not hasattr(self, 'pump_bridge'):
            self.pump_bridge = FPGAPumpBridge(self.core)
        return self.pump_bridge

    def _uart_disconnected(self, reason, now):
        """Discard uncertain execution once; never retry an old actuation."""
        serial, self.serial = self.serial, None
        if serial is not None:
            try: serial.close()
            except Exception: pass
        old = self.core
        old_pump = getattr(self, 'pump_bridge', None)
        template = getattr(self, '_uart_core_template', None)
        self.core = copy.deepcopy(template) if template is not None else ExtensionControl(
            side_camera_verified=old.side_camera_verified,
            screen_x_per_forward_sign=old.screen_x_per_forward_sign,
            fixed_traffic_camera_verified=old.fixed_traffic_camera_verified)
        c = self.core
        c.sequence, c.seen, c.used_motion = old.sequence, set(old.seen), set(old.used_motion)
        c.used_motion.update(old.numbered.observed_tasks)
        old_panel = (old.area.navigation.map.get('start_reference') or {}).get('generation')
        c.area.consumed_panel_reference = old_panel or old.area.consumed_panel_reference
        c.last_result = dict(accepted=False, completed=False, reason=reason,
                            stamp=now, previous_command_cancelled=True)
        c.effects = [('auto', dict(action='cancel', command_id='uart-recovery:' + str(now), issued_at=now, ttl_s=.5)),
                     ('actuator', dict(action='disarm', issued_at=now, ttl_s=.5)),
                     ('mission', dict(mode='inspect'))]
        self.actuator = {}
        self.parser = Parser()
        self.tx_queue, self.in_flight, self.received_at = [], None, None
        self.map_telemetry = MapTelemetry()
        self._map_queued_request = None
        self.pump_bridge = FPGAPumpBridge(c)
        if old_pump is not None:
            self.pump_bridge.seen = copy.copy(old_pump.seen)
        self.pump_bridge.cancel_on(now)
        self.error = reason
        self._uart_recovering = True
        self._uart_accept_after = now
        self._uart_stop_ack_at = None
        self._uart_stop_reply_at = None
        self._uart_stop_result = None
        self._uart_recovery_reason = 'uart_waiting_reopen'
        delay = getattr(self, '_uart_retry_delay', .5)
        self._uart_retry_at = time.monotonic() + delay
        self._uart_retry_delay = min(8., delay * 2.)

    def _try_uart_open(self, now):
        steady = time.monotonic()
        if (self.serial is not None or not getattr(self, '_uart_enabled', False)
                or getattr(self, '_uart_closed', False) or steady < self._uart_retry_at):
            return
        opened = None
        try:
            import serial
            opened = serial.Serial(self._uart_port, 115200, timeout=0, write_timeout=.2, exclusive=True)
            opened.reset_input_buffer()
            self.serial = opened
            self.parser = Parser()
            self.core.real_uart = True
            self.core.protocol_ready = False
            self._uart_recovering = True
            self._uart_recovery_started = steady
            self._uart_last_recovery_poll = -math.inf
            self._uart_stop_ack_at = None
            self._uart_stop_reply_at = None
            self._uart_stop_result = None
            self._uart_recovery_reason = 'uart_waiting_stop_reply'
            self._uart_stop_sequence = self.core.send(STOP, now=now)
            self._uart_last_stop = steady
            self.error = 'uart_reconnected_waiting_fresh_stop_and_feedback'
        except Exception as exc:
            if opened is not None:
                try: opened.close()
                except Exception: pass
            self._uart_disconnected('uart_open_failed:' + str(exc), now)

    def _uart_recovery_tick(self, now):
        steady = time.monotonic()
        if self.serial is None:
            return
        if steady-self._uart_recovery_started > 6.:
            self._uart_disconnected('uart_recovery_handshake_timeout', now)
            return
        c, n = self.core, self.core.numbered
        ack = self._uart_stop_ack_at
        reply_at = getattr(self, '_uart_stop_reply_at', None)
        stop_result = getattr(self, '_uart_stop_result', None)
        after = ack if ack is not None else reply_at
        # Existing firmware returns STOP result=1 outside a control page.
        # Treat this as a rejected STOP, never as an acknowledgement. Fresh
        # reports can still prove an idle vehicle and LOW B13, allowing the
        # transport to reconnect while all control-page permissions stay off.
        rejected_outside_scope = bool(stop_result == 1 and reply_at is not None and
            n.host.get('scope') is False and n.host.get('run') is False and
            n.host.get('nav_session') is False and n.point.get('state') in (0, 1, 2, 3))
        pump = self.pump().state(now)
        if after is None:
            reason = 'uart_waiting_stop_reply'
        elif ack is None and not rejected_outside_scope:
            reason = 'uart_waiting_confirmed_stop_or_idle_outside_control_page'
        elif not c.protocol_ready:
            reason = 'uart_waiting_capabilities'
        elif not c.area.navigation.stopped(now, after):
            reason = 'uart_waiting_fresh_vehicle_stop_and_host_state'
        elif n.point_request_at is None or n.point_request_at <= after:
            reason = 'uart_waiting_fresh_point_status'
        elif c.capabilities.get('pump_b13') and not (
                pump.get('feedback_fresh') is True and pump.get('pump_on') is False and
                type(pump.get('feedback_stamp')) in (int, float) and pump['feedback_stamp'] > after):
            reason = 'uart_waiting_fresh_b13_low'
        else:
            reason = ''
        self._uart_recovery_reason = reason
        if not reason:
            self._uart_recovering = False
            self._uart_retry_delay = .5
            self._uart_accept_after = now
            self.pump().cancel_on(now)
            self.error = None
            c.last_result = dict(accepted=False, completed=False,
                reason=('uart_reconnected_waiting_physical_control_page' if rejected_outside_scope
                        else 'uart_reconnected_idle_new_command_required'), stamp=now)
            return
        if (ack is None and (stop_result != 1 or n.host.get('scope') is True) and
                steady-self._uart_last_stop >= .8):
            self._uart_stop_reply_at = None
            self._uart_stop_result = None
            self._uart_stop_sequence = c.send(STOP, now=now)
            self._uart_last_stop = steady
        if steady-self._uart_last_recovery_poll >= .2:
            for command in (EXT_STATUS, HEARTBEAT, POINT_STATUS):
                c.send(command, now=now)
            if not c.protocol_ready: c.send(CAPABILITIES, now=now)
            self._uart_last_recovery_poll = steady

    def extension_state(self, now):
        recovery = bool(getattr(self, '_uart_recovering', False))
        state = dict(self.core.state(now), error=self.error,
                     rejected_frames=self.parser.bad_frames, uart_recovering=recovery,
                     uart_recovery_reason=getattr(self, '_uart_recovery_reason', ''),
                     uart_stop_result=getattr(self, '_uart_stop_result', None))
        if recovery:
            state.update(can_move=False, reason='uart_reconnecting_wait_for_fresh_feedback')
        return state

    def pump_request(self, msg):
        value = {}
        try:
            value = self.parse(msg)
            if value.get('enabled') is True and (getattr(self, '_uart_recovering', False) or
                    value.get('issued_at', -math.inf) <= getattr(self, '_uart_accept_after', -math.inf)):
                raise ValueError('uart_recovery_requires_new_pump_command')
            self.pump().request(value, self.now())
        except (ValueError, TypeError, KeyError) as exc:
            self.pump().last_result = dict(command_id=value.get('command_id'),
                accepted=False, reason=str(exc))
        self.flush()

    @staticmethod
    def parse(msg):
        value = json.loads(msg.data)
        if not isinstance(value, dict):
            raise ValueError('expected_object')
        return value

    def request(self, msg):
        data = {}
        try:
            data = self.parse(msg)
            recovery = getattr(self, '_uart_recovering', False)
            if recovery:
                if data.get('action') not in ('stop', 'finish_task', 'stop_spray'):
                    raise ValueError('uart_reconnecting_wait_for_fresh_feedback')
                if self.serial is not None:
                    self.core.send(STOP, now=self.now())
                    self.pump().off(self.now(), 'uart-recovery-operator-off')
                self.core.last_result = dict(command_id=data.get('command_id'), accepted=True,
                    completed=False, reason='uart_recovery_stop_requested', stamp=self.now())
                self.flush()
                return
            if (data.get('action') not in ('stop', 'finish_task', 'stop_spray') and
                    data.get('issued_at', -math.inf) <= getattr(self, '_uart_accept_after', -math.inf)):
                raise ValueError('uart_reconnected_new_command_required')
            if data.get('action') in ('manual_move', 'route_turn', 'seek_firefighter','traffic_drive', 'navigate_point', 'search_region'):
                reason = self.actuator_guard(self.now(), navigation_only=(
                    data.get('action') == 'search_region' and data.get('execute_fire') is False))
                if reason:
                    raise ValueError(reason)
            self.core.request(data, self.now())
        except (TypeError, ValueError, KeyError) as exc:
            self.core.last_result = dict(command_id=data.get('command_id'), accepted=False,
                completed=False, reason=str(exc), stamp=self.now())
        self.flush()

    def actuator_guard(self, now, *, navigation_only=False):
        value = self.actuator
        # Pump requests and actuator snapshots arrive on different ROS topics.
        # The sole UART owner must also consult its local queue/readback so a
        # just-accepted ON cannot race movement while an older LOW is cached.
        bridge = getattr(self, 'pump_bridge', None)
        if bridge is not None and bridge.available():
            pending = any(item['command'] == PUMP_ON and
                          not item.get('cancelled') for item in bridge.pending.values())
            physical = bridge.state(now)
            # A redundant OFF does not invalidate a still-fresh measured LOW.
            # An ON write clears that evidence inside the bridge, so an OFF
            # queued after ON still blocks until actual LOW is read back.
            if (pending or physical.get('error') or not physical.get('feedback_fresh')
                    or physical.get('pump_on') is not False):
                return 'pump_output_pending_or_not_confirmed_off'
        if value.get('manual_pump_active') is True or value.get('pump_requested_on') is True:
            return 'manual_pump_must_finish_before_motion'
        if navigation_only:
            return '' if navigation_off_disarmed(value, now) else 'pump_off_fresh_actuator_feedback_required'
        if not fresh(value, now) or value.get('pump_on') is not False or value.get('estop') or value.get('error'):
            return 'pump_off_fresh_actuator_feedback_required'
        if (value.get('guard') or {}).get('armed') is not False:
            return 'disarm_pump_before_vehicle_motion'
        return ''

    def actuators(self, msg):
        try:
            self.actuator = self.parse(msg)
        except (TypeError, ValueError):
            self.actuator = {}
        self.core.actuator = self.actuator

    def search_pump_guard(self, reason, now):
        """Hold the accepted search route through a bounded readback delay."""
        from .search_pump_recovery import SearchPumpRecovery
        if not hasattr(self, '_search_pump_recovery'):
            self._search_pump_recovery = SearchPumpRecovery()
        helper = self._search_pump_recovery
        area = self.core.area
        area.pump_readback_hold_prestart = area.pump_readback_hold_search = False
        search = area.search
        if (getattr(self, '_uart_recovering', False) or not search or not search.active
                or search.execute_fire is not False):
            helper.reset()
            area.pump_readback_recovery = helper.snapshot(now)
            return reason
        bridge = self.pump()
        pending_on = any(item['command'] == PUMP_ON and not item.get('cancelled')
                         for item in bridge.pending.values())
        decision = helper.step(area.navigation, self.actuator, bridge.state(now), reason, now,
            pending_on=pending_on, mission=self.core.mission_state)
        area.pump_readback_hold_prestart = decision.hold_prestart
        area.pump_readback_hold_search = decision.hold_search
        area.pump_readback_recovery = helper.snapshot(now)
        if decision.request_off:
            bridge.off(now, 'search-readback-off:' + str(now))
        if decision.prioritize_readback:
            bridge.last_poll = min(bridge.last_poll, now-.2)
            if decision.hold_prestart and now-getattr(self, '_search_recovery_poll_at', -math.inf) >= .2:
                # The preparation hold needs all three independent stop/plan
                # replies, not only B13 LOW. These queries cannot move the car.
                for command, stamp in ((EXT_STATUS, self.core.status_at),
                                       (HOST_STATUS, self.core.numbered.host_at),
                                       (POINT_STATUS, self.core.numbered.point_at)):
                    if stamp is None or now-stamp >= .3:
                        self.core.send(command, now=now)
                self._search_recovery_poll_at = now
        return decision.abort_reason if decision.handled else reason

    def auto_state(self, msg):
        try:
            self.core.auto_state = self.parse(msg)
        except (TypeError, ValueError):
            self.core.auto_state = {}

    def mission_state(self, msg):
        try:
            self.core.mission_state = self.parse(msg)
        except (TypeError, ValueError):
            self.core.mission_state = {}

    def targets(self, msg):
        try:
            self.core.observe_targets(self.parse(msg), self.now())
        except (TypeError, ValueError, KeyError):
            pass

    def firefighter_targets(self, msg):
        try:
            self.core.observe_firefighter_targets(self.parse(msg), self.now())
        except (TypeError, ValueError, KeyError):
            pass

    def traffic(self, msg):
        try:
            data = self.parse(msg)
            self.core.observe_traffic(data, self.now())
        except (TypeError, ValueError, KeyError):
            pass

    def depth(self, msg):
        stamp_ns = int(msg.header.stamp.sec)*1000000000 + int(msg.header.stamp.nanosec)
        # Perception's stereo path returns rectified XYZ and rectified pixels.
        # The fixed-camera sidecar rotates XYZ back by R1.T and retains its
        # undistorted pixels/header. Neither path may be relabelled as the other.
        frame = msg.header.frame_id
        frame_valid = ((frame == 'rectified_left_optical' and msg.rectified is True) or
                       (frame == 'undistorted_left_optical' and msg.rectified is False))
        self.core.area.observe_depth(dict(capture_stamp_ns=stamp_ns, sequence=int(msg.sequence),
            coordinate_frame=frame, position_frame=frame if frame_valid else '',
            frame_valid=frame_valid, rectified=bool(msg.rectified),
            stereo_calibration_id=msg.stereo_calibration_id if frame_valid else '',
            detections=[dict(class_id=int(d.class_id),
                bbox_xyxy=list(d.bbox_xyxy), depth_valid=frame_valid and bool(d.depth_valid),
                depth_reason=d.depth_reason if frame_valid else 'depth_frame_and_rectification_flag_mismatch',
                position_m=[d.position_m.x, d.position_m.y, d.position_m.z]) for d in msg.detections]), self.now())

    def queue_map_request(self, request, now, *, priority=False):
        """Reserve one read; its timeout starts only after a physical write."""
        if getattr(self, '_map_queued_request', None) is not None:
            return False
        sequence = self.core.send(request[0], request[1], now=now)
        self._map_queued_request = dict(sequence=sequence, request=request, priority=priority)
        return True

    def map_safety_priority(self, now):
        urgent = {STOP, PUMP_OFF, HEARTBEAT}
        if self.core.status_at is None or now-self.core.status_at >= .3:
            urgent.add(EXT_STATUS)
        host_at = self.core.numbered.host_at
        if host_at is None or now-host_at >= .3:
            urgent.add(HOST_STATUS)
        pump_at = self.pump().feedback_at
        if pump_at is None or now-pump_at >= .3:
            urgent.add(PUMP_STATUS)
        if getattr(self.core.area, 'pump_readback_hold_prestart', False):
            point_at = self.core.numbered.point_at
            if point_at is None or now-point_at >= .3:
                urgent.add(POINT_STATUS)
        return urgent

    def flush(self):
        area_leg = self.core.area.navigation.active or {}
        now = self.now()
        # A pose consists of a latch plus three matching replies. Giving every
        # read absolute last priority could starve that atomic snapshot behind
        # periodic status/pump/jog polls, falsely losing the same route during
        # centering. Service bounded pose reads ahead of ordinary polls while
        # STOP/OFF, heartbeat and ageing safety feedback keep priority.
        if (self.serial is not None and self.in_flight is None and
                getattr(self, '_map_queued_request', None) is None and
                not getattr(self, '_uart_recovering', False) and self.core.protocol_ready):
            # Recovery must remain reachable after a navigation failure. An
            # always-busy ordinary poll queue cannot permanently hide CAPS or
            # a missing MAP_ID; each remains bounded and strictly read-only.
            request = self.map_telemetry.next_recovery_request(now)
            if request is None:
                # Finish an already latched pose even while idle; this also
                # keeps the HMI's actual position observable after a stop.
                request = self.map_telemetry.next_pose_continuation(now)
            if (request is None and area_leg.get('continuous_route') and area_leg.get('phase') in (
                    'running', 'pausing', 'paused', 'resuming', 'restoring_speed',
                    'setting_retained_alignment_speed', 'centering_starting',
                    'centering_moving', 'centering_stopping', 'centering_verifying')):
                request = self.map_telemetry.next_control_pose_request(now)
            if request:
                self.queue_map_request(request, now, priority=True)
        # Establish writer provenance before this leg can enqueue START.
        # A missing record on an arbitrary already-starting leg is not proof
        # that its command was never physically sent.
        if area_leg.get('phase') == 'resetting':
            area_leg['start_write_tracking'] = True
        frames, self.core.outbox = self.core.outbox, []
        for frame in frames:
            command = frame[4]
            if command == 0x05:
                self.pump().cancel_on(self.now())
                # STOP supersedes queued actuation, including unstarted routes.
                discarded=[f for f in self.tx_queue if f[4] in (0x02,0x03,0x04,0x05,RESUME,RETAINED_JOG,0x08,0x10,0x11,0x12,0x13,0x17,PUMP_ON,0x20,0x21,0x22,0x23,0x24,0x25)]
                for old in discarded:self.pump().discard(old[3])
                self.tx_queue = [f for f in self.tx_queue if f not in discarded]
                self.tx_queue.insert(0, frame)
            elif command == PUMP_OFF:
                discarded = [f for f in self.tx_queue if f[4] in (PUMP_ON, PUMP_OFF)]
                for old in discarded:self.pump().discard(old[3])
                self.tx_queue = [f for f in self.tx_queue if f not in discarded]
                self.tx_queue.insert(1 if self.tx_queue and self.tx_queue[0][4] == 0x05 else 0, frame)
            elif (command in MAP_COMMANDS and self._map_queued_request is not None and
                    frame[3] == self._map_queued_request['sequence'] and
                    self._map_queued_request['priority']):
                urgent = self.map_safety_priority(now)
                index = max((i+1 for i,f in enumerate(self.tx_queue) if f[4] in urgent), default=0)
                self.tx_queue.insert(index, frame)
            elif command in (0x01,0x06,0x09,0x11,0x13,0x14,0x15,PUMP_STATUS,RETAINED_JOG_STATUS):
                discarded = [f for f in self.tx_queue if f[4] == command]
                for old in discarded:
                    if old[3] != frame[3]:
                        self.pump().discard(old[3])
                self.tx_queue = [f for f in self.tx_queue if f[4] != command]
                if command == PUMP_STATUS:
                    # Keep physical LOW evidence ahead of route downloads and
                    # ordinary polls, while STOP/OFF and the heartbeat keep priority.
                    urgent = (STOP, PUMP_OFF, HEARTBEAT)
                    index = 0
                    while index < len(self.tx_queue) and self.tx_queue[index][4] in urgent:
                        index += 1
                    self.tx_queue.insert(index, frame)
                else:
                    self.tx_queue.append(frame)
            else:
                self.tx_queue.append(frame)
        now = self.now()
        if self.in_flight is not None:
            timeout = .12 if (self.in_flight[1] in MAP_COMMANDS or self.in_flight[1] in PUMP_COMMANDS or
                self.in_flight[1] in (HEARTBEAT, EXT_STATUS, POINT_STATUS, CAPABILITIES, RETAINED_JOG_STATUS, 0x06) or
                self.tx_queue and self.tx_queue[0][4] in (0x05, PUMP_OFF)) else .6
            if now-self.in_flight[2] > timeout:
                self.in_flight = None
        if self.serial is not None:
            while self.tx_queue and self.in_flight is None:
                # Configure/ACK a paused plan normally, but a pending LOW
                # readback never authorizes START or another motion write.
                index = 0
                if getattr(self.core.area, 'pump_readback_hold_prestart', False):
                    index = next((i for i,f in enumerate(self.tx_queue)
                                  if f[4] not in (0x03, RESUME, RETAINED_JOG)), None)
                    if index is None:
                        break
                    # Keep OFF/STOP first, then complete the readbacks which
                    # release this hold. Queued START/RESUME/JOG remain blocked.
                    reads = (HEARTBEAT, EXT_STATUS, HOST_STATUS, POINT_STATUS, PUMP_STATUS)
                    index = next((i for i,f in enumerate(self.tx_queue) if f[4] in reads), index)
                    index = next((i for i,f in enumerate(self.tx_queue) if f[4] in (STOP, PUMP_OFF)), index)
                queued_map = getattr(self, '_map_queued_request', None)
                if (queued_map is not None and queued_map['priority'] and
                        self.tx_queue[index][3] == queued_map['sequence']):
                    urgent = self.map_safety_priority(now)
                    index = next((i for i,f in enumerate(self.tx_queue) if f[4] in urgent), index)
                frame = self.tx_queue.pop(index)
                map_request = getattr(self, '_map_queued_request', None)
                if map_request is not None and frame[3] == map_request['sequence'] and frame[4] in MAP_COMMANDS:
                    self.map_telemetry.expire(now)
                    if frame[4] in (POSE_XY, POSE_ANGLE, ROUTE_CURSOR):
                        work = self.map_telemetry.pose_work
                        if not work or map_request['request'][1][:2] != work['token']:
                            # A safety command may have outlived this token.
                            # Discard the queued read; never renew its capture time.
                            self.core.pending.pop(frame[3], None)
                            self._map_queued_request = None
                            continue
                else:
                    map_request = None
                if frame[4] == RETAINED_JOG and not self.core.area.navigation.centering_write_ready(frame[3], now):
                    self.core.pending.pop(frame[3], None)
                    self.core.stop(now, 'fire_centering_write_guard_rejected')
                    continue
                if frame[4] == RESUME and not (
                        self.core.numbered.resume_write_ready(frame[3], now) or
                        self.core.area.navigation.resume_write_ready(frame[3], now)):
                    self.core.pending.pop(frame[3], None)
                    area_resume = self.core.area.navigation.active or {}
                    if area_resume.get('phase') == 'resuming' and area_resume.get('sequence') == frame[3]:
                        self.core.stop(now, 'continuous_resume_write_guard_rejected')
                    elif (self.core.numbered.active or {}).get('phase') not in (
                            'stopping', 'confirming_arrival', 'arrived', 'cancelled', 'failed'):
                        self.core.stop(now, 'patrol_resume_write_guard_rejected')
                    continue
                frame = self.pump().before_write(frame, now)
                if frame is None:
                    continue
                try:
                    if self.serial.write(frame) != len(frame):
                        raise OSError('partial_uart_write')
                    written_at = self.now()
                    if map_request is not None:
                        self.map_telemetry.sent(frame[3], map_request['request'], written_at)
                        self._map_queued_request = None
                    if frame[4] == RETAINED_JOG:
                        self.core.area.navigation.centering.mark_written(frame[3], written_at)
                    if (frame[4] == 0x03 and area_leg.get('phase') == 'starting'
                            and area_leg.get('sequence') == frame[3]):
                        area_leg.update(start_written_sequence=frame[3], start_written_at=written_at)
                    self.core.pending[frame[3]] = (frame[4], written_at)
                    self.in_flight = (frame[3], frame[4], written_at)
                except Exception as exc:
                    self._uart_disconnected('uart_write_failed:' + str(exc), now)
                    break
        effects, self.core.effects = self.core.effects, []
        for key, value in effects:
            publish_json(self.outputs[key], value)

    def trace_fire_transition(self, snapshot):
        """Keep compact evidence of state changes, without logging every frame."""
        region = snapshot.get('region_search') or {}
        previous = getattr(self, '_last_fire_transition', None)
        if not region.get('active') and previous is None:
            return
        visual = region.get('centering') or {}
        navigation = region.get('navigation') or {}
        motion = navigation.get('centering') or {}
        recovery = region.get('pump_readback_recovery') or {}
        queue = [(item.get('batch_id'), item.get('target_id'), item.get('status'), item.get('reason'))
                 for item in visual.get('queue', [])[:32]]
        key = (region.get('command_id'), region.get('phase'), region.get('reason'),
               visual.get('current_batch_id'), visual.get('current_target_id'), tuple(queue),
               navigation.get('phase'), navigation.get('reason'), recovery.get('active'), recovery.get('kind'),
               motion.get('command_sequence'), motion.get('written_at'),
               motion.get('ack_at'), motion.get('jog_count'), motion.get('reason'),
               motion.get('endpoint_reached'), motion.get('remaining_mm'),
               motion.get('short_stop_count'), motion.get('stop_pose_samples'),
               motion.get('target_motion_exhausted'), motion.get('motion_exhausted_token'))
        if key == previous:
            return
        self._last_fire_transition = key if region.get('active') else None
        row = dict(stamp=snapshot.get('stamp'), command_id=region.get('command_id'),
            phase=region.get('phase'), reason=region.get('reason'), queue=queue,
            current_batch_id=visual.get('current_batch_id'), target_id=visual.get('current_target_id'),
            capture_stamp=visual.get('capture_stamp'), center_error_norm=visual.get('center_error_norm'),
            navigation_phase=navigation.get('phase'), navigation_reason=navigation.get('reason'),
            pump_readback_recovery=recovery, route_progress_mm=navigation.get('route_progress_mm'),
            motion={key: motion.get(key) for key in ('bounds_mm','stop_bounds_mm','stop_bound_quantization_mm','offset_mm','jog_count',
                'command_sequence','written_at','acknowledged_sequence','ack_at','jog_status','reason',
                'pose_reason','pose_failure','motion_failure','endpoint_reached','remaining_mm',
                'short_stop_count','stop_pose_samples','target_motion_exhausted','motion_target_key',
                'target_identity','jog_returning','motion_exhausted_token')})
        self.get_logger().info('fire_centering_transition ' + json.dumps(row, ensure_ascii=False))

    def read_uart(self, now):
        received = 0
        if self.serial is not None:
            try:
                for packet in self.parser.feed(self.serial.read(min(self.serial.in_waiting, 4096))):
                    received += 1
                    if (getattr(self, '_uart_recovering', False) and packet.command == (STOP | 0x80)
                            and packet.sequence == getattr(self, '_uart_stop_sequence', None)
                            and self.in_flight is not None
                            and self.in_flight[:2] == (packet.sequence, STOP)
                            and self.core.pending.get(packet.sequence, (None,))[0] == STOP):
                        self._uart_stop_reply_at = now
                        self._uart_stop_result = packet.data[0]
                        if packet.data[0] == 0:
                            self._uart_stop_ack_at = now
                    if self.in_flight is not None and (packet.sequence, packet.command) == (self.in_flight[0], self.in_flight[1] | 0x80):
                        self.in_flight = None
                    if self.pump().receive(packet, now):
                        pass
                    elif (packet.command & 0x7f) in MAP_COMMANDS:
                        if self.map_telemetry.receive(packet, now):
                            self.core.pending.pop(packet.sequence, None)
                    else:
                        self.core.receive(packet, now)
                    if packet.command == (EXT_STATUS | 0x80) and self.core.status_at == now:
                        # Translate only genuine measured firmware feedback.
                        event = FpgaEvent()
                        event.header.stamp = self.get_clock().now().to_msg()
                        event.event_type = FpgaEvent.STATUS
                        event.vehicle_stopped = bool(self.core.status.get('vehicle_stopped'))
                        self.event_pub.publish(event)
            except Exception as exc:
                self._uart_disconnected('uart_read_failed:' + str(exc), now)
        return received

    def service_pose_burst(self):
        """Finish a pending atomic pose without four executor round trips.

        The existing single writer and all flush guards still arbitrate every
        request. Wait for at most 2 ms per read / an 8 ms budget, with at most
        four complete frames; select sleeps instead of polling. Any non-pose write (including
        STOP/OFF/heartbeat) ends the burst and retains its normal priority.
        """
        deadline = time.monotonic() + .008
        received = 0
        for _ in range(40):
            if received >= 4:
                break
            if (self.serial is None or self.in_flight is None or
                    self.in_flight[1] not in POSE_COMMANDS or
                    getattr(self, '_uart_recovering', False)):
                break
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                break
            try:
                if not self.serial.in_waiting and not select.select(
                        [self.serial], [], [], min(.002, remaining))[0]:
                    break
            except (OSError, ValueError, TypeError):
                # A closed/unsupported descriptor is handled by normal UART
                # recovery; it must never cause a second writer or a retry.
                break
            count = self.read_uart(self.now())
            received += count
            if not count:
                # UART readiness can mean only the first byte of a frame.
                # Keep the parser's partial bytes and wait within the same
                # deadline; do not add another full executor round trip.
                continue
            self.flush()
        return received

    def tick(self):
        now = self.now()
        self._try_uart_open(now)
        self.read_uart(now)
        map_snapshot = self.map_telemetry.snapshot(now, internal_read_only=True)
        self.core.numbered.observe_map(map_snapshot)
        self.core.area.observe_map(dict(map_snapshot,
            stopped=self.core.area.navigation.stopped(now), stopped_stamp=self.core.status_at), now)
        a = self.core.active
        numbered = self.core.numbered.active
        area = self.core.area.navigation.active
        # Read the whole immutable patrol path before START. During motion,
        # coherent pose + cursor snapshots take four UART replies; 5 Hz leaves
        # capacity for heartbeat/status and keeps STOP at higher priority.
        area_active = bool(area and area.get('phase') not in
            ('idle', 'arrived', 'held', 'cancelled', 'failed', 'stopping'))
        retained = bool(area_active and area.get('continuous_route'))
        self.map_telemetry.full_route_required = bool(area_active and
            (retained or area.get('alignment_guard')))
        pose_period = (.2 if retained and area.get('phase') in
            ('running', 'setting_checkpoint_speed', 'pausing', 'paused', 'resuming',
             'centering_starting', 'centering_moving', 'centering_stopping', 'centering_verifying') else .5)
        if pose_period < self.map_telemetry.pose_period_s:
            self.map_telemetry.next_pose = now
        self.map_telemetry.pose_period_s = pose_period
        area_guard_active = bool(area and area['phase'] not in (
            'stopping', 'arrived', 'held', 'cancelled', 'failed',
            'pausing', 'paused', 'confirming_arrival'))
        search_active = bool(self.core.area.search and self.core.area.search.active
                             and self.core.area.search.execute_fire is False)
        search_reason = None
        if search_active:
            search_reason = self.search_pump_guard(self.actuator_guard(now, navigation_only=True), now)
            if search_reason:
                self.core.stop(now, search_reason)
        elif hasattr(self, '_search_pump_recovery'):
            self.search_pump_guard('', now)
        if (a and a['phase'] not in ('stopping', 'completed', 'fault') or
                area_guard_active or
                numbered and numbered['phase'] not in ('stopping', 'arrived', 'cancelled', 'failed',
                                                       'pausing', 'paused', 'resuming', 'confirming_arrival')):
            reason = '' if search_active else self.actuator_guard(now, navigation_only=bool(
                area_guard_active and area.get('execute_fire') is False))
            if reason:
                self.core.stop(now, reason)
        if getattr(self, '_uart_recovering', False):
            self._uart_recovery_tick(now)
        else:
            self.core.tick(now)
        self.pump().tick(now)
        self.flush()
        if self.service_pose_burst():
            now = self.now()
            map_snapshot = self.map_telemetry.snapshot(now, internal_read_only=True)
            self.core.numbered.observe_map(map_snapshot)
            self.core.area.observe_map(dict(map_snapshot,
                stopped=self.core.area.navigation.stopped(now), stopped_stamp=self.core.status_at), now)
        # Bulk map/path traffic stays in otherwise-idle UART slots. Active
        # navigation's bounded pose snapshots are serviced inside flush().
        self.map_telemetry.expire(now)
        if self.serial is None:
            self.map_telemetry.disconnected()
        elif (self.core.protocol_ready and self.core.status_at is not None and
              0 <= now-self.core.status_at <= .6 and self.in_flight is None and not self.tx_queue and
              getattr(self, '_map_queued_request', None) is None):
            request = self.map_telemetry.next_request(now)
            if request:
                self.queue_map_request(request, now)
                self.flush()
        if now-self.last_publish >= .1:
            state = self.extension_state(now)
            self.trace_fire_transition(state)
            publish_json(self.state_pub, state)
            generic_leg = self.core.area.navigation.active or {}
            numbered_leg = self.core.numbered.active or {}
            use_generic = bool(generic_leg and (self.core.area.busy() or
                not self.core.numbered.busy() and
                generic_leg.get('started_at', -1) >= numbered_leg.get('started_at', -1)))
            navigation = (self.core.area.navigation.state(now) if use_generic
                          else self.core.numbered.state(now))
            publish_json(self.navigation_pub, navigation)
            publish_json(self.navigation_proposal_pub, navigation)
            publish_json(self.map_pub, map_snapshot)
            if hasattr(self, 'pump_pub'):
                publish_json(self.pump_pub, self.pump().state(now))
            self.last_publish = now

    def destroy_node(self):
        self._uart_closed = True
        self.core.stop(self.now(), 'shutdown')
        # Drain only the outstanding reply for at most 120 ms. Then issue STOP
        # before closing even when an earlier request never acknowledged.
        if self.serial is not None and self.in_flight is not None:
            deadline=time.monotonic()+.12
            while time.monotonic()<deadline:
                try:
                    packets=self.parser.feed(self.serial.read(min(self.serial.in_waiting,4096)))
                    if any((p.sequence,p.command)==(self.in_flight[0],self.in_flight[1]|0x80) for p in packets):
                        break
                except Exception:
                    break
                time.sleep(.005)
        self.in_flight=None
        self.flush()
        if self.serial is not None:
            self.serial.close()
        return super().destroy_node()


def main():
    spin(ExtensionTask)
