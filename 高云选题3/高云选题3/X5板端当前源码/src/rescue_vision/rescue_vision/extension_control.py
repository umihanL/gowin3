"""Deterministic extension task owner for the existing ov5640 FPGA firmware.

Commands are accepted separately from physical completion. No ROS or I/O here.
"""
import math
import struct
from . import ov5640_protocol as wire
from .numbered_navigation import NumberedNavigation
from .region_control import RegionControl


def fresh(value, now, ttl=.6):
    stamp = value.get('stamp') if isinstance(value, dict) else None
    return type(stamp) in (float, int) and math.isfinite(stamp) and 0 <= now-stamp <= ttl


def start_guard(extension, now, capability):
    if not fresh(extension, now) or extension.get('real_uart') is not True:
        return 'real_extension_uart_required'
    if extension.get('protocol_ready') is not True or extension.get('status_fresh') is not True:
        return 'extension_firmware_or_feedback_unavailable'
    if (extension.get('capabilities') or {}).get(capability) is not True:
        return 'fpga_capability_not_implemented_' + capability
    if extension.get('can_move') is not True:
        return extension.get('reason') or 'extension_not_ready'
    return ''


def zebra_switch_guard(extension, now):
    """A waiting red-light task may disable detection, but never release motion."""
    if not fresh(extension,now) or extension.get('real_uart') is not True:
        return 'real_extension_uart_required'
    if extension.get('protocol_ready') is not True or extension.get('status_fresh') is not True:
        return 'extension_firmware_or_feedback_unavailable'
    if (extension.get('capabilities') or {}).get('zebra_switch') is not True:
        return 'fpga_capability_not_implemented_zebra_switch'
    if extension.get('vehicle_stopped') is not True:
        return 'measured_stop_required_for_zebra_switch'
    if extension.get('mode_transition') or extension.get('zebra_transition'):
        return 'extension_transition_in_progress'
    active=extension.get('active') or {}
    if active.get('phase') not in (None,'completed','fault') and not extension.get('traffic_waiting'):
        return 'stop_current_movement_before_zebra_switch'
    return ''


def dialogue_allowed(extension, now):
    return bool(fresh(extension,now) and extension.get('dialogue_enabled') is True and
                extension.get('task') in ('vision','dialogue','semantic'))


class CenterConfirmation:
    """Only consecutive distinct exposures of the SAME tracked flame count."""
    def __init__(self):
        self.identity = None
        self.last_capture = None
        self.hits = 0
        self.center_error_norm = None
        self.last_seen = None

    def observe(self, document, now):
        stamp = document.get('capture_stamp')
        if not document.get('valid') or not fresh({'stamp': stamp}, now, .3):
            self.hits = 0
            return False
        if self.last_capture is not None and stamp <= self.last_capture:
            return False
        if self.last_capture is not None and stamp-self.last_capture > .3:
            self.hits = 0
        self.last_capture = stamp
        size = document.get('image_size_wh', [])
        if len(size) != 2 or any(type(x) is not int or x < 1 for x in size):
            self.hits = 0
            return False
        candidates = []
        for target in document.get('targets', []):
            box = target.get('bbox_xyxy', [])
            if (target.get('selectable') is not True or type(target.get('target_id')) is not int
                    or len(box) != 4 or any(type(v) not in (float, int) or not math.isfinite(v) for v in box)):
                continue
            x1, y1, x2, y2 = box
            if not (0 <= x1 < x2 <= size[0] and 0 <= y1 < y2 <= size[1]):
                continue
            identity = (document.get('registry_session'), target['target_id'])
            error = ((x1+x2)/2/size[0]-.5, (y1+y2)/2/size[1]-.5)
            candidates.append((identity, error))
        # Select one stable track once; losing it cannot silently switch targets.
        if self.identity is None and candidates:
            candidates.sort(key=lambda item: abs(item[1][0]))
            self.identity = candidates[0][0]
        selected = next((item for item in candidates if item[0] == self.identity), None)
        if selected is None:
            self.hits = 0
            return False
        self.center_error_norm = selected[1]
        self.last_seen=stamp
        # Longitudinal movement can only align horizontal image position. The
        # vertical component is reported, not claimed as nozzle alignment.
        self.hits = self.hits+1 if abs(selected[1][0]) <= .08 else 0
        return self.hits >= 3


class ExtensionControl:
    def __init__(self, real_uart=False, side_camera_verified=False, screen_x_per_forward_sign=0,
                 fixed_traffic_camera_verified=False):
        self.real_uart = bool(real_uart)
        self.side_camera_verified = bool(side_camera_verified)
        self.screen_x_per_forward_sign = screen_x_per_forward_sign
        self.fixed_traffic_camera_verified = bool(fixed_traffic_camera_verified)
        self.task = 'vision'
        self.capabilities = {}
        self.protocol_ready = False
        self.status = {}
        self.status_at = None
        self.status_request_at = None
        self.last_result = None
        self.active = None
        self.sequence = 0
        self.used_motion = set()
        self.pending = {}
        self.outbox = []
        self.effects = []
        self.last_poll = -math.inf
        self.last_caps = -math.inf
        self.reason = 'real_extension_uart_required'
        self.center = CenterConfirmation()
        self.seen = set()
        self.traffic_session = None
        self.last_light = -math.inf
        self.last_vision_frame = None
        self.reference = {'confirmed': False}
        self.route_catalog = None
        self.route_steps = []
        self.traffic_wait_latched = False
        self.traffic_wait_started = None
        self.traffic_cycle_seen = False
        self.mode_transition=None
        self.fpga_task_mode=None
        self.zebra_transition=None
        self.numbered = NumberedNavigation(self)
        self.actuator = {}
        self.auto_state = {}
        self.mission_state = {}
        self.area = RegionControl(self)

    def send(self, command, data=b'\0\0\0\0', *, now, seq=None):
        if seq is None:
            for _ in range(256):
                self.sequence = (self.sequence+1) & 255
                if self.sequence not in self.pending and (not self.active or self.sequence != self.active['request_sequence']):
                    break
            seq = self.sequence
        self.pending[seq] = (command, now)
        self.outbox.append(wire.encode(seq, command, data))
        return seq

    def _motion_sequence(self):
        for seq in range(1, 256):
            if seq not in self.used_motion and seq not in self.pending:
                self.used_motion.add(seq)
                return seq
        raise ValueError('motion_identity_exhausted_restart_while_stopped')

    def state(self, now):
        recent = self.status_at is not None and 0 <= now-self.status_at <= .6
        busy = bool(self.area.busy() or self.numbered.busy() or self.mode_transition or self.zebra_transition or self.active and self.active['phase'] not in ('completed', 'fault'))
        stopped = recent and self.status.get('vehicle_stopped') is True
        ready = self.real_uart and self.protocol_ready and recent and stopped and not busy
        # mode_selected reflects the physical FPGA host/drive page permission.
        ready = ready and self.status.get('mode_selected') is True
        reason = ('real_extension_uart_required' if not self.real_uart else
                  'extension_firmware_unavailable' if not self.protocol_ready else
                  'extension_feedback_stale' if not recent else
                  'extension_movement_in_progress' if busy else
                  'fpga_physical_host_mode_required' if not self.status.get('mode_selected') else
                  'measured_stop_required' if not stopped else 'ready')
        result=dict(stamp=now, task=self.task, real_uart=self.real_uart,
                    dialogue_enabled=self.task in ('vision','dialogue','semantic'),
                    protocol_ready=self.protocol_ready, status_fresh=recent,
                    vehicle_stopped=bool(stopped), can_move=bool(ready), busy=busy,
                    capabilities=dict(self.capabilities), reason=reason,
                    fpga_task_mode=self.fpga_task_mode,mode_transition=self.mode_transition,
                    zebra_transition=dict(self.zebra_transition) if self.zebra_transition else None,
                    zebra_detection_enabled=(self.status.get('zebra_detection_enabled') if recent and
                        self.protocol_ready and self.capabilities.get('zebra_switch') is True else None),
                    active=dict(self.active) if self.active else None,
                    last_result=self.last_result, fpga_status=dict(self.status),
                    fpga_host=dict(selected=self.numbered.host.get('selected') is True,
                        scope=self.numbered.host.get('scope') is True,
                        link_ok=self.numbered.host.get('link_ok') is True,
                        fresh=fresh(dict(stamp=self.numbered.host_at), now)),
                    side_camera_verified=self.side_camera_verified,
                    screen_x_per_forward_sign=self.screen_x_per_forward_sign,
                    fixed_traffic_camera_verified=self.fixed_traffic_camera_verified,
                    horizontal_centered=bool(self.center.hits>=3), aim_locked=False,
                    target_id=self.center.identity[1] if self.center.identity else None,
                    target_registry_session=self.center.identity[0] if self.center.identity else None,
                    center_error_norm=self.center.center_error_norm,
                    center_confirmations=self.center.hits,
                    reference=dict(self.reference),
                    traffic_waiting=bool(recent and self.status.get('zebra_wait')))
        zebra_reason=zebra_switch_guard(result,now)
        result.update(zebra_can_change=not bool(zebra_reason),zebra_change_reason=zebra_reason)
        point_reason = self.numbered.readiness(now)
        point_reference = dict(self.numbered.reference)
        current_heading = self.numbered.reference_heading(point_reference.get('point_id'), now)
        point_reference.update(current_heading_deg=current_heading,
                               heading_confirmed=current_heading is not None and bool(stopped))
        result.update(point_reference=point_reference,
                      point_start_reference=self.numbered.start_reference(now),
                      numbered_navigation=self.numbered.state(now),
                      numbered_point_supported=self.numbered.supported,
                      numbered_heading_supported=self.numbered.heading_supported,
                      can_confirm_point_start=not bool(self.numbered.readiness(now, require_reference=False)),
                      can_navigate_point=not bool(point_reason), point_navigation_reason=point_reason)
        region_reason = self.area.readiness(now)
        region_observe_reason = self.area.readiness(now, execute_fire=False)
        result.update(region_ready=not bool(region_reason), region_reason=region_reason,
                      region_observe_ready=not bool(region_observe_reason),
                      region_observe_reason=region_observe_reason,
                      region_search=self.area.snapshot(now), region_reference=self.area.reference(now),
                      semantic_regions=self.area.catalog)
        return result

    def change_mode_after_stop(self, mode, now):
        if self.real_uart:
            self.mode_transition=dict(mode=mode,started_at=now,phase='waiting_stop',sequence=None)

    def request(self, request, now):
        cid, action = request.get('command_id'), request.get('action')
        if not isinstance(cid, str) or not 0 < len(cid) <= 100:
            raise ValueError('invalid_extension_command_id')
        if action in ('stop', 'finish_task', 'stop_spray'):
            if action != 'stop_spray':
                self.stop(now, 'operator_stop')
                self.task = ('vision' if action=='finish_task' else
                             'dialogue' if self.task=='semantic' else self.task)
                if self.real_uart:
                    self.change_mode_after_stop(0,now)
            return
        if cid in self.seen:
            raise ValueError('duplicate_extension_request')
        if 'patrol_fire' in request or 'fire_ignore_s' in request:
            raise ValueError('along_route_fire_replaced_by_semantic_region_search')
        if not fresh({'stamp': request.get('issued_at')}, now, min(5., request.get('ttl_s', 3.))):
            raise ValueError('expired_extension_request')
        if action == 'seek_fire' and request.get('after_fire', 'manual') != 'manual':
            raise ValueError('manual_spray_command_required_for_extension_tasks')
        if action == 'confirm_point_start':
            self.numbered.confirm_start(request, now)
            self.seen.add(cid)
            self.last_result = dict(command_id=cid, accepted=True, completed=True,
                                    reason='physical_point_reference_confirmed', stamp=now)
            return
        elif action == 'navigate_point':
            self.numbered.start(request, now)
        elif action == 'search_region':
            self.area.start(request, now)
        elif action == 'set_reference':
            reason = start_guard(self.state(now), now, 'route_turn')
            if reason:
                raise ValueError(reason)
            if request.get('confirmed') is not True:
                raise ValueError('physical_start_placement_confirmation_required')
            reference = {key: request.get(key) for key in ('x_cm', 'y_cm', 'heading_deg')}
            if not all(type(v) in (float,int) and math.isfinite(v) for v in reference.values()):
                raise ValueError('invalid_reference')
            if not self.route_catalog:
                raise ValueError('map_catalog_unavailable')
            if not (0 <= reference['x_cm'] < self.route_catalog['width_cm'] and
                    0 <= reference['y_cm'] < self.route_catalog['height_cm'] and -180 <= reference['heading_deg'] <= 360):
                raise ValueError('reference_outside_map')
            self.reference = dict(reference, confirmed=True, stamp=now)
            self.numbered.invalidate_reference('operator_map_reference_confirmed')
        elif action in ('extension_vision','extension_dialogue'):
            self.stop(now, 'visual_task_selected')
            self.task = 'dialogue' if action=='extension_dialogue' else 'vision'
            if self.real_uart:
                self.change_mode_after_stop(1,now)
            self.effects.append(('mission', {'mode': 'inspect'}))
        elif action=='set_zebra_detection':
            reason=zebra_switch_guard(self.state(now),now)
            if reason:raise ValueError(reason)
            enabled=request.get('enabled')
            if type(enabled) is not bool:raise ValueError('zebra_enabled_must_be_boolean')
            seq=self.send(wire.ZEBRA_ENABLE,bytes((int(enabled),0,0,0)),now=now)
            self.traffic_cycle_seen=False
            self.zebra_transition=dict(command_id=cid,enabled=enabled,phase='waiting_ack',
                sequence=seq,started_at=now,ack_at=None)
        elif action == 'traffic_task':
            self.stop(now, 'traffic_task_selected')
            self.task = 'traffic'
            self.traffic_wait_latched = False
            if self.real_uart:
                self.change_mode_after_stop(3,now)
            self.effects.append(('mission', dict(mode='traffic', request_id=cid, issued_at=now, ttl_s=2.)))
        elif action=='traffic_drive':
            reason=start_guard(self.state(now),now,'traffic')
            if reason:raise ValueError(reason)
            if self.task!='traffic' or self.fpga_task_mode!=3:
                raise ValueError('select_traffic_task_and_wait_ready_before_drive')
            if self.state(now)['zebra_detection_enabled'] is not True:
                raise ValueError('enable_fpga_zebra_detection_before_traffic_drive')
            seq=self._motion_sequence()
            self.numbered.invalidate_reference('traffic_movement_requested')
            self.reference['confirmed']=False
            self.active=dict(command_id=cid,action=action,phase='waiting_traffic_speed',
                request_sequence=seq,started_at=now,deadline=now+120.,after_fire='manual',
                wire_command=3,payload_hex=bytes((0,seq,0,0)).hex(),stop_reason=None,
                speed_sequence=self.send(0x02,struct.pack('<HH',100,0),now=now))
        elif action in ('manual_move', 'route_turn', 'seek_fire'):
            cap = {'manual_move': 'distance', 'route_turn': 'route_turn', 'seek_fire': 'velocity'}[action]
            reason = start_guard(self.state(now), now, cap)
            if reason:
                raise ValueError(reason)
            if action == 'seek_fire' and (not self.side_camera_verified or
                    type(self.screen_x_per_forward_sign) is not int or self.screen_x_per_forward_sign not in (-1,1)):
                raise ValueError('verify_fixed_side_camera_before_seek')
            if action == 'seek_fire' and (self.last_vision_frame is None or not 0 <= now-self.last_vision_frame <= .3):
                raise ValueError('fresh_yolo_exposure_required_for_seek')
            if action == 'manual_move':
                data = wire.move_payload(request.get('direction'), request.get('distance_m'))
                kind, duration = wire.MOVE, request['distance_m']/.1+5.
            elif action == 'route_turn':
                from .semantic_routes import build_junction_route
                if self.reference.get('confirmed') is not True:
                    raise ValueError('confirm_physical_start_before_junction_route')
                route = build_junction_route(self.route_catalog, self.reference,
                    request.get('junction_count'), request.get('turn'))
                self.route_steps = [(wire.SET_START, bytes((*route['start'],0,0)))]
                self.route_steps += [(wire.WAYPOINT, bytes((*p,0,0))) for p in route['waypoints']]
                self.route_steps += [(wire.SET_GOAL, bytes((*route['goal'],0,0))), (wire.PLAN, bytes(4))]
                data, kind, duration = bytes(4), 0x03, 120.
            else:
                if request.get('after_fire', 'manual') not in ('manual', 'auto'):
                    raise ValueError('invalid_after_fire')
                data, kind, duration = wire.velocity_payload(60), wire.VELOCITY, 30.
            seq = self._motion_sequence()
            self.numbered.invalidate_reference('extension_movement_requested')
            self.task = 'semantic'
            self.active = dict(command_id=cid, action=action, phase='waiting_mode',
                request_sequence=seq, started_at=now, deadline=now+duration,
                after_fire=request.get('after_fire', 'manual'), wire_command=kind,
                payload_hex=data.hex(), last_velocity=now, mode_sequence=None,
                stop_reason=None, completion_received=False)
            self.center = CenterConfirmation()
            self.reference['confirmed'] = False
            if action == 'route_turn':
                self.active['route'] = route
                self.active['payload_hex'] = bytes((1,seq,0,0)).hex()
            self.active['mode_sequence'] = self.send(wire.TASK_MODE, bytes((2, 0, 0, 0)), now=now)
            self.effects.append(('mission', {'mode': 'navigate', 'arrival_policy': 'manual'}))
        else:
            raise ValueError('unsupported_extension_action')
        self.seen.add(cid)
        self.last_result = dict(command_id=cid, accepted=True, completed=False,
                                reason='request_accepted_waiting_hardware', stamp=now)

    def stop(self, now, reason):
        stop_seq=None
        if self.real_uart:
            stop_seq=self.send(wire.STOP, now=now)
        self.numbered.cancel(now, reason, stop_seq)
        self.area.cancel(now, reason)
        if self.active and self.active['phase'] not in ('completed', 'fault'):
            self.active.update(phase='stopping', stop_reason=reason, stop_at=now,
                               stop_sequence=stop_seq,stop_ack_at=None,last_stop=now)

    def invalidate_start_references(self, reason):
        """A measured move consumes every operator-declared origin."""
        self.numbered.invalidate_reference(reason)
        self.reference['confirmed'] = False
        self.area.consumed_panel_reference = (
            self.area.navigation.map.get('start_reference') or {}).get('generation')

    def observe_targets(self, document, now):
        self.numbered.patrol.observe(document, now)
        self.area.observe_targets(document, now)
        if fresh({'stamp':document.get('capture_stamp')}, now, .3):
            self.last_vision_frame = document['capture_stamp']
        if self.active and self.active['action'] == 'seek_fire' and self.active['phase'] == 'seeking':
            if self.center.observe(document, now):
                self.stop(now, 'fire_horizontal_center_confirmed')
            elif self.center.center_error_norm is not None:
                error = self.center.center_error_norm[0]
                # Forward-only search must not drive farther once the selected
                # target is already beyond the center in the measured direction.
                if error*self.screen_x_per_forward_sign > .10:
                    self.stop(now, 'target_passed_center_stop_reposition_required')
                self.active['payload_hex'] = wire.velocity_payload(30 if abs(error)<.18 else 60).hex()

    def receive(self, packet, now):
        if self.area.navigation.on_event(packet, now):
            return
        if self.numbered.on_event(packet, now):
            return
        if packet.command in (wire.DONE, wire.FAILED, wire.LINK_TIMEOUT):
            if not self.active or packet.data[0] != self.active['request_sequence']:
                return
            if packet.command == wire.DONE and packet.data[1] == 3:
                self.active['completion_received'] = True
                if self.active['phase'] not in ('stopping', 'completed', 'fault'):
                    self.active.update(phase='stopping', stop_reason='fpga_distance_completed', stop_at=now)
            elif packet.command == wire.FAILED:
                self.stop(now, 'fpga_fault_' + str(packet.data[1]))
            return
        if packet.command == wire.ZEBRA:
            if self.task == 'traffic' and packet.data[1] == 1:
                self.traffic_session = packet.data[0]
                self.effects.append(('mission', dict(mode='traffic', request_id='fpga-zebra-'+str(now), issued_at=now, ttl_s=2.)))
            return
        pending = self.pending.get(packet.sequence)
        if pending is None or packet.command != (pending[0] | 0x80) or not 0 <= now-pending[1] <= .6:
            return
        self.pending.pop(packet.sequence,None)
        self.numbered.on_reply(packet, pending[1], now)
        self.area.navigation.on_reply(packet, pending[1], now)
        if packet.command == (wire.CAPABILITIES | 0x80):
            self.capabilities = wire.capabilities(packet.data)
            self.protocol_ready = True
            self.fpga_task_mode=packet.data[3]
        elif packet.command == (wire.EXT_STATUS | 0x80):
            old_zebra=self.status.get('zebra_detection_enabled')
            self.status, self.status_at = wire.status(packet.data), now
            self.status_request_at=pending[1]
            if self.status.get('vehicle_stopped') is False:
                self.invalidate_start_references('observed_vehicle_movement')
            waiting=self.status['zebra_wait'] or self.status['phase']==6
            zebra_changed=old_zebra is not self.status.get('zebra_detection_enabled')
            if zebra_changed:self.traffic_cycle_seen=False
            if self.task=='traffic' and waiting and (not self.traffic_wait_latched or zebra_changed):
                self.traffic_wait_started=now
                self.traffic_cycle_seen=False
                self.effects.append(('mission',dict(mode='reset_traffic')))
                self.effects.append(('mission',dict(mode='traffic',request_id='fpga-wait-'+str(now),issued_at=now,ttl_s=2.)))
            self.traffic_wait_latched=waiting
        elif self.zebra_transition and packet.sequence==self.zebra_transition['sequence']:
            transition=self.zebra_transition
            if packet.data[0]==0:
                transition.update(phase='waiting_readback',ack_at=now)
            else:
                self.last_result=dict(command_id=transition['command_id'],accepted=False,completed=False,
                    reason='fpga_zebra_switch_rejected_'+str(packet.data[0]),stamp=now)
                self.zebra_transition=None
        elif self.active and packet.sequence==self.active.get('stop_sequence') and pending[0]==wire.STOP:
            if packet.data[0]==0:self.active['stop_ack_at']=now
        elif self.mode_transition and packet.sequence==self.mode_transition.get('sequence'):
            if packet.data[0]==0:
                self.fpga_task_mode=self.mode_transition['mode']
                self.mode_transition.update(phase='waiting_idle',ack_at=now)
            else:
                self.last_result=dict(accepted=False,completed=False,reason='fpga_task_mode_rejected_'+str(packet.data[0]),stamp=now)
                self.mode_transition=None
        elif self.active and packet.sequence == self.active.get('mode_sequence'):
            if packet.data[0] != 0:
                self.stop(now, 'fpga_mode_rejected_' + str(packet.data[0]))
            elif self.active['phase'] == 'waiting_mode':
                self.fpga_task_mode=2
                self.active['phase'] = 'waiting_mode_stop'
                self.active['mode_ack_at']=now
        elif self.active and packet.sequence == self.active.get('route_upload_sequence'):
            if packet.data[0] != 0:
                self.stop(now, 'route_upload_rejected_' + str(packet.data[0]))
            elif self.active['phase'] == 'uploading_route':
                self._next_route_step(now)
        elif self.active and packet.sequence==self.active.get('speed_sequence'):
            if packet.data[0]!=0:self.stop(now,'traffic_speed_rejected_'+str(packet.data[0]))
            elif self.active['phase']=='waiting_traffic_speed':
                self.active['phase']='waiting_ack'
                self.send(0x03,bytes.fromhex(self.active['payload_hex']),now=now,seq=self.active['request_sequence'])
        elif self.active and packet.sequence == self.active['request_sequence']:
            if packet.data[0] != 0:
                self.stop(now, 'fpga_command_rejected_' + str(packet.data[0]))
            elif self.active['phase'] == 'waiting_ack':
                self.active['phase'] = 'seeking' if self.active['action'] == 'seek_fire' else 'moving'
                self.active['accepted_at']=now

    def _next_route_step(self, now):
        if self.route_steps:
            command, data = self.route_steps.pop(0)
            self.active['phase'] = 'uploading_route'
            self.active['upload_sent_at'] = now
            self.active['route_upload_sequence'] = self.send(command, data, now=now)
        else:
            self.active['phase'] = 'planning'
            self.active['planning_at'] = now

    def observe_traffic(self, document, now):
        # Never use preview labels to release the car. Mission's controller has
        # already applied repeated green / fresh frame / settled pose checks.
        if self.task != 'traffic' or not self.state(now)['traffic_waiting'] or now-self.last_light < .15:
            return
        stamp = document.get('published_at')
        if not fresh({'stamp': stamp}, now, .3):
            return
        if self.traffic_wait_started is None:return
        if stamp>=self.traffic_wait_started and document.get('state')=='WAIT_STOP':
            self.traffic_cycle_seen=True
        green = (self.zebra_transition is None and self.state(now)['zebra_detection_enabled'] is True and
                 document.get('move_allowed') is True and document.get('state') == 'RELEASED'
                 and self.traffic_cycle_seen and stamp>=self.traffic_wait_started+2.)
        self.send(wire.LIGHT, bytes((2 if green else 0, 0, 0, 0)), now=now)
        self.last_light = now

    def tick(self, now):
        self.pending = {seq: value for seq, value in self.pending.items() if now-value[1] <= .6}
        if self.real_uart and now-self.last_poll >= .2:
            self.send(wire.EXT_STATUS, now=now)
            self.send(wire.HEARTBEAT, now=now)
            self.last_poll = now
        if self.real_uart and not self.protocol_ready and now-self.last_caps >= 1.:
            self.send(wire.CAPABILITIES, now=now)
            self.last_caps = now
        self.numbered.tick(now)
        self.area.tick(now)
        zebra=self.zebra_transition
        if zebra:
            if (zebra['phase']=='waiting_readback' and self.status_request_at is not None and
                    self.status_request_at>=zebra['ack_at'] and self.status_at is not None and
                    0<=now-self.status_at<=.6 and self.status.get('zebra_detection_enabled') is zebra['enabled']):
                self.last_result=dict(command_id=zebra['command_id'],accepted=True,completed=True,
                    reason='fpga_zebra_state_verified',stamp=now)
                self.zebra_transition=None
            elif now-zebra['started_at']>2.:
                self.last_result=dict(command_id=zebra['command_id'],accepted=False,completed=False,
                    reason='fpga_zebra_switch_readback_timeout',stamp=now)
                self.zebra_transition=None
        transition=self.mode_transition
        if transition:
            measured=(self.status_at is not None and 0<=now-self.status_at<=.6 and
                      self.status_request_at is not None and self.status.get('vehicle_stopped'))
            if transition['phase']=='waiting_stop' and measured and self.status_request_at>=transition['started_at']:
                transition['sequence']=self.send(wire.TASK_MODE,bytes((transition['mode'],0,0,0)),now=now)
                transition['phase']='waiting_ack'
            elif transition['phase']=='waiting_idle' and measured and self.status_request_at>=transition['ack_at'] and self.status.get('phase')==0:
                self.mode_transition=None
            elif now-transition['started_at']>3.:
                self.last_result=dict(accepted=False,completed=False,reason='fpga_task_mode_transition_timeout',stamp=now)
                self.mode_transition=None
        a = self.active
        if not a:
            return
        if a['phase']=='completed':
            # Completing navigation or centering never arms water output.
            # The operator selects a target, then issues a separate spray command.
            return
        if a['phase']=='fault':return
        state = self.state(now)
        if a['phase'] != 'stopping':
            if not state['status_fresh']:
                self.stop(now, 'fpga_feedback_lost')
            elif a['action'] == 'seek_fire' and (self.last_vision_frame is None or now-self.last_vision_frame > .5):
                self.stop(now, 'yolo_camera_exposure_lost')
            elif (a['action']=='seek_fire' and self.center.identity is not None and
                  self.center.last_seen is not None and now-self.center.last_seen>.6):
                self.stop(now, 'selected_fire_track_lost')
            elif now >= a['deadline']:
                self.stop(now, 'extension_deadline_expired')
            elif a['phase'] in ('moving','seeking') and self.status.get('phase') in (5,9,12):
                self.stop(now,'fpga_motion_fault')
            elif (a['phase']=='moving' and self.status.get('phase') in (4,11) and
                    self.status_request_at is not None and self.status_request_at>=a.get('accepted_at',math.inf)):
                self.stop(now,'fpga_distance_completed')
            elif a['phase'] in ('waiting_mode', 'waiting_ack', 'waiting_traffic_speed') and now-a['started_at'] > 1.2:
                self.stop(now, 'fpga_ack_timeout')
            elif a['phase']=='waiting_mode_stop':
                if (self.status_request_at is not None and self.status_request_at>=a['mode_ack_at'] and
                        self.status.get('vehicle_stopped') and self.status.get('phase')==0):
                    if a['action']=='route_turn':self._next_route_step(now)
                    else:
                        a['phase']='waiting_ack';a['started_at']=now
                        self.send(a['wire_command'],bytes.fromhex(a['payload_hex']),now=now,seq=a['request_sequence'])
                elif now-a['mode_ack_at']>2.:
                    self.stop(now,'fpga_mode_stop_not_confirmed')
            elif a['phase'] == 'uploading_route' and now-a['upload_sent_at'] > .8:
                self.stop(now, 'fpga_route_upload_timeout')
            elif a['phase'] == 'planning':
                if self.status.get('phase') == 8 and self.status_at >= a['planning_at']:
                    a['phase'] = 'waiting_ack'
                    a['started_at'] = now
                    self.send(0x03, bytes.fromhex(a['payload_hex']), now=now, seq=a['request_sequence'])
                elif self.status.get('phase') in (9,12):
                    self.stop(now, 'fpga_route_planning_failed')
                elif now-a['planning_at'] > 15.:
                    self.stop(now, 'fpga_route_planning_timeout')
            elif a['phase'] == 'seeking' and now-a['last_velocity'] >= .2:
                self.send(wire.VELOCITY, bytes.fromhex(a['payload_hex']), now=now, seq=a['request_sequence'])
                a['last_velocity'] = now
        elif (state['vehicle_stopped'] and self.status_request_at is not None and
                self.status_request_at >= a.get('stop_at', now)):
            complete = a['stop_reason'] in ('fire_horizontal_center_confirmed', 'fpga_distance_completed')
            a['phase'] = 'completed' if complete else 'fault'
            a['completed_at']=now
            self.last_result = dict(command_id=a['command_id'], accepted=True, completed=complete,
                                    reason=a['stop_reason'], stamp=now)
            if self.task=='semantic':
                self.effects.append(('mission', {'mode': 'inspect'}))
        elif now-a.get('stop_at', now) > 2.:
            self.last_result = dict(command_id=a['command_id'], accepted=True, completed=False,
                                    reason='stop_requested_no_measured_stop', stamp=now)
        if (a['phase'] == 'stopping' and self.real_uart and a.get('stop_ack_at') is None and
                now-a.get('last_stop', -math.inf) >= .5):
            a['stop_sequence']=self.send(wire.STOP, now=now)
            a['last_stop'] = now
