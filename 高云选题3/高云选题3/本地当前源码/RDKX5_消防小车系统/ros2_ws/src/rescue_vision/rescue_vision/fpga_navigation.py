"""Semantic destination handoff. FPGA owns map, planning, localization and driving.

No ROS, route graph, odometry integration or steering computation is used here.
"""
import math
import secrets
from .manual_motion import validate as validate_manual


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def uint32(value):
    return isinstance(value, int) and not isinstance(value, bool) and 0 < value <= 0xffffffff


STATES = {0: 'idle', 1: 'accepted', 2: 'navigating', 3: 'paused',
          4: 'arrived', 5: 'cancelled', 6: 'failed'}


class FpgaNavigator:
    def __init__(self, catalog, session_id=None, status_timeout_s=.6, manual_motion_enabled=False):
        self.buildings = {}
        for item in catalog['buildings']:
            number = int(item['number'])
            if not 1 <= number <= 65535:
                raise ValueError('Building number outside UART uint16 range')
            value = dict(building_id=item['building_id'], building_number=f'{number:02d}',
                         building_name=item['name'])
            for alias in (item['building_id'], str(number), f'{number:02d}', item['name']):
                self.buildings[alias] = value
        self.session_id = session_id or secrets.randbits(32) or 1
        if not uint32(self.session_id):
            raise ValueError('Invalid navigation session ID')
        self.counter = 0
        self.status_timeout_s = status_timeout_s
        self.active = None
        self.mode = 'preview'
        self.status = None
        self.status_received = None
        self.last_sequence = None
        self.pending = None
        self.last_send = None
        self.reason = 'idle'
        self.requested_action = None
        self.seen_commands = set()
        self.manual_motion_enabled = bool(manual_motion_enabled)

    def busy(self):
        if not self.active or self.mode != 'execute':
            return False
        s = self.status or {}
        return not (s.get('state') in ('arrived', 'cancelled', 'failed')
                    and s.get('vehicle_stopped') is True and self.pending is None)

    def cancel_manual(self, now, reason):
        if not self.active or self.active.get('motion_kind') != 'manual' or not self.busy():
            return
        if self.requested_action in ('cancel', 'timeout'):
            return
        self.requested_action = 'cancel'
        self.pending = dict(self.active, action='cancel', requested_at=now, retry_until=now+3., stamp=now)
        self.last_send = None
        self.reason = reason

    def expire_manual(self, now):
        if self.active and self.active.get('motion_kind') == 'manual' and now >= self.active['deadline']:
            self.cancel_manual(now, 'manual_deadline_expired')

    def request(self, request, now):
        cid = request.get('command_id')
        if not isinstance(cid, str) or not 0 < len(cid) <= 100 or cid in self.seen_commands:
            raise ValueError('Missing or repeated command_id')
        issued, ttl = request.get('issued_at'), request.get('ttl_s', 5.)
        if not finite(issued) or not finite(ttl) or not 0 < ttl <= 10 or not 0 <= now-issued <= ttl:
            raise ValueError('Expired or invalid navigation request')
        action = request.get('action', 'navigate')
        if action not in ('navigate', 'manual_move', 'cancel', 'pause', 'resume'):
            raise ValueError('Unsupported navigation action')
        if action in ('navigate', 'manual_move') and self.busy() and (
                action == 'manual_move' or self.active.get('motion_kind') == 'manual'):
            raise ValueError('Cancel current movement and wait for stopped confirmation before changing motion mode')
        if action == 'manual_move':
            if not self.manual_motion_enabled:
                raise ValueError('manual_motion_not_enabled')
            if request.get('mode') != 'execute' or request.get('confirmed') is not True:
                raise ValueError('Manual movement requires explicit execute confirmation')
            motion = validate_manual(request)
            self.counter += 1
            if self.counter > 0xffffffff:
                raise ValueError('Navigation goal IDs exhausted; restart while stopped')
            self.active = dict(motion, motion_kind='manual', command_id=cid, goal_id=self.counter,
                session_id=self.session_id, building_number='0', building_id=None, building_name=None,
                arrival_policy='manual', deadline=now+motion['max_duration_s'])
            self.mode = 'execute'
            self.status = self.status_received = self.last_sequence = None
            self.reason = 'waiting_fpga_manual_acceptance'
        elif action == 'navigate':
            value = str(request.get('building_id', '')).strip()
            if value not in self.buildings:
                raise ValueError('Unknown building: '+value)
            mode = request.get('mode', 'preview')
            policy = request.get('arrival_policy', 'manual')
            if mode not in ('preview', 'execute') or policy not in ('manual', 'auto'):
                raise ValueError('Invalid navigation mode or arrival policy')
            if mode == 'execute' and request.get('confirmed') is not True:
                raise ValueError('Execution requires an explicit navigation command')
            if self.active and self.mode == 'execute' and mode == 'preview':
                raise ValueError('Cancel the active FPGA goal before previewing another destination')
            self.counter += 1
            if self.counter > 0xffffffff:
                raise ValueError('Navigation goal IDs exhausted; restart while stopped')
            self.active = dict(self.buildings[value], command_id=cid, goal_id=self.counter,
                               session_id=self.session_id, arrival_policy=policy, motion_kind='navigation')
            self.mode = mode
            self.status = None
            self.status_received = None
            self.last_sequence = None
            self.reason = 'destination_preview' if mode == 'preview' else 'waiting_fpga_acceptance'
        elif self.active is None:
            self.reason = 'idle'
        elif action == 'resume' and self.active.get('motion_kind') == 'manual':
            raise ValueError('Manual movement cannot resume; cancel and issue a new bounded request')
        elif action == 'resume' and (not self.status or self.status.get('state') != 'paused'):
            raise ValueError('Resume requires FPGA confirmation of the paused goal')
        self.seen_commands.add(cid)
        if len(self.seen_commands) > 4096:
            # Preserve recent IDs without unbounded growth; wire session/goal still rejects old arrivals.
            self.seen_commands = {cid}
        self.requested_action = action
        self.last_send = None
        if self.active and self.mode == 'execute':
            self.pending = dict(self.active, action=action, requested_at=now,
                                retry_until=now+3., stamp=now)
        else:
            self.pending = None
        if action in ('cancel', 'pause', 'resume') and self.active:
            self.reason = 'waiting_fpga_'+action
        return self.state(now)

    def next_command(self, now):
        self.expire_manual(now)
        if not self.pending:
            return None
        if now > self.pending['retry_until']:
            self.pending = None
            self.reason = 'fpga_command_ack_timeout'
            self.requested_action = 'timeout'
            return None
        if self.last_send is not None and now-self.last_send < .4:
            return None
        self.last_send = now
        return {**self.pending, 'stamp': now}

    def observe(self, status, now):
        self.expire_manual(now)
        if self.active is None or self.mode != 'execute':
            return False
        if self.requested_action == 'timeout':
            return False
        a = self.active
        if status.get('session_id') != a['session_id'] or status.get('goal_id') != a['goal_id']:
            return False
        if status.get('building_number') != int(a['building_number']):
            return False
        if a.get('motion_kind') == 'manual' and (
                status.get('motion_kind') != 'manual' or type(status.get('obstacle_clear')) is not bool):
            return False
        stamp = status.get('stamp')
        if not finite(stamp) or not 0 <= now-stamp <= self.status_timeout_s:
            return False
        seq = status.get('status_sequence')
        if not isinstance(seq, int) or isinstance(seq, bool) or not 0 <= seq <= 0xffffffff:
            return False
        if self.last_sequence is not None and not 0 < ((seq-self.last_sequence) & 0xffffffff) < 0x80000000:
            return False
        state = status.get('state')
        if state not in STATES.values():
            return False
        if type(status.get('vehicle_stopped')) is not bool or type(status.get('estop')) is not bool:
            return False
        # Cancellation/pause take effect locally at request time; later in-flight ARRIVED cannot undo them.
        if self.requested_action == 'cancel' and state not in ('cancelled', 'failed'):
            return False
        if self.requested_action == 'pause' and state not in ('paused', 'failed', 'cancelled'):
            return False
        cancel_ack = self.requested_action == 'cancel' and state == 'cancelled'
        if self.status and self.status['state'] in ('arrived', 'cancelled', 'failed') and state != self.status['state'] and not cancel_ack:
            return False
        self.status = dict(status)
        self.status_received = now
        self.last_sequence = seq
        self.reason = 'fpga_'+state
        expected = {'navigate': ('accepted', 'navigating', 'arrived', 'failed'),
                    'manual_move': ('accepted', 'navigating', 'arrived', 'failed'),
                    'pause': ('paused', 'failed', 'cancelled'),
                    'resume': ('accepted', 'navigating', 'arrived', 'failed'),
                    'cancel': ('cancelled', 'failed')}
        if self.pending and state in expected[self.pending['action']]:
            self.pending = None
        if a.get('motion_kind') == 'manual' and state in ('accepted', 'navigating') and (
                not status['obstacle_clear'] or status['estop'] or status.get('error_code', 0)):
            self.cancel_manual(now, 'manual_direction_obstacle_estop_or_error')
        return True

    def proposal(self, now):
        self.expire_manual(now)
        fresh = self.status_received is not None and 0 <= now-self.status_received <= self.status_timeout_s
        s = self.status or {}
        terminal = self.requested_action in ('cancel', 'pause', 'timeout')
        valid = self.mode == 'execute' and fresh and not s.get('estop') and not terminal
        if self.active and self.active.get('motion_kind') == 'manual':
            valid = valid and s.get('obstacle_clear') is True and now < self.active['deadline']
        arrived = valid and s.get('state') == 'arrived'
        moving = valid and s.get('state') in ('accepted', 'navigating') and self.pending is None
        result = dict(stamp=now, owner='fpga', mode=self.mode,
                    **(self.active or dict(command_id=None, goal_id=None, building_id=None,
                                         building_number=None, building_name=None,
                                         session_id=self.session_id, arrival_policy='manual')),
                    valid=bool(valid and (moving or arrived)), ready_to_move=bool(moving),
                    arrived=bool(arrived), vehicle_stopped=bool(fresh and s.get('vehicle_stopped')),
                    estop=bool(fresh and s.get('estop')), status_fresh=fresh,
                    reason=self.reason if fresh or self.mode == 'preview' else 'fpga_navigation_status_unavailable',
                    freshness_s=now-self.status_received if self.status_received is not None else None,
                    route_revision=0, edge_id=None, turn=None,
                    localization_owner='fpga', planning_owner='fpga', line_tracking_required=False)
        result.setdefault('distance_m', None)
        return result

    def state(self, now):
        return dict(stamp=now, owner='fpga', route=None, route_computed_on_x5=False,
                    **(self.active or {}), mode=self.mode, reason=self.reason,
                    arrived=self.proposal(now)['arrived'], proposal=self.proposal(now),
                    fpga_status=self.status, pending_action=self.pending['action'] if self.pending else None,
                    localization=dict(source='fpga', valid=None, computed_on_x5=False))
