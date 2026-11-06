"""Explicit automatic-run authority at the actuator boundary; no ROS or I/O.

Only a fresh /fire/auto/request start can authorize a run. State publications
and pump commands cannot grant that authority. A cancel retires the authority
before any queued arm is evaluated, even across different ROS publishers.
"""
from collections import OrderedDict
import math


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class AutoArmFence:
    def __init__(self, max_run_s=90.):
        if not _finite(max_run_s) or not 5 <= max_run_s <= 180:
            raise ValueError('invalid_auto_authority_duration')
        self.max_run_s = max_run_s
        self.active_run = None
        self.scan_only = False
        self.until = 0.
        self.started_at = 0.
        self.cancelled_through = -math.inf
        self.seen = OrderedDict()

    def request(self, command, now):
        if not isinstance(command, dict) or not _finite(now):
            raise ValueError('invalid_auto_authority_request')
        if command.get('action') == 'cancel':
            retired = self.active_run
            self.active_run = None
            self.until = 0.
            # Receipt time intentionally rejects a queued start created before
            # the operator cancellation was applied at this actuator.
            self.cancelled_through = max(self.cancelled_through, now)
            return retired
        cid, issued, ttl = (command.get(k) for k in ('command_id', 'issued_at', 'ttl_s'))
        if (command.get('action') not in ('start', 'scan') or not isinstance(cid, str) or not 1 <= len(cid) <= 100
                or not _finite(issued) or not _finite(ttl) or not 0 < ttl <= 5
                or not 0 <= now-issued <= ttl or issued <= self.cancelled_through):
            raise ValueError('auto_start_expired_or_precedes_cancel')
        if cid in self.seen:
            raise ValueError('auto_start_replayed')
        if self.active_run is not None and now <= self.until:
            raise ValueError('auto_authority_already_active')
        self.seen[cid] = issued
        while len(self.seen) > 512:
            self.seen.popitem(last=False)
        self.scan_only = command.get('action') == 'scan'
        self.active_run, self.started_at, self.until = cid, issued, now+self.max_run_s
        return None

    def authorized(self, run_id, now):
        return (isinstance(run_id, str) and bool(run_id) and run_id == self.active_run
                and _finite(now) and self.started_at <= now < self.until
                and self.started_at > self.cancelled_through)

    def retire(self, run_id):
        if run_id == self.active_run:
            self.active_run = None
            self.until = 0.

    def manual_disarm(self, now, preserve_run_id=None):
        if not _finite(now):
            raise ValueError('invalid_auto_cancel_time')
        if isinstance(preserve_run_id, str) and preserve_run_id:
            # Startup stop and start authorization may arrive in either order.
            # This never grants authority; it only preserves that exact fresh
            # start, or retires the previous run before the new start arrives.
            if self.active_run != preserve_run_id:
                self.retire(self.active_run)
            return
        self.request({'action': 'cancel'}, now)

    def check_arm(self, command, now):
        run_id, issued, ttl = (command.get(k) for k in ('auto_run_id', 'issued_at', 'ttl_s'))
        return (not self.scan_only and self.authorized(run_id, now) and _finite(issued) and _finite(ttl)
                and 0 < ttl <= 5 and 0 <= now-issued <= ttl
                and issued >= self.started_at and issued > self.cancelled_through)
