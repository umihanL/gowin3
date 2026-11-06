"""Pure, bounded water-pump permission state machine; no GPIO or ROS imports.

The caller must explicitly arm and continually supply fresh, verified gates.
This code determines the requested output only. A blocked/killed process cannot
turn hardware off; an independent timed hardware cutoff is still required.
"""
from collections.abc import Mapping
import math


class PumpGuard:
    DEFAULTS = dict(burst_s=.4, cooldown_s=1.0, max_bursts=3, stable_lock_s=.35)

    def __init__(self, config=None):
        if config is not None and not isinstance(config, Mapping):
            raise TypeError('Pump guard config must be a mapping')
        values = dict(self.DEFAULTS)
        unknown = set(config or {}) - values.keys()
        if unknown:
            raise ValueError('Unknown pump guard options: ' + ', '.join(sorted(map(str, unknown))))
        values.update(config or {})
        self.burst_s = self._number(values['burst_s'], 'burst_s', minimum=0, maximum=1,
                                    minimum_inclusive=False)
        self.cooldown_s = self._number(values['cooldown_s'], 'cooldown_s', minimum=.5,
                                       maximum=30)
        self.stable_lock_s = self._number(values['stable_lock_s'], 'stable_lock_s',
                                          minimum=0, maximum=30, minimum_inclusive=False)
        count = values['max_bursts']
        if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 3:
            raise ValueError('max_bursts must be an integer in 1..3')
        self.max_bursts = count
        self.session_max_bursts = count
        self._last_now = None
        self._requires_disarm = False
        self._fault = False
        self.armed = False
        self.pump_on = False
        self.burst_count = 0
        self.arm_deadline = None
        self.burst_deadline = None
        self.cooldown_deadline = None
        self._stable_since = None
        self.state = 'DISARMED'
        self.reason = 'not_armed'

    @staticmethod
    def _number(value, name, *, minimum, maximum, minimum_inclusive=True):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(name + ' must be a finite number')
        try:
            number = float(value)
        except (ValueError, OverflowError) as exc:
            raise ValueError(name + ' must be a finite number') from exc
        if not math.isfinite(number) or number > maximum or (
                number < minimum if minimum_inclusive else number <= minimum):
            raise ValueError(name + ' is outside its allowed range')
        return number

    def _off(self, state, reason, *, requires_disarm=False):
        self.pump_on = False
        self.armed = False
        self.state, self.reason = state, reason
        self.arm_deadline = self.burst_deadline = self.cooldown_deadline = None
        self._stable_since = None
        self._requires_disarm = requires_disarm

    def _clock(self, now):
        try:
            value = self._number(now, 'monotonic time', minimum=0, maximum=1e15)
        except ValueError:
            self._fault = True
            self._off('FAULT', 'invalid_monotonic_time', requires_disarm=True)
            raise
        if self._last_now is not None and value < self._last_now:
            self._fault = True
            self._off('FAULT', 'monotonic_clock_reversed', requires_disarm=True)
            return None
        self._last_now = value
        return value

    def snapshot(self):
        return dict(pump_on=self.pump_on, state=self.state, reason=self.reason,
                    armed=self.armed, burst_count=self.burst_count,
                    max_bursts=self.max_bursts, requires_disarm=self._requires_disarm,
                    session_max_bursts=self.session_max_bursts,
                    arm_deadline=self.arm_deadline, burst_deadline=self.burst_deadline,
                    cooldown_deadline=self.cooldown_deadline)

    def _finish_burst(self, now):
        self.pump_on = False
        self.burst_deadline = None
        self._stable_since = None
        if self.burst_count >= self.session_max_bursts:
            self._off('EXHAUSTED', 'burst_budget_exhausted', requires_disarm=True)
        else:
            # OFF is applied now; a delayed callback cannot count past elapsed
            # time as a cooldown while the actual requested output was still ON.
            self.cooldown_deadline = now + self.cooldown_s
            self.state, self.reason = 'COOLDOWN', 'minimum_off_interval'

    def arm(self, now, lease_s, max_bursts=None):
        """Accept one explicit request; duplicates cannot extend lease or budget.

        Returns ``arm_accepted`` in addition to the normal state fields. A
        budget-exhausted or faulted session needs a separate disarm call first.
        ``now`` is time.monotonic(), not a ROS/image epoch timestamp.
        """
        try:
            lease = self._number(lease_s, 'arm lease', minimum=0, maximum=30,
                                 minimum_inclusive=False)
            burst_limit = self.max_bursts if max_bursts is None else max_bursts
            if type(burst_limit) is not int or not 1 <= burst_limit <= self.max_bursts:
                raise ValueError('max_bursts must be within the configured burst budget')
        except ValueError:
            self._fault = True
            self._off('FAULT', 'invalid_arm_lease', requires_disarm=True)
            raise
        current = self._clock(now)
        accepted = False
        if current is None or self._fault or self._requires_disarm:
            pass
        elif self.armed:
            if current >= self.arm_deadline:
                self._off('DISARMED', 'arm_lease_expired')
            elif self.pump_on and current >= self.burst_deadline:
                self._finish_burst(current)
            # An already-armed call is never a new session, including when the
            # previous lease expired at this call. A separate request is needed.
        else:
            self.armed = True
            self.pump_on = False
            self.burst_count = 0
            self.session_max_bursts = burst_limit
            self.arm_deadline = current + lease
            self.burst_deadline = self.cooldown_deadline = None
            self._stable_since = None
            self.state, self.reason = 'WAIT_LOCK', 'waiting_for_stable_fresh_gates'
            accepted = True
        return dict(self.snapshot(), arm_accepted=accepted)

    def disarm(self, reason='operator_disarm'):
        """Immediately request OFF and reset the explicit-rearm interlock."""
        if not isinstance(reason, str) or not reason.strip():
            reason = 'operator_disarm'
        self._fault = False
        self._off('DISARMED', reason)
        return self.snapshot()

    def update(self, now, *, mission_ok, command_ok, aim_ok, estop=False):
        """Evaluate verified boolean gates; any loss cancels the current arm.

        The adapter checks acquisition age, mounting/coordinate/calibration
        identity, stopped chassis, traffic preemption, command lease and settling
        before providing these gates. This module never infers those conditions.
        """
        current = self._clock(now)
        if current is None or self._fault:
            return self.snapshot()
        for name, gate in (('mission_ok', mission_ok), ('command_ok', command_ok),
                           ('aim_ok', aim_ok), ('estop', estop)):
            if not isinstance(gate, bool):
                self._fault = True
                self._off('FAULT', 'non_boolean_' + name, requires_disarm=True)
                raise ValueError(name + ' must be a verified boolean')
        if estop:
            self._off('DISARMED', 'emergency_stop')
            return self.snapshot()
        if not self.armed:
            return self.snapshot()
        if current >= self.arm_deadline:
            self._off('DISARMED', 'arm_lease_expired')
            return self.snapshot()
        for gate, reason in ((mission_ok, 'mission_gate_lost'),
                             (command_ok, 'actuator_command_or_settle_gate_lost'),
                             (aim_ok, 'fresh_aim_gate_lost')):
            if not gate:
                self._off('DISARMED', reason)
                return self.snapshot()
        if self.pump_on:
            if current < self.burst_deadline:
                return self.snapshot()
            self._finish_burst(current)
            return self.snapshot()
        if self.cooldown_deadline is not None:
            if current < self.cooldown_deadline:
                return self.snapshot()
            self.cooldown_deadline = None
            self._stable_since = None
            self.state, self.reason = 'WAIT_LOCK', 'reconfirming_stable_fresh_gates'
        if self._stable_since is None:
            self._stable_since = current
        if current - self._stable_since < self.stable_lock_s:
            self.state, self.reason = 'WAIT_LOCK', 'waiting_for_stable_fresh_gates'
            return self.snapshot()
        # Permit a short final burst, clipped by the explicitly armed lease.
        self.burst_deadline = min(current + self.burst_s, self.arm_deadline)
        self.burst_count += 1
        self.pump_on = True
        self.state, self.reason = 'BURST', 'fresh_stable_lock'
        return self.snapshot()
