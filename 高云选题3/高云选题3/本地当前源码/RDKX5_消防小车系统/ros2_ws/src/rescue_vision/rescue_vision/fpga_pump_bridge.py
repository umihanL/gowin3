"""B13 pulse control through the existing exclusive FPGA UART owner.

ACK acknowledges a command; only a subsequent status query reports the output
register. Neither is a water-flow sensor. ON is never retried or renewed.
"""
import math
from collections import OrderedDict
from . import ov5640_protocol as wire

COMMANDS = (wire.PUMP_ON, wire.PUMP_OFF, wire.PUMP_STATUS)


class FPGAPumpBridge:
    def __init__(self, core):
        self.core = core
        self.pending = {}
        self.seen = OrderedDict()
        self.last_input_stamp = -math.inf
        self.cancel_before = -math.inf
        self.last_poll = -math.inf
        self.initialized = False
        self.generation = 0
        self.sample = {}
        self.feedback_at = None
        self.last_result = None
        self.error = ''
        self.awaiting_readback = None
        self.control_command_id = None
        self.feedback_command_id = None

    def available(self):
        return (self.core.real_uart and self.core.protocol_ready and
                self.core.capabilities.get('pump_b13') is True)

    def vehicle_ready(self, now):
        """Idle vehicle or the exact patrol-owned, verified paused fire round.

        A route pause is still a busy navigation job, so it must not grant
        general movement permission. Its supervisor grants only this narrow
        pump operation window after matching pause and measured-stop feedback.
        """
        if self.core.state(now).get('can_move') is True:
            return True
        area = getattr(self.core, 'area', None)
        if area is not None and area.pump_allowed(now):
            return True
        patrol = getattr(self.core.numbered, 'patrol', None)
        return bool(patrol is not None and patrol.pump_allowed(now) is True)

    def _send(self, command, now, **metadata):
        sequence = self.core.send(command, now=now)
        self.pending[sequence] = dict(command=command, queued_at=now, **metadata)
        return sequence

    def off(self, now, command_id='fpga-pump-off'):
        self.cancel_on(now)
        if self.core.real_uart:
            return self._send(wire.PUMP_OFF, now, command_id=command_id)

    def cancel_on(self, now):
        self.last_input_stamp = max(self.last_input_stamp, now)
        self.cancel_before = max(self.cancel_before, now)
        # Clear pending/queued ON even if the UART is temporarily unavailable.
        for metadata in self.pending.values():
            if metadata['command'] == wire.PUMP_ON:
                metadata['cancelled'] = True

    def request(self, value, now):
        if value.get('action') != 'set_pump' or type(value.get('enabled')) is not bool:
            raise ValueError('invalid_fpga_pump_request')
        cid = value.get('command_id')
        if not isinstance(cid, str) or not 1 <= len(cid) <= 120:
            raise ValueError('invalid_fpga_pump_request_id')
        if value['enabled'] is False:
            self.off(now, cid)
            return
        stamp = value.get('issued_at')
        duration = value.get('duration_ms')
        wire.pump_payload(duration)
        if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
                not 0 <= now-stamp <= .5 or stamp < self.last_input_stamp or stamp <= self.cancel_before):
            raise ValueError('stale_fpga_pump_request')
        if cid in self.seen:
            raise ValueError('duplicate_fpga_pump_request')
        self.seen[cid] = now
        while len(self.seen) > 512:
            self.seen.popitem(last=False)
        current = self.state(now)
        if not current['protocol_ready']:
            raise ValueError('fpga_pump_firmware_required')
        if not current['feedback_fresh'] or not current['pump_ready']:
            raise ValueError('fpga_pump_ready_feedback_required')
        if not self.vehicle_ready(now):
            raise ValueError('fpga_pump_idle_vehicle_required')
        if any(item['command'] == wire.PUMP_ON and not item.get('cancelled')
               for item in self.pending.values()):
            raise ValueError('fpga_pump_request_pending')
        deadline = stamp + duration/1000.
        if now >= deadline:
            raise ValueError('fpga_pump_request_expired')
        self.last_input_stamp = stamp
        self._send(wire.PUMP_ON, now, command_id=cid, deadline=deadline, issued_at=stamp)
        self.last_result = dict(command_id=cid, accepted=None, reason='fpga_pump_queued')

    def before_write(self, frame, now):
        """Called immediately before UART write; stale queued ON never fires."""
        # OFF/STOP/RESUME cannot raise B13. Preserve a genuinely read LOW for
        # its original TTL across these commands, but never carry an ON (or
        # unknown output) across them. Clearing a recent LOW made a normal
        # navigation handoff cancel itself on its very next 20 ms tick.
        physical = self.state(now)
        preserve_low = bool(physical['feedback_fresh'] and
                            physical['pump_on'] is False and not self.error)
        if frame[4] in (wire.STOP, wire.RESUME):
            self.cancel_on(now)
            self.generation += 1
            if not preserve_low:
                self.feedback_at = None
            self.feedback_command_id = None
            self.awaiting_readback = None
            self.control_command_id = None
            self.last_poll = -math.inf
        if frame[4] not in COMMANDS:
            return frame
        item = self.pending.get(frame[3])
        if item is None:
            return None
        if frame[4] == wire.PUMP_ON:
            remaining = math.floor((item['deadline']-now)*1000)
            valid = self.state(now)
            if (item.get('cancelled') or not valid['pump_ready'] or
                    not self.vehicle_ready(now) or remaining < 1):
                self.last_result = dict(command_id=item.get('command_id'), accepted=False,
                                        reason='fpga_pump_request_expired_or_cancelled')
                self.discard(frame[3])
                return None
            frame = wire.encode(frame[3], wire.PUMP_ON, wire.pump_payload(min(remaining, 1000)))
        if frame[4] in (wire.PUMP_ON, wire.PUMP_OFF):
            self.generation += 1
            self.control_command_id = item.get('command_id')
            if frame[4] == wire.PUMP_ON or not preserve_low:
                self.feedback_at = None
            self.feedback_command_id = None
            self.awaiting_readback = dict(command_id=item.get('command_id'),
                until=min(item.get('issued_at', item['queued_at'])+.3,
                          item.get('deadline', math.inf)), generation=self.generation)
        item.update(sent_at=now, generation=self.generation)
        return frame

    def discard(self, sequence):
        self.pending.pop(sequence, None)
        self.core.pending.pop(sequence, None)

    def receive(self, packet, now):
        if (packet.command & 0x7f) not in COMMANDS:
            return False
        item = self.pending.get(packet.sequence)
        if (item is None or packet.command != (item['command'] | 0x80) or
                'sent_at' not in item or not 0 <= now-item['sent_at'] <= .6):
            return True
        self.discard(packet.sequence)
        if item['generation'] != self.generation:
            return True
        if item['command'] == wire.PUMP_STATUS:
            try:
                self.sample = wire.pump_status(packet.data)
                # Use query time: a delayed response cannot acquire a new TTL.
                self.feedback_at = item['sent_at']
                self.feedback_command_id = self.control_command_id
                self.awaiting_readback = None
                if (not self.sample['pump_on'] and self.sample['lockout'] and
                        self.sample['reason_code'] in (6, 8)):
                    # A mode change/reset disarms B13 even while already LOW.
                    # Re-establish OFF only, then require another fresh readback.
                    # Never replay an ON, or clear movement/link/STOP faults here.
                    self.initialized = False
                if not self.sample['pump_on'] and self.sample['reason_code'] == 0:
                    self.error = ''
            except ValueError as exc:
                self.feedback_at = None
                self.error = str(exc)
        else:
            accepted = packet.data[0] == 0
            self.last_result = dict(command_id=item.get('command_id'), accepted=accepted,
                reason='fpga_pump_command_acknowledged' if accepted else
                       'fpga_pump_rejected_' + str(packet.data[0]))
            if not accepted:
                self.error = self.last_result['reason']
            self.last_poll = -math.inf
        return True

    def tick(self, now):
        for sequence, item in list(self.pending.items()):
            if now-item.get('sent_at', item['queued_at']) > .6:
                if item['command'] == wire.PUMP_ON:
                    self.last_result = dict(command_id=item.get('command_id'), accepted=None,
                                            reason='fpga_pump_ack_timeout_no_retry')
                self.discard(sequence)
        if not self.available():
            self.feedback_at = None
            self.awaiting_readback = None
            self.initialized = False
            return
        if not self.initialized:
            self.off(now, 'fpga-pump-startup-off')
            self.initialized = True
        if now-self.last_poll >= .2:
            self._send(wire.PUMP_STATUS, now)
            self.last_poll = now

    def state(self, now):
        protocol = bool(self.available())
        recent = (protocol and self.feedback_at is not None and
                  0 <= now-self.feedback_at <= .6)
        pending = self.awaiting_readback
        waiting = bool(protocol and pending and not self.error and
                       now < pending['until'] and not recent)
        return dict(self.sample, stamp=now, feedback_stamp=self.feedback_at,
            feedback_command_id=self.feedback_command_id if recent else None,
            feedback_fresh=recent, protocol_ready=protocol, real_uart=self.core.real_uart,
            pump_on=self.sample.get('pump_on') if recent else None,
            pump_ready=bool(recent and self.sample.get('pump_ready') and not self.error
                            and self.awaiting_readback is None),
            pin='B13', active_high=True, backend='fpga_uart', gate_control_only=True,
            reason=self.sample.get('reason') if recent else ('fpga_pump_feedback_stale'
                if protocol else 'fpga_pump_firmware_required'),
            error=self.error, last_result=self.last_result,
            pending_readback=waiting,
            pending_command_id=pending.get('command_id') if pending else None,
            pending_until=pending.get('until') if pending else None,
            last_command_id=(self.last_result or {}).get('command_id'),
            ack_accepted=(self.last_result or {}).get('accepted'))
