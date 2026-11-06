"""Production region/navigation/adapter regression with synthetic UART only."""
import copy
import unittest

import test_extension_node as adapter
from test_region_continuous_flow import ContinuousHarness
from rescue_vision import ov5640_protocol as wire
from rescue_vision.fpga_pump_bridge import FPGAPumpBridge
from rescue_vision.retained_route import local_coordinates


class SearchPumpIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        adapter.ExtensionAdapterTests.setUpClass()
        cls.module = adapter.ExtensionAdapterTests.module

    def setup_mission(self, running=False):
        h = self.h = ContinuousHarness()
        h.start()
        if running:
            h.drive_mission()
        else:
            h.ack()  # A genuine RESET acknowledgement, before route setup.
            h.feedback(session=False, run=False, motion=0, point_state=0)
            self.assertEqual(h.nav.active['phase'], 'reset_readback')
        return self.attach_node(h)

    def attach_node(self, h):
        self.h = h
        node = self.node = self.module.ExtensionTask.__new__(self.module.ExtensionTask)
        node.core = h.c
        node.serial = adapter.Serial()
        node.tx_queue = []
        node.in_flight = None
        node.error = None
        node.now = lambda: h.now + .002
        node.outputs = {key: object() for key in ('mission', 'auto', 'actuator')}
        node.pump_bridge = FPGAPumpBridge(h.c)
        node.pump_bridge.initialized = True
        node.pump_bridge.sample = wire.pump_status(bytes((30, 0, 0, 0)))
        node.pump_bridge.feedback_at = h.now - 1.  # Actual old LOW, not ON.
        h.c.outbox.clear()
        h.c.effects.clear()
        return h, node

    def setup_jog(self, moving):
        h = ContinuousHarness(); h.start(); h.drive_mission()
        points = h.route['points_mm']
        for index in range(3, len(points)-8):
            anchor = dict(image_mm=points[index], heading_deg=h.path_pose(index)[1])
            behind, ahead = local_coordinates(anchor, points[index-3]), local_coordinates(anchor, points[index+6])
            if behind[0] < -100 and ahead[0] > 100 and abs(behind[1]) < 10 and abs(ahead[1]) < 10:
                while h.path_cursor < index-2:
                    h.advance()
                break
        else:
            self.fail('real ROM route has no test straight section')
        h.targets = [(1, .7), (2, .3)]
        while h.search.phase != 'WAIT_STOP':
            h.advance()
        h.verify_pause()
        h.fire_offsets = {1: 40, 2: -40}
        for _ in range(30):
            h.feedback(tick=True)
            if h.nav.active['phase'] == 'centering_starting':
                break
        self.assertEqual(h.nav.active['phase'], 'centering_starting')
        raw = next(f for f in h.c.outbox if f[4] == wire.RETAINED_JOG)
        h, node = self.attach_node(h)
        h.c.outbox.append(raw)
        if moving:
            node.pump_bridge.feedback_at = h.now
            node.flush()
            self.assertEqual(node.serial.writes[-1][4], wire.RETAINED_JOG)
            self.assertEqual(h.ack(), wire.RETAINED_JOG)
            node.in_flight = None
        node.pump_bridge.feedback_at = h.now - 1.
        return h, node

    def setup_unwritten_start(self):
        h = ContinuousHarness(); h.start()
        reset_frame = next(f for f in h.c.outbox if f[3] == h.nav.active['sequence'])
        h, node = self.attach_node(h)
        h.c.outbox.append(reset_frame)
        node.flush()
        self.assertTrue(h.nav.active['start_write_tracking'])
        h.ack(); node.in_flight = None
        h.feedback(session=False, run=False, motion=0, point_state=0, tick=True)
        for _ in range(20):
            if h.nav.active['phase'] == 'planning':
                break
            h.ack(); h.feedback(tick=True)
        h.feedback(point_state=2, tick=True)
        self.assertEqual(h.nav.active['phase'], 'continuous_route_check')
        h.path = h.full_graph_path()
        nodes = h.c.area.route_planner.nodes
        points = [[nodes[i][0]/25.6, nodes[i][1]/25.6] for i in h.path]
        h.route = dict(valid=True, sampled=False, generation=h.route['generation']+1,
            stamp=h.now+.01, points_mm=points, sample_indices=list(range(len(points))), total_count=len(points))
        h.feedback(tick=True)
        self.assertEqual(h.nav.active['phase'], 'starting')
        start_frame = next(f for f in h.c.outbox if f[3] == h.nav.active['sequence'])
        h.c.outbox[:] = [start_frame]  # Earlier setup queries have already replied.
        return h, node

    def guard(self):
        h, node = self.h, self.node
        p = node.pump().state(node.now())
        h.c.actuator.update(pump_backend='fpga_uart', pump_pin='B13',
            pump_feedback_fresh=p['feedback_fresh'], pump_on=p['pump_on'])
        node.actuator = h.c.actuator
        reason = node.actuator_guard(node.now(), navigation_only=True)
        return node.search_pump_guard(reason, node.now())

    def acknowledge_written_off(self):
        node = self.node
        node.in_flight = None
        node.flush()
        raw = node.serial.writes[-1]
        self.assertEqual(raw[4], wire.PUMP_OFF)
        node.pump().receive(wire.Packet(raw[3], wire.PUMP_OFF | 128, bytes(4)), node.now())
        node.in_flight = None

    def deliver_new_low_readback(self):
        """Only a queried FPGA status frame restores the bridge's LOW TTL."""
        h, node = self.h, self.node
        sequence = node.pump()._send(wire.PUMP_STATUS, node.now())
        raw = next(f for f in h.c.outbox if f[3] == sequence)
        h.c.outbox.remove(raw)
        written = node.pump().before_write(raw, node.now())
        node.serial.write(written)
        reply = wire.Parser().feed(wire.encode(sequence, wire.PUMP_STATUS | 128, bytes((30, 0, 0, 0))))[0]
        self.assertTrue(node.pump().receive(reply, node.now() + .001))

    def test_1823_prestart_stale_low_holds_same_task_until_real_readback(self):
        h, node = self.setup_mission()
        task = h.nav.active['task_id']
        leg = h.nav.active
        pose = copy.deepcopy(h.nav.map.get('pose'))
        self.assertEqual(self.guard(), '')
        node.flush()
        self.assertEqual([f[4] for f in node.serial.writes], [wire.PUMP_OFF])
        # Configuration ACK parsing continues, while the region tick cannot
        # issue any new route START or silently consume a waypoint.
        for _ in range(4):
            h.feedback(advance=.05)
            self.assertEqual(self.guard(), '')
            h.c.area.tick(node.now())
            self.assertEqual(h.nav.active['phase'], 'reset_readback')
            self.assertEqual(h.search.phase, 'NAVIGATING')
        self.assertIs(h.nav.active, leg)
        self.assertEqual(h.nav.active['task_id'], task)
        self.assertEqual(h.nav.map.get('pose'), pose)
        self.assertEqual(len([f for f in node.serial.writes if f[4] == wire.PUMP_OFF]), 1)
        self.assertNotIn(wire.START, [f[4] for f in node.serial.writes])
        self.deliver_new_low_readback()
        self.assertEqual(self.guard(), '')
        self.assertFalse(h.c.area.pump_readback_hold_prestart)
        h.c.area.tick(node.now())
        self.assertEqual(h.nav.active['phase'], 'setting_region_preference')
        self.assertIs(h.nav.active, leg)
        self.assertNotIn(wire.PUMP_ON, [f[4] for f in node.serial.writes])

    def test_running_pause_stopped_feedback_low_and_same_route_then_resume(self):
        h, node = self.setup_mission(running=True)
        task, generation = h.nav.active['task_id'], h.nav.active['route_generation']
        pose = copy.deepcopy(h.nav.map['pose'])
        self.assertEqual(self.guard(), '')
        self.assertEqual(h.nav.active['phase'], 'pausing')
        self.assertEqual(h.nav.map['pose'], pose)  # STOP is not measured pose.
        node.flush()
        self.assertEqual(node.serial.writes[-1][4], wire.STOP)
        self.assertEqual(h.ack(), wire.STOP)
        self.acknowledge_written_off()
        h.feedback(run=False, motion=2)
        h.c.outbox.clear()  # Harness.feedback already delivered these query replies.
        self.assertEqual(self.guard(), '')
        h.c.area.tick(node.now())
        self.assertEqual(h.nav.active['phase'], 'paused')
        self.assertEqual(h.search.phase, 'NAVIGATING')
        self.assertNotIn(wire.RESUME, [f[4] for f in node.serial.writes])
        self.deliver_new_low_readback()
        self.assertEqual(self.guard(), '')
        self.assertEqual(h.nav.active['phase'], 'resuming')
        self.assertEqual(h.nav.active['task_id'], task)
        self.assertEqual(h.nav.active['route_generation'], generation)
        node.flush()
        self.assertEqual(node.serial.writes[-1][4], wire.RESUME)
        self.assertEqual(h.ack(), wire.RESUME)
        h.feedback(run=True, motion=2)
        h.c.area.tick(node.now())
        self.assertEqual(h.nav.active['phase'], 'running')
        self.assertEqual(sum(f[4] == wire.PUMP_OFF for f in node.serial.writes), 1)
        self.assertNotIn(wire.PUMP_ON, [f[4] for f in node.serial.writes])

    def test_start_hold_transports_status_without_start_or_resume(self):
        h, node = self.setup_mission()
        self.guard()
        # Include every held-motion opcode: only nonmotion traffic may pass.
        for command in (wire.START, wire.RESUME, wire.RETAINED_JOG, wire.EXT_STATUS):
            h.c.send(command, now=node.now())
        node.flush()
        self.assertEqual(node.serial.writes[-1][4], wire.PUMP_OFF)
        node.in_flight = None
        node.flush()
        self.assertEqual(node.serial.writes[-1][4], wire.EXT_STATUS)
        self.assertEqual([f[4] for f in node.tx_queue], [wire.START, wire.RESUME, wire.RETAINED_JOG])
        self.assertTrue(h.c.area.pump_readback_hold_prestart)

    def test_operator_cancel_after_recovery_pause_never_resumes(self):
        h, node = self.setup_mission(running=True)
        self.guard()
        h.c.stop(node.now(), 'operator_stop')
        self.assertFalse(h.search.active)
        self.deliver_new_low_readback()
        self.guard()
        node.flush()
        self.assertFalse(node._search_pump_recovery.active)
        self.assertNotIn(wire.RESUME, [f[4] for f in node.serial.writes] + [f[4] for f in node.tx_queue])
        self.assertFalse(h.c.area.pump_readback_hold_search)

    def test_actual_on_is_never_recovery_wait_and_timeout_preserves_reason(self):
        h, node = self.setup_mission()
        node.pump_bridge.feedback_at = h.now
        node.pump_bridge.sample = wire.pump_status(bytes((15, 0, 100, 0)))
        self.assertIn('output_on', self.guard())
        self.assertFalse(h.c.area.pump_readback_hold_prestart)
        h, node = self.setup_mission()
        self.guard()
        h.feedback(advance=3.01)
        reason = self.guard()
        self.assertIn('timeout:pump_output_pending_or_not_confirmed_off', reason)
        h.c.stop(node.now(), reason)
        self.assertFalse(h.search.active)
        self.assertEqual(h.search.reason, reason)

    def test_jog_moving_or_unwritten_stops_then_new_low_releases_original_visual_fsm(self):
        for moving in (False, True):
            with self.subTest(moving=moving):
                h, node = self.setup_jog(moving)
                a, center = h.nav.active, h.nav.centering
                identity = a['task_id'], a['route_generation'], center.session['session_id']
                pose = copy.deepcopy(h.nav.map['pose'])
                original_visual_phase = h.search.phase
                self.assertEqual(self.guard(), '')
                self.assertEqual(a['phase'], 'centering_stopping')
                self.assertEqual(h.nav.map['pose'], pose)
                node.flush()
                self.assertEqual(node.serial.writes[-1][4], wire.STOP)
                self.assertEqual(h.ack(), wire.STOP)
                self.acknowledge_written_off()
                h.local_moving = moving
                h.feedback(advance=.05, run=False, motion=2)
                h.c.outbox.clear()
                self.assertEqual(self.guard(), '')
                h.c.area.tick(node.now())
                sequence = center.session['query_sequence']
                self.assertIsNotNone(sequence)
                jog = center.session['jog']
                status = bytes((4, 1, jog['sequence'], 3)) if moving else bytes((0, 0, (jog['sequence']-1)&255, 3))
                h.now += .01
                h.c.receive(wire.Packet(sequence, wire.RETAINED_JOG_STATUS | 128, status), h.now)
                self.assertEqual(a['phase'], 'centering_verifying')
                if moving:
                    h.feedback(advance=.05)
                    self.guard(); h.c.area.tick(node.now())
                    self.assertEqual(a['phase'], 'centering_verifying')  # status alone is not standstill
                h.local_moving = False
                h.feedback(advance=.05)
                self.assertEqual(self.guard(), '')
                h.c.area.tick(node.now())
                self.assertEqual(a['phase'], 'paused')
                self.assertTrue(h.search.active)
                self.assertEqual(h.search.phase, original_visual_phase)
                self.assertFalse(h.nav.centering_proposal('forward', node.now())['valid'])
                self.assertFalse(node.pump_bridge.state(node.now())['feedback_fresh'])
                self.deliver_new_low_readback()
                self.assertEqual(self.guard(), '')
                self.assertFalse(h.c.area.pump_readback_hold_search)
                self.assertIsNone(center._pump_stop_recovery)
                self.assertTrue(h.nav.centering_proposal('forward', node.now())['valid'])
                h.c.area.tick(node.now())
                self.assertTrue(h.search.active)
                self.assertNotEqual(h.search.phase, original_visual_phase)
                self.assertEqual((a['task_id'], a['route_generation'], center.session['session_id']), identity)
                self.assertEqual(sum(f[4] == wire.PUMP_OFF for f in node.serial.writes), 1)
                self.assertEqual(sum(f[4] == wire.RETAINED_JOG for f in node.serial.writes), int(moving))
                self.assertNotIn(wire.PUMP_ON, [f[4] for f in node.serial.writes])

    def test_jog_recovery_does_not_override_real_on_operator_cancel_or_timeout(self):
        for change in ('on', 'operator', 'timeout'):
            with self.subTest(change=change):
                h, node = self.setup_jog(True)
                self.guard()
                if change == 'operator':
                    h.c.stop(node.now(), 'operator_stop')
                elif change == 'on':
                    node.pump_bridge.feedback_at = h.now
                    node.pump_bridge.sample = wire.pump_status(bytes((15, 0, 100, 0)))
                else:
                    h.now += 3.01
                reason = self.guard()
                if change != 'operator':
                    self.assertTrue(reason)
                    h.c.stop(node.now(), reason)
                self.assertFalse(h.search.active)
                self.assertFalse(h.nav.centering.active)
                self.assertIsNone(h.nav.centering._pump_stop_recovery)
                self.assertNotIn(wire.RESUME, [f[4] for f in h.c.outbox])

    def test_tracked_original_start_waits_for_low_and_records_only_successful_write(self):
        h, node = self.setup_unwritten_start()
        task, sequence = h.nav.active['task_id'], h.nav.active['sequence']
        self.assertNotIn('start_written_at', h.nav.active)
        self.assertEqual(self.guard(), '')
        self.assertTrue(h.c.area.pump_readback_hold_prestart)
        node.flush()
        self.assertNotIn(wire.START, [f[4] for f in node.serial.writes])
        self.assertNotIn('start_written_at', h.nav.active)
        off = node.serial.writes[-1]
        self.assertEqual(off[4], wire.PUMP_OFF)
        node.pump().receive(wire.Packet(off[3], wire.PUMP_OFF | 128, bytes(4)), node.now())
        node.in_flight = None
        self.deliver_new_low_readback()
        self.assertEqual(self.guard(), '')
        node.flush()
        self.assertEqual(node.serial.writes[-1][4], wire.START)
        self.assertEqual(node.serial.writes[-1][3], sequence)
        self.assertEqual(h.nav.active['task_id'], task)
        self.assertEqual(h.nav.active['start_written_sequence'], sequence)
        self.assertEqual(h.nav.active['start_written_at'], node.now())
        self.assertEqual(sum(f[4] == wire.START for f in node.serial.writes), 1)

    def test_partial_start_write_never_records_full_write_or_claims_safe_unwritten(self):
        h, node = self.setup_unwritten_start()
        node.serial.write = lambda raw: len(raw)-1
        node.flush()
        self.assertEqual(node.error, 'uart_write_failed:partial_uart_write')
        self.assertTrue(node._uart_recovering)
        self.assertIsNot(node.core, h.c)
        self.assertIsNone(node.core.area.navigation.active)
        self.assertIsNone(node.serial)
        self.assertNotIn('start_written_sequence', h.nav.active)
        self.assertNotIn('start_written_at', h.nav.active)


if __name__ == '__main__':
    unittest.main()
