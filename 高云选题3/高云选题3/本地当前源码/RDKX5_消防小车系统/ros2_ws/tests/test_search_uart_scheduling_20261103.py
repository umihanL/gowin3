"""Physical pump polling must not expire behind unrelated route telemetry."""
import unittest
import test_extension_node as adapter
from rescue_vision import ov5640_protocol as wire


class SearchUartSchedulingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        adapter.ExtensionAdapterTests.setUpClass()

    def node(self):
        fixture=adapter.ExtensionAdapterTests()
        fixture.setUp()
        return fixture.node

    def test_low_readback_precedes_route_configuration_but_not_stop_off(self):
        n=self.node()
        n.core.send(wire.EXT_STATUS,now=100.);n.flush()
        n.core.send(wire.SET_START,b'\x02\0\x03\0',now=100.)
        n.core.send(wire.POINT_STATUS,now=100.)
        n.pump()._send(wire.PUMP_STATUS,100.)
        n.flush()
        self.assertEqual([x[4] for x in n.tx_queue],
            [wire.PUMP_STATUS,wire.SET_START,wire.POINT_STATUS])
        n.core.send(wire.STOP,now=100.);n.pump().off(100.)
        n.pump()._send(wire.PUMP_STATUS,100.);n.flush()
        self.assertEqual([x[4] for x in n.tx_queue[:3]],
            [wire.STOP,wire.PUMP_OFF,wire.PUMP_STATUS])

    def test_unanswered_status_does_not_occupy_entire_low_feedback_lease(self):
        n=self.node();n.core.send(wire.EXT_STATUS,now=100.);n.flush()
        n.pump()._send(wire.PUMP_STATUS,100.01)
        n.now=lambda:100.13;n.flush()
        self.assertEqual([x[4] for x in n.serial.writes],[wire.EXT_STATUS,wire.PUMP_STATUS])

    def test_heartbeat_ahead_of_pump_poll_keeps_its_priority(self):
        n=self.node();n.core.send(wire.EXT_STATUS,now=100.);n.flush()
        n.core.send(wire.HEARTBEAT,now=100.)
        n.pump()._send(wire.PUMP_STATUS,100.);n.flush()
        self.assertEqual([x[4] for x in n.tx_queue],[wire.HEARTBEAT,wire.PUMP_STATUS])


if __name__=='__main__':unittest.main()
