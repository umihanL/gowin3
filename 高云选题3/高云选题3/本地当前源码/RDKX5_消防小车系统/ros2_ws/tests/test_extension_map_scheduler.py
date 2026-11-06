"""UART scheduling with real frame codecs and a fake firmware, no device access."""
import types
import unittest
import test_extension_node as fixture
from rescue_vision import ov5640_protocol as wire
from rescue_vision.fpga_map_transport import MapTelemetry, COMMANDS, CAPS


class FirmwareSerial:
    def __init__(self, clock):self.clock=clock;self.rx=bytearray();self.writes=[];self.closed=False
    @property
    def in_waiting(self):return len(self.rx)
    def read(self,size):
        data=bytes(self.rx[:size]);del self.rx[:size];return data
    def close(self):self.closed=True
    def write(self,raw):
        self.writes.append((self.clock[0],raw[4]))
        packet=wire.Parser().feed(raw)[0]
        data={wire.CAPABILITIES:bytes((0,1,31,1)),wire.EXT_STATUS:bytes((0,10,0,0)),
              wire.HEARTBEAT:bytes((0,0,19,0)),wire.HOST_STATUS:bytes((0,0,19,0)),
              wire.POINT_STATUS:bytes(4),wire.STOP:bytes(4)}.get(packet.command)
        # Missing map firmware replies must not delay ordinary control by 0.6s.
        if data is not None:self.rx.extend(wire.encode(packet.sequence,packet.command|0x80,data))
        return len(raw)


class MapSchedulingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.ExtensionAdapterTests.setUpClass();cls.module=fixture.ExtensionAdapterTests.module

    def node(self):
        n=self.module.ExtensionTask.__new__(self.module.ExtensionTask)
        n.core=fixture.ExtensionControl(True);n.parser=wire.Parser();n.error=None
        n.tx_queue=[];n.in_flight=None;n.outputs={};n.now=lambda:self.clock[0]
        n.serial=FirmwareSerial(self.clock);n.actuator={}
        n.map_telemetry=MapTelemetry();n.last_publish=-1.
        n.state_pub=n.navigation_pub=n.navigation_proposal_pub=n.map_pub=object()
        n.event_pub=types.SimpleNamespace(publish=lambda value:None)
        n.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:None))
        class Event:
            STATUS=1
            def __init__(self):self.header=types.SimpleNamespace(stamp=None)
        self.module.FpgaEvent=Event
        return n

    def test_missing_map_reply_preserves_heartbeat_and_stop_priority(self):
        self.clock=[100.];n=self.node()
        for _ in range(140):
            n.tick();self.clock[0]+=.02
        heartbeats=[t for t,cmd in n.serial.writes if cmd==wire.HEARTBEAT]
        self.assertGreater(len(heartbeats),8)
        self.assertLess(max(b-a for a,b in zip(heartbeats,heartbeats[1:])),.4)
        self.assertTrue(n.core.protocol_ready)
        self.assertLess(self.clock[0]-n.core.status_at,.3)
        self.assertEqual([cmd for _,cmd in n.serial.writes if cmd in COMMANDS],[CAPS])
        n.in_flight=None;n.tx_queue=[]
        request=(CAPS,bytes(4),'caps')
        seq=n.core.send(CAPS,now=self.clock[0]);n.map_telemetry.sent(seq,request,self.clock[0]);n.flush()
        n.core.stop(self.clock[0],'test_stop')
        for _ in range(8):n.tick();self.clock[0]+=.02
        self.assertTrue(any(cmd==wire.STOP and t<=self.clock[0] for t,cmd in n.serial.writes))


if __name__=='__main__':unittest.main()
