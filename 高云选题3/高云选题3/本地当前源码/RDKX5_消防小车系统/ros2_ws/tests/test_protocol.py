import pathlib
import sys
import struct
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision import protocol as p


class ProtocolTests(unittest.TestCase):
    def test_standard_crc_vector(self):self.assertEqual(p.crc16(b'123456789'),0x29b1)
    def test_partial_corrupted_and_combined_frames(self):
        a=p.encode(p.CROSSWALK,12,struct.pack('<I',12345));b=p.encode(p.STATUS,13,struct.pack('<HhhH',7,9000,10000,0))
        bad=bytearray(a);bad[-1]^=1
        parser=p.Parser();self.assertEqual(parser.feed(b'noise'+bad+a[:3]),[])
        decoded=parser.feed(a[3:]+b)
        self.assertEqual(len(decoded),2);self.assertEqual(p.event(decoded[0])['event_id'],12345)
        self.assertTrue(p.event(decoded[1])['measured_angles_valid']);self.assertEqual(parser.bad_frames,1)
    def test_bad_length_resync_and_embedded_magic(self):
        good=p.encode(p.CROSSWALK,1,struct.pack('<I',0x5aa5))
        parser=p.Parser();out=parser.feed(b'\xa5\x5a\x01\x10\x00\x00\xff\xff'+good)
        self.assertEqual(len(out),1);self.assertEqual(p.event(out[0])['event_id'],0x5aa5)
    def test_lease_needs_status_and_fresh_command(self):
        gate=p.CommandGate();gate.set_permission(True,7,250,10.,0.)
        self.assertFalse(gate.current(10.01)['allowed'])
        gate.observe_status(10.02);self.assertTrue(gate.current(10.03)['allowed'])
        self.assertFalse(gate.current(10.3)['allowed'])
        gate.set_permission(True,7,250,11.,.6);self.assertFalse(gate.current(11.)['allowed'])
    def test_red_stop_overwrites_green_and_invalid_gimbal_rejected(self):
        gate=p.CommandGate();gate.observe_status(2.)
        gate.set_permission(True,7,250,2.,0);self.assertTrue(gate.current(2.01)['allowed'])
        gate.set_permission(False,7,250,2.02,0);self.assertFalse(gate.current(2.03)['allowed'])
        with self.assertRaises(ValueError):p.gimbal_payload(float('nan'),90,20,250)
        with self.assertRaises(ValueError):p.gimbal_payload(-60,90,20,250)
        with self.assertRaises(ValueError):p.motion_payload(0,True,5000)
    def test_message_delay_and_retransmission_cannot_extend_permission(self):
        gate=p.CommandGate();gate.observe_status(10.)
        gate.set_permission(True,7,200,10.,.1)
        out=gate.current(10.06)
        self.assertTrue(out['allowed']);self.assertLessEqual(out['lease_ms'],40)
        self.assertFalse(gate.current(10.101)['allowed'])
    def test_cumulative_encoder_duplicate_gap_and_units(self):
        def packet(ms,mm,seq=1):return p.Packet(p.TELEMETRY,seq,struct.pack(p.TELEMETRY_FORMAT,ms,mm,100,200,3,450,150,1|2|4|8|32|64,2,7,9))
        decoder=p.TelemetryDecoder()
        first=decoder.decode(packet(1000,100),1.0);self.assertFalse(first['odometry_valid'])
        d=decoder.decode(packet(1020,103,2),1.02)
        self.assertTrue(d['odometry_valid']);self.assertAlmostEqual(d['delta_s_m'],.003)
        self.assertAlmostEqual(d['yaw_rate_rad_s'],.1);self.assertAlmostEqual(d['obstacle_distance_m'],.45)
        self.assertTrue(d['gyro_valid']);self.assertTrue(d['vehicle_stopped'])
        self.assertIsNone(decoder.decode(packet(1020,103,2),1.03))
        self.assertFalse(decoder.decode(packet(2000,120),2.0)['odometry_valid'])
    def test_maneuver_payload_is_stable_and_meter_units_explicit(self):
        data=p.maneuver_payload(123,1,True,.15,1.25,180)
        self.assertEqual(struct.unpack('<IBBHIH',data),(123,1,1,150,1250,180))
        out=p.Parser().feed(p.encode(p.MANEUVER,1,data))[0]
        self.assertEqual(out.payload,data)
        with self.assertRaises(ValueError):p.maneuver_payload(123,7,False,.1,1.,250)

if __name__=='__main__':unittest.main(verbosity=2)
