"""Map snapshots are coherent, bounded and independent of control acknowledgements."""
import sys
from pathlib import Path
import struct
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision import fpga_map_transport as wire
from rescue_vision.ov5640_protocol import Packet
from rescue_vision.fpga_map import load_catalog, live_map_snapshot


class MapTransportTests(unittest.TestCase):
    def test_region_preference_capability_accepts_bit6_without_inventing_old_support(self):
        self.reply((wire.CAPS,bytes(4),'caps'),bytes((0,1,63,1)))
        self.assertTrue(self.bridge.snapshot(100.02)['region_supported'])
        self.assertFalse(self.bridge.snapshot(100.02)['region_preference_supported'])
        self.reply((wire.CAPS,bytes(4),'caps'),bytes((0,1,127,1)),100.03)
        self.assertTrue(self.bridge.snapshot(100.05)['region_preference_supported'])
        self.reply((wire.CAPS,bytes(4),'caps'),bytes((0,1,255,1)),100.06)
        self.assertTrue(self.bridge.snapshot(100.08)['continuous_route_supported'])

    def test_cursor_is_atomic_with_pose_and_old_firmware_does_not_invent_it(self):
        self.ready()
        self.reply((wire.CAPS,bytes(4),'caps'),bytes((0,1,255,1)))
        self.reply((wire.POSE_LATCH,bytes(4),'pose_latch'),struct.pack('<HBB',42,15,9))
        self.reply(self.bridge.next_request(100.02),struct.pack('<hh',1200,300),100.02)
        self.reply(self.bridge.next_request(100.04),struct.pack('<hBB',-900,9,0x32),100.04)
        self.assertFalse(self.bridge.pose)
        req=self.bridge.next_request(100.06)
        self.assertEqual(req[:2],(wire.ROUTE_CURSOR,struct.pack('<HH',42,0)))
        self.reply(req,struct.pack('<HH',7,88),100.06)
        self.assertEqual((self.bridge.pose['route_generation'],self.bridge.pose['execution_index']),(7,88))
        self.assertEqual(self.bridge.pose['stamp'],100.)

    def test_invalid_cursor_drops_pose_instead_of_mixing_new_pose_with_old_progress(self):
        self.ready()
        self.reply((wire.CAPS,bytes(4),'caps'),bytes((0,1,255,1)))
        self.reply((wire.POSE_LATCH,bytes(4),'pose_latch'),struct.pack('<HBB',42,15,9))
        self.reply(self.bridge.next_request(100.02),struct.pack('<hh',1200,300),100.02)
        self.reply(self.bridge.next_request(100.04),struct.pack('<hBB',-900,9,0x32),100.04)
        self.reply(self.bridge.next_request(100.06),wire.INVALID,100.06)
        self.assertFalse(self.bridge.pose['valid'])
        self.assertIsNone(self.bridge.pose_work)

    def test_continuous_route_requires_all_points_not_display_samples(self):
        self.ready();self.bridge.full_route_required=True;self.start_route(count=300)
        self.assertEqual(self.bridge.route_work['indices'],list(range(300)))
        for i in range(300):
            self.reply(self.bridge.next_request(100.02),struct.pack('<HH',i*10,100),100.02)
        self.reply(self.bridge.next_request(100.04),struct.pack('<HH',5,0x8000|300),100.04)
        self.assertFalse(self.bridge.route['sampled'])
        self.assertEqual(self.bridge.route['total_count'],300)

    def test_one_lost_route_point_retries_without_discarding_download(self):
        self.ready();self.bridge.full_route_required=True;self.start_route(count=40)
        self.reply(self.bridge.next_request(100.02),struct.pack('<HH',10,100),100.02)
        req=self.bridge.next_request(100.04)
        self.bridge.sent(201,req,100.04)
        self.bridge.expire(100.18)
        self.assertEqual(self.bridge.route_work['points'],{0:[10,100]})
        self.assertEqual(self.bridge.next_request(100.18),req)
        self.reply(req,struct.pack('<HH',20,100),100.18)
        self.assertEqual(len(self.bridge.route_work['points']),2)
        self.assertEqual(len(self.bridge.download),39)  # 38 points + final generation verification

    def test_lost_pose_reply_does_not_clear_independent_route_download(self):
        self.ready();self.start_route(count=40)
        self.reply(self.bridge.next_request(100.02),struct.pack('<HH',10,100),100.02)
        self.reply((wire.POSE_LATCH,bytes(4),'pose_latch'),struct.pack('<HBB',42,15,9),100.04)
        req=self.bridge.next_request(100.06)
        self.bridge.sent(202,req,100.06)
        self.bridge.expire(100.20)
        self.assertIsNone(self.bridge.pose_work)
        self.assertEqual(self.bridge.route_work['points'],{0:[10,100]})
        self.assertTrue(self.bridge.download)
        self.assertFalse(self.bridge.pose['valid'])

    def test_region_reference_is_readonly_mark_not_measured_pose(self):
        self.reply((wire.CAPS,bytes(4),'caps'),bytes((0,1,63,1)))
        self.assertTrue(self.bridge.snapshot(100.02)['region_supported'])
        self.ready()
        self.reply((wire.CAPS,bytes(4),'caps'),bytes((0,1,63,1)),100.02)
        self.reply((wire.REFERENCE_GENERATION,bytes(4),'reference_head'),struct.pack('<I',4),100.03)
        self.reply(self.bridge.next_request(100.05),bytes((17,17))+struct.pack('<h',-900),100.05)
        self.reply(self.bridge.next_request(100.07),struct.pack('<I',4),100.07)
        ref=self.bridge.snapshot(100.09)['start_reference']
        self.assertEqual((ref['cell'],ref['heading_deg']),([17,17],-90.))
        self.assertFalse(ref['is_live_pose'])
        self.assertEqual(ref['generation'],4)
        self.reply((wire.REFERENCE_GENERATION,bytes(4),'reference_head'),bytes(4),100.1)
        self.assertFalse(self.bridge.snapshot(100.12)['start_reference']['valid'])

    def test_reference_read_during_touch_rotation_is_rejected(self):
        self.ready()
        self.reply((wire.REFERENCE_GENERATION,bytes(4),'reference_head'),struct.pack('<I',4),100.03)
        self.reply(self.bridge.next_request(100.05),bytes((17,17))+struct.pack('<h',-900),100.05)
        self.reply(self.bridge.next_request(100.07),struct.pack('<I',5),100.07)
        self.assertFalse(self.bridge.snapshot(100.09)['start_reference']['valid'])

    def test_heading_capability_accepts_new_firmware_without_inventing_old_support(self):
        self.ready()
        self.assertFalse(self.bridge.snapshot(100.02)['heading_supported'])
        self.reply((wire.CAPS,bytes(4),'caps'),bytes((0,1,31,1)),100.03)
        self.assertTrue(self.bridge.snapshot(100.04)['heading_supported'])
        self.bridge.disconnected()
        self.assertFalse(self.bridge.snapshot(100.05)['heading_supported'])

    def setUp(self):
        self.bridge=wire.MapTelemetry(max_route_samples=8)
        self.seq=0

    def reply(self, request, data, now=100.):
        self.seq=(self.seq+1)%256
        self.bridge.sent(self.seq,request,now)
        return self.bridge.receive(Packet(self.seq,request[0]|0x80,data),now+.01)

    def ready(self):
        self.reply((wire.CAPS,bytes(4),'caps'),bytes((0,1,15,1)))
        self.reply((wire.MAP_ID,bytes(4),'map_id'),(123).to_bytes(4,'little'))
        self.bridge.next_pose=self.bridge.next_route=self.bridge.next_barriers=999.

    def test_old_firmware_does_not_start_pose_queries(self):
        self.reply(self.bridge.next_request(100.),bytes((5,0,0,0)))
        self.assertIs(self.bridge.supported,False)
        self.assertIsNone(self.bridge.next_request(200.))

    def test_one_pending_reply_only_and_timeout_is_bounded(self):
        request=self.bridge.next_request(100.)
        self.bridge.sent(7,request,100.)
        self.assertIsNone(self.bridge.next_request(100.05))
        self.assertFalse(self.bridge.receive(Packet(8,0xb0,bytes((0,1,15,1))),100.06))
        self.bridge.expire(100.13)
        self.assertIsNone(self.bridge.waiting)
        self.assertIsNone(self.bridge.next_request(101.))
        self.assertEqual(self.bridge.next_request(106.)[0],wire.CAPS)

    def test_pose_uses_single_token_not_two_live_reads(self):
        self.ready()
        self.reply((wire.POSE_LATCH,bytes(4),'pose_latch'),struct.pack('<HBB',42,15,9))
        xy=self.bridge.next_request(100.02)
        self.assertEqual(xy[1],struct.pack('<HH',42,0))
        self.reply(xy,struct.pack('<hh',1200,300),100.02)
        angle=self.bridge.next_request(100.04)
        self.reply(angle,struct.pack('<hBB',-900,9,0x32),100.04)
        self.assertEqual((self.bridge.pose['x_mm'],self.bridge.pose['y_mm'],self.bridge.pose['heading_tenths']), (1200,300,-900))
        self.assertTrue(self.bridge.pose['valid'])
        self.assertEqual(self.bridge.pose['stamp'],100.)

    def test_invalid_token_drops_whole_pose(self):
        self.ready()
        self.reply((wire.POSE_LATCH,bytes(4),'pose_latch'),struct.pack('<HBB',42,15,9))
        self.reply(self.bridge.next_request(100.02),wire.INVALID,100.02)
        self.assertIsNone(self.bridge.pose_work)
        self.assertFalse(self.bridge.urgent)
        self.assertFalse(self.bridge.pose.get('valid'))

    def test_invalid_pose_flags_immediately_clear_last_car(self):
        self.ready();self.bridge.pose=dict(stamp=100.,valid=True,x_mm=200,y_mm=300)
        self.reply((wire.POSE_LATCH,bytes(4),'pose_latch'),struct.pack('<HBB',43,0,9))
        self.assertFalse(self.bridge.pose['valid'])
        self.assertFalse(self.bridge.urgent)

    def test_publish_heartbeat_does_not_refresh_uart_evidence(self):
        self.ready()
        self.assertAlmostEqual(self.bridge.snapshot(500.)['stamp'],100.01)
        self.bridge.disconnected()
        self.assertIsNone(self.bridge.snapshot(500.)['stamp'])

    def start_route(self,generation=5,count=40):
        self.reply((wire.ROUTE_HEADER,bytes(4),'route_head'),struct.pack('<HH',generation,0x8000|count))

    def test_sampled_route_needs_final_unchanged_generation(self):
        self.ready();self.start_route()
        indices=self.bridge.route_work['indices']
        self.assertEqual((len(indices),indices[0],indices[-1]),(8,0,39))
        for i in indices:
            req=self.bridge.next_request(100.02)
            self.assertEqual(struct.unpack('<HH',req[1]),(5,i))
            self.reply(req,struct.pack('<HH',i*10,100),100.02)
            self.assertFalse(self.bridge.route)
        req=self.bridge.next_request(100.04)
        self.reply(req,struct.pack('<HH',5,0x8000|40),100.04)
        self.assertEqual(self.bridge.route['points_mm'][-1],[390,100])
        self.assertTrue(self.bridge.route['sampled'])

    def test_replan_cannot_publish_mixed_route(self):
        self.ready();self.start_route(count=2)
        for i in range(2):self.reply(self.bridge.next_request(100.02),struct.pack('<HH',i*10,100),100.02)
        self.reply(self.bridge.next_request(100.04),struct.pack('<HH',6,0x8002),100.04)
        self.assertFalse(self.bridge.route)
        self.assertIsNone(self.bridge.route_work)
        self.start_route(generation=7,count=2)
        self.reply(self.bridge.next_request(100.05),wire.INVALID,100.05)
        self.assertFalse(self.bridge.route)

    def test_barriers_are_generation_checked_and_empty_is_known_empty(self):
        self.ready()
        self.reply((wire.BARRIER_HEADER,bytes(4),'barrier_head'),struct.pack('<HBB',8,0,0))
        self.reply(self.bridge.next_request(100.02),struct.pack('<HBB',8,0,0),100.02)
        self.assertEqual(self.bridge.barriers['segments_mm'],[])
        self.assertTrue(self.bridge.barriers['valid'])
        self.reply((wire.BARRIER_HEADER,bytes(4),'barrier_head'),struct.pack('<HBB',9,1,0))
        self.reply(self.bridge.next_request(100.03),struct.pack('<HH',200,300),100.03)
        self.reply(self.bridge.next_request(100.04),struct.pack('<HH',400,500),100.04)
        self.reply(self.bridge.next_request(100.05),struct.pack('<HBB',9,1,0),100.05)
        self.assertEqual(self.bridge.barriers['segments_mm'],[[200,300,400,500]])

    def test_ui_never_places_car_at_operator_start_or_goal(self):
        catalog=load_catalog(Path(__file__).resolve().parents[1]/'config/fpga_map_catalog.json')
        view=live_map_snapshot(catalog,dict(point_reference=dict(point_id=2,confirmed=True)),
                               dict(target_point=6),100.,self.bridge.snapshot(100.))
        self.assertIsNone(view['live']['pose'])
        self.assertFalse(view['reference']['is_live_pose'])
        self.assertEqual(view['selection']['requested_target'],6)


if __name__=='__main__':unittest.main()
