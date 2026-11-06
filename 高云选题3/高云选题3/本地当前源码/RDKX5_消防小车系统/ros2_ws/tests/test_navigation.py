import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rescue_vision'))
from rescue_vision.navigation import RoadGraph,Odometry,Navigator


def enabled_config():
    c=json.loads((ROOT/'config/navigation.json').read_text(encoding='utf-8'))
    c.update(map_registration_verified=True,map_registration_id='synthetic-test',execution_adapter_verified=True,
             initial_pose_verified=True,initial_pose_xy_yaw=[0.,0.,0.])
    return c


def telemetry(stamp,ds=.01,**kw):
    return dict(stamp=stamp,mcu_time_ms=round(stamp*1000),delta_s_m=ds,yaw_rate_rad_s=0.,
                odometry_valid=True,gyro_valid=True,line_valid=True,estop=False,vehicle_stopped=False,**kw)


def tiny_map():
    return dict(units='m',nodes=[dict(id='a',xy_m=[0,0]),dict(id='b',xy_m=[.4,0])],
                edges=[dict(id='ab',source='a',target='b',length_m=.4,geometry_m=[[0,0],[.4,0]])],
                buildings=[dict(id='03',name='测试楼',observation_points=[dict(node_id='b')])],
                preview_start_node='a',preview_start_yaw_rad=0.)


class NavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw=json.loads((ROOT/'config/navigation_map.json').read_text(encoding='utf-8'))
        cls.config=json.loads((ROOT/'config/navigation.json').read_text(encoding='utf-8'))

    def test_all_13_buildings_reachable_on_actual_map(self):
        graph=RoadGraph(self.raw)
        self.assertEqual(len(graph.buildings),13)
        for b in graph.buildings:
            route=graph.plan(self.raw['preview_start_node'],b,self.raw['preview_start_yaw_rad'])
            self.assertGreater(route['length_m'],0)
            self.assertTrue({eid for e in route['segments'] for eid in e['constituent_edge_ids']}.isdisjoint(graph.blocked))

    def test_polyline_length_and_endpoint_continuity(self):
        graph=RoadGraph(self.raw)
        for edge in graph.edges.values():
            points=edge['geometry_m']
            self.assertLess(math.dist(points[0],graph.nodes[edge['source']]['xy_m']),1e-5)
            self.assertLess(math.dist(points[-1],graph.nodes[edge['target']]['xy_m']),1e-5)
            self.assertAlmostEqual(sum(math.dist(a,b) for a,b in zip(points,points[1:])),edge['length_m'],places=5)
        self.assertTrue(any(len(e['geometry_m'])>20 for e in graph.edges.values()),'Curves must retain source geometry')

    def test_blocked_edge_replans_once(self):
        nav=Navigator(self.raw,self.config)
        nav.request(dict(command_id='p1',issued_at=1.,ttl_s=10,building_id='03'),1.)
        old=nav.route['segments'][0]['constituent_edge_ids'][0];revision=nav.revision
        self.assertTrue(nav.block_edges([old],1.1))
        self.assertNotIn(old,[eid for e in nav.route['segments'] for eid in e['constituent_edge_ids']])
        self.assertEqual(nav.revision,revision+1)
        self.assertFalse(nav.block_edges([old],1.2))
        self.assertEqual(nav.revision,revision+1)

    def test_unknown_or_disconnected_route(self):
        graph=RoadGraph(tiny_map());graph.block(['ab'])
        with self.assertRaises(ValueError):graph.plan('a','03')
        with self.assertRaises(ValueError):graph.resolve_building('999')

    def test_default_gate_never_moves(self):
        nav=Navigator(self.raw,self.config)
        with self.assertRaises(ValueError):
            nav.request(dict(command_id='e1',issued_at=1.,ttl_s=10,building_id=3,mode='execute',confirmed=True),1.)
        nav.request(dict(command_id='p1',issued_at=1.,ttl_s=10,building_id=3,mode='preview'),1.)
        self.assertFalse(nav.proposal(1.1)['ready_to_move'])
        self.assertEqual(nav.proposal(1.1)['reason'],'preview_only')

    def test_missing_gyro_or_unverified_map_stops(self):
        n=Navigator(tiny_map(),enabled_config());n.odom.update(telemetry(1.,0),1.);n.update_telemetry(telemetry(1.1,0),1.1)
        n.request(dict(command_id='go',issued_at=1.1,ttl_s=5,building_id=3,mode='execute',confirmed=True),1.1)
        n.config['map_registration_verified']=False
        self.assertFalse(n.proposal(1.1)['ready_to_move'])
        t=telemetry(1.2,.01);t['gyro_valid']=False
        n.update_telemetry(t,1.2);self.assertFalse(n.odom.valid)

    def test_encoder_reverse_and_gyro(self):
        o=Odometry(enabled_config());o.update(telemetry(1.,0),1.)
        self.assertTrue(o.update(telemetry(1.1,.02),1.1));self.assertAlmostEqual(o.x,.02)
        self.assertTrue(o.update(telemetry(1.2,-.01),1.2));self.assertAlmostEqual(o.x,.01)
        t=telemetry(1.3,0);t['yaw_rate_rad_s']=1.
        o.update(t,1.3);self.assertAlmostEqual(o.yaw,.1)

    def test_unverified_absolute_heading_not_applied(self):
        o=Odometry(enabled_config());o.update(telemetry(1.,0),1.)
        o.update(telemetry(1.1,0,yaw_valid=True,yaw_rad=2.),1.1)
        self.assertAlmostEqual(o.yaw,0.)

    def test_duplicate_replay_gap_and_stale_stop(self):
        o=Odometry(enabled_config());o.update(telemetry(1.,0),1.)
        o.update(telemetry(1.1,.02),1.1);x=o.x
        self.assertFalse(o.update(telemetry(1.1,.02),1.2));self.assertEqual(o.x,x)
        self.assertFalse(o.fresh(1.2))
        self.assertFalse(o.update(telemetry(2.,.02),2.));self.assertFalse(o.initialized)
        self.assertFalse(o.fresh(2.5))

    def test_timestamp_wrap_and_drift_bound(self):
        config=enabled_config();config['max_position_uncertainty_m']=.016
        o=Odometry(config);a=telemetry(1.,0);a['mcu_time_ms']=2**32-50;o.update(a,1.)
        b=telemetry(1.1,.04);b['mcu_time_ms']=50
        self.assertTrue(o.update(b,1.1));self.assertAlmostEqual(o.x,.04)
        self.assertFalse(o.valid)

    def test_ocr_is_not_map_localization(self):
        o=Odometry(enabled_config())
        self.assertFalse(o.pose_fixture(dict(landmark_id='BUILDING_SIGN_C_0',building_id='03',stamp=1.,observation_verified=True,vehicle_stopped=True),1.))

    def test_pause_resume_and_cancel(self):
        n=Navigator(tiny_map(),enabled_config());n.odom.update(telemetry(1.,0),1.);n.update_telemetry(telemetry(1.1,0),1.1)
        n.request(dict(command_id='go',issued_at=1.1,ttl_s=5,building_id=3,mode='execute',confirmed=True),1.1)
        self.assertTrue(n.proposal(1.1)['ready_to_move'])
        n.request(dict(command_id='pause',issued_at=1.2,ttl_s=5,action='pause'),1.2)
        self.assertEqual(n.command_id,'go');self.assertFalse(n.proposal(1.2)['ready_to_move'])
        n.request(dict(command_id='resume',issued_at=1.3,ttl_s=5,action='resume'),1.3)
        self.assertTrue(n.proposal(1.3)['ready_to_move'])
        n.request(dict(command_id='cancel',issued_at=1.4,ttl_s=5,action='cancel'),1.4)
        self.assertIsNone(n.route)

    def test_only_matching_completion_and_stop_can_arrive(self):
        n=Navigator(tiny_map(),enabled_config());n.odom.update(telemetry(1.,0),1.);n.update_telemetry(telemetry(1.1,0),1.1)
        n.request(dict(command_id='go',issued_at=1.1,ttl_s=5,building_id=3,mode='execute',confirmed=True),1.1)
        for i in range(8):n.update_telemetry(telemetry(1.2+i*.1,.05),1.2+i*.1)
        self.assertAlmostEqual(n.odom.x,.4);self.assertFalse(n.arrived)
        t=telemetry(2.,0,maneuver_done=True,maneuver_command_id='wrong',maneuver_edge_id='ab',maneuver_route_revision=1);t['vehicle_stopped']=True
        n.update_telemetry(t,2.);self.assertFalse(n.arrived)
        t['stamp']=2.1;t['mcu_time_ms']=2100;t['maneuver_command_id']='go'
        n.update_telemetry(t,2.1);self.assertTrue(n.arrived);self.assertFalse(n.proposal(2.1)['ready_to_move'])
        self.assertFalse(n.proposal(3.)['valid']);self.assertFalse(n.proposal(3.)['arrived'])
        self.assertTrue(n.proposal(3.)['arrival_achieved_previously'])

    def test_premature_done_not_arrival(self):
        n=Navigator(tiny_map(),enabled_config());n.odom.update(telemetry(1.,0),1.);n.update_telemetry(telemetry(1.1,0),1.1)
        n.request(dict(command_id='go',issued_at=1.1,ttl_s=5,building_id=3,mode='execute',confirmed=True),1.1)
        t=telemetry(1.2,.01,maneuver_done=True,maneuver_command_id='go',maneuver_edge_id='ab',maneuver_route_revision=1);t['vehicle_stopped']=True
        n.update_telemetry(t,1.2);self.assertFalse(n.arrived);self.assertTrue(n.paused)

    def test_old_revision_cannot_complete_new_route(self):
        n=Navigator(tiny_map(),enabled_config());n.odom.update(telemetry(1.,0),1.);n.update_telemetry(telemetry(1.1,0),1.1)
        n.request(dict(command_id='go',issued_at=1.1,ttl_s=5,building_id=3,mode='execute',confirmed=True),1.1)
        t=telemetry(1.2,.01,maneuver_done=True,maneuver_command_id='go',maneuver_edge_id='ab',maneuver_route_revision=0)
        n.update_telemetry(t,1.2);self.assertFalse(n.arrived);self.assertFalse(n.paused)

    def test_expired_request_and_nan_rejected(self):
        n=Navigator(tiny_map(),enabled_config())
        with self.assertRaises(ValueError):n.request(dict(command_id='go',issued_at=1.,ttl_s=1,building_id=3),3.)
        n.odom.update(telemetry(1.,0),1.)
        self.assertFalse(n.odom.update(telemetry(1.1,float('nan')),1.1))
        self.assertFalse(n.odom.valid)


if __name__=='__main__':unittest.main(verbosity=2)
