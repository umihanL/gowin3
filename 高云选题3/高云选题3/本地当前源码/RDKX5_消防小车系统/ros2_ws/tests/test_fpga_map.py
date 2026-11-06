import importlib.util
import json
from pathlib import Path
import re
import sys
import unittest

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS/'src/rescue_vision'))
from rescue_vision.fpga_map import live_map_snapshot, load_catalog


class FpgaMapTest(unittest.TestCase):
    def setUp(self):
        self.catalog = load_catalog(WS/'config/fpga_map_catalog.json')
        self.extension = dict(real_uart=True, protocol_ready=True, status_fresh=True,
            point_reference=dict(confirmed=True, point_id=2, source='operator', stamp=99))
        self.navigation = dict(stamp=100., status_fresh=True, start_point=2, target_point=5,
                               selected_point=dict(selected_point=4))
        self.telemetry = dict(supported=True, stamp=100., map_signature=self.catalog['map_signature'],
            pose=dict(valid=True, stamp=100., x_mm=350, y_mm=2850, heading_tenths=900, mode=9, status=2, phase=0),
            route=dict(valid=True, stamp=95., generation=2, total_count=20, points_mm=[[350,350],[1770,1730]]),
            barriers=dict(valid=True, stamp=100., generation=3, segments_mm=[[200,300,500,300]]))

    def snapshot(self):
        return live_map_snapshot(self.catalog, self.extension, self.navigation, 100., self.telemetry)

    def test_exact_rom_and_point_geometry(self):
        fpga = WS.parents[1]/'ov5640'
        if not fpga.exists():
            self.skipTest('source FPGA checkout is only required for source-identity test')
        raw = [1]*(352*320)
        for row, start, length in self.catalog['geometry']['road_runs']:
            raw[row*352+start:row*352+start+length] = [0]*length
        flat = [int(v) for v in (fpga/'src/navigation/map_raw_1cm.mem').read_text().split()]
        self.assertEqual(raw, flat)
        constant = (fpga/'src/navigation/nav_host_map_id.vh').read_text()
        self.assertIn(f"32'h{self.catalog['map_signature']:08x}", constant)
        for point in self.catalog['points']:
            self.assertEqual(point['raster_cm']['y']+point['display_cm']['y'], 320)

    def test_live_coordinate_units_and_reference_not_pose(self):
        result = self.snapshot()
        self.assertEqual(result['live']['pose']['x_cm'], 35)
        self.assertEqual(result['live']['pose']['y_cm'], 35)
        self.assertEqual(result['live']['pose']['heading_deg'], 90)
        self.assertEqual(result['live']['route']['points_cm'], [[35,35],[177,173]])
        self.assertTrue(result['live']['route']['sampled'])
        self.assertEqual(result['live']['barriers']['segments_cm'], [[20,30,50,30]])
        self.assertEqual(result['selection']['requested_target'], 5)
        self.assertEqual(result['selection']['fpga_selected_point'], 4)
        self.assertFalse(result['reference']['is_live_pose'])

    def test_offline_keeps_basemap_without_fake_telemetry(self):
        result = live_map_snapshot(self.catalog, {}, {}, 100.)
        self.assertEqual(len(result['points']), 9)
        self.assertEqual(result['geometry'], self.catalog['geometry'])
        for key in ('pose', 'route', 'barriers'):
            self.assertIsNone(result['live'][key])

    def test_fake_stale_or_unverified_transport_hides_live(self):
        for field in ('real_uart', 'protocol_ready', 'status_fresh'):
            with self.subTest(field=field):
                self.extension[field] = False
                self.assertIsNone(self.snapshot()['live']['pose'])
                self.extension[field] = True
        self.telemetry['stamp'] = 96.
        self.assertFalse(self.snapshot()['live']['fresh'])
        self.telemetry['stamp'] = 100.
        self.telemetry['map_signature'] ^= 1
        result = self.snapshot()
        self.assertEqual(result['live']['reason'], 'map_signature_mismatch')
        self.assertIsNone(result['live']['route'])

    def test_individual_expiry_unknown_and_invalid_no_fabrication(self):
        self.telemetry['pose']['stamp'] = 97.
        self.assertIsNone(self.snapshot()['live']['pose'])
        self.assertIsNotNone(self.snapshot()['live']['route'])
        self.telemetry['pose']['stamp'] = 100.
        self.telemetry['pose']['mode'] = 2  # Relative jog coordinates are not global map pose.
        self.assertIsNone(self.snapshot()['live']['pose'])
        self.telemetry['route']['generation'] = True
        self.assertIsNone(self.snapshot()['live']['route'])
        self.telemetry['barriers']['segments_mm'] = [[0, float('nan'), 0, 0]]
        self.assertIsNone(self.snapshot()['live']['barriers'])

    def test_empty_barrier_snapshot_is_known_clear_not_unknown(self):
        self.telemetry['barriers']['segments_mm'] = []
        self.assertEqual(self.snapshot()['live']['barriers']['segments_cm'], [])


if __name__ == '__main__':
    unittest.main()
