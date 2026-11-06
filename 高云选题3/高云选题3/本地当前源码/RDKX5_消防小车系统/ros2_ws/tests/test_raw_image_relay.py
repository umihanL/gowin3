import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.raw_image_relay import RawImageRelay


class RelayTests(unittest.TestCase):
    def demand(self, **changes):
        value = dict(stamp=10., process_fps_limit=3., source_topic='/image_right_raw',
            raw_input_topic='/rescue/camera/road_raw', process_on_receive=True,
            raw_subscription_active=True)
        value.update(changes)
        return value

    def relay(self):
        return RawImageRelay('/image_right_raw', '/rescue/camera/road_raw')

    def test_camera_rate_reduced_to_existing_analysis_rate(self):
        for rate in (3, 10, 15):
            relay = self.relay()
            relay.update(self.demand(process_fps_limit=rate), 10., 20.)
            self.assertEqual(sum(relay.due(20.+i/30+1e-8, 1) for i in range(90)), rate*3)

    def test_invalid_stale_wrong_source_and_inactive_demands_stop_relay(self):
        for changes in ({'stamp': 6.}, {'stamp': 11.}, {'process_fps_limit': math.nan},
                        {'process_fps_limit': 30.}, {'source_topic': '/image_left_raw'},
                        {'process_on_receive': False}, {'raw_subscription_active': False},
                        {'raw_input_topic': '/another'}):
            relay = self.relay()
            relay.update(self.demand(), 10., 20.)
            relay.update(self.demand(**changes), 10., 20.)
            self.assertFalse(relay.due(20., 1), changes)

    def test_no_reader_expired_report_and_silence_never_build_backlog(self):
        relay = self.relay()
        relay.update(self.demand(), 10., 20.)
        self.assertFalse(relay.due(20., 0))
        self.assertTrue(relay.due(20., 1))
        self.assertTrue(relay.due(22., 1))
        self.assertFalse(relay.due(22.01, 1))
        self.assertFalse(relay.due(23.01, 1))
        relay.update(self.demand(stamp=14.), 14., 24.)
        self.assertTrue(relay.due(24., 1))

    def test_periodic_reports_do_not_reset_cadence(self):
        relay = self.relay()
        frames = 0
        for i in range(300):
            if i % 30 == 0:
                relay.update(self.demand(stamp=10.+i/30), 10.+i/30, 20.+i/30)
            frames += relay.due(20.+i/30+1e-8, 1)
        self.assertEqual(frames, 30)

    def test_delayed_and_out_of_order_reports_cannot_extend_lease(self):
        relay = self.relay()
        relay.update(self.demand(), 12.9, 22.9)
        self.assertTrue(relay.due(22.9, 1))
        self.assertFalse(relay.due(23.01, 1))
        relay.update(self.demand(stamp=15.), 15., 25.)
        relay.update(self.demand(stamp=14., process_fps_limit=10.), 15., 25.)
        self.assertEqual(relay.rate, 3.)
        self.assertEqual(relay.expires, 28.)


if __name__ == '__main__':
    unittest.main()
