import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "rescue_vision"))
from rescue_vision.traffic import TrafficController, TrafficDetector


def board_image(color="green", panel=True, wrong_position=False):
    image = np.full((240, 320, 3), 185, dtype=np.uint8)
    if panel:
        cv2.rectangle(image, (140, 60), (180, 160), (12, 12, 12), -1)
    if color in ("red", "both"):
        cv2.circle(image, (160, 87), 7, (0, 0, 255), -1)
    if color in ("green", "both"):
        cv2.circle(image, (160, 87 if wrong_position else 133), 7, (0, 255, 0), -1)
    return image


def obs(t, color="green", bbox=None, frame_id=None):
    return {"capture_time": t, "frame_id": str(t) if frame_id is None else frame_id,
            "status": color, "confidence": 0.9,
            "fixture_bbox": bbox or [140, 60, 180, 160]}


class DetectorTests(unittest.TestCase):
    def test_only_supported_board_and_color_pass(self):
        d = TrafficDetector()
        self.assertEqual(d.detect(board_image("red"))["status"], "red")
        self.assertEqual(d.detect(board_image())["status"], "green")
        self.assertEqual(d.detect(board_image(panel=False))["status"], "unknown")
        self.assertEqual(d.detect(board_image(wrong_position=True))["status"], "unknown")
        self.assertEqual(d.detect(np.full((240, 320, 3), (0, 255, 0), np.uint8))["status"], "unknown")

    def test_conflict_and_glare_stop(self):
        d = TrafficDetector()
        self.assertEqual(d.detect(board_image("both"))["status"], "conflict")
        image = board_image("off")
        cv2.rectangle(image, (145, 80), (175, 120), (255, 255, 255), -1)
        self.assertEqual(d.detect(image)["status"], "overexposed")

    def test_roi_and_multiple_panels(self):
        image = board_image()
        image[:, 60:101] = image[:, 140:181]
        self.assertEqual(TrafficDetector().detect(image)["status"], "unknown")
        self.assertEqual(TrafficDetector({"roi_normalized": [0.4, 0.05, 0.9, 0.9]}).detect(image)["status"], "green")


class ControllerTests(unittest.TestCase):
    def ready(self):
        c = TrafficController()
        self.assertTrue(c.on_crosswalk("event-1", 0.0))
        c.tick(0.0, vehicle_stopped=True, gimbal_ready=True)
        return c

    def feed(self, c, start=.31, count=7, color="green", moving=False):
        for i in range(count):
            t = start + i*.1
            out = c.tick(t, obs(t, color), vehicle_stopped=not moving, gimbal_ready=True)
        return out

    def test_stop_and_gimbal_ready_are_required(self):
        c = TrafficController()
        c.on_crosswalk("event-1", 0)
        for t in (.1, .2, .3, .4, .5, .6):
            self.assertFalse(c.tick(t, obs(t), False, True)["move_allowed"])
        self.assertEqual(c.state, "WAIT_STOP")
        self.assertFalse(c.tick(.7, obs(.7), True, False)["move_allowed"])
        self.assertFalse(c.tick(.8, obs(.8), True, True)["move_allowed"])
        self.assertFalse(c.tick(1.0, obs(1.0), True, True)["move_allowed"])

    def test_complete_red_to_green_sequence_and_dedup(self):
        c = self.ready()
        self.assertEqual(self.feed(c, color="red")["state"], "RED_HOLD")
        self.assertFalse(c.on_crosswalk("event-1", 1.0))
        self.assertFalse(c.on_crosswalk("event-2", 1.0))
        out = self.feed(c, start=1.01)
        self.assertTrue(out["move_allowed"])
        self.assertEqual(out["crosswalk_id"], "event-1")
        self.assertIsNone(out["actual_pan_deg"])
        self.assertFalse(c.on_crossing_complete("wrong-event"))
        self.assertTrue(c.on_crossing_complete("event-1"))
        self.assertFalse(c.tick(2.0)["traffic_active"])
        self.assertFalse(c.on_crosswalk("event-1", 2.0))
        self.assertTrue(c.on_crosswalk("event-2", 2.0))

    def test_green_flicker_does_not_release(self):
        c = self.ready()
        for i in range(15):
            t = .31+i*.1
            status = "green" if i % 3 else "unknown"
            self.assertFalse(c.tick(t, obs(t, status), True, True)["move_allowed"])

    def test_red_and_conflict_immediately_revoke(self):
        for color in ("red", "conflict", "overexposed", "unknown"):
            c = self.ready()
            self.assertTrue(self.feed(c)["move_allowed"])
            out = c.tick(1.02, obs(1.02, color), False, True)
            self.assertFalse(out["move_allowed"])
            self.assertFalse(c.tick(1.12, obs(1.12), False, True)["move_allowed"])

    def test_moving_after_green_keeps_permission_until_red_or_complete(self):
        c = self.ready()
        self.assertTrue(self.feed(c)["move_allowed"])
        out = self.feed(c, start=1.01, count=12, moving=True)
        self.assertTrue(out["move_allowed"])
        self.assertEqual(out["state"], "RELEASED")
        self.assertTrue(c.on_crossing_complete("event-1"))
        self.assertFalse(c.tick(2.3, vehicle_stopped=False, gimbal_ready=True)["traffic_active"])
        c = self.ready()
        self.feed(c)
        self.assertFalse(c.tick(1.01, obs(1.01, "red"), False, True)["move_allowed"])

    def test_grant_expires_and_stale_repeat_cannot_refresh(self):
        c = self.ready()
        out = self.feed(c)
        self.assertTrue(out["move_allowed"])
        self.assertFalse(c.tick(1.30, obs(.91), False, True)["move_allowed"])
        c = self.ready()
        for t in (.31, .41, .51, .61, .71):
            out = c.tick(t, obs(.31, frame_id="frozen"), True, True)
            self.assertFalse(out["move_allowed"])
        c = self.ready()
        self.feed(c)
        self.assertFalse(c.tick(1.4, None, False, True)["move_allowed"])

    def test_spatial_change_restarts_confirmation(self):
        c = self.ready()
        for i in range(12):
            t = .31+i*.1
            bbox = [10, 60, 50, 160] if i % 2 else [140, 60, 180, 160]
            self.assertFalse(c.tick(t, obs(t, bbox=bbox), True, True)["move_allowed"])

    def test_gap_out_of_order_and_pre_settle_frames(self):
        c = self.ready()
        self.assertFalse(c.tick(.31, obs(.1), True, True)["move_allowed"])
        self.feed(c, start=.32, count=3)
        out = c.tick(.82, obs(.82), True, True)
        self.assertEqual(out["green_confirmation_frames"], 1)
        out = c.tick(.83, obs(.81, frame_id="out-of-order"), True, True)
        self.assertEqual(out["green_confirmation_frames"], 1)

    def test_scan_dwell_bound_and_timeout(self):
        c = self.ready()
        self.assertEqual(c.tick(.4, obs(.4, "unknown"), True, True)["desired_pan_deg"], 0)
        self.assertEqual(c.tick(.8, obs(.8, "unknown"), True, True)["desired_pan_deg"], -20)
        for i in range(1, 35):
            t = .8+i*.8
            out = c.tick(t, obs(t, "unknown"), True, True)
            self.assertGreaterEqual(out["desired_pan_deg"], -60)
            self.assertLessEqual(out["desired_pan_deg"], 60)
            self.assertFalse(out["move_allowed"])
        self.assertEqual(c.tick(46, None, True, True)["state"], "TIMEOUT")
        self.assertFalse(c.on_crosswalk("new", 47))
        c.reset()
        self.assertTrue(c.on_crosswalk("new", 48))

    def test_scan_reaches_left_boundary_then_reverses(self):
        c = self.ready()
        commands = []
        last = 0
        for index in range(1, 100):
            t = index * .1
            out = c.tick(t, obs(t, "unknown"), True, True)
            if out["desired_pan_deg"] != last:
                last = out["desired_pan_deg"]
                commands.append(last)
        self.assertEqual(commands[:6], [-20.0, -40.0, -60.0, -40.0, -20.0, 0.0])
        self.assertIn(60.0, commands)


if __name__ == "__main__":
    unittest.main(verbosity=2)
