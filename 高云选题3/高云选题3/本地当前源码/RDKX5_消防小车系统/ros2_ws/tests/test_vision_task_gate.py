import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.vision_task_gate import VisionTaskGate


class GateTests(unittest.TestCase):
    def test_preview_compatibility_and_profile_modes(self):
        self.assertTrue(VisionTaskGate(False).allowed(1,1))
        gate=VisionTaskGate(True)
        self.assertFalse(gate.allowed(100,5))
        for mode,expected in [('idle',False),('patrol',False),('navigate',False),('inspect',True),('fire',True)]:
            gate.update(dict(stamp=100.,mode=mode,traffic_active=False),100.,5.)
            self.assertEqual(gate.allowed(100.,5.1),expected)

    def test_old_frame_transition_traffic_and_timeout(self):
        gate=VisionTaskGate(True)
        gate.update(dict(stamp=100.,mode='inspect',traffic_active=False),100.,5.)
        self.assertFalse(gate.allowed(99.99,5.1))
        self.assertTrue(gate.allowed(100.1,5.4))
        self.assertFalse(gate.allowed(100.1,5.51))
        gate.update(dict(stamp=100.2,mode='fire',traffic_active=True),100.2,5.2)
        self.assertFalse(gate.allowed(100.2,5.3))
        gate.update(dict(stamp=100.4,mode='inspect',traffic_active=False),100.4,5.4)
        self.assertFalse(gate.allowed(100.2,5.4))
        gate.update(dict(stamp=float('nan'),mode='inspect',traffic_active=False),100.5,5.5)
        self.assertFalse(gate.allowed(100.6,5.6))


if __name__=='__main__':unittest.main()
