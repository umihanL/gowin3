import sys
import threading
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'rescue_vision'))
from rescue_vision.latest_slot import LatestSlot


class LatestSlotTests(unittest.TestCase):
    def test_busy_consumer_skips_old_values_and_does_not_replay(self):
        slot = LatestSlot()
        for value in range(100):
            slot.put(value)
        self.assertEqual(slot.take_after(0), (100, 99))
        self.assertIsNone(slot.take_after(100, timeout=0))
        slot.put(101)
        self.assertEqual(slot.take_after(100), (101, 101))

    def test_shutdown_wakes_waiter_and_rejects_more_frames(self):
        slot = LatestSlot()
        result = []
        thread = threading.Thread(target=lambda: result.append(slot.take_after(0, timeout=5)))
        thread.start()
        slot.close()
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [None])
        self.assertFalse(slot.put('late'))
        self.assertIsNone(slot.take_after(0))


if __name__ == '__main__':
    unittest.main()
