"""Real ROS node plus fake detector: prove disabled tasks do not invoke inference."""
import os
import json
import unittest
import time
import threading
from types import SimpleNamespace


@unittest.skipUnless(os.environ.get('RESCUE_RUN_ROS_TEST')=='1','Explicit isolated X5 test only')
class FastTaskGateTests(unittest.TestCase):
    def test_dialogue_rate_policy_restores_full_rate_for_control_and_stale_state(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        import rclpy
        from std_msgs.msg import String
        from rescue_vision.fast_mipi_perception_node import FastMipiPerception
        rclpy.init(args=['--ros-args', '-p', 'detector_enabled:=false', '-p', 'lens_enabled:=false'])
        node = FastMipiPerception()
        quality = (node.output_size, node.jpeg_quality, node.classes[:], node.max_fps,
                   node.input_rotation, node.source_topic)
        def update(**changes):
            value = dict(stamp=node.get_clock().now().nanoseconds/1e9, mode='inspect',
                traffic_active=False, auto_fire_active=False, pump_armed=False, motion_allowed=False)
            value.update(changes)
            node.mission_state(String(data=json.dumps(value)))
            node.dialogue_state(String(data=json.dumps(dict(
                stamp=node.get_clock().now().nanoseconds/1e9, busy=True, state='thinking'))))
        try:
            self.assertEqual(node.effective_process_fps(), 30.)
            update(); self.assertEqual(node.effective_process_fps(), 6.)
            for changes in (dict(mode='fire'), dict(traffic_active=True), dict(mode='navigate'),
                            dict(pump_armed=True), dict(auto_fire_active=True), dict(motion_allowed=True)):
                update(**changes); self.assertEqual(node.effective_process_fps(), 30.)
            update()
            node.workload_dialogue = (node.workload_dialogue[0], time.monotonic()-3.)
            self.assertEqual(node.effective_process_fps(), 30.)
            update()
            node.workload_mission = (node.workload_mission[0], time.monotonic()-1.)
            self.assertEqual(node.effective_process_fps(), 30.)
            update()
            node.dialogue_state(String(data='not-json'))
            self.assertEqual(node.effective_process_fps(), 30.)
            self.assertEqual(quality, (node.output_size, node.jpeg_quality, node.classes[:],
                node.max_fps, node.input_rotation, node.source_topic))
            self.assertEqual(node.output_size, (1280, 1088))
            self.assertEqual(node.jpeg_quality, 90)
        finally:
            node.destroy_node(); rclpy.shutdown()

    def test_both_actual_workers_limit_to_six_then_resume_without_backlog(self):
        """Exercise the real wait/LatestSlot loops with cheap synthetic computation.

        This measures scheduler cadence, not BPU throughput or camera quality.
        No camera, detector, actuator or microphone driver is started.
        """
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        import rclpy
        from std_msgs.msg import Header, String
        from rescue_vision.fast_mipi_perception_node import FastMipiPerception
        rclpy.init(args=['--ros-args', '-p', 'detector_enabled:=false', '-p', 'lens_enabled:=false'])
        node = FastMipiPerception()
        prepared, inferred = [], []
        def prepare(source, sequence):
            prepared.append(time.monotonic())
            return source
        def infer(frame):
            inferred.append((time.monotonic(), frame.sequence))
        node.prepare, node.infer = prepare, infer
        def feed(mode, duration):
            begin = time.monotonic()
            start_prepare, start_infer = len(prepared), len(inferred)
            last_refresh = 0.
            sequence = 0
            while time.monotonic()-begin < duration:
                now = time.monotonic()
                if now-last_refresh > .1:
                    stamp = node.get_clock().now().nanoseconds/1e9
                    node.mission_state(String(data=json.dumps(dict(stamp=stamp, mode=mode,
                        traffic_active=False, auto_fire_active=False, pump_armed=False, motion_allowed=False))))
                    node.dialogue_state(String(data=json.dumps(dict(stamp=stamp, busy=True, state='thinking'))))
                    last_refresh = now
                header = Header(); header.stamp = node.get_clock().now().to_msg()
                sequence += 1
                node.images.put(SimpleNamespace(header=header, sequence=sequence))
                time.sleep(.01)
            elapsed = time.monotonic()-begin
            return ((len(prepared)-start_prepare)/elapsed, (len(inferred)-start_infer)/elapsed)
        try:
            slow = feed('inspect', 1.7)
            fast = feed('fire', 1.2)
            for rate in slow:
                self.assertGreater(rate, 3.)
                self.assertLess(rate, 7.5)
            for rate in fast:
                self.assertGreater(rate, 15.)
                self.assertLessEqual(rate, 31.)
            self.assertEqual(node.effective_process_fps(), 30.)
            self.assertEqual(node.max_fps, 30.)
            print('Synthetic worker cadence: inspect=%s Hz, fire=%s Hz' % (slow, fast))
        finally:
            node.destroy_node(); rclpy.shutdown()

    def test_prepared_slot_drops_backlog_with_deterministically_blocked_inference(self):
        """Force a backlog explicitly, independently of the board's remap speed."""
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '83')
        import rclpy
        from std_msgs.msg import Header
        from rescue_vision.fast_mipi_perception_node import FastMipiPerception
        rclpy.init(args=['--ros-args', '-p', 'detector_enabled:=false', '-p', 'lens_enabled:=false'])
        node = FastMipiPerception()
        ready = threading.Event()
        prepared, inferred = [], []
        def prepare(source, sequence):
            prepared.append(sequence)
            if len(prepared) >= 3:
                ready.set()
            return source
        def infer(frame):
            if not inferred:
                ready.wait(timeout=2.)
            inferred.append(frame.sequence)
        node.prepare, node.infer = prepare, infer
        try:
            deadline = time.monotonic()+3.
            sequence = 0
            while time.monotonic() < deadline and len(inferred) < 3:
                header = Header(); header.stamp = node.get_clock().now().to_msg()
                sequence += 1
                node.images.put(SimpleNamespace(header=header, sequence=sequence))
                time.sleep(.01)
            self.assertTrue(ready.is_set())
            self.assertGreaterEqual(len(inferred), 3)
            self.assertGreater(node.dropped_prepared, 0)
            self.assertEqual(inferred, sorted(set(inferred)))
            self.assertGreater(inferred[1]-inferred[0], 1)
        finally:
            ready.set()
            node.destroy_node(); rclpy.shutdown()

    def test_mode_switch_skips_bpu_and_preserves_camera_image_geometry(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'),'83')
        import rclpy
        from rclpy.parameter import Parameter
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from rescue_vision.fast_mipi_perception_node import FastMipiPerception
        class Detector:
            def __init__(self):self.prepares=0;self.calls=0;self.last_timings={}
            def prepare(self,image):self.prepares+=1;return image
            def predict_prepared(self,image):self.calls+=1;return []
        rclpy.init(args=['--ros-args','-p','detector_enabled:=false','-p','lens_enabled:=false','-p','mission_gating:=true'])
        node=FastMipiPerception();detector=Detector();node.detector=detector
        def frame(sequence):
            image=Image();image.header.stamp=node.get_clock().now().to_msg()
            image.width=1280;image.height=1088;image.step=1280;image.encoding='mono8'
            image.data.frombytes(bytes(1280*1088))
            node.process(image,sequence)
        def mission(mode):
            node.mission_state(String(data=json.dumps(dict(stamp=node.get_clock().now().nanoseconds/1e9,
                                                         mode=mode,traffic_active=False))))
        try:
            frame(1);self.assertEqual(detector.calls,0)
            mission('patrol');frame(2);self.assertEqual(detector.prepares,0)
            mission('inspect');frame(3);self.assertEqual(detector.calls,1)
            self.assertTrue(node.performance['detector_active'])
            mission('idle');frame(4);self.assertEqual(detector.calls,1)
            self.assertFalse(node.performance['detector_active'])
            self.assertEqual(node.performance['output_resolution'],[1280,1088])
            result=node.set_parameters([Parameter('mission_gating',Parameter.Type.BOOL,False)])
            self.assertTrue(result[0].successful)
            frame(5);self.assertEqual(detector.calls,2)
        finally:
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
