"""Actual sidecar publication method, isolated from ROS and camera drivers."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


class DepthProvenance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=Path(__file__).resolve().parents[1]/'src/rescue_vision/rescue_vision/stereo_depth_node.py'
        tree=ast.parse(path.read_text('utf-8'))
        node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='StereoDepthNode')
        fn=next(n for n in node.body if isinstance(n,ast.FunctionDef) and n.name=='_publish')
        module=ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[]))
        scope=dict(copy=copy,age=lambda *_:0.,stamp_nanoseconds=lambda stamp:stamp.sec*10**9+stamp.nanosec)
        exec(compile(module,str(path),'exec'),scope)
        cls.publish=staticmethod(scope['_publish'])

    def test_only_verified_computed_depth_gets_matching_id_and_stale_id_is_cleared(self):
        for gate,reason,estimates,expected in [
                ('disabled','disabled',None,''),('ready','matching_right_frame_missing',None,''),
                ('disabled','computed',{0:dict(valid=True,reason='ok',depth_m=1.,xyz_m=[.1,.2,1.])},''),
                ('ready','computed',{0:dict(valid=False,reason='insufficient_depth')},''),
                ('ready','computed',{0:dict(valid=True,reason='ok',depth_m=1.,xyz_m=[.1,.2,1.])},'MEASURED_ID')]:
            with self.subTest(gate=gate,reason=reason,expected=expected):
                output=[]
                node=NS(max_age=.5,gate=gate,stereo=NS(calibrated=True,calibration_id='MEASURED_ID'),
                        publisher=NS(publish=output.append),counts=dict(published=0,valid_depth=0))
                msg=NS(header=NS(stamp=NS(sec=100,nanosec=0),frame_id='undistorted_left_optical'),
                       sequence=1,status='detector',stereo_calibration_id='STALE_INPUT_ID',
                       detections=[NS(depth_valid=True,depth_m=999.,depth_reason='stale',position_m=NS(x=1,y=2,z=999))])
                self.publish(node,msg,reason,estimates)
                self.assertEqual(output[0].stereo_calibration_id,expected)
                self.assertEqual(output[0].detections[0].depth_valid,bool(expected))


if __name__=='__main__':unittest.main(verbosity=2)
