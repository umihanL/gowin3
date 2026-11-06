"""Check offline rehearsal against known ray geometry; no hardware interfaces."""
import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np

PATH=Path(__file__).resolve().parents[1]/'tools/preview_fixed_camera_calibration.py'
SPEC=importlib.util.spec_from_file_location('geometry_rehearsal',PATH)
rehearsal=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rehearsal)


class RehearsalPhysicsTests(unittest.TestCase):
    def test_ray_plane_intersection_matches_known_one_degree_error(self):
        doc=rehearsal.illustrative_geometry()
        doc['tilt_origin_in_pan_m']=doc['nozzle_origin_in_tilt_m']=[0,0,0]
        model=rehearsal.FixedCameraAim(doc)
        result=rehearsal.plane_error(model,[1.,0.],np.array([1.,0.,0.]))
        self.assertAlmostEqual(result['straight_ray_error_mm'],1000*math.tan(math.radians(1)),places=9)
        self.assertAlmostEqual(result['vertical_error_mm'],0.)

    def test_solved_assumed_geometry_has_negligible_plane_error(self):
        rows=rehearsal.sensitivity_rows()
        self.assertEqual(len(rows),21)
        nominal=[r for r in rows if r['scenario']=='ideal_assumptions']
        self.assertTrue(all(r['straight_ray_error_mm'] < .01 for r in nominal))
        errors=[r['straight_ray_error_mm'] for r in rows if r['scenario']=='pan_zero_plus_1_deg']
        self.assertEqual(errors,sorted(errors))
        self.assertGreater(errors[-1],17.)


if __name__=='__main__':
    unittest.main(verbosity=2)
