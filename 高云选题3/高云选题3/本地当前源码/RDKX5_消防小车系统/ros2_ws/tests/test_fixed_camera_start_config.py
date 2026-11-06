"""Startup selects measured files; no calibration values or zero IDs invented."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from test_fixed_camera_aim import document
from test_fixed_camera_landing import landing_document

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('fixed_camera_start_config', ROOT/'tools/fixed_camera_start_config.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FixedCameraStartupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root/'config').mkdir()
        self.template = self.root/'config/fixed_camera_aim.template.json'
        self.template.write_bytes((ROOT/'config/fixed_camera_aim.template.json').read_bytes())
        self.local = self.root/'config/fixed_camera_aim.local.json'

    def tearDown(self):
        self.temp.cleanup()

    def test_missing_local_explicitly_uses_unmeasured_template(self):
        chosen, calibrated = module.resolve(self.root)
        self.assertEqual(chosen, self.template.resolve())
        self.assertFalse(calibrated)

    def test_valid_local_is_selected_without_rewriting_reference_or_geometry(self):
        raw = json.dumps(document()).encode()
        self.local.write_bytes(raw)
        chosen, calibrated = module.resolve(self.root)
        self.assertEqual(chosen, self.local.resolve())
        self.assertTrue(calibrated)
        self.assertEqual(self.local.read_bytes(), raw)
        self.assertEqual(json.loads(raw)['actuator_reference_id'], 'SYNTHETIC_AXES')

    def test_explicit_file_wins_and_can_be_workspace_relative(self):
        self.local.write_text('{bad-json')
        other = self.root/'config/measured camera.json'
        other.write_text(json.dumps(document()))
        self.assertEqual(module.resolve(self.root, 'config/measured camera.json')[0], other.resolve())

    def test_explicit_missing_file_never_falls_back_to_valid_local(self):
        self.local.write_text(json.dumps(document()))
        with self.assertRaises(OSError):
            module.resolve(self.root, 'config/missing.json')

    def test_invalid_existing_local_never_silently_uses_template(self):
        for content in ('{bad-json', '[]', '{}', self.template.read_text()):
            self.local.write_text(content)
            with self.assertRaises((ValueError, TypeError, KeyError)):
                module.resolve(self.root)
        self.local.unlink(); self.local.mkdir()
        with self.assertRaises(OSError):
            module.resolve(self.root)

    def test_calibrated_flag_cannot_bypass_existing_geometry_validator(self):
        value = document()
        value['R_camera_to_gimbal'] = [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
        self.local.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'determinant'):
            module.resolve(self.root)


class FixedCameraLandingStartupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root/'config').mkdir()
        self.template = self.root/'config/fixed_camera_landing.template.json'
        self.template.write_bytes((ROOT/'config/fixed_camera_landing.template.json').read_bytes())
        self.local = self.root/'config/fixed_camera_landing.local.json'

    def tearDown(self):
        self.temp.cleanup()

    def resolve(self, explicit=None):
        return module.resolve(self.root, explicit, kind='landing')

    def test_missing_local_uses_unmeasured_landing_template(self):
        chosen, calibrated = self.resolve()
        self.assertEqual(chosen, self.template.resolve())
        self.assertFalse(calibrated)

    def test_selected_local_preserves_measured_samples_and_reference_bindings(self):
        raw = json.dumps(landing_document()).encode()
        self.local.write_bytes(raw)
        chosen, calibrated = self.resolve()
        self.assertEqual(chosen, self.local.resolve())
        self.assertTrue(calibrated)
        self.assertEqual(self.local.read_bytes(), raw)
        self.assertEqual(json.loads(raw)['actuator_reference_id'], 'SYNTHETIC_AXES')

    def test_explicit_relative_landing_file_precedes_bad_local(self):
        self.local.write_text('{bad-json')
        other = self.root/'config/measured water.json'
        other.write_text(json.dumps(landing_document()))
        self.assertEqual(self.resolve('config/measured water.json'), (other.resolve(), True))

    def test_explicit_missing_or_uncalibrated_landing_cannot_fall_back(self):
        self.local.write_text(json.dumps(landing_document()))
        with self.assertRaises(OSError):
            self.resolve('config/missing-water.json')
        with self.assertRaisesRegex(ValueError, 'not_calibrated'):
            self.resolve('config/fixed_camera_landing.template.json')

    def test_bad_or_unmeasured_existing_landing_local_never_uses_template(self):
        for content in ('{bad-json', '[]', '{}', self.template.read_text()):
            with self.subTest(content=content[:50]):
                self.local.write_text(content)
                with self.assertRaises((ValueError, TypeError, KeyError)):
                    self.resolve()
        self.local.unlink(); self.local.mkdir()
        with self.assertRaises(OSError):
            self.resolve()

    def test_calibrated_flag_cannot_bypass_measured_grid_validator(self):
        for fault in ('missing_corner', 'no_independent_validation'):
            with self.subTest(fault=fault):
                value = landing_document()
                if fault == 'missing_corner':
                    value['measurements'].pop()
                else:
                    value['independent_validation'] = []
                self.local.write_text(json.dumps(value))
                with self.assertRaises(ValueError):
                    self.resolve()


if __name__ == '__main__':
    unittest.main()
