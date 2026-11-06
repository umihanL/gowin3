"""Temporary fixtures test intake invariants; they are never real photo samples."""
import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

from PIL import Image

SPEC = importlib.util.spec_from_file_location("real_capture_dataset", Path(__file__).resolve().parents[1] / "tools/real_capture_dataset.py")
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class IntakeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.original_project, self.original_dest = module.PROJECT, module.DEST
        module.PROJECT = self.base / "project"
        module.PROJECT.mkdir()
        module.DEST = module.PROJECT / "real_capture_validation"
        module.DEST.mkdir()
        self.photo = self.base / "fixture.png"
        Image.new("RGB", (40, 20), "gray").save(self.photo)
        self.manifest = self.base / "input.csv"
        self.row = dict(image_path=str(self.photo), session_id="fixture_session", pose_id="p01",
                        capture_utc="2026-10-08T12:00:00+00:00", camera_model="UNIT_TEST_FIXTURE",
                        layout="stereo_sbs", distance_m="0.4", angle_deg="20", lighting="normal",
                        occlusion="none", source_origin="real_camera", notes="Synthetic unit fixture ONLY")

    def tearDown(self):
        module.PROJECT, module.DEST = self.original_project, self.original_dest
        self.temp.cleanup()

    def write_manifest(self, rows=None):
        with self.manifest.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=module.FIELDS)
            writer.writeheader()
            writer.writerows(rows or [self.row])

    def test_pending_read_only(self):
        self.assertEqual(module.check_registered()["images"], 0)
        self.assertEqual(list(module.DEST.iterdir()), [])

    def test_register_preserves_bytes_and_isolates(self):
        self.write_manifest()
        rows = module.inspect_manifest(self.manifest)
        self.assertEqual(list(module.DEST.iterdir()), [])
        result = module.register(rows)
        self.assertEqual(result["images"], 1)
        self.assertFalse(result["use_for_training"])
        self.assertEqual(next((module.DEST / "raw").rglob("*.png")).read_bytes(), self.photo.read_bytes())
        with self.assertRaises(ValueError):
            module.register(module.inspect_manifest(self.manifest))

    def test_rejects_synthetic_source_directory(self):
        synthetic = module.PROJECT / "source_data"
        synthetic.mkdir()
        source = synthetic / "synthetic.png"
        source.write_bytes(self.photo.read_bytes())
        self.row["image_path"] = str(source)
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "prohibited"):
            module.inspect_manifest(self.manifest)

    def test_rejects_false_origin_and_duplicate(self):
        self.row["source_origin"] = "blender"
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "source_origin"):
            module.inspect_manifest(self.manifest)
        self.row["source_origin"] = "real_camera"
        self.write_manifest([self.row, self.row])
        with self.assertRaisesRegex(ValueError, "duplicate"):
            module.inspect_manifest(self.manifest)

    def test_tamper_is_detected(self):
        self.write_manifest()
        module.register(module.inspect_manifest(self.manifest))
        path = next((module.DEST / "raw").rglob("*.png"))
        path.write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "altered"):
            module.check_registered()


if __name__ == "__main__":
    unittest.main()
