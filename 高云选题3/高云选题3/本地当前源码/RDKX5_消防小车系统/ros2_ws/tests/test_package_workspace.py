"""Distribution must not expose secrets, embed old packages or rewrite status."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

SPEC = importlib.util.spec_from_file_location(
    "package_workspace", Path(__file__).resolve().parents[1] / "tools/package_workspace.py")
PACKAGER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PACKAGER)


class CurrentPackageTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.ws = self.root / "ros2_ws"
        self.write("src/rescue_vision/node.py", "print('current')\n")
        self.write("docs/完整学习与验收手册.md", "# Current manual\n")
        self.output = self.root / "release.zip"

    def write(self, relative, value):
        path = self.ws / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value, encoding="utf-8")
        return path

    def contents(self, include_models=False):
        result = PACKAGER.package_workspace(self.ws, self.output, include_models)
        with zipfile.ZipFile(self.output) as archive:
            content = {name: archive.read(name) for name in archive.namelist()}
        self.assertTrue(result["archive_verified"])
        return content

    def test_defaults_exclude_runtime_build_models_and_old_archives(self):
        for folder in ("models", "logs", "install", "build", ".venv", "archive"):
            self.write(folder + "/must_not_ship.txt", "private runtime")
        for relative in ("tools/old.zip", "config/old.tar.gz", "src/__pycache__/node.pyc",
                         "vendor/ocr_python/package.py", "tools/backup/old.py"):
            self.write(relative, "old or generated")
        content = self.contents()
        self.assertEqual(set(content), {"MANIFEST.json", "ros2_ws/src/rescue_vision/node.py",
                                       "ros2_ws/docs/完整学习与验收手册.md"})

    def test_secrets_removed_but_token_file_reference_is_preserved(self):
        for relative in ("config/dashboard.token", "config/.env.production", "config/private.pem",
                         "config/credentials.json", "config/client-secret.txt",
                         "config/secrets/session.txt", "vendor/.ssh/unknown_identity"):
            self.write(relative, "secret-value")
        self.write("config/device.json", json.dumps({"nested": {"access_token": "secret-value"}}))
        safe = {"token_file": "~/.local/state/rescue/dashboard.token", "max_tokens": 128}
        self.write("config/telemetry.json", json.dumps(safe))
        content = self.contents()
        self.assertIn("ros2_ws/config/telemetry.json", content)
        self.assertFalse(any(b"secret-value" in value for value in content.values()))
        self.assertEqual([p for p in content if p.startswith("ros2_ws/config/")],
                         ["ros2_ws/config/telemetry.json"])

    def test_include_models_keeps_weights_licenses_but_not_download_zip(self):
        self.write("models/llm/model.gguf", "weights")
        self.write("models/llm/LICENSE", "license")
        self.write("models/llm/UPSTREAM_MODEL_CARD.md", "model source")
        self.write("models/voice/download.zip", "cache")
        content = self.contents(True)
        self.assertIn("ros2_ws/models/llm/model.gguf", content)
        self.assertIn("ros2_ws/models/llm/LICENSE", content)
        self.assertIn("ros2_ws/models/llm/UPSTREAM_MODEL_CARD.md", content)
        self.assertNotIn("ros2_ws/models/voice/download.zip", content)

    def test_does_not_depend_on_or_rewrite_old_status_and_manifests(self):
        status = self.write("deployment_status.json", '{"status":"retain exactly"}\n')
        before = status.read_bytes()
        content = self.contents()
        self.assertEqual(status.read_bytes(), before)
        self.assertFalse((self.ws / "delivery_manifest.json").exists())
        self.assertFalse((self.ws.parent / "delivery_manifest.json").exists())
        self.assertNotIn("ros2_ws/deployment_status.json", content)

    def test_manifest_hashes_match_archive_and_make_no_acceptance_claim(self):
        content = self.contents()
        manifest = json.loads(content["MANIFEST.json"])
        self.assertIn("FPGA owns navigation", manifest["architecture"])
        self.assertTrue(manifest["acceptance_claim"].startswith("None."))
        for item in manifest["files"]:
            data = content["ros2_ws/" + item["path"]]
            self.assertEqual(hashlib.sha256(data).hexdigest(), item["sha256"])
            self.assertEqual(len(data), item["bytes"])
        self.assertEqual(self.output.with_suffix(".manifest.json").read_bytes(), content["MANIFEST.json"])

    def test_rerun_with_output_inside_workspace_cannot_embed_previous_release(self):
        self.output = self.ws / "tools/release.zip"
        self.contents()
        content = self.contents()
        self.assertNotIn("ros2_ws/tools/release.zip", content)
        self.assertNotIn("ros2_ws/tools/release.manifest.json", content)


if __name__ == "__main__":
    unittest.main()
