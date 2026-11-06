"""Small mutation tests for V7 baseline freezing; no full dataset audit or GPU."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from validate_merged_dataset import check_people_extension, sha


class PeopleExtensionChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name) / "dataset"
        self.root = Path(self.temp.name) / "dataset_people_v2"
        self.base.mkdir()
        self.root.mkdir()
        self.old = []
        for i, split in enumerate(("train", "val", "test")):
            row = dict(sample_id=f"v5__old{i}", source_version="v5", source_sample_id=f"old{i}",
                       split=split, group_id=f"cell{i}", pose_group=f"pose{i}", variant="original", eye="L",
                       image=f"images/{split}/v5__old{i}.png", label=f"labels/{split}/v5__old{i}.txt")
            self.old.append(row)
            for directory in (self.base, self.root):
                for kind, data in (("image", f"fixture RGB {i}"), ("label", "1 .5 .5 .2 .3\n")):
                    path = directory / row[kind]
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(data, encoding="utf-8")
                (directory / f"{split}.txt").write_text("./" + row["image"] + "\n", encoding="utf-8")
        (self.base / "metadata.jsonl").write_text("\n".join(json.dumps(r) for r in self.old), encoding="utf-8")
        sources = {v: {"dataset": f"../source_data/{v}/packaged", "manifest_sha256": v*32} for v in ("v5", "v6")}
        base_manifest = {"sources": sources, "files": {p.relative_to(self.base).as_posix(): sha(p) for p in self.base.rglob("*") if p.is_file()}}
        (self.base / "manifest.json").write_text(json.dumps(base_manifest), encoding="utf-8")
        self.manifest = dict(sources={**sources, "v7": {"dataset": "../source_data/v7/packaged"}},
                             baseline_dataset="../dataset", baseline_manifest_sha256=sha(self.base / "manifest.json"))
        self.rows = copy.deepcopy(self.old)
        self.sources = {"v5": {f"old{i}": dict(row, sample_id=f"old{i}", pair_id=f"old_pair{i}") for i, row in enumerate(self.old)}, "v6": {}}
        self.sources["v7"] = {"new0": dict(sample_id="new0", pair_id="v7_pair0", pose_group="v7_pose0",
                                             group_id="cell0", split="train", capture_plan={})}

    def check(self):
        errors = []
        def require(ok, message):
            if not ok:
                errors.append(message)
            return bool(ok)
        report = check_people_extension(self.root, self.rows, self.manifest, self.sources, require, sha)
        return errors, report

    def test_valid_extension(self):
        errors, report = self.check()
        self.assertEqual(errors, [])
        self.assertEqual(report["preserved_old_images"], {"train": 1, "val": 1, "test": 1})

    def test_mutated_heldout_image_label_and_order(self):
        (self.root / self.old[1]["image"]).write_bytes(b"changed pixels")
        (self.root / self.old[2]["label"]).write_bytes(b"0 .5 .5 .2 .3\n")
        (self.root / "test.txt").write_bytes(b"extra\n")
        errors, _ = self.check()
        self.assertTrue(any("image bytes changed" in x for x in errors))
        self.assertTrue(any("label bytes changed" in x for x in errors))
        self.assertTrue(any("image list bytes/order" in x for x in errors))

    def test_v7_old_pose_and_heldout_group_rejected(self):
        self.sources["v7"]["new0"].update(pose_group="pose1", pair_id="old_pair1", group_id="cell1", split="val")
        errors, _ = self.check()
        for text in ("not train-only", "not an original train group", "old capture pair", "old pose group"):
            self.assertTrue(any(text in x for x in errors), text)

    def test_old_training_sample_cannot_disappear(self):
        self.rows.pop(0)
        self.assertTrue(any("Old sample missing" in x for x in self.check()[0]))

    def test_baseline_change_detected(self):
        self.manifest["baseline_manifest_sha256"] = "invalid"
        (self.base / self.old[1]["image"]).write_bytes(b"changed baseline")
        errors, _ = self.check()
        self.assertTrue(any("manifest anchor changed" in x for x in errors))
        self.assertTrue(any("Baseline image modified" in x for x in errors))

    def test_extra_heldout_label_rejected(self):
        (self.root / "labels/test/extra.txt").write_bytes(b"")
        self.assertTrue(any("file inventory" in x for x in self.check()[0]))

    def test_v5_v6_only_backwards_compatible(self):
        del self.manifest["sources"]["v7"]
        self.assertEqual(self.check(), ([], {"applicable": False}))


if __name__ == "__main__":
    unittest.main()
