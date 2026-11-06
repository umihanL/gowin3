"""API routing and path regression tests; no torch, training or network required."""
from __future__ import annotations
import builtins
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import train
import yaml


class TrainingEntry(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="yolo 中文路径 ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.dataset = self.root / "dataset"
        self.dataset.mkdir()
        spec = {"path": ".", "names": train.NAMES}
        for split in ("train", "val", "test"):
            image = self.dataset / f"images/{split}/sample.png"
            image.parent.mkdir(parents=True)
            image.write_bytes(b"fixture image; real image decoding belongs to full validator")
            (self.dataset / f"{split}.txt").write_text(f"./images/{split}/sample.png\n", encoding="utf-8")
            spec[split] = f"{split}.txt"
        self.data = self.dataset / "data.yaml"
        self.data.write_text(yaml.safe_dump(spec), encoding="utf-8")
        self.weights = self.root / "example_run/weights/last.pt"
        self.weights.parent.mkdir(parents=True)
        self.weights.write_bytes(b"checkpoint routing fixture")
        self.policy = self.weights.parent.parent / "augmentation_policy.json"
        self.policy.write_text(json.dumps({"policy_version": train.TRANSFORM_POLICY,
                                           "trainer": "TinyTargetDetectionTrainer",
                                           "source_sha256": train.file_hash(train.ROOT / "tiny_target_training.py")}), encoding="utf-8")
        self.args = train.arguments(["--data", str(self.data), "--project", str(self.root / "runs"),
                                     "--from-scratch", "--device", "cpu"])
        self.model = Mock(task="detect", names=train.NAMES)
        self.model.val.return_value = SimpleNamespace(
            results_dict={"metrics/mAP50(B)": .42}, names=train.NAMES,
            save_dir=self.root / "validation", box=SimpleNamespace(ap_class_index=[0, 2],
            p=[.6, .7], r=[.8, .9], ap50=[.5, .6], ap=[.3, .4]))
        self.factory = Mock(return_value=self.model)
        self.fake_trainer = object()
        trainer_patch = patch.object(train, "training_trainer", return_value=self.fake_trainer)
        trainer_patch.start()
        self.addCleanup(trainer_patch.stop)

    def prep(self):
        with patch.object(train, "full_validation") as full:
            config, splits = train.prepare(self.args)
        full.assert_called_once_with(self.dataset)
        return config, splits

    def test_relative_dataset_is_independent_of_working_directory(self):
        original = self.data.read_bytes()
        root, splits = train.resolve_dataset(self.data)
        self.assertEqual(root, self.dataset)
        resolved = train.runtime_spec(self.data, splits, self.root / "runs")
        runtime = train.read_yaml(resolved)
        self.assertTrue(Path(runtime["train"]).is_absolute())
        self.assertTrue(Path(Path(runtime["train"]).read_text(encoding="utf-8").strip()).is_absolute())
        self.assertEqual(train.resolve_dataset(resolved)[1], splits)
        self.assertEqual(original, self.data.read_bytes())

    def test_check_only_does_not_import_torch_or_write_runtime(self):
        original_import = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name.split(".")[0] in {"torch", "ultralytics"}:
                raise AssertionError(f"Unexpected import {name}")
            return original_import(name, *args, **kwargs)
        with patch.object(train, "full_validation"), patch("builtins.__import__", side_effect=guarded), contextlib.redirect_stdout(io.StringIO()):
            train.main(["--data", str(self.data), "--project", str(self.root / "runs"), "--check-only"])
        self.assertFalse((self.root / "runs").exists())

    def test_pose_and_segmentation_rejected_before_validation(self):
        for name in ("yolo11n-pose.pt", "yolo11n-seg.pt"):
            with self.subTest(name=name), patch.object(train, "full_validation") as full:
                self.args.model = name
                with self.assertRaisesRegex(ValueError, "Pose/seg"):
                    train.prepare(self.args)
                full.assert_not_called()

    def test_wrong_classes_and_shared_images_rejected(self):
        spec = train.read_yaml(self.data)
        spec["names"][3] = "building_sign"
        self.data.write_text(yaml.safe_dump(spec), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "exactly"):
            train.resolve_dataset(self.data)
        del spec["names"][3]
        self.data.write_text(yaml.safe_dump(spec), encoding="utf-8")
        (self.dataset / "test.txt").write_text("./images/train/sample.png\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "leakage"):
            train.resolve_dataset(self.data)

    def test_smoke_routes_one_epoch_without_pretrained_download(self):
        self.args.smoke = True
        config, splits = self.prep()
        resolved = train.runtime_spec(self.data, splits, self.root / "runs", smoke=True)
        train.execute(self.args, config, resolved, self.factory)
        self.factory.assert_called_once_with("yolo11n.yaml", task="detect")
        call = self.model.train.call_args.kwargs
        self.assertEqual(call["epochs"], 1)
        self.assertEqual(call["imgsz"], 320)
        self.assertEqual(call["batch"], 2)
        self.assertFalse(call["pretrained"])
        self.assertEqual(call["mosaic"], 0)
        self.assertEqual(call["fliplr"], 0)
        self.assertEqual(call["data"], str(resolved))
        self.assertIs(call["trainer"], self.fake_trainer)
        self.assertEqual(call["scale"], 0)
        self.assertEqual(call["translate"], 0)

    def test_local_coco_weights_default(self):
        self.args.from_scratch = False
        config, splits = self.prep()
        resolved = train.runtime_spec(self.data, splits, self.root / "runs")
        with patch.object(train, "ROOT", self.root):
            train.execute(self.args, config, resolved, self.factory)
        self.factory.assert_called_once_with(str(self.root / "weights/yolo11n.pt"), task="detect")

    def test_true_resume_preserves_saved_training_settings(self):
        self.args.mode, self.args.from_scratch, self.args.weights = "resume", False, self.weights
        config, splits = self.prep()
        resolved = train.runtime_spec(self.data, splits, self.root / "runs")
        self.model.ckpt = {"epoch": 17, "optimizer": {"state": {}}, "train_args": {"data": str(resolved), "epochs": 100}}
        train.execute(self.args, config, resolved, self.factory)
        self.model.train.assert_called_once_with(trainer=self.fake_trainer, resume=True, device="cpu", workers=0)

    def test_stripped_or_relocated_checkpoint_cannot_fake_resume(self):
        self.args.mode, self.args.from_scratch, self.args.weights = "resume", False, self.weights
        config, splits = self.prep()
        resolved = train.runtime_spec(self.data, splits, self.root / "runs")
        for checkpoint, error in (({"epoch": -1, "optimizer": None}, "no resumable"),
                                  ({"epoch": 3, "optimizer": {}, "train_args": {"data": "missing.yaml"}}, "YAML is missing")):
            self.model.ckpt = checkpoint
            with self.subTest(checkpoint=checkpoint), self.assertRaisesRegex(ValueError, error):
                train.execute(self.args, config, resolved, self.factory)
        self.model.train.assert_not_called()

    def test_eval_modes_select_correct_split_and_absolute_runtime_data(self):
        self.args.from_scratch, self.args.weights = False, self.weights
        for split in ("val", "test"):
            self.args.mode = split
            config, splits = self.prep()
            resolved = train.runtime_spec(self.data, splits, self.root / "runs")
            train.execute(self.args, config, resolved, self.factory)
            self.assertEqual(self.model.val.call_args.kwargs["split"], split)
            self.assertEqual(self.model.val.call_args.kwargs["data"], str(resolved))

    def test_prediction_is_consumed_and_saved(self):
        self.args.mode, self.args.from_scratch, self.args.weights = "predict", False, self.weights
        self.args.source = self.dataset / "images/test"
        config, splits = self.prep()
        resolved = train.runtime_spec(self.data, splits, self.root / "runs")
        result = train.execute(self.args, config, resolved, self.factory)
        call = self.model.predict.call_args.kwargs
        self.assertTrue(call["save"] and call["save_txt"] and call["save_conf"] and call["stream"])
        with patch.object(train, "arguments", return_value=self.args), patch.object(train, "prepare", return_value=(config, splits)), \
                patch.object(train, "execute", return_value=iter([1, 2])) as execute, \
                patch.dict(sys.modules, {"ultralytics": Mock(YOLO=self.factory), "torch": Mock()}), contextlib.redirect_stdout(io.StringIO()):
            train.main()
            self.assertEqual(list(execute.return_value), [])

    def test_loaded_model_task_and_trained_classes_are_enforced(self):
        self.args.mode, self.args.from_scratch, self.args.weights = "val", False, self.weights
        config, splits = self.prep()
        resolved = train.runtime_spec(self.data, splits, self.root / "runs")
        self.model.task = "pose"
        with self.assertRaisesRegex(ValueError, "expected detect"):
            train.execute(self.args, config, resolved, self.factory)
        self.model.task, self.model.names = "detect", {0: "person"}
        with self.assertRaisesRegex(ValueError, "class names"):
            train.execute(self.args, config, resolved, self.factory)
        self.model.val.assert_not_called()

    def test_export_uses_separate_checkpoint_and_static_reference_options(self):
        self.args.mode, self.args.from_scratch, self.args.weights = "export-reference", False, self.weights
        config, splits = self.prep()
        resolved = train.runtime_spec(self.data, splits, self.root / "runs")
        original = self.weights.read_bytes()
        with patch.object(train, "ROOT", self.root):
            train.execute(self.args, config, resolved, self.factory)
        self.assertEqual(self.weights.read_bytes(), original)
        self.assertEqual(self.model.export.call_args.kwargs, dict(format="onnx", imgsz=640, batch=1, dynamic=False,
                                                                 simplify=False, opset=11, nms=False, device="cpu"))
        self.assertTrue(any((self.root / "weights/export_reference").glob("*.pt")))

    def test_cpu_mode_and_cuda_install_mismatch_are_explicit(self):
        torch = Mock()
        torch.cuda.is_available.return_value = False
        train.validate_device(torch, "cpu")
        with self.assertRaisesRegex(RuntimeError, "CUDA unavailable"):
            train.validate_device(torch, "0")
        torch.cuda.is_available.return_value = True
        torch.cuda.device_count.return_value = 1
        train.validate_device(torch, "0")
        with self.assertRaisesRegex(ValueError, "device count"):
            train.validate_device(torch, "1")
        with self.assertRaisesRegex(ValueError, "Use --device"):
            train.validate_device(torch, "not-a-device")

    def test_resume_rejects_missing_or_different_transform_policy(self):
        self.policy.unlink()
        with self.assertRaisesRegex(ValueError, "augmentation_policy.json"):
            train.verify_resume_policy(self.weights)
        self.policy.write_text(json.dumps({"policy_version": "old-default-affine"}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "policy differs"):
            train.verify_resume_policy(self.weights)
        self.policy.write_text(json.dumps({"policy_version": train.TRANSFORM_POLICY,
                                           "trainer": "TinyTargetDetectionTrainer", "source_sha256": "modified"}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "source changed"):
            train.verify_resume_policy(self.weights)

    def test_metrics_summary_preserves_returned_values_and_missing_classes(self):
        output = train.save_metrics_summary(self.model.val.return_value, "test")
        saved = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(saved["aggregate"], {"metrics/mAP50(B)": .42})
        self.assertEqual(saved["per_class"][0]["precision"], .6)
        self.assertEqual(saved["per_class"][2]["AP50_95"], .4)
        self.assertFalse(saved["per_class"][1]["evaluated"])
        self.assertNotIn("AP50", saved["per_class"][1])


if __name__ == "__main__":
    unittest.main()
