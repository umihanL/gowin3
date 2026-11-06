"""Supervisor routing/status tests; no model, GPU, validation or training."""
from __future__ import annotations
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "tools/run_training_job.py"
spec = importlib.util.spec_from_file_location("job", SCRIPT)
job = importlib.util.module_from_spec(spec)
spec.loader.exec_module(job)


class TrainingJob(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="job test ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.patch = patch.object(job, "ROOT", self.root)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.folder = self.root / "runs/_jobs/new_people"
        self.folder.mkdir(parents=True)
        self.config = self.root / "config/yolo11n_detect.yaml"
        self.config.parent.mkdir()
        self.config.write_text("task: detect\n", encoding="utf-8")
        self.state = dict(name="new_people", phase="preparing", started_at="test", epochs=100, batch=8,
                          dataset=str(self.root / "dataset_people_v2/data.yaml"), initialization_model="old/best.pt",
                          training_kind="finetune", training_images=3000, validation_images=252, test_images=252)

    def test_finetune_commands_use_new_data_for_train_and_test_without_resume(self):
        args = SimpleNamespace(name="new_people", epochs=80, batch=8, patience=20, config=self.config)
        data = self.root / "dataset_people_v2/data.yaml"
        train, test, best = job.commands(args, data, "old/best.pt")
        self.assertEqual(train[train.index("--mode") + 1], "train")
        self.assertEqual(train[train.index("--model") + 1], "old/best.pt")
        self.assertNotIn("resume", train)
        self.assertEqual(train[train.index("--epochs") + 1], "80")
        self.assertEqual(train[train.index("--patience") + 1], "20")
        for cmd in (train, test):
            self.assertEqual(cmd[cmd.index("--data") + 1], str(data))
            self.assertEqual(cmd[cmd.index("--config") + 1], str(self.config))
        self.assertEqual(test[test.index("--mode") + 1], "test")
        self.assertEqual(test[test.index("--weights") + 1], str(best))

    def test_dynamic_counts_and_links_appear_only_when_files_exist(self):
        job.save_status(self.folder, self.state)
        html = (self.folder / "progress.html").read_text(encoding="utf-8")
        self.assertIn("3000", html)
        self.assertIn("dataset_people_v2", html)
        self.assertIn("新增数据微调", html)
        self.assertNotIn('href="train.log"', html)
        self.assertNotIn('href="test.log"', html)
        self.assertNotIn('href="../../new_people/results.csv"', html)
        (self.folder / "train.log").write_text("started", encoding="utf-8")
        results = self.root / "runs/new_people/results.csv"
        results.parent.mkdir()
        results.write_text("epoch\n1\n", encoding="utf-8")
        job.save_status(self.folder, self.state)
        html = (self.folder / "progress.html").read_text(encoding="utf-8")
        self.assertIn('href="train.log"', html)
        self.assertIn('href="../../new_people/results.csv"', html)

    def test_stopped_state_is_terminal_and_backward_compatible(self):
        self.state["phase"] = "stopped"
        self.state.pop("training_kind")
        job.save_status(self.folder, self.state)
        html = (self.folder / "progress.html").read_text(encoding="utf-8")
        self.assertIn("训练已按要求停止", html)
        self.assertNotIn('http-equiv="refresh"', html)
        self.assertEqual(json.loads((self.folder / "status.json").read_text(encoding="utf-8"))["phase"], "stopped")

    def test_first_batch_marks_training_before_results_csv_exists(self):
        log = self.folder / "train.log"
        log.write_text("Epoch GPU_mem\n\r       1/100    1.14G    0.3  12 640: 0%\r", encoding="utf-8")
        job.update_training_progress(log, self.state)
        self.assertEqual(self.state["phase"], "training")
        self.assertEqual(self.state["active_epoch"], 1)
        self.assertIn("training_started_at", self.state)
        self.assertNotIn("latest_training_metrics", self.state)

    def test_stopped_state_cannot_be_reset_by_old_log(self):
        log = self.folder / "train.log"
        log.write_text("Starting training for 100 epochs", encoding="utf-8")
        self.state["phase"] = "stopped"
        job.update_training_progress(log, self.state)
        self.assertEqual(self.state["phase"], "stopped")

    def test_main_stopped_training_does_not_evaluate_test(self):
        self.folder.rmdir()
        def stop(command, folder, state, stage):
            state["phase"] = "stopped"
            return 130
        with patch.object(job, "inspect_inputs", return_value=(self.root / "dataset_people_v2/data.yaml", "old/best.pt", {"train": 3000, "val": 252, "test": 252})), \
                patch.object(job, "run_phase", side_effect=stop) as phase:
            job.main(["--name", "new_people", "--data", "dataset_people_v2/data.yaml", "--model", "old/best.pt"])
        self.assertEqual(phase.call_count, 1)
        state = json.loads((self.folder / "status.json").read_text(encoding="utf-8"))
        self.assertEqual(state["phase"], "stopped")
        self.assertEqual(state["training_images"], 3000)
        self.assertEqual(state["epochs"], 100)
        self.assertEqual(state["patience"], 25)

    def test_completed_training_evaluates_test_once_with_selected_config(self):
        self.folder.rmdir()
        def complete(command, folder, state, stage):
            if stage == "train":
                best = self.root / "runs/new_people/weights/best.pt"
                best.parent.mkdir(parents=True)
                best.write_bytes(b"test-double")
            return 0
        with patch.object(job, "inspect_inputs", return_value=(self.root / "dataset_people_v2/data.yaml", "old/best.pt", {"train": 4000, "val": 252, "test": 252})), \
                patch.object(job, "run_phase", side_effect=complete) as phase:
            job.main(["--name", "new_people", "--model", "old/best.pt", "--epochs", "80", "--patience", "20", "--config", str(self.config)])
        self.assertEqual([call.args[3] for call in phase.call_args_list], ["train", "test"])
        state = json.loads((self.folder / "status.json").read_text(encoding="utf-8"))
        self.assertEqual(state["phase"], "completed")
        self.assertEqual(state["patience"], 20)
        self.assertEqual(state["config"], str(self.config))

    def test_finetune_config_preserves_tiny_target_policy(self):
        import yaml
        config = yaml.safe_load((SCRIPT.parents[1] / "config/yolo11n_people_finetune.yaml").read_text(encoding="utf-8"))
        self.assertEqual(config["task"], "detect")
        self.assertEqual(config["optimizer"], "AdamW")
        self.assertEqual(config["lr0"], 0.0003)
        self.assertEqual(config["warmup_epochs"], 3.0)
        self.assertEqual(config["warmup_bias_lr"], 0.0)
        for key in ("degrees", "translate", "scale", "shear", "perspective", "flipud", "fliplr", "mosaic", "mixup", "copy_paste", "cutmix", "close_mosaic"):
            self.assertEqual(config[key], 0, key)
        self.assertFalse(config["multi_scale"])
        self.assertFalse(set(config) & {"data", "model", "resume"})


if __name__ == "__main__":
    unittest.main()
