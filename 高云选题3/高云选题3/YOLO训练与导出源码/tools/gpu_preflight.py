"""Load the initialization and run one real CUDA forward pass before training."""
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
(ROOT / ".ultralytics").mkdir(exist_ok=True)
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".ultralytics"))
os.environ.setdefault("YOLO_AUTOINSTALL", "false")


def main():
    import torch
    import torchvision
    import ultralytics
    from ultralytics import YOLO, settings

    assert torch.cuda.is_available(), "A real CUDA device is required"
    settings.update({"weights_dir": str(ROOT / "weights"), "runs_dir": str(ROOT / "runs"), "datasets_dir": str(ROOT / "dataset"), "sync": False})
    weights = ROOT / "weights/yolo11n.pt"
    source = json.loads((ROOT / "weights/PRETRAINED_SOURCE.json").read_text(encoding="utf-8"))
    assert hashlib.sha256(weights.read_bytes()).hexdigest() == source["sha256"]
    model = YOLO(str(weights), task="detect")
    assert model.task == "detect"
    model.to("cuda:0")
    model.model.eval()
    with torch.inference_mode():
        prediction = model.model(torch.zeros((1, 3, 640, 640), device="cuda:0"))
    tensor = prediction[0] if isinstance(prediction, (list, tuple)) else prediction
    torch.cuda.synchronize()
    assert tensor.is_cuda and torch.isfinite(tensor).all().item()
    report = dict(passed=True, torch=torch.__version__, torchvision=torchvision.__version__, ultralytics=ultralytics.__version__,
                  cuda_build=torch.version.cuda, cuda_device=torch.cuda.get_device_name(0),
                  checkpoint_loaded=True, checkpoint_task=model.task, initialization_class_count=len(model.names),
                  forward_input_shape=[1, 3, 640, 640], forward_output_shape=list(tensor.shape),
                  forward_on_cuda=tensor.is_cuda, max_allocated_memory_mb=round(torch.cuda.max_memory_allocated()/1024**2, 1),
                  weights_sha256=source["sha256"], training_performed=False)
    (ROOT / "gpu_preflight.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
