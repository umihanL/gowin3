"""Inspect this interpreter and NVIDIA driver; install nothing and load no model."""
from __future__ import annotations
import argparse
import importlib.util
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def inspect():
    report = {"python_executable": sys.executable, "python_version": platform.python_version(),
              "platform": platform.platform(), "virtual_environment": sys.prefix != sys.base_prefix,
              "packages": {}, "nvidia_smi": None, "trained_weights_present": False}
    for name in ("yaml", "PIL", "numpy", "torch", "ultralytics", "onnx"):
        spec = importlib.util.find_spec(name)
        info = {"installed": spec is not None}
        if spec is not None:
            from importlib.metadata import version, PackageNotFoundError
            try:
                info["version"] = version({"yaml": "PyYAML", "PIL": "Pillow"}.get(name, name))
            except PackageNotFoundError:
                info["version"] = "unknown"
        report["packages"][name] = info
    if report["packages"]["torch"]["installed"]:
        try:
            import torch
            report["torch_runtime"] = {"cuda_available": torch.cuda.is_available(), "cuda_build": torch.version.cuda,
                                       "devices": [torch.cuda.get_device_name(i) for i in range(torch.cuda.device_count())]}
        except Exception as exc:
            report["torch_runtime"] = {"import_error": str(exc)}
    command = shutil.which("nvidia-smi")
    if command:
        completed = subprocess.run([command, "--query-gpu=name,driver_version,memory.total,memory.free", "--format=csv,noheader"],
                                   capture_output=True, text=True, timeout=15)
        report["nvidia_smi"] = {"returncode": completed.returncode, "output": completed.stdout.strip(), "error": completed.stderr.strip()}
    root = Path(__file__).resolve().parents[1]
    report["trained_weights_present"] = any((root / "runs").glob("*/weights/best.pt"))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--require-python", action="store_true")
    parser.add_argument("--require-torch", action="store_true")
    parser.add_argument("--require-training", action="store_true")
    parser.add_argument("--require-cuda", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    report = inspect()
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if not args.quiet:
        print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    errors = []
    if args.require_python and not (3, 10) <= sys.version_info[:2] <= (3, 12):
        errors.append("Use Python 3.10-3.12; pass -PythonExe to the setup script if needed.")
    if args.require_torch or args.require_training or args.require_cuda:
        if not report["packages"]["torch"]["installed"] or report.get("torch_runtime", {}).get("import_error"):
            errors.append("A working PyTorch is missing. Run 01_setup_environment.cmd with CPU or CUDA (Check only installs data-check dependencies).")
    if args.require_training and not report["packages"]["ultralytics"]["installed"]:
        errors.append("Ultralytics is missing. Run 01_setup_environment.cmd with CPU, CUDA or Existing.")
    if args.require_cuda and not report.get("torch_runtime", {}).get("cuda_available"):
        errors.append("CUDA is unavailable. Check the NVIDIA driver and the selected PyTorch CUDA wheel.")
    for error in errors:
        print("ERROR: " + error, file=sys.stderr)
    if errors:
        sys.exit(2)
