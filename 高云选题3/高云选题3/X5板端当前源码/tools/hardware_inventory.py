#!/usr/bin/env python3
"""Read-only X5 device inventory; never opens audio, cameras, UART or actuators.

Run on X5: python3 tools/hardware_inventory.py --output reports/hardware.json
USB enumeration is not an end-to-end functional test. No microphone recordings,
ROS task publications, service restarts, or configuration changes are performed.
"""
import argparse
import concurrent.futures
import datetime
import glob
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess


SERVICES = ["rescue-mipi-camera", "rescue-mipi-vision", "rescue-yolo-preview",
            "rescue-task-modes", "rescue-edge-llm", "rescue-presentation"]
VOICE_KEYS = ["microphone_enabled", "tts_enabled", "auto_scene_speech",
              "model_dir", "audio_input_device", "sample_rate_hz",
              "tts_executable", "tts_voice", "tts_audio_device",
              "continuous_listening_enabled", "wake_words"]


def run(argv, timeout=8):
    try:
        result = subprocess.run(argv, capture_output=True, text=True,
                                errors="replace", timeout=timeout)
        return {"returncode": result.returncode, "stdout": result.stdout[:32768].strip(),
                "stderr": result.stderr[:4096].strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"error": str(exc)}


def read_optional(path):
    try:
        return Path(path).read_text(errors="replace")[:32768].strip()
    except OSError as exc:
        return {"error": str(exc)}


def inventory(workspace):
    if platform.system() != "Linux":
        raise RuntimeError("Run this inventory on the RDK X5 Linux board, not on Windows.")
    commands = {
        "network": ["ip", "-br", "-4", "addr"],
        "usb_devices": ["lsusb"],
        "alsa_capture_devices": ["arecord", "-l"],
        "alsa_playback_devices": ["aplay", "-l"],
        "alsa_capture_aliases": ["arecord", "-L"],
        "alsa_playback_aliases": ["aplay", "-L"],
        "memory": ["free", "-h"],
        "disk": ["df", "-h", str(workspace)],
    }
    for service in SERVICES:
        commands[service] = ["systemctl", "show", service, "--no-pager",
                             "--property=ActiveState,SubState,MainPID,Result,FragmentPath"]
    report = {"checked_at": datetime.datetime.now().astimezone().isoformat(),
              "hostname": platform.node(), "architecture": platform.machine(),
              "workspace": str(workspace), "os_release": read_optional("/etc/os-release"),
              "read_only": True, "audio_recorded": False, "hardware_commands_sent": False}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        report["commands"] = dict(zip(commands, pool.map(run, commands.values())))
    patterns = {"video": "/dev/video*", "mipi": "/dev/vin*", "ttyS": "/dev/ttyS*",
                "ttyUSB": "/dev/ttyUSB*", "ttyACM": "/dev/ttyACM*", "audio": "/dev/snd/*",
                "bpu": "/dev/*bpu*", "pwm": "/sys/class/pwm/pwmchip*"}
    report["device_nodes"] = {name: sorted(glob.glob(pattern)) for name, pattern in patterns.items()}
    report["serial_by_id"] = [{"path": p, "target": os.path.realpath(p)}
                              for p in sorted(glob.glob("/dev/serial/by-id/*"))]
    report["alsa_cards"] = read_optional("/proc/asound/cards")
    report["alsa_pcm"] = read_optional("/proc/asound/pcm")
    report["voice_configs"] = {}
    for path in sorted((workspace / "config").glob("voice*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8-sig"))
            report["voice_configs"][path.name] = {k: data[k] for k in VOICE_KEYS if k in data}
        except (OSError, ValueError) as exc:
            report["voice_configs"][path.name] = {"error": str(exc)}
    report["tools"] = {name: shutil.which(name) for name in
                       ("arecord", "aplay", "espeak-ng", "ros2", "colcon", "hb_perf")}
    report["modules_current_environment"] = {}
    for name in ("rclpy", "cv2", "numpy", "serial", "vosk", "sounddevice", "hobot_dnn", "hbm_runtime"):
        try:
            report["modules_current_environment"][name] = importlib.util.find_spec(name) is not None
        except (ImportError, ValueError):
            report["modules_current_environment"][name] = False
    report["limitations"] = [
        "Device enumeration does not prove correct wiring or useful camera/audio data.",
        "Config files describe saved settings; the running node may use another override.",
        "FPGA navigation, UART heartbeat, ROS frame rate, stereo calibration and audio quality require separate live checks.",
        "No microphone samples, speech recognition or speaker playback were tested."]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    payload = json.dumps(inventory(args.workspace.resolve()), ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)


if __name__ == "__main__":
    main()
