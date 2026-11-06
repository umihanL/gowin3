"""Compile a frozen Gowin project locally without an extra timing review or flashing."""
from __future__ import annotations

import argparse
import datetime
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

from build_project import digest, publish, read, source_hashes, validate_project, write_json


COMPILE_TCL = """cd [file dirname [file normalize [info script]]]
set started [clock seconds]
if {[catch {
 open_project ov5640.gprj
 set_option -top_module ov5640_ddr3_tft
 set_option -include_path {src/vga;src/navigation}
 set_option -output_base_name ov5640
 set_option -vcc 0.95
 set_option -use_cpu_as_gpio 1
 set_option -use_sspi_as_gpio 1
 set_option -gen_text_timing_rpt 1
 set_option -place_option 2
 set_option -route_option 0
 set_option -inc_pnr 0
 set_option -inc_place 0
 run all
} message]} {puts stderr $message;exit 1}
if {![file exists impl/pnr/ov5640.fs] || [file mtime impl/pnr/ov5640.fs] < $started} {
 puts stderr "No fresh bitstream generated";exit 1
}
exit 0
"""


def compile_project(root: Path, tool: Path, work_root: Path, label: str):
    root, tool, work_root = root.resolve(), tool.resolve(), work_root.resolve()
    if not str(tool).isascii() or not str(work_root).isascii():
        raise RuntimeError("Gowin executable and build workspace must use ASCII-only paths")
    if not tool.is_file():
        raise RuntimeError("Gowin executable not found: " + str(tool))
    if not re.fullmatch(r"[a-z0-9_]+", label):
        raise RuntimeError("Build label must contain lowercase ASCII letters, digits or underscores")
    validate_project(root)
    hashes = source_hashes(root)
    work_root.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix="ov5640_" + label + "_", dir=work_root))
    shutil.copytree(root / "src", stage / "src")
    for name in ("ov5640.gprj", "build.tcl"):
        shutil.copy2(root / name, stage / name)
    if source_hashes(stage) != hashes or source_hashes(root) != hashes:
        raise RuntimeError("Source changed while freezing the project; no build was started")
    script = stage / "compile_only.tcl"
    script.write_text(COMPILE_TCL, encoding="utf-8", newline="\n")
    started = time.time()
    write_json(stage / "build_inputs.json", dict(
        hardware_inputs=hashes, tool=str(tool), original_project=str(root),
        build_directory=str(stage), incremental=False, compile_script_sha256=digest(script),
        started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
    environment = dict(os.environ)
    environment.pop("GOWIN_INCREMENTAL_PR", None)
    print("Build workspace: " + str(stage), flush=True)
    print("Live log: " + str(stage / "build.log"), flush=True)
    with (stage / "build.log").open("w", encoding="utf-8") as log:
        result = subprocess.run([str(tool), script.name], cwd=stage, env=environment,
                                stdout=log, stderr=subprocess.STDOUT)
    unchanged = source_hashes(root) == hashes and source_hashes(stage) == hashes
    metrics = dict(exit_code=result.returncode, seconds=round(time.time() - started, 3),
                   source_unchanged=unchanged, hardware_tested=False, flashed=False,
                   additional_timing_analysis=False)
    write_json(stage / "build_metrics.json", metrics)
    if result.returncode:
        raise RuntimeError(f"Gowin exited {result.returncode}; previous outputs preserved. See {stage / 'build.log'}")
    if not unchanged:
        raise RuntimeError("Sources changed during compilation; new outputs were not published")
    for name in ("pnr/ov5640.fs", "pnr/ov5640.rpt.txt", "gwsynthesis/ov5640.vg"):
        path = stage / "impl" / name
        if not path.is_file() or not path.stat().st_size or path.stat().st_mtime < started:
            raise RuntimeError("Missing, empty or stale output: " + name)
    log = read(stage / "build.log")
    if ("Bitstream generation completed" not in log or "Placement and routing completed" not in log
            or re.search(r"^\s*ERROR\b", log, re.M)):
        raise RuntimeError("Gowin log does not confirm successful compilation")
    report = read(stage / "impl/pnr/ov5640.rpt.txt")
    pins = dict(re.findall(r'^\s*IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', read(root / "src/ov5640_ddr3_tft.cst"), re.M))
    actual = dict(re.findall(r"^\s*([^\s|]+)\s*\|[^|]*\|\s*([A-Z]+\d+)(?:/[^|\s]+)?\s*\|", report, re.M))
    if not pins or len(pins) != len(set(pins.values())) or any(actual.get(n) != p for n, p in pins.items()):
        raise RuntimeError("Routed pin assignments do not match the project CST")
    fs = stage / "impl/pnr/ov5640.fs"
    audit = dict(release="local_" + stage.name, status="compiled_local_only",
        bitstream="impl/pnr/ov5640.fs", sha256=digest(fs), bytes=fs.stat().st_size,
        generated_utc=datetime.datetime.fromtimestamp(fs.stat().st_mtime, datetime.timezone.utc).isoformat(),
        tool=str(tool), device=ET.parse(root / "ov5640.gprj").find(".//Device").get("pn"),
        build_workspace=str(stage), hardware_inputs=hashes, build_metrics=metrics,
        pins_verified=pins, pin_count=len(pins), compile_script_sha256=digest(script),
        additional_timing_analysis=False, timing_scope="Tool-generated reports retained without additional timing review.",
        hardware_tested=False, flashed=False)
    backup = publish(root, stage, audit)
    print(json.dumps(dict(bitstream=str(root / audit["bitstream"]), sha256=audit["sha256"],
        backup=str(backup), audit=str(root / "impl/pnr/ov5640_current_audit.json"),
        flashed=False, additional_timing_analysis=False)), flush=True)
    return audit


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--gowin", type=Path, required=True)
    parser.add_argument("--work-root", type=Path, default=Path("D:/FPGA/codex_builds"))
    parser.add_argument("--label", default="compile")
    args = parser.parse_args()
    compile_project(args.project, args.gowin, args.work_root, args.label)
