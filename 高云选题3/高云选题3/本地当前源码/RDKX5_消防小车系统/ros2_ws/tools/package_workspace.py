"""Package the current ROS workspace without changing deployment/test status.

Default: source, configuration, tools, tests, final docs and source licenses.
Use --include-models to include installed model assets. This is a portable file
archive, not evidence of hardware acceptance or a replacement for board setup.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
import zipfile


DEFAULT_WORKSPACE = Path(__file__).resolve().parents[1]
ROOT_DIRS = ("src", "config", "tools", "tests", "docs", "web", "vendor", ".vscode")
ROOT_FILES = ("env.sh", ".gitignore")
EXCLUDED_DIRS = frozenset({
    ".git", ".venv", "venv", "env", "build", "install", "log", "logs",
    "archive", "archives", "backup", "backups", "__pycache__", ".pytest_cache",
    ".mypy_cache", "node_modules", "dist", "wheels", "ocr_python", "ocr_wheels",
    "secrets", "secret", "credentials", "tokens", ".ssh", ".aws",
})
EXCLUDED_SUFFIXES = frozenset({
    ".zip", ".gz", ".tgz", ".tar", ".7z", ".rar", ".whl", ".deb",
    ".pyc", ".pyo", ".bak", ".old", ".tmp", ".log",
    ".pem", ".key", ".p12", ".pfx", ".token", ".secret",
})
PRIVATE_NAME = re.compile(
    r"(^|[._-])(secrets?|tokens?|credentials?|passwords?|id_rsa|id_ed25519)([._-]|$)",
    re.IGNORECASE,
)
SECRET_KEYS = frozenset({
    "token", "api_key", "apikey", "access_token", "refresh_token",
    "authorization", "password", "passwd", "client_secret", "private_key",
})


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def contains_secret_value(value) -> bool:
    if isinstance(value, dict):
        for key, nested in value.items():
            if str(key).lower().replace("-", "_") in SECRET_KEYS and nested not in (None, ""):
                return True
            if contains_secret_value(nested):
                return True
    elif isinstance(value, list):
        return any(contains_secret_value(item) for item in value)
    return False


def exclusion_reason(path: Path, workspace: Path, output_paths: set[Path]) -> str | None:
    relative = path.relative_to(workspace)
    if path.is_symlink() or any(parent.is_symlink() for parent in path.parents if parent != workspace):
        return "symbolic_link"
    if path.resolve() in output_paths:
        return "package_output"
    if any(part.lower() in EXCLUDED_DIRS or part.endswith(".egg-info") for part in relative.parts[:-1]):
        return "build_runtime_cache_or_archive_directory"
    if path.suffix.lower() in EXCLUDED_SUFFIXES:
        return "archive_cache_or_private_extension"
    if path.name.lower().startswith(".env") or PRIVATE_NAME.search(path.name):
        return "private_or_environment_filename"
    # Token-file paths are safe; populated credentials are not. Never print the
    # actual value when recording why a configuration file was excluded.
    if path.suffix.lower() == ".json":
        try:
            if contains_secret_value(json.loads(path.read_text(encoding="utf-8-sig"))):
                return "populated_credential_field"
        except (ValueError, UnicodeError):
            return "unreadable_json"
    return None


def package_workspace(workspace: Path, output: Path, include_models: bool = False) -> dict:
    workspace = workspace.resolve()
    output = output.resolve()
    if not (workspace / "src").is_dir():
        raise ValueError("Workspace must contain src/")
    if output.suffix.lower() != ".zip":
        raise ValueError("--output must be a .zip path")
    output.parent.mkdir(parents=True, exist_ok=True)
    sidecar = output.with_suffix(".manifest.json")
    output_paths = {output, sidecar}
    roots = ROOT_DIRS + (("models",) if include_models else ())
    candidates = set()
    # Prune private/build directories before traversing them.
    for name in roots:
        root = workspace / name
        if not root.is_dir() or root.is_symlink():
            continue
        for folder, directories, files in os.walk(root, followlinks=False):
            directories[:] = sorted(d for d in directories
                if d.lower() not in EXCLUDED_DIRS and not d.endswith(".egg-info")
                and not (Path(folder) / d).is_symlink())
            candidates.update(Path(folder) / name for name in files)
    candidates.update(workspace / name for name in ROOT_FILES if (workspace / name).is_file())
    included = []
    omitted = []
    for path in sorted(candidates):
        reason = exclusion_reason(path, workspace, output_paths)
        relative = path.relative_to(workspace).as_posix()
        if reason:
            omitted.append({"path": relative, "reason": reason})
        else:
            included.append({"path": relative, "bytes": path.stat().st_size, "sha256": sha256_file(path)})
    manifest = {
        "schema_version": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "artifact_type": "current_workspace_files",
        "archive_root": "ros2_ws",
        "includes_models": include_models,
        "architecture": "FPGA owns navigation, localization and vehicle control; RDK X5 owns perception, task coordination and interaction.",
        "acceptance_claim": "None. Packaging and checksums do not establish model accuracy or physical robot acceptance.",
        "installation_note": "Python environments, colcon build/install outputs, logs, caches and archives are excluded. Board dependencies and native libraries must be built for the target architecture.",
        "files": included,
        "omitted_files": omitted,
        "excluded_directories": sorted(EXCLUDED_DIRS),
    }
    manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    with tempfile.NamedTemporaryFile(prefix="rescue_package_", suffix=".zip", dir=output.parent, delete=False) as stream:
        temporary = Path(stream.name)
    try:
        with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for item in included:
                archive.write(workspace / item["path"], "ros2_ws/" + item["path"])
            archive.writestr("MANIFEST.json", manifest_bytes)
        with zipfile.ZipFile(temporary) as archive:
            if archive.testzip() is not None:
                raise RuntimeError("Archive CRC verification failed")
            for item in included:
                digest = hashlib.sha256(archive.read("ros2_ws/" + item["path"])).hexdigest()
                if digest != item["sha256"]:
                    raise RuntimeError("File changed while packaging: " + item["path"])
        os.replace(temporary, output)
        sidecar.write_bytes(manifest_bytes)
    finally:
        temporary.unlink(missing_ok=True)
    return {"zip": str(output), "manifest": str(sidecar), "files": len(included),
            "includes_models": include_models, "sha256": sha256_file(output),
            "bytes": output.stat().st_size, "archive_verified": True,
            "hardware_acceptance_performed": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    parser.add_argument("--output", type=Path,
                        help="ZIP destination; default is rescue_current_code.zip beside the workspace")
    parser.add_argument("--include-models", action="store_true",
                        help="Include model weights and their licenses; never include environments or caches")
    args = parser.parse_args()
    output = args.output or args.workspace.resolve().parent / "rescue_current_code.zip"
    print(json.dumps(package_workspace(args.workspace, output, args.include_models), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
