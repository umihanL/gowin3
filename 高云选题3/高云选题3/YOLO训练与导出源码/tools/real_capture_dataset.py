"""Explicit, provenance-preserving real photo intake. No training or camera access."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import shutil
from datetime import datetime, timezone
from pathlib import Path

from PIL import Image

PROJECT = Path(__file__).resolve().parents[1]
DEST = PROJECT / "real_capture_validation"
FIELDS = ("image_path", "session_id", "pose_id", "capture_utc", "camera_model", "layout",
          "distance_m", "angle_deg", "lighting", "occlusion", "source_origin", "notes")
ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}\Z")


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inside(path, parent):
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def inspect_manifest(manifest):
    rows = []
    seen = set()
    with manifest.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not set(FIELDS).issubset(reader.fieldnames):
            raise ValueError("CSV must contain all columns in manifest.template.csv")
        for line, row in enumerate(reader, 2):
            row = {key: (row.get(key) or "").strip() for key in FIELDS}
            path = Path(row["image_path"]).expanduser()
            path = (manifest.parent / path).resolve() if not path.is_absolute() else path.resolve()
            # Never reinterpret our synthetic datasets or prediction outputs as real captures.
            for root in PROJECT.iterdir():
                if root.is_dir() and (root.name.startswith("dataset") or root.name in
                                     {"source_data", "runs", "reports", "real_capture_validation"}):
                    if inside(path, root.resolve()):
                        raise ValueError(f"Line {line}: prohibited source directory: {root}")
            if row["source_origin"] != "real_camera":
                raise ValueError(f"Line {line}: source_origin must explicitly be real_camera")
            if not ID.fullmatch(row["session_id"]) or not ID.fullmatch(row["pose_id"]):
                raise ValueError(f"Line {line}: session/pose IDs require letters, digits, _ or -")
            if row["layout"] not in {"mono", "stereo_sbs"}:
                raise ValueError(f"Line {line}: layout must be mono or stereo_sbs")
            distance, angle = float(row["distance_m"]), float(row["angle_deg"])
            if not 0 < distance <= 100 or not 0 <= angle <= 90:
                raise ValueError(f"Line {line}: invalid measured distance/angle")
            captured = datetime.fromisoformat(row["capture_utc"].replace("Z", "+00:00"))
            if captured.utcoffset() is None:
                raise ValueError(f"Line {line}: capture_utc needs a timezone")
            if not all(row[key] for key in ("camera_model", "lighting", "occlusion")):
                raise ValueError(f"Line {line}: camera/light/occlusion must be recorded")
            with Image.open(path) as im:
                width, height = im.size
                im.verify()
            if row["layout"] == "stereo_sbs" and width % 2:
                raise ValueError(f"Line {line}: SBS frame width must be even")
            digest = sha256(path)
            if digest in seen:
                raise ValueError(f"Line {line}: duplicate original photo")
            seen.add(digest)
            row.update(image_path=str(path), sha256=digest, width=width, height=height,
                       distance_m=distance, angle_deg=angle, use_for_training=False,
                       intended_split="independent_validation")
            rows.append(row)
    if not rows:
        raise ValueError("No photos listed; template is intentionally empty")
    return rows


def check_registered():
    manifest = DEST / "manifest.jsonl"
    if not manifest.exists():
        return {"status": "pending_real_capture", "images": 0,
                "message": "No real captures registered; no real-world accuracy measured"}
    rows = [json.loads(line) for line in manifest.read_text(encoding="utf-8").splitlines() if line]
    seen = set()
    for row in rows:
        path = (DEST / row["stored_path"]).resolve()
        if not inside(path, (DEST / "raw").resolve()):
            raise ValueError("Registered file leaves raw directory")
        if row["source_origin"] != "real_camera" or row["use_for_training"] is not False:
            raise ValueError("Invalid real validation provenance/split")
        if row["sha256"] in seen or sha256(path) != row["sha256"]:
            raise ValueError(f"Duplicate or altered original: {path}")
        seen.add(row["sha256"])
        with Image.open(path) as im:
            if list(im.size) != [row["width"], row["height"]]:
                raise ValueError(f"Image size changed: {path}")
            im.verify()
    return {"status": "registered_unannotated", "images": len(rows),
            "sessions": len({r["session_id"] for r in rows}),
            "poses": len({(r["session_id"], r["pose_id"]) for r in rows}),
            "original_hashes": "PASS", "use_for_training": False,
            "note": "Human-declared real origin; content authenticity and annotations not verified"}


def register(rows):
    manifest = DEST / "manifest.jsonl"
    if manifest.exists():
        check_registered()
    existing = [json.loads(line) for line in manifest.read_text(encoding="utf-8").splitlines()
                if line] if manifest.exists() else []
    known = {row["sha256"] for row in existing}
    if any(row["sha256"] in known for row in rows):
        raise ValueError("At least one photo is already registered; no duplicate imports allowed")
    for row in rows:
        source = Path(row["image_path"])
        relative = Path("raw") / row["session_id"] / row["pose_id"] / (row["sha256"] + source.suffix.lower())
        target = DEST / relative
        if target.exists():
            raise ValueError(f"Destination already exists: {target}")
    copied = []
    try:
        for row in rows:
            source = Path(row["image_path"])
            relative = Path("raw") / row["session_id"] / row["pose_id"] / (row["sha256"] + source.suffix.lower())
            target = DEST / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive creation; original metadata is recorded rather than modified.
            with source.open("rb") as src, target.open("xb") as dst:
                copied.append(target)
                shutil.copyfileobj(src, dst)
            if sha256(target) != row["sha256"]:
                raise ValueError(f"Source changed during copy: {source}")
            row["stored_path"] = relative.as_posix()
            row["registered_utc"] = datetime.now(timezone.utc).isoformat()
        temporary = manifest.with_suffix(".jsonl.tmp")
        temporary.write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in existing + rows), encoding="utf-8")
        temporary.replace(manifest)
    except Exception:
        for target in copied:
            target.unlink(missing_ok=True)
        raise
    report = check_registered()
    (DEST / "STATUS.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("check", help="Read-only verification of already registered photos")
    for name in ("check-source", "register"):
        sub = commands.add_parser(name)
        sub.add_argument("--manifest", type=Path, required=True)
        if name == "register":
            sub.add_argument("--confirm-real", action="store_true", help="Human confirms actual camera photographs")
    args = parser.parse_args()
    try:
        if args.command == "check":
            result = check_registered()
        else:
            if args.command == "register" and not args.confirm_real:
                parser.error("register requires --confirm-real after human provenance review")
            rows = inspect_manifest(args.manifest.resolve())
            result = register(rows) if args.command == "register" else {
                "status": "source_metadata_checked", "images": len(rows), "use_for_training": False,
                "note": "Read-only; images remain at their original source paths"}
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(2, f"ERROR: {exc}\n")


if __name__ == "__main__":
    main()
