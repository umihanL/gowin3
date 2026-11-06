"""Manual camera capture only: explicit device, explicit format, C then SPACE to save."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path

from real_capture_dataset import FIELDS, ID, PROJECT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", type=int, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--layout", choices=("mono", "stereo_sbs"), required=True)
    parser.add_argument("--camera-model", required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--pose", required=True)
    parser.add_argument("--distance", type=float, required=True)
    parser.add_argument("--angle", type=float, required=True)
    parser.add_argument("--light", required=True)
    parser.add_argument("--occlusion", required=True)
    parser.add_argument("--notes", default="")
    parser.add_argument("--confirm-real", action="store_true")
    args = parser.parse_args()
    if not args.confirm_real:
        parser.error("--confirm-real is required; do not capture a synthetic preview or unknown camera")
    if args.device < 0 or min(args.width, args.height) < 2:
        parser.error("Specify a non-negative device index and valid measured full-frame dimensions")
    if args.layout == "stereo_sbs" and args.width % 2:
        parser.error("SBS full frame width must be even")
    if not ID.fullmatch(args.session) or not ID.fullmatch(args.pose):
        parser.error("Session and pose IDs require letters, digits, _ or -")
    if not 0 < args.distance <= 100 or not 0 <= args.angle <= 90:
        parser.error("Invalid measured distance/angle")
    if not all(value.strip() for value in (args.camera_model, args.light, args.occlusion)):
        parser.error("Camera, light and occlusion cannot be blank")

    # Import and device opening occur only after all user-supplied arguments pass.
    import cv2

    print(f"Opening explicitly requested device {args.device}; requested {args.width}x{args.height} ({args.layout}).")
    print("C: confirm this is the intended REAL camera, scene, and mono/SBS format.")
    print("SPACE: save one unaltered full frame after confirmation. Q: quit. No training.")
    cap = cv2.VideoCapture(args.device, cv2.CAP_DSHOW)
    try:
        if not cap.isOpened():
            raise RuntimeError("Could not open requested device; no fallback device will be opened")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        confirmed = False
        saved = 0
        printed_shape = False
        directory = PROJECT / "real_capture_inbox" / args.session / args.pose
        manifest = directory / "manifest.csv"
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError("Camera returned no frame")
            actual_h, actual_w = frame.shape[:2]
            if not printed_shape:
                print(f"Actual full frame: {actual_w}x{actual_h}; per-eye width for SBS: {actual_w // 2 if args.layout == 'stereo_sbs' else actual_w}")
                printed_shape = True
            if (actual_w, actual_h) != (args.width, args.height):
                raise RuntimeError("Actual frame size differs from request. Verify manufacturer's mode and restart explicitly.")
            display = frame.copy()
            message = f"{'READY: SPACE save' if confirmed else 'C: confirm REAL scene + layout'} | saved {saved} | Q quit"
            cv2.putText(display, message, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.imshow("Manual real validation capture", display)
            key = cv2.waitKey(10) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("c"), ord("C")):
                confirmed = True
                print("User confirmed actual camera, real scene and layout; SPACE now saves a full frame.")
            if key == 32 and confirmed:
                captured = datetime.now(timezone.utc)
                name = captured.strftime("%Y%m%dT%H%M%S_%fZ") + ".png"
                directory.mkdir(parents=True, exist_ok=True)
                path = directory / name
                ok, encoded = cv2.imencode(".png", frame)
                if not ok:
                    raise RuntimeError("Could not encode raw camera frame")
                with path.open("xb") as stream:
                    stream.write(encoded.tobytes())
                row = dict(image_path=str(path), session_id=args.session, pose_id=args.pose,
                           capture_utc=captured.isoformat(), camera_model=args.camera_model,
                           layout=args.layout, distance_m=args.distance, angle_deg=args.angle,
                           lighting=args.light, occlusion=args.occlusion, source_origin="real_camera",
                           notes=args.notes)
                need_header = not manifest.exists()
                with manifest.open("a", encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=FIELDS)
                    if need_header:
                        writer.writeheader()
                    writer.writerow(row)
                saved += 1
                print(f"Saved unaltered frame {path}")
        if saved:
            print(f"Pending intake CSV: {manifest}\nRun real_capture_dataset.py check-source, then register --confirm-real.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
