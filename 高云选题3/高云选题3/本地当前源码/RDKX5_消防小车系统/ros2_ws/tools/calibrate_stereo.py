#!/usr/bin/env python3
"""Measure a horizontal stereo pair using synchronized chessboard photographs."""
import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "rescue_vision"))
from rescue_vision.stereo import StereoDepth


def images_by_stem(folder):
    found = {}
    for p in sorted(folder.iterdir()):
        if p.suffix.lower() not in (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"):
            continue
        if p.stem in found:
            raise ValueError(f"Duplicate image stem: {p.stem}")
        found[p.stem] = p
    return found


def load_gray(path):
    image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"Cannot decode image: {path}")
    return image


def calibration_report(args):
    if not args.synchronized_capture_confirmed:
        raise ValueError("Use synchronized pairs and pass --synchronized-capture-confirmed; timestamps cannot be inferred from filenames")
    if min(args.cols, args.rows) < 4 or not .001 <= args.square_m <= .3:
        raise ValueError("Need at least 4x4 internal corners and measured square size 0.001..0.3 metres")
    left_files, right_files = images_by_stem(args.left), images_by_stem(args.right)
    if set(left_files) != set(right_files):
        raise ValueError("Left/right directories must contain exactly matching image stems")
    object_grid = np.zeros((args.rows * args.cols, 3), np.float32)
    object_grid[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_m
    names, left_points, right_points, rejected = [], [], [], []
    size = None
    for name in sorted(left_files):
        left, right = load_gray(left_files[name]), load_gray(right_files[name])
        if left.shape != right.shape or (size and left.shape[::-1] != size):
            raise ValueError("Every eye and every frame must have the same full sensor resolution")
        size = left.shape[::-1]
        ok_l, corners_l = cv2.findChessboardCornersSB(left, (args.cols, args.rows), cv2.CALIB_CB_NORMALIZE_IMAGE)
        ok_r, corners_r = cv2.findChessboardCornersSB(right, (args.cols, args.rows), cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok_l or not ok_r:
            rejected.append(name)
            continue
        names.append(name)
        left_points.append(corners_l.astype(np.float32))
        right_points.append(corners_r.astype(np.float32))
    if len(names) < 20:
        raise ValueError(f"Need >=20 usable synchronized pairs (recommend 30-50), found {len(names)}")
    # Every fifth frame is withheld from all intrinsic and stereo fitting.
    validation = [i for i in range(len(names)) if i % 5 == 0]
    train = [i for i in range(len(names)) if i not in validation]
    obj = [object_grid.copy() for _ in train]
    lp, rp = [left_points[i] for i in train], [right_points[i] for i in train]
    rms_l, K1, D1, _, _ = cv2.calibrateCamera(obj, lp, size, None, None)
    rms_r, K2, D2, _, _ = cv2.calibrateCamera(obj, rp, size, None, None)
    rms_stereo, K1, D1, K2, D2, R, T, _, _ = cv2.stereoCalibrate(
        obj, lp, rp, K1, D1, K2, D2, size,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7),
        flags=cv2.CALIB_FIX_INTRINSIC)
    report = {
        "calibrated": True,
        "calibration_id": "gs130w_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"),
        "camera_model": "pinhole", "translation_unit": "m", "image_size": list(size),
        "K1": K1.tolist(), "D1": D1.ravel().tolist(), "K2": K2.tolist(), "D2": D2.ravel().tolist(),
        "R": R.tolist(), "T": T.ravel().tolist(),
        "board": {"internal_corners": [args.cols, args.rows], "square_size_m": args.square_m},
        "synchronized_capture_confirmed_by_operator": True,
    }
    stereo = StereoDepth(report, {"num_disparities": min(96, (size[0] // 3 // 16) * 16)})
    epipolar, heldout_reprojection = [], []
    for i in validation:
        l = cv2.undistortPoints(left_points[i], K1, D1, R=stereo.R1, P=stereo.P1)
        r = cv2.undistortPoints(right_points[i], K2, D2, R=stereo.R2, P=stereo.P2)
        epipolar.extend(np.abs(l[:, 0, 1] - r[:, 0, 1]).tolist())
        ok, rvec, tvec = cv2.solvePnP(object_grid, left_points[i], K1, D1)
        if not ok:
            raise ValueError("Held-out board pose could not be estimated")
        projected_l, _ = cv2.projectPoints(object_grid, rvec, tvec, K1, D1)
        rotation, _ = cv2.Rodrigues(rvec)
        right_rvec, _ = cv2.Rodrigues(R @ rotation)
        projected_r, _ = cv2.projectPoints(object_grid, right_rvec, R @ tvec + T, K2, D2)
        heldout_reprojection.extend(np.linalg.norm(projected_l - left_points[i], axis=2).ravel().tolist())
        heldout_reprojection.extend(np.linalg.norm(projected_r - right_points[i], axis=2).ravel().tolist())
    coverage = []
    for corners in (lp, rp):
        all_points = np.concatenate(corners).reshape(-1, 2)
        coverage.append((np.ptp(all_points, axis=0) / np.asarray(size)).tolist())
    quality = {
        "train_rms_left_px": float(rms_l), "train_rms_right_px": float(rms_r),
        "train_stereo_rms_px": float(rms_stereo),
        "heldout_epipolar_mean_px": float(np.mean(epipolar)),
        "heldout_epipolar_p95_px": float(np.percentile(epipolar, 95)),
        "heldout_reprojection_mean_px": float(np.mean(heldout_reprojection)),
        "training_xy_coverage_left_right": coverage,
        "baseline_m": stereo.baseline_m,
        "fit_pair_count": len(train), "heldout_pair_count": len(validation),
        "rejected_missing_corners": rejected,
    }
    failures = []
    if max(rms_l, rms_r, rms_stereo) > args.max_rms_px:
        failures.append("training_reprojection_error")
    if quality["heldout_epipolar_mean_px"] > args.max_epipolar_mean_px or quality["heldout_epipolar_p95_px"] > args.max_epipolar_p95_px:
        failures.append("heldout_epipolar_error")
    if quality["heldout_reprojection_mean_px"] > args.max_heldout_reprojection_px:
        failures.append("heldout_reprojection_error")
    if min(v for eye in coverage for v in eye) < .45:
        failures.append("insufficient_image_coverage")
    quality["passed"] = not failures
    quality["failures"] = failures
    quality["thresholds"] = dict(train_rms_px=args.max_rms_px, epipolar_mean_px=args.max_epipolar_mean_px,
                                  epipolar_p95_px=args.max_epipolar_p95_px,
                                  heldout_reprojection_mean_px=args.max_heldout_reprojection_px,
                                  min_xy_coverage=.45)
    report["quality"] = quality
    report["calibrated"] = not failures
    report["source_pairs"] = [{"stem": name, "split": "validation" if i in validation else "fit",
                               "left_sha256": hashlib.sha256(left_files[name].read_bytes()).hexdigest(),
                               "right_sha256": hashlib.sha256(right_files[name].read_bytes()).hexdigest()}
                              for i, name in enumerate(names)]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", type=Path, required=True)
    parser.add_argument("--right", type=Path, required=True)
    parser.add_argument("--cols", type=int, required=True, help="Internal corner columns, not square count")
    parser.add_argument("--rows", type=int, required=True, help="Internal corner rows")
    parser.add_argument("--square-m", type=float, required=True, help="Measured square edge length in metres")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--synchronized-capture-confirmed", action="store_true")
    parser.add_argument("--max-rms-px", type=float, default=1.0)
    parser.add_argument("--max-epipolar-mean-px", type=float, default=.75)
    parser.add_argument("--max-epipolar-p95-px", type=float, default=1.5)
    parser.add_argument("--max-heldout-reprojection-px", type=float, default=1.5)
    args = parser.parse_args()
    try:
        for value in (args.max_rms_px, args.max_epipolar_mean_px, args.max_epipolar_p95_px,
                      args.max_heldout_reprojection_px):
            if not np.isfinite(value) or value <= 0:
                raise ValueError("Quality thresholds must be finite and positive")
        report = calibration_report(args)
    except (ValueError, OSError, cv2.error) as exc:
        parser.exit(2, f"Calibration failed: {exc}\n")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report["quality"], ensure_ascii=False, indent=2))
    if not report["calibrated"]:
        parser.exit(2, "Quality gate failed: saved diagnostic JSON with calibrated=false; do not deploy it for metric depth.\n")
    print(f"Measured calibration saved: {args.output}")


if __name__ == "__main__":
    main()
