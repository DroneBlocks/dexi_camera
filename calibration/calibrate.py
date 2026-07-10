#!/usr/bin/env python3
"""Offline intrinsic calibration from checkerboard PNGs. Runs on the Mac.

  python3 calibrate.py FRAME_DIR [--cols 7] [--rows 9] [--square 0.015] [--holdout 5]

Board default matches docs/assets/calib.io_checker_200x150_8x10_15.pdf
(7x9 interior corners, 15 mm squares).
"""
import argparse, glob, math, os, sys
import cv2
import numpy as np

SB_FLAGS = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE


def find_corners(paths, pattern):
    good, corners, shape = [], [], None
    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        shape = img.shape[::-1]
        ok, c = cv2.findChessboardCornersSB(img, pattern, flags=SB_FLAGS)
        if ok:
            good.append(p)
            corners.append(c)
    return good, corners, shape


def coverage_grid(corners, shape, n=6):
    w, h = shape
    grid = np.zeros((n, n), dtype=int)
    for c in corners:
        for pt in c.reshape(-1, 2):
            gx = min(int(pt[0] / w * n), n - 1)
            gy = min(int(pt[1] / h * n), n - 1)
            grid[gy, gx] += 1
    return grid


def per_view_error(objp, corners, rvecs, tvecs, K, D):
    errs = []
    for i, c in enumerate(corners):
        proj, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], K, D)
        d = c.reshape(-1, 2).astype(np.float64) - proj.reshape(-1, 2).astype(np.float64)
        errs.append(float(np.sqrt((d ** 2).sum(axis=1).mean())))
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frame_dir")
    ap.add_argument("--cols", type=int, default=7)
    ap.add_argument("--rows", type=int, default=9)
    ap.add_argument("--square", type=float, default=0.015)
    ap.add_argument("--holdout", type=int, default=5)
    ap.add_argument("--name", default="cam0")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    pattern = (a.cols, a.rows)
    paths = sorted(glob.glob(os.path.join(a.frame_dir, "*.png")))
    if not paths:
        sys.exit(f"no PNGs in {a.frame_dir}")

    print(f"scanning {len(paths)} frames for a {a.cols}x{a.rows} board ...")
    good, corners, shape = find_corners(paths, pattern)
    print(f"  detected board in {len(good)}/{len(paths)} frames  (image {shape[0]}x{shape[1]})")
    if len(good) < 12:
        sys.exit("need >= 12 detections; capture more views")

    print("\ncorner coverage (6x6 grid, counts):")
    grid = coverage_grid(corners, shape)
    for row in grid:
        print("  " + " ".join(f"{v:5d}" for v in row))
    empty = int((grid == 0).sum())
    print(f"  empty cells: {empty}/36" + ("   <-- push the board into the frame edges" if empty else "   good"))

    objp = np.zeros((a.cols * a.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square

    # holdout split
    idx = list(range(len(good)))
    hold = idx[:: max(1, len(idx) // a.holdout)][: a.holdout] if a.holdout else []
    fit = [i for i in idx if i not in hold]
    print(f"\nfitting on {len(fit)} views, holding out {len(hold)}")

    flags = cv2.CALIB_FIX_K3  # ROS plumb_bob: k1 k2 p1 p2 (k3=0)
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        [objp] * len(fit), [corners[i] for i in fit], shape, None, None, flags=flags)

    print(f"\nRMS reprojection error (fit): {rms:.4f} px")
    errs = per_view_error(objp, [corners[i] for i in fit], rvecs, tvecs, K, D)
    print(f"  per-view: min {min(errs):.3f}  median {np.median(errs):.3f}  max {max(errs):.3f}")
    worst = int(np.argmax(errs))
    print(f"  worst view: {os.path.basename(good[fit[worst]])} ({max(errs):.3f} px)")

    if hold:
        he = []
        for i in hold:
            ok, rv, tv = cv2.solvePnP(objp, corners[i], K, D)
            if ok:
                he += per_view_error(objp, [corners[i]], [rv], [tv], K, D)
        if he:
            print(f"  holdout: median {np.median(he):.3f}  max {max(he):.3f} px")

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    w, h = shape
    hfov = 2 * math.degrees(math.atan(w / (2 * fx)))
    vfov = 2 * math.degrees(math.atan(h / (2 * fy)))
    print(f"\nfx={fx:.2f}  fy={fy:.2f}  (fx/fy = {fx/fy:.4f})")
    print(f"cx={cx:.2f}  cy={cy:.2f}   (image centre {w/2:.1f}, {h/2:.1f}  -> offset {cx-w/2:+.1f}, {cy-h/2:+.1f})")
    print(f"D = {np.round(D.ravel(), 5).tolist()}")
    print(f"implied FOV: {hfov:.1f} deg horizontal, {vfov:.1f} deg vertical")

    print("\nsanity gates:")
    def gate(ok, msg):
        print(("  PASS  " if ok else "  FAIL  ") + msg)
    gate(rms < 0.3, f"RMS < 0.30 px (got {rms:.3f})")
    gate(abs(fx / fy - 1) < 0.01, f"fx/fy within 1% (got {abs(fx/fy-1)*100:.2f}%)")
    gate(abs(cx - w / 2) < 0.1 * w, f"cx within 10% of centre (off by {abs(cx-w/2):.1f} px)")
    gate(abs(cy - h / 2) < 0.1 * h, f"cy within 10% of centre (off by {abs(cy-h/2):.1f} px)")
    gate(empty == 0, "all 36 coverage cells populated")

    out = a.out or os.path.join(a.frame_dir, f"{a.name}_calib.yaml")
    d = D.ravel().tolist()
    while len(d) < 5:
        d.append(0.0)
    with open(out, "w") as f:
        f.write(f"image_width: {w}\nimage_height: {h}\ncamera_name: {a.name}\n")
        f.write("camera_matrix:\n  rows: 3\n  cols: 3\n  data: [" +
                ", ".join(f"{v:.6f}" for v in K.ravel()) + "]\n")
        f.write(f"distortion_model: plumb_bob\n")
        f.write("distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: [" +
                ", ".join(f"{v:.6f}" for v in d[:5]) + "]\n")
        f.write("rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n")
        P = np.hstack([K, np.zeros((3, 1))])
        f.write("projection_matrix:\n  rows: 3\n  cols: 4\n  data: [" +
                ", ".join(f"{v:.6f}" for v in P.ravel()) + "]\n")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
