#!/usr/bin/env python3
"""First-cut Jenga block detector from Orbbec depth (camera frame).

Pipeline:
  1. Capture a median depth frame from the Orbbec DaBai DC1 (depth-only).
  2. Deproject to a Nx3 point cloud in the camera optical frame.
  3. RANSAC-fit the dominant plane (the table).
  4. Per-pixel height-above-plane -> threshold for Jenga thickness band ->
     binary blocks mask.
  5. Connected components -> for each, project its 3D points onto the plane,
     fit a 2D minimum-area rectangle, gate on Jenga footprint (75 x 25 mm).
  6. Report each detection's (x, y, z) centroid in the camera frame plus its
     yaw within the plane, and draw rectangles on a colorized depth image.

Output:
  snapshots/detect_<ts>_vis.png       depth + detected block rects + labels
  snapshots/detect_<ts>_mask.png      raised-points mask (debugging)
  stdout                              per-block table of detections

NOTE: poses are in the **camera optical frame**, not the arm base frame.
Hand-eye calibration (camera -> flange) is a separate step required to put
detections in arm coordinates.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pyorbbecsdk as ob


SNAP_DIR = Path(__file__).resolve().parent / "snapshots"

# Jenga block dimensions in millimetres (standard set: 75 x 25 x 15).
JENGA_LONG_MM = 75.0
JENGA_SHORT_MM = 25.0
JENGA_THICK_MM = 15.0


@dataclass
class Detection:
    cx_m: float       # centroid in camera frame, metres
    cy_m: float
    cz_m: float
    length_mm: float
    width_mm: float
    height_mm: float  # mean height above plane within the cluster
    yaw_rad: float    # rotation of the long edge within the plane (around plane normal)
    n_points: int     # cluster size
    rect_px: tuple    # ((cx, cy), (w, h), angle_deg) from cv2.minAreaRect — for drawing


def grab_depth_mm(args) -> tuple[np.ndarray, "ob.OBCameraIntrinsic", ob.VideoStreamProfile]:
    """Open the DaBai depth stream, return a median uint16 depth image (mm)."""
    pipe = ob.Pipeline()
    profiles = pipe.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
    depth_p = profiles.get_default_video_stream_profile()
    cfg = ob.Config()
    cfg.enable_stream(depth_p)
    pipe.start(cfg)
    try:
        for _ in range(args.warmup_frames):
            pipe.wait_for_frames(args.timeout_ms)
        stack = []
        misses = 0
        while len(stack) < args.frames and misses < 10:
            fs = pipe.wait_for_frames(args.timeout_ms)
            if fs is None:
                misses += 1
                continue
            df = fs.get_depth_frame()
            if df is None:
                misses += 1
                continue
            w, h = df.get_width(), df.get_height()
            raw = np.frombuffer(df.get_data(), dtype=np.uint16).reshape(h, w)
            scale = df.get_depth_scale()
            mm = (raw.astype(np.float32) * scale) if scale != 1.0 else raw.astype(np.float32)
            stack.append(mm)
        if not stack:
            raise RuntimeError("no depth frames received")
        return np.median(np.stack(stack), axis=0), depth_p.get_intrinsic(), depth_p
    finally:
        pipe.stop()


def deproject(depth_mm: np.ndarray, intr) -> np.ndarray:
    """HxW depth (mm) -> HxWx3 array of camera-frame points in mm (z=0 if invalid)."""
    h, w = depth_mm.shape
    vs, us = np.indices((h, w))
    z = depth_mm
    x = (us - intr.cx) * z / intr.fx
    y = (vs - intr.cy) * z / intr.fy
    return np.stack([x, y, z], axis=-1)


def ransac_plane(points: np.ndarray, iterations: int, dist_thresh_mm: float,
                 rng: np.random.Generator) -> tuple[np.ndarray, float, np.ndarray]:
    """RANSAC fit ax + by + cz = d to Nx3 points (mm).

    Returns (normal[3], d, inlier_mask). ``normal`` is unit-length and oriented so
    its dot with the camera +z axis is non-negative (so the plane faces the camera).
    """
    n = points.shape[0]
    if n < 3:
        return np.array([0.0, 0.0, 1.0]), 0.0, np.zeros(n, dtype=bool)
    best_inliers = np.zeros(n, dtype=bool)
    best_count = 0
    best_normal = np.array([0.0, 0.0, 1.0])
    best_d = 0.0
    for _ in range(iterations):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = points[idx]
        v1, v2 = p1 - p0, p2 - p0
        normal = np.cross(v1, v2)
        nn = np.linalg.norm(normal)
        if nn < 1e-6:
            continue
        normal /= nn
        d = float(normal @ p0)
        dist = np.abs(points @ normal - d)
        inliers = dist < dist_thresh_mm
        c = int(inliers.sum())
        if c > best_count:
            best_count = c
            best_inliers = inliers
            best_normal = normal
            best_d = d
    # Re-fit on the inlier set for accuracy (least-squares plane via SVD).
    if best_count >= 50:
        pts = points[best_inliers]
        centroid = pts.mean(axis=0)
        _, _, vh = np.linalg.svd(pts - centroid, full_matrices=False)
        normal = vh[-1]
        normal /= np.linalg.norm(normal)
        d = float(normal @ centroid)
        best_normal, best_d = normal, d
        best_inliers = np.abs(points @ best_normal - best_d) < dist_thresh_mm
    if best_normal @ np.array([0.0, 0.0, 1.0]) < 0:  # face the camera
        best_normal = -best_normal
        best_d = -best_d
    return best_normal, best_d, best_inliers


def plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two orthonormal vectors spanning the plane (for 2D projection)."""
    n = normal / np.linalg.norm(normal)
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    return u, v


def detect(depth_mm: np.ndarray, intr, args, rng: np.random.Generator) -> tuple[
        list[Detection], np.ndarray, np.ndarray, np.ndarray]:
    """Run the detector. Returns (detections, mask_uint8, height_above_plane_mm, points_hw3_mm)."""
    h, w = depth_mm.shape
    pts_hw3 = deproject(depth_mm, intr)  # mm

    valid = (depth_mm > args.near_mm) & (depth_mm < args.far_mm)
    valid_pts = pts_hw3[valid]

    # Sub-sample for RANSAC speed.
    if valid_pts.shape[0] > args.ransac_max_points:
        sub_idx = rng.choice(valid_pts.shape[0], size=args.ransac_max_points, replace=False)
        sample = valid_pts[sub_idx]
    else:
        sample = valid_pts

    normal, d, _ = ransac_plane(sample, args.ransac_iters, args.plane_thresh_mm, rng)

    # Signed height of every valid pixel above the plane (mm). Positive = toward camera.
    signed = pts_hw3 @ normal - d
    height_above = -signed  # invert: blocks SIT on the plane and stick *out toward camera*,
                            # so they're at *less* signed depth -> negative signed -> positive height.
    # We want height above the table, so we want pixels closer to the camera than the plane.
    # In the camera optical frame z grows away from the camera; a block on a table sits at
    # smaller z than the table surface seen behind it. signed = (point - plane) along normal.
    # Plane normal faces the camera (-z-ish), so points closer to the camera have signed<0.
    # height_above = -signed gives positive values for raised blocks.

    mask = (
        valid
        & (height_above >= args.thick_min_mm)
        & (height_above <= args.thick_max_mm)
    )

    # Clean the mask: drop speckle, fill small holes.
    mask_u8 = (mask * 255).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)

    # Connected components.
    num_lbl, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)

    u_axis, v_axis = plane_basis(normal)
    detections: list[Detection] = []
    for lbl in range(1, num_lbl):
        area_px = int(stats[lbl, cv2.CC_STAT_AREA])
        if area_px < args.min_cluster_px or area_px > args.max_cluster_px:
            continue
        comp_mask = (labels == lbl)
        comp_pts = pts_hw3[comp_mask]  # Nx3, mm
        if comp_pts.shape[0] < args.min_cluster_px:
            continue

        # 2D projection of the cluster points onto the plane basis (u, v).
        centroid_3d = comp_pts.mean(axis=0)
        rel = comp_pts - centroid_3d
        uv = np.stack([rel @ u_axis, rel @ v_axis], axis=1).astype(np.float32)

        # minAreaRect needs at least 5 points; we already enforce min_cluster_px above.
        rect2d = cv2.minAreaRect(uv)  # ((cx, cy), (W, H), angle_deg) — all in mm in (u,v).
        (_, _), (w_mm, h_mm), angle_deg = rect2d
        long_mm = max(w_mm, h_mm)
        short_mm = min(w_mm, h_mm)
        # Convert the rect angle into a yaw of the long edge.
        # If h_mm > w_mm the "angle" is from u-axis to short edge; bump by 90.
        long_angle_deg = angle_deg if w_mm >= h_mm else angle_deg + 90.0
        yaw_rad = float(np.deg2rad(long_angle_deg))

        # Jenga gate: tolerate ±50% (depth blobs are noisy).
        if not (args.long_min <= long_mm <= args.long_max):
            continue
        if not (args.short_min <= short_mm <= args.short_max):
            continue

        # Pixel-space minAreaRect for visualization (over the depth image).
        ys, xs = np.nonzero(comp_mask)
        px = np.stack([xs, ys], axis=1).astype(np.float32)
        rect_px = cv2.minAreaRect(px)

        block_height_mm = float(height_above[comp_mask].mean())

        detections.append(Detection(
            cx_m=float(centroid_3d[0]) / 1000.0,
            cy_m=float(centroid_3d[1]) / 1000.0,
            cz_m=float(centroid_3d[2]) / 1000.0,
            length_mm=float(long_mm),
            width_mm=float(short_mm),
            height_mm=block_height_mm,
            yaw_rad=yaw_rad,
            n_points=int(comp_pts.shape[0]),
            rect_px=rect_px,
        ))
    return detections, mask_u8, height_above, pts_hw3


def colorize_depth(depth_mm: np.ndarray, near_mm: int, far_mm: int) -> np.ndarray:
    clipped = np.clip(depth_mm, near_mm, far_mm)
    norm = ((clipped - near_mm) * 255.0 / max(far_mm - near_mm, 1)).astype(np.uint8)
    norm[depth_mm == 0] = 0
    vis = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    vis[depth_mm == 0] = (0, 0, 0)
    return vis


def main() -> int:
    ap = argparse.ArgumentParser()
    # Capture
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--warmup-frames", type=int, default=30)
    ap.add_argument("--timeout-ms", type=int, default=1000)
    # Depth gating
    ap.add_argument("--near-mm", type=int, default=200,
                    help="closer depths are ignored (camera dead zone)")
    ap.add_argument("--far-mm", type=int, default=1500,
                    help="farther depths are ignored")
    # Plane fit
    ap.add_argument("--ransac-iters", type=int, default=200)
    ap.add_argument("--ransac-max-points", type=int, default=20000)
    ap.add_argument("--plane-thresh-mm", type=float, default=8.0)
    # Block thickness band (a Jenga is 15 mm thick; pad for noise + camera tilt)
    ap.add_argument("--thick-min-mm", type=float, default=5.0)
    ap.add_argument("--thick-max-mm", type=float, default=35.0)
    # Footprint gate (Jenga long = 75 mm, short = 25 mm; ±50% padding)
    ap.add_argument("--long-min", type=float, default=40.0)
    ap.add_argument("--long-max", type=float, default=110.0)
    ap.add_argument("--short-min", type=float, default=12.0)
    ap.add_argument("--short-max", type=float, default=40.0)
    # Cluster sizes
    ap.add_argument("--min-cluster-px", type=int, default=150)
    ap.add_argument("--max-cluster-px", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    SNAP_DIR.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    ctx = ob.Context()
    devs = ctx.query_devices()
    if devs.get_count() == 0:
        print("ERROR: no Orbbec device found on USB", file=sys.stderr)
        return 2
    info = devs.get_device_by_index(0).get_device_info()
    print(f"Device: {info.get_name()} sn={info.get_serial_number()} fw={info.get_firmware_version()}")

    depth_mm, intr, depth_p = grab_depth_mm(args)
    print(
        f"Depth: {depth_mm.shape[1]}x{depth_mm.shape[0]}  "
        f"intrinsics fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.cx:.1f} cy={intr.cy:.1f}"
    )

    detections, mask_u8, height_above, _ = detect(depth_mm.astype(np.uint16), intr, args, rng)

    ts = int(time.time())
    vis = colorize_depth(depth_mm.astype(np.uint16), args.near_mm, args.far_mm)
    # Overlay the raised-pixel mask in white for context.
    overlay = vis.copy()
    overlay[mask_u8 > 0] = (255, 255, 255)
    vis = cv2.addWeighted(vis, 0.5, overlay, 0.5, 0)

    for i, d in enumerate(detections):
        box = cv2.boxPoints(d.rect_px).astype(int)
        cv2.drawContours(vis, [box], 0, (0, 255, 0), 2)
        cx_px, cy_px = int(d.rect_px[0][0]), int(d.rect_px[0][1])
        cv2.circle(vis, (cx_px, cy_px), 3, (0, 255, 255), -1)
        cv2.putText(vis, f"#{i}", (cx_px + 5, cy_px - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

    vis_path = SNAP_DIR / f"detect_{ts}_vis.png"
    mask_path = SNAP_DIR / f"detect_{ts}_mask.png"
    cv2.imwrite(str(vis_path), vis)
    cv2.imwrite(str(mask_path), mask_u8)

    print()
    print(f"{len(detections)} candidate block(s) — saved overlay to {vis_path}")
    print(f"                                       mask to     {mask_path}")
    if detections:
        print()
        print(f"{'idx':>3}  {'x_m':>7}  {'y_m':>7}  {'z_m':>7}  "
              f"{'long_mm':>8}  {'short_mm':>9}  {'tall_mm':>8}  {'yaw_deg':>8}  {'n_pts':>6}")
        for i, d in enumerate(detections):
            print(
                f"{i:>3}  {d.cx_m:>7.3f}  {d.cy_m:>7.3f}  {d.cz_m:>7.3f}  "
                f"{d.length_mm:>8.1f}  {d.width_mm:>9.1f}  {d.height_mm:>8.1f}  "
                f"{np.rad2deg(d.yaw_rad):>8.1f}  {d.n_points:>6d}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
