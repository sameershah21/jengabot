#!/usr/bin/env python3
"""Orbbec Dabai DC1 smoke test — depth-only path.

Opens the Orbbec DEPTH stream (color stream omitted on purpose: on macOS the
UVC color interface needs root to open, while the depth IR interface does
not). Captures a median of ``--frames`` depth frames and writes:

  snapshots/depth_<ts>.png       16-bit, raw millimetres
  snapshots/depth_<ts>_vis.png   8-bit colormapped for human eyes

Verified specs on this hardware (DaBai DC1, fw RD1001):
  depth = 640x400 @ 30 fps, format Y11
  intrinsics fx=fy≈477, cx≈326, cy≈198
  raw uint16 * depth_scale = millimetres
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyorbbecsdk as ob


SNAP_DIR = Path(__file__).resolve().parent / "snapshots"


def list_profiles(label: str, profile_list) -> None:
    print(f"  {label} profiles ({profile_list.get_count()}):")
    for i in range(profile_list.get_count()):
        p = profile_list.get_stream_profile_by_index(i).as_video_stream_profile()
        print(
            f"    [{i:2d}] {p.get_width():4d}x{p.get_height():4d} "
            f"@ {p.get_fps():3d}fps  fmt={p.get_format()}"
        )


def colorize_depth(depth_mm: np.ndarray, near_mm: int, far_mm: int) -> np.ndarray:
    clipped = np.clip(depth_mm, near_mm, far_mm)
    norm = ((clipped - near_mm) * 255.0 / max(far_mm - near_mm, 1)).astype(np.uint8)
    norm[depth_mm == 0] = 0
    vis = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    vis[depth_mm == 0] = (0, 0, 0)
    return vis


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=5,
                    help="median depth frames to combine into the saved snapshot")
    ap.add_argument("--warmup-frames", type=int, default=30,
                    help="frames to discard before recording (structured-light settles)")
    ap.add_argument("--timeout-ms", type=int, default=1000)
    ap.add_argument("--near-mm", type=int, default=200)
    ap.add_argument("--far-mm", type=int, default=1500)
    args = ap.parse_args()

    SNAP_DIR.mkdir(parents=True, exist_ok=True)

    ctx = ob.Context()
    devs = ctx.query_devices()
    if devs.get_count() == 0:
        print("ERROR: no Orbbec device found on USB", file=sys.stderr)
        return 2
    info = devs.get_device_by_index(0).get_device_info()
    print(
        f"Device: {info.get_name()} pid=0x{info.get_pid():04x} "
        f"sn={info.get_serial_number()} fw={info.get_firmware_version()}"
    )

    pipe = ob.Pipeline()  # depth-only Pipeline (no explicit device — avoids color open)
    depth_profiles = pipe.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
    list_profiles("depth", depth_profiles)
    depth_p = depth_profiles.get_default_video_stream_profile()
    print(
        f"\nUsing depth {depth_p.get_width()}x{depth_p.get_height()}"
        f"@{depth_p.get_fps()} fmt={depth_p.get_format()}"
    )

    cfg = ob.Config()
    cfg.enable_stream(depth_p)
    pipe.start(cfg)

    try:
        # Warmup
        for _ in range(args.warmup_frames):
            pipe.wait_for_frames(args.timeout_ms)

        # Capture a stack and take the median per pixel (mm).
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
            scale = df.get_depth_scale()  # mm per unit
            depth_mm = (raw.astype(np.float32) * scale).astype(np.uint16) if scale != 1.0 else raw.copy()
            stack.append(depth_mm)

        if not stack:
            print("ERROR: never received a depth frame", file=sys.stderr)
            return 3

        median_mm = np.median(np.stack(stack), axis=0).astype(np.uint16)

        # Intrinsics (helpful for downstream deprojection).
        try:
            intr = depth_p.get_intrinsic()
            print(
                f"intrinsics fx={intr.fx:.1f} fy={intr.fy:.1f} "
                f"cx={intr.cx:.1f} cy={intr.cy:.1f}  "
                f"w={depth_p.get_width()} h={depth_p.get_height()}"
            )
        except Exception as e:
            print(f"(intrinsics unavailable: {e})")

        ts = int(time.time())
        depth_path = SNAP_DIR / f"depth_{ts}.png"
        vis_path = SNAP_DIR / f"depth_{ts}_vis.png"
        cv2.imwrite(str(depth_path), median_mm)
        cv2.imwrite(str(vis_path), colorize_depth(median_mm, args.near_mm, args.far_mm))

        valid = int((median_mm > 0).sum())
        total = median_mm.size
        nz = median_mm[median_mm > 0]
        d_min = int(nz.min()) if nz.size else 0
        d_max = int(nz.max()) if nz.size else 0
        d_med = int(np.median(nz)) if nz.size else 0
        print()
        print(
            f"depth {median_mm.shape[1]}x{median_mm.shape[0]}  "
            f"frames_used={len(stack)}/{args.frames}  "
            f"valid={valid}/{total} ({100*valid/total:.1f}%)  "
            f"min={d_min}mm med={d_med}mm max={d_max}mm"
        )
        print(f"  raw 16-bit mm  -> {depth_path}")
        print(f"  colorized      -> {vis_path}")
        return 0
    finally:
        pipe.stop()


if __name__ == "__main__":
    sys.exit(main())
