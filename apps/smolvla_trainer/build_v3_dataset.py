#!/usr/bin/env python3
"""Build a fresh LeRobot v3.0 dataset from episodes/*.jsonl + mp4s.

The original build_lerobot.py wrote v2.0 hand-rolled. The installed lerobot
needs v3.0 with a different file layout (data/chunk-000/file_000.parquet,
meta/tasks/, meta/episodes/, etc.). The cleanest way to get v3.0 right is
to use lerobot's own LeRobotDataset.create() + add_frame() + save_episode()
API instead of hand-writing parquet.

Usage:
  .venv/bin/python build_v3_dataset.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import av
import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset

REPO_ROOT = Path(__file__).resolve().parents[2]
TASK_DESCRIPTION = "jenga teleop demonstration"
FPS = 30  # video container fps (jsonl ticks are slower; we'll find the closest video frame per tick)


def parse_jsonl(path: Path) -> list[dict]:
    out = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        joints = obj.get("joints_deg")
        if not isinstance(joints, list) or len(joints) != 6:
            continue
        try:
            joints_f = [float(x) for x in joints]
            gripper = float(obj.get("gripper", 0.5))
        except (TypeError, ValueError):
            continue
        out.append({"t_us": int(obj["t_us"]), "joints_deg": joints_f, "gripper": gripper})
    return out


def decode_all_frames(video_path: Path, resize: tuple[int, int] | None = None) -> list[np.ndarray]:
    container = av.open(str(video_path))
    frames = []
    for frame in container.decode(video=0):
        rgb = frame.to_ndarray(format="rgb24")
        if resize is not None:
            from PIL import Image
            img = Image.fromarray(rgb).resize(resize, Image.BILINEAR)
            rgb = np.asarray(img)
        frames.append(rgb)
    container.close()
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=Path, default=REPO_ROOT / "episodes")
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "dataset_v3")
    ap.add_argument("--repo-id", default="jengabot/teleop")
    ap.add_argument("--img-size", type=int, default=256, help="square resize for both cams")
    args = ap.parse_args()

    jsonls = sorted(args.episodes.glob("episode_*.jsonl"))
    if not jsonls:
        raise SystemExit(f"no episode_*.jsonl in {args.episodes}")

    # Determine cam availability per episode
    eps_with_both = []
    for j in jsonls:
        top = args.episodes / f"{j.stem}_top.mp4"
        dab = args.episodes / f"{j.stem}_dabai.mp4"
        if top.exists() and dab.exists():
            eps_with_both.append((j, top, dab))

    if not eps_with_both:
        raise SystemExit("no episodes have BOTH top and dabai mp4; SmolVLA needs consistent cams")

    print(f"found {len(eps_with_both)} episode(s) with both top+dabai videos")

    if args.out.exists():
        import shutil
        shutil.rmtree(args.out)

    img_wh = (args.img_size, args.img_size)
    features = {
        "observation.state": {"dtype": "float32", "shape": (7,),
                              "names": ["j1", "j2", "j3", "j4", "j5", "j6", "gripper"]},
        "action": {"dtype": "float32", "shape": (7,),
                   "names": ["j1", "j2", "j3", "j4", "j5", "j6", "gripper"]},
        "observation.images.top": {"dtype": "video", "shape": (args.img_size, args.img_size, 3),
                                    "names": ["height", "width", "channel"]},
        "observation.images.dabai": {"dtype": "video", "shape": (args.img_size, args.img_size, 3),
                                      "names": ["height", "width", "channel"]},
    }

    ds = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=FPS,
        features=features,
        root=args.out,
        robot_type="piper_x_so101_teleop",
        use_videos=True,
    )

    for ep_i, (jp, tp, dp) in enumerate(eps_with_both):
        rows = parse_jsonl(jp)
        if not rows:
            print(f"  skip {jp.name} (0 rows)")
            continue
        print(f"  ep{ep_i}: {jp.name}  rows={len(rows)}")

        top_frames = decode_all_frames(tp, resize=img_wh)
        dab_frames = decode_all_frames(dp, resize=img_wh)
        t0 = rows[0]["t_us"]
        ep_dur_s = (rows[-1]["t_us"] - t0) / 1e6
        print(f"    top frames={len(top_frames)}  dabai frames={len(dab_frames)}  jsonl dur={ep_dur_s:.1f}s")

        # Map each jsonl row's timestamp to nearest video frame index per cam.
        def pick(n_video_frames):
            if n_video_frames == 0:
                return [0] * len(rows)
            return [min(n_video_frames - 1,
                        int(round((r["t_us"] - t0) / 1e6 / ep_dur_s * (n_video_frames - 1)))
                            if ep_dur_s > 0 else 0)
                    for r in rows]

        top_idx = pick(len(top_frames))
        dab_idx = pick(len(dab_frames))

        for fi, r in enumerate(rows):
            state = np.array(r["joints_deg"] + [r["gripper"]], dtype=np.float32)
            frame = {
                "observation.state": state,
                "action": state.copy(),  # proxy
                "observation.images.top": top_frames[top_idx[fi]],
                "observation.images.dabai": dab_frames[dab_idx[fi]],
                "task": TASK_DESCRIPTION,
            }
            ds.add_frame(frame)
        ds.save_episode()
        print(f"    ep{ep_i} saved")

    print(f"\ndone. dataset at {args.out}")


if __name__ == "__main__":
    main()
