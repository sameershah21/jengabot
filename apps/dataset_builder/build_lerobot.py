#!/usr/bin/env python3
"""Convert jengabot/episodes/*.jsonl + matching mp4s into a HF LeRobot v2 dataset.

Input layout (existing):
  episodes/
    episode_<TS>.jsonl              # leader JSON-line stream
    episode_<TS>_dabai.mp4          # arm-mounted RGB (optional)
    episode_<TS>_top.mp4            # icspring top-down RGB (optional)

Output layout (LeRobot v2):
  dataset/
    data/chunk-000/episode_NNNNNN.parquet
    videos/chunk-000/observation.images.dabai/episode_NNNNNN.mp4
    videos/chunk-000/observation.images.top/episode_NNNNNN.mp4
    meta/info.json
    meta/tasks.jsonl
    meta/episodes.jsonl

PROXY: `action` == `observation.state` for now. Both come from the leader's
commanded joints + gripper. To get a real `observation.state` from Bruce's
own feedback, run follower_play with --observed-log <path>, then re-build.

Usage:
  .venv/bin/python build_lerobot.py
  .venv/bin/python build_lerobot.py --episodes /custom/path --out /custom/dataset
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path

import pandas as pd

ROBOT_TYPE = "piper_x_so101_teleop"
TASK_DESCRIPTION = "jenga teleop demonstration (SO-101 leader, PiPER-X follower)"
DEFAULT_FPS = 30  # video container fps; jsonl is ~2 Hz from SO-101 board

REPO_ROOT = Path(__file__).resolve().parents[2]


def parse_jsonl(path: Path) -> list[dict]:
    """Yield one row per valid JSON line. Skip malformed lines."""
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
        gripper = obj.get("gripper", 0.5)
        try:
            joints_f = [float(x) for x in joints]
            gripper_f = float(gripper)
        except (TypeError, ValueError):
            continue
        out.append({
            "t_us": int(obj["t_us"]),
            "joints_deg": joints_f,
            "gripper": gripper_f,
        })
    return out


def video_resolution(path: Path) -> tuple[int, int] | None:
    """Return (width, height) of the first video stream, or None on failure."""
    try:
        r = subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "v:0",
             "-show_entries", "stream=width,height", "-of", "csv=p=0:s=,",
             str(path)],
            capture_output=True, text=True, check=True,
        )
        w, h = r.stdout.strip().split(",")
        return int(w), int(h)
    except Exception:
        return None


def build(episodes_dir: Path, out_dir: Path) -> None:
    jsonls = sorted(episodes_dir.glob("episode_*.jsonl"))
    if not jsonls:
        print(f"no episodes found in {episodes_dir}")
        return

    print(f"found {len(jsonls)} episode jsonl(s) in {episodes_dir}")

    # Wipe + recreate output skeleton
    if out_dir.exists():
        shutil.rmtree(out_dir)
    (out_dir / "data" / "chunk-000").mkdir(parents=True)
    (out_dir / "videos" / "chunk-000" / "observation.images.dabai").mkdir(parents=True)
    (out_dir / "videos" / "chunk-000" / "observation.images.top").mkdir(parents=True)
    (out_dir / "meta").mkdir()

    episodes_meta = []
    total_frames = 0
    dabai_wh = None
    top_wh = None
    seen_dabai = False
    seen_top = False

    for ep_idx, jpath in enumerate(jsonls):
        rows = parse_jsonl(jpath)
        if not rows:
            print(f"  skip {jpath.name}: 0 valid rows")
            continue

        t0 = rows[0]["t_us"]
        df_rows = []
        for fidx, r in enumerate(rows):
            ts_s = (r["t_us"] - t0) / 1e6
            state = r["joints_deg"] + [r["gripper"]]   # 7-vector
            action = list(state)                       # proxy
            df_rows.append({
                "episode_index": ep_idx,
                "frame_index": fidx,
                "timestamp": ts_s,
                "observation.state": state,
                "action": action,
                "task_index": 0,
            })
        df = pd.DataFrame(df_rows)
        out_parquet = out_dir / "data" / "chunk-000" / f"episode_{ep_idx:06d}.parquet"
        df.to_parquet(out_parquet, index=False)

        # Copy videos
        stem = jpath.stem  # e.g. "episode_20260524_123234"
        for cam_key, suffix in [("dabai", "_dabai"), ("top", "_top")]:
            src = episodes_dir / f"{stem}{suffix}.mp4"
            if not src.exists():
                continue
            dst = out_dir / "videos" / "chunk-000" / f"observation.images.{cam_key}" / f"episode_{ep_idx:06d}.mp4"
            shutil.copy2(src, dst)
            wh = video_resolution(src)
            if cam_key == "dabai":
                seen_dabai = True
                if wh: dabai_wh = wh
            else:
                seen_top = True
                if wh: top_wh = wh

        episodes_meta.append({
            "episode_index": ep_idx,
            "tasks": [TASK_DESCRIPTION],
            "length": len(rows),
        })
        total_frames += len(rows)
        print(f"  ep{ep_idx:03d} ← {jpath.name}: {len(rows)} frames")

    # meta/tasks.jsonl
    with (out_dir / "meta" / "tasks.jsonl").open("w") as f:
        f.write(json.dumps({"task_index": 0, "task": TASK_DESCRIPTION}) + "\n")

    # meta/episodes.jsonl
    with (out_dir / "meta" / "episodes.jsonl").open("w") as f:
        for e in episodes_meta:
            f.write(json.dumps(e) + "\n")

    # meta/info.json (LeRobot v2 schema)
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": [7],
            "names": ["j1", "j2", "j3", "j4", "j5", "j6", "gripper"],
        },
        "action": {
            "dtype": "float32",
            "shape": [7],
            "names": ["j1", "j2", "j3", "j4", "j5", "j6", "gripper"],
        },
        "timestamp": {"dtype": "float32", "shape": [1]},
        "frame_index": {"dtype": "int64", "shape": [1]},
        "episode_index": {"dtype": "int64", "shape": [1]},
        "task_index": {"dtype": "int64", "shape": [1]},
    }
    if seen_dabai:
        w, h = dabai_wh or (1280, 720)
        features["observation.images.dabai"] = {
            "dtype": "video",
            "shape": [h, w, 3],
            "names": ["height", "width", "channel"],
            "video_info": {"video.fps": DEFAULT_FPS, "video.codec": "h264", "video.pix_fmt": "yuv420p"},
        }
    if seen_top:
        w, h = top_wh or (1920, 1080)
        features["observation.images.top"] = {
            "dtype": "video",
            "shape": [h, w, 3],
            "names": ["height", "width", "channel"],
            "video_info": {"video.fps": DEFAULT_FPS, "video.codec": "h264", "video.pix_fmt": "yuv420p"},
        }

    info = {
        "codebase_version": "v2.0",
        "robot_type": ROBOT_TYPE,
        "total_episodes": len(episodes_meta),
        "total_frames": total_frames,
        "total_tasks": 1,
        "total_videos": (1 if seen_dabai else 0) * len(episodes_meta)
                       + (1 if seen_top else 0) * len(episodes_meta),
        "total_chunks": 1,
        "chunks_size": 1000,
        "fps": DEFAULT_FPS,
        "splits": {"train": f"0:{len(episodes_meta)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": features,
        "_jengabot_notes": (
            "action == observation.state (leader-command proxy). Run "
            "follower_play with --observed-log to capture real observation."
        ),
    }
    with (out_dir / "meta" / "info.json").open("w") as f:
        json.dump(info, f, indent=2)

    print(f"\nwrote {len(episodes_meta)} episodes, {total_frames} frames -> {out_dir}")
    print(f"  dabai videos present: {seen_dabai}   top videos present: {seen_top}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--episodes", type=Path, default=REPO_ROOT / "episodes",
                    help="folder of episode_*.jsonl + mp4s")
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "dataset",
                    help="output LeRobot v2 dataset folder (wiped + recreated)")
    args = ap.parse_args()
    build(args.episodes, args.out)


if __name__ == "__main__":
    main()
