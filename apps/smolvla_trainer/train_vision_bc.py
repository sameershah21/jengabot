#!/usr/bin/env python3
"""Vision-conditioned BC trainer ("VLA-style" baseline).

Input  : top-camera frame (3xHxW) + state (7-vec)
Output : action (7-vec)
Model  : small CNN encoder + concat(state) -> MLP head
Loss   : MSE

This is the fallback / second deliverable if the real SmolVLA fine-tune
in train_smolvla.py is too slow on a Mac. It runs on MPS in a few minutes
and produces a real .pt checkpoint that consumes images at inference.

Usage:
  .venv/bin/python train_vision_bc.py
  .venv/bin/python train_vision_bc.py --epochs 30 --img-size 96
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import av
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset, random_split

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATASET = REPO_ROOT / "dataset"
MODELS_DIR = Path(__file__).resolve().parent / "models"
CAM_KEY = "top"  # one of: "top", "dabai"


def decode_video_frames(video_path: Path, n_frames_target: int, img_size: int) -> np.ndarray:
    """Decode a video into n_frames_target evenly-sampled RGB frames at img_size."""
    container = av.open(str(video_path))
    stream = container.streams.video[0]
    total = stream.frames or 0
    frames = []
    for frame in container.decode(video=0):
        rgb = frame.to_ndarray(format="rgb24")
        frames.append(rgb)
    container.close()
    if not frames:
        raise RuntimeError(f"no frames decoded from {video_path}")
    # Sample n_frames_target evenly
    src = np.array(frames)  # (T, H, W, 3)
    T = len(src)
    if n_frames_target <= T:
        idx = np.linspace(0, T - 1, n_frames_target).astype(int)
    else:
        idx = np.linspace(0, T - 1, n_frames_target).astype(int)
    sampled = src[idx]
    # Resize (cheap nearest via stride) to img_size x img_size
    out = []
    for f in sampled:
        h, w, _ = f.shape
        # center crop to square then nearest-resample
        s = min(h, w)
        f2 = f[(h - s) // 2:(h - s) // 2 + s, (w - s) // 2:(w - s) // 2 + s]
        # Simple resize via numpy stride (PIL/torchvision optional)
        from PIL import Image
        img = Image.fromarray(f2).resize((img_size, img_size), Image.BILINEAR)
        out.append(np.asarray(img))
    return np.stack(out, axis=0)  # (N, H, W, 3) uint8


class VisionTeleopDataset(Dataset):
    def __init__(self, dataset_dir: Path, img_size: int = 96, cam_key: str = CAM_KEY):
        data_dir = dataset_dir / "data" / "chunk-000"
        vid_dir = dataset_dir / "videos" / "chunk-000" / f"observation.images.{cam_key}"
        parquets = sorted(data_dir.glob("episode_*.parquet"))
        if not parquets:
            raise FileNotFoundError(f"no parquet in {data_dir}")

        all_states, all_actions, all_imgs = [], [], []
        kept = 0
        for p in parquets:
            ep = int(p.stem.split("_")[-1])
            vid = vid_dir / f"episode_{ep:06d}.mp4"
            if not vid.exists():
                print(f"  skip ep{ep:03d}: no {cam_key} video")
                continue
            df = pd.read_parquet(p)
            n = len(df)
            frames = decode_video_frames(vid, n, img_size)  # (n, H, W, 3) uint8
            states = np.array([list(s) for s in df["observation.state"]], dtype=np.float32)
            actions = np.array([list(a) for a in df["action"]], dtype=np.float32)
            all_states.append(states)
            all_actions.append(actions)
            all_imgs.append(frames)
            kept += 1
            print(f"  ep{ep:03d}: {n} frames, video decoded -> {frames.shape}")

        if not all_states:
            raise RuntimeError(f"no episodes with both parquet + {cam_key} video found")

        self.states = torch.from_numpy(np.concatenate(all_states, axis=0))
        self.actions = torch.from_numpy(np.concatenate(all_actions, axis=0))
        imgs = np.concatenate(all_imgs, axis=0)  # (N, H, W, 3)
        # to (N, 3, H, W) float in [0, 1]
        self.imgs = torch.from_numpy(imgs).permute(0, 3, 1, 2).float() / 255.0
        self.dim = self.states.shape[1]
        self.img_size = img_size
        print(f"loaded {len(self.states)} frames from {kept} episodes, "
              f"state dim={self.dim}, img={img_size}x{img_size}")

    def __len__(self):
        return len(self.states)

    def __getitem__(self, idx):
        return self.imgs[idx], self.states[idx], self.actions[idx]


class VisionBC(nn.Module):
    """Tiny CNN -> flatten -> concat state -> MLP -> action."""

    def __init__(self, state_dim: int, img_size: int = 96, hidden: int = 256):
        super().__init__()
        # CNN: 3 -> 16 -> 32 -> 64, each /2
        self.conv = nn.Sequential(
            nn.Conv2d(3, 16, 5, stride=2, padding=2), nn.ReLU(inplace=True),
            nn.Conv2d(16, 32, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 4)),  # -> (64, 4, 4) = 1024 feats
            nn.Flatten(),
        )
        feat = 64 * 4 * 4
        self.head = nn.Sequential(
            nn.Linear(feat + state_dim, hidden), nn.ReLU(inplace=True),
            nn.Linear(hidden, hidden), nn.ReLU(inplace=True),
            nn.Linear(hidden, state_dim),
        )
        nn.init.zeros_(self.head[-1].weight)
        nn.init.zeros_(self.head[-1].bias)

    def forward(self, img, state):
        z = self.conv(img)
        return self.head(torch.cat([z, state], dim=-1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--hidden", type=int, default=256)
    ap.add_argument("--img-size", type=int, default=96)
    ap.add_argument("--cam-key", default=CAM_KEY, choices=["top", "dabai"])
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--val-split", type=float, default=0.1)
    ap.add_argument("--device", default="auto")
    args = ap.parse_args()

    if args.device == "auto":
        device = torch.device("mps" if torch.backends.mps.is_available()
                              else "cuda" if torch.cuda.is_available()
                              else "cpu")
    else:
        device = torch.device(args.device)
    print(f"device: {device}")

    ds = VisionTeleopDataset(args.dataset, img_size=args.img_size, cam_key=args.cam_key)
    n_val = max(1, int(len(ds) * args.val_split))
    n_train = len(ds) - n_val
    tr_ds, vl_ds = random_split(ds, [n_train, n_val],
                                 generator=torch.Generator().manual_seed(0))
    print(f"train={n_train} val={n_val}")
    tr_dl = DataLoader(tr_ds, batch_size=args.batch, shuffle=True)
    vl_dl = DataLoader(vl_ds, batch_size=args.batch)

    policy = VisionBC(ds.dim, img_size=args.img_size, hidden=args.hidden).to(device)
    n_params = sum(p.numel() for p in policy.parameters())
    print(f"policy: VisionBC, hidden={args.hidden}, params={n_params:,}")

    opt = torch.optim.AdamW(policy.parameters(), lr=args.lr, weight_decay=1e-4)
    loss_fn = nn.MSELoss()

    t0 = time.time()
    history = []
    for epoch in range(args.epochs):
        policy.train()
        tr = 0.0; n = 0
        for img, s, a in tr_dl:
            img = img.to(device); s = s.to(device); a = a.to(device)
            opt.zero_grad()
            p = policy(img, s)
            l = loss_fn(p, a)
            l.backward()
            opt.step()
            tr += l.item() * len(s); n += len(s)
        tr /= n

        policy.eval()
        vl = 0.0; nv = 0
        with torch.no_grad():
            for img, s, a in vl_dl:
                img = img.to(device); s = s.to(device); a = a.to(device)
                p = policy(img, s)
                vl += loss_fn(p, a).item() * len(s); nv += len(s)
        vl /= max(nv, 1)
        history.append({"epoch": epoch, "train_loss": tr, "val_loss": vl})
        print(f"ep {epoch:3d}  train={tr:.5f}  val={vl:.5f}")

    dt = time.time() - t0
    final_val = history[-1]["val_loss"]
    print(f"\ntrained in {dt:.1f}s, final val loss = {final_val:.5f}")

    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    full_path = MODELS_DIR / f"vision_bc_{stamp}.pt"
    meta_path = MODELS_DIR / f"vision_bc_{stamp}_meta.json"

    torch.save({
        "state_dict": policy.state_dict(),
        "state_dim": ds.dim,
        "img_size": args.img_size,
        "hidden": args.hidden,
        "cam_key": args.cam_key,
        "num_params": n_params,
        "final_val_loss": final_val,
    }, full_path)
    print(f"saved -> {full_path}")

    meta = {
        "stamp": stamp,
        "model": "VisionBC (CNN+state -> action MLP)",
        "dataset": str(args.dataset),
        "cam_key": args.cam_key,
        "epochs": args.epochs,
        "batch": args.batch,
        "hidden": args.hidden,
        "img_size": args.img_size,
        "lr": args.lr,
        "device": str(device),
        "params": n_params,
        "train_frames": n_train,
        "val_frames": n_val,
        "final_train_loss": history[-1]["train_loss"],
        "final_val_loss": final_val,
        "elapsed_sec": round(dt, 2),
        "checkpoint": str(full_path),
    }
    with meta_path.open("w") as f:
        json.dump(meta, f, indent=2)
    print(f"saved meta -> {meta_path}")


if __name__ == "__main__":
    main()
