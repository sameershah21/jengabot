#!/usr/bin/env python3
"""Behavioral-cloning trainer for the jengabot LeRobot dataset.

Input:  observation.state  (7-vec: 6 joint angles + gripper)
Output: action             (7-vec: 6 joint commands + gripper)
Loss:   MSE
Model:  small 3-layer MLP (~50K params).

This is *not* full ACT — no action chunking, no transformer attention, no
vision input. It's a behavioral-cloning baseline that exercises the same
data pipeline an ACT trainer would use and produces a deployable .pt
checkpoint. With the leader-as-state-proxy data, it essentially learns
the identity map; once we have real follower observation logs from
follower_play --observed-log, the same trainer will learn a non-trivial
controller.

Output:
  models/bc_<timestamp>.pt           full state_dict + meta
  models/bc_<timestamp>_int8.pt      dynamic-quantized variant
  models/bc_<timestamp>_meta.json    hparams + final loss

Run:
  .venv/bin/python train_bc.py
  .venv/bin/python train_bc.py --epochs 200 --hidden 256
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset, random_split


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATASET = REPO_ROOT / "dataset"
MODELS_DIR = Path(__file__).resolve().parent / "models"


class TeleopDataset(Dataset):
    def __init__(self, dataset_dir: Path):
        data_dir = dataset_dir / "data" / "chunk-000"
        parquets = sorted(data_dir.glob("episode_*.parquet"))
        if not parquets:
            raise FileNotFoundError(f"no parquet files in {data_dir}")
        dfs = [pd.read_parquet(p) for p in parquets]
        df = pd.concat(dfs, ignore_index=True)
        self.states = torch.tensor(
            [list(s) for s in df["observation.state"]], dtype=torch.float32
        )
        self.actions = torch.tensor(
            [list(a) for a in df["action"]], dtype=torch.float32
        )
        assert self.states.shape == self.actions.shape, \
            f"state {self.states.shape} != action {self.actions.shape}"
        self.dim = self.states.shape[1]
        print(f"loaded {len(self.states)} frames from {len(parquets)} episodes, "
              f"dim={self.dim}")

    def __len__(self):
        return len(self.states)

    def __getitem__(self, idx):
        return self.states[idx], self.actions[idx]


class BCPolicy(nn.Module):
    def __init__(self, dim: int, hidden: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(dim, hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, dim),
        )
        # initialize last layer near zero so untrained policy stays still
        nn.init.zeros_(self.net[-1].weight)
        nn.init.zeros_(self.net[-1].bias)

    def forward(self, x):
        return self.net(x)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--hidden", type=int, default=128)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--val-split", type=float, default=0.1)
    ap.add_argument("--device", default="auto",
                    help="auto | mps | cpu (cuda probably not on Mac)")
    args = ap.parse_args()

    if args.device == "auto":
        if torch.backends.mps.is_available():
            device = torch.device("mps")
        elif torch.cuda.is_available():
            device = torch.device("cuda")
        else:
            device = torch.device("cpu")
    else:
        device = torch.device(args.device)
    print(f"device: {device}")

    ds = TeleopDataset(args.dataset)
    n_val = max(1, int(len(ds) * args.val_split))
    n_train = len(ds) - n_val
    train_ds, val_ds = random_split(
        ds, [n_train, n_val], generator=torch.Generator().manual_seed(0)
    )
    print(f"train={n_train} val={n_val}")

    train_dl = DataLoader(train_ds, batch_size=args.batch, shuffle=True)
    val_dl = DataLoader(val_ds, batch_size=args.batch)

    policy = BCPolicy(ds.dim, hidden=args.hidden).to(device)
    n_params = sum(p.numel() for p in policy.parameters())
    print(f"policy: 3-layer MLP, hidden={args.hidden}, params={n_params:,}")

    opt = torch.optim.AdamW(policy.parameters(), lr=args.lr, weight_decay=1e-4)
    loss_fn = nn.MSELoss()

    t0 = time.time()
    history = []
    for epoch in range(args.epochs):
        policy.train()
        tr = 0.0
        n = 0
        for s, a in train_dl:
            s = s.to(device); a = a.to(device)
            opt.zero_grad()
            p = policy(s)
            l = loss_fn(p, a)
            l.backward()
            opt.step()
            tr += l.item() * len(s); n += len(s)
        tr /= n

        policy.eval()
        vl = 0.0; nv = 0
        with torch.no_grad():
            for s, a in val_dl:
                s = s.to(device); a = a.to(device)
                p = policy(s)
                vl += loss_fn(p, a).item() * len(s); nv += len(s)
        vl /= max(nv, 1)
        history.append({"epoch": epoch, "train_loss": tr, "val_loss": vl})
        if epoch % 10 == 0 or epoch == args.epochs - 1:
            print(f"ep {epoch:4d}  train={tr:.5f}  val={vl:.5f}")

    dt = time.time() - t0
    final_val = history[-1]["val_loss"]
    print(f"\ntrained in {dt:.1f}s, final val loss = {final_val:.5f}")

    # Save full + quantized
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    full_path = MODELS_DIR / f"bc_{stamp}.pt"
    meta_path = MODELS_DIR / f"bc_{stamp}_meta.json"

    torch.save({
        "state_dict": policy.state_dict(),
        "dim": ds.dim,
        "hidden": args.hidden,
        "num_params": n_params,
        "final_val_loss": final_val,
    }, full_path)
    print(f"saved full → {full_path}")

    # Dynamic int8 quantization (CPU model). Move policy to cpu first.
    cpu_policy = BCPolicy(ds.dim, hidden=args.hidden)
    cpu_policy.load_state_dict(policy.state_dict())
    cpu_policy.eval()
    try:
        q_policy = torch.quantization.quantize_dynamic(
            cpu_policy, {nn.Linear}, dtype=torch.qint8
        )
        q_path = MODELS_DIR / f"bc_{stamp}_int8.pt"
        torch.save(q_policy.state_dict(), q_path)
        q_size = q_path.stat().st_size
        print(f"saved int8 → {q_path}  ({q_size/1024:.1f} KB)")
    except Exception as e:
        print(f"quantization skipped: {e}")
        q_path = None

    meta = {
        "stamp": stamp,
        "dataset": str(args.dataset),
        "epochs": args.epochs,
        "batch": args.batch,
        "hidden": args.hidden,
        "lr": args.lr,
        "device": str(device),
        "params": n_params,
        "train_frames": n_train,
        "val_frames": n_val,
        "final_train_loss": history[-1]["train_loss"],
        "final_val_loss": final_val,
        "elapsed_sec": round(dt, 2),
        "checkpoint": str(full_path),
        "checkpoint_int8": str(q_path) if q_path else None,
    }
    with meta_path.open("w") as f:
        json.dump(meta, f, indent=2)
    print(f"saved meta → {meta_path}")


if __name__ == "__main__":
    main()
