# smolvla_trainer

Fine-tune Hugging Face's **SmolVLA** (450M-param VLA) on the JengaBot
teleop dataset, on an Apple Silicon Mac, using `lerobot[smolvla]` v3.0.

Also ships a smaller "VLA-style" vision-conditioned behavioral-cloning
baseline (`train_vision_bc.py`) that runs in seconds on MPS — useful
both as a sanity test and as a deployable artifact when GPU isn't an
option.

## What's in here

| File                       | What it does                                                                                       |
|----------------------------|----------------------------------------------------------------------------------------------------|
| `build_v3_dataset.py`      | Convert raw `episodes/*.jsonl + mp4` into a **LeRobot v3.0** dataset (uses official LeRobotDataset.create()) |
| `train_vision_bc.py`       | Train a small CNN + state -> action MLP on the v2 parquet dataset. Saves `models/vision_bc_*.pt`   |
| `models/vision_bc_*.pt`    | Trained vision-BC checkpoint (1.4 MB)                                                              |
| `runs/smolvla_real/checkpoints/000200/pretrained_model/` | The real fine-tuned SmolVLA inference checkpoint. **Committed via git-lfs** (865 MB) and mirrored to https://huggingface.co/pilarclark/jengabot-smolvla-jenga |

## Reproducing the SmolVLA fine-tune

```bash
# 1. Set up venv (Python 3.11, uv recommended)
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python 'lerobot[smolvla]' av pillow

# 2. Rebuild the v3 dataset from raw episodes (only the 3 with both cams)
.venv/bin/python build_v3_dataset.py

# 3. Fine-tune SmolVLA from the published base checkpoint
.venv/bin/lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --policy.device=mps \
  --policy.push_to_hub=false \
  --policy.repo_id=jengabot/smolvla_jenga \
  --dataset.repo_id=jengabot/teleop \
  --dataset.root=../../dataset_v3 \
  --batch_size=2 --steps=200 --save_freq=200 --log_freq=10 \
  --output_dir=./runs/smolvla_real \
  --wandb.enable=false \
  --rename_map='{"observation.images.top":"observation.images.camera1","observation.images.dabai":"observation.images.camera2"}'
```

## Result of our run

- Model: `lerobot/smolvla_base` (450M params, ~100M learnable, VLM
  backbone `HuggingFaceTB/SmolVLM2-500M-Video-Instruct`)
- Device: Apple Silicon MPS
- Dataset: 3 episodes, 566 frames, 2 cameras (top + dabai), 7-DoF state/action
- Steps: 200 @ batch_size=2 (cosine schedule, peak LR 1e-4)
- Time: **85 seconds** end-to-end (after one-time SmolVLM weight download)
- Loss trace (every 10 steps), see `runs/smolvla_real/loss_curve.txt`:

  | step |  loss |
  |-----:|------:|
  |   10 | 0.403 |
  |   50 | 0.197 |
  |  100 | 0.162 |
  |  150 | 0.130 |
  |  200 | 0.120 |

The checkpoint is real and loadable via
`SmolVLAPolicy.from_pretrained("runs/smolvla_real/checkpoints/000200/pretrained_model")`.

## Caveats

- Only 3 episodes / 566 frames — far below the recommended 30–500
  episodes for a SmolVLA fine-tune. Treat this as a **proof of training
  loop**, not a deployable autonomous policy.
- `action == observation.state` in the dataset (the leader-command proxy).
  Re-record episodes with `follower_play --observed-log` to capture real
  Bruce state, then re-build the v3 dataset and re-train for a non-trivial controller.
- The pretrained model expects 3 cameras (`camera1/2/3`); we have 2.
  `--rename_map` maps our `top->camera1` and `dabai->camera2`; `camera3`
  is silently dropped by lerobot. Adding a third view (e.g., depth as RGB)
  would let the model use the full 3-cam tower it was pretrained with.
- The 865 MB safetensors checkpoint is committed via **git-lfs**
  (`.gitattributes` tracks `*.safetensors`). You'll need `git-lfs`
  installed locally for `git clone` to pull the weights. The same
  checkpoint is also published as a Hugging Face model repo at
  https://huggingface.co/pilarclark/jengabot-smolvla-jenga and can be
  loaded directly with
  `SmolVLAPolicy.from_pretrained("pilarclark/jengabot-smolvla-jenga")`.
- The 394 MB optimizer state (`checkpoints/000200/training_state/`) is
  gitignored — only useful for resuming training, not inference.

## Vision-BC fallback

If installing the full `lerobot[smolvla]` stack is too heavy (it pulls
torch, transformers, torchcodec, wandb, etc.), `train_vision_bc.py` is
a 1.4 MB, ~13k-param model that does the same shape job (image + state
-> action) and trains in ~10 seconds on MPS:

```bash
.venv/bin/python train_vision_bc.py --epochs 120 --img-size 96 --batch 32
# saves models/vision_bc_<timestamp>.pt
```
