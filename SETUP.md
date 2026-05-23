# JengaBot — Setup & Hackathon Plan

Working notes for the JengaBot build. Captures the hardware/software path, the rail mechanism, and the phased demo plan.

## Hardware

- **Arm:** AgileX PiPER-X (6-DoF, ~625 mm reach, 1.5 kg payload)
- **CAN adapter:** candleLight USB-to-CAN dongle (VID `0x1D50` / PID `0x606F`, bytewerk firmware = gs_usb protocol)
- **Camera:** icspring USB camera (visible on host — earmarked for π₀.₅ observation later)
- **Host (Phase 1):** macOS (Apple Silicon)
- **Host (Phase 2):** Linux + NVIDIA GPU on Alibaba Cloud

## The rail mechanism (Sameer's sketch — `IMG_4257.HEIC`)

Two stations, both built from parallel rails ("train tracks"):

- **FROM station** (top half of sketch) — source. Blocks are **pre-sorted by orientation** and **laid out flat** on the rails. The arm reaches to known slot positions to pick.
- **TO station** (bottom half of sketch) — destination. Parallel rails act as the **foundation only**: the first layer rests across them like railroad ties; subsequent layers stack directly on prior blocks.

### PoC progression

| Phase | Blocks/layer | Rotation | Layers | Notes |
|---|---|---|---|---|
| 1 | 2 | none (all same orientation) | 3–4 | Proves pick + place + vertical stack. Tower is rectangular — skipping rotation avoids a layer-misalignment problem that doesn't exist in real Jenga. |
| 2 | 3 | 90° per layer | as many as reach allows | Real Jenga geometry — 7.5 cm × 7.5 cm square layers stack cleanly. |

Standard Jenga block dimensions: **1.5 cm × 2.5 cm × 7.5 cm** (3:1:5 ratio), 54 blocks total.

## Software path

### Why not the official `agilexrobotics/piper_sdk`?

- `setup.py` declares `platforms=['Linux']`.
- Default CAN backend is `socketcan` — Linux kernel feature, **does not exist on macOS**.
- Alternate `slcan` backend (see `demo/V2/piper_set_can.py`) needs candleLight reflashed to slcan firmware — risky.
- [Issue #24](https://github.com/agilexrobotics/piper_sdk/issues/24) confirms no working macOS path; Docker workaround also failed for users.

### What we use instead: [`vivym/piper-sdk-rs`](https://github.com/vivym/piper-sdk-rs)

A community Rust SDK with cross-platform CAN access via `rusb` (libusb). Explicitly supports macOS gs_usb adapters with `.gs_usb_auto()` / `.gs_usb_serial(...)`.

Tradeoff: Rust, not Python. Acceptable for Phase 1 (waypoint scripting / record-replay). Phase 2 (π₀.₅ VLA training & inference) runs on Linux+GPU regardless, so the Python ↔ Rust split aligns with the hardware split.

## Reproducing the macOS bring-up

Prereqs (already present on Sameer's machine):

```bash
brew install libusb           # 1.0.30
rustup default stable         # rustc ≥ 1.95 (edition 2024 + let chains in if)
```

Clone and build the smoke-test example only (avoids pulling the MuJoCo addon):

```bash
cd ~/learn/github/jengabot
git clone https://github.com/vivym/piper-sdk-rs.git
cd piper-sdk-rs
cargo build -p piper-sdk --example gs_usb_direct_test
```

Run it (needs `sudo` on macOS — libusb has to claim the USB interface):

```bash
sudo ./target/debug/examples/gs_usb_direct_test
```

### Expected output (Piper alive)

You should see 20 CAN frames decoded with IDs in these ranges:

| CAN ID | Meaning (Piper protocol) |
|---|---|
| `0x2A1`–`0x2A6` | Joint 1–6 position feedback |
| `0x2A7`, `0x2A8` | End-effector / gripper state |
| `0x251`–`0x256` | Joint motor diagnostics |
| `0x265` | Arm status word |

Confirmed working **2026-05-22** on Apple Silicon + candleLight + PiPER-X. Frames stream at ~120 Hz.

## Other useful examples in `piper-sdk-rs`

State / monitoring:
- `state_api_demo` — clean parsed joint angles + end-effector pose (read-only)
- `robot_monitor` — continuous state monitoring
- `frame_dump` — raw CAN frame logger

Motion (write-path — clear the arm's workspace first):
- `high_level_simple_move` — minimal commanded motion
- `position_control_demo` — full joint-position control loop
- `high_level_trajectory_demo` — multi-waypoint trajectory
- `high_level_gripper_control` — gripper open/close

Record / replay (used by `apps/cli/piper-cli`):
- `standard_recording` — capture CAN frames to file
- `replay_mode` — play back a recording

On macOS, examples that take `--interface` expect a **GS-USB serial number** rather than `can0`:

```bash
cargo run -p piper-sdk --example state_api_demo -- --interface <serial>
```

(Linux equivalent: `--interface can0`.)

## Phase plan

### Phase 1 — Working demo on Mac (Path B, hand-scripted)

Stations have known geometry, so no perception is needed. The arm just needs a list of (FROM slot → TO slot) pose pairs.

Tasks (see project task list):
1. Lock rail coordinate system on paper (rail spacing, slot pitch, slot count, Piper base origin, station placement inside ~625 mm reach).
2. ~~Verify connectivity~~ — done 2026-05-22, see above.
3. Read joint state via `state_api_demo` (parsed values, not raw frames).
4. First commanded motion via `high_level_simple_move`.
5. Single-block pick from FROM.
6. Single-block place on TO foundation.
7. Chain into 2-block × 3–4 layer tower.

### Phase 2 — VLA on Linux+GPU (Path A, learned policy)

Hardware moves to the Ubuntu box on Alibaba Cloud (or wherever the GPU lives).

1. Install [`agilexrobotics/data_tools`](https://github.com/agilexrobotics/data_tools) for teleop episode recording.
2. Collect N teleop demos of pick-from-FROM → place-on-TO.
3. Convert to LeRobot dataset format.
4. LoRA fine-tune π₀.₅ via [`agilexrobotics/openpi-agilex`](https://github.com/agilexrobotics/openpi-agilex) (LoRA fits in ≥22.5 GB VRAM — RTX 4090 is enough).
5. Spin up a policy server, swap into the demo loop.

GPU requirements (from openpi-agilex README):

| Mode | VRAM | Example GPU |
|---|---|---|
| Inference | > 8 GB | RTX 4090 |
| Fine-tuning (LoRA) | > 22.5 GB | RTX 4090 |
| Fine-tuning (full) | > 70 GB | A100 80 GB / H100 |

## Open items before the demo

- Rail coordinate measurements (waiting on physical rails).
- Gripper choice / mounting on the Piper — single-block grasp geometry not yet verified.
- USB permissions on macOS: every run currently needs `sudo`. Acceptable for hackathon, look into a launchd / IOKit entitlement later if it becomes friction.

## Team

Sameer · Volk · Bruce · Jonathan
