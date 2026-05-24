# piper-sdk-rs patches

Three example binaries we wrote for the JengaBot hackathon, against
[vivym/piper-sdk-rs](https://github.com/vivym/piper-sdk-rs). The upstream
SDK clone itself is gitignored; this folder is the durable copy.

## Files

| File | What it does |
|---|---|
| `exit_teach_mode.rs` | Raw GS-USB sender. Writes a CAN 0x150 frame with `teach_command=EndRecord (0x02)` to take the arm out of drag-teach mode. Needed because teach mode is non-volatile (survives power cycle) and `enable_position_mode` silently times out while the arm is in teach. |
| `position_control_demo.rs` | Patched upstream example: extended startup timeouts (5 s firmware / 10 s feedback / 15 s disable & enable), added a Maintenance → Standby transition, switched serial→`gs_usb_auto`, and **streams position commands at 50 Hz instead of single-shot** (the single-shot version reported "complete" but the arm never moved). |
| `joint_sweep.rs` | New example. Exercises each of J1–J6 individually through ±20° while holding the other joints at the home pose. 50 Hz streaming, ~3 s per segment. |
| `gripper_test.rs` | New example. Streams `set_gripper` at 50 Hz through OPEN → CLOSE → HALF → CLOSE-HARD → OPEN. Observed: command `1.0` saturates at gripper position ≈ 0.696, command `0.5` lands at 0.498 — so the upper end of the SDK's `[0,1]` doesn't quite reach mechanical full-open. |
| `record_pose.rs` | Connect → Standby (motors off) → drag the arm by hand → type pose name + Enter to append current joint angles to `poses.txt`. `q` + Enter to quit. |
| `play_poses.rs` | Reads `poses.txt` and streams each pose at 50 Hz for `--segment-secs` (default 4 s). `--sequence name_a,name_b,...` to play a subset in order. |
| `frame_scan.rs` | Read-only raw CAN frame dump (200+ frames). Prints unique CAN IDs with hit counts + sample data. Use to diagnose firmware protocol shifts. |
| `feedback_check.rs` | Read-only joint position observer for N seconds. Never enables/disables motors, so dropping the handle won't trigger auto-disable. Use to verify protocol-ID patches before any motion test. |
| `leader_stream.rs` | Read-only joint angle streamer for bilateral teleop. Operator drags the arm by hand in drag-teach mode; this binary emits JSON lines `{"t_us": ..., "joints_deg": [...]}` at 50 Hz default. Optional `--out file.jsonl` for replay. Runs on **Raymond** (the leader). |
| `follower_play.rs` | Companion to `leader_stream`. Reads JSON lines from stdin and streams them to the connected arm as 50 Hz position commands. Per-tick joint step clamped (`--max-step-deg`, default 2°), per-joint limits enforced (from manual), watchdog holds last pose on stdin silence. Runs on **Bruce** (the follower). `--hold-seconds N` smoke-tests the follower path alone (no input). |

## Firmware patches

| File | What it does |
|---|---|
| `firmware-1.8-3-id-shift.patch` | **PiPER-X firmware S-V1.8-3 fix.** Shifts cold-feedback CAN IDs by +0xFF: `0x2A1`→`0x3A0` (ROBOT_STATUS), `0x2A2-4`→`0x3A1-3` (END_POSE), `0x2A5-7`→`0x3A4-6` (JOINT_FEEDBACK), `0x2A8`→`0x3A7` (GRIPPER). Hot data (`0x251-6`) and low-speed (`0x261-6`) unchanged. Apply with `git apply` inside the piper-sdk-rs checkout. Diagnosed via `frame_scan` reading raw 1.8-3 frames; verified via `feedback_check`. |
| `firmware-1.8-3-yolo.patch` | **DANGEROUS — only use as fallback.** Bypasses the SDK's mode-confirmation / enabled / disabled / freshness checks. Was used as a stop-gap before the real ID fix was found. Caused an arm-drop incident because joint-position reads still failed after motors were enabled. Prefer `firmware-1.8-3-id-shift.patch`. |

## Apply / build

```bash
# Clone upstream (gitignored in this repo)
cd /Users/sameershah/learn/github/jengabot
git clone https://github.com/vivym/piper-sdk-rs.git

# Drop our files over the upstream examples
cp examples/piper-sdk-rs-patches/*.rs piper-sdk-rs/crates/piper-sdk/examples/

# Build (Rust ≥ 1.95, brew install libusb)
cd piper-sdk-rs
cargo build -p piper-sdk --example exit_teach_mode
cargo build -p piper-sdk --example position_control_demo
cargo build -p piper-sdk --example joint_sweep
```

## Run (macOS)

```bash
# If arm is stuck in drag-teach (control_mode=2), run this first:
sudo ./target/debug/examples/exit_teach_mode
# unplug + replug candleLight before the next command

sudo ./target/debug/examples/position_control_demo --interface auto
# or
sudo ./target/debug/examples/joint_sweep --amplitude-deg 20 --segment-secs 3
```

A sudoers drop-in lets these run without password prompts:

```
$(whoami) ALL=(root) NOPASSWD: /Users/sameershah/learn/github/jengabot/piper-sdk-rs/target/debug/examples/*
```

## Known quirks

- The candleLight dongle frequently needs an unplug/replug between runs.
  Each failed/partial SDK init can leave the USB stack in a state where the
  next run errors at `Failed to set bitrate` or `Infrastructure(Timeout)`.
- `position_control_demo`'s upstream comment claims single-shot
  `send_position_command` matches the Python SDK — it doesn't, at least on
  PiPER-X firmware S-V1.8-2. The streaming patch is required.
- `exit_teach_mode` returns "send timeout" after a few frames. The first 2–3
  reliably make it to the arm, which is enough to clear teach mode. Treat
  the timeout as informational.
