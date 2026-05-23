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
