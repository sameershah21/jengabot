# JengaBot bring-up — get PiPER-X arms moving on macOS

Stand-alone guide for someone joining the project. Skipping straight to "what
do I install, what do I run, what breaks" instead of recapping the whole
debugging journey. The full story is in `SETUP.md` and the branch commits.

> Verified end-to-end on macOS (Apple Silicon, darwin 25.5) + bytewerk
> candleLight USB-CAN dongle + AgileX PiPER-X on firmware **S-V1.8-2** and
> **S-V1.8-3** (each needs slightly different SDK state — see step 4).

## The arms

We have two PiPER-X arms in this project, with names everyone uses:

| Arm | Role | Notes |
|---|---|---|
| **Raymond's arm** | **Leader** (joining shortly) | The operator drags this one by hand. Firmware unknown until plugged in — run `frame_scan` against it to confirm and decide whether the ID-shift patch applies. |
| **Bruce's arm** | **Follower** (currently the only one connected) | Originally on firmware S-V1.8-2; got updated to S-V1.8-3 mid-project. Has been the workhorse for every example we built — `joint_sweep`, `gripper_test`, `record_pose`, etc. Now also the follower in bilateral teleop. |

Each arm needs its **own candleLight dongle and its own CAN bus**. Don't try
to chain them on one bus — the SDK driver claims the gs_usb interface
exclusively, so two arms = two USB devices.

Most of this doc covers single-arm bring-up (using Bruce's arm). The
leader/follower section near the end covers the two-arm pipe
(Raymond → Bruce).

---

## 1. Hardware checklist

You need all of this physically on the desk before software helps:

- AgileX **PiPER-X** robotic arm (or PiPER — same SDK, same CAN protocol)
- **24 V power supply** for the arm (came with it). Min 24 V, max 26 V, ≥ 10 A.
- **candleLight USB-to-CAN adapter** (bytewerk, VID `0x1D50` / PID `0x606F`).
  This is what plugs into your Mac. Don't try anything else on macOS — only
  candleLight has a cross-platform userspace driver that works without
  reflashing.
- The arm's **power + CAN aviation plug** cable (came in the box).
- A working **USB cable** between your Mac and the dongle. Some cables are
  charge-only — if the dongle's LED stays dark, swap the cable.
- A **second PiPER-X + second dongle** if you're doing leader/follower teleop
  (see end of this doc).

Optional but useful:
- A **two-finger gripper** for the arm flange (also CAN-driven).
- An **icSpring** or similar USB camera if you'll do VLA work later.

---

## 2. macOS software prereqs

Everything except Rust is one-liners.

```bash
# Homebrew (skip if installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# libusb (gs_usb backend uses this; no kernel driver on macOS)
brew install libusb

# Rust toolchain via rustup
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
# Make sure stable is ≥ 1.95 (edition-2024, let-chains in if-conditions)
rustup default stable
rustup update stable
rustc --version  # should print 1.95+

# GitHub CLI (optional; needed if you'll clone private branches via gh)
brew install gh
gh auth login
```

Linux works too (Ubuntu 22.04 tested). On Linux the dongle is reachable via
`SocketCAN` after `sudo modprobe gs_usb && sudo ip link set can0 up type can
bitrate 1000000`. Linux examples use `--interface can0` instead of `--interface
auto`. Everything below assumes macOS.

---

## 3. Clone the repos

`vivym/piper-sdk-rs` is our hardware SDK (Rust). `sameershah21/jengabot` holds
our patches and JengaBot-specific examples.

```bash
mkdir -p ~/learn/github/jengabot && cd ~/learn/github/jengabot

# Our patches + examples + plan
git clone https://github.com/sameershah21/jengabot.git .
# Pick the branch with the firmware-1.8-3 ID-shift fix:
git checkout arm-2

# The upstream SDK we patch (intentionally in .gitignore — keep it separate)
git clone https://github.com/vivym/piper-sdk-rs.git
```

---

## 4. Apply patches to the SDK

Two things to copy across:

**(a) The example binaries we wrote.** Each is a stand-alone tool that
exercises one part of the stack.

```bash
cp examples/piper-sdk-rs-patches/*.rs piper-sdk-rs/crates/piper-sdk/examples/
```

After the `cp`, the following Rust files must exist in
`piper-sdk-rs/crates/piper-sdk/examples/`:

| File | Source | Purpose |
|---|---|---|
| `gs_usb_direct_test.rs` | **upstream** (vivym/piper-sdk-rs) | Raw USB smoke test |
| `exit_teach_mode.rs` | ours (patches folder) | Send raw CAN 0x150 to exit drag-teach |
| `frame_scan.rs` | ours | Passive 200+ frame CAN ID histogram |
| `feedback_check.rs` | ours | Read-only joint observer (verify ID patch) |
| `position_control_demo.rs` | ours (replaces upstream) | Patched demo: extended timeouts, Maintenance→Standby, 50 Hz streaming |
| `joint_sweep.rs` | ours | Exercise each joint ±N° individually |
| `gripper_test.rs` | ours | OPEN→CLOSE→HALF→CLOSE-HARD→OPEN cycle |
| `record_pose.rs` | ours | Drag the arm by hand → save pose to `poses.txt` |
| `play_poses.rs` | ours | Stream a recorded `poses.txt` sequence |
| `leader_stream.rs` | ours | JSON-line joint stream from a hand-dragged arm (used on Raymond) |
| `follower_play.rs` | ours | Reads JSON lines from stdin, streams them as position commands to the connected arm (used on Bruce) |

Sanity check after `cp`:

```bash
ls piper-sdk-rs/crates/piper-sdk/examples/ | grep -E '^(gs_usb_direct_test|exit_teach_mode|frame_scan|feedback_check|position_control_demo|joint_sweep|gripper_test|record_pose|play_poses|leader_stream|follower_play)\.rs$' | sort | uniq -c
# Should print 11 lines, each with count = 1.
```

**(b) The firmware patches.** Decide based on what your arm reports.
First check by running `frame_scan` against your arm (step 7) — if you see
`0x3A0–0x3A7` you're on **S-V1.8-3** and need the ID shift; if you see
`0x2A1–0x2A8` you're on **S-V1.8-2** (or earlier 1.8.x) and need nothing.

```bash
# Only if your arm is on firmware S-V1.8-3:
cd piper-sdk-rs
git apply ../examples/piper-sdk-rs-patches/firmware-1.8-3-id-shift.patch
cd ..
```

Do **not** apply `firmware-1.8-3-yolo.patch`. It was a stop-gap that bypassed
SDK confirmation checks. It caused an arm-drop incident — the ID-shift patch
is the real fix. The yolo file is preserved only as a paper trail.

---

## 5. Build the example binaries

Builds all 11 examples in one cargo invocation. Each `--example` flag matches
exactly one of the files listed in section 4(a).

```bash
cd piper-sdk-rs
cargo build -p piper-sdk \
  --example gs_usb_direct_test \
  --example frame_scan \
  --example feedback_check \
  --example exit_teach_mode \
  --example position_control_demo \
  --example joint_sweep \
  --example gripper_test \
  --example record_pose \
  --example play_poses \
  --example leader_stream \
  --example follower_play
```

First build takes 2–3 minutes (lots of transitive crates). Re-builds after
small edits are seconds. Binaries land at
`piper-sdk-rs/target/debug/examples/<name>`.

After a successful build, verify all 11 binaries exist:

```bash
ls -1 target/debug/examples/ | grep -E '^(gs_usb_direct_test|exit_teach_mode|frame_scan|feedback_check|position_control_demo|joint_sweep|gripper_test|record_pose|play_poses|leader_stream|follower_play)$'
# Should print 11 lines.
```

---

## 6. Set up passwordless sudo for the binaries

macOS needs root to claim the USB device through libusb. Rather than typing
your password every time, drop a sudoers rule that only covers our examples:

```bash
echo "$(whoami) ALL=(root) NOPASSWD: $HOME/learn/github/jengabot/piper-sdk-rs/target/debug/examples/*" \
  | sudo tee /etc/sudoers.d/piper-hackathon
sudo chmod 440 /etc/sudoers.d/piper-hackathon
```

Now `sudo ./target/debug/examples/<name>` runs without prompting.

---

## 7. Bring-up sequence (run in this order)

Plug the dongle into the Mac and the CAN cable into the arm. Power the arm.
The LED on the dongle should be **lit**; the LED on top of the arm should be
**off** (drag-teach off → CAN-controllable).

Always **unplug + replug the candleLight between runs** — the SDK or our raw
USB tools leave the gs_usb device in a half-claimed state, and the next start
will fail with `Failed to set bitrate` or `Infrastructure(Timeout)`. This is
the most common false-alarm in the whole stack.

```bash
cd ~/learn/github/jengabot/piper-sdk-rs
```

### 7a. Smoke test — does USB work?

```bash
sudo ./target/debug/examples/gs_usb_direct_test
```

Expected: discovers the candleLight (`✓ 找到设备`), sets bitrate to 1 Mbit,
starts the device, then prints 20 CAN frames. Frames should come from the arm
within ~50 ms. If you get no frames, the **CAN cable** is the issue (re-seat
both ends, check the arm's power LED).

### 7b. Frame inventory — confirm which firmware

```bash
sudo ./target/debug/examples/frame_scan --frames 500
```

Look at the unique CAN IDs printed in the summary table:

- **0x2A1–0x2A8** present → firmware **S-V1.8-2** or earlier. Do **not** apply
  the ID-shift patch.
- **0x3A0–0x3A7** present (and no 0x2A*) → firmware **S-V1.8-3**. Apply the
  ID-shift patch (step 4b) if you haven't yet, then rebuild.
- **0x251–0x256** and **0x261–0x266** are present on both firmwares.

### 7c. Read-only feedback check

```bash
sudo ./target/debug/examples/feedback_check --seconds 5
```

Expected: prints joint angles at ~2 Hz for 5 seconds, with `samples ok=N
err=0`. Values should match what the arm is physically holding. If you see
zero samples and many errors, the ID-shift patch isn't applied (or applied to
the wrong firmware).

This example is **safe** — it never enables motion, so dropping the handle
will not auto-disable motors. Use it whenever you change patches or want to
verify the read path without movement risk.

### 7d. First commanded motion (the arm WILL move)

Workspace clear of people and obstacles. If the arm is significantly off from
home, support it physically the first time.

```bash
sudo ./target/debug/examples/joint_sweep --amplitude-deg 5 --segment-secs 2
```

Sweeps each of J1–J6 individually +5° → home → -5° → home, then disables
cleanly to Standby. Total runtime ~50 seconds. The arm holds itself at the
last commanded pose after motors disable; brakes are not separately engaged.

After that succeeds, you can try bigger amplitudes (`--amplitude-deg 20`),
the full `position_control_demo`, or `gripper_test`.

---

## 8. Other useful binaries

- `gripper_test` — streams `set_gripper` through OPEN → CLOSE → HALF →
  CLOSE-HARD → OPEN. PiPER's gripper maxes out at ≈0.696 in the SDK's
  normalized `[0, 1]` range.
- `record_pose` — connects in Standby (motors off), lets you drag the arm by
  hand, type a pose name + Enter to append the current joint angles to
  `poses.txt`. If the arm is too stiff to drag, **single-click** the button
  between J5 and J6 → LED solid green → joints go limp → you can drag. Click
  again to stop recording before exiting, then run `exit_teach_mode` so the
  SDK can re-enable.
- `play_poses` — reads `poses.txt` and streams each pose at 50 Hz for 4 s by
  default. `--sequence home,from_slot_1,to_slot_1,home` plays a subset in
  order.
- `exit_teach_mode` — raw GS-USB sender. Writes CAN 0x150 with
  `teach_command=EndRecord (0x02)` to take the arm out of drag-teach mode if
  the physical button isn't available. Teach mode is **non-volatile** — power
  cycling does not clear it. Until teach mode is exited, `enable_position_mode`
  silently times out.
- `leader_stream` — read-only joint angle streamer. Used to make the connected
  arm act as the **leader** in bilateral teleop. The operator drags the arm
  by hand in drag-teach mode (single-click button → LED solid green so
  joints go limp); this binary emits one JSON line per tick:
  `{"t_us": 123456789, "joints_deg": [...]}` at 50 Hz default. Never enables
  motion, never changes state, so dropping does not auto-disable. Optional
  `--out leader.jsonl` records a session for later replay. See section 13
  for the leader/follower setup.

---

## 9. Things that go wrong (and the fix)

| Symptom | Cause | Fix |
|---|---|---|
| `Error: "GS-USB device not found"` | Dongle not on USB. | Replug; check LED. If LED off → swap USB cable. |
| `Error: Infrastructure(Timeout)` right after `Detaching kernel driver` | Dongle in stuck state from previous run. | Unplug + replug candleLight. |
| `Error: Infrastructure(Can(Device(... "Failed to set bitrate: USB error: Input/Output Error")))` | Same — dirty dongle state. | Same — replug. |
| Connected, enters position mode, then `Error: Timeout 5000` on read | Wrong firmware patch state. | Run `frame_scan`. If `0x3A*` IDs, apply `firmware-1.8-3-id-shift.patch` and rebuild. |
| `enable_position_mode` times out | Arm is in drag-teach mode (`teach_status: 1`). | Single-click button on arm to stop teach → LED off → retry. Or `sudo ./target/debug/examples/exit_teach_mode`. |
| `Error: "robot is not in confirmed Standby; run stop first"` | Arm in Maintenance (motors enabled, no mode set). | Our patched examples already handle this. If you're running upstream `position_control_demo`, copy ours from `examples/piper-sdk-rs-patches/`. |
| Arm reports motion but doesn't physically move | Single-shot `send_position_command` is silent on PiPER-X firmware. | Use streaming at ~50 Hz. Our examples do this. |
| Arm **drops** when an example errors mid-motion | The SDK type-state pattern auto-disables motors on `Active` drop. There are no separate brakes. | Always support the arm during first runs of new code. Use small amplitudes (`--amplitude-deg 5`). |

---

## 10. Safety baselines

- **Workspace.** Per AgileX manual: 626.75 mm reach hemisphere from the base.
  Clear it of people, tools, and your laptop screen.
- **Joint limits** (from the manual): J1 ±154°, J2 0°–195°, J3 −175°–0°,
  J4 ±106°, J5 ±75°, J6 ±100°.
- **Zero point.** Per AgileX: when switching from teach to CAN control, the
  arm must be physically at zero (folded, J2 down, J3 fully back). Otherwise
  mode switches may be rejected silently.
- **Drag-teach LED** (top of arm, between J5/J6):
  off = motors holding, on-solid = drag-teach record, blinking = playback.

---

## 11. After arm bring-up: what we're building

- **Phase 1 (now, macOS, single arm):** mechanical Jenga setup. Blocks are
  pre-sorted on parallel "FROM" rails, the arm picks one at a time and places
  on parallel "TO" rails. Hand-recorded poses for each slot (`record_pose` +
  `play_poses`).
- **Phase 2 (cloud GPU, Linux):** fine-tune π₀.₅ VLA on teleop demonstrations
  collected with `agilexrobotics/data_tools`, then deploy via
  `agilexrobotics/openpi-agilex`. Requires a Linux box with ≥22.5 GB GPU
  VRAM (RTX 4090 enough for LoRA).
- **Leader/follower teleop (next):** Raymond's arm mirrors Bruce's. See
  section 13 below.

See `SETUP.md` and `RESOURCES.md` in the repo root for deeper background and
links to AgileX docs.

---

## 13. Leader/follower (Raymond → Bruce) — record + replay

The intended demo is bilateral teleop: operator drags **Raymond** by hand,
**Bruce** mirrors the joint angles. On Linux the SDK's bundled
`dual_arm_bilateral_control` does this live over MIT mode with both arms
on independent CAN buses.

**On macOS, *live* simultaneous two-dongle operation does not work
reliably.** Documented at length in our `macos-no-reset-on-start.patch`
notes and corroborated by [candleLight_fw issue #38](https://github.com/candle-usb/candleLight_fw/issues/38)
and [candleLightJS notes](https://github.com/ieb/candleLightJS): macOS
IOKit's USB stack enters a "kernel halt-state" under sustained
concurrent reads against multiple gs_usb dongles. The only recovery
documented in the community is **physical USB unplug + replug**, which
defeats live teleop. We tried:

- Same-time pipe (init race) — both SDKs reset() each other's USB
  descriptor.
- Staggered init via named pipe + 6 s sleep — fixes init race, ongoing
  RX still dies with `USB receive failed: Other error`.
- Patching the SDK to drop the up-front `reset()` call
  (`macos-no-reset-on-start.patch`) — improves init reliability but
  does not save the RX thread.
- Separate USB controllers (verified physically) — doesn't help.

What we use instead is **record-then-replay**: capture Raymond's motion
to a file with one dongle plugged, then plug only Bruce and replay the
file. Same architecture, same JSON wire format, no concurrent dongles.

### Workflow

```
┌──────────────────────────────────┐    ┌──────────────────────────────────┐
│ Step 1 — RECORD (Raymond only)   │    │ Step 2 — REPLAY (Bruce only)     │
│                                  │    │                                  │
│  unplug Bruce                    │    │  unplug Raymond                  │
│  plug Raymond                    │    │  plug Bruce                      │
│  Raymond LED solid green         │    │  Bruce LED off                   │
│  drag arm by hand                │    │                                  │
│                                  │    │  sudo .../follower_play \        │
│  sudo .../leader_stream \        │    │     --interface BRUCE_SERIAL \   │
│     --interface RAYMOND_SERIAL \ │    │     < raymond.jsonl              │
│     --rate 50 \                  │    │                                  │
│     --out raymond.jsonl          │    │  Bruce executes the recorded     │
│                                  │    │  trajectory at 50 Hz with        │
│  Ctrl+C when done                │    │  --max-step-deg clamp.           │
└──────────────────────────────────┘    └──────────────────────────────────┘
```

### Step 1 — record Raymond

1. **Unplug Bruce's dongle** entirely. Raymond is the only candleLight
   on USB. Confirms: `ioreg -p IOUSB | grep -c "candleLight"` returns 1.
2. **Single-click the button between Raymond's J5/J6** so the LED is
   solid green (drag-teach engaged, joints compliant).
3. Start recording:
   ```bash
   sudo /Users/sameershah/learn/github/jengabot/piper-sdk-rs/target/debug/examples/leader_stream \
       --interface 002500335246570520323934 \
       --rate 50 \
       --out raymond.jsonl
   ```
   (Substitute Raymond's serial. We saw `002500335246570520323934`
   today; verify with `ioreg`.)
4. **Move Raymond by hand** through the trajectory you want Bruce to
   reproduce. The terminal will print JSON lines as you move.
5. **Ctrl+C** when finished. `raymond.jsonl` now holds one JSON record
   per 20 ms of motion.
6. **Single-click Raymond's button again** to turn the LED off
   (drag-teach off — gets Raymond back to CAN mode).

### Step 2 — replay onto Bruce

1. **Unplug Raymond's dongle.** Plug Bruce's. Only Bruce on USB.
2. Confirm Bruce's LED is **off** (CAN-controllable, not drag-teach).
3. Workspace clear; ready to support Bruce if anything looks wrong.
4. Stream the recorded trajectory:
   ```bash
   sudo /Users/sameershah/learn/github/jengabot/piper-sdk-rs-v1.8-2/target/debug/examples/follower_play \
       --interface 0028002A4148570C20343133 \
       --max-step-deg 2 \
       < raymond.jsonl
   ```
   (Substitute Bruce's serial. We saw `0028002A4148570C20343133` today.)
5. Bruce will:
   - Connect, transition Maintenance → Standby if needed
   - Enable position mode
   - Seed its target at its current pose
   - Read JSON lines from stdin at the input's natural rate
   - Stream `send_position_command` at 50 Hz with `max_step_deg`
     clamping
   - Disable cleanly when the input file ends (EOF on stdin)

### Tuning replay

- **Slower replay** for safety on first run: pre-process the file to
  duplicate each line N times or to interleave a `sleep`. Or modify
  `follower_play` to accept a `--rate` override that decouples its
  stream rate from the input rate.
- **Smaller steps:** drop `--max-step-deg` to `1.0` for slower, safer
  motion at the cost of lag.
- **Re-record:** if Raymond's trajectory had a glitch, just re-record.
  JSON files are tiny (~1 KB/sec).

### When two machines become available

The exact same wire format works over a network. Future setup once a
second machine is available:

```bash
# Machine A (Raymond), live source
sudo .../leader_stream --interface RAYMOND_SERIAL --rate 50 | nc -l 9000

# Machine B (Bruce), live consumer
nc machine-a.local 9000 | sudo .../follower_play --interface BRUCE_SERIAL
```

Same `follower_play`, same `leader_stream`, same JSON lines — only the
transport between them changes.

### Alternative — bundled SDK dual-arm example (Linux only in practice)

`piper-sdk-rs` ships `dual_arm_bilateral_control`, which uses MIT mode
and explicit `--left-serial` / `--right-serial` flags. Works on Linux
because SocketCAN serializes per-arm CAN drivers properly. On macOS it
hits the same IOKit halt-state described above.

```bash
sudo ./target/debug/examples/dual_arm_bilateral_control \
    --left-serial  <RAYMOND_SERIAL> \
    --right-serial <BRUCE_SERIAL>   \
    --mode master-follower
```

### Safety reminders for the follower

- Bruce's motors **auto-disable on `Active` drop** if the follower
  process errors out — the arm falls. Support it physically for the
  first session; use short runs (Ctrl+C within 5 s) to confirm clean
  disable.
- `follower_play` holds last pose on stdin silence (default 500 ms
  watchdog) rather than going limp. If you want a hard stop on EOF,
  shorten `--watchdog-ms` and trap the resulting "input gone" by
  Ctrl+C'ing the follower yourself.
- Per-joint clamp uses the PiPER-X joint limits from the manual
  (J1 ±154°, J2 0–195°, etc.) — out-of-range targets are silently
  clipped, not commanded.

---

## 12. Who to ping

- **Sameer** (this repo, macOS Rust path, firmware-1.8-3 ID patch)
- **Volk** ([`JengaMaxxers`](https://github.com/volkthienpreecha/JengaMaxxers) —
  Windows Python FastAPI single-arm web dashboard)
- **AgileX support** at `support@agilex.ai` for firmware downloads,
  ArmRobot.exe (Windows-only flashing tool), and Yuque docs access.
