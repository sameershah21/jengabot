# JengaBot bring-up — get a PiPER-X arm moving on macOS

Stand-alone guide for someone joining the project. Skipping straight to "what
do I install, what do I run, what breaks" instead of recapping the whole
debugging journey. The full story is in `SETUP.md` and the branch commits.

> Verified end-to-end on macOS (Apple Silicon, darwin 25.5) + bytewerk
> candleLight USB-CAN dongle + AgileX PiPER-X on firmware **S-V1.8-2** and
> **S-V1.8-3** (each needs slightly different SDK state — see step 4).

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
  --example play_poses
```

First build takes 2–3 minutes (lots of transitive crates). Re-builds after
small edits are seconds.

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
- **Leader/follower teleop (next):** second PiPER-X mirrors the first.
  `dual_arm_bilateral_control` example in `piper-sdk-rs` does master-follower
  via MIT mode (back-drivable leader, position-streamed follower). Needs
  two dongles + two serials.

See `SETUP.md` and `RESOURCES.md` in the repo root for deeper background and
links to AgileX docs.

---

## 12. Who to ping

- **Sameer** (this repo, macOS Rust path, firmware-1.8-3 ID patch)
- **Volk** ([`JengaMaxxers`](https://github.com/volkthienpreecha/JengaMaxxers) —
  Windows Python FastAPI single-arm web dashboard)
- **AgileX support** at `support@agilex.ai` for firmware downloads,
  ArmRobot.exe (Windows-only flashing tool), and Yuque docs access.
