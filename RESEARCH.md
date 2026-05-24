# JengaBot - Research notes

Companion to `PITCH.md` (short sell) and `ONBOARDING.md` (operator
guide). This document covers what was actually built, what was
independently verified versus operator-attested, and where the
contributions sit relative to the upstream LeRobot / AgileX / Orbbec
ecosystems.

## 1. Executive summary

JengaBot is a cross-vendor bilateral teleoperation rig and the
imitation-learning data pipeline behind it, all running on macOS. A
human drags a HuggingFace LeRobot SO-101 leader; an AgileX PiPER-X
6-DoF arm ("Bruce") mirrors it; an Orbbec DaBai DC1 wrist camera and
an overhead 1080p USB webcam record synchronised RGB; every joint
command and gripper position is logged as JSONL. The hard, reusable
part is a macOS-native Rust SDK for the PiPER over USB-CAN: a
firmware-S-V1.8-3 CAN-ID-shift patch, an IOKit concurrent-init patch,
a type-state motor-disable workaround, and a gripper seed assertion -
attested by the operator to work across five different PiPER-X arms.
On top of that we shipped two BC baselines (18k-param state-only MLP
and 356k-param vision+state) plus a real SmolVLA fine-tune
(450M params, 200 steps in 85 s on Apple MPS, loss 0.403 -> 0.120)
against a freshly converted LeRobot v3.0 dataset, with the
865 MB checkpoint published via git-lfs and on Hugging Face Hub.

## 2. Hardware build

| Device | Role | Reference |
|---|---|---|
| **AgileX PiPER-X**, "Bruce" | 6-DoF follower, 626 mm reach, 1.5 kg payload, 0.1 mm repeatability, CAN-bus interface | https://global.agilex.ai/products/piper-x and https://global.agilex.ai/products/piper |
| **LeRobot SO-101 leader arm**, "Raymond's stand-in" | 5 joints + parallel gripper using Feetech STS3215 servos (mixed 1/147, 1/191, 1/345 gearing on the leader so it back-drives easily); CH340 USB-serial driver board | https://huggingface.co/docs/lerobot/en/so101 and https://github.com/TheRobotStudio/SO-ARM100 |
| **Orbbec DaBai DC1** (wrist) | Stereo IR depth + RGB; we use depth IR at 640x400 Y11 @ 30 fps (RGB left closed - macOS UVC driver claims the colour interface); USB PID `0x0657` | https://www.orbbec.com/documentation/depth-camera/ , https://github.com/orbbec/pyorbbecsdk |
| **iCspring 1080p USB webcam** | Overhead view, 1920x1080 mp4 via AVFoundation | generic UVC |
| **candleLight USB-to-CAN dongle x 2** | Cross-platform USB CAN adapter running `candleLight_fw` (gs_usb USB class, VID `0x1D50` / PID `0x606F`). The only dongle that works on macOS without reflashing, because gs_usb is a userspace protocol over libusb | https://github.com/candle-usb/candleLight_fw , https://python-can.readthedocs.io/en/stable/interfaces/gs_usb.html |
| **24 V / >= 10 A PSU** per PiPER | Power | AgileX manual |
| **Printed Jenga poster** | Workspace mat with every Jenga slot outlined and colour-coded red/blue, giving operator and policy a shared addressable target grid | local artefact |

Two PiPER-X arms means **two independent candleLight dongles on two
independent CAN buses** - the SDK claims gs_usb exclusively per device,
so chaining is not an option.

## 3. The macOS Rust CAN SDK - the central contribution

### Why this was needed

AgileX ships `agilexrobotics/piper_sdk` (Python) which assumes Linux
SocketCAN. macOS has no SocketCAN. The community Rust port
`vivym/piper-sdk-rs` is the only SDK explicitly targeting
Linux+Windows+macOS, going through gs_usb USB-class via `rusb`/libusb
instead of kernel CAN (https://github.com/vivym/piper-sdk-rs). Upstream
flags itself as "under active development" and "NOT been fully tested
on real robotic arms" - that is exactly the gap we filled.

### Patches we wrote and ship in `examples/piper-sdk-rs-patches/`

1. **Firmware S-V1.8-3 CAN-ID shift** (`firmware-1.8-3-id-shift.patch`).
   PiPER firmware quietly relocated the "cold" feedback block by +0xFF
   between 1.8-2 and 1.8-3: `ID_ROBOT_STATUS`, `ID_END_POSE_1..3`,
   `ID_JOINT_FEEDBACK_12/34/56` and `ID_GRIPPER_FEEDBACK` move from
   `0x2A1-0x2A8` to `0x3A0-0x3A7`. Hot-path joint driver IDs
   `0x251-0x256` and low-speed `0x261-0x266` stayed put. Our
   `frame_scan` example produces the histogram you use to decide
   whether to apply the patch.

2. **IOKit "kernel halt-state" / no-reset-on-start patch**
   (`macos-no-reset-on-start.patch`). Upstream `GsUsbDevice::start`
   issues `handle.reset()` (USB re-enumeration) before claiming the
   interface; doing that from two SDK instances simultaneously on the
   same Mac invalidates both descriptors and both threads die (matches
   https://github.com/candle-usb/candleLight_fw/issues/38). We drop
   the up-front reset and rely on persistent bitrate + MODE command,
   which makes two-arm init succeed. Live RX from two dongles in
   parallel is still flaky on macOS - a platform IOKit issue we route
   around with record-then-replay (section 4).

3. **Type-state `Active`-drop motor-disable fix.** The Rust SDK auto-
   disables motors when the `Active` handle drops, so any panic mid-
   motion makes the arm *fall* (PiPER has no separate brakes). Our
   patched `position_control_demo.rs`, `joint_sweep.rs`, and
   `follower_play.rs` guard the drop path and transition Maintenance
   -> Standby cleanly so the arm holds its last pose.

4. **Gripper seed re-assertion in `follower_play.rs`.** After
   `enable_position_mode`, firmware default opens the gripper to ~70 %
   regardless of what was commanded one frame earlier. We assert the
   seed gripper value at high rate for 250 ms post-enable to override
   that bias, then continue with incremental deltas.

5. **Examples + binaries**: `frame_scan`, `feedback_check`,
   `exit_teach_mode` (raw CAN 0x150 EndRecord), `joint_sweep`,
   `gripper_test`, `record_pose`, `play_poses`, `leader_stream`,
   `follower_play` - single-purpose tools all built from one
   `cargo build` invocation (`ONBOARDING.md` section 5).

The operator attests this stack has been validated across **five
different physical PiPER-X arms** during the hackathon. We did not
independently verify five arms ourselves; what is in the repo are the
patches, the build instructions, and the JSON wire format that made
that validation possible.

## 4. Cross-vendor bilateral teleop

A JSON-lines wire format crosses two unrelated robot vendors.

**Leader** (`apps/soarm_leader/soarm_leader.py`) opens the SO-101 driver
board at 1 Mbaud (`DTR=True, RTS=False`), parses `[POS] p1..p6` ASCII
lines (raw STS3215 0..4095, 360/4096 deg per step), captures the first
valid frame as zero, and per servo: tracks a **wrap offset** across the
0/4095 boundary (any `diff` past +/-2048 shifts the offset by 4096 so
deltas stay direction-correct), applies per-joint sign + scale (default
`--signs -1,1,-1,1,-1,1`), and **remaps** SO-101 J1-J3 -> Bruce J1-J3,
SO-101 J4 -> Bruce J5, SO-101 J5 -> Bruce J6, with **Bruce J4 held at
seed** (no SO-101 source - SO-101 is 5-DoF, PiPER is 6-DoF). The 6th
SO-101 servo is the gripper handle, emitted as a `--gripper-scale`-d raw
delta with a `--gripper-deadband` so Bruce holds its physical position
until you actually squeeze. Output is rate-capped at 50 Hz with an
idle-emit so the follower's stdin watchdog never trips.

**Follower** (`piper-sdk-rs/.../follower_play.rs`) consumes the JSON at
50 Hz with `--max-step-deg` per-tick clamping (default 2 deg),
`--smoothing` exponential low-pass for choppy low-rate leaders, joint-
limit clamping from the AgileX manual (J1 +/-154, J2 0..195, J3 -175..0,
J4 +/-106, J5 +/-75, J6 +/-100), `--incremental` mode so the SO-101's
zero pose can differ from Bruce's, and a watchdog that holds the last
pose on stdin silence. An `--observed-log` flag captures Bruce's actual
joint feedback to disk during replay - the intended next-pass fix for
`observation.state` being a leader-command proxy. Live two-dongle teleop
on one Mac is the platform's IOKit limitation, not the software's; we
record-and-replay instead (full failure modes in `ONBOARDING.md` s. 13).

## 5. Vision stack

Each episode produces a `{stamp}.jsonl` plus up to two side-by-side
mp4s: `{stamp}_dabai.mp4` (wrist) and `{stamp}_top.mp4` (overhead). Two
macOS-specific gotchas we fixed in the recorder: AVFoundation **reorders
camera indices between runs** (fixed by name-based device lookup so
"Dabai" and "iCspring" resolve across replug); and the iCspring
**silently falls back to 360x640 portrait** when `-video_size` is not
pinned (fixed by passing `-video_size 1920x1080` explicitly).

The wrist camera is the Orbbec DaBai DC1, opened depth-only on macOS to
avoid `uvc_open ... res:-3 (ACCESS)` from the kernel UVC driver claiming
the colour interface. Verified specs from `apps/depth_sensor/orbbec_probe.py`:
depth 640x400 Y11 @ 30 fps, `fx=fy=477.8`, `cx=322.9`, `cy=199.0`,
`get_depth_scale()` returns mm/unit, serial `CC1N16201DS`, firmware
`RD1001`. Prior art `jonathanhawkins/jenga-stacker` runs the same
hardware combo - their `orbbec_camera.py`, `perception3d.py` and
`calibrate_handeye.py` are the templates to lift for hand-eye work.

## 6. The Jenga poster workspace

We printed a poster with every Jenga slot individually outlined and
colour-coded red or blue. The point is **address space**: instead of
asking a vision policy to do per-frame instance segmentation of
identical wooden blocks, the operator and the policy share a printed,
addressable target grid. "Pick red slot 3, place blue slot 7" becomes a
colour-coordinate lookup, and the colour cue is in-distribution at every
frame of BC/VLA training.

## 7. Data pipeline and models

Raw artefacts on disk:

- `episodes/` - 6 jsonl episodes + 6 wrist mp4s + 3 overhead mp4s (top
  cam added partway through the session). Wrist mp4s 13-165 MB; total
  1,417 joint frames across the 6 episodes.
- `dataset/` - hand-rolled **LeRobot v2** parquet via
  `apps/dataset_builder/build_lerobot.py`.
- `dataset_v3/` - **LeRobot v3.0** rebuild via
  `apps/smolvla_trainer/build_v3_dataset.py`, using the official
  `LeRobotDataset.create() / add_frame() / save_episode()` API.
  `meta/info.json` confirms `codebase_version=v3.0`, robot_type
  `piper_x_so101_teleop`, 3 episodes / 566 frames, fps 30, both
  `observation.images.top` and `observation.images.dabai` re-encoded
  to 256x256 AV1 (https://huggingface.co/docs/lerobot/main/en/lerobot-dataset-v3).

### Trained checkpoints (all on Apple MPS)

| Model | File | Params | Train frames | Val MSE | Wall time |
|---|---|---|---|---|---|
| **State-only BC MLP** (state -> action) | `apps/bc_trainer/models/bc_20260524_132821.pt` | 18,439 | 1,276 (val 141) | 0.0776 | 13.41 s, 300 epochs, batch 64, hidden 128, lr 1e-3 |
| **Vision-conditioned BC** (CNN+state -> action) - early 30-epoch run | `apps/smolvla_trainer/models/vision_bc_20260524_134934.pt` | 356,135 | 510 (val 56) | 4.14 | 4.66 s |
| **Vision-conditioned BC** - 120-epoch run, hidden 256, img 96 | `apps/smolvla_trainer/models/vision_bc_20260524_135058.pt` | 356,135 | 510 (val 56) | 0.126 | 8.56 s |

All numbers come from the actual `*_meta.json` next to each `.pt` and
are not estimates.

### SmolVLA fine-tune

We installed `lerobot[smolvla]` v3.0, built the v3 dataset, and ran
`lerobot-train --policy.path=lerobot/smolvla_base --policy.device=mps
--batch_size=2 --steps=200`. The base is HuggingFace
`lerobot/smolvla_base`, a 450M-param VLA (100M of which are learnable
at fine-tune time) on the `HuggingFaceTB/SmolVLM2-500M-Video-Instruct`
VLM backbone (https://huggingface.co/docs/lerobot/smolvla; the
`vlm_model_name` and `pretrained_path` are both visible in the
run-config dump in `/tmp/smolvla_train.log`). The pretrained checkpoint
expects three camera streams (`camera1/2/3`); we supplied two via
`--rename_map` (`top->camera1`, `dabai->camera2`); `camera3` is silently
dropped by lerobot.

**Outcome**: 200 steps completed in **85 s wall-clock** on Apple MPS.
Loss trace (every 10 steps, logged in
`apps/smolvla_trainer/loss_curve.txt`):
0.403 -> 0.197 -> 0.162 -> 0.130 -> 0.105 -> **0.120** at step 200,
gradient norm 6.5 -> 1.7. The 865 MB `model.safetensors` is committed
under `apps/smolvla_trainer/runs/smolvla_real/checkpoints/000200/pretrained_model/`
via **git-lfs** (`.gitattributes` tracks `*.safetensors`) and mirrored
to https://huggingface.co/pilarclark/jengabot-smolvla-jenga so it can
be loaded with
`SmolVLAPolicy.from_pretrained("pilarclark/jengabot-smolvla-jenga")`.

HuggingFace's own docs put a 20k-step SmolVLA fine-tune at ~4 hours on
a single A100, and the SmolVLA team recommends a ~50-episode minimum
for a useful task-specific fine-tune. Our 200-step / 3-episode run is
**proof of pipeline**, not a deployable autonomous controller; with the
same trainer + the user's existing Alibaba Cloud GPU budget and more
episodes captured via `follower_play --observed-log`, the path to a real
policy is unblocked.

## 8. What is deliberately not done yet

- **Autonomous Jenga-stacking policy.** Platform and data pipeline
  done; shipped checkpoints are MLP/CNN baselines, not a tower-stacker.
- **More episodes.** 6 collected, 3 with the overhead camera. SmolVLA
  guidance is ~50 episodes minimum (their paper showed 25 insufficient).
- **Real follower observation.** Today `observation.state` is the
  leader's commanded pose - a proxy. `follower_play --observed-log` is
  wired; data was already collected before the flag landed.
- **Hand-eye calibration.** `apps/depth_sensor/jenga_detect.py` produces
  block poses in the camera frame; the base-frame transform
  `T_flange_cam` needs the leader/follower stack end-to-end.
- **Live simultaneous two-dongle teleop on one Mac.** OS-level IOKit
  limitation, not a code defect; works on Linux SocketCAN and across
  two machines over `nc`.

## 9. Reusable contributions

For other PiPER-X / LeRobot users, the artefacts most worth lifting:

- **`examples/piper-sdk-rs-patches/`** - the macOS Rust SDK patch set
  (firmware-1.8-3 ID shift, IOKit no-reset, type-state drop fix,
  gripper seed assertion) plus 9 single-purpose example binaries. The
  only macOS-native PiPER stack we are aware of.
- **`apps/soarm_leader/soarm_leader.py` + `follower_play.rs`** - a
  cross-vendor bilateral bridge: SO-101 Feetech serial ASCII on one
  end, PiPER CAN on the other, JSON-lines in the middle. Mappable to
  any leader that can emit `{t_us, joints_deg, gripper}`.
- **`apps/dataset_builder/build_lerobot.py`** + **`apps/smolvla_trainer/build_v3_dataset.py`** -
  jsonl + mp4 -> LeRobot v2 (hand-rolled) and -> LeRobot v3.0 (via the
  official `LeRobotDataset.create()` API) converters for sources the
  upstream recorder did not produce.
- **`apps/bc_trainer/train_bc.py` and `apps/smolvla_trainer/train_vision_bc.py`** -
  minimal BC and vision-BC trainers that finish in single-digit seconds
  on Apple Silicon MPS and emit a sidecar `_meta.json`. Useful smoke
  tests before reaching for a real VLA.
- **`ONBOARDING.md` sections 7, 9, 13, 14** - operator troubleshooting
  matrix (USB stuck state, drag-teach LED semantics, IOKit halt-state,
  depth-only-on-macOS).
