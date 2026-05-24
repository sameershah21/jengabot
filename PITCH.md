# JengaBot

## One-line pitch

Two robotic arms, two cameras, and the first working macOS Rust SDK for the
AgileX PiPER-X, all stacking color-coded Jenga blocks from teleop demonstrations.

## 60-second elevator version

JengaBot is a bilateral teleoperation and imitation-learning rig built for
robotic Jenga. A human moves an SO-101 leader arm; an AgileX PiPER-X follower
named Bruce mirrors it, wrist-mounted depth camera and overhead webcam both
recording the whole episode. The blocks live on a printed poster where every
slot is outlined and color-coded red or blue, so the operator and any future
vision policy share the same unambiguous targets.

The unsexy hard part: AgileX ships a Linux-only SDK. We hardened a Rust
PiPER-X SDK that actually runs on macOS over libusb, patched a firmware CAN-ID
shift, fixed the IOKit halt that blocked multi-arm init, and verified it on
five different PiPER robots. On top of that we built the leader bridge, the
dual-camera recorder, a LeRobot v2 dataset pipeline, and a trained
behavioral-cloning checkpoint, all in one hackathon, all on Mac.

## What we actually built

- macOS Rust SDK for the PiPER-X over libusb / gs_usb, with patches living
  in `examples/piper-sdk-rs-patches/`:
  - firmware S-V1.8-3 CAN-ID shift (0x2A1-0x2A8 to 0x3A0-0x3A7)
  - IOKit "kernel halt-state" override so two arms can init concurrently
  - type-state Active-to-Drop fix that was disabling motors mid-session
  - tested working on 5 different PiPER-X robots
- Cross-vendor leader bridge: `apps/soarm_leader/soarm_leader.py` reads
  Feetech STS3215 servos from the SO-101 over CH340 USB-serial, remaps
  SO-101 joints (J1-J3, J4 to J5, J5 to J6, hold Bruce J4), handles
  per-servo +/-4096 wraparound, applies sign/scale/smoothing, emits JSON.
- Rust follower replay: `piper-sdk-rs/crates/piper-sdk/examples/follower_play.rs`
  consumes the JSON, per-tick clamps, watchdog, and re-asserts the gripper
  seed for 250 ms after enable to override the firmware's auto-70%-open.
- Dual-camera record: Orbbec Dabai DC1 on Bruce's wrist plus iCspring 1080p
  overhead, both writing synced mp4 alongside the jsonl episode log.
  Name-based AVFoundation lookup so camera indices stop reshuffling between
  runs, and the silent 360x640 portrait fallback is gone.
- ML pipeline: `apps/dataset_builder/build_lerobot.py` builds HuggingFace
  LeRobot v2 parquet. `apps/bc_trainer/train_bc.py` trains a BC MLP on
  Apple MPS in ~13 seconds. Trained checkpoint shipped in-repo at
  `apps/bc_trainer/models/bc_20260524_132821.pt` (75 KB, 18,439 params,
  val MSE 0.078, 6 episodes / 1,417 frames).
- Workspace: printed Jenga poster with each block individually outlined
  and color-coded red/blue.

## Why this is hard / why it matters

- AgileX's official PiPER SDK is Linux-only SocketCAN. Nobody had this
  arm working on macOS in a reusable way. We do, in Rust, and the patches
  are the rare contribution the broader PiPER community can pick up.
- Bilateral teleop across two unrelated robot vendors and two physically
  different kinematic chains (SO-101's 5+gripper into PiPER's 6+gripper)
  is a real integration problem, not a library call.
- Two cameras synced with proprioception in a single episode log is the
  exact shape modern VLA training expects. We have the data plumbing,
  not just a demo video.
- We did all of it on Mac, with no Linux fallback, the entire hackathon.

## Live demo script

1. Show the poster: red and blue outlined Jenga slots, blocks staged.
2. Hand on the SO-101 leader, walk a pick-and-place arc in free space so
   the judge sees the bridge translating joints in real time on the
   replay UI.
3. Run `follower_play` against a recorded episode. Bruce executes the
   pick on the poster while the wrist Dabai and overhead iCspring
   preview both stream on the laptop.
4. Open the dataset: one parquet row, both video frames, joint state,
   action. Same schema LeRobot expects.
5. Show the trained `.pt` checkpoint and the training log: 13 seconds
   on MPS, val MSE 0.078.

## What's next

- The shipped model is a behavioral-cloning MLP, not ACT, not SmolVLA,
  not a VLA. The 6-episode / 1,417-frame dataset is a baseline, not a
  policy that will stack a tower. Scaling the dataset and swapping in
  an ACT head is the obvious next step.
- Live simultaneous leader + follower on one Mac is blocked by macOS
  IOKit when both arms are on the same host. Our SDK supports it on
  Linux or split across two Macs; we recorded and replayed instead.
  Not a code defect, a platform constraint we will route around.
- `observation.state` in the current dataset is a leader-command proxy.
  Next collection pass uses `follower_play --observed-log` to capture
  true Bruce feedback.
- Autonomous Jenga stacking (vision-conditioned policy on the poster)
  is the goal the platform is built for. The data pipeline and the
  hardware bring-up are done; the autonomous stacker is the next sprint.

## Repo & artifacts

- GitHub: https://github.com/sameershah21/jengabot
- Leader bridge: `apps/soarm_leader/soarm_leader.py`
- Rust follower: `piper-sdk-rs/crates/piper-sdk/examples/follower_play.rs`
- macOS / firmware patches: `examples/piper-sdk-rs-patches/`
- Dataset builder: `apps/dataset_builder/build_lerobot.py`
- BC trainer: `apps/bc_trainer/train_bc.py`
- Trained checkpoint: `apps/bc_trainer/models/bc_20260524_132821.pt`
- Full setup writeup: `ONBOARDING.md`
