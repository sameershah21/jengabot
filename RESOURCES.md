# JengaBot — Piper / Agilex Resources

Official AgileX repos and docs, with notes on which ones we're actually using.

## Piper SDK (Python — official)

- **piper_sdk** — https://github.com/agilexrobotics/piper_sdk
  - Linux-only (`socketcan`). Doesn't work on macOS — see [Issue #24](https://github.com/agilexrobotics/piper_sdk/issues/24).
- **pyAgxArm** — https://github.com/agilexrobotics/pyAgxArm
  - Newer Python wrapper. Same Linux constraint.
- **pyAgxArm API reference** — https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/piper_api.md
- **Demos (V2)** — https://github.com/agilexrobotics/piper_sdk/tree/master/piper_sdk/demo/V2

## What we're actually using on macOS

- **vivym/piper-sdk-rs** — https://github.com/vivym/piper-sdk-rs
  - Cross-platform Rust SDK (Linux/Windows/macOS) via `rusb` + gs_usb.
  - Our `position_control_demo` runs on this. See `SETUP.md`.

## ROS

- **ROS 1** — https://github.com/agilexrobotics/piper_ros
- **ROS 2** — https://github.com/agilexrobotics/agx_arm_ros

(Not in current plan; reach-for if we move to a ROS-based teleop or perception stack.)

## Windows GUI

- **ArmRobot upper-computer software** (Windows only) — https://agilexsupport.yuque.com/staff-hso6mo/zcobo3/oeg25xsf8uqgq60f
  - Useful for calibration / mode toggling outside our Mac dev flow.

## Gripper / claw

- **OpenClawPi** — https://github.com/vanstrong12138/OpenClawPi
  - Community deployment for AgileX OpenClaw end-effector. Pull this in when we attach a gripper for actual block picking.

## VLA / learning

- **openpi-agilex** — https://github.com/agilexrobotics/openpi-agilex
  - π₀.₅ VLA fine-tuning, Linux + NVIDIA GPU. Phase 2 (Alibaba Cloud).
- **agilexrobotics/data_tools** — https://github.com/agilexrobotics/data_tools
  - Teleop episode collection → LeRobot dataset.

## Quick reference — what runs where

| Layer | Platform | Tool |
|---|---|---|
| Phase 1 connectivity + waypoint demo | macOS | `piper-sdk-rs` (Rust) |
| Gripper integration | Linux/macOS | OpenClawPi + Rust send_raw |
| Teleop data collection | Linux | `data_tools` |
| VLA fine-tune | Linux + GPU (Alibaba Cloud) | `openpi-agilex` |
| Pitch + judging | anywhere | this repo |
