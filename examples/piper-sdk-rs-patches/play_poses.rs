//! play_poses — execute a sequence of recorded poses (poses.txt)
//!
//! Reads the poses.txt produced by record_pose. Supports two run modes:
//!
//!   * default: plays every pose in file order
//!   * --sequence "name_a,name_b,name_c": plays only the named poses, in
//!     the listed order. Each name must exist in the file.
//!
//! Streams the position command at 50 Hz for `--segment-secs` per pose
//! (default 4s), so the arm actually moves to each target.
//!
//! Run:
//!   sudo ./target/debug/examples/play_poses --interface auto
//!   sudo ./target/debug/examples/play_poses --interface auto \
//!        --sequence home,from_slot_1,to_slot_1,home
//!
//! Gripper:
//!   The pose file format does NOT include gripper state. For pick-and-place
//!   sequences, interleave: move-to-pose, gripper-close, move-to-pose,
//!   gripper-open. That's a job for a richer "playbook" example later.

use clap::Parser;
use piper_sdk::client::state::MotionCapability;
use piper_sdk::client::state::*;
use piper_sdk::client::{MotionConnectedPiper, MotionConnectedState};
use piper_sdk::prelude::*;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::time::{Duration, Instant};

const STREAM_PERIOD: Duration = Duration::from_millis(20); // 50 Hz

#[derive(Parser, Debug)]
#[command(name = "play_poses")]
struct Args {
    #[cfg_attr(target_os = "linux", arg(long, default_value = "can0"))]
    #[cfg_attr(not(target_os = "linux"), arg(long, default_value = "auto"))]
    interface: String,

    #[arg(long, default_value = "1000000")]
    baud_rate: u32,

    /// Pose file
    #[arg(long, default_value = "poses.txt")]
    file: PathBuf,

    /// Comma-separated sequence of pose names to play (default: all in file order)
    #[arg(long)]
    sequence: Option<String>,

    /// Seconds per pose
    #[arg(long, default_value = "4")]
    segment_secs: u64,
}

#[derive(Debug, Clone)]
struct Pose {
    name: String,
    joints: [f64; 6],
}

fn parse_poses(path: &PathBuf) -> std::result::Result<Vec<Pose>, Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    let mut poses = Vec::new();
    for line in BufReader::new(file).lines() {
        let line = line?;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let toks: Vec<&str> = line.split_whitespace().collect();
        if toks.len() != 7 {
            return Err(format!("bad line (need name + 6 joints): {line}").into());
        }
        let mut joints = [0.0; 6];
        for (i, t) in toks[1..].iter().enumerate() {
            joints[i] = t.parse()?;
        }
        poses.push(Pose {
            name: toks[0].to_string(),
            joints,
        });
    }
    Ok(poses)
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    piper_sdk::init_logger!();
    let args = Args::parse();

    let all_poses = parse_poses(&args.file)?;
    println!("📂 Loaded {} pose(s) from {}", all_poses.len(), args.file.display());

    let by_name: HashMap<String, Pose> = all_poses
        .iter()
        .map(|p| (p.name.clone(), p.clone()))
        .collect();

    let playlist: Vec<Pose> = match &args.sequence {
        Some(s) => {
            let mut out = Vec::new();
            for name in s.split(',') {
                let name = name.trim();
                match by_name.get(name) {
                    Some(p) => out.push(p.clone()),
                    None => {
                        return Err(format!("pose `{name}` not in file").into());
                    }
                }
            }
            out
        }
        None => all_poses,
    };

    println!("▶ Playlist: {}\n", playlist.iter().map(|p| p.name.as_str()).collect::<Vec<_>>().join(" → "));

    let robot = {
        #[cfg(target_os = "linux")]
        {
            PiperBuilder::new()
                .socketcan(&args.interface)
                .baud_rate(args.baud_rate)
                .feedback_timeout(Duration::from_secs(10))
                .firmware_timeout(Duration::from_secs(5))
                .build()?
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = &args.interface;
            PiperBuilder::new()
                .gs_usb_auto()
                .baud_rate(args.baud_rate)
                .feedback_timeout(Duration::from_secs(10))
                .firmware_timeout(Duration::from_secs(5))
                .build()?
        }
    };

    let robot = robot.require_motion()?;
    match robot {
        MotionConnectedPiper::Strict(MotionConnectedState::Standby(robot)) => run(robot, &playlist, args.segment_secs)?,
        MotionConnectedPiper::Soft(MotionConnectedState::Standby(robot)) => run(robot, &playlist, args.segment_secs)?,
        MotionConnectedPiper::Strict(MotionConnectedState::Maintenance(robot)) => {
            println!("⚠️  Maintenance → disabling all → Standby");
            let robot = robot.request_disable_all()?;
            let robot = robot.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            run(robot, &playlist, args.segment_secs)?
        },
        MotionConnectedPiper::Soft(MotionConnectedState::Maintenance(robot)) => {
            println!("⚠️  Maintenance → disabling all → Standby");
            let robot = robot.request_disable_all()?;
            let robot = robot.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            run(robot, &playlist, args.segment_secs)?
        },
    }
    Ok(())
}

fn run<Capability>(
    robot: Piper<Standby, Capability>,
    playlist: &[Pose],
    segment_secs: u64,
) -> std::result::Result<(), Box<dyn std::error::Error>>
where
    Capability: MotionCapability,
{
    println!("⚡ Enabling position mode...");
    let robot = robot.enable_position_mode(PositionModeConfig {
        timeout: Duration::from_secs(15),
        ..PositionModeConfig::default()
    })?;

    let observer = robot.observer();
    let segment = Duration::from_secs(segment_secs);

    for (i, pose) in playlist.iter().enumerate() {
        println!("\n[{}/{}] → {}", i + 1, playlist.len(), pose.name);
        let target = JointArray::from([
            Rad(pose.joints[0]),
            Rad(pose.joints[1]),
            Rad(pose.joints[2]),
            Rad(pose.joints[3]),
            Rad(pose.joints[4]),
            Rad(pose.joints[5]),
        ]);
        let start = Instant::now();
        while start.elapsed() < segment {
            robot.send_position_command(&target)?;
            std::thread::sleep(STREAM_PERIOD);
        }
        if let Ok(actual) = observer.joint_positions() {
            let err_deg = target
                .iter()
                .zip(actual.iter())
                .map(|(t, a)| (t.0 - a.0).abs().to_degrees())
                .fold(0.0_f64, f64::max);
            println!("   reached (max joint error {:.2}°)", err_deg);
        }
    }

    println!("\n🛑 Disabling...");
    let _robot = robot.disable(DisableConfig::default())?;
    println!("✅ Done");
    Ok(())
}
