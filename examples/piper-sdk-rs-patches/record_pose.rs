//! record_pose — capture the arm's current joint angles to a poses.txt file
//!
//! Flow:
//!   1. Connect, ensure Standby (motors disabled — arm is hand-draggable).
//!   2. You manually move the arm to a desired pose.
//!   3. Type a pose name (e.g. `from_slot_1`) and press Enter — angles are
//!      captured and appended to `poses.txt`.
//!   4. Type `q` + Enter to quit.
//!
//! poses.txt format (one pose per line, hashes for comments):
//!   name j1 j2 j3 j4 j5 j6      (joint angles in radians)
//!
//! Run:
//!   sudo ./target/debug/examples/record_pose --interface auto
//!   sudo ./target/debug/examples/record_pose --interface auto --out my_poses.txt
//!
//! NOTE: if the arm is stiff (motors not fully disabled), put it in drag-teach
//! mode using the physical button on the Piper before recording. Run
//! `exit_teach_mode` after recording so the SDK can re-enable.

use clap::Parser;
use piper_sdk::client::state::MotionCapability;
use piper_sdk::client::state::*;
use piper_sdk::client::{MotionConnectedPiper, MotionConnectedState};
use piper_sdk::prelude::*;
use std::fs::OpenOptions;
use std::io::{BufRead, Write};
use std::path::PathBuf;
use std::time::Duration;

#[derive(Parser, Debug)]
#[command(name = "record_pose")]
struct Args {
    #[cfg_attr(target_os = "linux", arg(long, default_value = "can0"))]
    #[cfg_attr(not(target_os = "linux"), arg(long, default_value = "auto"))]
    interface: String,

    #[arg(long, default_value = "1000000")]
    baud_rate: u32,

    /// Output file (appends if exists)
    #[arg(long, default_value = "poses.txt")]
    out: PathBuf,
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    piper_sdk::init_logger!();
    let args = Args::parse();

    println!("📝 record_pose — appends to {}", args.out.display());

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
        MotionConnectedPiper::Strict(MotionConnectedState::Standby(robot)) => loop_capture(robot, &args.out)?,
        MotionConnectedPiper::Soft(MotionConnectedState::Standby(robot)) => loop_capture(robot, &args.out)?,
        MotionConnectedPiper::Strict(MotionConnectedState::Maintenance(robot)) => {
            println!("⚠️  Maintenance → disabling all → Standby");
            let robot = robot.request_disable_all()?;
            let robot = robot.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            loop_capture(robot, &args.out)?
        },
        MotionConnectedPiper::Soft(MotionConnectedState::Maintenance(robot)) => {
            println!("⚠️  Maintenance → disabling all → Standby");
            let robot = robot.request_disable_all()?;
            let robot = robot.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            loop_capture(robot, &args.out)?
        },
    }
    Ok(())
}

fn loop_capture<Capability>(
    robot: Piper<Standby, Capability>,
    out_path: &PathBuf,
) -> std::result::Result<(), Box<dyn std::error::Error>>
where
    Capability: MotionCapability,
{
    println!("✅ Standby — motors disabled. Move arm by hand. Then:");
    println!("   • Type pose name + Enter to capture (e.g. `from_slot_1`)");
    println!("   • Type `q` + Enter to quit\n");

    let observer = robot.observer();
    let mut file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(out_path)?;

    // Header if file is empty
    if file.metadata()?.len() == 0 {
        writeln!(file, "# JengaBot pose recordings")?;
        writeln!(file, "# format: name j1 j2 j3 j4 j5 j6      (radians)")?;
    }

    let stdin = std::io::stdin();
    let mut count = 0;
    for line in stdin.lock().lines() {
        let line = line?;
        let name = line.trim();
        if name.is_empty() {
            continue;
        }
        if name == "q" || name == "quit" {
            println!("Quitting.");
            break;
        }
        if name.contains(char::is_whitespace) {
            println!("(name must be a single token, try again)");
            continue;
        }

        let positions = match observer.joint_positions() {
            Ok(p) => p,
            Err(e) => {
                println!("(read failed: {e} — try again)");
                continue;
            }
        };

        write!(file, "{}", name)?;
        for p in positions.iter() {
            write!(file, " {:.6}", p.0)?;
        }
        writeln!(file)?;
        file.flush()?;

        count += 1;
        print!("✓ captured `{}` (#{}):", name, count);
        for (i, p) in positions.iter().enumerate() {
            print!(" J{}={:.2}°", i + 1, p.to_deg().0);
        }
        println!("\n");
    }

    println!("Saved {} pose(s) to {}", count, out_path.display());
    Ok(())
}
