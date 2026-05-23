//! Gripper test — open / close / partial-position
//!
//! Connects, enables position mode (no joint motion), then cycles the
//! gripper. Streams gripper commands at 50 Hz like we do for joint
//! position commands.
//!
//! Run:
//!   sudo ./target/debug/examples/gripper_test --interface auto

use clap::Parser;
use piper_sdk::client::state::MotionCapability;
use piper_sdk::client::state::*;
use piper_sdk::client::{MotionConnectedPiper, MotionConnectedState};
use piper_sdk::prelude::*;
use std::time::{Duration, Instant};

const STREAM_PERIOD: Duration = Duration::from_millis(20); // 50 Hz

#[derive(Parser, Debug)]
#[command(name = "gripper_test")]
struct Args {
    #[cfg_attr(target_os = "linux", arg(long, default_value = "can0"))]
    #[cfg_attr(not(target_os = "linux"), arg(long, default_value = "auto"))]
    interface: String,

    #[arg(long, default_value = "1000000")]
    baud_rate: u32,

    /// Seconds to hold each gripper command (open / close / partial steps)
    #[arg(long, default_value = "2")]
    hold_secs: u64,
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    piper_sdk::init_logger!();
    let args = Args::parse();

    println!("🤏 Piper gripper test");
    println!("hold={}s\n", args.hold_secs);

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
        MotionConnectedPiper::Strict(MotionConnectedState::Standby(robot)) => run(robot, &args)?,
        MotionConnectedPiper::Soft(MotionConnectedState::Standby(robot)) => run(robot, &args)?,
        MotionConnectedPiper::Strict(MotionConnectedState::Maintenance(robot)) => {
            println!("⚠️  Maintenance → disabling all → Standby");
            let robot = robot.request_disable_all()?;
            let robot = robot.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            run(robot, &args)?
        },
        MotionConnectedPiper::Soft(MotionConnectedState::Maintenance(robot)) => {
            println!("⚠️  Maintenance → disabling all → Standby");
            let robot = robot.request_disable_all()?;
            let robot = robot.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            run(robot, &args)?
        },
    }
    Ok(())
}

fn run<Capability>(
    robot: Piper<Standby, Capability>,
    args: &Args,
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
    let hold = Duration::from_secs(args.hold_secs);

    let steps: &[(&str, f64, f64)] = &[
        ("OPEN (1.0, effort 0.3)", 1.0, 0.3),
        ("CLOSE (0.0, effort 0.3)", 0.0, 0.3),
        ("HALF OPEN (0.5, effort 0.3)", 0.5, 0.3),
        ("CLOSE HARDER (0.0, effort 0.7)", 0.0, 0.7),
        ("OPEN AGAIN (1.0, effort 0.3)", 1.0, 0.3),
    ];

    for (label, pos, effort) in steps {
        println!("\n→ {}", label);
        let start = Instant::now();
        while start.elapsed() < hold {
            robot.set_gripper(*pos, *effort)?;
            std::thread::sleep(STREAM_PERIOD);
        }
        let g = observer.gripper_state();
        println!("   observed position={:.3}  effort={:.3}  enabled={}",
            g.position, g.effort, g.enabled);
    }

    println!("\n🛑 Disabling...");
    let _robot = robot.disable(DisableConfig::default())?;
    println!("✅ Done");
    Ok(())
}
