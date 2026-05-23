//! Joint Sweep — exercise each joint J1..J6 individually
//!
//! Streams position commands at 50 Hz to sweep one joint at a time through
//! +Δ, return-to-start, -Δ, return-to-start, then moves on to the next joint.
//! All other joints are held at their initial pose for safety.
//!
//! Run:
//!   sudo ./target/debug/examples/joint_sweep --interface auto
//!
//! Optional flags:
//!   --amplitude-deg 20.0    sweep amplitude per joint (default 20°)
//!   --segment-secs 3        seconds per move segment (default 3)

use clap::Parser;
use piper_sdk::client::state::MotionCapability;
use piper_sdk::client::state::*;
use piper_sdk::client::types::RobotError;
use piper_sdk::client::{MotionConnectedPiper, MotionConnectedState};
use piper_sdk::prelude::*;
use std::time::{Duration, Instant};

const INITIAL_MONITOR_SNAPSHOT_TIMEOUT: Duration = Duration::from_millis(200);
const INITIAL_MONITOR_SNAPSHOT_POLL_INTERVAL: Duration = Duration::from_millis(5);
const STREAM_PERIOD: Duration = Duration::from_millis(20); // 50 Hz

#[derive(Parser, Debug)]
#[command(name = "joint_sweep")]
struct Args {
    #[cfg_attr(target_os = "linux", arg(long, default_value = "can0"))]
    #[cfg_attr(not(target_os = "linux"), arg(long, default_value = "auto"))]
    interface: String,

    #[arg(long, default_value = "1000000")]
    baud_rate: u32,

    /// Sweep amplitude per joint in degrees
    #[arg(long, default_value = "20.0")]
    amplitude_deg: f64,

    /// Seconds per move segment (out, back, etc.)
    #[arg(long, default_value = "3")]
    segment_secs: u64,
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    piper_sdk::init_logger!();
    let args = Args::parse();

    println!("🤖 Piper joint sweep");
    println!("amplitude={}°  segment={}s", args.amplitude_deg, args.segment_secs);
    println!();

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
    let home = wait_for_monitor_snapshot(
        INITIAL_MONITOR_SNAPSHOT_TIMEOUT,
        INITIAL_MONITOR_SNAPSHOT_POLL_INTERVAL,
        || observer.joint_positions(),
    )?;

    println!("📍 Home pose:");
    for (i, p) in home.iter().enumerate() {
        println!("   J{}: {:>7.2}°", i + 1, p.to_deg().0);
    }
    println!();

    let amp_rad = args.amplitude_deg.to_radians();
    let segment = Duration::from_secs(args.segment_secs);

    for j in 0..6 {
        println!("🔁 J{}: +{:.0}° → home → -{:.0}° → home",
            j + 1, args.amplitude_deg, args.amplitude_deg);
        stream_to(&robot, &home, j, amp_rad, segment)?;
        stream_to(&robot, &home, j, 0.0, segment)?;
        stream_to(&robot, &home, j, -amp_rad, segment)?;
        stream_to(&robot, &home, j, 0.0, segment)?;
    }

    println!("\n🛑 Disabling...");
    let _robot = robot.disable(DisableConfig::default())?;
    println!("✅ Done");
    Ok(())
}

fn stream_to<Capability>(
    robot: &Piper<Active<PositionMode>, Capability>,
    home: &JointArray<Rad>,
    joint_idx: usize,
    delta_rad: f64,
    duration: Duration,
) -> std::result::Result<(), Box<dyn std::error::Error>>
where
    Capability: MotionCapability,
{
    let mut target = *home;
    target[joint_idx] = Rad(home[joint_idx].0 + delta_rad);

    let start = Instant::now();
    while start.elapsed() < duration {
        robot.send_position_command(&target)?;
        std::thread::sleep(STREAM_PERIOD);
    }
    Ok(())
}

fn wait_for_monitor_snapshot<T, Read>(
    timeout: Duration,
    poll_interval: Duration,
    mut read: Read,
) -> std::result::Result<T, RobotError>
where
    Read: FnMut() -> std::result::Result<T, RobotError>,
{
    let start = Instant::now();
    loop {
        match read() {
            Ok(value) => return Ok(value),
            Err(
                RobotError::MonitorStateIncomplete { .. } | RobotError::MonitorStateStale { .. },
            ) => {},
            Err(other) => return Err(other),
        }
        if start.elapsed() >= timeout {
            return Err(RobotError::Timeout {
                timeout_ms: timeout.as_millis() as u64,
            });
        }
        let remaining = timeout.saturating_sub(start.elapsed());
        let sleep_duration = poll_interval.min(remaining);
        if sleep_duration.is_zero() {
            return Err(RobotError::Timeout {
                timeout_ms: timeout.as_millis() as u64,
            });
        }
        std::thread::sleep(sleep_duration);
    }
}
