//! feedback_check — verify the SDK can read joint positions on the
//! current arm WITHOUT enabling/disabling any motion mode.
//!
//! Does the connection bring-up (so cold feedback flows), then reads the
//! observer's joint_positions() in a loop for `--seconds` seconds. Never
//! calls enable_position_mode or disable. Never transitions to Standby
//! either — the arm sits in whatever state it was in, so dropping the
//! handle does not trigger the SDK's auto-disable-on-Active path.
//!
//! Use this to confirm a protocol-ID patch (e.g. firmware S-V1.8-3 cold
//! feedback shifted from 0x2A* to 0x3A*) before any motion test.
//!
//! Run:
//!   sudo ./target/debug/examples/feedback_check
//!   sudo ./target/debug/examples/feedback_check --seconds 10

use clap::Parser;
use piper_sdk::client::{MotionConnectedPiper, MotionConnectedState};
use piper_sdk::prelude::*;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(name = "feedback_check")]
struct Args {
    #[cfg_attr(target_os = "linux", arg(long, default_value = "can0"))]
    #[cfg_attr(not(target_os = "linux"), arg(long, default_value = "auto"))]
    interface: String,

    #[arg(long, default_value = "1000000")]
    baud_rate: u32,

    #[arg(long, default_value = "5")]
    seconds: u64,
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    piper_sdk::init_logger!();
    let args = Args::parse();

    println!("feedback_check — read-only joint position observer, {}s", args.seconds);

    let connected = {
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
    println!("connected.");

    let motion = connected.require_motion()?;
    let start = Instant::now();
    let mut samples = 0u64;
    let mut errors = 0u64;
    let mut last_print = Instant::now();
    let read_once = || -> std::result::Result<JointArray<Rad>, piper_sdk::client::types::RobotError> {
        match &motion {
            MotionConnectedPiper::Strict(s) => match s {
                MotionConnectedState::Standby(p) => p.observer().joint_positions(),
                MotionConnectedState::Maintenance(p) => p.observer().joint_positions(),
            },
            MotionConnectedPiper::Soft(s) => match s {
                MotionConnectedState::Standby(p) => p.observer().joint_positions(),
                MotionConnectedState::Maintenance(p) => p.observer().joint_positions(),
            },
        }
    };
    while start.elapsed() < Duration::from_secs(args.seconds) {
        match read_once() {
            Ok(jp) => {
                samples += 1;
                if last_print.elapsed() >= Duration::from_millis(500) {
                    print!("[{:>5.1}s] joints:", start.elapsed().as_secs_f64());
                    for (i, p) in jp.iter().enumerate() {
                        print!(" J{}={:>7.2}°", i + 1, p.to_deg().0);
                    }
                    println!();
                    last_print = Instant::now();
                }
            }
            Err(e) => {
                errors += 1;
                if errors % 50 == 1 {
                    eprintln!("[{:>5.1}s] read err: {e}", start.elapsed().as_secs_f64());
                }
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    println!("\nsamples ok={}  err={}", samples, errors);
    // Intentionally do NOT transition states or disable: motion handle drops
    // from Maintenance/Standby (no motor power change) so the arm stays put.
    Ok(())
}
