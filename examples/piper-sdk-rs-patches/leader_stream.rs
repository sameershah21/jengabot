//! leader_stream — continuously emit joint angles of the connected arm.
//!
//! Use case: bilateral teleop with the *leader* arm. The operator drags the
//! arm by hand (drag-teach mode, single-click the button between J5/J6 so
//! the LED is solid green). This binary reads joint positions via the SDK
//! observer and emits one JSON line per tick to stdout. A follower process
//! reads those lines and commands its own arm to match.
//!
//! Read-only. Never enables/disables motors, never changes control mode —
//! so dropping the handle does not trigger any auto-disable, no drop risk.
//!
//! Stdout format (one JSON object per line):
//!   {"t_us": 123456789, "joints_deg": [j1, j2, j3, j4, j5, j6]}
//!
//! Run:
//!   sudo ./target/debug/examples/leader_stream
//!   sudo ./target/debug/examples/leader_stream --rate 100 --out leader.jsonl
//!   sudo ./target/debug/examples/leader_stream --human   # readable, not JSON

use clap::Parser;
use piper_sdk::client::{MotionConnectedPiper, MotionConnectedState};
use piper_sdk::prelude::*;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[derive(Parser, Debug)]
#[command(name = "leader_stream")]
struct Args {
    #[cfg_attr(target_os = "linux", arg(long, default_value = "can0"))]
    #[cfg_attr(not(target_os = "linux"), arg(long, default_value = "auto"))]
    interface: String,

    #[arg(long, default_value = "1000000")]
    baud_rate: u32,

    /// Stream rate in Hz
    #[arg(long, default_value = "50")]
    rate: u32,

    /// Optional file to append JSON lines to (in addition to stdout)
    #[arg(long)]
    out: Option<std::path::PathBuf>,

    /// Print human-readable angles instead of JSON
    #[arg(long, default_value = "false")]
    human: bool,
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    piper_sdk::init_logger!();
    let args = Args::parse();

    let period = Duration::from_secs_f64(1.0 / args.rate as f64);

    eprintln!(
        "leader_stream — {} Hz, format={}{}",
        args.rate,
        if args.human { "human" } else { "json" },
        args.out
            .as_ref()
            .map(|p| format!(", out={}", p.display()))
            .unwrap_or_default(),
    );
    eprintln!(
        "drag-teach the arm by hand (single-click button between J5/J6 → LED green)."
    );
    eprintln!("Ctrl+C to stop.\n");

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
            let builder = PiperBuilder::new()
                .baud_rate(args.baud_rate)
                .feedback_timeout(Duration::from_secs(10))
                .firmware_timeout(Duration::from_secs(5));
            let builder = if args.interface == "auto" {
                builder.gs_usb_auto()
            } else {
                builder.gs_usb_serial(&args.interface)
            };
            builder.build()?
        }
    };
    eprintln!("connected.");

    let motion = connected.require_motion()?;

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

    let mut sink: Option<BufWriter<File>> = if let Some(path) = &args.out {
        Some(BufWriter::new(File::create(path)?))
    } else {
        None
    };

    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = stop.clone();
        ctrlc::set_handler(move || stop.store(true, Ordering::SeqCst)).ok();
    }

    let stdout = std::io::stdout();
    let mut out = stdout.lock();

    let mut next_tick = Instant::now();
    while !stop.load(Ordering::SeqCst) {
        let now = Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        }
        next_tick += period;

        let positions = match read_once() {
            Ok(p) => p,
            Err(_) => continue,
        };

        let t_us = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);

        if args.human {
            writeln!(
                out,
                "[{:>13} us] {:>7.2}° {:>7.2}° {:>7.2}° {:>7.2}° {:>7.2}° {:>7.2}°",
                t_us,
                positions[0].to_deg().0,
                positions[1].to_deg().0,
                positions[2].to_deg().0,
                positions[3].to_deg().0,
                positions[4].to_deg().0,
                positions[5].to_deg().0,
            )?;
        } else {
            writeln!(
                out,
                "{{\"t_us\":{},\"joints_deg\":[{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}]}}",
                t_us,
                positions[0].to_deg().0,
                positions[1].to_deg().0,
                positions[2].to_deg().0,
                positions[3].to_deg().0,
                positions[4].to_deg().0,
                positions[5].to_deg().0,
            )?;
        }
        out.flush().ok();

        if let Some(f) = sink.as_mut() {
            writeln!(
                f,
                "{{\"t_us\":{},\"joints_deg\":[{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}]}}",
                t_us,
                positions[0].to_deg().0,
                positions[1].to_deg().0,
                positions[2].to_deg().0,
                positions[3].to_deg().0,
                positions[4].to_deg().0,
                positions[5].to_deg().0,
            )?;
        }
    }

    if let Some(f) = sink.as_mut() {
        f.flush().ok();
    }
    eprintln!("\nstopped.");
    Ok(())
}
