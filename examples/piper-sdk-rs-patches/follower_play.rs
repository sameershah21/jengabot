//! follower_play — drive the connected arm from a stream of leader poses.
//!
//! Reads JSON lines from stdin (or a file via shell `< file.jsonl`) of the
//! form emitted by `leader_stream`:
//!
//!   {"t_us": 1234567890, "joints_deg": [j1, j2, j3, j4, j5, j6]}
//!
//! Streams the latest received target to the arm at 50 Hz. Lines that
//! arrive faster than 50 Hz simply update the shared target; lines that
//! arrive slower are tolerated up to the `--watchdog-ms` window, after
//! which the follower holds its last commanded pose rather than panicking.
//!
//! Safety:
//! - Per-tick step is clamped (`--max-step-deg`, default 2°). A jumpy
//!   input cannot whip a joint.
//! - Each joint is clamped to the PiPER-X joint limits documented in
//!   the AgileX manual.
//! - The arm holds the last good pose during stdin silence.
//!
//! Use case: Raymond (leader) -> Bruce (follower) bilateral teleop. For
//! now, run with no stdin connected to verify the SDK init path lights
//! up; the arm will hold its current pose.
//!
//! Run:
//!   # Live pipe (when leader is connected):
//!   sudo .../leader_stream | sudo .../follower_play
//!
//!   # From a recorded leader session:
//!   sudo .../follower_play < leader.jsonl
//!
//!   # Smoke test (no input, just confirm connect + hold):
//!   sudo .../follower_play --hold-seconds 5

use clap::Parser;
use piper_sdk::client::state::{DisableConfig, PositionModeConfig};
use piper_sdk::client::{MotionConnectedPiper, MotionConnectedState};
use piper_sdk::prelude::*;
use std::io::BufRead;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const STREAM_PERIOD: Duration = Duration::from_millis(20); // 50 Hz

// PiPER-X joint limits in degrees (from AgileX quick-start manual).
const JOINT_LIMITS_DEG: [(f64, f64); 6] = [
    (-154.0, 154.0), // J1 ±154°
    (0.0, 195.0),    // J2 0° ~ 195°
    (-175.0, 0.0),   // J3 -175° ~ 0°
    (-106.0, 106.0), // J4 ±106°
    (-75.0, 75.0),   // J5 ±75°
    (-100.0, 100.0), // J6 ±100°
];

#[derive(Parser, Debug)]
#[command(name = "follower_play")]
struct Args {
    #[cfg_attr(target_os = "linux", arg(long, default_value = "can0"))]
    #[cfg_attr(not(target_os = "linux"), arg(long, default_value = "auto"))]
    interface: String,

    #[arg(long, default_value = "1000000")]
    baud_rate: u32,

    /// Max change in any joint per 20ms tick, in degrees. Higher = snappier
    /// follower but bigger jolt risk on jumpy input.
    #[arg(long, default_value = "2.0")]
    max_step_deg: f64,

    /// Treat incoming joints_deg as DELTAS added to the seed pose, instead
    /// of as absolute joint angles. Use this when the leader's zero pose
    /// differs from the follower's (e.g. SO-101 leader → PiPER follower).
    #[arg(long, default_value = "false")]
    incremental: bool,

    /// Exponential smoothing coefficient on the target, [0..1).
    /// 0 = no smoothing (snappy, can feel choppy when leader rate is low).
    /// 0.9 = moderate smoothing (~200 ms time constant @ 50 Hz tick).
    /// 0.95 = heavier smoothing (~400 ms time constant). Best for
    /// low-rate leaders like SO-101's ~2 Hz board firmware.
    #[arg(long, default_value = "0.0")]
    smoothing: f64,

    /// If no leader update arrives in this many ms, hold last pose.
    #[arg(long, default_value = "500")]
    watchdog_ms: u64,

    /// If set, run with no stdin for this many seconds (smoke test that
    /// connect + position-mode + hold-at-current-pose all work). Then
    /// exit.
    #[arg(long)]
    hold_seconds: Option<u64>,
}

#[derive(Debug, Clone, Copy)]
struct TargetState {
    joints_deg: [f64; 6],
    /// Normalized gripper command 0.0 (closed) .. 1.0 (open). None = no
    /// gripper update from this line (preserve previous command).
    gripper: Option<f64>,
    last_update: Instant,
    initialized: bool,
}

fn clamp_joint(idx: usize, deg: f64) -> f64 {
    let (lo, hi) = JOINT_LIMITS_DEG[idx];
    deg.clamp(lo, hi)
}

fn clamp_step(prev_deg: f64, target_deg: f64, max_step: f64) -> f64 {
    let delta = (target_deg - prev_deg).clamp(-max_step, max_step);
    prev_deg + delta
}

/// Parse a JSON line. Manual parsing (no serde_json dep on the example),
/// tolerant of whitespace.
/// Expects: {"t_us":N,"joints_deg":[..,..,..,..,..,..],"gripper":<f>}
/// Returns (joints, optional gripper).
fn parse_line(line: &str) -> Option<([f64; 6], Option<f64>)> {
    let start = line.find("\"joints_deg\"")?;
    let after = &line[start..];
    let l = after.find('[')?;
    let r = after.find(']')?;
    let inside = &after[l + 1..r];
    let mut out = [0.0; 6];
    for (i, tok) in inside.split(',').enumerate() {
        if i >= 6 {
            return None;
        }
        out[i] = tok.trim().parse().ok()?;
    }

    // Optional gripper field. Cheap manual scan; no serde dep.
    let gripper = line.find("\"gripper\"").and_then(|gs| {
        let tail = &line[gs..];
        let colon = tail.find(':')?;
        // Pick characters that can form a float, including leading minus / decimal
        let value_str: String = tail[colon + 1..]
            .chars()
            .skip_while(|c| c.is_whitespace())
            .take_while(|c| c.is_ascii_digit() || *c == '.' || *c == '-' || *c == 'e' || *c == 'E' || *c == '+')
            .collect();
        value_str.parse::<f64>().ok()
    });

    Some((out, gripper))
}

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    piper_sdk::init_logger!();
    let args = Args::parse();

    eprintln!(
        "follower_play — 50Hz stream, max_step={:.2}°, watchdog={}ms{}",
        args.max_step_deg,
        args.watchdog_ms,
        args.hold_seconds.map(|s| format!(", hold_seconds={s} (smoke)")).unwrap_or_default(),
    );

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
    match motion {
        MotionConnectedPiper::Strict(MotionConnectedState::Standby(p)) => run(p, args),
        MotionConnectedPiper::Soft(MotionConnectedState::Standby(p)) => run(p, args),
        MotionConnectedPiper::Strict(MotionConnectedState::Maintenance(p)) => {
            eprintln!("Maintenance -> Standby (disable + wait)...");
            let p = p.request_disable_all()?;
            let p = p.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            run(p, args)
        }
        MotionConnectedPiper::Soft(MotionConnectedState::Maintenance(p)) => {
            eprintln!("Maintenance -> Standby (disable + wait)...");
            let p = p.request_disable_all()?;
            let p = p.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            run(p, args)
        }
    }
}

fn run<C>(
    standby: Piper<piper_sdk::client::state::Standby, C>,
    args: Args,
) -> std::result::Result<(), Box<dyn std::error::Error>>
where
    C: piper_sdk::client::state::MotionCapability,
{
    eprintln!("enabling position mode...");
    let active = standby.enable_position_mode(PositionModeConfig {
        timeout: Duration::from_secs(15),
        ..PositionModeConfig::default()
    })?;

    // Seed the target with the arm's current pose so the first tick doesn't snap.
    let observer = active.observer();
    let initial = {
        let mut got = None;
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            if let Ok(p) = observer.joint_positions() {
                got = Some([
                    p[0].to_deg().0,
                    p[1].to_deg().0,
                    p[2].to_deg().0,
                    p[3].to_deg().0,
                    p[4].to_deg().0,
                    p[5].to_deg().0,
                ]);
                break;
            }
            thread::sleep(Duration::from_millis(20));
        }
        got
    };
    let seed = initial.unwrap_or([0.0; 6]);
    eprintln!(
        "seeded follower at current pose: {:?}",
        seed.map(|x| (x * 100.0).round() / 100.0)
    );

    // Read Bruce's current gripper position so we can use it as the seed
    // for incremental-gripper mode. Leader emits deltas; follower adds
    // them to this seed and clamps to [0, 1].
    let gripper_seed: f64 = observer.gripper_state().position;
    eprintln!("seeded gripper at: {:.3}", gripper_seed);

    let shared = Arc::new(Mutex::new(TargetState {
        joints_deg: seed,
        gripper: None,
        last_update: Instant::now(),
        initialized: initial.is_some(),
    }));

    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = stop.clone();
        ctrlc::set_handler(move || stop.store(true, Ordering::SeqCst)).ok();
    }

    // stdin reader thread (only spawn if not in smoke-test mode)
    if args.hold_seconds.is_none() {
        let shared = shared.clone();
        let stop = stop.clone();
        thread::Builder::new()
            .name("stdin-reader".into())
            .spawn(move || {
                let stdin = std::io::stdin();
                for line in stdin.lock().lines() {
                    if stop.load(Ordering::SeqCst) {
                        break;
                    }
                    let Ok(line) = line else { break };
                    let line = line.trim();
                    if line.is_empty() {
                        continue;
                    }
                    if let Some((joints, gripper)) = parse_line(line) {
                        let mut s = shared.lock().unwrap();
                        s.joints_deg = joints;
                        if gripper.is_some() {
                            s.gripper = gripper;
                        }
                        s.last_update = Instant::now();
                        s.initialized = true;
                    } else {
                        eprintln!("(skipped malformed line: {})", &line[..line.len().min(80)]);
                    }
                }
                eprintln!("(stdin EOF or closed)");
            })?;
    }

    let watchdog = Duration::from_millis(args.watchdog_ms);
    let mut prev_sent = seed;
    // Smoothed target — slews exponentially toward the latest leader target
    // so low-rate leaders (like SO-101's ~2 Hz board firmware) don't produce
    // "burst-then-wait" follower motion.
    let mut target_smoothed = seed;
    let smoothing = args.smoothing.clamp(0.0, 0.999);
    let run_until: Option<Instant> = args
        .hold_seconds
        .map(|s| Instant::now() + Duration::from_secs(s));

    while !stop.load(Ordering::SeqCst) {
        if let Some(deadline) = run_until {
            if Instant::now() >= deadline {
                break;
            }
        }

        let (target, gripper_cmd) = {
            let s = shared.lock().unwrap();
            let t = if !s.initialized || s.last_update.elapsed() > watchdog {
                prev_sent
            } else if args.incremental {
                let mut t = [0.0; 6];
                for i in 0..6 {
                    t[i] = seed[i] + s.joints_deg[i];
                }
                t
            } else {
                s.joints_deg
            };
            (t, s.gripper)
        };

        // Apply exponential smoothing to the raw target before clamping.
        // target_smoothed = α * target_smoothed + (1-α) * target
        if smoothing > 0.0 {
            for i in 0..6 {
                target_smoothed[i] = smoothing * target_smoothed[i]
                    + (1.0 - smoothing) * target[i];
            }
        } else {
            target_smoothed = target;
        }

        // Clamp + step-limit
        let mut next = [0.0_f64; 6];
        for i in 0..6 {
            let clamped = clamp_joint(i, target_smoothed[i]);
            next[i] = clamp_step(prev_sent[i], clamped, args.max_step_deg);
        }

        let arr = JointArray::from([
            Rad(next[0].to_radians()),
            Rad(next[1].to_radians()),
            Rad(next[2].to_radians()),
            Rad(next[3].to_radians()),
            Rad(next[4].to_radians()),
            Rad(next[5].to_radians()),
        ]);
        if let Err(e) = active.send_position_command(&arr) {
            eprintln!("send err: {e}");
        }
        if let Some(g) = gripper_cmd {
            // In --incremental, leader emits a DELTA on the gripper;
            // follower adds it to the seed captured at startup so Bruce
            // starts wherever it physically is (not slammed to 0 or 1).
            // In absolute mode, leader emits the final 0..1 directly.
            // 0.5 effort = moderate; PiPER gripper saturates near 0.696
            // on the OPEN end per gripper_test findings.
            let target_g = if args.incremental {
                (gripper_seed + g).clamp(0.0, 1.0)
            } else {
                g.clamp(0.0, 1.0)
            };
            if let Err(e) = active.set_gripper(target_g, 0.5) {
                eprintln!("gripper err: {e}");
            }
        }
        prev_sent = next;
        thread::sleep(STREAM_PERIOD);
    }

    eprintln!("disabling...");
    let _ = active.disable(DisableConfig {
        timeout: Duration::from_secs(15),
        ..DisableConfig::default()
    })?;
    eprintln!("done.");
    Ok(())
}
