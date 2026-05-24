//! replay_ui — tiny web UI for replaying recorded poses on Bruce.
//!
//! Owns one PiPER SDK connection for the lifetime of the process.
//! Serves a static HTML page + a JSON state endpoint + Play/Repeat/Stop
//! HTTP POST controls.
//!
//! Run:
//!   sudo .../replay_ui --interface 0028002A4148570C20343133 \
//!                      --poses /Users/sameershah/learn/github/jengabot/poses.txt
//!   then open http://127.0.0.1:8080 in a browser.

use anyhow::{anyhow, Context, Result};
use axum::{
    extract::State,
    http::StatusCode,
    response::{Html, IntoResponse, Json},
    routing::{get, post},
    Router,
};
use clap::Parser;
use piper_sdk::client::state::{
    Active, DisableConfig, MotionCapability, PositionMode, PositionModeConfig, Standby,
};
use piper_sdk::client::types::RobotError;
use piper_sdk::client::{MotionConnectedPiper, MotionConnectedState};
use piper_sdk::prelude::*;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const STREAM_PERIOD: Duration = Duration::from_millis(20); // 50 Hz
const JOINT_LIMITS_DEG: [(f64, f64); 6] = [
    (-154.0, 154.0),
    (0.0, 195.0),
    (-175.0, 0.0),
    (-106.0, 106.0),
    (-75.0, 75.0),
    (-100.0, 100.0),
];

#[derive(Parser, Debug, Clone)]
#[command(name = "replay_ui")]
struct Args {
    #[cfg_attr(target_os = "linux", arg(long, default_value = "can0"))]
    #[cfg_attr(not(target_os = "linux"), arg(long, default_value = "auto"))]
    interface: String,
    #[arg(long, default_value = "1000000")]
    baud_rate: u32,
    #[arg(long, default_value = "poses.txt")]
    poses: PathBuf,
    /// Seconds per pose during playback
    #[arg(long, default_value = "3")]
    segment_secs: u64,
    /// Max joint change per 20ms tick (deg)
    #[arg(long, default_value = "1.5")]
    max_step_deg: f64,
    /// HTTP listen address
    #[arg(long, default_value = "127.0.0.1:8080")]
    listen: SocketAddr,
}

#[derive(Debug, Clone)]
struct Pose {
    name: String,
    joints: [f64; 6],
}

fn parse_poses(path: &PathBuf) -> Result<Vec<Pose>> {
    let f = File::open(path).with_context(|| format!("open {}", path.display()))?;
    let mut out = Vec::new();
    for line in BufReader::new(f).lines() {
        let line = line?;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let toks: Vec<&str> = line.split_whitespace().collect();
        if toks.len() != 7 {
            return Err(anyhow!("bad pose line (need name + 6 floats): {line}"));
        }
        let mut joints = [0.0; 6];
        for (i, t) in toks[1..].iter().enumerate() {
            joints[i] = t.parse()?;
        }
        out.push(Pose { name: toks[0].into(), joints });
    }
    Ok(out)
}

// === Shared state between control thread and HTTP handlers ===========

#[derive(Debug, Clone, Serialize)]
struct UiState {
    connected: bool,
    status: String,                  // "idle" | "playing" | "stopping" | "error: …"
    current_pose: Option<String>,    // name of pose currently streaming to
    elapsed_pose_secs: f64,
    pose_count_total: usize,
    pose_index: usize,
    iteration: u64,                  // how many times the playlist has run end-to-end
    joints_deg: [f64; 6],            // last observed joint angles
    poses: Vec<String>,              // playlist names (for UI)
}

#[derive(Debug, Clone, Deserialize, Default)]
struct PlayRequest {
    /// If set, only play these poses in this order. Else: all poses in file order.
    sequence: Option<Vec<String>>,
    /// Number of full passes through the sequence. None = 1, 0 = infinite (until /stop).
    repeats: Option<u64>,
}

#[derive(Debug, Clone)]
enum Command {
    Play { sequence: Option<Vec<String>>, repeats: Option<u64> },
    Stop,
}

struct AppState {
    cmd_tx: std::sync::mpsc::Sender<Command>,
    ui: Mutex<UiState>,
    stop_flag: Arc<AtomicBool>,
}

type SharedApp = Arc<AppState>;

// === Control thread: owns SDK, executes Play/Stop, updates UiState ===

fn clamp_joint(idx: usize, d: f64) -> f64 {
    let (lo, hi) = JOINT_LIMITS_DEG[idx];
    d.clamp(lo, hi)
}

fn clamp_step(prev: f64, target: f64, max_step: f64) -> f64 {
    let delta = (target - prev).clamp(-max_step, max_step);
    prev + delta
}

fn read_joints<C>(
    active: &Piper<Active<PositionMode>, C>,
) -> std::result::Result<[f64; 6], RobotError>
where
    C: MotionCapability,
{
    let p = active.observer().joint_positions()?;
    Ok([
        p[0].to_deg().0, p[1].to_deg().0, p[2].to_deg().0,
        p[3].to_deg().0, p[4].to_deg().0, p[5].to_deg().0,
    ])
}

fn run_control_thread(
    args: Args,
    poses: Vec<Pose>,
    ui: Arc<Mutex<UiState>>,
    cmd_rx: std::sync::mpsc::Receiver<Command>,
    stop_flag: Arc<AtomicBool>,
) -> Result<()> {
    // Connect once for the lifetime of the UI.
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

    {
        let mut s = ui.lock().unwrap();
        s.connected = true;
        s.status = "connecting…".into();
    }

    let motion = connected.require_motion()?;
    match motion {
        MotionConnectedPiper::Strict(MotionConnectedState::Standby(p)) => {
            drive_loop_soft(p, args, poses, ui, cmd_rx, stop_flag)
        }
        MotionConnectedPiper::Soft(MotionConnectedState::Standby(p)) => {
            drive_loop_soft(p, args, poses, ui, cmd_rx, stop_flag)
        }
        MotionConnectedPiper::Strict(MotionConnectedState::Maintenance(p)) => {
            let p = p.request_disable_all()?;
            let p = p.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            drive_loop_soft(p, args, poses, ui, cmd_rx, stop_flag)
        }
        MotionConnectedPiper::Soft(MotionConnectedState::Maintenance(p)) => {
            let p = p.request_disable_all()?;
            let p = p.wait_until_disabled(DisableConfig {
                timeout: Duration::from_secs(15),
                ..DisableConfig::default()
            })?;
            drive_loop_soft(p, args, poses, ui, cmd_rx, stop_flag)
        }
    }
}

fn drive_loop_soft<C>(
    standby: Piper<Standby, C>,
    args: Args,
    poses: Vec<Pose>,
    ui: Arc<Mutex<UiState>>,
    cmd_rx: std::sync::mpsc::Receiver<Command>,
    stop_flag: Arc<AtomicBool>,
) -> Result<()>
where
    C: MotionCapability,
{
    {
        let mut s = ui.lock().unwrap();
        s.status = "enabling position mode…".into();
    }
    let active = standby.enable_position_mode(PositionModeConfig {
        timeout: Duration::from_secs(15),
        ..PositionModeConfig::default()
    })?;
    {
        let mut s = ui.lock().unwrap();
        s.status = "idle".into();
    }

    // Initial pose for "hold" + step clamping.
    let mut prev = read_joints(&active).unwrap_or([0.0; 6]);
    {
        let mut s = ui.lock().unwrap();
        s.joints_deg = prev;
    }

    while !stop_flag.load(Ordering::Acquire) {
        // Block waiting for a command (with periodic timeout so we also publish state).
        let cmd = match cmd_rx.recv_timeout(Duration::from_millis(200)) {
            Ok(c) => c,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                // hold + observe
                if let Ok(j) = read_joints(&active) {
                    let mut s = ui.lock().unwrap();
                    s.joints_deg = j;
                }
                if let Err(e) = active.send_position_command(&joints_to_arr(prev)) {
                    let mut s = ui.lock().unwrap();
                    s.status = format!("error: {e}");
                }
                continue;
            }
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        };

        match cmd {
            Command::Stop => {
                let mut s = ui.lock().unwrap();
                s.status = "idle".into();
                s.current_pose = None;
            }
            Command::Play { sequence, repeats } => {
                let playlist: Vec<Pose> = match &sequence {
                    Some(names) => names
                        .iter()
                        .filter_map(|n| poses.iter().find(|p| &p.name == n).cloned())
                        .collect(),
                    None => poses.clone(),
                };
                if playlist.is_empty() {
                    let mut s = ui.lock().unwrap();
                    s.status = "error: empty playlist".into();
                    continue;
                }
                let total_passes = repeats.unwrap_or(1);
                let infinite = total_passes == 0;
                let pass_total = if infinite { 0 } else { total_passes };

                {
                    let mut s = ui.lock().unwrap();
                    s.poses = playlist.iter().map(|p| p.name.clone()).collect();
                    s.pose_count_total = playlist.len();
                    s.iteration = 0;
                    s.status = "playing".into();
                }

                let mut pass: u64 = 0;
                'play: loop {
                    if !infinite && pass >= pass_total {
                        break;
                    }
                    for (idx, pose) in playlist.iter().enumerate() {
                        // Drain any new commands (stop wins)
                        if let Ok(c) = cmd_rx.try_recv() {
                            if matches!(c, Command::Stop) {
                                let mut s = ui.lock().unwrap();
                                s.status = "idle".into();
                                s.current_pose = None;
                                break 'play;
                            }
                        }
                        if stop_flag.load(Ordering::Acquire) {
                            break 'play;
                        }

                        {
                            let mut s = ui.lock().unwrap();
                            s.current_pose = Some(pose.name.clone());
                            s.pose_index = idx;
                            s.elapsed_pose_secs = 0.0;
                            s.iteration = pass;
                        }

                        let start = Instant::now();
                        let segment = Duration::from_secs(args.segment_secs);
                        while start.elapsed() < segment {
                            if stop_flag.load(Ordering::Acquire) {
                                break 'play;
                            }
                            // Clamp + step-limit
                            let mut next = [0.0_f64; 6];
                            for i in 0..6 {
                                let c = clamp_joint(i, pose.joints[i]);
                                next[i] = clamp_step(prev[i], c, args.max_step_deg);
                            }
                            if let Err(e) = active.send_position_command(&joints_to_arr(next)) {
                                let mut s = ui.lock().unwrap();
                                s.status = format!("error: {e}");
                            }
                            prev = next;
                            if let Ok(j) = read_joints(&active) {
                                let mut s = ui.lock().unwrap();
                                s.joints_deg = j;
                                s.elapsed_pose_secs = start.elapsed().as_secs_f64();
                            }
                            thread::sleep(STREAM_PERIOD);
                        }
                    }
                    pass += 1;
                }
                {
                    let mut s = ui.lock().unwrap();
                    s.status = "idle".into();
                    s.current_pose = None;
                }
            }
        }
    }

    // Best-effort disable on shutdown
    let _ = active.disable(DisableConfig {
        timeout: Duration::from_secs(15),
        ..DisableConfig::default()
    });
    Ok(())
}

fn joints_to_arr(d: [f64; 6]) -> JointArray<Rad> {
    JointArray::from([
        Rad(d[0].to_radians()),
        Rad(d[1].to_radians()),
        Rad(d[2].to_radians()),
        Rad(d[3].to_radians()),
        Rad(d[4].to_radians()),
        Rad(d[5].to_radians()),
    ])
}

// === HTTP handlers ===========

async fn state_handler(State(app): State<SharedApp>) -> impl IntoResponse {
    let s = app.ui.lock().unwrap().clone();
    Json(s)
}

async fn play_handler(
    State(app): State<SharedApp>,
    body: Option<Json<PlayRequest>>,
) -> impl IntoResponse {
    let req = body.map(|b| b.0).unwrap_or_default();
    if app
        .cmd_tx
        .send(Command::Play { sequence: req.sequence, repeats: req.repeats })
        .is_err()
    {
        return (StatusCode::INTERNAL_SERVER_ERROR, "control thread dead");
    }
    (StatusCode::ACCEPTED, "play queued")
}

async fn stop_handler(State(app): State<SharedApp>) -> impl IntoResponse {
    let _ = app.cmd_tx.send(Command::Stop);
    (StatusCode::ACCEPTED, "stop queued")
}

async fn index_handler() -> impl IntoResponse {
    Html(include_str!("index.html"))
}

// === main ===========

#[tokio::main]
async fn main() -> Result<()> {
    piper_sdk::init_logger!();
    let args = Args::parse();
    let poses = parse_poses(&args.poses)?;
    eprintln!("loaded {} poses from {}", poses.len(), args.poses.display());

    let pose_names: Vec<String> = poses.iter().map(|p| p.name.clone()).collect();
    let ui = Arc::new(Mutex::new(UiState {
        connected: false,
        status: "starting…".into(),
        current_pose: None,
        elapsed_pose_secs: 0.0,
        pose_count_total: poses.len(),
        pose_index: 0,
        iteration: 0,
        joints_deg: [0.0; 6],
        poses: pose_names,
    }));
    let stop_flag = Arc::new(AtomicBool::new(false));
    {
        let stop_flag = stop_flag.clone();
        ctrlc::set_handler(move || stop_flag.store(true, Ordering::Release)).ok();
    }

    let (cmd_tx, cmd_rx) = std::sync::mpsc::channel();

    // Spawn SDK-owning control thread
    {
        let ui = ui.clone();
        let stop_flag = stop_flag.clone();
        let args = args.clone();
        thread::Builder::new()
            .name("piper-control".into())
            .spawn(move || {
                if let Err(e) = run_control_thread(args, poses, ui.clone(), cmd_rx, stop_flag) {
                    eprintln!("[control] fatal: {e}");
                    let mut s = ui.lock().unwrap();
                    s.status = format!("fatal: {e}");
                }
            })?;
    }

    let app_state = Arc::new(AppState {
        cmd_tx,
        ui: Mutex::new(ui.lock().unwrap().clone()),
        stop_flag: stop_flag.clone(),
    });

    // background task: copy from shared ui (control thread mutates) into app_state.ui
    {
        let ui_src = ui.clone();
        let app_state = app_state.clone();
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_millis(50)).await;
                let snap = ui_src.lock().unwrap().clone();
                *app_state.ui.lock().unwrap() = snap;
            }
        });
    }

    let router = Router::new()
        .route("/", get(index_handler))
        .route("/state", get(state_handler))
        .route("/play", post(play_handler))
        .route("/stop", post(stop_handler))
        .with_state(app_state);

    let listener = tokio::net::TcpListener::bind(&args.listen).await?;
    eprintln!("UI on http://{}", args.listen);
    axum::serve(listener, router).await?;
    Ok(())
}
