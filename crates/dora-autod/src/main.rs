//! The upstream autonomous brain, running as a dora node.
//!
//! This is the port of `autod` (`origin/autonomous-mode`, commit `df9a750`): the pure
//! state machine in `brain.rs` and the ToF obstacle model in `obstacle.rs`, driven by the same
//! sockets the upstream daemon uses.
//!
//! ```text
//! tofd ──tof.stream (15 Hz)──┐
//!                            ▼
//! robotd ──robot.state──▶ dora-autod ──▶ robot.move · robot.head · robot.mouth
//!    ▲        (50 Hz)        │           robot.pose · robot.sound · robot.do
//!    └───────────────────────┘
//! ```
//!
//! Unlike the minimal `dora-brain` reactive loop, this brain owns the body: it stands still,
//! looks around, wanders, naps, dances, sneezes and runs the upstream energy/mood model. It is
//! a direct socket client like upstream `autod`; the first port deliberately keeps that shape
//! so the state machine can be verified before it is split across dora topics.
//!
//! Environment:
//!
//! - `ROBOTD_SOCKET` (required): `robotd`'s IPC socket. Without it the node logs and exits.
//! - `TOF_SOCKET` (optional): `tofd`'s socket. Without ToF the duck may look around but never
//!   advance, exactly as upstream.
//! - `AUTOD_HZ` (50), `AUTOD_MAX_SPEED` (0.30), `AUTOD_MAX_TURN` (1.50), `AUTOD_SEED`.
//! - `AUTOD_STAND_UP` is not implemented: the launcher/robot has already stood the duck up.
//!   Upstream's once-per-boot `robot.init`/`robot.enable` is a separate slice.
//!
//! Not yet wired: the microphone (`heard`/`petting`), the pad-takeover source from `robot.state`,
//! and dora topics for state/ToF. The brain accepts those inputs; they are `None`/`false` here,
//! so Petted and sound reactions do not fire.

mod brain;
mod obstacle;

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Duration, Instant};

use dora_node_api::{DoraNode, Event};
use duck_ipc_proto as proto;

const DEFAULT_HZ: u32 = 50;
const DEFAULT_MAX_SPEED: f64 = 0.30;
const DEFAULT_MAX_TURN: f64 = 1.50;

/// What the state thread last saw, and what the depth thread has made of the room.
#[derive(Default)]
struct Shared {
    history: obstacle::History,
    model: obstacle::Model,
    latest: Option<Latest>,
    /// The state stream ended: `robotd` is gone.
    lost: bool,
}

/// The fields of a `robot.state` frame the tick loop reads.
#[derive(Debug, Clone)]
struct Latest {
    sample: obstacle::Sample,
    policy: String,
    fallen: bool,
}

/// What the robot has, from the subscribe ack.
#[derive(Debug, Clone, Copy)]
struct Abilities {
    ground_pick: bool,
    sit_stand: bool,
}

/// The tick loop's fixed settings, read once from the environment.
#[derive(Debug, Clone, Copy)]
struct Settings {
    hz: u32,
    max_speed: f64,
    max_turn: f64,
    seed: u64,
}

fn env_path(name: &str) -> Option<PathBuf> {
    std::env::var_os(name).map(PathBuf::from)
}

fn env_f64(name: &str, default: f64) -> f64 {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse::<f64>().ok())
        .filter(|value| value.is_finite())
        .unwrap_or(default)
}

fn env_u64(name: &str, default: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .unwrap_or(default)
}

fn main() -> std::io::Result<()> {
    let _ = tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .try_init();

    // The node has no dora inputs or outputs yet, but it must still consume the event stream:
    // a dataflow stop is delivered as `Event::Stop`, and ignoring it makes dora wait out its
    // kill grace period. The drive loop runs on a thread so this one can see that event.
    let (_node, mut events) =
        DoraNode::init_from_env().map_err(|err| std::io::Error::other(err.to_string()))?;

    let Some(robotd) = env_path("ROBOTD_SOCKET") else {
        eprintln!("dora-autod: ROBOTD_SOCKET is not set; no brain will be published");
        return Ok(());
    };
    let tof = env_path("TOF_SOCKET");
    if tof.is_none() {
        eprintln!("dora-autod: TOF_SOCKET is not set; the duck will look but not walk");
    }

    let settings = Settings {
        hz: env_u64("AUTOD_HZ", u64::from(DEFAULT_HZ)).clamp(1, 200) as u32,
        max_speed: env_f64("AUTOD_MAX_SPEED", DEFAULT_MAX_SPEED).max(0.0),
        max_turn: env_f64("AUTOD_MAX_TURN", DEFAULT_MAX_TURN).max(0.0),
        seed: env_u64(
            "AUTOD_SEED",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0x5eed, |d| d.as_nanos() as u64),
        ),
    };

    let running = Arc::new(AtomicBool::new(true));
    let drive_running = Arc::clone(&running);
    let (result_tx, result_rx) = mpsc::channel();
    let handle = std::thread::spawn(move || {
        let result = run(robotd, tof, settings, drive_running);
        let _ = result_tx.send(result.is_ok());
        result
    });

    loop {
        if let Ok(ok) = result_rx.try_recv() {
            running.store(false, Ordering::Relaxed);
            let result = handle.join().unwrap_or(Ok(()));
            return if ok {
                result
            } else {
                Err(std::io::Error::other("autod drive loop stopped"))
            };
        }
        match events.recv_timeout(Duration::from_millis(50)) {
            Some(Event::Stop(_)) | None => break,
            Some(_) => {}
        }
    }

    eprintln!("dora-autod: dora is stopping the dataflow");
    running.store(false, Ordering::Relaxed);
    handle.join().unwrap_or(Ok(()))
}

fn run(
    robotd: PathBuf,
    tof: Option<PathBuf>,
    settings: Settings,
    running: Arc<AtomicBool>,
) -> std::io::Result<()> {
    let mut commands = UnixStream::connect(&robotd)?;
    let shared = Arc::new(Mutex::new(Shared::default()));

    let abilities = subscribe(&robotd, &shared)?;
    eprintln!(
        "dora-autod: autonomous — {:?}; max_speed={:.2} max_turn={:.2}",
        abilities, settings.max_speed, settings.max_turn
    );
    if let Some(tof) = tof {
        spawn_depth(tof, Arc::clone(&shared));
    }

    drive(&mut commands, &shared, abilities, settings, &running)
}

/// Open the state stream, and read it on a thread of its own from then on.
fn subscribe(socket: &Path, shared: &Arc<Mutex<Shared>>) -> std::io::Result<Abilities> {
    let mut stream = UnixStream::connect(socket)?;
    let call = proto::Request::call(
        proto::Id::Number(1),
        &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: None }),
    );
    let mut line = serde_json::to_vec(&call)?;
    line.push(b'\n');
    stream.write_all(&line)?;

    let mut reader = BufReader::new(stream);
    // The ack is the first line that is not a notification; a state frame can beat it.
    let ack = loop {
        let mut raw = String::new();
        if reader.read_line(&mut raw)? == 0 {
            return Err(std::io::Error::other("robotd closed the state stream"));
        }
        if let Ok(response) = serde_json::from_str::<proto::Response>(&raw)
            && response.id.is_some()
        {
            break response.result_as::<proto::SubscribeResult>().ok();
        }
    };
    let abilities = match ack {
        Some(ack) => Abilities {
            ground_pick: ack.ground_pick.is_some(),
            sit_stand: ack.sitstand.is_some(),
        },
        None => Abilities {
            ground_pick: false,
            sit_stand: false,
        },
    };

    let shared = Arc::clone(shared);
    std::thread::spawn(move || {
        let mut raw = String::new();
        loop {
            raw.clear();
            match reader.read_line(&mut raw) {
                Ok(0) | Err(_) => break,
                Ok(_) => {}
            }
            let Ok(request) = serde_json::from_str::<proto::Request>(&raw) else {
                continue;
            };
            let Some(state) = request.as_state() else {
                continue;
            };
            let latest = Latest {
                sample: obstacle::Sample::of(&state),
                policy: state.policy,
                fallen: state.safety.fallen,
            };
            let mut shared = shared.lock().unwrap_or_else(|p| p.into_inner());
            shared.history.push(latest.sample);
            shared.latest = Some(latest);
        }
        shared.lock().unwrap_or_else(|p| p.into_inner()).lost = true;
    });
    Ok(abilities)
}

/// Read depth frames into the obstacle model, forever. `tofd` being down is not fatal — the duck
/// simply stops advancing, because the path is never observed — and it is retried.
fn spawn_depth(socket: PathBuf, shared: Arc<Mutex<Shared>>) {
    std::thread::spawn(move || {
        let mut warned = false;
        loop {
            if let Err(err) = depth(&socket, &shared) {
                if !warned {
                    eprintln!(
                        "dora-autod: no depth frames from {}: {err}; the duck will look around but not walk until they come",
                        socket.display()
                    );
                    warned = true;
                }
                std::thread::sleep(Duration::from_secs(5));
            }
        }
    });
}

fn depth(socket: &Path, shared: &Mutex<Shared>) -> std::io::Result<()> {
    let mut stream = UnixStream::connect(socket)?;
    let call = proto::Request::call(proto::Id::Number(1), &proto::Call::TofStream);
    let mut line = serde_json::to_vec(&call)?;
    line.push(b'\n');
    stream.write_all(&line)?;

    let mut reader = BufReader::new(stream);
    let mut raw = String::new();
    let mut announced = false;
    loop {
        raw.clear();
        if reader.read_line(&mut raw)? == 0 {
            return Err(std::io::Error::other("tofd closed the depth stream"));
        }
        let Ok(request) = serde_json::from_str::<proto::Request>(&raw) else {
            // The ack, or a refusal: a sensor that is unavailable says so here.
            if let Ok(response) = serde_json::from_str::<proto::Response>(&raw)
                && let Ok(ack) = response.result_as::<proto::TofStreamResult>()
                && !ack.accepted
            {
                return Err(std::io::Error::other(format!(
                    "tofd refused the stream: {}",
                    ack.unavailable.unwrap_or_default()
                )));
            }
            continue;
        };
        let Some(frame) = request.as_tof_frame() else {
            continue;
        };
        if !announced {
            eprintln!("dora-autod: depth frames arriving — obstacle avoidance on");
            announced = true;
        }
        let mut shared = shared.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(at) = shared.history.nearest(frame.t_ns) {
            shared.model.observe(&frame, &at);
        }
    }
}

fn latest(shared: &Mutex<Shared>) -> (Option<Latest>, bool) {
    let shared = shared.lock().unwrap_or_else(|p| p.into_inner());
    (shared.latest.clone(), shared.lost)
}

/// Driving labels: the policy is on and the robot is the brain's to move.
fn standing(policy: &str) -> bool {
    matches!(policy, "walk" | "stand")
}

/// Parked on the sitstand network. The loop labels a robot sat down by `sit_toggle` `sit` for as
/// long as it stays down (`rise` is the way back up, and busy); `seated` is the same network as the
/// driving state names it.
fn seated(policy: &str) -> bool {
    matches!(policy, "sit" | "seated")
}

/// Not driving at all: limp, homing, or the policy is off. The brain sends nothing — it never
/// re-enables what somebody else disabled.
fn off(policy: &str) -> bool {
    matches!(policy, "held" | "homing" | "limp_fall" | "limp_pose")
}

/// The tick loop.
fn drive(
    commands: &mut UnixStream,
    shared: &Mutex<Shared>,
    abilities: Abilities,
    settings: Settings,
    running: &AtomicBool,
) -> std::io::Result<()> {
    let mut brain = brain::Brain::new(settings.max_speed, settings.max_turn, settings.seed);
    let period = Duration::from_secs_f64(1.0 / f64::from(settings.hz));
    let mut last_tick = Instant::now();
    let mut posing = false;
    let mut wheee_on = false;
    let mut idle_reason: Option<&'static str> = None;
    let mut frame: Vec<u8> = Vec::with_capacity(512);
    let mut next_id = 2_u64;

    while running.load(Ordering::Relaxed) {
        let tick = Instant::now();
        let dt = tick.duration_since(last_tick).as_secs_f64().min(0.1);
        last_tick = tick;

        let (state, lost) = latest(shared);
        if lost {
            return Err(std::io::Error::other("robotd closed the state stream"));
        }
        let Some(state) = state else {
            std::thread::sleep(period);
            continue;
        };

        if off(&state.policy) {
            if idle_reason != Some("off") {
                eprintln!(
                    "dora-autod: policy {} is not driving — waiting (Start turns it on)",
                    state.policy
                );
                idle_reason = Some("off");
            }
            sleep_rest(period, tick);
            continue;
        }
        idle_reason = None;

        let sitting = seated(&state.policy);
        let busy = state.fallen || !(standing(&state.policy) || sitting);
        let obstacle = {
            let shared = shared.lock().unwrap_or_else(|p| p.into_inner());
            shared.model.summary(&state.sample)
        };
        let world = brain::World {
            busy,
            petting: false,
            heard: None,
            can_ground_pick: abilities.ground_pick,
            can_sit_stand: abilities.sit_stand,
            sitting,
            obstacle,
            pose: state.sample.pose,
        };
        let step = brain.step(dt, &world);

        if busy {
            // Something else owns the robot. A head target now would yank it mid-skill.
            sleep_rest(period, tick);
            continue;
        }

        // One write for the tick's continuous intents: they describe one instant.
        frame.clear();
        let mut push = |call: &proto::Call| -> std::io::Result<()> {
            serde_json::to_writer(&mut frame, &proto::Request::notify(call))?;
            frame.push(b'\n');
            Ok(())
        };
        push(&proto::Call::RobotMove(proto::MoveParams {
            vx: step.cmd[0],
            vy: step.cmd[1],
            vyaw: step.cmd[2],
        }))?;
        push(&proto::Call::RobotHead(proto::HeadParams {
            neck_pitch: step.head[0],
            // The brain's pitch is "positive = up"; the alpha's +head_pitch looks down.
            head_pitch: -step.head[1],
            head_yaw: step.head[2],
            head_roll: step.head[3],
        }))?;
        push(&proto::Call::RobotMouth(proto::MouthParams {
            open: step.mouth,
        }))?;
        // Body pose works on the velstand gait as well as on a standing network, so it goes out
        // whenever the brain poses, and once more with `active: false` when it stops.
        if step.body_active || posing {
            push(&proto::Call::RobotPose(proto::PoseParams {
                z: step.body[0],
                pitch: step.body[1],
                roll: step.body[2],
                active: step.body_active,
            }))?;
            posing = step.body_active;
        }
        if let Some(sound) = step.sound {
            push(&proto::Call::RobotSound(proto::SoundParams {
                tag: match sound {
                    brain::Sound::Chirp => proto::SoundTag::Chirp,
                    brain::Sound::Coo => proto::SoundTag::Coo,
                },
                hold: None,
            }))?;
        }
        if step.wheee || wheee_on {
            push(&proto::Call::RobotSound(proto::SoundParams {
                tag: proto::SoundTag::Wheee,
                hold: Some(step.wheee),
            }))?;
            wheee_on = step.wheee;
        }
        commands.write_all(&frame)?;
        commands.flush()?;

        if step.ground_pick {
            skill(commands, &mut next_id, "ground_pick")?;
        }
        if step.sit_toggle {
            skill(commands, &mut next_id, "sit_toggle")?;
        }

        sleep_rest(period, tick);
    }

    // A graceful dataflow stop should not leave the last walk command latched until the
    // deadman expires.
    let mut frame = Vec::new();
    serde_json::to_writer(
        &mut frame,
        &proto::Request::notify(&proto::Call::RobotMove(proto::MoveParams::default())),
    )?;
    frame.push(b'\n');
    commands.write_all(&frame)?;
    commands.flush()?;
    Ok(())
}

fn sleep_rest(period: Duration, tick: Instant) {
    if let Some(remaining) = period.checked_sub(tick.elapsed()) {
        std::thread::sleep(remaining);
    }
}

/// Ask for a skill, answered: a refusal is logged with its reason by [`request`].
fn skill(commands: &mut UnixStream, next_id: &mut u64, name: &str) -> std::io::Result<()> {
    let call = proto::Call::RobotDo(proto::DoParams {
        skill: name.to_owned(),
    });
    request(commands, next_id, &call).map(|_| ())
}

/// Send a request and read its answer, on a connection that carries nothing else.
fn request(
    stream: &mut UnixStream,
    next_id: &mut u64,
    call: &proto::Call,
) -> std::io::Result<Option<proto::Response>> {
    let id = proto::Id::Number(*next_id);
    *next_id += 1;
    let mut line = serde_json::to_vec(&proto::Request::call(id, call))?;
    line.push(b'\n');
    stream.write_all(&line)?;
    stream.flush()?;

    let mut reader = BufReader::new(stream.try_clone()?);
    let mut answer = String::new();
    if reader.read_line(&mut answer)? == 0 {
        return Err(std::io::Error::other("robotd closed the connection"));
    }
    match serde_json::from_str::<proto::Response>(&answer) {
        Ok(response) => {
            if let Some(error) = &response.error {
                eprintln!("dora-autod: refused: {} ({})", error.message, error.code);
            } else if let Ok(result) = response.result_as::<proto::IntentResult>()
                && !result.accepted
            {
                eprintln!("dora-autod: not accepted: {:?}", result.reason);
            }
            Ok(Some(response))
        }
        Err(err) => {
            eprintln!(
                "dora-autod: unparsable answer: {err}; raw={}",
                answer.trim()
            );
            Ok(None)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The labels the loop publishes, sorted into the three things the brain does about them.
    #[test]
    fn policy_labels_sort_into_drive_wait_and_off() {
        for label in ["walk", "stand"] {
            assert!(standing(label) && !off(label), "{label}");
        }
        for label in ["held", "homing", "limp_fall", "limp_pose"] {
            assert!(off(label) && !standing(label), "{label}");
        }
        for label in ["ground_pick", "rise", "roulade", "roulade:unwind", "sit"] {
            assert!(!off(label) && !standing(label), "{label}");
        }
        assert!(seated("sit") && !seated("rise") && !seated("walk"));
    }
}
