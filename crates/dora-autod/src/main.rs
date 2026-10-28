//! The upstream autonomous brain as a set of dora topics.
//!
//! This is the second version of the `autod` port. The first one was a single node that opened
//! `robotd` and `tofd` itself; this one consumes the graph:
//!
//! ```text
//! dora-robotd-state ──robot_state──┐
//!                                  ├──▶ dora-autod ──cmd_vel / head / mouth / body / sound / skill──▶ robotd-wrapper
//! dora-tof          ──tof──────────┘
//! ```
//!
//! `brain.rs` and `obstacle.rs` are still the upstream code (`origin/autonomous-mode`, commit
//! `df9a750`, Apache-2.0; see `LICENSE`). `main.rs` is now a pure dora adapter: it keeps the
//! last second of `robot_state` samples, pairs each ToF frame with the nearest sample by
//! `t_ns`, builds the upstream `World`, calls `Brain::step`, and publishes the resulting
//! intents. It opens no socket itself.
//!
//! The wrapper is the only intent sink. `cmd_vel` keeps the existing `robot.move` path; the
//! other topics are new inputs on `dora-robotd-wrapper` (`head`, `mouth`, `body`, `sound`,
//! `skill`). A `skill` is a `robot.do` request; the wrapper handles the reply.
//!
//! `robot_state` is a compact JSON object from `dora-robotd-state`, not the whole
//! `RobotState`: `t_ns`, policy, fallen, sitting, head joints, gravity, trunk height, pose and
//! the subscribe ack's abilities. `tof` is the serialized `TofFrame`, so the wire contract is
//! still `duck-ipc-proto`.
//!
//! Env: `AUTOD_MAX_SPEED` (0.30), `AUTOD_MAX_TURN` (1.50), `AUTOD_SEED`. The tick comes from
//! `dora/timer/millis/20`; there is no separate rate knob.
//!
//! Still not wired: microphone (`heard`/`petting`) and pad takeover (`move.source`). Petted
//! therefore cannot trigger, and a pad must not drive while this graph runs.

mod brain;
mod obstacle;

use std::time::{Duration, Instant};

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use duck_ipc_proto::TofFrame;
use eyre::Result;
use serde::Deserialize;

const TICK: &str = "tick";
const ROBOT_STATE: &str = "robot_state";
const TOF: &str = "tof";
const CMD_VEL: &str = "cmd_vel";
const HEAD: &str = "head";
const MOUTH: &str = "mouth";
const BODY: &str = "body";
const SOUND: &str = "sound";
const SKILL: &str = "skill";

/// A robot_state older than this means `robotd` stopped publishing; send nothing and let the
/// deadman stop the duck. The state node reconnects on its own, and this recovers when it does.
const STATE_TIMEOUT: Duration = Duration::from_millis(500);

/// The compact `robot_state` message published by `dora-robotd-state`.
#[derive(Clone, Debug, Deserialize)]
struct StateMessage {
    t_ns: u64,
    policy: String,
    fallen: bool,
    sitting: bool,
    head: [f64; 4],
    gravity: [f64; 3],
    #[serde(default)]
    trunk_height_m: Option<f64>,
    pose: [f64; 3],
    can_ground_pick: bool,
    can_sit_stand: bool,
}

impl StateMessage {
    fn sample(&self) -> obstacle::Sample {
        obstacle::Sample {
            t_ns: self.t_ns,
            head: self.head,
            gravity: self.gravity,
            trunk_height_m: self.trunk_height_m,
            pose: self.pose,
        }
    }
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

fn main() -> Result<()> {
    let _ = tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .try_init();

    let (mut node, mut events) = DoraNode::init_from_env()?;
    let max_speed = env_f64("AUTOD_MAX_SPEED", 0.30).max(0.0);
    let max_turn = env_f64("AUTOD_MAX_TURN", 1.50).max(0.0);
    let seed = env_u64(
        "AUTOD_SEED",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0x5eed, |d| d.as_nanos() as u64),
    );
    eprintln!("dora-autod: max_speed={max_speed:.2} max_turn={max_turn:.2}");

    let mut brain = brain::Brain::new(max_speed, max_turn, seed);
    let mut history = obstacle::History::default();
    let mut model = obstacle::Model::default();
    let mut latest: Option<StateMessage> = None;
    let mut last_state: Option<Instant> = None;
    let mut last_tick = Instant::now();
    let mut posing = false;
    let mut wheee_on = false;
    let mut idle_reason: Option<&'static str> = None;
    let mut first_state = true;
    let mut bad_states = 0_u64;
    let mut bad_tofs = 0_u64;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == TICK => {
                let now = Instant::now();
                let dt = now.duration_since(last_tick).as_secs_f64().min(0.1);
                last_tick = now;

                let Some(state) = latest.as_ref() else {
                    continue;
                };
                if last_state.is_none_or(|at| now.duration_since(at) > STATE_TIMEOUT) {
                    if idle_reason != Some("stale") {
                        eprintln!(
                            "dora-autod: no fresh robot_state — holding still (deadman protects)"
                        );
                        idle_reason = Some("stale");
                    }
                    continue;
                }
                if off(&state.policy) {
                    if idle_reason != Some("off") {
                        eprintln!(
                            "dora-autod: policy {} is not driving — waiting (Start turns it on)",
                            state.policy
                        );
                        idle_reason = Some("off");
                    }
                    continue;
                }
                idle_reason = None;

                let busy = state.fallen || !(standing(&state.policy) || state.sitting);
                let obstacle = model.summary(&state.sample());
                let world = brain::World {
                    busy,
                    petting: false,
                    heard: None,
                    can_ground_pick: state.can_ground_pick,
                    can_sit_stand: state.can_sit_stand,
                    sitting: state.sitting,
                    obstacle,
                    pose: state.pose,
                };
                let step = brain.step(dt, &world);

                // While a skill or a sit transition owns the robot, the brain freezes but the
                // graph sends nothing: last-writer-wins would otherwise fight the skill.
                if busy {
                    continue;
                }

                let parameters = metadata.parameters.clone();
                let cmd: Vec<f32> = step.cmd.iter().map(|value| *value as f32).collect();
                node.send_output(
                    DataId::from(CMD_VEL.to_owned()),
                    parameters.clone(),
                    cmd.into_arrow(),
                )?;
                // The brain's head pitch is "positive = up"; robotd's is the opposite.
                let head: Vec<f32> = [step.head[0], -step.head[1], step.head[2], step.head[3]]
                    .iter()
                    .map(|value| *value as f32)
                    .collect();
                node.send_output(
                    DataId::from(HEAD.to_owned()),
                    parameters.clone(),
                    head.into_arrow(),
                )?;
                node.send_output(
                    DataId::from(MOUTH.to_owned()),
                    parameters.clone(),
                    vec![step.mouth as f32].into_arrow(),
                )?;
                if step.body_active || posing {
                    let body: Vec<f32> = [
                        step.body[0],
                        step.body[1],
                        step.body[2],
                        if step.body_active { 1.0 } else { 0.0 },
                    ]
                    .iter()
                    .map(|value| *value as f32)
                    .collect();
                    node.send_output(
                        DataId::from(BODY.to_owned()),
                        parameters.clone(),
                        body.into_arrow(),
                    )?;
                    posing = step.body_active;
                }
                if let Some(sound) = step.sound {
                    let tag = match sound {
                        brain::Sound::Chirp => "chirp",
                        brain::Sound::Coo => "coo",
                    };
                    send_string(&mut node, SOUND, parameters.clone(), tag)?;
                }
                if step.wheee != wheee_on {
                    let tag = if step.wheee { "wheee:1" } else { "wheee:0" };
                    send_string(&mut node, SOUND, parameters.clone(), tag)?;
                    wheee_on = step.wheee;
                }
                if step.ground_pick {
                    send_string(&mut node, SKILL, parameters.clone(), "ground_pick")?;
                }
                if step.sit_toggle {
                    send_string(&mut node, SKILL, parameters.clone(), "sit_toggle")?;
                }
            }
            Event::Input { id, data, .. } if id.as_str() == ROBOT_STATE => {
                let Ok(bytes) = Vec::<u8>::try_from(&data) else {
                    bad_states += 1;
                    continue;
                };
                match serde_json::from_slice::<StateMessage>(&bytes) {
                    Ok(state) => {
                        if first_state {
                            eprintln!(
                                "dora-autod: first state policy={} sitting={} abilities ground_pick={} sit_stand={}",
                                state.policy,
                                state.sitting,
                                state.can_ground_pick,
                                state.can_sit_stand
                            );
                            first_state = false;
                        }
                        history.push(state.sample());
                        last_state = Some(Instant::now());
                        latest = Some(state);
                    }
                    Err(err) => {
                        bad_states += 1;
                        if bad_states == 1 || bad_states.is_multiple_of(100) {
                            eprintln!("dora-autod: bad robot_state: {err} (attempt {bad_states})");
                        }
                    }
                }
            }
            Event::Input { id, data, .. } if id.as_str() == TOF => {
                let Ok(bytes) = Vec::<u8>::try_from(&data) else {
                    bad_tofs += 1;
                    continue;
                };
                match serde_json::from_slice::<TofFrame>(&bytes) {
                    Ok(frame) => {
                        if let Some(at) = history.nearest(frame.t_ns) {
                            model.observe(&frame, &at);
                        }
                    }
                    Err(err) => {
                        bad_tofs += 1;
                        if bad_tofs == 1 || bad_tofs.is_multiple_of(100) {
                            eprintln!("dora-autod: bad tof frame: {err} (attempt {bad_tofs})");
                        }
                    }
                }
            }
            Event::Input { id, .. } => eprintln!("dora-autod: ignoring input `{id}`"),
            _ => {}
        }
    }

    // A graceful stop should not leave the last walk command latched until the deadman expires.
    let _ = node.send_output(
        DataId::from(CMD_VEL.to_owned()),
        Default::default(),
        vec![0.0_f32; 3].into_arrow(),
    );
    Ok(())
}

fn send_string(
    node: &mut DoraNode,
    output: &str,
    parameters: dora_node_api::MetadataParameters,
    value: &str,
) -> Result<()> {
    node.send_output(
        DataId::from(output.to_owned()),
        parameters,
        value.as_bytes().to_vec().into_arrow(),
    )?;
    Ok(())
}

/// Driving labels: the policy is on and the robot is the brain's to move.
fn standing(policy: &str) -> bool {
    matches!(policy, "walk" | "stand")
}

/// Not driving at all: limp, homing, or the policy is off. The brain sends nothing.
fn off(policy: &str) -> bool {
    matches!(policy, "held" | "homing" | "limp_fall" | "limp_pose")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_compact_state_and_builds_a_sample() {
        let json = serde_json::json!({
            "t_ns": 42,
            "policy": "walk",
            "fallen": false,
            "sitting": false,
            "head": [0.1, 0.2, 0.3, 0.4],
            "gravity": [0.0, 0.0, -1.0],
            "trunk_height_m": 0.116,
            "pose": [1.0, 2.0, 0.5],
            "can_ground_pick": true,
            "can_sit_stand": false
        });
        let state: StateMessage = serde_json::from_value(json).unwrap();
        let sample = state.sample();
        assert_eq!(sample.t_ns, 42);
        assert_eq!(sample.head, [0.1, 0.2, 0.3, 0.4]);
        assert_eq!(sample.pose, [1.0, 2.0, 0.5]);
        assert!(state.can_ground_pick && !state.can_sit_stand);
    }

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
    }
}
