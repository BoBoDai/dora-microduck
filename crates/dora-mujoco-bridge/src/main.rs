//! Drives a MuJoCo model of the duck as a dora node.
//!
//! Scaffold state: no MuJoCo binding yet. The node runs a deterministic
//! synthetic gait over the joints in `duck_ipc_proto::JOINT_NAMES` so the
//! dataflow can be exercised before the simulator is wired in.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use eyre::Result;
use std::f32::consts::TAU;

const JOINT_COMMAND_IN: &str = "joint_command";
const JOINT_STATE_OUT: &str = "joint_state";

const STEP_SECONDS: f32 = 0.01;

struct Sim {
    elapsed: f32,
    targets: Vec<f32>,
}

impl Sim {
    fn new() -> Self {
        Self {
            elapsed: 0.0,
            targets: vec![0.0; duck_ipc_proto::JOINT_NAMES.len()],
        }
    }

    /// Advance one control step and return the measured joint angles.
    fn step(&mut self) -> Vec<f32> {
        self.elapsed += STEP_SECONDS;
        self.targets
            .iter()
            .enumerate()
            .map(|(joint, target)| {
                let phase = TAU * 0.5 * self.elapsed + joint as f32 * 0.15;
                target + 0.1 * phase.sin()
            })
            .collect()
    }

    fn apply(&mut self, command: Vec<f32>) {
        for (target, value) in self.targets.iter_mut().zip(command) {
            *target = value;
        }
    }
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut sim = Sim::new();
    let mut steps = 0_u64;

    while let Some(event) = events.recv() {
        let Event::Input { id, metadata, data } = event else {
            continue;
        };

        match id.as_str() {
            "tick" => {
                let state = sim.step();
                let joints = state.len();
                node.send_output(
                    DataId::from(JOINT_STATE_OUT.to_owned()),
                    metadata.parameters,
                    state.into_arrow(),
                )?;

                steps += 1;
                if steps.is_multiple_of(100) {
                    eprintln!("stepped {steps} times over {joints} joints");
                }
            }
            JOINT_COMMAND_IN => {
                let command: Vec<f32> = (&data).try_into()?;
                sim.apply(command);
            }
            other => eprintln!("dora-mujoco-bridge: ignoring input `{other}`"),
        }
    }

    Ok(())
}
