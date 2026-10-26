//! Bridges `robotd` into the dora dataflow.
//!
//! Scaffold state: the daemon link is not wired yet. The node passes simulated
//! joint state back out as a command so `dataflows/sim.yml` runs end to end.
//! Replace the `TODO` below with the IPC client once the socket contract is
//! chosen; `duck_ipc_proto` already carries the request/response types.

use dora_node_api::{dora_core::config::DataId, DoraNode, Event, IntoArrow};
use eyre::{eyre, Result};

const JOINT_STATE_IN: &str = "joint_state";
const JOINT_COMMAND_OUT: &str = "joint_command";

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut forwarded = 0_u64;

    while let Some(event) = events.recv() {
        let Event::Input { id, metadata, data } = event else {
            continue;
        };

        if id.as_str() != JOINT_STATE_IN {
            eprintln!("dora-robotd-wrapper: ignoring input `{id}`");
            continue;
        }

        let positions: Vec<f32> = (&data).try_into()?;
        let expected = duck_ipc_proto::JOINT_NAMES.len();
        if positions.len() != expected {
            return Err(eyre!(
                "expected {expected} joint positions, got {}",
                positions.len()
            ));
        }

        // TODO: send `positions` to robotd and use the command it reports back.
        // Until then mirror the state so the simulated loop keeps ticking.
        node.send_output(
            DataId::from(JOINT_COMMAND_OUT.to_owned()),
            metadata.parameters,
            positions.into_arrow(),
        )?;

        forwarded += 1;
        if forwarded.is_multiple_of(100) {
            eprintln!("forwarded {forwarded} joint states ({expected} joints each)");
        }
    }

    Ok(())
}
