//! Turns `cmd_vel` into a `robot.move` notification on `robotd`'s unix socket.
//!
//! The keyboard node publishes `[vx, vy, vyaw]` at 20 Hz; this node forwards
//! each one as a JSON-RPC notification (no `id`, no reply). With no
//! `ROBOTD_SOCKET` in the environment the command is only logged, which is what
//! a dataflow without a running daemon wants.
//!
//! State and perception are separate nodes: `dora-robotd-state` owns the
//! `robot.state` subscription, and this node stays a pure intent sink.

use dora_node_api::{DoraNode, Event};
use eyre::{Result, WrapErr, eyre};
use serde_json::json;
use std::io::Write;
use std::os::unix::net::UnixStream;
use std::path::PathBuf;

const CMD_VEL_IN: &str = "cmd_vel";

struct RobotdLink {
    path: Option<PathBuf>,
    stream: Option<UnixStream>,
    last_logged: Option<[f32; 3]>,
}

impl RobotdLink {
    fn from_env() -> Self {
        let path = std::env::var_os("ROBOTD_SOCKET").map(PathBuf::from);
        if path.is_none() {
            eprintln!("dora-robotd-wrapper: ROBOTD_SOCKET is not set; cmd_vel is only logged");
        }
        Self {
            path,
            stream: None,
            last_logged: None,
        }
    }

    fn send_move(&mut self, vx: f32, vy: f32, vyaw: f32) -> Result<()> {
        let Some(path) = self.path.clone() else {
            // The keyboard publishes every tick; log only when the command changes.
            let command = [vx, vy, vyaw];
            if self.last_logged != Some(command) {
                self.last_logged = Some(command);
                eprintln!("cmd_vel vx={vx:.2} vy={vy:.2} vyaw={vyaw:.2} (no ROBOTD_SOCKET)");
            }
            return Ok(());
        };

        let line = json!({
            "jsonrpc": "2.0",
            "method": "robot.move",
            "params": { "vx": vx, "vy": vy, "vyaw": vyaw },
        })
        .to_string()
            + "\n";

        if self.stream.is_none() {
            self.stream = Some(
                UnixStream::connect(&path)
                    .wrap_err_with(|| format!("connect to robotd at {}", path.display()))?,
            );
            eprintln!("dora-robotd-wrapper: connected to {}", path.display());
        }

        if self
            .stream
            .as_mut()
            .expect("connected above")
            .write_all(line.as_bytes())
            .is_ok()
        {
            return Ok(());
        }

        // A restarted daemon leaves the old connection dead; reconnect once.
        self.stream = None;
        let mut stream = UnixStream::connect(&path)
            .wrap_err_with(|| format!("reconnect to robotd at {}", path.display()))?;
        stream
            .write_all(line.as_bytes())
            .wrap_err("send robot.move after reconnect")?;
        self.stream = Some(stream);
        Ok(())
    }
}

fn main() -> Result<()> {
    // Keep the node alive for the life of the dataflow; this node has no
    // outputs of its own and sends its commands over the robotd socket.
    let (_node, mut events) = DoraNode::init_from_env()?;
    let mut robotd = RobotdLink::from_env();

    while let Some(event) = events.recv() {
        let Event::Input { id, data, .. } = event else {
            continue;
        };

        match id.as_str() {
            CMD_VEL_IN => {
                let command: Vec<f32> = (&data).try_into()?;
                if command.len() != 3 {
                    return Err(eyre!(
                        "cmd_vel must be [vx, vy, vyaw], got {} values",
                        command.len()
                    ));
                }
                robotd.send_move(command[0], command[1], command[2])?;
            }
            other => eprintln!("dora-robotd-wrapper: ignoring input `{other}`"),
        }
    }

    Ok(())
}
