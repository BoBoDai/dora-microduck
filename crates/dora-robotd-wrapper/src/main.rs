//! Turns dora intent topics into `robotd` JSON-RPC calls.
//!
//! This is the single intent sink for the graph. `sim.yml` and `reactive.yml` only connect
//! `cmd_vel`, which becomes `robot.move`; `autod.yml` also connects `head`, `mouth`, `body`,
//! `sound` and `skill`. The wrapper never decides anything: it forwards last-value intents and
//! turns a `skill` into a `robot.do` request.
//!
//! With no `ROBOTD_SOCKET` every intent is logged once (and `cmd_vel` when it changes), which is
//! what a graph-only smoke test wants.

use dora_node_api::{DoraNode, Event};
use eyre::{Result, WrapErr, eyre};
use serde_json::{Value, json};
use std::collections::HashSet;
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;

const CMD_VEL: &str = "cmd_vel";
const HEAD: &str = "head";
const MOUTH: &str = "mouth";
const BODY: &str = "body";
const SOUND: &str = "sound";
const SKILL: &str = "skill";

struct IntentSink {
    path: Option<PathBuf>,
    stream: Option<UnixStream>,
    next_id: u64,
    last_cmd: Option<[f32; 3]>,
    logged: HashSet<&'static str>,
}

impl IntentSink {
    fn from_env() -> Self {
        let path = std::env::var_os("ROBOTD_SOCKET").map(PathBuf::from);
        if path.is_none() {
            eprintln!("dora-robotd-wrapper: ROBOTD_SOCKET is not set; intents are only logged");
        }
        Self {
            path,
            stream: None,
            next_id: 1,
            last_cmd: None,
            logged: HashSet::new(),
        }
    }

    fn log_once(&mut self, key: &'static str, message: impl std::fmt::Display) {
        if self.logged.insert(key) {
            eprintln!("dora-robotd-wrapper: {message} (no ROBOTD_SOCKET)");
        }
    }

    fn ensure(&mut self) -> Result<&mut UnixStream> {
        if self.stream.is_none() {
            let path = self
                .path
                .clone()
                .ok_or_else(|| eyre!("ROBOTD_SOCKET is not set"))?;
            let stream = UnixStream::connect(&path)
                .wrap_err_with(|| format!("connect to robotd at {}", path.display()))?;
            eprintln!("dora-robotd-wrapper: connected to {}", path.display());
            self.stream = Some(stream);
        }
        Ok(self.stream.as_mut().expect("connected above"))
    }

    fn write_line(&mut self, line: &str) -> Result<()> {
        if self.ensure()?.write_all(line.as_bytes()).is_ok() {
            return Ok(());
        }

        // A restarted daemon leaves the old connection dead; reconnect once.
        self.stream = None;
        self.ensure()?.write_all(line.as_bytes())?;
        Ok(())
    }

    fn notify(&mut self, method: &str, params: Value) -> Result<()> {
        if self.path.is_none() {
            self.log_once(leak(method), format!("{method} {params}"));
            return Ok(());
        }
        let line = json!({
            "jsonrpc": "2.0",
            "method": method,
            "params": params,
        })
        .to_string()
            + "\n";
        self.write_line(&line)
    }

    fn request(&mut self, method: &str, params: Value) -> Result<()> {
        if self.path.is_none() {
            self.log_once(leak(method), format!("{method} {params}"));
            return Ok(());
        }

        let id = self.next_id;
        self.next_id += 1;
        let line = json!({
            "jsonrpc": "2.0",
            "id": id,
            "method": method,
            "params": params,
        })
        .to_string()
            + "\n";
        self.write_line(&line)?;

        let mut reader = BufReader::new(
            self.stream
                .as_ref()
                .expect("write_line connected")
                .try_clone()?,
        );
        let mut answer = String::new();
        if reader.read_line(&mut answer)? == 0 {
            return Err(eyre!("robotd closed the connection after {method}"));
        }
        let response: Value =
            serde_json::from_str(&answer).wrap_err("unparsable robotd response")?;
        if let Some(error) = response.get("error") {
            eprintln!("dora-robotd-wrapper: {method} refused: {error}");
        } else if response
            .get("result")
            .and_then(|result| result.get("accepted"))
            .and_then(Value::as_bool)
            == Some(false)
        {
            eprintln!(
                "dora-robotd-wrapper: {method} not accepted: {}",
                response
                    .get("result")
                    .and_then(|result| result.get("reason"))
                    .and_then(Value::as_str)
                    .unwrap_or("no reason")
            );
        }
        Ok(())
    }

    fn move_command(&mut self, command: [f32; 3]) -> Result<()> {
        if self.path.is_none() {
            if self.last_cmd != Some(command) {
                self.last_cmd = Some(command);
                eprintln!(
                    "cmd_vel vx={:.2} vy={:.2} vyaw={:.2} (no ROBOTD_SOCKET)",
                    command[0], command[1], command[2]
                );
            }
            return Ok(());
        }
        self.notify(
            "robot.move",
            json!({ "vx": command[0], "vy": command[1], "vyaw": command[2] }),
        )
    }
}

/// `&'static str` for the log-once set without allocating a new string every tick.
fn leak(value: &str) -> &'static str {
    match value {
        "robot.head" => "head",
        "robot.mouth" => "mouth",
        "robot.pose" => "body",
        "robot.sound" => "sound",
        "robot.do" => "skill",
        _ => "other",
    }
}

fn sound_params(name: &str) -> Option<Value> {
    match name {
        "alarm" | "greet" | "inquire" | "peck" | "chirp" | "coo" => Some(json!({ "tag": name })),
        "wheee" | "wheee:1" => Some(json!({ "tag": "wheee", "hold": true })),
        "wheee:0" => Some(json!({ "tag": "wheee", "hold": false })),
        _ => None,
    }
}

fn floats(data: &dora_node_api::DoraArray) -> Result<Vec<f32>> {
    Vec::<f32>::try_from(data)
}

fn text(data: &dora_node_api::DoraArray) -> Result<String> {
    let bytes = Vec::<u8>::try_from(data)?;
    String::from_utf8(bytes).map_err(|err| eyre!("not UTF-8: {err}"))
}

fn main() -> Result<()> {
    // Keep the node alive for the life of the dataflow; all effects go out over robotd.
    let (_node, mut events) = DoraNode::init_from_env()?;
    let mut robotd = IntentSink::from_env();

    while let Some(event) = events.recv() {
        let Event::Input { id, data, .. } = event else {
            continue;
        };

        match id.as_str() {
            CMD_VEL => {
                let command = floats(&data)?;
                if command.len() != 3 {
                    return Err(eyre!(
                        "cmd_vel must be [vx, vy, vyaw], got {} values",
                        command.len()
                    ));
                }
                robotd.move_command([command[0], command[1], command[2]])?;
            }
            HEAD => {
                let head = floats(&data)?;
                if head.len() != 4 {
                    return Err(eyre!("head must be 4 values, got {}", head.len()));
                }
                robotd.notify(
                    "robot.head",
                    json!({
                        "neck_pitch": head[0],
                        "head_pitch": head[1],
                        "head_yaw": head[2],
                        "head_roll": head[3],
                    }),
                )?;
            }
            MOUTH => {
                let mouth = floats(&data)?;
                if mouth.len() != 1 {
                    return Err(eyre!("mouth must be 1 value, got {}", mouth.len()));
                }
                robotd.notify("robot.mouth", json!({ "open": mouth[0] }))?;
            }
            BODY => {
                let body = floats(&data)?;
                if body.len() != 4 {
                    return Err(eyre!("body must be 4 values, got {}", body.len()));
                }
                robotd.notify(
                    "robot.pose",
                    json!({
                        "z": body[0],
                        "pitch": body[1],
                        "roll": body[2],
                        "active": body[3] != 0.0,
                    }),
                )?;
            }
            SOUND => {
                let name = text(&data)?;
                let Some(params) = sound_params(&name) else {
                    robotd.log_once("unknown-sound", format!("unknown sound {name:?}"));
                    continue;
                };
                robotd.notify("robot.sound", params)?;
            }
            SKILL => {
                let skill = text(&data)?;
                robotd.request("robot.do", json!({ "skill": skill }))?;
            }
            other => eprintln!("dora-robotd-wrapper: ignoring input `{other}`"),
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_skill_is_a_request_with_a_waited_reply() {
        use std::os::unix::net::UnixListener;
        use std::thread;

        let dir = std::env::temp_dir().join(format!("dora-wrapper-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("robotd.sock");
        let listener = UnixListener::bind(&path).unwrap();

        let server = thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut reader = BufReader::new(stream.try_clone().unwrap());
            let mut line = String::new();
            reader.read_line(&mut line).unwrap();
            let request: Value = serde_json::from_str(&line).unwrap();
            assert_eq!(request["method"], "robot.do");
            assert_eq!(request["params"]["skill"], "ground_pick");
            let response = json!({
                "jsonrpc": "2.0",
                "id": request["id"].clone(),
                "result": { "accepted": true },
            });
            stream
                .write_all(format!("{response}\n").as_bytes())
                .unwrap();
        });

        let mut sink = IntentSink {
            path: Some(path),
            stream: None,
            next_id: 1,
            last_cmd: None,
            logged: HashSet::new(),
        };
        sink.request("robot.do", json!({ "skill": "ground_pick" }))
            .unwrap();
        server.join().unwrap();
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn sound_names_become_params() {
        assert_eq!(sound_params("chirp"), Some(json!({ "tag": "chirp" })));
        assert_eq!(
            sound_params("wheee:1"),
            Some(json!({ "tag": "wheee", "hold": true }))
        );
        assert_eq!(
            sound_params("wheee:0"),
            Some(json!({ "tag": "wheee", "hold": false }))
        );
        assert_eq!(sound_params("nope"), None);
    }
}
