//! Publishes `robotd`'s `robot.state` stream into the dataflow.
//!
//! The upstream simulator already runs the real `robotd` and the real 50 Hz
//! control loop, so the graph reads the same state a robot would report: joint
//! angles, odometry, and the trunk IMU. The previous `joint_state` source was a
//! synthetic gait in `dora-mujoco-bridge`; this node replaces it so behaviour
//! nodes are not built on made-up measurements.
//!
//! `robotd` streams state as JSON-RPC notifications over the socket named by
//! `ROBOTD_SOCKET`. They arrive on a reader thread and overwrite the latest
//! sample; the dora loop publishes that sample on its own 20 ms tick, so a slow
//! downstream node cannot back-pressure the subscription. If the daemon
//! restarts, the reader reconnects and subscribes again.
//!
//! Outputs, all as Arrow `f32` arrays:
//!
//! - `joint_state`: 15 measured joint angles, in `duck_ipc_proto::JOINT_NAMES`
//!   order.
//! - `odom`: `[x, y, z, yaw]` in the odometry frame, metres and radians.
//! - `imu`: `[qw, qx, qy, qz, gx, gy, gz]` when the daemon reports an IMU.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use duck_ipc_proto::RobotState;
use eyre::{Result, WrapErr};
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

const TICK: &str = "tick";
const JOINT_STATE: &str = "joint_state";
const ODOM: &str = "odom";
const IMU: &str = "imu";
const SUBSCRIBE_HZ: u32 = 50;
const RECONNECT_DELAY: Duration = Duration::from_millis(500);

type Latest = Arc<Mutex<Option<Arc<RobotState>>>>;

/// Keeps a subscription alive, reconnecting when `robotd` goes away.
fn spawn_reader(path: PathBuf, latest: Latest) {
    thread::Builder::new()
        .name("robotd-state".to_owned())
        .spawn(move || {
            let mut failures = 0_u32;
            loop {
                match subscribe(&path, &latest) {
                    Ok(()) => {
                        failures = 0;
                        eprintln!(
                            "dora-robotd-state: robotd closed the subscription; reconnecting"
                        );
                    }
                    Err(err) => {
                        failures += 1;
                        // A daemon that is down for a while must not fill the log
                        // at two lines a second.
                        if failures == 1 || failures.is_multiple_of(20) {
                            eprintln!(
                                "dora-robotd-state: {err}; retrying in {} ms",
                                RECONNECT_DELAY.as_millis()
                            );
                        }
                    }
                }
                thread::sleep(RECONNECT_DELAY);
            }
        })
        .expect("failed to spawn the robotd-state reader");
}

/// One subscription, until the connection closes or fails.
fn subscribe(path: &Path, latest: &Mutex<Option<Arc<RobotState>>>) -> Result<()> {
    let mut stream = UnixStream::connect(path)
        .wrap_err_with(|| format!("connect to robotd at {}", path.display()))?;
    eprintln!("dora-robotd-state: subscribed to {}", path.display());

    let request = serde_json::json!({
        "jsonrpc": "2.0",
        "id": 1,
        "method": duck_ipc_proto::method::ROBOT_SUBSCRIBE,
        "params": { "hz": SUBSCRIBE_HZ },
    });
    stream.write_all(request.to_string().as_bytes())?;
    stream.write_all(b"\n")?;

    for line in BufReader::new(stream).lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        match state_from_line(&line) {
            Ok(Some(state)) => {
                *latest.lock().expect("state mutex poisoned") = Some(Arc::new(state));
            }
            // The `robot.subscribe` acknowledgement, or a reply to some other
            // request on the same connection.
            Ok(None) => {}
            Err(err) => eprintln!("dora-robotd-state: {err}"),
        }
    }

    Ok(())
}

/// A `robot.state` notification, or `None` for anything else on the line.
fn state_from_line(line: &str) -> Result<Option<RobotState>> {
    let value: serde_json::Value =
        serde_json::from_str(line).wrap_err("malformed JSON from robotd")?;
    if value.get("method").and_then(serde_json::Value::as_str)
        != Some(duck_ipc_proto::method::ROBOT_STATE)
    {
        return Ok(None);
    }

    let params = value
        .get("params")
        .cloned()
        .unwrap_or(serde_json::Value::Null);
    let state: RobotState =
        serde_json::from_value(params).wrap_err("failed to parse robot.state")?;
    Ok(Some(state))
}

fn joint_state(state: &RobotState) -> Option<Vec<f32>> {
    (state.joints.len() == duck_ipc_proto::JOINT_NAMES.len())
        .then(|| state.joints.iter().map(|value| *value as f32).collect())
}

fn odom(state: &RobotState) -> Vec<f32> {
    let [x, y, z] = state.odom.position;
    vec![x as f32, y as f32, z as f32, state.odom.yaw as f32]
}

fn imu(state: &RobotState) -> Option<Vec<f32>> {
    let imu = state.imu?;
    let [qw, qx, qy, qz] = imu.quat;
    let [gx, gy, gz] = imu.gyro;
    Some(vec![
        qw as f32, qx as f32, qy as f32, qz as f32, gx as f32, gy as f32, gz as f32,
    ])
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let latest: Latest = Arc::new(Mutex::new(None));

    match std::env::var_os("ROBOTD_SOCKET") {
        Some(path) => spawn_reader(PathBuf::from(path), Arc::clone(&latest)),
        None => {
            eprintln!("dora-robotd-state: ROBOTD_SOCKET is not set; no state will be published")
        }
    }

    let mut published = 0_u64;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == TICK => {
                let state = latest.lock().expect("state mutex poisoned").clone();
                let Some(state) = state else {
                    continue;
                };
                let Some(joints) = joint_state(&state) else {
                    if published == 0 {
                        eprintln!(
                            "dora-robotd-state: expected {} joints, got {}; not publishing",
                            duck_ipc_proto::JOINT_NAMES.len(),
                            state.joints.len()
                        );
                    }
                    continue;
                };

                let odom = odom(&state);
                if published == 0 {
                    eprintln!("dora-robotd-state: first state odom={odom:?} joints={joints:?}");
                }
                let count = published + 1;
                if count.is_multiple_of(100) {
                    eprintln!(
                        "dora-robotd-state: published {count} robot states; \
                         odom={odom:?} joints[..3]={:?}",
                        &joints[..3]
                    );
                }

                node.send_output(
                    DataId::from(JOINT_STATE.to_owned()),
                    metadata.parameters.clone(),
                    joints.into_arrow(),
                )?;
                node.send_output(
                    DataId::from(ODOM.to_owned()),
                    metadata.parameters.clone(),
                    odom.into_arrow(),
                )?;
                if let Some(imu) = imu(&state) {
                    node.send_output(
                        DataId::from(IMU.to_owned()),
                        metadata.parameters.clone(),
                        imu.into_arrow(),
                    )?;
                }

                published += 1;
            }
            Event::Input { id, .. } => eprintln!("dora-robotd-state: ignoring input `{id}`"),
            _ => {}
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(joints: Vec<f64>) -> RobotState {
        let targets = joints.clone();
        serde_json::from_value(serde_json::json!({
            "t": 1.0,
            "move": { "requested": [0.0, 0.0, 0.0], "applied": [0.0, 0.0, 0.0] },
            "head": [0.0, 0.0, 0.0, 0.0],
            "policy": "stand",
            "safety": { "fallen": false, "limp": false },
            "loop": { "hz": 50.0, "missed": 0 },
            "joints": joints,
            "targets": targets,
            "odom": { "position": [1.0, 2.0, 3.0], "yaw": 0.5 },
            "imu": { "gyro": [0.25, 0.5, 0.75], "quat": [1.0, 0.0, 0.0, 0.0] }
        }))
        .expect("valid robot state fixture")
    }

    #[test]
    fn parses_a_state_notification() {
        let line = serde_json::json!({
            "jsonrpc": "2.0",
            "method": "robot.state",
            "params": serde_json::to_value(state(vec![0.0; 15])).unwrap(),
        })
        .to_string();

        let parsed = state_from_line(&line).unwrap().expect("a state");
        assert_eq!(parsed.joints.len(), 15);
    }

    #[test]
    fn ignores_the_subscribe_acknowledgement() {
        let line = r#"{"jsonrpc":"2.0","id":1,"result":{"accepted":true}}"#;
        assert!(state_from_line(line).unwrap().is_none());
    }

    #[test]
    fn rejects_a_wrong_joint_count() {
        assert!(joint_state(&state(vec![0.0; 14])).is_none());
    }

    #[test]
    fn converts_odom_and_imu() {
        let state = state(vec![0.0; 15]);
        assert_eq!(odom(&state), vec![1.0, 2.0, 3.0, 0.5]);
        assert_eq!(
            imu(&state).unwrap(),
            vec![1.0, 0.0, 0.0, 0.0, 0.25, 0.5, 0.75]
        );
    }
}
