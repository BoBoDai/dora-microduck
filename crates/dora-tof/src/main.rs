//! Publishes `tofd`'s 8×8 depth stream as a dora topic.
//!
//! `tofd` serves `tof.stream` on a Unix socket: JSON-RPC notifications carrying a `TofFrame`
//! (`t_ns`, `distance_mm[64]`, `status[64]`). This node connects, subscribes, and republishes
//! each frame as JSON bytes on `tof`. The brain parses it and pairs it with the nearest
//! `robot_state` sample by `t_ns`; keeping the frame opaque here means the wire contract lives
//! in `duck-ipc-proto`, not in two parsers.
//!
//! `TOF_SOCKET` names the endpoint. The simulated duck uses
//! `target/duck-sim/state/duck-a-tof.sock`; a real robot uses `/run/tofd/tof.sock`. Without it
//! the node logs and publishes nothing, like `camera` without `MEDIA_FRAME_SOCKET`.
//!
//! The socket is long-lived (unlike `media.frame`, which answers one frame per connection), so
//! this node keeps the stream open and only reconnects on error. A read timeout lets a dora
//! stop interrupt it instead of waiting for the next frame.

use std::io::{BufRead, BufReader, ErrorKind, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use duck_ipc_proto::{Request, Response, TofStreamResult};
use eyre::{Result, WrapErr, eyre};

const TOF: &str = "tof";
const RECONNECT_DELAY: Duration = Duration::from_millis(500);
const READ_TIMEOUT: Duration = Duration::from_millis(200);

fn main() -> Result<()> {
    let (node, mut events) = DoraNode::init_from_env()?;
    let Some(path) = std::env::var_os("TOF_SOCKET").map(PathBuf::from) else {
        eprintln!("dora-tof: TOF_SOCKET is not set; no depth will be published");
        while events.recv().is_some() {}
        return Ok(());
    };

    let running = Arc::new(AtomicBool::new(true));
    let source_running = Arc::clone(&running);
    let handle = thread::Builder::new()
        .name("dora-tof".to_owned())
        .spawn(move || source(path, node, source_running))
        .expect("failed to spawn the ToF reader");

    while let Some(event) = events.recv() {
        if matches!(event, Event::Stop(_)) {
            break;
        }
    }
    running.store(false, Ordering::Relaxed);
    let _ = handle.join();
    Ok(())
}

fn source(path: PathBuf, mut node: DoraNode, running: Arc<AtomicBool>) {
    let mut failures = 0_u32;
    while running.load(Ordering::Relaxed) {
        match stream(&path, &mut node, &running) {
            Ok(()) => {
                failures = 0;
                if running.load(Ordering::Relaxed) {
                    eprintln!("dora-tof: tofd closed the stream; reconnecting");
                }
            }
            Err(err) => {
                failures += 1;
                if failures == 1 || failures.is_multiple_of(100) {
                    eprintln!(
                        "dora-tof: {err}; retrying in {} ms",
                        RECONNECT_DELAY.as_millis()
                    );
                }
            }
        }
        thread::sleep(RECONNECT_DELAY);
    }
}

fn stream(path: &Path, node: &mut DoraNode, running: &AtomicBool) -> Result<()> {
    let stream = UnixStream::connect(path)
        .wrap_err_with(|| format!("connect to tofd at {}", path.display()))?;
    stream.set_read_timeout(Some(READ_TIMEOUT))?;
    let mut writer = stream.try_clone()?;
    let call = Request::call(
        duck_ipc_proto::Id::Number(1),
        &duck_ipc_proto::Call::TofStream,
    );
    let mut line = serde_json::to_vec(&call)?;
    line.push(b'\n');
    writer.write_all(&line)?;
    writer.flush()?;

    let mut reader = BufReader::new(stream);
    let mut raw = String::new();
    let mut frames = 0_u64;
    loop {
        if !running.load(Ordering::Relaxed) {
            return Ok(());
        }
        raw.clear();
        match reader.read_line(&mut raw) {
            Ok(0) => return Err(eyre!("tofd closed the depth stream")),
            Ok(_) => {}
            Err(err) if matches!(err.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {
                continue;
            }
            Err(err) => return Err(err.into()),
        }

        // Frames are notifications, not responses. Parse them first: a `Response` with every
        // optional field absent would otherwise swallow a frame.
        if let Ok(request) = serde_json::from_str::<Request>(&raw)
            && let Some(frame) = request.as_tof_frame()
        {
            let bytes = serde_json::to_vec(&frame)?;
            node.send_output(
                DataId::from(TOF.to_owned()),
                Default::default(),
                bytes.into_arrow(),
            )?;
            frames += 1;
            if frames == 1 || frames.is_multiple_of(150) {
                eprintln!(
                    "dora-tof: frame {frames}: {}x{} t_ns={}",
                    frame.rows, frame.cols, frame.t_ns
                );
            }
            continue;
        }

        if let Ok(response) = serde_json::from_str::<Response>(&raw) {
            if let Some(error) = &response.error {
                return Err(eyre!("tofd refused the stream: {}", error.message));
            }
            if response.id.is_some()
                && let Ok(ack) = response.result_as::<TofStreamResult>()
            {
                if !ack.accepted {
                    return Err(eyre!(
                        "tofd refused the stream: {}",
                        ack.unavailable.unwrap_or_default()
                    ));
                }
                eprintln!("dora-tof: connected to {}", path.display());
            }
        }
    }
}
