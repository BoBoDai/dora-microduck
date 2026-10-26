//! Terminal teleoperation as a dora node.
//!
//! dora gives nodes a null stdin, so keys are read from `/dev/tty`. Every timer
//! tick publishes `cmd_vel` as `[vx, vy, vyaw]`. A terminal cannot report key
//! release, so a key counts as held for as long as it repeats; 400 ms without a
//! recognised key publishes zero, which stops the duck on release.
//!
//! Run the dataflow from a terminal: without a controlling terminal the node
//! logs a warning and publishes zeros instead of failing.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use eyre::Result;
use std::fs::{File, OpenOptions};
use std::io::{self, Read};
use std::os::fd::{AsRawFd, RawFd};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const CMD_VEL: &str = "cmd_vel";
/// Longer than a key repeat gap, short enough to stop promptly on release.
const HOLD: Duration = Duration::from_millis(400);
/// Above the ~0.25 m/s deadband the MuJoCo twin shows at low commands.
const VX: f32 = 0.30;
const VY: f32 = 0.15;
const VYAW: f32 = 0.50;

#[derive(Default)]
struct Keys {
    vx: f32,
    vy: f32,
    vyaw: f32,
    last: Option<Instant>,
    quit: bool,
    /// 0 = idle, 1 = saw ESC, 2 = saw `ESC [`.
    escape: u8,
}

impl Keys {
    fn apply(&mut self, byte: u8) {
        if self.escape == 1 {
            self.escape = if byte == b'[' { 2 } else { 0 };
            return;
        }
        if self.escape == 2 {
            self.escape = 0;
            match byte {
                b'A' => self.set_vx(VX),
                b'B' => self.set_vx(-VX),
                // Arrow keys: left turns left, which `MoveParams` spells positive.
                b'C' => self.set_vyaw(-VYAW),
                b'D' => self.set_vyaw(VYAW),
                _ => {}
            }
            return;
        }

        match byte {
            0x1b => self.escape = 1,
            b'w' | b'W' => self.set_vx(VX),
            b's' | b'S' => self.set_vx(-VX),
            b'a' | b'A' => self.set_vy(VY),
            b'd' | b'D' => self.set_vy(-VY),
            b' ' => self.stop(),
            b'q' | b'Q' | 0x03 => self.quit = true,
            _ => {}
        }
    }

    fn set_vx(&mut self, value: f32) {
        self.vx = value;
        self.touch();
    }

    fn set_vy(&mut self, value: f32) {
        self.vy = value;
        self.touch();
    }

    fn set_vyaw(&mut self, value: f32) {
        self.vyaw = value;
        self.touch();
    }

    fn touch(&mut self) {
        self.last = Some(Instant::now());
    }

    fn stop(&mut self) {
        self.vx = 0.0;
        self.vy = 0.0;
        self.vyaw = 0.0;
        self.last = None;
    }

    /// The command for this tick, zeroed once the last key has gone stale.
    fn command(&mut self) -> [f32; 3] {
        if self.quit {
            return [0.0; 3];
        }
        let held = self.last.is_some_and(|at| at.elapsed() < HOLD);
        if !held {
            self.stop();
        }
        [self.vx, self.vy, self.vyaw]
    }
}

/// Raw input, normal output: dora's own log lines share this terminal and still
/// need `ONLCR`, or every newline becomes a staircase.
struct RawMode {
    fd: RawFd,
    original: libc::termios,
}

impl RawMode {
    fn enable(fd: RawFd) -> io::Result<Self> {
        let mut original: libc::termios = unsafe { std::mem::zeroed() };
        if unsafe { libc::tcgetattr(fd, &mut original) } != 0 {
            return Err(io::Error::last_os_error());
        }
        // A node started in the background — under `script`, or from a service —
        // would otherwise be stopped by SIGTTOU/SIGTTIN on the first termios
        // write or read. Ignoring them keeps the node running; a background read
        // simply fails and the reader exits.
        unsafe {
            libc::signal(libc::SIGTTOU, libc::SIG_IGN);
            libc::signal(libc::SIGTTIN, libc::SIG_IGN);
        }
        let mut raw = original;
        unsafe { libc::cfmakeraw(&mut raw) };
        raw.c_oflag |= libc::OPOST | libc::ONLCR;
        if unsafe { libc::tcsetattr(fd, libc::TCSANOW, &raw) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(Self { fd, original })
    }
}

impl Drop for RawMode {
    fn drop(&mut self) {
        unsafe {
            libc::tcsetattr(self.fd, libc::TCSANOW, &self.original);
        }
    }
}

fn spawn_reader(mut tty: File, keys: Arc<Mutex<Keys>>) {
    std::thread::spawn(move || {
        let mut byte = [0u8; 1];
        loop {
            match tty.read(&mut byte) {
                Ok(1) => {
                    let mut keys = keys.lock().expect("keyboard state poisoned");
                    keys.apply(byte[0]);
                    if keys.quit {
                        break;
                    }
                }
                Ok(_) => {}
                Err(err) if err.kind() == io::ErrorKind::Interrupted => {}
                Err(_) => break,
            }
        }
    });
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let keys = Arc::new(Mutex::new(Keys::default()));

    let raw_mode = match OpenOptions::new().read(true).write(true).open("/dev/tty") {
        Ok(tty) => match RawMode::enable(tty.as_raw_fd()) {
            Ok(raw) => {
                spawn_reader(tty.try_clone()?, Arc::clone(&keys));
                eprintln!(
                    "dora-keyboard: W/S vx={VX}, A/D vy={VY}, arrows vyaw={VYAW}, space stop, q quit"
                );
                Some(raw)
            }
            Err(err) => {
                eprintln!("dora-keyboard: /dev/tty is not a terminal ({err}); keyboard disabled");
                None
            }
        },
        Err(err) => {
            eprintln!("dora-keyboard: cannot open /dev/tty ({err}); keyboard disabled");
            None
        }
    };

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                let (command, quit) = {
                    let mut keys = keys.lock().expect("keyboard state poisoned");
                    (keys.command(), keys.quit)
                };
                node.send_output(
                    DataId::from(CMD_VEL.to_owned()),
                    metadata.parameters,
                    command.to_vec().into_arrow(),
                )?;
                if quit {
                    break;
                }
            }
            Event::Input { id, .. } => eprintln!("dora-keyboard: ignoring input `{id}`"),
            _ => {}
        }
    }

    // A final zero does not wait for robotd's deadman on a graceful stop.
    let _ = node.send_output(
        DataId::from(CMD_VEL.to_owned()),
        Default::default(),
        vec![0.0_f32; 3].into_arrow(),
    );

    drop(raw_mode);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn keys(bytes: &[u8]) -> Keys {
        let mut keys = Keys::default();
        for byte in bytes {
            keys.apply(*byte);
        }
        keys
    }

    #[test]
    fn wasd_sets_each_axis() {
        assert_eq!(keys(b"w").command(), [VX, 0.0, 0.0]);
        assert_eq!(keys(b"s").command(), [-VX, 0.0, 0.0]);
        assert_eq!(keys(b"a").command()[1], VY);
        assert_eq!(keys(b"d").command()[1], -VY);
    }

    #[test]
    fn arrows_steer_and_drive() {
        assert_eq!(keys(b"\x1b[D").command()[2], VYAW);
        assert_eq!(keys(b"\x1b[C").command()[2], -VYAW);
        assert_eq!(keys(b"\x1b[A").command()[0], VX);
        assert_eq!(keys(b"\x1b[B").command()[0], -VX);
    }

    #[test]
    fn space_stops() {
        assert_eq!(keys(b"wa ").command(), [0.0; 3]);
    }

    #[test]
    fn stale_key_zeroes() {
        let mut keys = keys(b"w");
        keys.last = Some(Instant::now() - HOLD - Duration::from_millis(1));
        assert_eq!(keys.command(), [0.0; 3]);
    }

    #[test]
    fn q_quits_with_a_zero_command() {
        let mut keys = keys(b"wq");
        assert!(keys.quit);
        assert_eq!(keys.command(), [0.0; 3]);
    }
}
