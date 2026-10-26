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
            // Swallow the final byte of an escape sequence. Arrow keys and
            // function keys end in letters that mean something here otherwise —
            // `ESC [ D` (left arrow) would read as a strafe.
            self.escape = 0;
            return;
        }

        match byte {
            0x1b => self.escape = 1,
            b'w' | b'W' => self.set_vx(VX),
            b's' | b'S' => self.set_vx(-VX),
            b'a' | b'A' => self.set_vy(VY),
            b'd' | b'D' => self.set_vy(-VY),
            // Turning on Q/E rather than the arrows, which MuJoCo's viewer uses
            // to step the simulation.
            b'q' | b'Q' => self.set_vyaw(VYAW),
            b'e' | b'E' => self.set_vyaw(-VYAW),
            b'x' | b'X' => self.stop(),
            0x03 => self.quit = true,
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
    /// Foreground group before this node claimed it, when it had to.
    original_foreground: Option<libc::pid_t>,
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
        Ok(Self {
            fd,
            original,
            original_foreground: None,
        })
    }

    /// dora spawns nodes from its daemon, and the node can land in a background
    /// process group: the tty opens and goes raw, but the first read fails with
    /// EIO. Take the foreground here, and give it back on drop so the second
    /// Ctrl-C still reaches dora.
    fn claim_terminal(&mut self) {
        let pgrp = unsafe { libc::getpgrp() };
        let foreground = unsafe { libc::tcgetpgrp(self.fd) };
        eprintln!("dora-keyboard: /dev/tty ready (pgrp {pgrp}, foreground {foreground})");
        if foreground < 0 || foreground == pgrp {
            return;
        }
        if unsafe { libc::tcsetpgrp(self.fd, pgrp) } == 0 {
            self.original_foreground = Some(foreground);
            eprintln!(
                "dora-keyboard: took the terminal's foreground group ({foreground} -> {pgrp})"
            );
        } else {
            eprintln!(
                "dora-keyboard: cannot take the terminal's foreground: {}",
                io::Error::last_os_error()
            );
        }
    }
}

impl Drop for RawMode {
    fn drop(&mut self) {
        if let Some(foreground) = self.original_foreground.take() {
            unsafe {
                libc::tcsetpgrp(self.fd, foreground);
            }
        }
        unsafe {
            libc::tcsetattr(self.fd, libc::TCSANOW, &self.original);
        }
    }
}

fn spawn_reader(mut tty: File, keys: Arc<Mutex<Keys>>) {
    std::thread::spawn(move || {
        let mut byte = [0u8; 1];
        let mut first = true;
        loop {
            match tty.read(&mut byte) {
                Ok(1) => {
                    if first {
                        first = false;
                        eprintln!("dora-keyboard: first key received");
                    }
                    let mut keys = keys.lock().expect("keyboard state poisoned");
                    keys.apply(byte[0]);
                    if keys.quit {
                        break;
                    }
                }
                Ok(_) => {}
                Err(err) if err.kind() == io::ErrorKind::Interrupted => {}
                Err(err) => {
                    eprintln!("dora-keyboard: tty read stopped: {err}");
                    break;
                }
            }
        }
    });
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let keys = Arc::new(Mutex::new(Keys::default()));

    let raw_mode = match OpenOptions::new().read(true).write(true).open("/dev/tty") {
        Ok(tty) => match RawMode::enable(tty.as_raw_fd()) {
            Ok(mut raw) => {
                raw.claim_terminal();
                spawn_reader(tty.try_clone()?, Arc::clone(&keys));
                eprintln!(
                    "dora-keyboard: W/S vx={VX}, A/D vy={VY}, Q/E turn={VYAW}, X stop, Ctrl-C quit"
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
    fn qe_steers() {
        assert_eq!(keys(b"q").command()[2], VYAW);
        assert_eq!(keys(b"e").command()[2], -VYAW);
    }

    #[test]
    fn x_stops() {
        assert_eq!(keys(b"waqx").command(), [0.0; 3]);
    }

    #[test]
    fn arrow_sequences_are_swallowed() {
        // `ESC [ D` ends in `D`, which would strafe right if the sequence were
        // not consumed first.
        let command = keys(b"\x1b[A\x1b[B\x1b[C\x1b[D").command();
        assert_eq!(command, [0.0; 3]);
    }

    #[test]
    fn stale_key_zeroes() {
        let mut keys = keys(b"w");
        keys.last = Some(Instant::now() - HOLD - Duration::from_millis(1));
        assert_eq!(keys.command(), [0.0; 3]);
    }

    #[test]
    fn ctrl_c_quits_with_a_zero_command() {
        let mut keys = keys(b"w\x03");
        assert!(keys.quit);
        assert_eq!(keys.command(), [0.0; 3]);
    }
}
