//! Terminal teleoperation as a dora node.
//!
//! dora gives nodes a null stdin, so keys are read from `/dev/tty`. Every timer
//! tick publishes `cmd_vel` as `[vx, vy, vyaw]`. Hold to move, release to stop:
//! terminals report no key-up, so a key counts as held while the terminal keeps
//! repeating it. The OS delays the first repeat, so the first window is longer;
//! once repeats are flowing the timeout is short and release stops promptly.
//! X stops immediately, and Ctrl-C stops the whole dataflow.
//!
//! Each axis keeps its own hold timer. A shared one let a key on one axis keep
//! another axis's value alive: releasing W and holding A left `vx` latched, and
//! the duck walked forward while strafing.
//!
//! Every escape sequence the terminal can send for an arrow or function key is
//! consumed, so its final byte (`A` in `ESC [ A`) never reads as a command.
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
/// Covers the OS's initial key-repeat delay, before the first repeat arrives.
const HOLD_START: Duration = Duration::from_millis(800);
/// Once repeats are flowing, this is the release timeout.
const HOLD_REPEAT: Duration = Duration::from_millis(250);
/// Above the ~0.25 m/s deadband the walking policy shows going forward.
const VX_FORWARD: f32 = 0.30;
/// The backward deadband is higher: measured on the twin, 0.30 moved nothing at
/// all while 0.35 walked off every time, so this keeps a margin above the edge.
const VX_BACKWARD: f32 = 0.40;
/// Strafe is ignored below about 0.4 m/s by the walking policy; 0.15 did nothing.
const VY: f32 = 0.50;
/// The policy barely turns below about 1.2 rad/s; 0.5 only wobbled.
const VYAW: f32 = 1.50;

/// A partial escape sequence is abandoned after this long. Terminals send one
/// sequence in a single burst, so only a stray Esc key ever hits it.
const ESCAPE_TIMEOUT: Duration = Duration::from_millis(100);

/// Which part of an escape sequence is being consumed.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Escape {
    /// Saw ESC; the next byte decides what follows.
    Start,
    /// `ESC [` — parameters run to a final byte in `0x40..=0x7e`.
    Csi,
    /// `ESC O` — one final byte follows.
    Ss3,
}

/// One axis of the command and the key currently holding it.
///
/// State per axis, not per command: a key pressed on one axis must not keep another
/// axis's value alive. A single shared timer did exactly that — the repeats of A
/// refreshed the timer that was holding W's `vx`, so a released W never expired.
#[derive(Default)]
struct Hold {
    /// Last recognised key that set this axis, lower-cased; a repeat is the same
    /// byte again.
    key: Option<u8>,
    last_at: Option<Instant>,
    /// Whether a repeat has arrived for the current press.
    repeating: bool,
}

impl Hold {
    /// Records a press of `key`. A different key starts a new press and gets the
    /// long initial window until its own repeats begin.
    fn press(&mut self, key: u8) {
        self.repeating = self.key == Some(key);
        self.key = Some(key);
        self.last_at = Some(Instant::now());
    }

    /// Whether the axis is still held, ending it once its key stops repeating.
    ///
    /// Clearing on expiry matters: pressing the same key again later must read as a
    /// fresh press with the long window, not as a repeat of a press long released.
    fn held(&mut self) -> bool {
        let Some(at) = self.last_at else {
            return false;
        };
        let timeout = if self.repeating {
            HOLD_REPEAT
        } else {
            HOLD_START
        };
        if at.elapsed() > timeout {
            *self = Self::default();
            return false;
        }
        true
    }
}

#[derive(Default)]
struct Keys {
    vx: f32,
    vy: f32,
    vyaw: f32,
    vx_hold: Hold,
    vy_hold: Hold,
    vyaw_hold: Hold,
    quit: bool,
    escape: Option<Escape>,
    escape_at: Option<Instant>,
}

impl Keys {
    fn apply(&mut self, byte: u8) {
        if let Some(state) = self.escape {
            self.escape = match state {
                // `ESC [ 1 ; 5 D`, `ESC O D`, and `ESC w` (Option+key) all end
                // elsewhere; consuming to the end keeps `D` from strafing.
                Escape::Start => match byte {
                    b'[' => Some(Escape::Csi),
                    b'O' => Some(Escape::Ss3),
                    _ => None,
                },
                Escape::Csi => {
                    if (0x40..=0x7e).contains(&byte) {
                        None
                    } else {
                        Some(Escape::Csi)
                    }
                }
                Escape::Ss3 => None,
            };
            self.escape_at = Some(Instant::now());
            return;
        }

        if let Some(key) = recognised(byte) {
            match key {
                b'w' | b's' => self.vx_hold.press(key),
                b'a' | b'd' => self.vy_hold.press(key),
                b'q' | b'e' => self.vyaw_hold.press(key),
                // X stops outright; there is no hold to keep alive.
                _ => {}
            }
        }

        match byte {
            0x1b => {
                self.escape = Some(Escape::Start);
                self.escape_at = Some(Instant::now());
            }
            b'w' | b'W' => self.vx = VX_FORWARD,
            b's' | b'S' => self.vx = -VX_BACKWARD,
            b'a' | b'A' => self.vy = VY,
            b'd' | b'D' => self.vy = -VY,
            // Turning on Q/E rather than the arrows, which MuJoCo's viewer uses
            // to step the simulation.
            b'q' | b'Q' => self.vyaw = VYAW,
            b'e' | b'E' => self.vyaw = -VYAW,
            b'x' | b'X' => self.stop(),
            0x03 => self.quit = true,
            _ => {}
        }
    }

    fn stop(&mut self) {
        self.vx = 0.0;
        self.vy = 0.0;
        self.vyaw = 0.0;
        self.vx_hold = Hold::default();
        self.vy_hold = Hold::default();
        self.vyaw_hold = Hold::default();
    }

    /// The command for this tick, each axis zeroed once its own key stops repeating.
    fn command(&mut self) -> [f32; 3] {
        if self.quit {
            return [0.0; 3];
        }
        // A lone Esc press leaves the parser waiting for a sequence that never
        // comes; let the next key through instead of swallowing it forever.
        if let Some(at) = self.escape_at
            && at.elapsed() > ESCAPE_TIMEOUT
        {
            self.escape = None;
            self.escape_at = None;
        }
        if !self.vx_hold.held() {
            self.vx = 0.0;
        }
        if !self.vy_hold.held() {
            self.vy = 0.0;
        }
        if !self.vyaw_hold.held() {
            self.vyaw = 0.0;
        }
        [self.vx, self.vy, self.vyaw]
    }
}

/// The key a byte drives, lower-cased, for repeat detection. Unrecognised bytes
/// (including escape-sequence finals) return `None`, so they cannot keep a
/// command alive.
fn recognised(byte: u8) -> Option<u8> {
    match byte {
        b'w' | b'W' => Some(b'w'),
        b's' | b'S' => Some(b's'),
        b'a' | b'A' => Some(b'a'),
        b'd' | b'D' => Some(b'd'),
        b'q' | b'Q' => Some(b'q'),
        b'e' | b'E' => Some(b'e'),
        b'x' | b'X' => Some(b'x'),
        _ => None,
    }
}

/// Raw input, normal output: dora's own log lines share this terminal and still
/// need `ONLCR`, or every newline becomes a staircase.
struct RawMode {
    fd: RawFd,
    original: libc::termios,
    /// The terminal's foreground group before this node ran.
    foreground: Option<libc::pid_t>,
    /// Whether this node had to take the foreground itself.
    claimed: bool,
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
        // Keys pressed before raw mode existed — during the simulator's startup
        // the terminal echoes them as `^[[A` — are still queued. Drop them so
        // the node starts from nothing rather than replaying them.
        unsafe {
            libc::tcflush(fd, libc::TCIFLUSH);
        }
        Ok(Self {
            fd,
            original,
            foreground: None,
            claimed: false,
        })
    }

    /// dora gives each node its own process group, so the node lands in the
    /// background and the first tty read would fail with EIO. Take the
    /// foreground here, and give it back on drop so a later Ctrl-C reaches dora.
    fn claim_terminal(&mut self) {
        let pgrp = unsafe { libc::getpgrp() };
        let foreground = unsafe { libc::tcgetpgrp(self.fd) };
        eprintln!("dora-keyboard: /dev/tty ready (pgrp {pgrp}, foreground {foreground})");
        if foreground > 0 {
            self.foreground = Some(foreground);
        }
        if foreground < 0 || foreground == pgrp {
            return;
        }
        if unsafe { libc::tcsetpgrp(self.fd, pgrp) } == 0 {
            self.claimed = true;
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
        if self.claimed
            && let Some(foreground) = self.foreground
        {
            unsafe {
                libc::tcsetpgrp(self.fd, foreground);
            }
        }
        // A key still held as the dataflow stops would otherwise be typed into
        // the shell once echo comes back.
        unsafe {
            libc::tcflush(self.fd, libc::TCIFLUSH);
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
                    "dora-keyboard: hold W vx={VX_FORWARD} / S vx=-{VX_BACKWARD}, A/D vy={VY}, Q/E turn={VYAW}; release stops; X stop; Ctrl-C ends"
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

    let interrupted = keys.lock().expect("keyboard state poisoned").quit;
    let foreground = raw_mode.as_ref().and_then(|raw| raw.foreground);

    // A final zero does not wait for robotd's deadman on a graceful stop.
    let _ = node.send_output(
        DataId::from(CMD_VEL.to_owned()),
        Default::default(),
        vec![0.0_f32; 3].into_arrow(),
    );

    drop(raw_mode);

    // dora gives each node its own process group, so the Ctrl-C read in raw mode
    // only reached this node. Restore the terminal first, then send the SIGINT
    // the terminal would have sent: dora stops and the launcher takes the twin
    // down with one keypress.
    if interrupted && let Some(pgrp) = foreground {
        eprintln!("dora-keyboard: Ctrl-C — stopping the dataflow");
        unsafe {
            libc::kill(-pgrp, libc::SIGINT);
        }
    }

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
        assert_eq!(keys(b"w").command(), [VX_FORWARD, 0.0, 0.0]);
        assert_eq!(keys(b"s").command(), [-VX_BACKWARD, 0.0, 0.0]);
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
    fn held_arrow_sequences_never_leak_a_command() {
        // A held arrow key repeats the same sequence; not one of them may reach
        // the command map.
        let mut keys = Keys::default();
        for _ in 0..40 {
            for byte in b"\x1b[D\x1b[A\x1b[C\x1b[B" {
                keys.apply(*byte);
            }
        }
        assert_eq!(keys.command(), [0.0; 3]);
        assert!(keys.vx_hold.key.is_none());
        assert!(keys.vy_hold.key.is_none());
        assert!(keys.vyaw_hold.key.is_none());
    }

    #[test]
    fn parametrised_and_ss3_sequences_are_swallowed() {
        // iTerm2, Terminal.app's modified arrows, and application cursor mode.
        assert_eq!(keys(b"\x1b[1;5D").command(), [0.0; 3]);
        assert_eq!(keys(b"\x1b[1;2A").command(), [0.0; 3]);
        assert_eq!(keys(b"\x1bOD").command(), [0.0; 3]);
        assert_eq!(keys(b"\x1b[200~").command(), [0.0; 3]);
    }

    #[test]
    fn option_key_is_swallowed() {
        // Option+W arrives as `ESC w`; it must not walk the duck.
        assert_eq!(keys(b"\x1bw").command(), [0.0; 3]);
    }

    #[test]
    fn a_lone_escape_does_not_eat_the_next_key() {
        let mut keys = Keys::default();
        keys.apply(0x1b);
        keys.escape_at = Some(Instant::now() - ESCAPE_TIMEOUT - Duration::from_millis(1));
        let _ = keys.command();
        keys.apply(b'w');
        assert_eq!(keys.command(), [VX_FORWARD, 0.0, 0.0]);
    }

    #[test]
    fn first_press_survives_the_initial_repeat_delay() {
        let mut keys = keys(b"w");
        keys.vx_hold.last_at = Some(Instant::now() - Duration::from_millis(700));
        assert_eq!(keys.command(), [VX_FORWARD, 0.0, 0.0]);
    }

    #[test]
    fn release_stops_after_repeats() {
        let mut keys = keys(b"ww");
        keys.vx_hold.last_at = Some(Instant::now() - HOLD_REPEAT - Duration::from_millis(1));
        assert_eq!(keys.command(), [0.0; 3]);
    }

    #[test]
    fn tap_stops_after_the_initial_window() {
        let mut keys = keys(b"w");
        keys.vx_hold.last_at = Some(Instant::now() - HOLD_START - Duration::from_millis(1));
        assert_eq!(keys.command(), [0.0; 3]);
    }

    /// Releasing W and holding A must stop the forward axis once W's own repeats
    /// stop, even though A keeps repeating. A shared timer used to refresh on A's
    /// repeats and leave `vx` latched for as long as A was held.
    #[test]
    fn a_held_axis_does_not_keep_the_previous_axis_alive() {
        let mut keys = keys(b"wwa");
        assert_eq!(keys.command(), [VX_FORWARD, VY, 0.0]);

        // W's last repeat is now older than its release window; A's is current.
        keys.vx_hold.last_at = Some(Instant::now() - HOLD_REPEAT - Duration::from_millis(1));
        assert_eq!(keys.command(), [0.0, VY, 0.0]);
    }

    /// A second press of the same key after it has stopped repeating is a new
    /// press and gets the long initial window again.
    #[test]
    fn pressing_a_key_again_after_release_starts_a_new_press() {
        let mut keys = keys(b"ww");
        keys.vx_hold.last_at = Some(Instant::now() - HOLD_REPEAT - Duration::from_millis(1));
        assert_eq!(keys.command(), [0.0; 3]);

        keys.apply(b'w');
        assert!(!keys.vx_hold.repeating, "a new press is not a repeat");
        keys.vx_hold.last_at = Some(Instant::now() - Duration::from_millis(700));
        assert_eq!(keys.command(), [VX_FORWARD, 0.0, 0.0]);
    }

    /// Two keys on different axes held together keep both axes alive, each on its
    /// own repeats — the case the per-axis timers must not break.
    #[test]
    fn a_chord_holds_both_axes() {
        let mut keys = keys(b"wqwq");
        keys.vx_hold.last_at = Some(Instant::now() - Duration::from_millis(10));
        keys.vyaw_hold.last_at = Some(Instant::now() - Duration::from_millis(10));
        assert_eq!(keys.command(), [VX_FORWARD, 0.0, VYAW]);
    }

    #[test]
    fn ctrl_c_quits_with_a_zero_command() {
        let mut keys = keys(b"w\x03");
        assert!(keys.quit);
        assert_eq!(keys.command(), [0.0; 3]);
    }
}
