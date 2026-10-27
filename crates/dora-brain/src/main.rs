//! Reactive behaviour for the duck: turn towards a target, walk, stop close.
//!
//! This is the first behaviour layer, not the upstream 16-state brain. It
//! consumes `perception`'s `target = [visible, bearing, area]` and
//! `robotd-state`'s `imu = [qw, qx, qy, qz, gx, gy, gz]`, and publishes
//! `cmd_vel = [vx, vy, vyaw]` at 20 Hz. `robotd-wrapper` remains the only
//! intent sink; `robotd` still decides whether the command is safe.
//!
//! The control is deliberately bang-bang rather than proportional. The
//! walking policy has a yaw deadband: commands below about 1.2 rad/s barely
//! turn the duck, and forward commands below about 0.25 m/s barely move it.
//! Asking for a gentle correction therefore produces a duck that wobbles and
//! goes nowhere. The node emits the values the policy actually executes and
//! uses hysteresis on the bearing to avoid chattering between turn and walk:
//!
//! - `|bearing| > 0.45`: turn towards the target at `BRAIN_TURN` (1.5 rad/s)
//! - `|bearing| < 0.30`: drive forward at `BRAIN_FORWARD` (0.30 m/s)
//! - between the two: keep the previous decision
//!
//! The band is wide on purpose. A 0.10–0.20 rad window sounds precise, but the
//! policy's turn has enough latency that the duck overshoots, reverses, and
//! oscillates in place instead of walking. Five degrees off is still in front
//! of a target that is a few tens of centimetres away.
//!
//! `area` is the fraction of the image covered by the target. Above
//! `BRAIN_CLOSE_AREA` (0.06) the duck stops; it only resumes below
//! `BRAIN_CLOSE_RELEASE` (0.02), so a target hovering at the threshold does
//! not make it creep. If the target disappears, the duck turns right for up to
//! `BRAIN_SEARCH_MS` (8 s, about one full circle at the policy's real yaw
//! rate), then stops. A fall or a lost/stale IMU stops the command
//! immediately; the tilt limit is `BRAIN_TILT_LIMIT_DEG` (45°).
//!
//! All the values are tunable through `BRAIN_*` environment variables; see
//! `Config::from_env`. `BRAIN_REQUIRE_IMU=0` lets the node run without
//! `robotd-state`, for graph-only checks, but the default refuses to drive
//! blind.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use eyre::Result;
use std::time::{Duration, Instant};

const TARGET: &str = "target";
const IMU: &str = "imu";
const TICK: &str = "tick";
const CMD_VEL: &str = "cmd_vel";

const ZERO: [f32; 3] = [0.0, 0.0, 0.0];

#[derive(Clone, Copy, Debug)]
struct Config {
    forward: f32,
    turn: f32,
    align_start: f32,
    align_stop: f32,
    close_area: f32,
    close_release: f32,
    target_timeout: Duration,
    search_for: Duration,
    tilt_limit: f32,
    imu_timeout: Duration,
    require_imu: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            forward: 0.30,
            turn: 1.50,
            align_start: 0.45,
            align_stop: 0.30,
            close_area: 0.06,
            close_release: 0.02,
            target_timeout: Duration::from_millis(300),
            search_for: Duration::from_millis(8000),
            tilt_limit: 45_f32.to_radians(),
            imu_timeout: Duration::from_millis(500),
            require_imu: true,
        }
    }
}

impl Config {
    fn from_env() -> Self {
        let defaults = Self::default();
        Self {
            forward: env_f32("BRAIN_FORWARD", defaults.forward, 0.0, 1.0),
            turn: env_f32("BRAIN_TURN", defaults.turn, 0.0, 3.0),
            align_start: env_f32("BRAIN_ALIGN_START", defaults.align_start, 0.0, 1.0),
            align_stop: env_f32("BRAIN_ALIGN_STOP", defaults.align_stop, 0.0, 1.0),
            close_area: env_f32("BRAIN_CLOSE_AREA", defaults.close_area, 0.0, 1.0),
            close_release: env_f32("BRAIN_CLOSE_RELEASE", defaults.close_release, 0.0, 1.0),
            target_timeout: Duration::from_millis(env_u64(
                "BRAIN_TARGET_TIMEOUT_MS",
                defaults.target_timeout.as_millis() as u64,
            )),
            search_for: Duration::from_millis(env_u64(
                "BRAIN_SEARCH_MS",
                defaults.search_for.as_millis() as u64,
            )),
            tilt_limit: env_f32("BRAIN_TILT_LIMIT_DEG", 45.0, 0.0, 90.0).to_radians(),
            imu_timeout: Duration::from_millis(env_u64(
                "BRAIN_IMU_TIMEOUT_MS",
                defaults.imu_timeout.as_millis() as u64,
            )),
            require_imu: env_bool("BRAIN_REQUIRE_IMU", defaults.require_imu),
        }
    }
}

fn env_f32(name: &str, default: f32, min: f32, max: f32) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse::<f32>().ok())
        .filter(|value| value.is_finite())
        .unwrap_or(default)
        .clamp(min, max)
}

fn env_u64(name: &str, default: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .unwrap_or(default)
}

fn env_bool(name: &str, default: bool) -> bool {
    let Ok(value) = std::env::var(name) else {
        return default;
    };
    match value.trim().to_ascii_lowercase().as_str() {
        "1" | "true" | "yes" | "on" => true,
        "0" | "false" | "no" | "off" => false,
        _ => default,
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Detection {
    visible: bool,
    bearing: f32,
    area: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    Stopped,
    NoImu,
    Tilt,
    Search,
    Turn,
    Forward,
    Arrived,
}

impl Mode {
    fn as_str(self) -> &'static str {
        match self {
            Self::Stopped => "stopped",
            Self::NoImu => "no-imu",
            Self::Tilt => "tilt",
            Self::Search => "search",
            Self::Turn => "turn",
            Self::Forward => "forward",
            Self::Arrived => "arrived",
        }
    }
}

struct Brain {
    cfg: Config,
    target: Option<Detection>,
    target_at: Option<Instant>,
    imu: Option<[f32; 7]>,
    imu_at: Option<Instant>,
    aligned: bool,
    arrived: bool,
    search_started: Option<Instant>,
    search_dir: f32,
    mode: Mode,
    ticks: u64,
}

impl Brain {
    fn new(cfg: Config) -> Self {
        Self {
            cfg,
            target: None,
            target_at: None,
            imu: None,
            imu_at: None,
            aligned: false,
            arrived: false,
            search_started: None,
            // The standing policy tends to drift left, so looking right first
            // usually finds a target in front of the duck instead of turning
            // away from it. One direction for a full circle is simpler than a
            // sweep and covers every heading.
            search_dir: -1.0,
            mode: Mode::Stopped,
            ticks: 0,
        }
    }

    /// `[visible, bearing, area]` from `perception`. Returns false when the
    /// message does not fit the contract; the node logs and drops it.
    fn observe_target(&mut self, values: &[f32], now: Instant) -> bool {
        if values.len() != 3 || !values.iter().all(|value| value.is_finite()) {
            return false;
        }
        let detection = Detection {
            visible: values[0] >= 0.5,
            bearing: values[1].clamp(-1.0, 1.0),
            area: values[2].clamp(0.0, 1.0),
        };
        self.target = Some(detection);
        self.target_at = Some(now);
        if detection.visible {
            self.search_started = None;
        }
        true
    }

    /// `[qw, qx, qy, qz, gx, gy, gz]` from `robotd-state`.
    fn observe_imu(&mut self, values: &[f32], now: Instant) -> bool {
        if values.len() != 7 || !values.iter().all(|value| value.is_finite()) {
            return false;
        }
        let mut imu = [0.0; 7];
        imu.copy_from_slice(values);
        self.imu = Some(imu);
        self.imu_at = Some(now);
        true
    }

    fn step(&mut self, now: Instant) -> [f32; 3] {
        self.ticks += 1;
        let (mode, command) = self.compute(now);
        self.report(now, mode, command);
        command
    }

    fn compute(&mut self, now: Instant) -> (Mode, [f32; 3]) {
        let imu_fresh = self
            .imu_at
            .is_some_and(|at| now.duration_since(at) <= self.cfg.imu_timeout);

        if self.cfg.require_imu {
            let Some(imu) = self.imu.filter(|_| imu_fresh) else {
                return (Mode::NoImu, ZERO);
            };
            let Some(tilt) = tilt_rad(&imu) else {
                return (Mode::NoImu, ZERO);
            };
            if tilt > self.cfg.tilt_limit {
                return (Mode::Tilt, ZERO);
            }
        } else if imu_fresh
            && let Some(imu) = self.imu
            && let Some(tilt) = tilt_rad(&imu)
            && tilt > self.cfg.tilt_limit
        {
            return (Mode::Tilt, ZERO);
        }

        self.drive(now)
    }

    fn drive(&mut self, now: Instant) -> (Mode, [f32; 3]) {
        let fresh = self
            .target
            .zip(self.target_at)
            .filter(|(detection, at)| {
                detection.visible && now.duration_since(*at) <= self.cfg.target_timeout
            })
            .map(|(detection, _)| detection);

        let Some(target) = fresh else {
            return self.search(now);
        };

        self.search_started = None;

        if self.arrived {
            if target.area >= self.cfg.close_release {
                return (Mode::Arrived, ZERO);
            }
            self.arrived = false;
        }
        if target.area >= self.cfg.close_area {
            self.arrived = true;
            return (Mode::Arrived, ZERO);
        }

        // Hysteresis: start turning only past `align_start`, stop turning once
        // inside `align_stop`. Between them, keep the previous decision.
        if self.aligned {
            if target.bearing.abs() >= self.cfg.align_start {
                self.aligned = false;
            }
        } else if target.bearing.abs() <= self.cfg.align_stop {
            self.aligned = true;
        }

        if self.aligned {
            (Mode::Forward, [self.cfg.forward, 0.0, 0.0])
        } else {
            let yaw = if target.bearing > 0.0 {
                -self.cfg.turn
            } else {
                self.cfg.turn
            };
            (Mode::Turn, [0.0, 0.0, yaw])
        }
    }

    fn search(&mut self, now: Instant) -> (Mode, [f32; 3]) {
        let started = *self.search_started.get_or_insert(now);
        if self.cfg.search_for.is_zero() || now.duration_since(started) >= self.cfg.search_for {
            return (Mode::Stopped, ZERO);
        }
        (Mode::Search, [0.0, 0.0, self.search_dir * self.cfg.turn])
    }

    fn report(&mut self, now: Instant, mode: Mode, command: [f32; 3]) {
        let changed = mode != self.mode;
        self.mode = mode;
        if !changed && !self.ticks.is_multiple_of(50) {
            return;
        }

        let (visible, bearing, area, age) = match (self.target, self.target_at) {
            (Some(target), Some(at)) => (
                target.visible,
                target.bearing,
                target.area,
                now.duration_since(at).as_millis(),
            ),
            _ => (false, 0.0, 0.0, 0),
        };
        eprintln!(
            "dora-brain: mode={} cmd=[{:.2}, {:.2}, {:.2}] visible={} bearing={:+.2} area={:.3} age={}ms",
            mode.as_str(),
            command[0],
            command[1],
            command[2],
            visible,
            bearing,
            area,
            age
        );
    }
}

/// Roll/pitch angle magnitude from the trunk quaternion, radians.
fn tilt_rad(imu: &[f32; 7]) -> Option<f32> {
    let (w, x, y, z) = (imu[0], imu[1], imu[2], imu[3]);
    let norm = (w * w + x * x + y * y + z * z).sqrt();
    if !norm.is_finite() || norm < 1e-6 {
        return None;
    }
    let (w, x, y, z) = (w / norm, x / norm, y / norm, z / norm);
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    Some(roll.abs().max(pitch.abs()))
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let config = Config::from_env();
    eprintln!(
        "dora-brain: forward={:.2} m/s turn={:.2} rad/s align={:.2}/{:.2} close={:.2}/{:.2} \
         search={} ms require_imu={}",
        config.forward,
        config.turn,
        config.align_start,
        config.align_stop,
        config.close_area,
        config.close_release,
        config.search_for.as_millis(),
        config.require_imu
    );
    let mut brain = Brain::new(config);
    let mut bad_targets = 0_u64;
    let mut bad_imus = 0_u64;

    while let Some(event) = events.recv() {
        let Event::Input { id, metadata, data } = event else {
            continue;
        };

        match id.as_str() {
            TARGET => {
                let Ok(values) = Vec::<f32>::try_from(&data) else {
                    bad_targets += 1;
                    if bad_targets == 1 || bad_targets.is_multiple_of(100) {
                        eprintln!("dora-brain: target is not an f32 array (attempt {bad_targets})");
                    }
                    continue;
                };
                if !brain.observe_target(&values, Instant::now()) {
                    bad_targets += 1;
                    if bad_targets == 1 || bad_targets.is_multiple_of(100) {
                        eprintln!(
                            "dora-brain: target must be [visible, bearing, area] (attempt {bad_targets})"
                        );
                    }
                }
            }
            IMU => {
                let Ok(values) = Vec::<f32>::try_from(&data) else {
                    bad_imus += 1;
                    if bad_imus == 1 || bad_imus.is_multiple_of(100) {
                        eprintln!("dora-brain: imu is not an f32 array (attempt {bad_imus})");
                    }
                    continue;
                };
                if !brain.observe_imu(&values, Instant::now()) {
                    bad_imus += 1;
                    if bad_imus == 1 || bad_imus.is_multiple_of(100) {
                        eprintln!(
                            "dora-brain: imu must be [qw, qx, qy, qz, gx, gy, gz] (attempt {bad_imus})"
                        );
                    }
                }
            }
            TICK => {
                let command = brain.step(Instant::now());
                node.send_output(
                    DataId::from(CMD_VEL.to_owned()),
                    metadata.parameters,
                    command.to_vec().into_arrow(),
                )?;
            }
            other => eprintln!("dora-brain: ignoring input `{other}`"),
        }
    }

    // A graceful stop does not wait for robotd's deadman.
    let _ = node.send_output(
        DataId::from(CMD_VEL.to_owned()),
        Default::default(),
        vec![0.0_f32; 3].into_arrow(),
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        Config::default()
    }

    fn identity() -> [f32; 7] {
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }

    fn brain_with_imu(config: Config) -> (Brain, Instant) {
        let now = Instant::now();
        let mut brain = Brain::new(config);
        assert!(brain.observe_imu(&identity(), now));
        (brain, now)
    }

    fn target(visible: bool, bearing: f32, area: f32) -> [f32; 3] {
        [if visible { 1.0 } else { 0.0 }, bearing, area]
    }

    #[test]
    fn turns_towards_a_target_on_the_right() {
        let (mut brain, now) = brain_with_imu(config());
        assert!(brain.observe_target(&target(true, 0.5, 0.01), now));
        let command = brain.step(now);
        assert_eq!(brain.mode, Mode::Turn);
        assert_eq!(command[0], 0.0);
        assert!(
            command[2] < 0.0,
            "positive bearing is right, yaw is left-positive"
        );
    }

    #[test]
    fn turns_towards_a_target_on_the_left() {
        let (mut brain, now) = brain_with_imu(config());
        assert!(brain.observe_target(&target(true, -0.5, 0.01), now));
        let command = brain.step(now);
        assert_eq!(brain.mode, Mode::Turn);
        assert!(command[2] > 0.0);
    }

    #[test]
    fn walks_when_aligned_and_far() {
        let (mut brain, now) = brain_with_imu(config());
        assert!(brain.observe_target(&target(true, 0.0, 0.01), now));
        let command = brain.step(now);
        assert_eq!(brain.mode, Mode::Forward);
        assert_eq!(command, [0.30, 0.0, 0.0]);
    }

    #[test]
    fn stops_when_close_and_resumes_after_release() {
        let (mut brain, now) = brain_with_imu(config());
        assert!(brain.observe_target(&target(true, 0.0, 0.12), now));
        assert_eq!(brain.step(now), ZERO);
        assert_eq!(brain.mode, Mode::Arrived);

        // Between release (0.02) and close (0.06) the stop stays latched.
        assert!(brain.observe_target(&target(true, 0.0, 0.05), now));
        assert_eq!(brain.step(now), ZERO);
        assert_eq!(brain.mode, Mode::Arrived);

        assert!(brain.observe_target(&target(true, 0.0, 0.01), now));
        let command = brain.step(now);
        assert_eq!(brain.mode, Mode::Forward);
        assert_eq!(command, [0.30, 0.0, 0.0]);
    }

    #[test]
    fn bearing_hysteresis_avoids_chatter() {
        let (mut brain, now) = brain_with_imu(config());
        assert!(brain.observe_target(&target(true, 0.05, 0.01), now));
        assert_eq!(brain.step(now), [0.30, 0.0, 0.0]);
        assert_eq!(brain.mode, Mode::Forward);

        // Above align_stop but below align_start: stay in the previous mode.
        assert!(brain.observe_target(&target(true, 0.35, 0.01), now));
        assert_eq!(brain.step(now), [0.30, 0.0, 0.0]);

        // Past align_start: turn.
        assert!(brain.observe_target(&target(true, 0.50, 0.01), now));
        let command = brain.step(now);
        assert_eq!(brain.mode, Mode::Turn);
        assert!(command[2] < 0.0);
    }

    #[test]
    fn searches_then_stops_when_the_target_is_lost() {
        let mut config = config();
        config.search_for = Duration::from_secs(8);
        config.imu_timeout = Duration::from_secs(30);
        let (mut brain, now) = brain_with_imu(config);

        let first = brain.step(now);
        assert_eq!(brain.mode, Mode::Search);
        assert_eq!(first[2], -1.50);

        // One direction for the whole search: no sweep back and forth.
        let later = brain.step(now + Duration::from_millis(4000));
        assert_eq!(brain.mode, Mode::Search);
        assert_eq!(later[2], -1.50);

        let stopped = brain.step(now + Duration::from_millis(8100));
        assert_eq!(brain.mode, Mode::Stopped);
        assert_eq!(stopped, ZERO);
    }

    #[test]
    fn a_stale_target_starts_the_search() {
        let (mut brain, now) = brain_with_imu(config());
        assert!(brain.observe_target(&target(true, 0.0, 0.01), now));
        assert_eq!(brain.step(now), [0.30, 0.0, 0.0]);

        let command = brain.step(now + Duration::from_millis(301));
        assert_eq!(brain.mode, Mode::Search);
        assert_eq!(command[2], -1.50);
    }

    #[test]
    fn stops_on_tilt() {
        let mut brain = Brain::new(config());
        let half = std::f32::consts::FRAC_1_SQRT_2;
        let tilted = [half, half, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert!(brain.observe_imu(&tilted, Instant::now()));
        assert_eq!(brain.step(Instant::now()), ZERO);
        assert_eq!(brain.mode, Mode::Tilt);
    }

    #[test]
    fn refuses_to_drive_without_an_imu() {
        let mut brain = Brain::new(config());
        assert!(brain.observe_target(&target(true, 0.0, 0.01), Instant::now()));
        assert_eq!(brain.step(Instant::now()), ZERO);
        assert_eq!(brain.mode, Mode::NoImu);
    }

    #[test]
    fn can_run_without_an_imu_when_asked() {
        let mut config = config();
        config.require_imu = false;
        let mut brain = Brain::new(config);
        assert!(brain.observe_target(&target(true, 0.0, 0.01), Instant::now()));
        assert_eq!(brain.step(Instant::now()), [0.30, 0.0, 0.0]);
    }

    #[test]
    fn a_stale_imu_stops_the_command() {
        let (mut brain, now) = brain_with_imu(config());
        assert!(brain.observe_target(&target(true, 0.0, 0.01), now + Duration::from_millis(400)));
        assert_eq!(brain.step(now + Duration::from_millis(600)), ZERO);
        assert_eq!(brain.mode, Mode::NoImu);
    }

    #[test]
    fn tilt_comes_from_the_quaternion() {
        assert!(tilt_rad(&identity()).unwrap() < 1e-6);
        let half = std::f32::consts::FRAC_1_SQRT_2;
        let quarter = [half, half, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert!((tilt_rad(&quarter).unwrap() - std::f32::consts::FRAC_PI_2).abs() < 1e-5);
        assert!(tilt_rad(&[0.0; 7]).is_none());
    }

    #[test]
    fn rejects_malformed_messages() {
        let mut brain = Brain::new(config());
        assert!(!brain.observe_target(&[1.0, 0.0], Instant::now()));
        assert!(!brain.observe_target(&[1.0, f32::NAN, 0.0], Instant::now()));
        assert!(!brain.observe_imu(&[1.0, 0.0, 0.0], Instant::now()));
    }

    #[test]
    fn without_search_it_stops_when_the_target_is_lost() {
        let mut config = config();
        config.search_for = Duration::ZERO;
        let (mut brain, now) = brain_with_imu(config);
        assert_eq!(brain.step(now), ZERO);
        assert_eq!(brain.mode, Mode::Stopped);
    }
}
