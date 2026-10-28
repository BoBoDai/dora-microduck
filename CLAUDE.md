# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A [dora-rs](https://dora-rs.ai/) dataflow that drives a [Microduck](https://github.com/pollen-robotics/microduck) robot. The upstream Microduck source lives in `vendor/microduck` as a git submodule. This repo commits the submodule pointer plus the dataflow crates — never the submodule's source.

## Prerequisites

- **Rust**: `rust-toolchain.toml` pins the `stable` channel. The workspace MSRV is 1.95 because `dora-node-api` 1.1.0 requires it.
- **dora CLI must match the `dora-node-api` version** in `Cargo.toml` (currently 1.1.0). Node/daemon protocol is not compatible across versions; bumping the crate without reinstalling the CLI leaves a dataflow that starts but cannot communicate.
- **The submodule must be initialized** before building — the path dependencies point into it.
- **The simulated camera is optional** (`DUCK_SIM_CAMERAS=a`): it needs GStreamer. On macOS `brew install gstreamer libnice-gstreamer`; on a board the upstream `vendor/microduck/scripts/setup-gstreamer.sh` installs it. The dora nodes themselves do not link GStreamer; the real `mediad` does. `scripts/dora-sim reactive` enables the simulated camera, so it needs GStreamer too.
- **`scripts/dora-sim camera` needs Python 3 + OpenCV/numpy.** Without `cv2`, use `scripts/dora-sim run` and the frame-sink dump instead.

```bash
git submodule update --init --recursive
dora --version    # must report 1.1.0
```

If the CLI is missing or stale:

```bash
cargo install dora-cli --version 1.1.0 --locked
```

## Commands

```bash
cargo build --workspace
cargo check --workspace
cargo clippy --workspace --all-targets -- -D warnings

# One command: start the MuJoCo twin, run the dataflow, stop the twin.
scripts/dora-sim
scripts/dora-sim run --stop-after 5
scripts/dora-sim up | status | drive 0.3 | ctl health | down
scripts/dora-sim camera           # OpenCV window over media.frame; starts the twin if needed
scripts/dora-sim reactive         # camera -> perception -> brain; starts the target-scene twin
scripts/dora-sim autod            # upstream 12-state brain, split into dora nodes

# Raw frames out of the dora camera node (one .uyvy + one .json per dump).
DUCK_SIM_CAMERAS=a FRAME_DUMP_DIR=/tmp/duck-frames scripts/dora-sim run --stop-after 5

# Dataflow only (the twin must already be running for cmd_vel to reach robotd).
dora run dataflows/sim.yml
dora run dataflows/sim.yml --stop-after 10   # 1.1.0 accepts bare seconds
dora run dataflows/reactive.yml               # no keyboard; needs IMU and frames to move
dora run dataflows/autod.yml                  # robotd-state + tof + autod + wrapper

# Inspect a graph without starting it
dora graph dataflows/sim.yml --mermaid
dora graph dataflows/reactive.yml --mermaid
```

- `dora run` starts an isolated local daemon and exits once the dataflow stops; `dora up` is not needed.
- Per-run logs and the session file land in `dataflows/out/`, which is gitignored.
- `cargo test --workspace` runs the keyboard parser tests plus the state/camera/frame-sink/perception/brain/autod/tof unit tests; target one crate with `cargo test -p dora-keyboard`. The perception tests build synthetic UYVY frames (including the 90° mount mapping); the brain tests cover bearing sign, hysteresis, close/release, search timeout, stale IMU and tilt. `dora-autod` carries the 20 upstream tests plus adapter tests, including a ten-minute behaviour soak.
- `dora run dataflows/sim.yml --stop-after 4` without `ROBOTD_SOCKET`/`MEDIA_FRAME_SOCKET` is a graph-only smoke test: `robotd-state` logs that no state will be published, `camera` logs that no frames will be published, the wrapper logs one `cmd_vel … (no ROBOTD_SOCKET)` line, and without a controlling terminal the keyboard logs "keyboard disabled" and publishes zeros. That is expected. The real end-to-end test is `scripts/dora-sim run --stop-after 5`, which should show `dora-robotd-state: subscribed to …` and `published N robot states`, then stop the twin cleanly. Add `DUCK_SIM_CAMERAS=a` to exercise `dora-camera`; that path needs GStreamer, and `duck-sim` refuses to start a camera without it.
- `dora run dataflows/reactive.yml --stop-after 4` without sockets is the graph-only autonomous smoke test: the wrapper logs one zero `cmd_vel`, `camera`/`robotd-state` log their missing sockets, and `brain` stays in `mode=no-imu` because `BRAIN_REQUIRE_IMU` defaults to 1. That is deliberate: the brain must not drive without state. `BRAIN_REQUIRE_IMU=0` is only for isolating the graph.
- `scripts/dora-sim reactive --stop-after 12` is the end-to-end autonomy test: it copies `scenes/scene_target.xml` next to the simulator's robot model, starts the twin with `DUCK_SIM_SCENE=target DUCK_SIM_KEYFRAME=HOME DUCK_SIM_CAMERAS=a`, and should log `visible=true`, `mode=turn`, `mode=forward`, then `mode=arrived`; the odom should move forward and then stop. It stops the twin on exit only if it started it.
- `scripts/dora-sim autod --stop-after 20` is the upstream-port test: it should start/reuse the twin, set `ROBOTD_SOCKET`/`TOF_SOCKET`, log `dora-tof: connected`/frame lines and `dora-autod: first state ... abilities ...`, emit `→ state=...` transitions (Ruffle/LookAround/Chill/Wander are plausible in 20 s), and move odom when Wander starts. The wrapper should connect and receive `cmd_vel/head/mouth/body/sound`. Use `DUCK_SIM_VIEWER=0` when the macOS MuJoCo window hangs.
- `scripts/dora-sim run` is the end-to-end launcher test: it should print "standing", connect the wrapper to `target/duck-sim/state/duck-a.sock`, run the dataflow, then "stopping the MuJoCo twin" with no leftover processes.

## Architecture

### Workspace layout

- Root `Cargo.toml` is a **virtual workspace** (`members = ["crates/*"]`, `exclude = ["vendor/microduck"]`). It has no `[package]`.
- `vendor/microduck` has its **own** workspace and `Cargo.lock`. Never add its crates as members here. Depend on them through `[workspace.dependencies]` path entries, which is how `duck-ipc-proto` is consumed.
- The root `Cargo.lock` is the workspace lock and is committed. The submodule's lockfile belongs to upstream and is not used by this workspace.
- Submodule crates inherit fields (`version`, `edition`, deps) from the *submodule's* workspace root, not this one. That is expected.

### Nodes

- `crates/dora-keyboard` — reads keys from `/dev/tty` (dora gives nodes a null stdin) and publishes `cmd_vel` (`[vx, vy, vyaw]`) on a 50 ms tick. Hold to move: W/S vx, A/D vy, Q/E yaw; X stops immediately. Terminals report no key-up, so a key is held while it repeats — the first window is 800 ms to cover the OS repeat delay, then 250 ms once repeats are flowing. Each axis times out on its own: a key on one axis must not keep another axis's last value alive. Ctrl-C quits and restores the terminal, then signals dora's process group so one press stops the dataflow. Arrow and space sequences are swallowed on purpose — the MuJoCo viewer binds them to stepping and pausing. The parser consumes whole escape sequences (CSI with parameters, SS3, `ESC`+key) rather than one byte after `ESC [`, because a stray final `D` would strafe. dora can spawn the node in a background process group, so it claims the tty's foreground group and restores it on exit; `first key received` in the node log confirms input works. Defaults are 0.30 forward, 0.40 backward, 0.50 strafe and 1.50 yaw: every axis has a deadband (about 0.25 m/s forward, 0.30–0.35 backward, ~0.4 m/s strafe), and below ~1.2 rad/s it barely yaws, so smaller "gentle" values only wobble the duck. Backward needs more than forward: −0.30 moves nothing at all. Without a tty it warns and publishes zeros.
- `crates/dora-robotd-state` — subscribes to `robot.subscribe` on `ROBOTD_SOCKET` and publishes the measured state on a 20 ms tick: `joint_state` (15 `f32`), `odom` (`[x, y, z, yaw]`), `imu` (`[qw, qx, qy, qz, gx, gy, gz]`) and `robot_state` (compact JSON: `t_ns`, policy, fallen, sitting, head joints, gravity, trunk height, pose, and the subscribe ack's `ground_pick`/`sitstand` abilities). A reader thread reconnects and resubscribes when robotd restarts; with `ROBOTD_SOCKET` unset it logs and publishes nothing.
- `crates/dora-camera` — pulls `media.frame` on `MEDIA_FRAME_SOCKET` at 33 ms, writes the raw UYVY bytes straight into a dora sample (IPC header first, then `read_exact` into the sample's data region), and publishes `image` with `width`/`height`/`format`/`rotate` metadata. `media.frame` answers one frame per connection, so the node reconnects each tick and logs only the first connection (or a recovery). It installs a warn-level tracing subscriber so dora's own `zenoh direct path` / `daemon path` diagnostics are visible. With the socket unset it logs and publishes nothing.
- `crates/dora-perception` — consumes `image` and publishes `target` as `f32[3]` (`[visible, bearing (-1 left, +1 right), area fraction]`). It is a colour threshold, not a model: UYVY chroma for saturated orange/red, centroid of matching pixels, minimum area `PERCEPTION_MIN_AREA` (default 0.001). It reads `width`/`height`/`format`/`rotate` metadata and maps raw coordinates into the upright image before computing bearing (`rotate=90` means `x_upright = height - 1 - y_raw`), so the 460 KB frame is never rotated. Thresholds are `PERCEPTION_V_MIN`/`U_MAX`/`Y_MIN`. Invalid frames are logged and not published.
- `crates/dora-brain` — consumes `target` and `imu`, publishes `cmd_vel` at 50 ms. It is a reactive v0, not the 16-state brain: bang-bang turn/forward with hysteresis (`BRAIN_ALIGN_START=0.45`, `BRAIN_ALIGN_STOP=0.30`), close latch (`BRAIN_CLOSE_AREA=0.06`, release `0.02`), target timeout 300 ms, search right for up to `BRAIN_SEARCH_MS` (8 s), tilt limit 45°, and IMU timeout 500 ms. `BRAIN_REQUIRE_IMU=1` default refuses to drive without state. Positive bearing (target right) produces negative `vyaw` because `robot.move` defines positive yaw as left. Values are intentionally at the policy's deadband edges, not "gentle".
- `crates/dora-frame-sink` — debug consumer for `image` in `sim.yml`: reads the `UInt8Array` in place and logs every 30th frame. It exercises the shared-memory path and is the frame-dump tool. Set `FRAME_DUMP_DIR` (with `FRAME_DUMP_EVERY`, default 30, and `FRAME_DUMP_LIMIT`, default 10; 0 = unlimited) to write the raw UYVY frames plus a `.json` sidecar; this is a debug path, not the production route.
- `crates/dora-mujoco-bridge` — a deterministic synthetic gait over the 15 joints, retained as a graph-only stub. It is **not** in `dataflows/sim.yml`: the upstream twin already runs the real `robotd` against MuJoCo, and a second simulator in the graph would disagree with it.
- `crates/dora-robotd-wrapper` — the single intent sink. `cmd_vel` becomes `robot.move`; `head`/`mouth`/`body`/`sound` become `robot.head`/`mouth`/`pose`/`sound` notifications; `skill` becomes a `robot.do` request and its reply is logged. Only inputs connected in a graph are sent; with `ROBOTD_SOCKET` unset every intent is logged once (`cmd_vel` when it changes). State lives in `dora-robotd-state`.

### Dataflow

`dataflows/sim.yml` has five nodes; the state, camera, and control paths are separate:

```
keyboard ──cmd_vel──▶ robotd-wrapper ──robot.move──▶ robotd (ROBOTD_SOCKET)

robotd ──robot.state──▶ robotd-state ──joint_state / odom / imu──▶ (behaviour nodes)

mediad ──media.frame──▶ camera ──image (SHM)──▶ frame-sink (debug)
```

- `path:` in the YAML is resolved **relative to the dataflow file's directory**, hence `../target/debug/<bin>`.
- Payloads are **Arrow arrays of `f32`** via `IntoArrow` / `TryFrom<&ArrowData>`, not JSON. Joint count and order must come from `duck_ipc_proto::JOINT_NAMES`.
- Both `robotd-state` and `robotd-wrapper` read `ROBOTD_SOCKET`; `camera` reads `MEDIA_FRAME_SOCKET`. Unset means no state or no frames and logged commands, not a failed dataflow.
- `camera` declares `shared_memory_pool_size: 64MB`; a 720p UYVY frame is ~2 MB in the sample (payload plus the UInt8 validity bitmap), so the default 8 MB pool would only hold a few frames.
- `frame-sink` is a debug consumer, not part of the target architecture; `perception` is the real consumer in `reactive.yml`.
- The keyboard node needs a controlling terminal; `dora run` from a non-tty (CI, an agent shell) only exercises the zero-command path.

`dataflows/reactive.yml` is the separate autonomous graph, not a superset of `sim.yml`:

```
robotd-state ──imu──▶ brain
camera ──image (SHM)──▶ perception ──target──▶ brain ──cmd_vel──▶ robotd-wrapper
```

- It has no keyboard and no `frame-sink`; `brain` is the only `cmd_vel` writer.
- `target` is `f32[3]`; `imu` is `f32[7]`; `cmd_vel` is `f32[3]`.
- `scripts/dora-sim reactive` sets `DUCK_SIM_SCENE=target`, `DUCK_SIM_KEYFRAME=HOME`, `DUCK_SIM_CAMERAS` to the selected duck's letter (unless overridden), copies `scenes/scene_target.xml` to the simulator's robot directory, and exports `MEDIA_FRAME_SOCKET`.
- Do not run `sim.yml`, `reactive.yml` and `autod.yml` against the same robot: each writes intents, and `autod` writes the whole body. An arbiter is the next step.

`dataflows/autod.yml` is the upstream brain as four nodes:

```
robotd-state ──robot_state──▶ autod ──cmd_vel/head/mouth/body/sound/skill──▶ robotd-wrapper
tof          ──tof──────────▶
```

- `robotd-state` and `tof` are the only source nodes with sockets; `autod` is pure dora; the
  wrapper is the sink. `tof`'s JSON is the `duck_ipc_proto::TofFrame`, so the wire contract is
  not duplicated.
- `autod` calls the brain only on its 20 ms tick, and sends nothing while `busy`/`off` or while
  `robot_state` is older than 500 ms, so it does not fight a skill, a disabled policy, or a dead
  state stream (the deadman stops the duck).
- The target scene has no keyframes; `HOME` is a placement, not a keyframe. The copied file must sit next to `robot_groundcontact.xml` because its `meshdir="assets"` is relative to that directory. `scene_ball.xml`'s ball is kicked away on startup and is not a stable perception target.

### Simulation launcher (A1)

`scripts/dora-sim` is deliberately **outside** the dataflow:

1. It extracts the MuJoCo body from `microduck_rl` (`origin/develop`, overridable with `MICRODUCK_RL_REF`) into `target/duck-sim/rl`, symlinks that checkout's `.venv`, and never modifies the other repository. Override the source checkout with `MICRODUCK_RL`.
2. It starts the upstream `vendor/microduck/scripts/duck-sim`, which owns daemon build, policy fetch, params, sockets, health checks and standing.
3. It turns terminal echo off first (`stty -g` saved, restored by the exit trap). The twin takes about ten seconds to come up, and the keyboard node does not exist until after that; without this, keys pressed while waiting are echoed by the driver as `^[[A`.
4. It exports `ROBOTD_SOCKET` and runs `dora run` in the foreground. With `DUCK_SIM_CAMERAS=a` it also exports `MEDIA_FRAME_SOCKET` for the simulated camera's `media.frame` socket; that option needs GStreamer, and the upstream launcher refuses to start a camera without it.
5. On exit it runs `duck-sim down` **only if it started the twin**; an already-running twin is left alone.

`scripts/dora-sim camera` is a separate viewer, not part of the dataflow: it starts the twin with `DUCK_SIM_CAMERAS` set to the selected duck's letter if none is running, then runs `scripts/duck-camera-view` (Python + OpenCV) against `media.frame`. If it started the twin, the exit trap stops it; an already-running twin is left alone. The viewer is a debug tool, not a dora node. It does **not** run dora or the keyboard: to walk while watching, run `scripts/dora-sim run` and `scripts/dora-sim camera` in two terminals and focus the dora terminal before pressing keys, or use `scripts/dora-sim drive`. The view is portrait because the camera is mounted 90°; the `apartment` scene currently segfaults `body_server` on macOS, so the default scene is the safe one.

`scripts/dora-sim autod` is the upstream-brain launcher: it builds `dora-robotd-state`, `dora-tof`, `dora-autod` and `dora-robotd-wrapper`, starts/reuses the twin, exports the absolute `ROBOTD_SOCKET` and `TOF_SOCKET` (`target/duck-sim/state/duck-a-tof.sock`), and runs `dora run dataflows/autod.yml`. It has no keyboard and does not need GStreamer. `DUCK_SIM_VIEWER=0` avoids the macOS MuJoCo window when it hangs. A twin started without ToF cannot support it (the brain only looks around); the launcher dies if the ToF socket is absent.

`scripts/dora-sim reactive` is the autonomy launcher. It builds the reactive nodes, copies `scenes/scene_target.xml` into the extracted simulator's `robot/microduck/` directory (next to `robot_groundcontact.xml`, because the included model's `meshdir` is relative), and starts/reuses the twin with `DUCK_SIM_SCENE=target`, `DUCK_SIM_KEYFRAME=HOME`, `DUCK_SIM_CAMERAS=a` unless those are already set. It exports `MEDIA_FRAME_SOCKET` and runs `dora run dataflows/reactive.yml`. If a twin was already running with another scene it warns and reuses it; the brain then has no orange target and falls back to search. It stops the twin on exit only if it started it. The target is a fixed orange sphere at 0.8 m, no collision and no freejoint, so it stays visible instead of being kicked away like `scene_ball.xml`.

Do **not** turn the upstream daemons into dora nodes. `robotd` is binary-only and safety-critical; `btd`/`configd`/`updaterd` must survive a dead `robotd` and depend on systemd units and socket permissions. Dora owns the graph; systemd/upstream owns the daemons. The one seam worth taking over later is the MuJoCo body protocol from `duck_control::sim` (`hello/read/write/gain/torque/slow`, protocol 1).

### Dependency constraints

- `robotd` is a **binary-only** crate (no `src/lib.rs`), so it cannot be used as a path dependency. Talk to it over IPC and use `duck-ipc-proto` for the contract types.
- `kinematics` is a workspace dependency for `dora-autod`'s ToF reprojection; it is pure Rust (`roxmltree`), not the `tof` driver crate.
- Prefer the light submodule crates (`duck-ipc-proto`, `robotd-params`, `kinematics`). Depending on `duck-control` pulls in `ort`, `rustypot`, and `serialport`, which need native toolchains.

### Version coupling

- `dora-node-api` ↔ `dora-cli` must stay equal.
- When the dora MSRV changes, update both `rust-version` in `Cargo.toml` and the note in `rust-toolchain.toml`.
- `cargo update` after bumping dora is fine, but keep `Cargo.lock` committed.

## Conventions

- Source comments and commit messages are in English; `README.md` is in Chinese.
- Do not edit files inside `vendor/microduck` as part of a change here. Upstream changes belong in that repository; bump the submodule pointer when they are needed.
- Keep the stub/TODO markers accurate: if a node gains a real backend, update both its module docs and the "当前状态" section of `README.md`.

## Git conventions

### What goes into a commit

- **Submodule**: only the `160000` gitlink for `vendor/microduck` and the `.gitmodules` entry. Never commit its source, its own `Cargo.lock`, or anything else under `vendor/microduck/`.
- **`Cargo.lock`**: always commit it. It is the workspace lockfile and must move together with `Cargo.toml`.
- **Never commit**: `target/`, `.idea/`, `dataflows/out/`, OS files. Do not use `git add -f` to bypass `.gitignore` for generated output.
- **Version bumps**: a dora upgrade changes `Cargo.toml` (dependency and `rust-version`), `Cargo.lock`, `rust-toolchain.toml`, and the prerequisites in `README.md` in the same commit.
- Keep related changes together; keep pure refactors separate from behavior changes.

### Commit messages

Use [Conventional Commits](https://www.conventionalcommits.org/), in English, imperative mood:

```
<type>(<scope>): <subject>
```

- **type** (required): `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, `revert`.
  - `feat` — user-visible capability or behavior (a node actually does something new)
  - `fix` — bug fix
  - `build` / `deps` — dependency or toolchain bumps (e.g. a dora version upgrade)
  - `docs` — `README.md`, `CLAUDE.md`, comments-only changes
  - `chore` — housekeeping that fits no other type (`.gitignore`, file moves)
- **scope** (optional): the crate or area, e.g. `robotd-wrapper`, `mujoco-bridge`, `dora`, `dataflow`, `deps`. Omit it for workspace-wide changes.
- **subject**: lowercase after the colon, imperative, ≤ 72 characters including the prefix, no trailing period.
- **One line only**: no body and no footer. If the reason is not obvious, fold a short clause into the subject.
- **Breaking change**: add `!` before the colon (`feat(deps)!: require rustc 1.95`).
- No tool/agent attribution.
- The existing root commit predates this rule; do not rewrite history to retrofit it.

Examples:

```
feat: scaffold dora dataflow workspace for Microduck
```

```
fix(robotd-wrapper): reject joint states with the wrong length
```

```
build(deps): bump dora to 1.1.0
```

### When to commit and push

- Commit only when the user asks; do not commit as a side effect of another task.
- Push only when the user asks. The remote is `origin` = `git@github.com:BoBoDai/dora-microduck.git` (SSH; HTTPS auth failed).
- `main` is the only branch and the first commit is its root commit. If a remote and a shared default branch are added later, put new work on a topic branch instead of committing to the default branch directly.
- Before committing, run `cargo clippy --workspace --all-targets -- -D warnings`; for dataflow changes also run the 4-second `dora run` smoke test from the Commands section.

