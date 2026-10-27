# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A [dora-rs](https://dora-rs.ai/) dataflow that drives a [Microduck](https://github.com/pollen-robotics/microduck) robot. The upstream Microduck source lives in `vendor/microduck` as a git submodule. This repo commits the submodule pointer plus the dataflow crates — never the submodule's source.

## Prerequisites

- **Rust**: `rust-toolchain.toml` pins the `stable` channel. The workspace MSRV is 1.95 because `dora-node-api` 1.1.0 requires it.
- **dora CLI must match the `dora-node-api` version** in `Cargo.toml` (currently 1.1.0). Node/daemon protocol is not compatible across versions; bumping the crate without reinstalling the CLI leaves a dataflow that starts but cannot communicate.
- **The submodule must be initialized** before building — the path dependencies point into it.
- **The simulated camera is optional** (`DUCK_SIM_CAMERAS=a`): it needs GStreamer. On macOS `brew install gstreamer libnice-gstreamer`; on a board the upstream `vendor/microduck/scripts/setup-gstreamer.sh` installs it. The dora nodes themselves do not link GStreamer; the real `mediad` does.
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

# Raw frames out of the dora camera node (one .uyvy + one .json per dump).
DUCK_SIM_CAMERAS=a FRAME_DUMP_DIR=/tmp/duck-frames scripts/dora-sim run --stop-after 5

# Dataflow only (the twin must already be running for cmd_vel to reach robotd).
dora run dataflows/sim.yml
dora run dataflows/sim.yml --stop-after 10   # 1.1.0 accepts bare seconds

# Inspect the graph without starting it
dora graph dataflows/sim.yml --mermaid
```

- `dora run` starts an isolated local daemon and exits once the dataflow stops; `dora up` is not needed.
- Per-run logs and the session file land in `dataflows/out/`, which is gitignored.
- `cargo test --workspace` runs the keyboard parser tests plus the state/camera/frame-sink unit tests; target one crate with `cargo test -p dora-keyboard`.
- `dora run dataflows/sim.yml --stop-after 4` without `ROBOTD_SOCKET`/`MEDIA_FRAME_SOCKET` is a graph-only smoke test: `robotd-state` logs that no state will be published, `camera` logs that no frames will be published, the wrapper logs one `cmd_vel … (no ROBOTD_SOCKET)` line, and without a controlling terminal the keyboard logs "keyboard disabled" and publishes zeros. That is expected. The real end-to-end test is `scripts/dora-sim run --stop-after 5`, which should show `dora-robotd-state: subscribed to …` and `published N robot states`, then stop the twin cleanly. Add `DUCK_SIM_CAMERAS=a` to exercise `dora-camera`; that path needs GStreamer, and `duck-sim` refuses to start a camera without it.
- `scripts/dora-sim run` is the end-to-end launcher test: it should print "standing", connect the wrapper to `target/duck-sim/state/duck-a.sock`, run the dataflow, then "stopping the MuJoCo twin" with no leftover processes.

## Architecture

### Workspace layout

- Root `Cargo.toml` is a **virtual workspace** (`members = ["crates/*"]`, `exclude = ["vendor/microduck"]`). It has no `[package]`.
- `vendor/microduck` has its **own** workspace and `Cargo.lock`. Never add its crates as members here. Depend on them through `[workspace.dependencies]` path entries, which is how `duck-ipc-proto` is consumed.
- The root `Cargo.lock` is the workspace lock and is committed. The submodule's lockfile belongs to upstream and is not used by this workspace.
- Submodule crates inherit fields (`version`, `edition`, deps) from the *submodule's* workspace root, not this one. That is expected.

### Nodes

- `crates/dora-keyboard` — reads keys from `/dev/tty` (dora gives nodes a null stdin) and publishes `cmd_vel` (`[vx, vy, vyaw]`) on a 50 ms tick. Hold to move: W/S vx, A/D vy, Q/E yaw; X stops immediately. Terminals report no key-up, so a key is held while it repeats — the first window is 800 ms to cover the OS repeat delay, then 250 ms once repeats are flowing. Each axis times out on its own: a key on one axis must not keep another axis's last value alive. Ctrl-C quits and restores the terminal, then signals dora's process group so one press stops the dataflow. Arrow and space sequences are swallowed on purpose — the MuJoCo viewer binds them to stepping and pausing. The parser consumes whole escape sequences (CSI with parameters, SS3, `ESC`+key) rather than one byte after `ESC [`, because a stray final `D` would strafe. dora can spawn the node in a background process group, so it claims the tty's foreground group and restores it on exit; `first key received` in the node log confirms input works. Defaults are 0.30 forward, 0.40 backward, 0.50 strafe and 1.50 yaw: every axis has a deadband (about 0.25 m/s forward, 0.30–0.35 backward, ~0.4 m/s strafe), and below ~1.2 rad/s it barely yaws, so smaller "gentle" values only wobble the duck. Backward needs more than forward: −0.30 moves nothing at all. Without a tty it warns and publishes zeros.
- `crates/dora-robotd-state` — subscribes to `robot.subscribe` on `ROBOTD_SOCKET` and publishes the measured state on a 20 ms tick: `joint_state` (15 `f32`), `odom` (`[x, y, z, yaw]`) and `imu` (`[qw, qx, qy, qz, gx, gy, gz]`). A reader thread reconnects and resubscribes when robotd restarts; with `ROBOTD_SOCKET` unset it logs and publishes nothing.
- `crates/dora-camera` — pulls `media.frame` on `MEDIA_FRAME_SOCKET` at 33 ms, writes the raw UYVY bytes straight into a dora sample (IPC header first, then `read_exact` into the sample's data region), and publishes `image` with `width`/`height`/`format`/`rotate` metadata. `media.frame` answers one frame per connection, so the node reconnects each tick and logs only the first connection (or a recovery). It installs a warn-level tracing subscriber so dora's own `zenoh direct path` / `daemon path` diagnostics are visible. With the socket unset it logs and publishes nothing.
- `crates/dora-frame-sink` — debug consumer for `image`: reads the `UInt8Array` in place and logs every 30th frame. It exists to exercise the shared-memory path until `perception` replaces it. Set `FRAME_DUMP_DIR` (with `FRAME_DUMP_EVERY`, default 30, and `FRAME_DUMP_LIMIT`, default 10; 0 = unlimited) to write the raw UYVY frames plus a `.json` sidecar; this is a debug path, not the production route.
- `crates/dora-mujoco-bridge` — a deterministic synthetic gait over the 15 joints, retained as a graph-only stub. It is **not** in `dataflows/sim.yml`: the upstream twin already runs the real `robotd` against MuJoCo, and a second simulator in the graph would disagree with it.
- `crates/dora-robotd-wrapper` — turns `cmd_vel` into a `robot.move` JSON-RPC notification on `ROBOTD_SOCKET` (only logged when unset). It is a pure intent sink; state lives in `dora-robotd-state`.

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
- `frame-sink` is a debug consumer, not part of the target architecture; `perception` replaces it.
- The keyboard node needs a controlling terminal; `dora run` from a non-tty (CI, an agent shell) only exercises the zero-command path.

### Simulation launcher (A1)

`scripts/dora-sim` is deliberately **outside** the dataflow:

1. It extracts the MuJoCo body from `microduck_rl` (`origin/develop`, overridable with `MICRODUCK_RL_REF`) into `target/duck-sim/rl`, symlinks that checkout's `.venv`, and never modifies the other repository. Override the source checkout with `MICRODUCK_RL`.
2. It starts the upstream `vendor/microduck/scripts/duck-sim`, which owns daemon build, policy fetch, params, sockets, health checks and standing.
3. It turns terminal echo off first (`stty -g` saved, restored by the exit trap). The twin takes about ten seconds to come up, and the keyboard node does not exist until after that; without this, keys pressed while waiting are echoed by the driver as `^[[A`.
4. It exports `ROBOTD_SOCKET` and runs `dora run` in the foreground. With `DUCK_SIM_CAMERAS=a` it also exports `MEDIA_FRAME_SOCKET` for the simulated camera's `media.frame` socket; that option needs GStreamer, and the upstream launcher refuses to start a camera without it.
5. On exit it runs `duck-sim down` **only if it started the twin**; an already-running twin is left alone.

`scripts/dora-sim camera` is a separate viewer, not part of the dataflow: it starts the twin with `DUCK_SIM_CAMERAS` set to the selected duck's letter if none is running, then runs `scripts/duck-camera-view` (Python + OpenCV) against `media.frame`. If it started the twin, the exit trap stops it; an already-running twin is left alone. The viewer is a debug tool, not a dora node. It does **not** run dora or the keyboard: to walk while watching, run `scripts/dora-sim run` and `scripts/dora-sim camera` in two terminals and focus the dora terminal before pressing keys, or use `scripts/dora-sim drive`. The view is portrait because the camera is mounted 90°; the `apartment` scene currently segfaults `body_server` on macOS, so the default scene is the safe one.

Do **not** turn the upstream daemons into dora nodes. `robotd` is binary-only and safety-critical; `btd`/`configd`/`updaterd` must survive a dead `robotd` and depend on systemd units and socket permissions. Dora owns the graph; systemd/upstream owns the daemons. The one seam worth taking over later is the MuJoCo body protocol from `duck_control::sim` (`hello/read/write/gain/torque/slow`, protocol 1).

### Dependency constraints

- `robotd` is a **binary-only** crate (no `src/lib.rs`), so it cannot be used as a path dependency. Talk to it over IPC and use `duck-ipc-proto` for the contract types.
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

