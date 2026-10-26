# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A [dora-rs](https://dora-rs.ai/) dataflow that drives a [Microduck](https://github.com/pollen-robotics/microduck) robot. The upstream Microduck source lives in `vendor/microduck` as a git submodule. This repo commits the submodule pointer plus the dataflow crates — never the submodule's source.

## Prerequisites

- **Rust**: `rust-toolchain.toml` pins the `stable` channel. The workspace MSRV is 1.95 because `dora-node-api` 1.1.0 requires it.
- **dora CLI must match the `dora-node-api` version** in `Cargo.toml` (currently 1.1.0). Node/daemon protocol is not compatible across versions; bumping the crate without reinstalling the CLI leaves a dataflow that starts but cannot communicate.
- **The submodule must be initialized** before building — the path dependencies point into it.

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

# Dataflow only (the twin must already be running for cmd_vel to reach robotd).
dora run dataflows/sim.yml
dora run dataflows/sim.yml --stop-after 10   # 1.1.0 accepts bare seconds

# Inspect the graph without starting it
dora graph dataflows/sim.yml --mermaid
```

- `dora run` starts an isolated local daemon and exits once the dataflow stops; `dora up` is not needed.
- Per-run logs and the session file land in `dataflows/out/`, which is gitignored.
- `cargo test --workspace` runs the keyboard parser tests; target one crate with `cargo test -p dora-keyboard`.
- `dora run dataflows/sim.yml --stop-after 4` is the quick end-to-end smoke test: expect "stepped N times over 15 joints" and "forwarded N joint states (15 joints each)", then all three nodes finishing successfully. Without a controlling terminal the keyboard node logs "keyboard disabled" and publishes zeros; that is the expected non-interactive result.
- `scripts/dora-sim run` is the end-to-end launcher test: it should print "standing", connect the wrapper to `target/duck-sim/state/duck-a.sock`, run the dataflow, then "stopping the MuJoCo twin" with no leftover processes.

## Architecture

### Workspace layout

- Root `Cargo.toml` is a **virtual workspace** (`members = ["crates/*"]`, `exclude = ["vendor/microduck"]`). It has no `[package]`.
- `vendor/microduck` has its **own** workspace and `Cargo.lock`. Never add its crates as members here. Depend on them through `[workspace.dependencies]` path entries, which is how `duck-ipc-proto` is consumed.
- The root `Cargo.lock` is the workspace lock and is committed. The submodule's lockfile belongs to upstream and is not used by this workspace.
- Submodule crates inherit fields (`version`, `edition`, deps) from the *submodule's* workspace root, not this one. That is expected.

### Nodes

- `crates/dora-keyboard` — reads held keys from `/dev/tty` (dora gives nodes a null stdin) and publishes `cmd_vel` (`[vx, vy, vyaw]`) on a 50 ms tick. A terminal cannot report key release, so a key counts as held for 400 ms after its last repeat. Defaults are 0.30 / 0.15 / 0.50; 0.30 is above the ~0.25 m/s deadband the MuJoCo twin shows. Without a tty it warns and publishes zeros.
- `crates/dora-mujoco-bridge` — owns the clock (`dora/timer/millis/10`), advances the simulation, publishes `joint_state`. Currently a deterministic synthetic gait; the MuJoCo binding is still a TODO.
- `crates/dora-robotd-wrapper` — turns `cmd_vel` into a `robot.move` JSON-RPC notification on `ROBOTD_SOCKET` (only logged when unset). It also consumes `joint_state`, validates the length against `duck_ipc_proto::JOINT_NAMES` (15 joints), and publishes `joint_command`; that mirror is the stub for the real state stream.

### Dataflow

`dataflows/sim.yml` forms a feedback loop, with the keyboard feeding the wrapper:

```
keyboard ──cmd_vel──▶ robotd-wrapper ──robot.move──▶ robotd (ROBOTD_SOCKET)

dora/timer/millis/10 ──▶ mujoco-bridge ──joint_state──▶ robotd-wrapper
                              ▲                              │
                              └──────── joint_command ───────┘
```

- `path:` in the YAML is resolved **relative to the dataflow file's directory**, hence `../target/debug/<bin>`.
- dora supports this cycle (verified); it is message-passing, not a topological DAG restriction.
- Payloads are **Arrow arrays of `f32`** via `IntoArrow` / `TryFrom<&ArrowData>`, not JSON. Joint count and order must come from `duck_ipc_proto::JOINT_NAMES`.
- The keyboard node needs a controlling terminal; `dora run` from a non-tty (CI, an agent shell) only exercises the zero-command path.

### Simulation launcher (A1)

`scripts/dora-sim` is deliberately **outside** the dataflow:

1. It extracts the MuJoCo body from `microduck_rl` (`origin/develop`, overridable with `MICRODUCK_RL_REF`) into `target/duck-sim/rl`, symlinks that checkout's `.venv`, and never modifies the other repository. Override the source checkout with `MICRODUCK_RL`.
2. It starts the upstream `vendor/microduck/scripts/duck-sim`, which owns daemon build, policy fetch, params, sockets, health checks and standing.
3. It exports `ROBOTD_SOCKET` and runs `dora run` in the foreground.
4. On exit it runs `duck-sim down` **only if it started the twin**; an already-running twin is left alone.

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
- Push only when the user asks. This repo currently has **no remote** configured.
- `main` is the only branch and the first commit is its root commit. If a remote and a shared default branch are added later, put new work on a topic branch instead of committing to the default branch directly.
- Before committing, run `cargo clippy --workspace --all-targets -- -D warnings`; for dataflow changes also run the 4-second `dora run` smoke test from the Commands section.

