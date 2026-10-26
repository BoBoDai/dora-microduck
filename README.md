# dora-microduck

用 [dora-rs](https://dora-rs.ai/) 数据流驱动 [Microduck](https://github.com/pollen-robotics/microduck) 实验的项目仓库。

上游 Microduck 以 git submodule 形式放在 `vendor/microduck`，本仓库只提交子模块的 commit 指针（gitlink），不包含其源码。

## 目录结构

```
.
├── Cargo.toml                  # 虚拟 workspace：members = crates/*，exclude = vendor/microduck
├── Cargo.lock                  # workspace 级锁文件，提交
├── crates/
│   ├── dora-robotd-wrapper/    # 把 robotd 接入 dora 图的节点
│   └── dora-mujoco-bridge/     # MuJoCo 仿真节点
├── dataflows/
│   └── sim.yml                 # 仿真数据流定义
├── vendor/
│   └── microduck/              # git submodule（上游仓库，不随本仓库提交）
└── README.md
```

## 前置条件

- Rust 1.95+（stable；仓库内的 `rust-toolchain.toml` 已固定到 stable）
- dora CLI 1.1.0（`dora-node-api` 与此版本对齐，节点与 daemon 的协议必须一致）

```bash
dora --version   # dora-cli 1.1.0
```

如果尚未安装（或版本不是 1.1.0）：

```bash
cargo install dora-cli --version 1.1.0 --locked
```

## 初始化

克隆时一并拉取子模块：

```bash
git clone --recurse-submodules <repo-url>
```

已经克隆过的话：

```bash
git submodule update --init --recursive
```

子模块未初始化时，`crates/*` 对 `vendor/microduck/duck-ipc-proto` 的 path 依赖会导致 cargo 直接报错（而不是一条友好的缺依赖提示）。

## 构建与运行

```bash
cargo build
dora run dataflows/sim.yml
```

`dora run` 直接在本机执行数据流，不需要先 `dora up`。想定时退出可以加 `--stop-after 5s`。

数据流拓扑：

```
dora/timer/millis/10 ──▶ mujoco-bridge ──joint_state──▶ robotd-wrapper
                              ▲                              │
                              └──────── joint_command ───────┘
```

两节点构成反馈环：仿真节点发布关节状态，wrapper 返回 robotd 会下发的指令。

## 当前状态

这是骨架，两个节点都能构建、能跑通数据流，但关键的下游连接尚未接入：

- `dora-robotd-wrapper`：还没有真正连接 robotd 的 IPC socket，目前把收到的关节状态原样回传（见 `src/main.rs` 中的 `TODO`）。后续使用 `duck_ipc_proto` 里的请求/响应类型实现。
- `dora-mujoco-bridge`：还没有引入 MuJoCo 绑定，目前用一个确定性的合成步态代替仿真。

## 关于 Cargo.lock

根目录的 `Cargo.lock` 是**整个 workspace** 的锁文件，必须提交。子模块 `vendor/microduck` 有它自己的 `Cargo.lock`，属于上游仓库，本仓库不使用也不重复提交。

`crates/*` 通过 path 依赖引用子模块中的 crate 时，根 `Cargo.lock` 只会记录这些 crate 及其依赖的版本，不会把子模块源码带进本仓库。
