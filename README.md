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
│   ├── dora-keyboard/          # 终端键盘 → cmd_vel 节点
│   └── dora-mujoco-bridge/     # MuJoCo 仿真节点
├── dataflows/
│   └── sim.yml                 # 仿真数据流定义
├── scripts/
│   └── dora-sim                # 一键启动：MuJoCo twin + dora 数据流
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

## 一键启动仿真

```bash
scripts/dora-sim                  # 启动 twin → 跑 dora 数据流 → 退出时关闭 twin
scripts/dora-sim run --stop-after 5
scripts/dora-sim up               # 只启动 twin，保持运行
scripts/dora-sim status           # health / 50 Hz / standing
scripts/dora-sim drive 0.3        # 向前走 8 秒
scripts/dora-sim ctl health       # 任意 robotctl 命令
scripts/dora-sim down             # 关闭 twin
```

- `run` 必须在前台终端执行：键盘节点读 `/dev/tty`，dora 给节点的 stdin 是 null
- 脚本会自动设置 `ROBOTD_SOCKET=target/duck-sim/state/duck-a.sock`，`cmd_vel` 直接进模拟的 robotd
- MuJoCo body 从 `microduck_rl` 的 `origin/develop` 提取到 `target/duck-sim/rl`（不修改那个仓库）；可用 `MICRODUCK_RL`、`MICRODUCK_RL_REF`、`DUCK_SIM_DUCK` 覆盖
- 如果 twin 已经在运行，`run` 会复用它，退出时也不会关掉它；只有本次启动的才会在退出时 `down`

## 手动分步运行

```bash
cargo build
dora run dataflows/sim.yml
```

`dora run` 直接在本机执行数据流，不需要先 `dora up`。想定时退出可以加 `--stop-after 5s`。

数据流拓扑：

```
keyboard ──cmd_vel──▶ robotd-wrapper ──robot.move──▶ robotd（设置 ROBOTD_SOCKET 时）

dora/timer/millis/10 ──▶ mujoco-bridge ──joint_state──▶ robotd-wrapper
                              ▲                              │
                              └──────── joint_command ───────┘
```

`mujoco-bridge` 和 `robotd-wrapper` 构成反馈环：仿真节点发布关节状态，wrapper 返回 robotd 会下发的指令。`keyboard` 节点的 `cmd_vel` 由 wrapper 转成 `robot.move`；未设置 `ROBOTD_SOCKET` 时只打印日志。

## 键盘遥操作

`dora run` 要在终端里执行：dora 给节点的 stdin 是 null，`keyboard` 节点直接读 `/dev/tty`。没有控制终端时它打印警告并发布零，不会让数据流失败。

| 键 | 效果 |
|---|---|
| W / S | vx = +0.30 / -0.30 m/s |
| A / D | vy = +0.50 / -0.50 m/s |
| Q / E | vyaw = +1.50 / -1.50 rad/s |
| X | 立即停止 |
| Ctrl-C | 恢复终端并停止整个数据流 |

命令是**按住才动**的：按住 W 前进，松开后约 250 ms 停；系统的首次按键重复有几百毫秒延迟，所以第一次按下会先保持 800 ms，等重复开始后改用 250 ms 判断松开。W 和 Q 同时按住就是“前进 + 转弯”。X 可以立即停。

速度值不是随便取的，这套策略对横向和转向有明显的死区（实测，每次 5 秒）：

| 指令 | 结果 |
|---|---|
| vx 0.30 | 前进 0.57 m（约 0.11 m/s） |
| vy 0.15 / 0.30 | 几乎不动（毫米级） |
| vy 0.50 | 左移 0.41 m |
| vyaw 0.50 / 1.00 | 3 秒内只转 0.07 / 0.14 rad |
| vyaw 1.50 | 0.73–0.96 rad/s |
| vyaw 2.50 | 约 1.48 rad/s |

所以 A/D 和 Q/E 取的是能让策略真正执行的值；调小到“看起来温和”的数值，鸭子只会晃一晃。

启动时节点会打印 `/dev/tty ready (pgrp X, foreground Y)`。如果两个值不同，说明 dora 把它放进了后台进程组，它会自动接管终端前台（`took the terminal's foreground group`），退出时再还回去。按下任意键后出现 `first key received` 表示输入通路正常；没有这一行，说明按键去了别的窗口（通常是 MuJoCo viewer），点一下终端再试。

在键盘节点启动之前（尤其是 twin 启动的十几秒里），终端还是规范模式且开着回显，此时按键会被回显成 `^[[A` 这样的字符。`scripts/dora-sim` 会在启动 twin 前先关掉回显，节点接管后再进入 raw 模式，退出时统一恢复；节点自己也会丢掉启动前积压的输入。

方向键和空格不映射：MuJoCo viewer 用它们做单步和暂停。方向键的各种形式（`ESC [ A`、`ESC O A`、带参数的 `ESC [ 1 ; 5 D`、粘贴标记 `ESC [ 200 ~`）都会被节点整条吞掉，序列末尾的 `D` 不会误触发右平移；Option+字母（`ESC w`）同理。

默认 vx 取 0.30，是因为这套 MuJoCo 模型在 0.25 m/s 以下基本不走（实测 0.20 三秒只移动 1 mm）。

要真正驱动 robotd：

```bash
export ROBOTD_SOCKET=/path/to/duck-a.sock
dora run dataflows/sim.yml
```

## 当前状态

这是骨架，三个节点都能构建、能跑通数据流，但关键的下游连接还在补：

- `dora-keyboard`：终端键盘已经可用（`/dev/tty` + termios raw mode）。
- `dora-robotd-wrapper`：`cmd_vel` 已经能转成 `robot.move` 发到 `ROBOTD_SOCKET`；`joint_state` 仍然是原样回传的占位实现（见 `src/main.rs` 中的 `TODO`），尚未订阅 robotd 的真实状态流。
- `dora-mujoco-bridge`：还没有引入 MuJoCo 绑定，目前用一个确定性的合成步态代替仿真。

## 关于 Cargo.lock

根目录的 `Cargo.lock` 是**整个 workspace** 的锁文件，必须提交。子模块 `vendor/microduck` 有它自己的 `Cargo.lock`，属于上游仓库，本仓库不使用也不重复提交。

`crates/*` 通过 path 依赖引用子模块中的 crate 时，根 `Cargo.lock` 只会记录这些 crate 及其依赖的版本，不会把子模块源码带进本仓库。
