# dora-microduck

用 [dora-rs](https://dora-rs.ai/) 数据流驱动 [Microduck](https://github.com/pollen-robotics/microduck) 实验的项目仓库。

上游 Microduck 以 git submodule 形式放在 `vendor/microduck`，本仓库只提交子模块的 commit 指针（gitlink），不包含其源码。

## 目录结构

```
.
├── Cargo.toml                  # 虚拟 workspace：members = crates/*，exclude = vendor/microduck
├── Cargo.lock                  # workspace 级锁文件，提交
├── crates/
│   ├── dora-robotd-state/      # robot.subscribe → joint_state / odom / imu
│   ├── dora-camera/            # media.frame → image（SHM）
│   ├── dora-perception/        # image → target（可见 / 方位 / 面积）
│   ├── dora-brain/             # target + imu → cmd_vel
│   ├── dora-autod/             # 上游 autod：12 状态行为机 + ToF 避障
│   ├── dora-frame-sink/        # 调试消费者：原地读 image，不拷贝
│   ├── dora-robotd-wrapper/    # cmd_vel → robot.move
│   ├── dora-keyboard/          # 终端键盘 → cmd_vel
│   └── dora-mujoco-bridge/     # 合成关节状态 stub（不在数据流中使用）
├── dataflows/
│   ├── sim.yml                 # 遥操作：键盘 + 相机调试消费者
│   ├── reactive.yml            # 反应式行为：camera → perception → brain
│   └── autod.yml               # 上游 autod 大脑（单节点，直连 robotd/tofd）
├── scenes/
│   └── scene_target.xml        # 演示用世界：鸭子前方 0.8 m 一个固定橙色目标
├── docs/
│   ├── architecture.md         # 键盘实现、上游架构与 dora 插入点
│   └── call-graph.md           # 调用关系与调用流程
├── scripts/
│   ├── dora-sim                # 一键启动：MuJoCo twin + dora 数据流
│   └── duck-camera-view        # media.frame 相机窗口（Python + OpenCV）
├── vendor/
│   └── microduck/              # git submodule（上游仓库，不随本仓库提交）
└── README.md
```

## 文档

- [`docs/architecture.md`](docs/architecture.md)：键盘实现、上游 Microduck 架构、dora 的插入点和当前状态。
- [`docs/call-graph.md`](docs/call-graph.md)：调用关系图、调用流程和调用表。

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

**可选：模拟相机。** 在这个仓库的仿真里，只有 `DUCK_SIM_CAMERAS=a` 时才需要 GStreamer。macOS：

```bash
brew install gstreamer libnice-gstreamer
```

真机的 `mediad` 本身也依赖 GStreamer；板子上由上游的
`vendor/microduck/scripts/setup-gstreamer.sh` 安装（provisioning / preinstall 会自动跑）。没有
GStreamer 时数据流照常运行，只是 `camera` 节点拿不到帧。

**可选：相机窗口。** `scripts/dora-sim camera` 需要 Python 3 + OpenCV/numpy：

```bash
python3 -c "import cv2, numpy"
# 没有的话：pip install opencv-python
```

## 初始化

克隆时一并拉取子模块（普通 `git clone` 不会拉，`vendor/microduck` 会是空目录）：

```bash
git clone --recurse-submodules git@github.com:BoBoDai/dora-microduck.git
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
DUCK_SIM_CAMERAS=a scripts/dora-sim   # 同时启动模拟相机（需要 GStreamer）
scripts/dora-sim camera           # 打开相机窗口（必要时先启动 twin）
scripts/dora-sim reactive         # 反应式行为：相机看目标 → 转向/前进 → 靠近停下
scripts/dora-sim autod            # 上游 12 状态大脑 + ToF 避障
scripts/dora-sim up               # 只启动 twin，保持运行
scripts/dora-sim status           # health / 50 Hz / standing
scripts/dora-sim drive 0.3        # 向前走 8 秒
scripts/dora-sim ctl health       # 任意 robotctl 命令
scripts/dora-sim down             # 关闭 twin
```

- `run` 必须在前台终端执行：键盘节点读 `/dev/tty`，dora 给节点的 stdin 是 null
- 脚本会自动设置 `ROBOTD_SOCKET=target/duck-sim/state/duck-a.sock`：`cmd_vel` 直接进模拟的 robotd，`robotd-state` 也从这里订阅状态
- 启用模拟相机时，脚本还会设置 `MEDIA_FRAME_SOCKET=target/duck-sim/state/duck-a-frame.sock`；没启用时脚本提示 `camera disabled`，`camera` 节点打印 `MEDIA_FRAME_SOCKET is not set; no frames will be published`，其余图照常运行
- MuJoCo body 从 `microduck_rl` 的 `origin/develop` 提取到 `target/duck-sim/rl`（不修改那个仓库）；可用 `MICRODUCK_RL`、`MICRODUCK_RL_REF`、`DUCK_SIM_DUCK` 覆盖
- 如果 twin 已经在运行，`run` 会复用它，退出时也不会关掉它；只有本次启动的才会在退出时 `down`

### 看相机窗口

```bash
scripts/dora-sim camera
```

- 打开一个 OpenCV 窗口，显示 `media.frame` 的原始帧，并已按相机安装角（仿真 90°）转正；
- 如果 twin 没在运行，这条命令会先用当前 `DUCK_SIM_DUCK` 对应的 `DUCK_SIM_CAMERAS` 字母（默认 `a`）启动它；退出时如果 twin 是它启动的，会一并关闭；已经在运行的 twin 不会被动；
- 关闭窗口、按 `q` 或 Esc 退出；
- 默认场景是光板地面，窗口里可能只是一片地面/网格；想看房间用
  `DUCK_SIM_SCENE=apartment scripts/dora-sim camera`；不想同时看到 MuJoCo 仿真窗口就加
  `DUCK_SIM_VIEWER=0`；
- 当前 macOS + microduck_rl 的 `apartment` 场景会让 MuJoCo `body_server` segfault；遇到就回到默认场景；
- 相机是 90° 侧装的，转正后画面是竖屏（仿真 360×640，真机 720×1280），所以窗口是竖条——这是安装方向，不是裁切；
- `camera` 只负责看画面，不运行 dora/键盘。要边走边看，用两个终端：
  `DUCK_SIM_CAMERAS=a scripts/dora-sim run` 和 `scripts/dora-sim camera`（后者会复用已启动的 twin）。按 W 前先点回运行 dora 的终端，相机窗口会抢焦点；或者 twin 已经起来时直接
  `scripts/dora-sim drive 0.3`；
- 需要 Python 的 OpenCV/numpy：`python3 -c "import cv2"`。没有的话 `pip install opencv-python`；
- 只想快速验证、不弹窗：`scripts/duck-camera-view --socket target/duck-sim/state/duck-a-frame.sock --frames 3 --no-window`。

这条流程不需要浏览器或 WebRTC：窗口直接读 `media.frame` 的原始帧。

## 反应式行为（perception + brain）

```bash
scripts/dora-sim reactive              # 启动 twin + 自主数据流，Ctrl-C 结束
scripts/dora-sim reactive --stop-after 12
```

这条命令用 `dataflows/reactive.yml`，跑的是最小闭环；它需要模拟相机，所以需要 GStreamer
（见“前置条件”）：

```
camera ──image（SHM）──▶ perception ──target──▶ brain ──cmd_vel──▶ robotd-wrapper ──robot.move──▶ robotd
                                                    ▲
                          robotd-state ──imu────────┘
```

`target` 是 Arrow `f32[3]`：`[visible, bearing, area]`。`bearing` 在 `[-1, 1]`，正数表示目标在
右边；`area` 是目标像素占整帧的比例。`perception` 不做模型推理，只对 UYVY 的色度做橙色阈值，
取匹配像素的质心；这已经够让 `brain` 转向、前进和停下。后续换 NPU 检测器时，保持这个
消息契约即可。

相机是 90° 侧装的，但 `perception` 不旋转整帧：它用 `rotate` 元数据把原始坐标映射到转正后的
横坐标，再算方位。仿真的原始帧是 640×360，转正后是 360×640 竖屏。

### 演示世界

`scripts/dora-sim reactive` 默认把 `scenes/scene_target.xml` 复制到仿真器的 robot 目录，然后以
`DUCK_SIM_SCENE=target DUCK_SIM_KEYFRAME=HOME DUCK_SIM_CAMERAS=a` 启动 twin。场景就是标准
ground-contact 鸭子，正前方 0.8 m 放一个固定的橙色球；球没有碰撞体、没有 freejoint，不会被
站起来的动作踢走，相机能稳定看到它。场景文件必须复制到 `robot_groundcontact.xml` 旁边，因为
被 include 的模型用相对的 `meshdir="assets"`。

如果 twin 已经在运行，`reactive` 会复用它；但那个 twin 如果不是用 `target` 场景启动的，就没有
橙色目标，`brain` 会搜索一段时间后停下。先 `scripts/dora-sim down` 再重跑即可。

### 行为

`brain` 每 50 ms 发一次 `cmd_vel`，使用**开关式**控制而不是比例控制，因为策略对速度和转向都有
死区：前进低于约 0.25 m/s、转向低于约 1.2 rad/s 时几乎不动，给小值只会让鸭子原地晃。

| 条件 | 动作 |
|---|---|
| `abs(bearing) > 0.45` | 原地转向目标（`vyaw = ±1.50`） |
| `abs(bearing) < 0.30` | 前进（`vx = 0.30`） |
| 两者之间 | 保持上一步，避免抖动 |
| `area ≥ 0.06` | 停下，锁定 `arrived` |
| `area < 0.02` | 解除锁定，重新靠近 |
| 目标消失超过 300 ms | 向右转搜索，最多 8 s，然后停 |
| IMU 倾角 > 45° 或 IMU 超过 500 ms 没更新 | 立即发零 |
| 没有 IMU（`ROBOTD_SOCKET` 未设置） | 默认拒绝驱动，打 `mode=no-imu` |

这些值都可以用 `BRAIN_*` 环境变量改，例如：

```bash
BRAIN_FORWARD=0.35 BRAIN_CLOSE_AREA=0.08 scripts/dora-sim reactive
```

| 变量 | 默认 | 说明 |
|---|---|---|
| `BRAIN_FORWARD` / `BRAIN_TURN` | 0.30 / 1.50 | 前进 m/s、转向 rad/s |
| `BRAIN_ALIGN_START` / `BRAIN_ALIGN_STOP` | 0.45 / 0.30 | 转向的进入/退出方位门限 |
| `BRAIN_CLOSE_AREA` / `BRAIN_CLOSE_RELEASE` | 0.06 / 0.02 | 停下的进入/退出面积门限 |
| `BRAIN_TARGET_TIMEOUT_MS` | 300 | 多久没收到可见目标算丢失 |
| `BRAIN_SEARCH_MS` | 8000 | 搜索多久后停；`0` = 丢失立即停 |
| `BRAIN_TILT_LIMIT_DEG` | 45 | 倾角保护 |
| `BRAIN_REQUIRE_IMU` | 1 | 没有 IMU 时是否拒绝驱动（只做图测试可设 0） |

`perception` 的阈值也可以调：`PERCEPTION_V_MIN`、`PERCEPTION_U_MAX`、`PERCEPTION_Y_MIN`
和 `PERCEPTION_MIN_AREA`（占整帧比例，默认 0.001）。

### 日志与验收

一次成功的运行会看到：

```
dora-camera: ... frame N: 640x360 UYVY rotate=90 ...
dora-perception: frame M: visible=true bearing=+0.23 area=0.013
dora-brain: mode=turn cmd=[0.00, 0.00, -1.50] ...
dora-brain: mode=forward cmd=[0.30, 0.00, 0.00] ...
dora-brain: mode=arrived cmd=[0.00, 0.00, 0.00] ...
dora-robotd-state: published 400 robot states; odom=[0.32, 0.02, ...]
```

在本机的 `target` 场景里实测：鸭子先向右找到目标，转向后前进约 0.3 m，在离目标约 0.4–0.5 m
处停下并保持；`area` 在靠近时从约 0.013 长到约 0.06。目标丢失时会转圈搜索，8 s 后停。

### 边界

- **`sim.yml` 和 `reactive.yml` 不要同时跑。** 两张图都有自己的 `cmd_vel` 写者（键盘 / brain），
  同时运行会变成 last-writer-wins。现在还没有 arbiter；要手动接管，先停掉 `reactive`。
- `reactive.yml` 里没有键盘节点；想用键盘用 `scripts/dora-sim run`。
- 这是**行为层 v0**，不是上游 16 状态大脑：没有 mood、记忆、探索、NPU 检测，也没有多目标
  选择。它只证明 `感知 → 决策 → intent` 的图能闭环。
- 停下的距离由 `area` 决定，受相机视野和头部遮挡影响；想更近/更远就调
  `BRAIN_CLOSE_AREA`。目标太近时会掉出画面，所以不是“贴上去”。
- `robotd` 的安全层、deadman 和跌倒失力仍然有效；`brain` 只是另一个 intent 发送方。

## 上游 autod 的 12 状态大脑（移植）

```bash
scripts/dora-sim autod                 # 启动 twin + 上游 autod，Ctrl-C 结束
scripts/dora-sim autod --stop-after 30
DUCK_SIM_VIEWER=0 scripts/dora-sim autod   # 不弹 MuJoCo 窗口
```

这不是 `reactive.yml` 的放大版，而是把上游 `autod` 原样移植成一个 dora 节点。源分支是
`pollen-robotics/microduck` 的 `origin/autonomous-mode`，提交 `df9a750`；`brain.rs` 和
`obstacle.rs` 是上游代码，保留 Apache-2.0 许可证（`crates/dora-autod/LICENSE`）。

**16 还是 12？** 上游文档说的是 16 状态，但当前移植分支故意没有带 `BallPlay`、`Held`、
`Startle` 和 `Preen`：BallPlay 缺球检测器，Held 的抱起步检测被废弃，Startle/Preen 效果
不好。实际移植的是 12 个状态：

| 状态 | 做什么 |
|---|---|
| Chill | 站着，缓慢小幅度头部漂移，呼吸式张嘴 |
| LookAround | 大幅度张望、歪头 |
| Wander | 走向里程计坐标系里的目标点（1–3 m），用 novelty grid 避开走过的格子 |
| TurnInPlace | 原地转向，通常朝最少去过的方向 |
| Zoomies | 全速乱窜，只在 ToF 前方新鲜且开阔时进入 |
| Stretch | 伸脖、打哈欠、咕咕叫、甩头 |
| Ruffle | 快速抖羽毛 |
| Dance | 身体姿态律动，头部跟着摇 |
| Sneeze | 吸气、憋住、CHOO、发懵 |
| GroundPick | 跑 `ground_pick` 技能，然后开心地叫 |
| Nap | `sit_toggle`：打瞌睡或坐着放松，结束再站起来 |
| Petted | 被摸时的融化/兴奋/困倦反应（音频输入还没接，当前不会触发） |

它还带 **energy/mood**（走路消耗、休息恢复，影响状态权重、速度和叫的频率）、**novelty
grid**（0.5 m 格子、约 15 分钟衰减）、**ToF 避障**（新鲜度、距离、左右哪边更空、卡住时
转出去）和随机叫声。这些都来自上游，不是这次重写的。

### 数据流形状

`autod.yml` 只有一个节点，和上游 daemon 一样直接连 socket：

```
robotd ──robot.state（50 Hz）──┐
                               ├──▶ dora-autod ──▶ robot.move / robot.head / robot.mouth
tofd   ──tof.stream（15 Hz）───┘                └──▶ robot.pose / robot.sound / robot.do
```

- `ROBOTD_SOCKET` 必需；`TOF_SOCKET` 可选，但没有 ToF 时它只张望、不前进（上游规则：
  没看过的路不走）。
- `scripts/dora-sim autod` 会设置两个 socket，并启动/复用 twin。
- 它**直接发 intent**，不经过 `robotd-wrapper`；这是第一版为了保持上游形状。之后可以把
  `robot.state` 和 `tof.stream` 拆成 dora topic，让图里能看见它们。
- 它不读相机，也不需要 GStreamer；相机链路和 `reactive.yml` 是另一条。

### 当前没接的部分

- **麦克风**：`heard` 和 `petting` 固定为 `None`/`false`，所以 `Petted` 不会触发，声音
  反应只剩自主叫声。
- **手柄接管**：上游用 `robot.state.move.source` 判断 `padd` 正在驾驶，然后让位；当前
  proto 还没有这个字段，所以不要一边用手柄一边跑 autod，先 `scripts/dora-sim down`。
- **开机自站**：上游 `autod` 会在每次 boot 第一次 `robot.init`/`robot.enable`；当前版本
  不做，交给 `scripts/dora-sim` 的 standing 或机器人自己的 Start。
- **v38 的 `robot.head alpha`**：主分支 proto 没有这个字段，快速甩头的幅度会被 `robotd`
  的默认平滑吃掉。
- **BallPlay/Held/Startle/Preen**：上游自己也没带，见上。
- **dora 化**：状态/ToF 仍走 Unix socket，不是 dora topic；这是下一步，不是已完成。

### 验收

`cargo test -p dora-autod` 跑了上游的 20 个测试：十分钟行为不越界、Wander 走到目标、目标
避开障碍、没有 ToF 时绝不前进、GroundPick 等待真正执行、坐着的人会被站起来、声音回答等。
本机 `scripts/dora-sim autod --stop-after 60` 实测依次出现 `LookAround → Chill → Dance →
Zoomies → LookAround → TurnInPlace → Wander`，odom 有位移，ToF 日志出现
`depth frames arriving — obstacle avoidance on`，没有协议拒绝。

## 手动分步运行

```bash
cargo build
dora run dataflows/sim.yml
```

`dora run` 直接在本机执行数据流，不需要先 `dora up`。想定时退出可以加 `--stop-after 5s`。

数据流拓扑：

```
keyboard ──cmd_vel──▶ robotd-wrapper ──robot.move──▶ robotd（设置 ROBOTD_SOCKET 时）

robotd ──robot.state（订阅）──▶ robotd-state ──joint_state / odom / imu──▶（行为节点）

mediad ──media.frame──▶ camera ──image（SHM）──▶ frame-sink（调试消费者）
```

`robotd-state` 通过 `ROBOTD_SOCKET` 发 `robot.subscribe`，把真实的关节角、里程计和 IMU 发布进图；未设置 socket 时它打印 `no state will be published`，数据流仍然运行。`camera` 通过 `MEDIA_FRAME_SOCKET` 拉取 `media.frame` 的原始 UYVY 帧，直接写进 dora 的 SHM sample；`frame-sink` 是调试消费者，原地读 `Uint8Array`。`keyboard` 的 `cmd_vel` 由 wrapper 转成 `robot.move`；未设置 socket 时只打印日志。上游 twin 已经在跑真 `robotd` + MuJoCo，图里没有第二个仿真器。

自主图是另一张 YAML：

```bash
dora run dataflows/reactive.yml
# 或者用 scripts/dora-sim reactive 连 twin 一起启动
```

拓扑是 `camera → perception → brain → robotd-wrapper`，再加上 `robotd-state → brain` 的 IMU；
没有键盘。两张图不要同时跑，原因见上面的“边界”。

上游 autod 是第三张图，只有一个节点：

```bash
ROBOTD_SOCKET=... TOF_SOCKET=... dora run dataflows/autod.yml
```

它自己连 socket；`scripts/dora-sim autod` 会把两个地址设好。也不要和 `sim.yml`/`reactive.yml`
同时跑：它直接写所有 intent。

## 键盘遥操作

`dora run` 要在终端里执行：dora 给节点的 stdin 是 null，`keyboard` 节点直接读 `/dev/tty`。没有控制终端时它打印警告并发布零，不会让数据流失败。

| 键 | 效果 |
|---|---|
| W / S | vx = +0.30 / -0.40 m/s |
| A / D | vy = +0.50 / -0.50 m/s |
| Q / E | vyaw = +1.50 / -1.50 rad/s |
| X | 立即停止 |
| Ctrl-C | 恢复终端并停止整个数据流 |

命令是**按住才动**的：按住 W 前进，松开后约 250 ms 停；系统的首次按键重复有几百毫秒延迟，所以第一次按下会先保持 800 ms，等重复开始后改用 250 ms 判断松开。W 和 Q 同时按住就是“前进 + 转弯”。X 可以立即停。三个轴各自计时：松开 W 再按 A，前进速度会在 W 停止重复后独立到期，不会被 A 的重复一直续命。

速度值不是随便取的，这套策略对横向和转向有明显的死区（实测，每次 5 秒）：

| 指令 | 结果 |
|---|---|
| vx 0.30 | 前进 0.57 m（约 0.11 m/s） |
| vx -0.30 | 完全不动（后退死区比前进高） |
| vx -0.35 | 后退 0.87 m（约 0.17 m/s） |
| vx -0.40 | 后退 1.07 m（约 0.21 m/s） |
| vy 0.15 / 0.30 | 几乎不动（毫米级） |
| vy 0.50 | 左移 0.41 m |
| vyaw 0.50 / 1.00 | 3 秒内只转 0.07 / 0.14 rad |
| vyaw 1.50 | 0.73–0.96 rad/s |
| vyaw 2.50 | 约 1.48 rad/s |

所以 A/D 和 Q/E 取的是能让策略真正执行的值；调小到“看起来温和”的数值，鸭子只会晃一晃。

启动时节点会打印 `/dev/tty ready (pgrp X, foreground Y)`。如果两个值不同，说明 dora 把它放进了后台进程组，它会自动接管终端前台（`took the terminal's foreground group`），退出时再还回去。按下任意键后出现 `first key received` 表示输入通路正常；没有这一行，说明按键去了别的窗口（通常是 MuJoCo viewer），点一下终端再试。

在键盘节点启动之前（尤其是 twin 启动的十几秒里），终端还是规范模式且开着回显，此时按键会被回显成 `^[[A` 这样的字符。`scripts/dora-sim` 会在启动 twin 前先关掉回显，节点接管后再进入 raw 模式，退出时统一恢复；节点自己也会丢掉启动前积压的输入。

方向键和空格不映射：MuJoCo viewer 用它们做单步和暂停。方向键的各种形式（`ESC [ A`、`ESC O A`、带参数的 `ESC [ 1 ; 5 D`、粘贴标记 `ESC [ 200 ~`）都会被节点整条吞掉，序列末尾的 `D` 不会误触发右平移；Option+字母（`ESC w`）同理。

默认前进 vx 取 0.30，是因为这套 MuJoCo 模型在 0.25 m/s 以下基本不走（实测 0.20 三秒只移动 1 mm）。后退的死区更高：-0.30 完全不动，-0.35 起才稳定迈步，所以 S 取 -0.40 留出余量；这也是“站着按 S 不动”的原因。

要真正驱动 robotd、读到状态并拿到相机帧：

```bash
export ROBOTD_SOCKET=/path/to/duck-a.sock
export MEDIA_FRAME_SOCKET=/run/mediad/media.sock   # 真机
dora run dataflows/sim.yml
```

`robotd-wrapper` 和 `robotd-state` 都读 `ROBOTD_SOCKET`：前者发 `robot.move`，后者订阅 `robot.state`。`camera` 读 `MEDIA_FRAME_SOCKET`；不设置时它只打印 `no frames will be published`。

## 构建与测试

```bash
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
```

`cargo test --workspace` 覆盖键盘解析、状态节点的解析/转换、相机节点的协议头解析、帧导出逻辑、
perception 的 UYVY 检测/旋转映射、brain 的转向/前进/到达/搜索/倾角逻辑，以及 autod 的
20 个上游行为/避障测试。端到端 smoke test：

```bash
scripts/dora-sim run --stop-after 5
# 带相机：DUCK_SIM_CAMERAS=a scripts/dora-sim run --stop-after 5
# 自主闭环：scripts/dora-sim reactive --stop-after 12
# 上游大脑：DUCK_SIM_VIEWER=0 scripts/dora-sim autod --stop-after 12
```

## 当前状态

控制链、状态链、相机数据面和最小行为闭环现在都能跑：

- `dora-keyboard`：终端键盘已经可用（`/dev/tty` + termios raw mode）。
- `dora-robotd-wrapper`：纯 intent sink，`cmd_vel` 转成 `robot.move` 发到 `ROBOTD_SOCKET`；不再镜像 `joint_state`。
- `dora-robotd-state`：通过 `robot.subscribe` 订阅 `robot.state`，发布 `joint_state`（15 个关节）、`odom` 和 `imu`；robotd 重启后会自动重连。
- `dora-camera`：通过 `media.frame` 拉取原始 UYVY 帧，直接写进 SHM sample；`shared_memory_pool_size: 64MB`。已在真实 sim 相机上验证（640×360、30 fps、`zenoh direct path` + SHM）；`scripts/dora-sim camera` 直接看画面，`FRAME_DUMP_DIR` 导出帧。
- `dora-perception`：橙色目标阈值 + 质心，发布 `target = [visible, bearing, area]`；按 `rotate` 把原始坐标映射到转正后的水平轴，不旋转整帧。
- `dora-brain`：反应式行为 v0；转向/前进/到达/搜索，带方位滞回、目标超时、IMU 倾角和 `arrived` 锁定。已在 `target` 场景上跑通：找到目标 → 前进约 0.3 m → 停下保持。
- `dora-autod`：上游 `autod` 的行为机移植（12 状态、energy/mood、novelty grid、ToF 避障），作为单个 dora 节点直接连 `robotd`/`tofd`。已在 sim 上跑出 Chill/LookAround/Dance/Zoomies/Wander 和位移；20 个上游测试通过。见上面的“上游 autod 的 12 状态大脑”。
- `dora-frame-sink`：调试消费者，原地读 `Uint8Array` 并打印帧信息；支持 `FRAME_DUMP_DIR` 导出。它和 `perception` 分别属于 `sim.yml` 和 `reactive.yml`。
- `dora-mujoco-bridge`：仍是确定性合成步态的 stub，已从数据流移除；上游 twin 的 MuJoCo 才是仿真来源。

下一步是把 autod 的 `robot.state`/`tof.stream` 拆成 dora topic、补上手柄接管/音频输入，
再做 intent arbiter（键盘、reactive brain、autod 安全共存）、NPU 鸭子检测器，以及上游
故意没带的四个状态；见 `docs/architecture.md` §4。

## 调试：导出 dora 收到的相机帧

`dora-frame-sink` 支持把**它实际收到的** `image` 原样写到磁盘，用于确认 dora 这条路径看到的画面：

```bash
DUCK_SIM_CAMERAS=a FRAME_DUMP_DIR=/tmp/duck-frames scripts/dora-sim run --stop-after 5
ls /tmp/duck-frames
# frame-000001.uyvy
# frame-000001.json
```

`run` 会复用已经运行的 twin；如果它启动时没有相机，先 `scripts/dora-sim down` 再执行上面的命令。

- `FRAME_DUMP_EVERY`：每多少帧导出一张，默认 30（30 fps 下约 1 张/秒）；
- `FRAME_DUMP_LIMIT`：最多导出多少张，默认 10；`0` 表示不限制；
- `.json` sidecar 带 `width`、`height`、`format`、`rotate`、`captured_at_unix_us`；
- 原始帧是未旋转的 UYVY。仿真相机是 640×360、安装角 90°，转成 PNG：

```bash
ffmpeg -f rawvideo -pix_fmt uyvy422 -s 640x360 \
  -i /tmp/duck-frames/frame-000001.uyvy \
  -vf transpose=1 /tmp/frame.png
open /tmp/frame.png
```

真机如果配的是 720p/1080p，把 `-s` 换成 `1280x720` / `1920x1080`；sidecar 里有准确尺寸。`transpose=1` 是顺时针 90°，和上游 `uyvy::Turn::Right` 的安装修正方向一致。

## 常见问题

**相机窗口为什么是竖的？**
相机在头上是 90° 侧装的；传感器是 640×360 横屏（真机 1280×720），转正后就是 360×640（真机 720×1280）竖屏。这是安装方向，不是裁切。上游 `robotctl monitor` 的相机块也是同样的竖幅。

**打开 `camera` 窗口后按 W 不动？**
`scripts/dora-sim camera` 只开窗口，不运行 dora/键盘。要边走边看，用两个终端：`DUCK_SIM_CAMERAS=a scripts/dora-sim run` 和 `scripts/dora-sim camera`；按 W 前点回运行 dora 的终端（相机窗口会抢焦点）。或者 twin 已经起来时用 `scripts/dora-sim drive 0.3`。

**为什么浏览器打不开 `:8080`？**
那是上游 `mediad` 自带的 WebRTC 控制台，不是本项目的相机窗口流程，也不是必须的。它需要 twin 常驻且 WebRTC 正常；本项目用 `scripts/dora-sim camera` 直接读 `media.frame`，不经过浏览器。

**`scripts/dora-sim reactive` 里鸭子不动？**
先看日志：`mode=no-imu` 表示没收到 `robotd-state` 的 IMU（`ROBOTD_SOCKET` 没设或 twin 没起），
`visible=false` 且一直 `mode=search` 表示画面里没有橙色目标。用 `scripts/dora-sim down` 后重跑，
让脚本用 `scene_target.xml` 起 twin；或者只做图测试时设 `BRAIN_REQUIRE_IMU=0`。

**reactive 为什么停在离目标还有半米？**
停下由 `area` 决定，不是距离传感器；目标太近会掉出相机视野或被脖子/头部遮挡
`area ≥ 0.06` 时就停。想靠近一点就调大 `BRAIN_CLOSE_AREA`（例如 0.08），但可能出现一停就走、
或者在视野边缘丢失目标；调小则停得更远。

**能同时用键盘和 brain 吗？**
现在不能安全地同时跑：`sim.yml` 的键盘、`reactive.yml` 的 brain，以及 `autod.yml` 的 autod
都会写 intent，同时运行是 last-writer-wins。先停掉一张图；下一步会加 intent arbiter。

**`scripts/dora-sim autod` 里鸭子只张望不走？**
看日志有没有 `depth frames arriving — obstacle avoidance on`。没有 ToF 时 autod 只转向、
不前进（上游规则）。还要确认 `robot.state.policy` 是 `walk`/`stand`；如果机器人是坐着的、
`held` 或刚被别的客户端禁用，autod 会等。默认场景的 twin 由 `scripts/dora-sim` 站起来；
真机需要先 Start 或 `robotctl robot enable`。另外手柄接管还没接，跑 autod 时不要同时用手柄。

**`DUCK_SIM_SCENE=apartment` 起不来？**
当前 macOS + microduck_rl 的 `apartment` 场景会让 MuJoCo `body_server` segfault；用默认场景即可。
`DUCK_SIM_SCENE=ball` 里的球会被站起来的动作踢走，不适合做稳定的 perception 目标；演示用
`scene_target.xml` 的固定橙色球。

## 关于 Cargo.lock

根目录的 `Cargo.lock` 是**整个 workspace** 的锁文件，必须提交。子模块 `vendor/microduck` 有它自己的 `Cargo.lock`，属于上游仓库，本仓库不使用也不重复提交。

`crates/*` 通过 path 依赖引用子模块中的 crate 时，根 `Cargo.lock` 只会记录这些 crate 及其依赖的版本，不会把子模块源码带进本仓库。
