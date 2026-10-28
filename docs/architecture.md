# 键盘控制、上游 Microduck 与 dora 的插入点

> 本文回答三个问题：
>
> 1. 当前键盘控制是怎么实现的，为什么有这么多终端细节；
> 2. 上游 Microduck 原来是怎么设计的，哪些约束不能动；
> 3. dora 可以插在哪里、为什么插在那里有价值，以及它不解决什么。
>
> 适用版本：本仓库 `crates/*`、`dataflows/sim.yml`、`scripts/dora-sim`；上游 `vendor/microduck`；
> dora 1.1.0。代码变化后，本文相关段落要一起更新。
>
> 调用关系图见 [`call-graph.md`](call-graph.md)。

## 0. 摘要

- 键盘节点解决的是两个现实问题：dora 给节点的 stdin 是 null，终端又没有 key-up。它读
  `/dev/tty`、自己进 raw mode、用按键重复判断“按住”，并且每个轴独立计时。
- 上游 Microduck 的强项是**反射层**：`robotd` 独占电机、50 Hz 控制环、安全、策略；七个
  daemon 用 JSON-RPC/NDJSON over Unix socket 通信，systemd 管生命周期。
- 上游缺的是**反射之上的组合层**：新行为要么塞进 `robotd` 的 tick，要么写一个 ad-hoc
  socket client；感知没有通用数据面；自主大脑没有移植。
- dora 插在 `robotd` 之上：节点图负责行为组合，Arrow + 共享内存负责大帧数据面，
  coordinator 负责生命周期。`robotd` 仍然是唯一电机权威，dora 不碰安全，也不进恢复路径。
- 本仓库已经接通控制链、状态链、相机数据面和最小行为闭环：`keyboard → robotd-wrapper →
  robotd`、`robotd → robotd-state → joint_state / odom / imu`，以及
  `mediad → camera → image（SHM）→ perception → target → brain → cmd_vel → wrapper`。
  `perception` 是最笨的颜色检测，`brain` 是反应式行为 v0；另外已经把上游 `autod` 的
  12 状态行为机（energy/mood、novelty grid、ToF 避障）移植成 `dora-autod`。NPU 检测器、
  上游故意没带的四个状态、手柄接管和音频输入仍然没有。

## 1. 当前键盘控制

### 1.1 数据流位置

```
dora/timer/millis/50 ──tick──▶ keyboard ──cmd_vel──▶ robotd-wrapper ──robot.move──▶ robotd
```

- `keyboard` 每 50 ms 发布一次 `cmd_vel = [vx, vy, vyaw]`；没有按键时也发布 `[0, 0, 0]`。
- `robotd-wrapper` 把 `cmd_vel` 转成 JSON-RPC notification：

  ```json
  {"jsonrpc":"2.0","method":"robot.move","params":{"vx":0.3,"vy":0.0,"vyaw":0.0}}
  ```

  没有 `id`，不需要回复。`ROBOTD_SOCKET` 未设置时，wrapper 只在命令变化时打印日志。
- 连接断了会重连一次，覆盖 `robotd` 重启后旧连接失效的场景。
- `joint_state` / `odom` / `imu` 来自 `dora-robotd-state` 的 `robot.subscribe`；不再是
  `mujoco-bridge` 的占位环，见 §4。

### 1.2 为什么读 `/dev/tty`

- dora 给每个节点的 stdin 是 null，读 stdin 拿不到按键。
- `/dev/tty` 是进程的控制终端；只要数据流是从终端启动的，节点就能打开它。
- 没有控制终端（CI、agent shell、systemd）时，节点打印 `keyboard disabled`，继续发布零，
  不让数据流失败。
- `scripts/dora-sim` 在启动 twin 前先 `stty -echo -icanon min 1 time 0`，避免等待的十几秒里
  按键被回显成 `^[[A`；退出 trap 恢复终端。

### 1.3 终端 raw mode

`RawMode::enable` 的关键步骤：

- `tcgetattr` 保存原 termios，退出时恢复。
- 忽略 `SIGTTOU` / `SIGTTIN`：dora 可能把节点放进后台进程组，第一次 termios 写或 tty 读
  会被这两个信号停掉；忽略后节点能继续跑，后台读失败则 reader 退出。
- `cfmakeraw` 关掉行缓冲、回显、`ISIG` 等，按键按字节到达。
- 但重新打开 `OPOST | ONLCR`：dora 自己的日志和这个节点共用一个终端，关掉 `ONLCR` 会把
  每个换行变成阶梯。
- `tcflush(TCIFLUSH)` 丢掉启动前积压的按键，避免“启动时按的键”在 raw mode 生效后被回放。
- 退出时 `Drop`：先 `tcflush`（避免按住键的字符回到 shell），再 `tcsetattr` 恢复原状态。

### 1.4 前台进程组

dora 给每个节点独立进程组，节点通常是后台组；后台读 tty 会 `EIO`。

- `claim_terminal()` 记录 `tcgetpgrp` 的原前台组，然后 `tcsetpgrp` 把自己设为前台。
- 退出时在 `Drop` 里把前台还回去。
- 这样 Ctrl-C 才能再次到达 dora；否则终端的前台组是 dora/launcher，键盘节点收不到按键。
- 日志里 `pgrp X, foreground Y` 和 `took the terminal's foreground group` 用来确认这一步。

### 1.5 读取与转义序列

- `spawn_reader` 在独立线程里每次读 1 字节，写入 `Arc<Mutex<Keys>>`；第一次收到键打印
  `first key received`，用来确认输入通路。
- 方向键、功能键、Option+键都以 `ESC` 开头。如果只把 `ESC [` 后面的第一个字节当命令，
  `ESC [ D` 的 `D` 会被误读成“右平移”。
- 解析器是一个小状态机 `Escape::{Start, Csi, Ss3}`：
  - `ESC [`：一直消费到 `0x40..=0x7e` 的终止字节，覆盖 `ESC [ 1 ; 5 D`、`ESC [ 200 ~`
    （bracketed paste）等；
  - `ESC O`：再吞一个字节，覆盖 application cursor mode 的 `ESC O D`；
  - `ESC` + 字母：Option+键，吞掉；
  - 单独的 `ESC`：100 ms 内没有后续就放弃等待，避免把下一个按键也吃掉。
- 方向键和空格不映射，留给 MuJoCo viewer 做单步/暂停。

### 1.6 键位与速度

| 键 | 命令 | 说明 |
|---|---|---|
| W / S | vx = +0.30 / −0.40 m/s | 前进 / 后退 |
| A / D | vy = +0.50 / −0.50 m/s | 左 / 右平移 |
| Q / E | vyaw = +1.50 / −1.50 rad/s | 左 / 右转 |
| X | 全零 | 立即停止，并清空三个轴的 hold |
| Ctrl-C | 退出 | 恢复终端，停止数据流 |

数值来自实测死区，不是“温和值”：

- 前进：约 0.25 m/s 以下不走；0.20 三秒只移动 1 mm，所以取 0.30。
- 后退：−0.30 完全不动，−0.35 才稳定迈步；取 −0.40 留余量。
- 平移：0.15 / 0.30 几乎只有毫米级移动；取 0.50。
- 转向：0.5 / 1.0 rad/s 三秒只转 0.07 / 0.14 rad；1.5 约 0.73–0.96 rad/s；2.5 约
  1.48 rad/s。取 1.50。

### 1.7 “按住才动”：用按键重复代替 key-up

终端不上报 key-up，所以“按住”只能从“终端还在重复这个键”推断。

每个轴一个 `Hold`：

```rust
struct Hold {
    key: Option<u8>,      // 当前按住的键（小写）
    last_at: Option<Instant>,
    repeating: bool,      // 本次按下是否已经收到过重复
}
```

- 首次按下：`HOLD_START = 800 ms`，覆盖操作系统的首次重复延迟。
- 一旦收到同一个键的重复：切到 `HOLD_REPEAT = 250 ms`，松开后很快停。
- `press(key)`：同一个键 → `repeating = true`；不同的键 → 新的一次按下，重新拿 800 ms。
- `held()`：超时 → 整个 `Hold` 重置，返回 false；`command()` 把该轴清零。
- **三个轴各自计时**。如果共用一个计时器，按住 A 的重复会续上 W 的计时，松开 W 后 `vx`
  会一直锁住（鸭子边走边平移）。
- 超时后 `Hold` 被重置，所以以后再次按同一个键算新按下，重新拿长窗口。
- 和弦：W + Q 各自按自己的重复计时，两个轴都能保持。

### 1.8 Ctrl-C 与退出

- raw mode 下 `ISIG` 关闭，Ctrl-C 是字节 `0x03`，不会产生 SIGINT。
- `quit = true` → `command()` 返回全零 → 主循环 break。
- 退出前补发一次 `[0, 0, 0]`，不必等 `robotd` 的 deadman 超时。
- 恢复终端；如果是 Ctrl-C，向记录的前台进程组发 SIGINT，让 `dora run` 和 launcher 一起停。
- **这个语义只在 `dora run`（前台）下成立。** 用 `dora up` + `dora start --detach` 时，
  终端前台组是 shell/CLI，不是 dora；键盘节点把 SIGINT 发给它不会停数据流。托管模式请用
  `dora stop <name>`，不要依赖 Ctrl-C。

### 1.9 已知边界

- 没有 tty：警告 + 持续发零。
- 终端被 MuJoCo viewer 抢走：`first key received` 不出现，点一下终端再试。
- 后台进程组：自动抢前台；抢不到打印错误，按键可能收不到。
- reader 线程出错（EOF/EIO）：打印 `tty read stopped`，主循环继续发零。
- 50 ms tick 与 10 ms 的 bridge tick 独立，键盘不参与仿真时钟。

### 1.10 测试

`cargo test -p dora-keyboard` 覆盖：

- WASD/QE 映射、X 停止；
- 方向键序列、按住方向键连发、参数化序列、SS3、bracketed paste、Option+键；
- 单独 Esc 不吞下一个键；
- 首次按下 800 ms 窗口、释放后 250 ms 停止；
- 轴间不串扰、重按算新按下、和弦；
- Ctrl-C 退出并输出零。

这些都是纯解析/计时逻辑，不需要真实终端。

## 2. 上游 Microduck

### 2.1 七个 daemon

上游是 **Rust、无框架、一个 workspace**。七个 daemon 由 systemd 管，跑在 RK3566 上：

| 服务 | 拥有 | 监听 | 备注 |
|---|---|---|---|
| `robotd` | 电机控制、运动学、里程计、步态策略、传感器循环、安全 | `/run/robotd.sock` | 唯一碰电机的进程；50 Hz |
| `configd` | wifi、机器人身份/名字、配对 PIN、手柄绑定、重启 | `/run/configd.sock` | `robotd` 死了也要活 |
| `updaterd` | 发布校验、安装、原子切换、健康门、回滚 | `/run/updaterd.sock` | 恢复路径 |
| `btd` | BLE GATT，传输适配器 | BLE GATT service | 不拥有状态；`robotd` 死了也要活 |
| `padd` / `netpadd` | 手柄 → intents | `/run/padd/pad.sock` / UDP | 普通 intent client |
| `mediad` | 相机/麦克风、编码、WebRTC、`media.frame` | `:8080` / `:8443` / `/run/mediad/media.sock` | 重；崩溃不能带走 `robotd` |
| `tofd` | 头部 8×8 ToF | `/run/tofd/tof.sock` | 只发布，不读别人 |

关键不变量：

- `robotd` 是唯一能碰电机的进程；客户端只发 **intent**，安全层决定能不能执行。
- `btd`、`configd`、`updaterd` 必须在 `robotd` 死掉时仍然工作——它们是恢复路径。
- 每个状态只有一个写者；其他服务只读或订阅。
- 控制环永远不阻塞在另一个服务上；跨服务读取都是 last-value-wins 缓存。

### 2.2 robotd：反射层

- 50 Hz 循环，一条 UART 上的 15 个舵机 + IMU。
- 加载 ONNX 策略（walk/stand/sitstand/...），根据 intent 选择。
- 安全：关节限位、跌倒 → 失力、intent deadman、热/电池限制。
- `robot.health` 来自控制环是否满足 deadline 和策略状态，不是一个“进程还活着”的探针。
- 客户端 API（节选）：
  - `robot.move {vx, vy, vyaw}`：twist intent，notification，无回复；
  - `robot.head`、`robot.look`、`robot.pose`、`robot.mouth`、`robot.sound`；
  - `robot.do`、`robot.enable`、`robot.relax`、`robot.init`、`robot.stop`；
  - `robot.subscribe`：把连接变成 `robot.state` 流，默认每个 tick（50 Hz），可传 `hz` 降频。
- `robot.state`（节选字段）：`t`、`move`、`head`、`policy`、`safety`、`loop`、`joints`、
  `targets`、`velocities`、`currents_ma`、`odom`、`imu`、`frames`、`skeleton`。
  这些字段对行为层足够：朝向、IMU、关节、安全状态、相机/ToF 在躯干坐标系的位姿。

### 2.3 IPC：控制面与数据面

上游把两类流量分得很清楚：

| | 控制面 | 数据面 |
|---|---|---|
| 内容 | 命令、配置、状态、感知事件 | 视频/音频帧 |
| 大小/频率 | 几十字节，≤100 Hz | 640×480 RGB @30fps ≈ 27 MB/s |
| 机制 | Unix socket RPC | **never crosses a socket** |

- 控制面：JSON-RPC 2.0，一行一个对象（NDJSON），每个服务一个 Unix socket；所有调用都有
  超时；没有 broker。选 Unix socket 而不是 localhost 端口，是为了文件权限、`SO_PEERCRED`
  和“不会因为绑错地址把固件更新暴露到网络”。
- 数据面：帧不经过 socket。如果帧必须跨进程，最后手段是共享内存（shm/dmabuf ring buffer），
  socket 只传“第 N 帧在 offset …”这样的元数据。
- 现成的大帧入口：`media.frame`，在 `/run/mediad/media.sock` 上。一次请求返回 JSON 头 +
  原始 UYVY 字节（默认 720p30 约 1.8 MiB；仿真的 360p30 约 460 KB）。它是**按需拉取**：
  请求登记后等下一帧，不是缓存最后一帧；而且**一个连接只返回一帧**，客户端下一条请求要
  重新连接。这是给本机 recorder/perception 进程用的，不是 WebRTC 控制通道。
- `tofd` 的 8×8 深度帧在 `/run/tofd/tof.sock`，15 Hz，独立订阅。

### 2.4 仿真

- `robotd --sim host:port` 用 `duck_control::sim::RemoteIo` 替代舵机总线：每个 tick 的关节
  位置/速度/IMU 从 MuJoCo 进程经 TCP 进来，策略的目标值回去。控制环、策略、安全、运动学、
  里程计、IPC 全是真机代码，不区分仿真。
- `tofd --sim` 从同一个仿真器拿 8×8 深度；`mediad --sim-camera` 拿渲染出来的头部相机图像。
- `scripts/duck-sim` 负责 body、daemon、策略、socket、健康检查和站立。
- 相机是 opt-in：`DUCK_SIM_CAMERAS=a`；相机帧端口默认 7901，每个相机 +1。macOS 需要
  GStreamer；仿真默认 `360p30`，按 90° 安装角转正后是竖屏（360×640）。当前 macOS 上
  `apartment` 场景会让 `body_server` segfault，默认场景是安全的。
- 仿真 socket 在状态目录下：`duck-a.sock`（robotd）、`duck-a-tof.sock`、`duck-a-frame.sock`
  （`media.frame`）。真机上分别是 `/run/robotd.sock`、`/run/tofd/tof.sock`、
  `/run/mediad/media.sock`。
- 它能测：daemon 和客户端的 IPC、`robotctl`、console、updater、chorale、策略站立/行走。
- 它不能测：驱动（Dynamixel 总线、BLE 射频、相机 ISP、NPU、硬件编码器）。

### 2.5 上游自己承认的缺口

- **行为栈/大脑没移植。** 原型里的 16 状态机（Chill、Wander、Zoomies、Startle、Dance、
  Nap、BallPlay、Petted……）在 daemon 里没有对应实现。输入其实都齐了：声音事件、ToF 深度、
  障碍点、语音标签、BLE 邻居、共享节拍、RSSI。上游文档把它称为 “the biggest untracked gap”。
- **新行为只能挂在 `robotd` 的 tick 上。** `sound.rs`、`theremin.rs`、`chorale.rs`、
  `pet-detect/`、`vetbook.rs` 都是控制进程里的子系统；上游自己的描述是 “each hangs off the
  tick, none may block it”。这不是长期架构，是行为往脊髓里塞。
- **感知没有通用数据面。** 相机帧走 `media.frame` 或 WebRTC；NPU 鸭子检测器还缺帧路径；
  没有一个“相机 → 检测 → 融合 → 大脑”的图。
- **生命周期和可观测是按 daemon 做的。** 每个行为是 systemd unit 或 `robotd` 里的模式；
  没有图级别的 start/stop/restart、热替换、统一 logs/topics/record/replay。Roadmap 的
  “missing” 一栏写的是：app、SDK、policy channel、autonomous brain。

这些缺口就是 dora 的插入点。

### 2.6 不可突破的约束

- `robotd` 是唯一电机权威。任何图都不能绕过它直接写舵机。
- 控制环不能阻塞在另一个服务上（所以 `robotd` 用 last-value-wins 缓存，不做同步 RPC）。
- `configd` / `updaterd` / `btd` 不依赖 `robotd`、不依赖 ML/媒体栈；dora 也不能进这条恢复路径。
- 每个状态只有一个写者。
- 帧不跨 socket；必须跨进程就共享内存。

## 3. dora 的插入点与优点

### 3.1 位置：`robotd` 之上

```
perception ──SHM──▶ fusion ──SHM──▶ brain ──intent──▶ robotd
    ▲                                      │
    └──────── robot.state / media.frame ───┘
```

- `robotd` 和它的 daemon 保持原样：反射、安全、50 Hz、恢复路径。
- dora 只做三件事：**图**（行为组合）、**数据面**（大帧）、**生命周期**（启停/重启/观测）。
- 这个位置和 ROS 在真机上的位置一样：ROS 也不会替代 `robotd`，真机接口仍然是
  `robot.move`；区别只是 dora 更轻、YAML 即图、Rust 节点可直接跑。

### 3.2 插入点一：行为图

问题：新行为没有地方住，只能塞进 `robotd` 的 tick 或写一个 ad-hoc socket client。

dora 提供：

- 每个行为一个节点/进程，输入输出显式声明；
- 可单独重启、替换、增删（`dora node restart/stop/add/remove/replace`）；
- 多速率：每个节点自己的 timer（50 Hz 控制反馈、10 Hz 遥测、事件驱动的语音）；
- 多语言：Rust 节点和 Python 节点可以挂在同一张图上。

优点：

- 行为崩了不会带走电机控制；`robotd` 的 deadman 仍然兜底。
- 改行为不需要发 daemon release，也不需要重编 `robotd`。
- 行为可以单独测试、单独 record/replay。

### 3.3 插入点二：感知数据面与零拷贝

问题：上游原则是“帧不跨 socket”，但现成的路径要么是按需 `media.frame`，要么是 WebRTC；
缺少一个通用的 perception → fusion → brain 的数据面。本仓库用 `camera → perception →
brain` 补上了最小的一段（见 §4.5），但还不是通用融合层。

dora 1.1.0 的实现：

- 单条消息 ≥ **4096 字节**（`ZERO_COPY_THRESHOLD`，可用 `DORA_ZERO_COPY_THRESHOLD` 覆盖）
  且同机、SHM 池分配成功时，走 zenoh 共享内存。
- 每个节点默认 SHM 池 **8 MB**（`shared_memory_pool_size` 或 `DORA_NODE_SHM_POOL_SIZE` 可调）。
- 单条 IPC 上限 256 MB；Arrow buffer 64 字节对齐，sample 128 字节对齐。

拷贝次数：

| 发送方式 | dora 侧 payload 拷贝 | 说明 |
|---|---|---|
| `send_output_raw(id, params, len, \|buf\| ...)` | **0** | IPC 头预写进 SHM sample，调用方直接写数据区 |
| `send_output(id, params, arrow_array)` | **1** | IPC 编码进 SHM sample；之后 `put` 移动 buffer，不再拷贝 |
| IPC fallback | **2** | 复杂类型会先产生 payload 大小的中间 `Vec` |
| SHM 池满 / 无 provider / 跨机 | 额外拷贝 | 回退到堆或 daemon TCP |

接收端 `decode_arrow_ipc_zero_copy` 让 Arrow buffer 直接 alias 收到的 SHM 内存，**0 拷贝**；
只有转成 `Vec<T>` 时才 copy。

优点：

- 相机帧成为节点的一个输出，不需要 JSON/base64/socket；和上游“帧不跨 socket”的原则一致。
- 感知节点和大脑节点在同一台机器上时，1.8 MiB 的帧不再每跳复制一次。
- 大帧仍然受 `robotd` 的 intent 边界保护：只有小 intent 能到电机。

注意：

- 跨机不成立：SHM 只在本机；远端是网络传输。
- 池子要按“同时在途的帧数”配，不是按单帧大小；消费者持有 buffer 会 pin 住 SHM 段。
- 池满时 dora 不阻塞，退回堆/TCP，有拷贝；大消息不会走 zenoh 堆路径（会分片丢消息）。
- `dora topic echo`、`dora record`、WS 的 `TopicSubscribe` 是调试/落盘路径，会拷贝，不要当
  生产数据面。

### 3.4 插入点三：生命周期与管理

问题：上游每个行为是 systemd unit 或 `robotd` 里的模式；没有图级别的生命周期。

dora 提供：

- coordinator 模式：`dora up` 后 `dora start/stop/restart/list/logs/inspect/top`；
- 节点级：`dora node list/info/restart/stop/add/remove/replace/connect/disconnect`；
- 运行时参数：`dora param list/get/set/delete`；
- 消息调试：`dora topic list/echo/hz/info/pub`；
- 录制/回放/追踪：`dora record/replay/trace`；
- YAML 里的自愈：`restart_policy: never|on-failure|always`、`max_restarts`、
  `restart_delay`（指数退避）、`max_restart_delay`、`restart_window`、`startup_timeout`、
  `health_check_timeout`。

优点：

- 感知节点可以崩了自动回来；换一个检测器不需要重启整张图。
- 图级别的日志、节点状态、资源占用、topic 频率都能看到。
- 仿真和真机用同一套启停语义。

注意：

- `dora run` 是隔离 daemon，没有 coordinator；要用管理能力必须 `dora up`。
- 托管模式下键盘节点的 Ctrl-C 语义会变（见 §1.8），用 `dora stop`。
- manager 放在图外（systemd/脚本/外部进程），不要做成 dora 节点管理自己所在的图。

### 3.5 插入点四：仿真到真机

上游的 `duck-sim` 已经跑真 `robotd` + 真策略 + MuJoCo body；dora 图看到的
`robot.state` / `media.frame` 契约在仿真和真机上一致。

- 仿真：`ROBOTD_SOCKET=target/duck-sim/state/duck-a.sock`，相机 socket
  `target/duck-sim/state/duck-a-frame.sock`。
- 真机：`ROBOTD_SOCKET=/run/robotd.sock`（或 ssh 转发），相机 `/run/mediad/media.sock`。
- 同一张图，只换 source/sink 的地址；行为节点不需要 `if sim` 分支。

注意：仿真不建模驱动（Dynamixel、BLE、ISP、NPU、编码器）；能在仿真里跑通的行为，
到了真机仍然要验证传感器和驱动。

### 3.6 插入点五：多语言与第三方行为

- dora 节点可以是 Rust、Python；第三方模型/行为可以作为一个节点挂上图，而不是新写一个
  daemon 或 socket client。
- 动态拓扑允许运行时 `add/remove/connect/disconnect`，不需要改 YAML 重跑。
- 节点只拿到 intent API，不拿到电机总线；安全边界仍然是 `robotd`。

注意：板子是 RK3566 + 1 GB RAM，dora + zenoh + coordinator 上板要实测；默认部署是 dora 在
开发机/主机上，通过 socket 连真机 `robotd`。

### 3.7 边界：dora 不解决什么

- 50 Hz 控制环、策略推理、电机安全：留在 `robotd`。
- intent 仲裁和 deadman：`robotd` 已经有，图不要绕过。
- 大脑算法：16 状态机、mood、novelty grid、startle——dora 只给它们一个家。
- 恢复路径：`configd`/`updaterd`/`btd` 必须继续独立于 `robotd` 和 dora。
- 跨机零拷贝：不成立；跨机走网络。
- dora 的 coordinator 是一个额外的 broker。上游架构对核心 daemon 明确避免 broker；所以
  dora 必须待在 `robotd` 之上、恢复路径之外。dora 挂了，`robotd` 的 deadman 会让鸭子停下，
  恢复 daemon 照常工作——这是可以接受的失败模式。

### 3.8 对照表

| 问题 | 上游现状 | dora 插入点 | 留在外面 |
|---|---|---|---|
| 新行为没地方住 | `robotd` 的 tick 或 ad-hoc client | 行为节点，独立进程/重启 | 安全、50 Hz |
| 感知数据面 | `media.frame` 按需拉取 / WebRTC | camera 节点 + SHM 零拷贝 | 相机采集、ISP |
| 自主大脑 | 未移植 | brain 节点消费 perception topics | 算法、模型、数据 |
| 生命周期 | 每个 daemon 一个 systemd unit | coordinator + restart policy | 恢复 daemon |
| 仿真/真机 | 同一套 daemon | 同一张图，换 socket | 驱动 |
| 第三方行为 | 新写 client | 多语言节点 + 动态拓扑 | intent 仲裁 |

## 4. 本仓库现状与目标切片

### 4.1 现状

- `dora-keyboard`：真实可用，细节见 §1。
- `dora-robotd-wrapper`：纯 intent sink，`cmd_vel` → `robot.move` 真实可用。
- `dora-robotd-state`：通过 `robot.subscribe` 订阅 `robot.state`，发布 `joint_state`（15 f32）、
  `odom`（`[x, y, z, yaw]`）和 `imu`（`[qw, qx, qy, qz, gx, gy, gz]`）；robotd 重启后自动
  重连并重新订阅。
- `dora-camera`：通过 `media.frame` 拉取原始 UYVY 帧，把 IPC 头和二进制尾部直接写进 sample，
  发布 `image` 和 `width`/`height`/`format`/`rotate` 元数据；`shared_memory_pool_size: 64MB`。
  已在真实 sim 相机上验证（640×360、30 fps、`zenoh direct path` + SHM），也验证过假的
  `media.frame` 服务；`scripts/dora-sim camera` 直接看画面，`FRAME_DUMP_DIR` 导出帧。
- `dora-perception`：消费 `image`，发布 `target = [visible, bearing, area]`（`f32[3]`）。
  它不是模型，是 UYVY 色度阈值 + 质心：橙色/红色目标的 `V` 高、`U` 低，地板/天空接近
  `U=V=128`。不旋转整帧，而是用 `rotate` 把原始坐标映射到转正后的水平轴（90° 时
  `x_upright = height - 1 - y_raw`），所以 460 KB 的帧只读一遍。阈值由 `PERCEPTION_*`
  覆盖。
- `dora-brain`：消费 `target` 和 `robotd-state` 的 `imu`，每 50 ms 发布 `cmd_vel`。反应式
  v0：`|bearing| > 0.45` 转向，`< 0.30` 前进，中间保持；`area ≥ 0.06` 停下并锁定，
  `< 0.02` 才解除；目标丢失 300 ms 后向右搜索，最多 8 s；倾角 > 45° 或 IMU 超过 500 ms
  没更新就发零；默认没有 IMU 拒绝驱动。转向用策略真正能执行的值（1.5 rad/s），不做
  “温和”比例控制，否则鸭子只晃不走。细节见 §4.5。
- `dora-frame-sink`：`sim.yml` 里的调试消费者，原地读 `Uint8Array` 并打印帧信息，同时让
  camera 的输出有订阅者，从而真正走 SHM 路径。设置 `FRAME_DUMP_DIR` 可以把收到的原始
  UYVY 帧和 `.json` 元数据写到磁盘，供 ffmpeg 离线查看；`FRAME_DUMP_EVERY`（默认 30）和
  `FRAME_DUMP_LIMIT`（默认 10，0 = 不限）控制频率和上限。它和 `perception` 属于两张图。
- `dora-mujoco-bridge`：仍是确定性合成步态的 stub，已从数据流移除；上游 twin 的 MuJoCo
  才是仿真来源。
- `dataflows/sim.yml`：五个节点，遥操作：键盘控制链、状态链、相机调试消费者；假反馈环已删除。
- `dataflows/reactive.yml`：五个节点，自主闭环：`camera → perception → brain → wrapper`，
  加上 `robotd-state → brain` 的 IMU；没有键盘。
- `dora-tof`：连接 `TOF_SOCKET`，订阅 `tof.stream`，把每个 `TofFrame` 序列化成 JSON
  发布到 dora `tof` topic；socket 长连接，错误重连，读超时让 dora stop 能打断。
- `dora-autod`：上游 `origin/autonomous-mode` 的 `autod` 移植（提交 `df9a750`，Apache-2.0，
  `crates/dora-autod/LICENSE`）。`brain.rs` 是 12 状态行为机 + energy/mood + novelty grid，
  `obstacle.rs` 是 ToF 避障；`main.rs` 是纯 dora 适配层：消费 `robot_state` 和 `tof`，
  按 `t_ns` 配对，发布 `cmd_vel/head/mouth/body/sound/skill`。它不再开 socket。
- `dataflows/autod.yml`：四个节点，
  `robotd-state → autod ← tof`、`autod → wrapper`。只有 `robotd-state`/`tof` 读 socket，
  `wrapper` 写 intent。
- `scenes/scene_target.xml`：演示世界，标准鸭子 + 前方 0.8 m 一个固定橙色球。球没有碰撞体、
  没有 freejoint，不会被站起来的动作踢走。脚本把它复制到仿真器的 robot 目录，因为模型
  用相对的 `meshdir="assets"`。
- `scripts/dora-sim`：负责 twin 生命周期、tty echo、`ROBOTD_SOCKET` /
  `MEDIA_FRAME_SOCKET`、`dora run`；`camera` 子命令用 `scripts/duck-camera-view`
  （Python/OpenCV）打开 `media.frame` 窗口；`reactive` 子命令复制 target 场景、启动
  `reactive.yml`。

也就是说：**控制链、状态链、相机数据面和最小行为闭环现在都是真的。** 缺的是 arbiter、
NPU 检测器和真正的 16 状态大脑。

### 4.2 目标图

```
robotd-state ──imu──┐
                    ├──▶ brain ──cmd_vel──▶ wrapper ──robot.move──▶ robotd
camera ──image──▶ perception ──target──┘
```

这张图现在就是 `dataflows/reactive.yml` 的实际内容；`robotd-state` 还会发布 `joint_state`
和 `odom`，但这一版没有消费者。`camera → frame-sink` 仍然在 `sim.yml` 里，作为遥操作时的
帧路径调试分支；两张图不要同时跑，因为 `keyboard` 和 `brain` 都会写 `cmd_vel`，而目前还
没有 arbiter。

### 4.3 分步

1. **真状态（已完成）**：`dora-robotd-state` 订阅 `robot.subscribe`，发布 `joint_state`
   （15 f32）、`odom`、`imu`；`sim.yml` 的 `joint_state` 来源已换成它，`mujoco-bridge` 已从
   图中移除。不要在 dora 节点里重新实现 MuJoCo。
2. **相机 + SHM（已完成）**：`dora-camera` 按 30 Hz 调 `media.frame`，用
   `allocate_data_sample` + `encode_uint8_ipc_header` + `send_output_sample` 把二进制尾部
   直接读进 sample；节点配 `shared_memory_pool_size: 64MB`。已在本机装好 GStreamer 并跑通
   模拟相机：640×360 UYVY（`360p30`），约 460 KB/帧，日志确认
   `output image: … B -> zenoh direct path` 和 `entering zenoh put of an SHM buffer`。
   `media.frame` 每个连接只返回一帧，所以节点每个 tick 重连一次（只在首连/失败后打印）；
   协议本身也已用假 `media.frame` 服务单独验证。
3. **最小 perception + brain（已完成）**：颜色阈值检测，输出 `target =
   [visible, bearing, area]`；`brain` 做反应式控制（转向/前进/到达/搜索）。`robotd-wrapper`
   仍是唯一 intent sink。`reactive.yml` 和 `scenes/scene_target.xml` 一起提供可复现的实验；
   细节见 §4.5。实测在 target 场景上找到目标、前进约 0.3 m、在约 0.4–0.5 m 处停下并保持。
4. **上游 autod 移植（已完成）**：把 `origin/autonomous-mode` 的 `autod/src/brain.rs` 和
   `obstacle.rs` 原样带进 `dora-autod`，配一个 dora 适配层；12 个状态、energy/mood、
   novelty grid、ToF gate 都在。已在 sim 上跑出 LookAround/Chill/Dance/Zoomies/Wander 和
   位移。细节见 §4.6。
5. **autod 的 dora 化（已完成）**：`robotd-state` 增加 `robot_state`，新增 `dora-tof`，
   `dora-autod` 改成消费 topic，wrapper 扩展成 `cmd_vel/head/mouth/body/sound/skill` 的
   唯一 intent sink。仍有缺口：手柄接管（`move.source`）、麦克风（`heard`/`petting`）和
   v38 `head alpha`。
6. **intent arbiter**：键盘、reactive brain、autod 都写 intent，现在不能安全共存。加一个
   arbiter 节点（manual 优先/可切换，带超时回退）或给 wrapper 增加第二个输入，再决定
   哪个是默认写者。这一层应该在合并 `sim.yml`/`reactive.yml`/`autod.yml` 之前完成。
7. **管理**：YAML 加 `restart_policy: on-failure`、`startup_timeout`、
   `health_check_timeout`；切片稳定后再加 `dora up` + `scripts/dora-sim managed`。
8. **之后**：上游故意没带的 BallPlay/Held/Startle/Preen、NPU 鸭子检测器、真机部署、
   板载资源测量。

### 4.4 验收

- `joint_state` / `odom` 来自真实 `robot.state`，不再是合成步态；smoke test 的首帧已是站立时
  的真实值（已完成）。
- 相机帧（>4 KB）在日志里走 `zenoh direct path`，不是 `daemon path`；已在真实 sim 相机
  （640×360）和假 `media.frame` 服务上验证（已完成）。GStreamer 已安装；`apartment` 场景
  当前会让 `body_server` segfault，默认场景可用。
- 目标进入视野时鸭子转向/前进，靠近后停下并保持（已完成：`scripts/dora-sim reactive
  --stop-after 12`，日志出现 `visible=true` → `mode=turn` → `mode=forward` →
  `mode=arrived`，odom x 约 0.02 → 0.32）。
- 目标丢失 300 ms 后进入 `mode=search`，8 s 后停止；IMU 倾角超过 45°、IMU 超过 500 ms
  未更新或没有 IMU 时发零（单元测试覆盖；真机还需要实测策略对 1.5 rad/s 的响应）。
- 上游 autod 移植（四节点版）已用 `scripts/dora-sim autod --stop-after 20` 验证：
  `dora-tof: connected`/frame 日志到达，状态出现 Ruffle/LookAround/Chill/Wander，Wander 后
  odom x 约 0.07 → 0.29 m；wrapper 连上 robotd，没有类型错误或协议拒绝。`--stop-after`
  干净退出（已完成）。
- brain 被杀掉时，`robotd` 的 deadman 让鸭子停下；如果加了 restart policy，brain 能回来。
- `dora node list` 显示所有节点健康。

当前未覆盖：键盘和 brain 同时运行（没有 arbiter）、NPU/模型检测、目标身份/多目标、
mood/记忆/探索、真机部署。

### 4.5 最小感知与行为（reactive v0）

#### 消息契约

```
camera.image (UInt8, SHM)
  metadata: width, height, format="UYVY", rotate, bytes, captured_at_unix_us

perception.target (f32[3])
  [visible (0/1), bearing (-1 左 .. +1 右), area (占整帧比例)]

brain.cmd_vel (f32[3])
  [vx, vy, vyaw]，和键盘节点同一个契约，进同一个 wrapper

robotd-state.imu (f32[7])
  [qw, qx, qy, qz, gx, gy, gz]
```

`target` 用一条消息发三个字段而不是三个 topic，是为了让 `visible`、`bearing`、`area`
来自同一帧，不会被跨 topic 的调度拆开。

#### perception

- 输入格式必须是 `UYVY`；`width`/`height` 与 payload 长度不符时丢弃并限频打日志。
- 每个 4 字节宏像素是 `U, Y0, V, Y1`，两个像素共享色度。命中条件是
  `V ≥ PERCEPTION_V_MIN`（默认 150）、`U ≤ PERCEPTION_U_MAX`（120）、
  `Y ≥ PERCEPTION_Y_MIN`（50）。橙色（RGB 255,140,0）约 `U=40, V=197, Y=158`；
  地板/天空/白线接近 `U=V=128`，所以色差是信号。
- 面积小于 `PERCEPTION_MIN_AREA`（默认 0.001，即 640×360 的约 230 像素）不发布可见目标。
- **不旋转整帧。** 相机是 90° 侧装的；如果先 `transpose` 再算质心，每帧都要多一次
  460 KB 的拷贝/重排。节点只把每个命中像素的原始坐标映射到转正后的水平轴：

  ```
  rotate 0:   x_upright = x_raw
  rotate 90:  x_upright = height - 1 - y_raw      （顺时针转正）
  rotate 180: x_upright = width - 1 - x_raw
  rotate 270: x_upright = y_raw
  ```

  转正后的宽度在 90/270 时是原始 `height`。方位
  `bearing = 2 * (mean_x_upright + 0.5) / upright_width - 1`，正数在右。
- 这是**颜色检测**，不是鸭子检测器：同色的其他物体、强反光、不同灯光都会影响它。替换成
  NPU 模型时保持 `target` 契约即可，brain 不用改。

#### brain

控制律是开关式 + 滞回，不是比例控制。原因是策略死区：前进低于约 0.25 m/s、转向低于约
1.2 rad/s 时几乎不动；给“温和”的小值只会让鸭子原地晃。转向还可用，但方向由 `bearing`
的符号决定：

| 条件 | 输出 |
|---|---|
| `abs(bearing) > 0.45` | `vyaw = -sign(bearing) * 1.50`（目标在右 → 负 yaw，因为正 yaw 是左转） |
| `abs(bearing) < 0.30` | `vx = 0.30`，`vyaw = 0` |
| `0.30 ≤ abs(bearing) ≤ 0.45` | 保持上一步，避免在阈值附近抖动 |
| `area ≥ 0.06` | 全零，锁定 `arrived` |
| `arrived` 且 `area ≥ 0.02` | 继续全零 |
| `arrived` 且 `area < 0.02` | 解除锁定，重新对齐/靠近 |
| 没有可见目标超过 300 ms | 向右转 1.50 rad/s，最多 8 s；之后全零 |

宽度 0.30–0.45 是实测调出来的：0.10–0.20 的窄带理论上更准，但转向延迟让鸭子反复过冲，
在原地左右摇。离目标几十厘米时 5° 的偏差仍然在正前方。

安全：

- IMU 四元数算 roll/pitch；超过 `BRAIN_TILT_LIMIT_DEG`（45°）立即全零。
- IMU 超过 `BRAIN_IMU_TIMEOUT_MS`（500 ms）没更新，或 `BRAIN_REQUIRE_IMU=1`（默认且没有
  IMU）时全零，日志 `mode=no-imu`。这是刻意的：没有姿态就不自动走。
- 目标消息超时（300 ms）不会继续沿旧方位走，而是进入搜索，然后停。
- `robotd` 的安全层、deadman、跌倒失力照常生效；`brain` 只是另一个 intent 发送方。

调参（`BRAIN_*`）：

| 变量 | 默认 | 说明 |
|---|---|---|
| `BRAIN_FORWARD` / `BRAIN_TURN` | 0.30 / 1.50 | 前进 m/s、转向 rad/s |
| `BRAIN_ALIGN_START` / `BRAIN_ALIGN_STOP` | 0.45 / 0.30 | 进入/退出转向的方位门限 |
| `BRAIN_CLOSE_AREA` / `BRAIN_CLOSE_RELEASE` | 0.06 / 0.02 | 停下/解除停下的面积门限 |
| `BRAIN_TARGET_TIMEOUT_MS` | 300 | 目标多久没更新算丢失 |
| `BRAIN_SEARCH_MS` | 8000 | 搜索上限；0 = 丢失立即停 |
| `BRAIN_TILT_LIMIT_DEG` | 45 | 倾角保护 |
| `BRAIN_IMU_TIMEOUT_MS` | 500 | IMU 超时 |
| `BRAIN_REQUIRE_IMU` | 1 | 没有 IMU 是否拒绝驱动 |

#### 场景与启动

`scenes/scene_target.xml` 是标准 ground-contact 鸭子 + 正前方 0.8 m 一个半径 5 cm 的橙色球。
球 `contype=0 conaffinity=0` 且没有 freejoint：不会被站起来的动作踢走，也不会和鸭子碰撞。
场景没有 keyframe，所以必须 `DUCK_SIM_KEYFRAME=HOME`。

`scripts/dora-sim reactive`：

1. `cargo build` 相关节点；
2. `prepare_rl` 确保 `target/duck-sim/rl` 存在；
3. 把 `scenes/scene_target.xml` 复制到
   `target/duck-sim/rl/src/mjlab_microduck/robot/microduck/scene_target.xml`，因为 include
   的 `robot_groundcontact.xml` 用相对的 `meshdir="assets"`；
4. 以 `DUCK_SIM_SCENE=target`、`DUCK_SIM_KEYFRAME=HOME`、`DUCK_SIM_CAMERAS=a` 启动（或复用）
   twin；
5. 导出 `MEDIA_FRAME_SOCKET`、`ROBOTD_SOCKET`，运行 `dora run dataflows/reactive.yml`。

#### 实测与边界

- 相机 640×360 UYVY、30 Hz；perception 每帧读一遍，未观察到节点拖慢 50 Hz 控制环。
- 第一次看到目标时 `area` 约 0.001–0.013，方位可能在 ±0.98；brain 先转向，进入
  `forward` 后前进。靠近时 `area` 长到 0.06 左右，`mode=arrived`，odom x 从约 0.02 到
  0.32（目标在 x=0.8），随后保持不动。
- 目标丢失时会向右转搜索；8 s 没找到就停。停在约 0.4–0.5 m，是因为再近目标会掉出相机
  视野/被头部遮挡，不是距离估计。
- 目前没有 arbiter，`sim.yml` 和 `reactive.yml` 不能同时跑；也没有多目标、目标身份、
  16 状态、mood、记忆和 NPU。`brain` 是行为层 v0，不是替代上游大脑。

### 4.6 上游 autod 的移植

#### 来源与范围

- 源：`pollen-robotics/microduck` 的 `origin/autonomous-mode`，提交 `df9a750e95ac5a57f7efeb0cf158a4092167018e`。
- `crates/dora-autod/src/brain.rs`、`obstacle.rs` 是上游代码，保留 Apache-2.0
  （`crates/dora-autod/LICENSE`）；`main.rs` 是本仓库的 dora 适配层。
- 上游文档说 16 状态，但当前分支故意只带 12 个：`BallPlay`（缺球检测器）、`Held`
  （抱起步检测废弃）、`Startle`、`Preen`（效果不好）不在移植范围。不要为了凑 16 个自己
  发明四个状态。

#### 形状

`autod.yml` 现在是四个节点：

```
robotd-state ──robot_state──┐
                            ├──▶ autod ──cmd_vel/head/mouth/body/sound/skill──▶ wrapper
tof          ──tof──────────┘
```

`robotd-state` 和 `tof` 是 source（分别读 `ROBOTD_SOCKET`、`TOF_SOCKET`），`autod` 是纯
dora 节点，`wrapper` 是唯一 sink。这样 `dora topic echo/hz` 能看到 state/ToF，autod 也能
和别的节点共用数据面。`robot_state` 是紧凑 JSON（`t_ns`、policy、fallen、sitting、head、
gravity、trunk height、pose、abilities），`tof` 是序列化的 `TofFrame`；两者都按 `t_ns` 配对。
`robot_state` 超过 500 ms 没更新时，`autod` 停止发布 intent，靠 `robotd` 的 deadman 停下；
state 恢复后继续。

#### 行为

12 个状态和上游一样：Chill、LookAround、Wander、TurnInPlace、Zoomies、Stretch、Ruffle、
Dance、Sneeze、GroundPick、Nap、Petted。核心不是状态名，而是：

- **energy/mood**：走路/zoomies 消耗，chill/nap/petting 恢复；影响状态权重、速度、叫声频率。
- **novelty grid**：0.5 m 格子，记录停留时间，约 15 分钟衰减；Wander 目标朝没去过的空间，
  并避开最近被障碍挡住的方向。
- **ToF gate**：按 `t_ns` 把帧和最近的状态配对；用 IMU 重力把 beam 水平化；只信可信
  status；同一 hit 需要邻域确认；障碍记在 odometry 帧里并随鸭子移动重新表达；按距离
  减速/转向/停。没有 ToF 时只转不走。
- **Wander 的最小速度**：策略在低速下站着不动，所以 `autod` 从不请求低于 0.15 m/s 的
  `vx`，每次起步先给 0.6 s 全速。
- **声音/头部/身体**：叫声、张嘴、头部动画、Dance/Stretch/Petted 的 body pose 都通过
  `robot.head/mouth/pose/sound` 发出。

#### 当前未接

- 麦克风：`heard`/`petting` 固定为 `None`/`false`，`Petted` 不会触发，Petted 的声音反应
  也不会有。
- 手柄接管：上游靠 `robot.state.move.source == "padd"` 判断；当前 submodule 主分支 proto
  没有 `source`，所以没有让位逻辑。跑 autod 时不要同时用手柄。
- 开机自站：上游的 `robot.init`/`robot.enable` 一次-per-boot 逻辑没带；sim 由
  `scripts/dora-sim` 站，真机由 Start/robotctl 站。
- v38 的 `robot.head alpha`：主分支 `HeadParams` 没有这个字段，快速甩头被默认平滑削弱。
- socket 只留在 source/sink：`robotd-state`、`tof`、`wrapper` 仍需要 socket；`autod` 本身
  已经完全走 dora。

#### 验证

- `cargo test -p dora-autod`：20 个上游测试 + adapter 测试全过（十分钟行为不越界、Wander
  到目标、避开被挡方向、无 ToF 不前进、GroundPick 等待 busy、坐着的鸭子会被站起来、声音
  回答等）。
- 本机 `DUCK_SIM_VIEWER=0 scripts/dora-sim autod --stop-after 20`：`dora-tof: connected` 和
  frame 日志出现，`dora-autod: first state ... abilities ...` 出现，状态有
  Ruffle/LookAround/Chill/Wander，Wander 后 odom x 约 0.07 → 0.29 m；wrapper 连上 robotd，
  没有类型错误/协议拒绝。`--stop-after` 干净退出。
- 未能验证：真实手柄接管、麦克风/Petted、真机 head alpha、GroundPick/Nap 在 sim 里的
  完整技能时长（状态机测试覆盖，但 20 s 随机运行没有必然抽到）。

## 5. 参考

本仓库：

- `crates/dora-keyboard/src/main.rs`：键盘解析、raw mode、hold 计时、测试。
- `crates/dora-robotd-state/src/main.rs`：`robot.subscribe` → `joint_state` / `odom` / `imu`。
- `crates/dora-camera/src/main.rs`：`media.frame` → `image`，二进制尾部直接读进 sample。
- `crates/dora-perception/src/main.rs`：UYVY 颜色检测、旋转坐标映射、`target` 输出。
- `crates/dora-brain/src/main.rs`：反应式控制、滞回、搜索、IMU/倾角保护。
- `crates/dora-autod/src/brain.rs`、`obstacle.rs`：上游 12 状态行为机与 ToF 避障（Apache-2.0）。
- `crates/dora-tof/src/main.rs`：`tof.stream` → dora `tof`。
- `crates/dora-autod/src/main.rs`：dora 适配层、state/ToF 配对与 intent 发布。
- `crates/dora-frame-sink/src/main.rs`：调试消费者，原地读 `Uint8Array`。
- `crates/dora-robotd-wrapper/src/main.rs`：`cmd_vel/head/mouth/body/sound/skill` → `robot.move/head/mouth/pose/sound/do`。
- `crates/dora-mujoco-bridge/src/main.rs`：合成步态 stub（不在数据流中）。
- `dataflows/sim.yml`：遥操作五个节点和各自的 tick。
- `dataflows/reactive.yml`：自主闭环五个节点。
- `dataflows/autod.yml`：上游 autod 的四节点图。
- `scenes/scene_target.xml`：固定橙色目标的演示世界。
- `scripts/dora-sim`：twin 生命周期、`ROBOTD_SOCKET` / `MEDIA_FRAME_SOCKET`、target 场景。
- `scripts/duck-camera-view`：`media.frame` 相机窗口（Python/OpenCV，调试工具）。
- `README.md`、`CLAUDE.md`：命令、版本耦合、约定。

上游：

- `vendor/microduck/docs/design/architecture.md` §1（服务拆分）、§2.1–2.4（控制面/数据面、
  IPC、帧不跨 socket）。
- `vendor/microduck/docs/design/robotd-design.md` §4.5（挂在 tick 上的五个子系统）。
- `vendor/microduck/docs/robot/simulation.md`（`robotd --sim`、`tofd --sim`、
  `mediad --sim-camera`、`DUCK_SIM_CAMERAS`）。
- `vendor/microduck/docs/ideas/autonomous_behavior.md`（16 状态机、缺失的大脑）。
- `origin/autonomous-mode:autod/src/brain.rs`、`autod/src/obstacle.rs` 和
  `docs/design/autonomous.md`（上游 12 状态移植、energy/mood、novelty grid、ToF gate；
  提交 `df9a750`）。
- `vendor/microduck/kinematics/src/tof.rs`（ToF beam 反投影、Posture、Zone）。
- `vendor/microduck/docs/project/npu-bringup.md`（检测器和缺失的帧路径）。
- `vendor/microduck/duck-ipc-proto/src/lib.rs`（`robot.*` 方法、`RobotState`、`media.frame`、
  `MediaFrameHeader`、`JOINT_NAMES`）。

dora 1.1.0：

- `dora-node-api`：`ZERO_COPY_THRESHOLD`（4096）、`SampleAllocator`、`send_output_raw`、
  `send_output`、`decode_arrow_ipc_zero_copy`。
- `dora-core` descriptor：`restart_policy`、`max_restarts`、`restart_delay`、
  `max_restart_delay`、`restart_window`、`startup_timeout`、`health_check_timeout`。
- `dora-cli`：`dora up/start/stop/restart/list/logs/inspect/top/node/param/topic`。
