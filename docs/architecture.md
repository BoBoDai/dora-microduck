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
- 本仓库已经接通控制链和状态链：`keyboard → robotd-wrapper → robotd`，以及
  `robotd → robotd-state → joint_state / odom / imu`。感知和行为节点还没开始。

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
  原始 UYVY 字节（640×480 约 1.8 MiB）。它是**按需拉取**：请求登记后等下一帧，不是缓存
  最后一帧。这是给本机 recorder/perception 进程用的，不是 WebRTC 控制通道。
- `tofd` 的 8×8 深度帧在 `/run/tofd/tof.sock`，15 Hz，独立订阅。

### 2.4 仿真

- `robotd --sim host:port` 用 `duck_control::sim::RemoteIo` 替代舵机总线：每个 tick 的关节
  位置/速度/IMU 从 MuJoCo 进程经 TCP 进来，策略的目标值回去。控制环、策略、安全、运动学、
  里程计、IPC 全是真机代码，不区分仿真。
- `tofd --sim` 从同一个仿真器拿 8×8 深度；`mediad --sim-camera` 拿渲染出来的头部相机图像。
- `scripts/duck-sim` 负责 body、daemon、策略、socket、健康检查和站立。
- 相机是 opt-in：`DUCK_SIM_CAMERAS=a`；相机帧端口默认 7901，每个相机 +1。
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
缺少一个通用的 perception → fusion → brain 的数据面。

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
- `dora-mujoco-bridge`：仍是确定性合成步态的 stub，已从 `sim.yml` 移除；上游 twin 的 MuJoCo
  才是仿真来源。
- `dataflows/sim.yml`：三条节点，状态链和控制链分离；假反馈环已删除。
- `scripts/dora-sim`：负责 twin 生命周期、tty echo、`ROBOTD_SOCKET`、`dora run`。

也就是说：**控制链和状态链现在都是真的。** 缺的是感知和行为。

### 4.2 目标图

```
robotd-state ──joint_state / odom / imu──┐
                                         ├──▶ brain ──cmd_vel──▶ wrapper ──robot.move──▶ robotd
camera ──image──▶ perception ──target────┘
```

`robotd-state` 已经是当前实现；`camera`、`perception`、`brain` 还是计划。

### 4.3 分步

1. **真状态（已完成）**：`dora-robotd-state` 订阅 `robot.subscribe`，发布 `joint_state`
   （15 f32）、`odom`、`imu`；`sim.yml` 的 `joint_state` 来源已换成它，`mujoco-bridge` 已从
   图中移除。不要在 dora 节点里重新实现 MuJoCo。
2. **相机 + SHM（下一步）**：启动 sim 时 `DUCK_SIM_CAMERAS=a`；新增 `dora-camera`，按 30 Hz 调
   `media.frame`，用 `send_output_raw` 把二进制尾部直接读进 sample；节点配
   `shared_memory_pool_size: 64MB`。确认日志出现 `zenoh direct path`。
3. **最小 perception + brain**：先不接 NPU/模型，做一个最笨的检测（颜色/亮度/运动），
   输出 `target_visible`、`target_bearing`；`brain` 做反应式控制（可见则转向/前进，
   丢失则停）。`robotd-wrapper` 仍是唯一 intent sink。
4. **管理**：YAML 加 `restart_policy: on-failure`、`startup_timeout`、
   `health_check_timeout`；切片稳定后再加 `dora up` + `scripts/dora-sim managed`。
5. **之后**：16 状态大脑、NPU 鸭子检测器、真机部署、板载资源测量。

### 4.4 验收

- `joint_state` / `odom` 来自真实 `robot.state`，不再是合成步态；smoke test 的首帧已是站立时
  的真实值（已完成）。
- 相机帧（>4 KB）在日志里走 `zenoh direct path`，不是 `daemon path`。
- 目标进入视野时鸭子转向/前进，目标丢失时停下。
- brain 被杀掉时，`robotd` 的 deadman 让鸭子停下；如果加了 restart policy，brain 能回来。
- `dora node list` 显示所有节点健康。

## 5. 参考

本仓库：

- `crates/dora-keyboard/src/main.rs`：键盘解析、raw mode、hold 计时、测试。
- `crates/dora-robotd-state/src/main.rs`：`robot.subscribe` → `joint_state` / `odom` / `imu`。
- `crates/dora-robotd-wrapper/src/main.rs`：`cmd_vel` → `robot.move`。
- `crates/dora-mujoco-bridge/src/main.rs`：合成步态 stub（不在 `sim.yml` 中）。
- `dataflows/sim.yml`：三条节点和 tick。
- `scripts/dora-sim`：twin 生命周期和 `ROBOTD_SOCKET`。
- `README.md`、`CLAUDE.md`：命令、版本耦合、约定。

上游：

- `vendor/microduck/docs/design/architecture.md` §1（服务拆分）、§2.1–2.4（控制面/数据面、
  IPC、帧不跨 socket）。
- `vendor/microduck/docs/design/robotd-design.md` §4.5（挂在 tick 上的五个子系统）。
- `vendor/microduck/docs/robot/simulation.md`（`robotd --sim`、`tofd --sim`、
  `mediad --sim-camera`、`DUCK_SIM_CAMERAS`）。
- `vendor/microduck/docs/ideas/autonomous_behavior.md`（16 状态机、缺失的大脑）。
- `vendor/microduck/docs/project/npu-bringup.md`（检测器和缺失的帧路径）。
- `vendor/microduck/duck-ipc-proto/src/lib.rs`（`robot.*` 方法、`RobotState`、`media.frame`、
  `MediaFrameHeader`、`JOINT_NAMES`）。

dora 1.1.0：

- `dora-node-api`：`ZERO_COPY_THRESHOLD`（4096）、`SampleAllocator`、`send_output_raw`、
  `send_output`、`decode_arrow_ipc_zero_copy`。
- `dora-core` descriptor：`restart_policy`、`max_restarts`、`restart_delay`、
  `max_restart_delay`、`restart_window`、`startup_timeout`、`health_check_timeout`。
- `dora-cli`：`dora up/start/stop/restart/list/logs/inspect/top/node/param/topic`。
