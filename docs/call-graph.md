# 鸭子的调用关系

> 这张图回答“谁调用谁、用什么方式调用”。它把上游 Microduck 的既有调用关系与 dora 数据流
> 叠在一起看，**不改变 `robotd` 的权威边界**：所有进入 `robotd` 的箭头都是 intent，没有
> 任何一条能直接写舵机。
>
> 配套阅读：`docs/architecture.md` §2（上游架构）与 §3（dora 插入点）。

## 1. 总览（组件关系）

图例：

- 实线箭头：请求/响应（JSON-RPC call，可能有回复）。
- 虚线箭头：通知/订阅推送（server → client，不需要回复）。
- 粗箭头：大块数据（相机帧、WebRTC、共享内存）。
- 虚线框节点：计划中的 dora 节点，当前仓库还没有。

```mermaid
flowchart TB
  subgraph clients["客户端"]
    phone["Phone app"]
    pad["Gamepad"]
    cli["robotctl / duckctl"]
    space["HF Space"]
  end

  subgraph transport["传输 / 前端"]
    btd["btd（BLE）"]
    padd["padd / netpadd"]
    mediad["mediad<br/>WebRTC + media.frame"]
  end

  subgraph daemons["常驻 daemon（systemd 监督）"]
    robotd["robotd<br/>50 Hz · intents · safety · policies"]
    configd["configd"]
    updaterd["updaterd"]
    tofd["tofd"]
  end

  subgraph hardware["硬件 / OS"]
    bus["15 个舵机 + IMU"]
    camhw["相机 / 麦克风"]
    tofhw["8×8 ToF"]
    radio["BLE 射频"]
    speaker["扬声器"]
    os["BlueZ / NetworkManager"]
  end

  subgraph dora["dora dataflow（可选，在 robotd 之上）"]
    kb["keyboard"]
    wrapper["robotd-wrapper"]
    state["robotd-state"]
    camera["camera"]
    percep["perception"]
    brain["brain"]
  end

  phone -->|"BLE"| btd
  phone -->|"WebRTC"| mediad
  pad -->|"USB / BLE / UDP"| padd
  cli -->|"robot.*"| robotd
  cli -->|"net.* / system.*"| configd
  cli -->|"update.*"| updaterd
  cli -->|"tof.stream"| tofd
  cli -->|"media.frame"| mediad

  btd -->|"robot.* intents"| robotd
  btd -->|"net.* / system.*"| configd
  btd -->|"update.*"| updaterd
  padd -->|"robot.move / robot.do"| robotd

  kb -->|"cmd_vel"| wrapper
  wrapper -->|"robot.move（notification）"| robotd
  robotd -.->|"robot.state（50 Hz 订阅推送）"| state
  state -.->|"joint_state / odom / imu"| brain
  camera -->|"media.frame 请求"| mediad
  mediad ==>|"header + UYVY 字节"| camera
  camera ==>|"image（≥4 KB，SHM）"| percep
  percep -->|"target"| brain
  brain -->|"cmd_vel（计划）"| wrapper

  mediad ==>|"WebRTC H.264 + control"| space
  mediad --> camhw
  tofd --> tofhw
  robotd --> bus
  robotd -->|"aplay 子进程"| speaker
  btd --> radio
  configd -->|"D-Bus"| os

  updaterd -.->|"robot.health"| robotd
  updaterd -->|"systemctl restart"| systemd["systemd（监督者）"]
  systemd -.->|"start / stop / restart"| daemons

  classDef planned stroke-dasharray: 5 5,stroke:#999,fill:#f6f6f6;
  class state,camera,percep,brain planned;
```

读法：

- 左边是客户端，中间是传输/daemon，下面是硬件。
- `robotd` 是唯一通向舵机的箭头；`btd`、`padd`、`robotctl`、dora 的 wrapper 都只是 intent
  的发送方。
- `mediad` 同时是相机所有者（`camhw`）和两个数据出口：WebRTC（给人/远端）和
  `media.frame`（给本机 perception）。
- `tofd` 单独拥有 ToF，不经过 `robotd`。
- `configd` / `updaterd` / `btd` 是恢复路径：它们不依赖 `robotd`，`robotd` 死了也要能工作。
- dora 的 `keyboard → wrapper` 是当前唯一的真实控制链；`state`、`camera`、`perception`、
  `brain` 是目标切片，虚线框标出。

## 2. 调用流程（当前真实路径）

如果只读一张图，读这张。它按**调用顺序**画：矩形是处理步骤，菱形是分支，虚线是订阅推送。
`X` 键和单独的 Esc 在 `K7` 的状态机里处理；`Ctrl-C` 单独画出来，因为它会恢复终端并停掉整个
数据流。

```mermaid
flowchart TD
  subgraph K["dora-keyboard（每 50 ms tick）"]
    K0["启动：DoraNode::init_from_env()"] --> K1{"能打开 /dev/tty？"}
    K1 -- "否" --> K2["打印 keyboard disabled<br/>每个 tick 发 [0,0,0]"]
    K1 -- "是" --> K3["termios raw mode<br/>tcsetpgrp 抢前台<br/>spawn 读线程"]
    K3 --> K4{"事件"}
    K4 -- "tick（50 ms）" --> K5["Keys::command()<br/>逐轴检查 Hold 超时"]
    K5 --> K6["send_output('cmd_vel', [vx,vy,vyaw])"]
    K4 -- "按键字节" --> K7["转义序列状态机<br/>更新 Hold / vx / vy / vyaw"]
    K7 --> K4
    K4 -- "Ctrl-C（0x03）" --> K8["quit：发最后一帧 0<br/>恢复终端 → SIGINT 前台组"]
    K2 --> K6
  end

  K6 --> K9["dora daemon 路由 cmd_vel"] --> W1
  subgraph W["dora-robotd-wrapper"]
    W1["events.recv() 收到 cmd_vel"] --> W2{"长度 == 3？"}
    W2 -- "否" --> W3["eyre 报错并退出"]
    W2 -- "是" --> W4{"ROBOTD_SOCKET 设置？"}
    W4 -- "否" --> W5["命令变化时打日志"]
    W4 -- "是" --> W6{"已连接？"}
    W6 -- "否" --> W7["UnixStream::connect"]
    W6 -- "是" --> W8["write_all(robot.move NDJSON)"]
    W7 --> W8
    W8 --> W9{"写成功？"}
    W9 -- "否" --> W10["重连一次，再写"]
    W9 -- "是" --> W11["完成"]
    W10 --> W11
  end

  W8 --> R1
  subgraph R["robotd（50 Hz）"]
    R1["解析 JSON-RPC notification"] --> R2["写入 intent slot<br/>last-value-wins"]
    R2 --> R3["控制 tick"]
    R3 --> R4{"intent deadman 未超时？"}
    R4 -- "否" --> R5["停止 / 保持站立"]
    R4 -- "是" --> R6{"安全层允许？<br/>关节限位 / 跌倒 / 热"}
    R6 -- "否" --> R5
    R6 -- "是" --> R7["策略推理 → 关节目标"]
    R7 --> R8["Dynamixel 写 15 个舵机"]
  end

  R8 --> H["15 个舵机 + IMU"]
  R3 -.->|"robot.state 订阅推送（50 Hz）"| S1
  subgraph S["dora-robotd-state（计划）"]
    S1["收到 robot.state"] --> S2["发布 joint_state / odom / imu"]
  end
  S2 -.-> B["dora brain（计划）"]
```

图里没有画出来、但在同一层发生的两件事：

- `dora-mujoco-bridge` 目前还在发合成 `joint_state`，经 wrapper 回传 `joint_command` 形成一个
  **占位反馈环**；它不是 `robotd` 的真实状态，见 `docs/architecture.md` §4。
- 键盘节点和 wrapper 之间的 `cmd_vel` 是 Arrow `f32[3]`，每 50 ms 一条；dora 的 daemon/zenoh
  负责路由，不改变命令语义。

## 3. 一次控制的时序

这张图把“键盘驱动”和“感知驱动”放在同一条时间轴上，并标出哪些是当前实现、哪些是计划。

```mermaid
sequenceDiagram
  autonumber
  participant K as keyboard（dora，当前）
  participant W as robotd-wrapper（dora，当前）
  participant R as robotd
  participant S as robotd-state（计划）
  participant C as camera（计划）
  participant M as mediad
  participant P as perception（计划）
  participant B as brain（计划）

  K->>W: cmd_vel [vx, vy, vyaw]（每 50 ms）
  W->>R: robot.move（notification，无 id、无回复）
  R-->>S: robot.state（50 Hz 订阅推送）
  S-->>B: joint_state / odom / imu
  C->>M: media.frame（请求一帧）
  M-->>C: JSON header + UYVY 字节（约 1.8 MiB）
  C-->>P: image（≥4 KB，SHM 零拷贝）
  P-->>B: target_visible / target_bearing
  B->>W: cmd_vel（行为层 intent，计划）

  Note over R: 安全层仍在 robotd：关节限位、<br/>跌倒失力、intent deadman；<br/>图不能绕过。
  Note over W,B: 键盘与 brain 不能同时直接写同一个输入；<br/>需要一个 arbiter 节点，<br/>或 wrapper 增加第二个 cmd_vel 输入。
```

时序里的关键点：

- `robot.move` 是 **notification**：没有 `id`，没有回复，发出去就结束。`robotd` 的安全层
  决定是否执行。
- `robot.state` 是 **订阅推送**：连接建立后 `robotd` 按 50 Hz 推（可用 `hz` 降频），不是
  客户端轮询。
- `media.frame` 是 **按需拉取**：请求登记后等下一帧，不是读缓存；一帧约 1.8 MiB，JSON 头
  之后紧跟原始 UYVY 字节。
- 大帧进入 dora 图后走 SHM（≥4096 B、同机、池子有空间），`send_output_raw` 是 0 拷贝，
  `send_output` 是 1 拷贝，接收端 0 拷贝。

## 4. 调用关系表

| 调用方 | 被调用方 | 方法 / 数据 | 类型 |
|---|---|---|---|
| Phone app | `btd` | BLE GATT → 路由后的 `robot.*` / `net.*` / `update.*` | transport |
| Phone app | `mediad` | WebRTC 媒体 + control datachannel | 双向数据 |
| Gamepad | `padd` / `netpadd` | `robot.move` / `robot.do` 等 | intent |
| `robotctl` / `duckctl` | `robotd` | `robot.move`、`robot.state`、`robot.health`、`robot.subscribe`… | 请求/响应 + 订阅 |
| `robotctl` | `configd` | `net.*`、`system.*` | 请求/响应 |
| `robotctl` | `updaterd` | `update.*`、进度通知 | 请求/响应 + 订阅 |
| `robotctl` | `tofd` | `tof.stream` | 订阅 |
| `robotctl` | `mediad` | `media.frame` | 请求 + 二进制 |
| `btd` | `robotd` | 路由后的 `robot.*` intents | transport |
| `btd` | `configd` / `updaterd` | `net.*` / `system.*` / `update.*` | transport |
| `padd` | `robotd` | `robot.move` 等 intents | notification |
| `mediad` | `robotd` | 状态/配置查询 | 请求/响应 |
| dora `keyboard` | `robotd-wrapper` | `cmd_vel` Arrow `f32[3]` | dora 消息 |
| dora `robotd-wrapper` | `robotd` | `robot.move` | notification |
| `robotd` | dora `robotd-state` | `robot.state` | 订阅推送 |
| dora `camera` | `mediad` | `media.frame` | 请求 + 二进制 |
| dora `camera` | `perception` | `image`（原始字节） | dora 消息（SHM） |
| `perception` | `brain` | `target_visible` / `target_bearing` | dora 消息 |
| `brain` | `robotd-wrapper` | `cmd_vel` | dora 消息（计划） |
| `updaterd` | `robotd` | `robot.health` | 请求/响应 |
| `updaterd` | `systemd` | `systemctl restart` | 进程控制 |
| `systemd` | 所有 daemon | start / stop / restart / watchdog | 监督 |
| `robotd` | 舵机总线 + IMU | Dynamixel 协议 | 硬件 |
| `mediad` | 相机 / 麦克风 | V4L2 / GStreamer | 硬件 |
| `tofd` | ToF | I²C / SPI | 硬件 |
| `configd` | BlueZ / NetworkManager | D-Bus | OS |

## 5. 边界说明

- **所有进入 `robotd` 的箭头都是 intent。** 没有任何客户端、transport、dora 节点能直接写
  舵机；`robotd` 内部的安全层（关节限位、跌倒失力、intent deadman）是最后一道门。
- **`robotd` 的控制环不等待任何调用方。** 它读的是 last-value-wins 缓存；一个卡住的
  `mediad` 或 dora 节点只会让感知退化，不会给电机控制加抖动。
- **dora 的 coordinator 不在数据路径上。** 它只负责 start/stop/restart/logs；节点之间的
  Arrow/SHM 不经过它。数据面的大帧更不经过它。
- **dora 是可选的。** 去掉 dora，上游的调用关系照常工作：`robotctl`、`padd`、`btd`、
  `mediad` 仍然能驱动和观察鸭子。
- **恢复路径不能被 dora 依赖。** `configd`、`updaterd`、`btd` 在 `robotd` 死掉时仍要工作；
  dora 挂在它们上面会把恢复路径变成“先有 dora 才能修机器人”。
- **跨机不共享内存。** `media.frame` 和 dora 的 SHM 都是同机路径；跨机走 WebRTC/网络。
