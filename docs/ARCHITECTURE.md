# 架构说明

## 总览

```
                    ┌─────────────────────────────────────────┐
                    │  宇树 R1 EDU  (Jetson Nano, Ubuntu 20.04)│
                    │  ┌─────────────────────────────────┐    │
                    │  │  src/main.py  (状态机)           │    │
                    │  └────────────┬────────────────────┘    │
                    │               │                         │
  R1 前置摄像头 ────┼──► vision/ ──┴─► control/ ──► robot/   │
                    │   (手势+跟随)    (平滑+映射)   (SDK) ───┼──► DDS 网络
                    │                                         │      │
                    │  view/  ◄───────────────────────────────┘      │
                    │  (本地窗口 + Web 流 @:8080)                   │
                    │                                              │
                    │  192.168.123.164  ◄──── 你的电脑 (192.168.123.x)
                    └─────────────────────────────────────────┘
```

## 模块职责

| 包 | 职责 | 关键文件 |
|----|------|----------|
| `src/robot/` | 宇树 R1 SDK 封装 (LocoClient + 视频 + 状态订阅) | `sdk_client.py` |
| `src/vision/` | 手势识别 + 人体跟随 + UI 叠加 | `hand_gesture.py`, `body_follow.py`, `overlays.py` |
| `src/control/` | 速度平滑 / 手势映射 / 跟随控制 | `velocity_smoother.py`, `gesture_to_command.py`, `follow_controller.py` |
| `src/view/` | OpenCV 本地窗口 + Web MJPEG 流 | `viewer.py` |
| `src/main.py` | 状态机主控 | - |

## 数据流 (一帧)

```
 1. 取一帧 ── robot.get_image()  OR  本机 USB 摄像头
        │
 2. 手势识别 ── vision.hand_gesture.detect(frame)
        │     → 21 关键点 → 5 指状态 → Gesture
        │
 3. 去抖 ── GestureDebouncer (连续 N 帧相同才生效)
        │
 4. 状态机决策 (main.py)
        │     GESTURE  → mapper.to_velocity(stable)
        │     FOLLOW   → BodyFollower.step(frame) → (vx, vyaw)
        │
 5. 平滑 ── VelocitySmoother.update(target)
        │     一阶低通 + 限步 + 限幅 + 衰减
        │
 6. 下发 ── robot.move(vx, vy, vyaw)         (内部 LocoClient.Move + continous_move=True)
        │
 7. UI ── draw overlays → Viewer (本地 + Web)
        │
 8. 状态切换判定 ── STOP 持续 1.5s → FOLLOW; 拳头 → 回 GESTURE
```

## 状态机 (本程序 + 机器人 FSM 双层)

### 本程序状态

```
        ┌──────┐  args  ┌──────────┐
        │ BOOT ├───────►│   IDLE   │ (启动后立刻进入, 几乎不驻留)
        └──┬───┘        └────┬─────┘
           │                 │
           │    默认          │ --follow
           ▼                 ▼
       ┌────────┐  STOP×1.5s ┌────────┐
       │GESTURE │◄───────────┤ FOLLOW │
       └────┬───┘  BACKWARD  └────┬───┘
            │      ×0.5s         │
            │                    │
            └──► 用户按 q/ESC ──►┌──────────┐
                                 │ STOPPED  │ ──► 退出
                                 └──────────┘
```

### 机器人 FSM (R1 LocoClient)

```
              ┌────────────┐
              │  ZERO_TORQUE│ FSM 0  (ZeroTorque)
              └──────┬─────┘
                     ▼
              ┌────────────┐
              │   DAMP     │ FSM 1   ← 急停, 切断电机
              └──────┬─────┘
                     ▼
              ┌────────────┐
   ┌──────────►│  STANCE    │ FSM 4   ← 默认, App 切调试模式后机器人自动在此
   │           └──────┬─────┘
   │                  │ Start()
   │                  ▼
   │           ┌────────────┐
   │           │  RUNNING   │ FSM 811 ← 真正在走, Move() 才会被接受
   │           └──────┬─────┘
   │                  │ Stance() / 速度清零超时
   │                  ▼
   └──────────  STANCE
   │           ┌────────────┐
   │  Lie2StandUp │LIE_TO_STAND│ FSM 701
   │           └────────────┘
   │
   │  StandUp2Lie ┌────────────┐
   └─────────────►│STAND_TO_LIE│ FSM 702
                 └────────────┘
```

`R1Client.enter_locomotion()` 内部调 `LocoClient.Start()` 把机器人推到 FSM 811;
`R1Client.exit_locomotion()` 先 `StopMove()` 再 `LocoClient.Stance()` 拉回 FSM 4。
**关键不变量**: 不在 FSM 811 时发 `Move`, 机器人**不响应**。所以默认启动时本程序不发 locomotion, 除非显式加 `--enable-loco`。

## 关键设计

### 1. Dry-run 模式
`R1Client(dry_run=True)` 创建一个不依赖 SDK 的实现，让你在没有机器人时也能用本机 USB 摄像头跑视觉模块和状态机。

```bash
python3 src/main.py eth0 --dry-run
```

### 2. 视频源降级
- 优先机器人前置摄像头 (Go2 VideoClient 复用, R1 共用同一套 video_service)
- 失败 / dry-run → 本机 USB 摄像头 (index 0)
- 都没有 → ERROR 退出

### 3. 速度平滑
防止识别抖动导致指令跳变：
- **一阶低通** α=0.3 (新值占 30%)
- **限步** vx 每周期最多变 0.05 m/s
- **限幅** vx ≤ 0.5 m/s, vyaw ≤ 1.0 rad/s
- **衰减** 没有目标时, 速度在 ~0.8s 内回到 0

### 4. Move 持续模式
R1 `LocoClient.Move(vx, vy, vyaw, continous_move=False)` 默认 1 秒后自动停速度。
我们的控制循环周期性发指令，所以 `R1Client.move()` 内部固定传 `continous_move=True`，
让机器人持续保持当前速度。如果循环卡住或主程序崩了, R1 的内置 1s 超时仍然会兜底把机器人刹住。

### 5. 线程模型
- 主循环单线程 (简单、可控)
- Web 流用一个后台 HTTP 服务器线程
- SDK 调用本身线程不安全 → 主循环串行调用
- DDS 订阅 (`SportModeState_`) 由 cyclonedds 内部 worker 线程回调, 写共享状态时用锁保护

### 6. 配置驱动
`config.yaml` 集中所有可调参数 (速度、增益、跟随距离、locomotion 是否自动进等)，改配置不用动代码。

## 关键主题 (DDS topics)

R1 高层走 `unitree_sdk2py.r1.loco.r1_loco_client` (LocoClient, RPC over DDS);
视频走 `unitree_sdk2py.go2.video.video_client` (R1 与 Go2 共用 video_service);
状态走 DDS 直订 `SportModeState_`。

| 主题 | 方向 | 用途 | 来源 |
|------|------|------|------|
| `rt/sportmodestate` | in | 机器人高层状态 (mode/position/velocity/IMU) | `ChannelSubscriber(SportModeState_)` |
| `rt/lf/sportmodestate` | in | 同上, 新固件用此名 | fallback |
| `rt/api/sport/request` | out | LocoClient RPC 请求 (高层运控) | `LocoClient._Call()` |
| video_service (RPC) | out/in | 视频流 JPEG 取帧 | `VideoClient.GetImageSample()` |

> 注: R1 内部另一套 low-level 走 `rt/lowcmd` / `rt/lowstate` (HG 命名空间),
> 那是 26 关节直接控制用的 (见 `example/r1/low_level/`), 本项目不用。

## 后续可扩展

- ✋ 多手势 (拳打开/OK/胜利) → 触发不同行为
- 🗣️ 语音控制 (R1 SDK 暂无高层 audio client, 可走 HG low-level 自行发布)
- 🚧 避障 (R1 没雷达, 靠前置摄像头做深度估计)
- 🗺️ SLAM 建图 (用前置摄像头做 V-SLAM)
- 📡 远程图传: Web 流用 H.264 而不是 MJPEG
- 🤖 多机协同
