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
| `src/robot/` | 宇树 SDK 封装 (运控 + 视频) | `sdk_client.py` |
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
 6. 下发 ── robot.move(vx, vy, vyaw)
        │
 7. UI ── draw overlays → Viewer (本地 + Web)
        │
 8. 状态切换判定 ── STOP 持续 1.5s → FOLLOW; 拳头 → 回 GESTURE
```

## 状态机

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

## 关键设计

### 1. Dry-run 模式
`R1Client(dry_run=True)` 创建一个不依赖 SDK 的实现，让你在没有机器人时也能用本机 USB 摄像头跑视觉模块。

```bash
python3 src/main.py eth0 --dry-run    # 用本机摄像头跑完整流程
```

### 2. 视频源降级
- 优先机器人前置摄像头
- 失败 / dry-run → 本机 USB 摄像头 (index 0)
- 都没有 → ERROR 退出

### 3. 速度平滑
防止识别抖动导致指令跳变：
- **一阶低通** α=0.3 (新值占 30%)
- **限步** vx 每周期最多变 0.05 m/s
- **限幅** vx ≤ 0.5 m/s, vyaw ≤ 1.0 rad/s
- **衰减** 没有目标时, 速度在 ~0.8s 内回到 0

### 4. 线程模型
- 主循环单线程 (简单、可控)
- Web 流用一个后台 HTTP 服务器线程
- SDK 调用本身线程不安全 → 主循环串行调用

### 5. 配置驱动
`config.yaml` 集中所有可调参数 (速度、增益、跟随距离等)，改配置不用动代码。

## 关键主题 (DDS topics)

R1 EDU 通过 `unitree_sdk2py.g1.*` 模块访问 (与 G1 共享高层接口):

| 主题 | 方向 | 用途 |
|------|------|------|
| `rt/sportmoderequest` | out | 高层运控指令 (Move, StandUp...) |
| `rt/highstate` | in | 机器人状态 (IMU, 关节角, 电池...) |
| `rt/frontvideostream` | in | 前置摄像头 JPEG 流 |

## 后续可扩展

- ✋ 多手势 (拳打开/OK/胜利) → 触发不同行为
- 🗣️ 语音控制 (`unitree_sdk2py.g1.audio`)
- 🚧 避障 (`unitree_sdk2py.go2.obstacles_avoid`) — R1 是否支持待验证
- 🗺️ SLAM 建图 (用前置摄像头做 V-SLAM)
- 📡 远程图传: Web 流用 H.264 而不是 MJPEG
- 🤖 多机协同
