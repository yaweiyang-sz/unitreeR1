# Unitree R1 EDU 手势控制与跟随项目

让宇树 R1 EDU（自带 Jetson Nano + Linux）通过**手势**控制前进/后退/左转/右转/停止，并能**跟随**画面里指定的人，**视野**实时显示在屏幕上。

> 适配机型：**Unitree R1 EDU**（自带 Jetson Nano 主控）
> 通信：宇树官方 `unitree_sdk2_python` (DDS / CycloneDDS 0.10.x) 跨网卡
> 上位机：机器人内置 PC2（`192.168.123.164`，SSH 登录开发）
> 开发主机：Windows 10/11 远程编辑 + SSH 部署

## 真正使用的 SDK 路径

| 用途 | 模块 | 备注 |
|------|------|------|
| 运控 | `unitree_sdk2py.r1.loco.r1_loco_client.LocoClient` | R1 自己的 LocoClient，FSM 风格 |
| 视频 | `unitree_sdk2py.go2.video.video_client.VideoClient` | R1 无独立 video_client, 复用 Go2 (同一套 video_service) |
| 状态 | `ChannelSubscriber("rt/sportmodestate", SportModeState_)` | DDS 直订, `unitree_go.msg.dds_` 命名空间 |
| DDS 通道 | `ChannelFactoryInitialize(0, network_interface)` | `iface` 即 DDS 出口网卡 |

> **R1 生命周期 (FSM)**: `ZERO_TORQUE(0)` → `DAMP(1)` → `STANCE(4)` → `RUNNING(811)` ←→ `LIE_TO_STAND(701)` / `STAND_TO_LIE(702)`
> 默认不进入 `RUNNING` (FSM 811), 启动加 `--enable-loco` 才会让机器人真的走。

---

## 功能一览

| 模块 | 能力 |
|------|------|
| 🎮 **手势控制** | 5 类手势：✋停 / ⬆前 / ⬇后 / ⬅左 / ➡右 |
| 🚶 **人体跟随** | 锁定目标人，保持固定距离与朝向 |
| 👀 **视野显示** | 实时看到机器人前方画面，带识别框/手势标签 |
| 🛡️ **安全限速** | 最大线速度/角速度可配、速度平滑、急停按钮 |
| 🧪 **SDK 集成测试** | 独立脚本验证网络/状态/视频/运控 |
| 🎬 **Dry-run 模拟** | 没接机器人时也能跑视觉模块（用本机 USB 摄像头） |

---

## 快速开始

```bash
# 0) 在 R1 上准备好 unitree_sdk2_python (从源码装, 见 docs/DEPLOY.md)

# 1) SSH 登录到机器人内置 PC（Jetson Nano）
ssh unitree@192.168.123.164   # 默认密码 123

# 2) 部署代码 + 装依赖
cd ~
git clone <your-repo> unitreeR1
cd unitreeR1
pip3 install -r requirements.txt

# 3) SDK 集成测试
#    3a) 只验证连接 + 状态订阅, 不动机器人
python3 scripts/test_sdk_connection.py eth0
#    3b) 跑完整 FSM 流程: Stance -> Start -> Move -> Stance
python3 scripts/test_sdk_connection.py eth0 --enter-loco --yes
#    3c) 视频流
python3 scripts/test_video_stream.py eth0 --save frame.jpg

# 4) 启动主控
#    4a) 真机但停在 Stance (FSM 4), 视觉/视频能跑
python3 src/main.py eth0
#    4b) 真机 + 真的进入 locomotion (FSM 811), 会要求按 Enter 确认
python3 src/main.py eth0 --enable-loco
#    4c) 干跑, 用本机 USB 摄像头, 完全不连机器人
python3 src/main.py eth0 --dry-run
```

详细步骤见 `docs/DEPLOY.md`。

---

## 文档导航

- [架构说明](docs/ARCHITECTURE.md) — 模块划分、数据流、R1 FSM 状态机
- [部署指南](docs/DEPLOY.md) — Jetson Nano 上从零搭建 (cyclonedds + SDK + 依赖)
- [手势定义](docs/GESTURES.md) — 手势 ↔ 动作映射
- [常见问题](docs/TROUBLESHOOTING.md) — 网络 / DDS / 视频 / FSM

---

## 项目结构

```
unitreeR1/
├── src/
│   ├── robot/        # 宇树 R1 SDK 封装 (LocoClient + VideoClient + SportModeState_)
│   ├── vision/       # 手势识别 + 人体跟随
│   ├── control/      # 手势→指令、跟随控制、速度平滑
│   ├── view/         # 视野显示（本地 + Web）
│   ├── main.py       # 状态机主控
│   └── config.py     # 配置加载
├── scripts/          # 独立测试脚本
├── docs/             # 文档
├── config.yaml       # 全局配置
└── requirements.txt  # 依赖清单
```

---

## CLI 参数速查

```
python3 src/main.py <iface> [选项]
  <iface>            机器人所在网卡 (DDS 出口), 如 eth0
  --config FILE      配置文件 (默认 config.yaml)
  --dry-run          完全不连机器人, 用本机 USB 摄像头模拟
  --no-window        不显示 OpenCV 窗口
  --webcam N         fallback USB 摄像头 index (默认 0)
  --follow           启动后默认进 FOLLOW 模式
  --enable-loco      真机下让机器人进入 locomotion (FSM 811), 会要求二次确认
  --enter-loco-now   跳过 enter-loco 确认提示 (脚本场景)
  --sport-state-topic TOPIC  覆盖 SportModeState_ 订阅主题 (默认 rt/sportmodestate)
```

---

## 安全提示 ⚠️

1. **首次运行**请把机器人用保护支架悬吊，或周围留足 2m 空间
2. **随时准备物理急停**（机器人胸口按钮 / 软切 Damp 切断电机)
3. 默认最大线速度 `0.3 m/s`、角速度 `0.5 rad/s`，可在 `config.yaml` 调整
4. 跟随模式**必须有障碍物感知**或人工监督，避免撞墙
5. 默认不进入 locomotion (FSM 811), 启动加 `--enable-loco` 才会让机器人真的走
6. 退出主程序时会自动 `StopMove() → Stance()`, 机器人会回到平衡站立
