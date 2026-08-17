# Unitree R1 EDU 手势控制与跟随项目

让宇树 R1 EDU（自带 Jetson Nano + Linux）通过**手势**控制前进/后退/左转/右转/停止，并能**跟随**画面里指定的人，**视野**实时显示在屏幕上。

> 适配机型：**Unitree R1 EDU**（自带 Jetson Nano 主控）
> 通信：宇树官方 `unitree_sdk2_python`（DDS / CycloneDDS 0.10.2）
> 上位机：机器人内置 PC2（`192.168.123.164`，SSH 登录开发）
> 开发主机：Windows 10/11 远程编辑 + SSH 部署

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
# 1. SSH 登录到机器人内置 PC（Jetson Nano）
ssh unitree@192.168.123.164   # 默认密码 123

# 2. 克隆并安装
cd ~
git clone <your-repo> unitreeR1
cd unitreeR1
pip3 install -r requirements.txt

# 3. SDK 集成测试
python3 scripts/test_sdk_connection.py eth0
python3 scripts/test_video_stream.py eth0

# 4. 启动主控
python3 src/main.py eth0
```

详细步骤见 `docs/DEPLOY.md`。

---

## 文档导航

- [架构说明](docs/ARCHITECTURE.md) — 模块划分、数据流
- [部署指南](docs/DEPLOY.md) — Jetson Nano 上从零搭建
- [手势定义](docs/GESTURES.md) — 手势 ↔ 动作映射
- [常见问题](docs/TROUBLESHOOTING.md) — 网络 / DDS / 视频

---

## 项目结构

```
unitreeR1/
├── src/
│   ├── robot/        # 宇树 SDK 封装（运控 + 视频）
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

## 安全提示 ⚠️

1. **首次运行**请把机器人用保护支架悬吊，或周围留足 2m 空间
2. **随时准备物理急停**（机器人胸口按钮）
3. 默认最大线速度 `0.3 m/s`、角速度 `0.5 rad/s`，可在 `config.yaml` 调整
4. 跟随模式**必须有障碍物感知**或人工监督，避免撞墙
