# 部署指南 (Jetson Nano / R1 EDU 内置 PC)

R1 EDU 自带 Jetson Nano 主控（Ubuntu 20.04 + Python 3.8），我们把程序直接跑在它上面。开发机（Windows）只负责编辑代码 → 同步到机器人 → 启动运行。

## 0. 网络准备

确认 R1 EDU 已经开机，Jetson Nano 通过网线或 WiFi 与你的电脑在同一网段。

| 设备 | IP | 角色 |
|------|-----|------|
| R1 机载 PC1 (主控) | 192.168.123.161 | 运动控制 (不要 SSH) |
| **R1 机载 PC2 (Jetson Nano)** | **192.168.123.164** | **你 SSH 上去开发** |
| 你的电脑 (Windows) | 192.168.123.x | 编辑代码、部署 |

SSH 登录：

```bash
ssh unitree@192.168.123.164
# 默认密码: 123
```

⚠️ **如果密码不对**：宇树 R1 EDU 出厂默认是 `unitree / 123`，但部分批次可能要求你先用 App 连接热点 → 设置 WiFi → 才能用 SSH。参考随箱的快速开始文档。

## 1. 在机器人上准备环境

SSH 进去后:

```bash
# 检查 Python
python3 --version
# 应输出 Python 3.8.x

# 检查 pip
python3 -m pip --version
# 如果没有: sudo apt install python3-pip
```

### 1.1 安装 unitree_sdk2_python (从源码)

R1 EDU 走的是 SDK2 的 R1 分支，路径是 `unitree_sdk2py.r1.loco.r1_loco_client.LocoClient`。
视频是 Go2 路径 `unitree_sdk2py.go2.video.video_client.VideoClient`（R1 没有自己的 video_client，复用 Go2 的）。

```bash
# 1. 安装 cyclonedds 0.10.x
sudo apt update
sudo apt install -y build-essential cmake git

mkdir -p ~/unitree_ws/thirdparty && cd ~/unitree_ws/thirdparty
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_DDSPERF=OFF
cmake --build . --target install
cd ~

# 2. 编译并安装 unitree_sdk2_python
export CYCLONEDDS_HOME=~/unitree_ws/thirdparty/cyclonedds/install
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH

# 永久生效 (写到 ~/.bashrc)
echo 'export CYCLONEDDS_HOME=~/unitree_ws/thirdparty/cyclonedds/install' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PATH=$CYCLONEDDS_HOME/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
cd ~
```

> 验证 SDK 装好且模块路径对:
> ```bash
> python3 -c "from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient; print('r1 loco ok')"
> python3 -c "from unitree_sdk2py.go2.video.video_client import VideoClient; print('go2 video ok')"
> python3 -c "from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_; print('SportModeState ok')"
> ```

### 1.2 安装 OpenCV + MediaPipe

```bash
sudo apt install -y libopencv-dev python3-opencv
pip3 install opencv-python PyYAML

# MediaPipe 在 aarch64 上装官方预编译包即可
pip3 install mediapipe==0.10.18
# 如果装不上 (网络/版本问题), 退到:
# pip3 install mediapipe==0.10.14
```

### 1.3 部署项目代码

**方法 A: 用 git 同步 (推荐)**

在你的开发机 (Windows + Git Bash / WSL) 上:

```bash
# 先把代码传到 Git 仓库
cd D:\workspace\unitreeR1
git init
git add . && git commit -m "init"

# 然后在机器人上 clone
ssh unitree@192.168.123.164
cd ~
git clone <你的仓库地址> unitreeR1
cd unitreeR1
pip3 install -r requirements.txt
```

**方法 B: 用 deploy 脚本**

```bash
# 在 Windows 上 (Git Bash / WSL)
cd D:\workspace\unitreeR1
bash scripts/deploy_to_jetson.sh
```

**方法 C: scp / rsync 手动**

```bash
rsync -avz --exclude '__pycache__' --exclude '.git' \
  D:\workspace\unitreeR1/ unitree@192.168.123.164:~/unitreeR1/
```

## 2. 进入调试模式

**重要**: R1 EDU 必须先用 App 或遥控器进入调试模式，高层运控接口才有效。

1. 打开 R1 配套的 Unitree App（手机或平板）
2. 找到 R1 蓝牙 → 连接
3. 切换到"调试模式" / "开发者模式"（具体名称以 App 为准）
4. App 上应该显示 PC1/PC2 的 IP
5. **进调试模式后, 机器人应在 FSM 4 (Stance) —— 不动, 保持平衡站立**

## 3. 验证 SDK 集成 (按顺序跑)

```bash
ssh unitree@192.168.123.164
cd ~/unitreeR1

# 测试 1: 仅连接 + 读状态 (不动机器人)
python3 scripts/test_sdk_connection.py eth0

# 测试 1b: 跑完整 FSM 流程 (机器人会真的走两步, 需要支架 + 周围 ≥ 2m)
python3 scripts/test_sdk_connection.py eth0 --enter-loco --yes

# 测试 2: 视频流
python3 scripts/test_video_stream.py eth0 --save frame.jpg

# 测试 3: 手势识别 (用 USB 摄像头, 不需要机器人)
python3 scripts/test_gesture_webcam.py

# 测试 4: 人体跟随 (用 USB 摄像头)
python3 scripts/test_pose_webcam.py
```

如果 `test_sdk_connection.py` 报 "init 失败":
- 确认 R1 已开机 + 在调试模式
- `ifconfig` 看你 PC2 的网卡名 (常见 `eth0`、`enp0s3`)，把命令里的 `eth0` 换成实际名字
- 确认机器人和 PC2 在同一网段: `ping 192.168.123.161`
- 确认 cyclonedds 安装: `python3 -c "import cyclonedds; print(cyclonedds.__version__)"`

## 4. 启动主控

```bash
# 默认: 真机, 但不进入 locomotion (机器人停在 Stance, 只有视频/视觉流跑通)
python3 src/main.py eth0

# 真机 + 真的进入 locomotion (会要求按 Enter 二次确认)
python3 src/main.py eth0 --enable-loco

# 服务器环境 (没显示器):
python3 src/main.py eth0 --no-window

# 调试 (用本机 USB 摄像头, 不连机器人):
python3 src/main.py eth0 --dry-run

# 启动后直接进 FOLLOW 模式 (跳过手势):
python3 src/main.py eth0 --enable-loco --follow
```

主控启动后:
- **OpenCV 窗口**: 直接看画面 (有显示器时)
- **Web 浏览器**: 同一局域网的电脑/手机访问 `http://192.168.123.164:8080/`
- 按 `q` 或 `ESC` 退出 / 急停
- 退出时会自动 `StopMove() → Stance()`, 机器人会回到平衡站立

## 5. 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| `Could not locate cyclonedds` | 环境变量没设 | `export CYCLONEDDS_HOME=...` |
| `LocoClient.Init` 超时 | 网络不通 / 没在调试模式 | 1) ping 161 2) 进 App 切调试模式 |
| `ImportError: unitree_sdk2py.r1.loco` | SDK 没装 / 装错分支 | `cd unitree_sdk2_python && pip3 install -e .` |
| `cv2.imshow` 报错 | Jetson 无显示器 | 加 `--no-window` |
| Web 流 8080 访问不到 | 防火墙 / 端口占用 | `sudo ufw allow 8080` 或换端口 |
| 视频 `GetImageSample` 拿不到 | 摄像头服务没起 | App 里"开启视频" / 重启机器人 |
| 机器人不响应 Move | 没在 FSM 811 (RUNNING) | 加 `--enable-loco` 或 config 打开 auto_enter_locomotion |
| 状态一直 unknown | 订阅主题名不对 | 用 `--sport-state-topic rt/lf/sportmodestate` 试一下 |
| 机器人走动 1s 后停 | 没设 `continous_move=True` (SDK 客户端 bug) | 确认用的是我们改过的 sdk_client.py |

详见 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)。

## 6. 性能调优 (Jetson Nano)

Jetson Nano 算力有限 (472 GFLOPS)，如果你想流畅跑：

```bash
# 1) jetson-clocks 拉满 (需要 root)
sudo jetson_clocks

# 2) 关掉一些重负载
# 在 config.yaml 里:
#   gesture.debounce_frames: 10     # 跳帧
#   follow.kp_yaw: 0.003            # 大幅动作, 减少微调
#   viewer.opencv_window: false     # 服务器模式
#   viewer.web_port: 8080           # 用 Web 流代替 OpenCV
```

如果 MediaPipe 跑不动:
- 用 `--model-complexity 0` (已经在 BodyFollower 里设了)
- 降低摄像头分辨率: `cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)`
- 退回到 OpenCV DNN + MobileNet SSD (后续版本会加)
