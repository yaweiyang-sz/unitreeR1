# 常见问题排查

## 部署 / 环境

### `Could not locate cyclonedds`

**原因**：`CYCLONEDDS_HOME` 环境变量没设对。

**解决**：
```bash
# 检查
echo $CYCLONEDDS_HOME
# 应该是 /home/unitree/unitree_ws/thirdparty/cyclonedds/install

# 永久生效
echo 'export CYCLONEDDS_HOME=~/unitree_ws/thirdparty/cyclonedds/install' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### `ImportError: No module named unitree_sdk2py` / `No module named unitree_sdk2py.r1.loco`

**原因**：SDK 没装好，或者装的是别的版本/分支。

**解决**：
```bash
cd ~/unitree_sdk2_python
pip3 install -e .
# 验证三个关键模块都能 import
python3 -c "from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient; print('r1 loco ok')"
python3 -c "from unitree_sdk2py.go2.video.video_client import VideoClient; print('go2 video ok')"
python3 -c "from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_; print('SportModeState ok')"
```

### MediaPipe 装不上 / 装上后 import 报错

**原因**：aarch64 上某些老版本 MediaPipe 缺依赖。

**解决**：
```bash
# 优先装 0.10.18
pip3 install mediapipe==0.10.18
# 不行就试 0.10.14
# 都不行就退回到 OpenCV DNN
```

## 网络 / 机器人连接

### `LocoClient.Init()` 超时

**检查顺序**：
1. **机器人开机了吗？** 红色 LED 在闪说明在启动
2. **在调试模式吗？** App 里要切到"调试模式"或"开发者模式"
3. **网线/WiFi 通吗？**
   ```bash
   ping 192.168.123.161   # 机器人主控
   ping 192.168.123.164   # 你的 Jetson
   ```
4. **网卡名对吗？**
   ```bash
   ifconfig | grep 192.168
   # 看 inet 是 192.168.123.x 的那块网卡叫什么 (eth0? enp0s3?)
   ```
5. **DDS 域冲突?** 重启机器人再试

### `unitree_sdk2py.g1.sport.g1_sport_client` 不存在

**老代码** (v1 之前) 引用的是 G1 的接口。R1 EDU 走的是自己的 R1 接口。
新代码用 `unitree_sdk2py.r1.loco.r1_loco_client.LocoClient`，
方法名也不一样 (R1 是 FSM 风格: `Start()` / `Stance()` / `Damp()`, 不是 G1 风格的 `StandUp()` / `BalanceStand()`)。

如果你看到 import `g1_sport_client` / `G1SportClient` 的报错, 说明你拉的是老代码, 请 `git pull` 拉最新。

### 视频取不到 (`GetImageSample` 返回非 0)

- **App 里没开视频流** → App → 设置 → 视频
- **机器人没在调试模式** → 切调试模式
- **R1 没有自己的 video_client, 必须用 Go2 的**:
  ```python
  from unitree_sdk2py.go2.video.video_client import VideoClient
  ```
  这是 R1 与 Go2 共享 video_service, 不是错路。
- **网络抖动** → 重试

```bash
# 单独跑视频测试看具体错误
python3 scripts/test_video_stream.py eth0
```

### 状态一直 `unknown` / `mode=0`

- 默认订阅主题是 `rt/sportmodestate`。部分新固件用 `rt/lf/sportmodestate`:
  ```bash
  python3 src/main.py eth0 --sport-state-topic rt/lf/sportmodestate
  ```
  sdk_client.py 内部也会自动 fallback 一次, 但显式传更稳。
- 也可能是订阅没建好, 用 cyclonedds 命令行看 topic 列表:
  ```bash
  cyclonedds ls
  ```

### 机器人不响应 Move (发速度了但不动)

- 99% 的情况是**机器人没在 FSM 811 (RUNNING)**. R1 不在 locomotion 时 Move 指令会被拒。
- 启动时加 `--enable-loco` 让程序显式调 `LocoClient.Start()`:
  ```bash
  python3 src/main.py eth0 --enable-loco
  ```
  屏幕上 telemetry 会显示 `fsm: running`。如果一直是 `fsm: stand`, 说明 Start() 没生效:
  - 检查 App 是否真的在调试模式
  - 启动时让机器人**已经在平衡站立状态** (Stance, FSM 4), 不能是 DAMP/LIE/未启动

### 走动 1 秒后突然停

- 这就是没设 `continous_move=True` 的症状。R1 `LocoClient.Move(vx,vy,vyaw)` 默认 `duration=1.0` 秒, 1s 后自动清零。
- 我们 sdk_client.py 内部已经固定 `continous_move=True`, 如果你看到这个症状说明你跑的是老代码, 请 `git pull`。

## 视觉 / 手势

### 手势识别率低 / 经常 UNKNOWN

- **光线太暗或逆光** → 加灯或换个位置
- **手离镜头太远** → 0.5 ~ 1.5 米是最佳
- **背景太乱** → 用纯色背景更容易识别
- **手指并拢/弯曲不到位** → 看 `docs/GESTURES.md` 调整

调试方法：
```bash
python3 scripts/test_gesture_webcam.py
# 看 conf 数值, 应该 > 0.7 才好
```

### 跟随时机器人左右抖

- 减小 `kp_yaw`，或者
- 加大 `deadzone_yaw` (在 `config.yaml`)

```yaml
follow:
  kp_yaw: 0.0015          # 原 0.0025
  deadzone_yaw: 50        # 原 30
```

### 跟随距离不稳

- 调 `target_bbox_area` (期望人体在画面中的像素面积)
- 调 `kp_distance`
- 加大 `deadzone_area` 减少抖动

### MediaPipe 在 Jetson Nano 上跑得太慢 (FPS < 5)

- 确认 `jetson_clocks` 拉满: `sudo jetson_clocks`
- 降低摄像头分辨率
- 关掉可视化 OpenCV 窗口 (`--no-window` + Web 流)

## 主程序 / 状态机

### 状态机不切换 (手势识别了但模式没变)

- 看 `--no-window` 模式下没有 OpenCV 窗口，但 web 流应该能看
- 检查状态是否停在 `IDLE` (从没识别到手)
- `STOP` 持续 1.5 秒才会进跟随 — 多等一会儿
- 如果 `fsm: stand` 但你希望它在 `running`, 缺 `--enable-loco`

### 急停 (STOPPED) 退不出去

按 `q` / `ESC` 退出主程序，重新跑。急停逻辑只清速度，不退出程序。

## 视野显示

### 看不到 OpenCV 窗口

```bash
# 检查 DISPLAY 环境变量
echo $DISPLAY
# Jetson 上没显示器时:
python3 src/main.py eth0 --no-window
# 然后用 Web 流看
```

### Web 流访问不到 (http://192.168.123.164:8080)

- 端口可能被防火墙挡: `sudo ufw allow 8080`
- 端口被占用: 改 `config.yaml` 里的 `viewer.web_port`
- 浏览器输错 IP: 确认你 PC 的 `192.168.123.164` 是 PC2 (Jetson) 而不是 PC1

### Web 视频卡顿

- 局域网速度太慢 (WiFi 信号差)
- 改 `viewer.py` 里的 `quality=70` 调低
- MJPEG 本身效率不高，远程看就足够；如要低延迟，考虑用 WebRTC (后续扩展)

## 调试技巧

```bash
# 看 DDS 主题是否在跑 (装 cyclonedds-tools)
sudo apt install cyclonedds-tools
cyclonedds ls

# 看网络是否通
ping 192.168.123.161

# 强制 dry-run, 用本机摄像头先跑通视觉
python3 src/main.py eth0 --dry-run

# 看 SDK 报错细节 (最小复现, 不进 locomotion)
python3 -c "
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient
ChannelFactoryInitialize(0, 'eth0')
c = LocoClient(); c.SetTimeout(5.0); c.Init()
print('LocoClient init ok')
# 试进入 locomotion
c.Start()
print('Start ok (FSM 811)')
import time; time.sleep(0.5)
c.Move(0.1, 0, 0, True)
time.sleep(2)
c.StopMove()
c.Stance()
print('Stance ok (FSM 4)')
"
```

## 已知限制 (R1 EDU)

- **R1 EDU 关节数 26** (与 G1 相同) 但尺寸更小, 重心/动力学更敏感
- **R1 EDU 没有激光雷达**: 跟随时主要靠视觉，无距离传感器，跟得太近可能撞
- **MediaPipe 在 Jetson Nano 上帧率有限**: ~10 FPS 是常态，不影响功能但响应有延迟
- **R1 状态消息走 `unitree_go` 命名空间** (不是 `unitree_hg`)。R1 high-level example
  里 import `unitree_go_msg_dds__SportModeState_` 是对的, 不要去找 `unitree_hg_msg_dds__SportModeState_`

如果遇到本文档没列出的问题，把：
- 完整错误堆栈
- `python3 --version`, `pip3 list | grep -E "mediapipe|opencv|unitree"`
- 机器人是否在调试模式、是否在 Stance (FSM 4)
- `cyclonedds ls` 输出 (看订阅主题)
- 网络连接情况

贴到 issue，一起查。
