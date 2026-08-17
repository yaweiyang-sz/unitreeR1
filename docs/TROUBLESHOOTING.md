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

### `ImportError: No module named unitree_sdk2py`

**原因**：SDK 没装好。

**解决**：
```bash
cd ~/unitree_sdk2_python
pip3 install -e .
python3 -c "import unitree_sdk2py; print('ok')"
```

### MediaPipe 装不上 / 装上后 import 报错

**原因**：aarch64 上某些老版本 MediaPipe 缺依赖。

**解决**：
```bash
# 优先装 0.10.18
pip3 install mediapipe==0.10.18
# 不行就试 0.10.14
pip3 install mediapipe==0.10.14
# 都不行就退回到 OpenCV DNN
```

## 网络 / 机器人连接

### `SportClient.Init()` 超时

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

### `G1SportClient` / `G1VideoClient` 不存在

R1 EDU 的高层运控接口在 `unitree_sdk2py.g1.sport.g1_sport_client` (与 G1 共享)。如果报错说找不到：

```bash
# 看 SDK 实际装了哪些模块
python3 -c "from unitree_sdk2py.g1.sport import g1_sport_client; print('ok')"
python3 -c "from unitree_sdk2py.g1.video import g1_video_client; print('ok')"
```

如果某个模块确实没有，编辑 `src/robot/sdk_client.py` 的 import fallback 部分，临时用 `go2` 或其他模块顶替。

### 视频取不到 (`GetImageSample` 返回 -1)

- **App 里没开视频流** → App → 设置 → 视频
- **机器人没在调试模式** → 切调试模式
- **网络抖动** → 重试

```bash
# 单独跑视频测试看具体错误
python3 scripts/test_video_stream.py eth0
```

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

### 机器人不响应运动指令

- **进入了 STOPPED 状态** → 按 'q' 退出后重启
- **App 切回了非调试模式** → 重新进调试
- **高层运控被关闭** → App → 设置 → 开启 sport_mode

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

# 看 SDK 报错细节
python3 -c "
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.sport.g1_sport_client import G1SportClient
ChannelFactoryInitialize(0, 'eth0')
c = G1SportClient(); c.SetTimeout(5.0); c.Init()
print('sport ok')
c.BalanceStand()
print('balance ok')
"
```

## 已知限制 (R1 EDU)

- **R1 EDU 关节数比 G1 少**：部分动作 (如整手挥手) 不可用，但 Move / StandUp / StopMove 这些基础接口是支持的
- **R1 EDU 没有激光雷达**：跟随时主要靠视觉，无距离传感器，跟得太近可能撞
- **MediaPipe 在 Jetson Nano 上帧率有限**：~10 FPS 是常态，不影响功能但响应有延迟

如果遇到本文档没列出的问题，把：
- 完整错误堆栈
- `python3 --version`, `pip3 list | grep -E "mediapipe|opencv|unitree"`
- 机器人是否在调试模式
- 网络连接情况

贴到 issue，一起查。
