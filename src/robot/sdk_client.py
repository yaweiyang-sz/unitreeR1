"""宇树 R1 EDU 客户端封装。

依赖: unitree_sdk2py (高层运控 + 视频流)
真实硬件: R1 EDU + Jetson Nano 内置 PC + 网线直连 192.168.123.x
"""
from __future__ import annotations

import enum
import math
import random
import threading
import time
from dataclasses import dataclass
from typing import Optional

from ..logger import setup_logger

log = setup_logger("r1.sdk")


class R1Mode(enum.Enum):
    REAL = "real"
    DRY_RUN = "dry_run"


@dataclass
class VideoFrame:
    """一帧视频 + 元数据。"""
    jpeg_bytes: Optional[bytes]  # 原始 JPEG 字节
    width: int
    height: int
    timestamp: float
    source: str  # "robot" / "webcam" / "stub"


# -------------------- 真实 SDK 实现 --------------------

class _RealR1Client:
    """包装宇树官方 SDK，提供高层运控和视频流。"""

    def __init__(self, network_iface: str):
        self.iface = network_iface
        self._sport = None
        self._video = None
        self._state_client = None
        self._last_state: Optional[dict] = None
        self._lock = threading.Lock()
        self._connected = False

    def initialize(self) -> None:
        # 延迟导入，避免没装 SDK 时主程序启动失败
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.sport.g1_sport_client import G1SportClient  # type: ignore
        try:
            from unitree_sdk2py.g1.video.g1_video_client import G1VideoClient  # type: ignore
        except ImportError:
            from unitree_sdk2py.go2.video.video_client import VideoClient as G1VideoClient  # fallback

        log.info(f"连接 R1 网卡: {self.iface}")
        ChannelFactoryInitialize(0, self.iface)

        self._sport = G1SportClient()
        self._sport.SetTimeout(10.0)
        self._sport.Init()

        self._video = G1VideoClient()
        self._video.SetTimeout(3.0)
        ret = self._video.Init()
        if ret != 0:
            log.warning(f"VideoClient.Init 返回 {ret}，可能无摄像头流")

        # 状态订阅是可选的，主要用于电量/姿态
        try:
            from unitree_sdk2py.g1.robot_state.g1_robot_state_client import G1RobotStateClient  # type: ignore
            self._state_client = G1RobotStateClient()
            self._state_client.SetTimeout(3.0)
            self._state_client.Init()
        except Exception as e:  # noqa: BLE001
            log.warning(f"RobotStateClient 初始化失败: {e}")
            self._state_client = None

        # 让机器人站稳
        try:
            self._sport.BalanceStand()
        except Exception as e:  # noqa: BLE001
            log.warning(f"BalanceStand 失败（可能已在站立）: {e}")

        self._connected = True
        log.info("R1 客户端就绪")

    def stand_up(self) -> None:
        assert self._sport, "未初始化"
        self._sport.StandUp()
        log.info("→ StandUp")

    def stand_down(self) -> None:
        assert self._sport, "未初始化"
        self._sport.StandDown()
        log.info("→ StandDown")

    def balance_stand(self) -> None:
        assert self._sport, "未初始化"
        self._sport.BalanceStand()

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        """单位: vx/vy m/s, vyaw rad/s。R1 限制参考 config.yaml。"""
        if not self._sport:
            return
        with self._lock:
            self._sport.Move(vx, vy, vyaw)

    def stop_move(self) -> None:
        self.move(0.0, 0.0, 0.0)
        log.info("→ StopMove")

    def get_state(self) -> dict:
        if self._state_client:
            try:
                st = self._state_client.GetState()
                # 兼容不同 SDK 版本：可能是 dataclass 或 dict
                self._last_state = _to_dict(st) if st else {}
            except Exception as e:  # noqa: BLE001
                log.debug(f"读 state 异常: {e}")
        return self._last_state or {}

    def get_image(self) -> tuple[int, Optional[bytes]]:
        if not self._video:
            return -1, None
        try:
            code, data = self._video.GetImageSample()
            return code, bytes(data) if data else None
        except Exception as e:  # noqa: BLE001
            log.debug(f"读 image 异常: {e}")
            return -1, None

    def shutdown(self) -> None:
        try:
            if self._sport:
                self._sport.StopMove()
        except Exception:  # noqa: BLE001
            pass
        self._connected = False
        log.info("R1 客户端已关闭")


# -------------------- Dry-run 实现 --------------------

class _DryRunR1Client:
    """完全脱离硬件的模拟客户端，用于开发视觉/控制逻辑。"""

    def __init__(self, network_iface: str):
        self.iface = network_iface
        self._last_cmd = (0.0, 0.0, 0.0)
        self._state = {
            "mode": "stand",
            "battery": 87,
            "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "tick": 0,
        }
        self._stand = True
        log.info("[DRY-RUN] 已创建模拟 R1 客户端")

    def initialize(self) -> None:
        log.info("[DRY-RUN] 初始化 (no-op)")

    def stand_up(self) -> None:
        self._stand = True
        self._state["mode"] = "stand"
        log.info("[DRY-RUN] StandUp")

    def stand_down(self) -> None:
        self._stand = False
        self._state["mode"] = "down"
        log.info("[DRY-RUN] StandDown")

    def balance_stand(self) -> None:
        self._stand = True
        self._state["mode"] = "balance"

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        self._last_cmd = (vx, vy, vyaw)
        self._state["tick"] += 1
        # 给 IMU 一些扰动
        self._state["imu"]["yaw"] = (self._state["imu"]["yaw"] + vyaw * 0.05) % (2 * math.pi)
        if self._state["tick"] % 20 == 0:
            log.debug(f"[DRY-RUN] move vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f}")

    def stop_move(self) -> None:
        self.move(0.0, 0.0, 0.0)
        log.info("[DRY-RUN] StopMove")

    def get_state(self) -> dict:
        return dict(self._state)

    def get_image(self) -> tuple[int, Optional[bytes]]:
        # dry-run 模式不直接产图，由外部 webcam 提供
        return -1, None

    def shutdown(self) -> None:
        log.info("[DRY-RUN] 关闭")


# -------------------- 统一接口 --------------------

class R1Client:
    """对外统一接口：根据 mode 选择真实/模拟实现。"""

    def __init__(self, network_iface: str, mode: R1Mode = R1Mode.REAL):
        self.mode = mode
        if mode == R1Mode.DRY_RUN:
            self._impl: _RealR1Client | _DryRunR1Client = _DryRunR1Client(network_iface)
        else:
            self._impl = _RealR1Client(network_iface)

    # ---- 委托所有方法 ----
    def initialize(self) -> None:
        self._impl.initialize()

    def stand_up(self) -> None:
        self._impl.stand_up()

    def stand_down(self) -> None:
        self._impl.stand_down()

    def balance_stand(self) -> None:
        self._impl.balance_stand()

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        self._impl.move(vx, vy, vyaw)

    def stop_move(self) -> None:
        self._impl.stop_move()

    def get_state(self) -> dict:
        return self._impl.get_state()

    def get_image(self) -> tuple[int, Optional[bytes]]:
        return self._impl.get_image()

    def shutdown(self) -> None:
        self._impl.shutdown()

    # ---- 便捷别名 ----
    @property
    def is_dry_run(self) -> bool:
        return self.mode == R1Mode.DRY_RUN


# -------------------- 工具函数 --------------------

def _to_dict(obj) -> dict:
    """把 SDK 返回的对象尽量转成 dict（兼容 dataclass/namedtuple/普通对象）。"""
    if obj is None:
        return {}
    if isinstance(obj, dict):
        return obj
    if hasattr(obj, "__dict__"):
        return {k: _unwrap(v) for k, v in vars(obj).items()}
    if hasattr(obj, "_asdict"):  # namedtuple
        return {k: _unwrap(v) for k, v in obj._asdict().items()}
    return {"raw": str(obj)}


def _unwrap(v):
    if isinstance(v, (int, float, str, bool)) or v is None:
        return v
    if isinstance(v, (list, tuple)):
        return [_unwrap(x) for x in v]
    return _to_dict(v)
