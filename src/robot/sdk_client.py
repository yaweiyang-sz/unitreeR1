"""Unitree R1 EDU 客户端封装 — 真实 SDK 路径。

底层库: unitree_sdk2py (宇树官方 Python SDK)
- 运控: unitree_sdk2py.r1.loco.r1_loco_client.LocoClient
        R1 用 FSM 状态机驱动 (Damp / Stance / Start / Lie2StandUp / StandUp2Lie / ZeroTorque)
        没有 BalanceStand / StandUp 这种 G1/Go2 风格的方法。
- 视频: unitree_sdk2py.go2.video.video_client.VideoClient
        R1 SDK 没有自己的 video_client，复用 Go2 的 (与 R1 同一套 video_service)。
- 状态: ChannelSubscriber("rt/sportmodestate", SportModeState_)
        R1 高层只走 unitree_go 命名空间下的 SportModeState_; 主题名与 Go2 一致。

通信走 DDS (cyclonedds 0.10.x) 跨网卡，ChannelFactoryInitialize(0, iface) 指定出口网卡。
"""
from __future__ import annotations

import enum
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from ..logger import setup_logger

log = setup_logger("r1.sdk")


# ============================================================
#  枚举 / 数据类
# ============================================================

class R1Mode(enum.Enum):
    REAL = "real"
    DRY_RUN = "dry_run"


class R1FsmState(enum.Enum):
    """高层 FSM (来自 SportModeState_.mode 字段的语义化)。"""
    UNKNOWN = "unknown"
    ZERO_TORQUE = "zero_torque"     # FSM 0
    DAMP = "damp"                   # FSM 1
    STAND = "stand"                 # FSM 4
    RUNNING = "running"             # FSM 811
    LIE_TO_STAND = "lie_to_stand"   # FSM 701
    STAND_TO_LIE = "stand_to_lie"   # FSM 702


@dataclass
class VideoFrame:
    """一帧视频 + 元数据。"""
    jpeg_bytes: Optional[bytes]
    width: int
    height: int
    timestamp: float
    source: str  # "robot" / "webcam" / "stub"


# DDS 主题 (R1 与 Go2 共用, 部分新固件用 lf 前缀)
SPORT_STATE_TOPIC_DEFAULT = "rt/sportmodestate"
SPORT_STATE_TOPIC_ALT = "rt/lf/sportmodestate"


# ============================================================
#  真实 SDK 实现
# ============================================================

class _RealR1Client:
    """包装宇树 R1 LocoClient，提供高层运控 + 视频 + 状态订阅。"""

    def __init__(self, network_iface: str, sport_state_topic: str = SPORT_STATE_TOPIC_DEFAULT):
        self.iface = network_iface
        self.sport_state_topic = sport_state_topic

        # SDK 对象 (延迟到 initialize 中实例化)
        self._loco = None
        self._video = None
        self._state_sub = None

        self._lock = threading.Lock()
        self._connected = False

        # 状态缓存 (DDS 订阅回调写入)
        self._last_state: dict = {}
        self._fsm_state: R1FsmState = R1FsmState.UNKNOWN
        self._state_lock = threading.Lock()

    # ---- 初始化 ----

    def initialize(self) -> None:
        """建立 DDS 通道 + 初始化运控/视频/状态订阅。

        注意: 这里 **不** 自动进入 locomotion (FSM 811)。
        进入 locomotion 是危险动作, 必须由调用方显式调用 enter_locomotion()。
        """
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelSubscriber,
        )
        from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient
        from unitree_sdk2py.go2.video.video_client import VideoClient
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

        log.info(f"连接 R1 网卡: {self.iface}  (DDS domain=0)")
        ChannelFactoryInitialize(0, self.iface)

        # ---- 运控 ----
        self._loco = LocoClient()
        self._loco.SetTimeout(10.0)
        self._loco.Init()
        log.info("LocoClient.Init() ✓")

        # ---- 视频 ----
        self._video = VideoClient()
        self._video.SetTimeout(3.0)
        ret = self._video.Init()
        if ret != 0:
            log.warning(f"VideoClient.Init() 返回 {ret}, 可能无视频流或服务未启用")
        else:
            log.info("VideoClient.Init() ✓")

        # ---- 状态订阅 ----
        try:
            self._state_sub = ChannelSubscriber(self.sport_state_topic, SportModeState_)
            self._state_sub.Init(self._on_sport_state, 10)
            log.info(f"SportModeState 订阅 ✓  topic={self.sport_state_topic}")
        except Exception as e:  # noqa: BLE001
            # 部分新固件用 lf 前缀, 退到 alt
            if self.sport_state_topic != SPORT_STATE_TOPIC_ALT:
                log.warning(f"订阅 {self.sport_state_topic} 失败 ({e}), 尝试 {SPORT_STATE_TOPIC_ALT}")
                self.sport_state_topic = SPORT_STATE_TOPIC_ALT
                self._state_sub = ChannelSubscriber(self.sport_state_topic, SportModeState_)
                self._state_sub.Init(self._on_sport_state, 10)
                log.info(f"SportModeState 订阅 ✓  topic={self.sport_state_topic}")
            else:
                raise

        self._connected = True
        log.info("R1 客户端就绪 (尚未进入 locomotion)")

    def _on_sport_state(self, msg) -> None:
        """DDS 回调: 把 SportModeState_ 转成 dict 缓存。"""
        with self._state_lock:
            self._last_state = {
                "stamp_sec": getattr(msg.stamp, "sec", 0) if msg.stamp else 0,
                "error_code": int(msg.error_code) if msg.error_code is not None else 0,
                "mode": int(msg.mode) if msg.mode is not None else 0,
                "progress": float(msg.progress) if msg.progress is not None else 0.0,
                "gait_type": int(msg.gait_type) if msg.gait_type is not None else 0,
                "position": list(msg.position) if msg.position is not None else [0.0, 0.0, 0.0],
                "velocity": list(msg.velocity) if msg.velocity is not None else [0.0, 0.0, 0.0],
                "yaw_speed": float(msg.yaw_speed) if msg.yaw_speed is not None else 0.0,
                "body_height": float(msg.body_height) if msg.body_height is not None else 0.0,
                "foot_raise_height": float(msg.foot_raise_height) if msg.foot_raise_height is not None else 0.0,
                "imu_rpy": (
                    list(msg.imu_state.rpy) if msg.imu_state is not None and msg.imu_state.rpy is not None
                    else [0.0, 0.0, 0.0]
                ),
            }
            self._fsm_state = _mode_to_fsm(self._last_state["mode"])

    # ---- 状态机高层操作 ----

    def get_fsm_state(self) -> R1FsmState:
        with self._state_lock:
            return self._fsm_state

    def enter_locomotion(self) -> None:
        """进入 locomotion 模式 (FSM 811)。

        前置条件:
        1. 机器人已开机, App 切到调试模式
        2. 机器人当前在 Stance (FSM 4) — 用支架悬吊, 周围无障碍
        3. 周围 ≥ 2m 空间
        """
        assert self._loco, "未初始化"
        log.warning("→ Start()  进入 locomotion  (FSM 811) — 危险操作, 请确认安全")
        self._loco.Start()
        time.sleep(0.3)  # 给 FSM 切换一点时间

    def exit_locomotion(self) -> None:
        """退出 locomotion: 先停速度, 再回 stance (FSM 4)。"""
        if not self._loco:
            return
        try:
            self._loco.StopMove()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._loco.Stance()
            log.info("→ Stance()  (FSM 4) 退出 locomotion")
        except Exception as e:  # noqa: BLE001
            log.warning(f"Stance() 失败: {e}")

    def damp(self) -> None:
        """切到 Damp (FSM 1) — 切断电机, 自由落体, 急停用。"""
        assert self._loco, "未初始化"
        log.warning("→ Damp()  (FSM 1) 切断电机, 机器人会瘫软 — 急停!")
        self._loco.Damp()

    def stand_up_from_lie(self) -> None:
        """从躺姿站起 (FSM 701)。"""
        assert self._loco, "未初始化"
        self._loco.Lie2StandUp()

    def lie_down(self) -> None:
        """从站姿躺下 (FSM 702)。"""
        assert self._loco, "未初始化"
        self._loco.StandUp2Lie()

    def balance_stand(self) -> None:
        """兼容旧接口: R1 没有 BalanceStand, 用 Stance 等价。"""
        if not self._loco:
            return
        try:
            self._loco.Stance()
        except Exception:  # noqa: BLE001
            pass

    def stand_up(self) -> None:
        """兼容旧接口: R1 没有 StandUp, 用 Lie2StandUp。"""
        if not self._loco:
            return
        try:
            self._loco.Lie2StandUp()
        except Exception:  # noqa: BLE001
            pass

    def stand_down(self) -> None:
        """兼容旧接口: R1 没有 StandDown, 用 StandUp2Lie。"""
        if not self._loco:
            return
        try:
            self._loco.StandUp2Lie()
        except Exception:  # noqa: BLE001
            pass

    # ---- 运控 ----

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        """设置运动速度 (m/s, rad/s)。

        R1 LocoClient 的 Move(vx, vy, vyaw, continous_move=False) 默认 1 秒后自动停。
        我们的控制循环是周期性的 (默认 30 Hz), 所以用 continous_move=True 保持持续。
        如果你只是想"踩一下油门"再放手, 传一个 < 1.0s 的 duration, 这里不暴露 — 都在外面用速度表达。
        """
        if not self._loco:
            return
        with self._lock:
            try:
                self._loco.Move(vx, vy, vyaw, True)  # continous_move=True
            except Exception as e:  # noqa: BLE001
                log.debug(f"Move 异常: {e}")

    def stop_move(self) -> None:
        if not self._loco:
            return
        with self._lock:
            try:
                self._loco.StopMove()
            except Exception:  # noqa: BLE001
                pass

    # ---- 状态 / 视频读取 ----

    def get_state(self) -> dict:
        with self._state_lock:
            return dict(self._last_state) if self._last_state else {}

    def get_image(self) -> tuple[int, Optional[bytes]]:
        """取一帧视频。返回 (code, jpeg_bytes)。code=0 成功。"""
        if not self._video:
            return -1, None
        try:
            code, data = self._video.GetImageSample()
            return code, bytes(data) if data else None
        except Exception as e:  # noqa: BLE001
            log.debug(f"读 image 异常: {e}")
            return -1, None

    # ---- 关闭 ----

    def shutdown(self) -> None:
        log.info("R1 客户端关闭中...")
        try:
            if self._loco:
                self._loco.StopMove()
        except Exception:  # noqa: BLE001
            pass
        self._connected = False
        log.info("R1 客户端已关闭")


# ============================================================
#  Dry-run 实现
# ============================================================

class _DryRunR1Client:
    """完全脱离硬件的模拟客户端。"""

    def __init__(self, network_iface: str, sport_state_topic: str = SPORT_STATE_TOPIC_DEFAULT):
        self.iface = network_iface
        self._fsm = R1FsmState.STAND
        self._state = {
            "mode": 4,  # stance
            "position": [0.0, 0.0, 0.0],
            "velocity": [0.0, 0.0, 0.0],
            "yaw_speed": 0.0,
            "imu_rpy": [0.0, 0.0, 0.0],
            "progress": 0.0,
        }
        self._last_cmd = (0.0, 0.0, 0.0)
        log.info("[DRY-RUN] 已创建模拟 R1 客户端")

    def initialize(self) -> None:
        log.info("[DRY-RUN] 初始化 (no-op)")

    def get_fsm_state(self) -> R1FsmState:
        return self._fsm

    def enter_locomotion(self) -> None:
        self._fsm = R1FsmState.RUNNING
        log.info("[DRY-RUN] Start()  → RUNNING")

    def exit_locomotion(self) -> None:
        self._last_cmd = (0.0, 0.0, 0.0)
        self._fsm = R1FsmState.STAND
        log.info("[DRY-RUN] Stance()  → STAND")

    def damp(self) -> None:
        self._fsm = R1FsmState.DAMP
        log.info("[DRY-RUN] Damp()")

    def stand_up_from_lie(self) -> None:
        self._fsm = R1FsmState.STAND
        log.info("[DRY-RUN] Lie2StandUp()")

    def lie_down(self) -> None:
        self._fsm = R1FsmState.LIE_TO_STAND
        log.info("[DRY-RUN] StandUp2Lie()")

    def balance_stand(self) -> None:
        self._fsm = R1FsmState.STAND
        self._state["mode"] = 4

    def stand_up(self) -> None:
        self._fsm = R1FsmState.STAND

    def stand_down(self) -> None:
        self._fsm = R1FsmState.LIE_TO_STAND

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        self._last_cmd = (vx, vy, vyaw)
        # 模拟机器人位置/速度/IMU 的变化
        self._state["velocity"] = [vx, vy, 0.0]
        self._state["yaw_speed"] = vyaw
        self._state["position"] = [
            self._state["position"][0] + vx * 0.05,
            self._state["position"][1] + vy * 0.05,
            self._state["position"][2],
        ]
        self._state["imu_rpy"] = [
            self._state["imu_rpy"][0],
            self._state["imu_rpy"][1],
            (self._state["imu_rpy"][2] + vyaw * 0.05) % (2 * math.pi),
        ]

    def stop_move(self) -> None:
        self.move(0.0, 0.0, 0.0)
        log.info("[DRY-RUN] StopMove()")

    def get_state(self) -> dict:
        return dict(self._state)

    def get_image(self) -> tuple[int, Optional[bytes]]:
        return -1, None

    def shutdown(self) -> None:
        log.info("[DRY-RUN] 关闭")


# ============================================================
#  统一接口
# ============================================================

class R1Client:
    """对外统一接口: 根据 mode 选择真实/模拟。"""

    def __init__(self, network_iface: str, mode: R1Mode = R1Mode.REAL, sport_state_topic: str = SPORT_STATE_TOPIC_DEFAULT):
        self.mode = mode
        if mode == R1Mode.DRY_RUN:
            self._impl: _RealR1Client | _DryRunR1Client = _DryRunR1Client(network_iface, sport_state_topic)
        else:
            self._impl = _RealR1Client(network_iface, sport_state_topic)

    @property
    def iface(self) -> str:
        return self._impl.iface

    @property
    def is_dry_run(self) -> bool:
        return self.mode == R1Mode.DRY_RUN

    # ---- 委托所有方法 ----
    def initialize(self) -> None:
        self._impl.initialize()

    def enter_locomotion(self) -> None:
        self._impl.enter_locomotion()

    def exit_locomotion(self) -> None:
        self._impl.exit_locomotion()

    def damp(self) -> None:
        self._impl.damp()

    def stand_up_from_lie(self) -> None:
        self._impl.stand_up_from_lie()

    def lie_down(self) -> None:
        self._impl.lie_down()

    def balance_stand(self) -> None:
        self._impl.balance_stand()

    def stand_up(self) -> None:
        self._impl.stand_up()

    def stand_down(self) -> None:
        self._impl.stand_down()

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        self._impl.move(vx, vy, vyaw)

    def stop_move(self) -> None:
        self._impl.stop_move()

    def get_state(self) -> dict:
        return self._impl.get_state()

    def get_fsm_state(self) -> R1FsmState:
        return self._impl.get_fsm_state()

    def get_image(self) -> tuple[int, Optional[bytes]]:
        return self._impl.get_image()

    def shutdown(self) -> None:
        self._impl.shutdown()


# ============================================================
#  工具函数
# ============================================================

def _mode_to_fsm(mode: int) -> R1FsmState:
    """把 SportModeState.mode 数值映射到可读 FSM 名称。

    R1 官方 (LocoClient) 用的 FSM id:
        0   zero_torque
        1   damp
        4   stance
        701 lie_to_stand
        702 stand_to_lie
        811 running (locomotion)
    """
    return {
        0: R1FsmState.ZERO_TORQUE,
        1: R1FsmState.DAMP,
        4: R1FsmState.STAND,
        701: R1FsmState.LIE_TO_STAND,
        702: R1FsmState.STAND_TO_LIE,
        811: R1FsmState.RUNNING,
    }.get(mode, R1FsmState.UNKNOWN)
