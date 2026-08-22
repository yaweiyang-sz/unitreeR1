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


def _env_flag(name: str, default: bool = False) -> bool:
    """读环境变量当 bool flag。'1'/'true'/'yes'/'on' (大小写不敏感) 视为 True。"""
    import os
    v = os.environ.get(name, "").strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "on")


# ============================================================
#  真实 SDK 实现
# ============================================================

class _RealR1Client:
    """包装宇树 R1 LocoClient，提供高层运控 + 视频 + 状态订阅。"""

    def __init__(
        self,
        network_iface: str,
        sport_state_topic: str = SPORT_STATE_TOPIC_DEFAULT,
        enable_state_subscription: bool = False,
    ):
        self.iface = network_iface
        self.sport_state_topic = sport_state_topic
        # ⚠️ 默认关: ChannelSubscriber 跟 LocoClient RPC 抢 cyclonedds 资源,
        # 某些 R1 固件上会让 shutdown 段错误。要富信息 (IMU/位置/速度) 时手动开。
        self.enable_state_subscription = enable_state_subscription

        # SDK 对象 (延迟到 initialize 中实例化)
        self._loco = None
        self._video = None
        self._state_sub = None

        self._lock = threading.Lock()
        self._connected = False
        # 必须在 enter_locomotion() 之后, move() 才会真发到 R1
        # 避免在 Stance 下误发 SetVelocity 让 R1 固件进异常状态
        self._in_locomotion: bool = False
        # 我们本地跟踪的 R1 FSM 状态 (用于安全门, 不真去问 R1)
        # 默认 UNKNOWN, 每次调 enter_locomotion/exit_locomotion/damp/ZeroTorque 都会更新
        self._known_fsm: R1FsmState = R1FsmState.UNKNOWN
        # poll_fsm 默认关闭 — 防止 GetFsmId RPC 把 R1 固件弄坏
        # 想开就调 enable_poll_fsm() 或设环境变量 R1_ENABLE_POLL_FSM=1
        self._enable_poll_fsm: bool = _env_flag("R1_ENABLE_POLL_FSM", False)

        # 状态缓存 (DDS 订阅回调写入)
        self._last_state: dict = {}
        self._fsm_state: R1FsmState = R1FsmState.UNKNOWN
        self._state_lock = threading.Lock()
        # GetFsmId RPC 只调一次 (避免重复触发潜在 segfault)
        self._fsm_rpc_tried: bool = False

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
        # Go2 VideoClient.Init() 不显式 return, 成功时为 None, 失败时抛异常
        # 这里 None/0 都视为成功, 只在真返回错误码时告警
        if ret is not None and ret != 0:
            log.warning(f"VideoClient.Init() 返回 {ret}, 可能无视频流或服务未启用")
        else:
            log.info("VideoClient.Init() ✓")

        # ---- 状态: 主路径 LocoClient RPC GetFsmId() (必拿得到, 已通过 Init 验证通道)
        #      副路径 ChannelSubscriber(SportModeState_) 仅用于富信息 (IMU/位置/速度);
        #      主题名找不到就静默降级, 不阻塞初始化 ----
        # 注意: 这里不主动 GetFsmId() — 部分 R1 固件对 GET_FSM_ID 的参数格式敏感,
        # 乱发会触发 cyclonedds C 层 segfault。改成由调用方显式 poll_fsm() 触发。
        # DDS 订阅 (best-effort, 默认关 — 见 __init__ enable_state_subscription)
        if self.enable_state_subscription:
            try:
                self._state_sub = ChannelSubscriber(self.sport_state_topic, SportModeState_)
                self._state_sub.Init(self._on_sport_state, 10)
                log.info(f"SportModeState 订阅 ✓  topic={self.sport_state_topic}")
            except Exception as e:  # noqa: BLE001
                if self.sport_state_topic != SPORT_STATE_TOPIC_ALT:
                    log.warning(f"订阅 {self.sport_state_topic} 失败 ({e}), 尝试 {SPORT_STATE_TOPIC_ALT}")
                    try:
                        self.sport_state_topic = SPORT_STATE_TOPIC_ALT
                        self._state_sub = ChannelSubscriber(self.sport_state_topic, SportModeState_)
                        self._state_sub.Init(self._on_sport_state, 10)
                        log.info(f"SportModeState 订阅 ✓  topic={self.sport_state_topic}")
                    except Exception as e2:  # noqa: BLE001
                        log.warning(
                            f"alt 主题 {SPORT_STATE_TOPIC_ALT} 也订阅失败 ({e2}), "
                            f"SportModeState 订阅降级 — 富信息 (IMU/位置) 暂不可用, FSM 仍可读"
                        )
                        self._state_sub = None
                else:
                    log.warning(
                        f"SportModeState 订阅降级 — 富信息 (IMU/位置) 暂不可用, FSM 仍可读: {e}"
                    )
                    self._state_sub = None
        else:
            log.info("SportModeState 订阅默认关闭 (需 enable_state_subscription=True 才建)")

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
            # DDS 拿到的 mode 跟 FSM id 同空间 (4=stance, 811=running), 一并缓存
            self._fsm_state = _mode_to_fsm(self._last_state["mode"])

    def _refresh_fsm_via_rpc(self) -> None:
        """通过 LocoClient.GetFsmId() RPC 拉一次当前 FSM id (子进程隔离)。

        这是**主路径**, 通道已通过 LocoClient.Init() 验证。
        返回值形如 {"data": <fsm_id>} (与 SetFsmId 互为反向)。

        ⚠️ 必须在子进程里跑: 部分 R1 固件对 GET_FSM_ID 的参数格式敏感, 参数不对
        会让 R1 端返回乱码, cyclonedds C 层解析时 segfault, Python try/except 拦不住。
        子进程隔离后, 即便 segfault 也只是这个探测子进程死掉, 主程序不受影响。
        """
        if not self._loco:
            return
        if self._fsm_rpc_tried:
            return
        self._fsm_rpc_tried = True
        try:
            fsm_id = _get_fsm_id_in_subprocess(timeout=2.0)
            if fsm_id is not None:
                with self._state_lock:
                    self._fsm_state = _mode_to_fsm(int(fsm_id))
                    self._last_state["mode"] = int(fsm_id)
        except Exception as e:  # noqa: BLE001
            log.debug(f"GetFsmId RPC 探测失败: {e}")

    # ---- 状态机高层操作 ----

    def get_fsm_state(self) -> R1FsmState:
        with self._state_lock:
            return self._fsm_state

    def poll_fsm(self) -> R1FsmState:
        """主动通过 RPC 拉一次 FSM (DDS 订阅没建好或主程序需要确认时用)。

        ⚠️ **默认关闭** — 历史经验: 给 Stance 下的 R1 发 GetFsmId RPC 会让 R1 固件
        进入异常状态, 后续所有 RPC 都段错误。需要时由调用方显式打开 (env var
        R1_ENABLE_POLL_FSM=1 或 调 enable_poll_fsm())。底层走子进程隔离,
        即使 segfault 也不会拖死主进程, 但 R1 端状态还是会被搞坏。
        """
        if not self._enable_poll_fsm:
            log.debug("poll_fsm 跳过 (未启用, 设环境变量 R1_ENABLE_POLL_FSM=1 打开)")
            return self.get_fsm_state()
        self._refresh_fsm_via_rpc()
        return self.get_fsm_state()

    def enable_poll_fsm(self) -> None:
        """显式启用 poll_fsm() 主动 RPC 拉 FSM。默认关闭, 防止误调让 R1 端固件异常。"""
        self._enable_poll_fsm = True
        log.info("poll_fsm 已启用 (会真给 R1 发 GetFsmId RPC, 仅在 R1 已知健康时用)")

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
        self._in_locomotion = True
        self._known_fsm = R1FsmState.RUNNING
        time.sleep(0.3)  # 给 FSM 切换一点时间

    def exit_locomotion(self) -> None:
        """退出 locomotion: 先停速度, 再回 stance (FSM 4)。

        ⚠️ 安全门: 如果我们本地跟踪到 R1 在 ZERO_TORQUE 模式 (FSM 0), 拒绝发
        Stance() — ZERO_TORQUE 状态下 R1 电机零力矩, 软件方式不能直接切到
        Stance (R1 端会进未定义状态), 只能 power cycle 恢复。
        """
        if not self._loco:
            return
        # 安全门: ZeroTorque 状态拒绝切 Stance
        if self._known_fsm == R1FsmState.ZERO_TORQUE:
            log.error(
                "  !!!!! exit_locomotion() 拒绝执行 — R1 当前在 ZERO_TORQUE (FSM 0) !!!!!\n"
                "  ZERO_TORQUE 状态下 R1 电机零力矩, 软件无法切到 Stance, 必须 power cycle R1 恢复"
            )
            return
        try:
            # 只有在 locomotion 状态才发 StopMove, 避免 Stance 下误发
            if self._in_locomotion:
                self._loco.StopMove()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._loco.Stance()
            self._known_fsm = R1FsmState.STAND
            log.info("→ Stance()  (FSM 4) 退出 locomotion")
        except Exception as e:  # noqa: BLE001
            log.warning(f"Stance() 失败: {e}")
        self._in_locomotion = False

    def damp(self, force: bool = False) -> None:
        """切到 Damp (FSM 1) — 切断电机, 自由落体, **只有急停才用**。

        ⚠️ 极度危险: 历史上 R1 固件在 Damp 状态下 DDS 响应会让 cyclonedds
        segfault, 且没有软件方式能让 R1 恢复, 必须物理 power cycle。
        默认拒绝调用, 必须显式 force=True 才能执行。
        """
        assert self._loco, "未初始化"
        if not force:
            log.error(
                "  !!!!! Damp() 被拒绝 — 极度危险操作, 必须 power cycle R1 才能恢复 !!!!!\n"
                "  如果是真的急停 (机器人要撞墙了), 调 damp(force=True)"
            )
            return
        log.warning("→ Damp()  (FSM 1) 切断电机 — R1 会瘫软且只能 power cycle 恢复")
        self._loco.Damp()
        self._in_locomotion = False
        self._known_fsm = R1FsmState.DAMP

    def zero_torque(self) -> None:
        """切到 ZeroTorque (FSM 0) — 电机零力矩, 自由落体。

        ⚠️ 极度危险, 跟 damp 类似。R1 在 ZeroTorque 状态下软件不能切到 Stance,
        必须 power cycle 恢复。默认还是允许调 (因为 ZeroTorque 是 R1 官方支持的
        状态, 而 Damp 是"切电机"那种), 但调用后会锁住后续的 Stance 操作。
        """
        assert self._loco, "未初始化"
        log.warning("→ ZeroTorque()  (FSM 0) 电机零力矩 — 后续 Stance 调用会被拒绝")
        self._loco.ZeroTorque()
        self._in_locomotion = False
        self._known_fsm = R1FsmState.ZERO_TORQUE

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

        ⚠️ 安全门: 没调 enter_locomotion() 之前, 这个方法是 **no-op**。
        原因是 R1 在 Stance (FSM 4) 下收到 SetVelocity 是未定义行为, 历史上会把
        R1 固件弄成半坏状态, 后续所有 RPC 都段错误。所以加一个 flag 卡住。
        """
        if not self._loco:
            return
        if not self._in_locomotion:
            # 不真发, 但要 log 出来 (debug 级别, 避免循环里刷屏)
            log.debug(
                f"move({vx:.2f},{vy:.2f},{vyaw:.2f}) ignored — 未在 locomotion, "
                f"需先调 enter_locomotion()"
            )
            return
        with self._lock:
            try:
                self._loco.Move(vx, vy, vyaw, True)  # continous_move=True
            except Exception as e:  # noqa: BLE001
                log.debug(f"Move 异常: {e}")

    def stop_move(self) -> None:
        """清零速度。仅在 locomotion 状态生效 (见 move() 的安全门)。"""
        if not self._loco:
            return
        if not self._in_locomotion:
            log.debug("stop_move ignored — 未在 locomotion")
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
        """清理客户端状态。**不**主动给 R1 发任何指令 (避免关进程时 segfault)。

        想要停机器人, 显式调:
            - stop_move()         # 速度清零 (FSM 811 下生效)
            - exit_locomotion()   # Stance (FSM 4) 完整退出
            - damp()              # 急停, 切断电机

        还会调用 ChannelFactory.Finalize() (如果 SDK 暴露了的话) — 把 DDS
        participant 干净地从网络下线, 避免下一次脚本创建新 participant 时
        拿到陈旧 participant ID 引起 R1 端固件异常。
        """
        log.info("R1 客户端关闭中... (不发指令, 只清客户端对象 + DDS Finalize)")
        self._connected = False
        # 尝试 Finalize DDS factory, 让 participant 干净下线
        try:
            from unitree_sdk2py.core.channel import ChannelFactory  # type: ignore
            if hasattr(ChannelFactory, "Finalize"):
                ChannelFactory.Finalize()
                log.debug("ChannelFactory.Finalize() ✓")
        except Exception as e:  # noqa: BLE001
            log.debug(f"ChannelFactory.Finalize 不可用或失败: {e}")
        log.info("R1 客户端已关闭")


# ============================================================
#  Dry-run 实现
# ============================================================

class _DryRunR1Client:
    """完全脱离硬件的模拟客户端。"""

    def __init__(self, network_iface: str, sport_state_topic: str = SPORT_STATE_TOPIC_DEFAULT):
        self.iface = network_iface
        self._fsm = R1FsmState.STAND
        self._known_fsm = R1FsmState.STAND  # dry-run 假设 R1 一直在 Stance, 由方法更新
        self._in_locomotion = False  # 与 _RealR1Client 一致
        self._enable_poll_fsm: bool = _env_flag("R1_ENABLE_POLL_FSM", False)
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

    def poll_fsm(self) -> R1FsmState:
        if not self._enable_poll_fsm:
            return self._fsm
        return self._fsm

    def enable_poll_fsm(self) -> None:
        self._enable_poll_fsm = True

    def enter_locomotion(self) -> None:
        self._fsm = R1FsmState.RUNNING
        self._known_fsm = R1FsmState.RUNNING
        self._in_locomotion = True
        log.info("[DRY-RUN] Start()  → RUNNING")

    def exit_locomotion(self) -> None:
        # 安全门: 跟 _RealR1Client 一样, ZeroTorque 状态拒绝切 Stance
        if self._known_fsm == R1FsmState.ZERO_TORQUE:
            log.error(
                "  !!!!! [DRY-RUN] exit_locomotion() 拒绝 — 当前 ZeroTorque !!!!!"
            )
            return
        self._last_cmd = (0.0, 0.0, 0.0)
        self._fsm = R1FsmState.STAND
        self._known_fsm = R1FsmState.STAND
        self._in_locomotion = False
        # 清掉 stale velocity, 避免退出后 UI 看着还在动
        self._state["velocity"] = [0.0, 0.0, 0.0]
        self._state["yaw_speed"] = 0.0
        log.info("[DRY-RUN] Stance()  → STAND")

    def damp(self, force: bool = False) -> None:
        if not force:
            log.error(
                "  !!!!! [DRY-RUN] Damp() 被拒绝 — 必须 force=True !!!!!"
            )
            return
        self._fsm = R1FsmState.DAMP
        self._known_fsm = R1FsmState.DAMP
        self._in_locomotion = False
        log.info("[DRY-RUN] Damp() (force=True)")

    def zero_torque(self) -> None:
        self._fsm = R1FsmState.ZERO_TORQUE
        self._known_fsm = R1FsmState.ZERO_TORQUE
        self._in_locomotion = False
        log.info("[DRY-RUN] ZeroTorque() — 后续 Stance 调用会被拒绝")

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
        if not self._in_locomotion:
            log.debug(f"[DRY-RUN] move({vx:.2f},{vy:.2f},{vyaw:.2f}) ignored — 未在 locomotion")
            return
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

    def __init__(
        self,
        network_iface: str,
        mode: R1Mode = R1Mode.REAL,
        sport_state_topic: str = SPORT_STATE_TOPIC_DEFAULT,
        enable_state_subscription: bool = False,
    ):
        self.mode = mode
        if mode == R1Mode.DRY_RUN:
            self._impl: _RealR1Client | _DryRunR1Client = _DryRunR1Client(network_iface, sport_state_topic)
        else:
            self._impl = _RealR1Client(network_iface, sport_state_topic, enable_state_subscription)

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

    def damp(self, force: bool = False) -> None:
        self._impl.damp(force=force)

    def zero_torque(self) -> None:
        self._impl.zero_torque()

    def set_known_fsm(self, fsm: R1FsmState) -> None:
        """外部 (比如订阅 SportModeState 的回调) 可以告诉 R1Client 当前 FSM,
        让 exit_locomotion 的安全门基于真实状态做判断, 不只是本地跟踪。
        """
        self._impl._known_fsm = fsm
        # 顺便: 已知 FSM 也能更新 _in_locomotion (RUNNING 时 = True, 其他 = False)
        self._impl._in_locomotion = (fsm == R1FsmState.RUNNING)

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

    def poll_fsm(self) -> R1FsmState:
        return self._impl.poll_fsm()

    def enable_poll_fsm(self) -> None:
        """显式启用 poll_fsm() 的 GetFsmId RPC。默认关闭。"""
        self._impl.enable_poll_fsm()

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


def _get_fsm_id_in_subprocess(timeout: float = 2.0) -> Optional[int]:
    """在子进程里调 LocoClient.GetFsmId()。

    用 multiprocessing 隔离 cyclonedds 潜在的 segfault, 父进程最多等 timeout 秒。
    返回 fsm_id (int) 或 None (失败/超时/segfault)。

    之所以要子进程: cyclonedds 的 C 代码 segfault 时, Python 的 try/except 拦不住,
    整个进程直接挂掉, 主程序也跟着死。子进程隔离后, 即便崩了, 父进程 timeout
    后还能继续走 (FSM 不可用而已, Move 指令照发)。
    """
    import json as _json
    import multiprocessing as _mp

    q: _mp.Queue = _mp.Queue(maxsize=1)
    p = _mp.Process(target=_get_fsm_id_worker, args=(q,), daemon=True)
    p.start()
    p.join(timeout=timeout)
    if p.is_alive():
        p.terminate()
        p.join(0.5)
        if p.is_alive():
            p.kill()
        return None
    if not q.empty():
        v = q.get_nowait()
        return v if v >= 0 else None
    return None


def _get_fsm_id_worker(q) -> None:
    """子进程 worker: 跑 LocoClient.GetFsmId() 并把 fsm_id 塞进 Queue。

    必须是**模块顶层函数**才能在 Windows spawn multiprocessing 下 pickle。
    """
    try:
        from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient
        c = LocoClient()
        c.SetTimeout(2.0)
        c.Init()
        code, data = c._Call(7001, '{"data":0}')  # ROBOT_API_ID_LOCO_GET_FSM_ID
        if code == 0 and data:
            import json as _json
            d = _json.loads(data) if isinstance(data, (str, bytes)) else data
            fsm_id = d.get("data") if isinstance(d, dict) else None
            q.put(int(fsm_id) if fsm_id is not None else -1)
        else:
            q.put(-1)
    except Exception:
        q.put(-1)
