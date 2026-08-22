"""R1 EDU 挥手交互版 — 手势 → 手臂动作

== 用途 ==
跟 src/main.py / src/main_motion.py 分开, 专门调试"手势 → 手臂交互"链路。
- main.py        : 安全测试版, 只切 FSM, 不发运动
- main_motion.py : 运动版, 手势控制走两步 (vx 方向)
- main_wave.py   : 本程序, 手势控制挥右手 / 伸手臂 (arm task)

== 启动序列 (跟 main_motion.py 相同, 用户实测路径) ==
    ZeroTorque (0) → Damp (1) → Stance (4) → Start (811) → Lie2StandUp (701) → 速度=0
    最后停在 Start 模式 + 速度=0 = 动态稳定站立, 这是做 arm 任务的前提 (Stance 下不稳)

== 手势 → 手臂任务 ==
    ✋  手掌 (STOP)     → robot.WaveHand(turn_flag=False)  挥右手
    ✊  拳头 (BACKWARD) → robot.ShakeHand(stage=0)         伸手握手 (代替拥抱)
    ☝️  食指 (FORWARD)  → no-op (本程序不响应)
    ✋←/→ LEFT/RIGHT  → no-op (本程序不响应)

    ⚠️ 触发规则:
        手势从 UNKNOWN 变成 STOP 或 BACKWARD 时, 触发一次 arm 任务。
        同一手势保持稳定期间不重复触发, 避免一直挥手。
        手势变回 UNKNOWN 后才解除锁定, 下次同手势可再次触发。
        STOP ↔ BACKWARD 直接切换也会触发 (各算各的)。

== 显式参数 (顶部, 调这里) ==
    DEBOUNCE_FRAMES       = 6
    WAVE_TURN_FLAG        = False    # WaveHand 是否带转身
    SHAKE_STAGE           = 0        # ShakeHand 阶段 (0/1/-1 切换)

== SDK 限制说明 ==
    R1 Python SDK (unitree_sdk2py.r1.loco.LocoClient) 只暴露了两个高层 arm 任务:
        - WaveHand  (task_id 0/1, 挥手)
        - ShakeHand (task_id 2/3, 握手)
    没有 hug/embrace 任务, 也没有"双臂做自定义姿态"的现成 API。
    本程序用 ShakeHand 代替"拥抱"语义; 真要做拥抱要走底层 joint 控制 (r1_arm_sdk_dds_example),
    那是另一条路, 跟本程序无关。

== 跑法 ==
    # Jetson 真机
    python3 src/main_wave.py eth10

    # 干跑
    python3 src/main_wave.py eth10 --dry-run

    # 不显示窗口
    python3 src/main_wave.py eth10 --no-window
"""
from __future__ import annotations

import argparse
import enum
import sys
import time
from pathlib import Path
from typing import Optional

# 让脚本既可作为模块跑, 也可直接跑
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from src.logger import setup_logger
from src.robot.sdk_client import R1Client, R1FsmState, R1Mode
from src.vision.hand_gesture import HandGestureDetector, GestureDebouncer, Gesture

log = setup_logger("r1.wave")


# ============================================================
#  显式参数 (不读 config, 改这里就行)
# ============================================================
# Arm 任务参数
WAVE_TURN_FLAG = False    # WaveHand(turn_flag) — False=单纯挥手, True=挥手带转身
SHAKE_STAGE = 0           # ShakeHand(stage)    — 0/1/-1 切换

# 启动序列时序 (跟 main_motion.py 一致)
ZERO_TORQUE_WAIT = 0.5
DAMP_WAIT = 0.5
STANCE_WAIT = 0.5
LOCO_WAIT = 1.0
STANDUP_WAIT = 3.0

# 视觉参数
DEBOUNCE_FRAMES = 6
ROI = (0.20, 0.10, 0.60, 0.80)   # 中央 60% × 80% 区域
MIN_HAND_AREA_PCT = 3.0
MAX_HAND_AREA_PCT = 50.0

# 任务间最小间隔 (秒), 避免连续调用 SDK 太快
ARM_TASK_COOLDOWN = 0.3


# ============================================================
#  应用状态机
# ============================================================
class AppState(enum.Enum):
    BOOT = "boot"
    IDLE = "idle"          # 空闲, 等手势
    EXECUTING = "executing"  # 正在执行 arm 任务 (冷却中, 忽略新触发)
    STOPPED = "stopped"
    ERROR = "error"


# ============================================================
#  启动序列: 跟 main_motion.py 一致
# ============================================================
def startup_sequence(robot: R1Client) -> bool:
    """ZeroTorque → Damp → Stance → Start → Lie2StandUp → 速度=0

    必须停在 Start 模式 + 速度=0, 因为:
    - Stance (锁定站立) 不能抗扰动, 做 arm 任务可能摔
    - Start + 速度=0 是动态稳定站立, 抗扰动
    """
    log.info("=" * 60)
    log.info("启动序列: ZeroTorque → Damp → Stance → Start → Lie2StandUp → 速度=0")
    log.info("=" * 60)

    log.info("[1/6] ZeroTorque()  → FSM 0   (零力矩)")
    robot.zero_torque()
    time.sleep(ZERO_TORQUE_WAIT)
    robot.set_known_fsm(R1FsmState.STAND)  # 绕开后续 exit_locomotion 安全门

    log.info("[2/6] Damp(force=True)  → FSM 1   (切电机, 状态机路径)")
    robot.damp(force=True)
    time.sleep(DAMP_WAIT)
    robot.set_known_fsm(R1FsmState.STAND)

    log.info("[3/6] Stance()  → FSM 4   (锁定站立)")
    robot.balance_stand()  # 内部调 self._loco.Stance()
    time.sleep(STANCE_WAIT)

    log.info("[4/6] Start()  → FSM 811 (Locomotion, 走跑模式)")
    robot.enter_locomotion()
    time.sleep(LOCO_WAIT)

    log.info("[5/6] Lie2StandUp()  → FSM 701 (从躺姿站起)")
    robot.stand_up_from_lie()
    log.info(f"       等待 {STANDUP_WAIT}s 让站起动作完成...")
    time.sleep(STANDUP_WAIT)

    log.info("[6/6] move(0, 0, 0)  →  Start 模式 + 速度=0  (动态稳定站立)")
    robot.move(0.0, 0.0, 0.0)

    log.info("=" * 60)
    log.info("✓ 启动序列完成 — 在 Locomotion (FSM 811) + 速度=0, 准备接收手势")
    log.info("=" * 60)
    return True


# ============================================================
#  挥手/握手控制器
# ============================================================
class ArmTaskController:
    """把手势映射到 R1 的 arm 任务 (WaveHand / ShakeHand)。

    触发规则:
        手势从 UNKNOWN 变成 STOP/BACKWARD 时, 触发一次 arm 任务。
        同一手势保持稳定期间不重复触发 (用 triggered[] 标记)。
        手势变回 UNKNOWN 后解除锁定, 下次同手势可再次触发。
        STOP ↔ BACKWARD 直接切换也会触发, 因为各算各的 triggered 标志。
    """

    def __init__(self, robot: R1Client):
        self.robot = robot
        self.triggered = {
            Gesture.STOP: False,
            Gesture.BACKWARD: False,
        }
        self.last_gesture: Gesture = Gesture.UNKNOWN
        self.last_task_t: float = 0.0
        self.last_action: str = "-"

    def step(self, gesture: Gesture, now: float) -> str:
        """每帧调用, 处理手势 → arm 任务。
        返回一个状态描述 (log 用).
        """
        # 1) 手势回到 UNKNOWN, 解除触发锁定
        if gesture == Gesture.UNKNOWN:
            self.triggered[Gesture.STOP] = False
            self.triggered[Gesture.BACKWARD] = False
            return "idle(waiting)"

        # 2) 冷却中, 不触发新任务 (避免连续挥手/握手的 SDK 调用挤在一起)
        if (now - self.last_task_t) < ARM_TASK_COOLDOWN:
            return f"cooldown({ARM_TASK_COOLDOWN - (now - self.last_task_t):.2f}s)"

        # 3) 手势没变, 已经被触发过, 啥也不做
        if self.triggered.get(gesture, False):
            return f"already_triggered:{gesture.value}"

        # 4) 触发对应 arm 任务
        if gesture == Gesture.STOP:
            log.info("  ▶ 手势 STOP → WaveHand (挥右手)")
            self.robot.wave_hand(turn_flag=WAVE_TURN_FLAG)
            self.last_action = "WaveHand"
            self.triggered[Gesture.STOP] = True
            self.last_task_t = now
            return "executed:WaveHand"

        if gesture == Gesture.BACKWARD:
            log.info("  ▶ 手势 BACKWARD → ShakeHand (伸手握手 / 代替拥抱)")
            self.robot.shake_hand(stage=SHAKE_STAGE)
            self.last_action = "ShakeHand"
            self.triggered[Gesture.BACKWARD] = True
            self.last_task_t = now
            return "executed:ShakeHand"

        # 5) 其他手势 (FORWARD/LEFT/RIGHT) 本程序不响应
        return f"ignored:{gesture.value}"


# ============================================================
#  视频源 (跟 main_motion.py 一致)
# ============================================================
def open_video_source(robot: R1Client, webcam_index: int) -> tuple[str, callable, Optional[cv2.VideoCapture]]:
    if not robot.is_dry_run:
        code, data = robot.get_image()
        if code == 0 and data:
            log.info("视频源: 机器人前置摄像头")
            return "robot", lambda: robot.get_image(), None
    cap = cv2.VideoCapture(webcam_index)
    if cap.isOpened():
        log.info(f"视频源: 本机 USB 摄像头 index={webcam_index}")
        return "webcam", lambda: _read_webcam(cap), cap
    cap.release()
    log.error("没有可用的视频源")
    return "none", lambda: (-1, None), None


def _read_webcam(cap: cv2.VideoCapture) -> tuple[int, Optional[bytes]]:
    ok, frame = cap.read()
    if not ok:
        return -1, None
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return -1, None
    return 0, buf.tobytes()


# ============================================================
#  简化版 HUD
# ============================================================
def draw_hud(
    frame,
    gesture: Gesture,
    info: str,
    last_action: str,
    fingers_state: Optional[dict],
):
    h, w = frame.shape[:2]
    # 顶部黑条
    cv2.rectangle(frame, (0, 0), (w, 90), (0, 0, 0), -1)
    if fingers_state:
        fs = fingers_state
        bits = "".join("1" if fs.get(k, False) else "0" for k in ("thumb", "index", "middle", "ring", "pinky"))
    else:
        bits = "-----"
    cv2.putText(frame, f"Gesture: {gesture.value}  [{bits}]", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"Info: {info}", (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(frame, f"Last action: {last_action}", (10, 84),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    # 底部图例
    legend = "[掌=挥右手]  [拳=握手(代替拥抱)]  [q/ESC=退]"
    cv2.putText(frame, legend, (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)


# ============================================================
#  CLI
# ============================================================
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="R1 EDU 挥手交互 (手势 → arm 任务)")
    p.add_argument("iface", nargs="?", default="eth10", help="接 R1 的网卡 (如 eth10)")
    p.add_argument("--dry-run", action="store_true", help="不连机器人, 干跑视觉")
    p.add_argument("--no-window", action="store_true", help="不显示 OpenCV 窗口")
    p.add_argument("--webcam", type=int, default=0, help="dry-run 模式下用的 USB 摄像头 index")
    return p.parse_args()


# ============================================================
#  主循环
# ============================================================
def main() -> int:
    args = parse_args()
    mode = R1Mode.DRY_RUN if args.dry_run else R1Mode.REAL
    robot = R1Client(args.iface, mode=mode, enable_state_subscription=False)

    state = AppState.BOOT
    gesture_det: Optional[HandGestureDetector] = None
    debouncer: Optional[GestureDebouncer] = None
    arm: Optional[ArmTaskController] = None
    cap: Optional[cv2.VideoCapture] = None

    try:
        robot.initialize()
        log.info("✓ R1 客户端就绪")

        if not startup_sequence(robot):
            log.error("启动序列失败, 退出")
            state = AppState.ERROR
            return 1

        # 视觉模块
        gesture_det = HandGestureDetector(
            roi_x_pct=ROI[0], roi_y_pct=ROI[1],
            roi_w_pct=ROI[2], roi_h_pct=ROI[3],
            min_hand_area_pct=MIN_HAND_AREA_PCT,
            max_hand_area_pct=MAX_HAND_AREA_PCT,
        )
        debouncer = GestureDebouncer(debounce_frames=DEBOUNCE_FRAMES)
        arm = ArmTaskController(robot)

        # 视频源
        src_name, src_read, cap = open_video_source(robot, args.webcam)
        if src_name == "none":
            state = AppState.ERROR
            return 1

        log.info("进入主循环 — 按 'q' 或 ESC 退出")
        state = AppState.IDLE

        while True:
            now = time.monotonic()

            # 1) 拿一帧
            code, jpeg = src_read()
            if code != 0 or jpeg is None:
                time.sleep(0.02)
                continue
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # 2) 手势识别 + 去抖
            gr = gesture_det.detect(frame)
            stable = debouncer.update(gr.gesture if gr else Gesture.UNKNOWN)

            # 3) arm 任务
            info = arm.step(stable, now)

            # 4) 显示
            if not args.no_window:
                draw_hud(
                    frame,
                    gesture=stable,
                    info=info,
                    last_action=arm.last_action,
                    fingers_state=gr.fingers_state if gr else None,
                )
                cv2.imshow("R1 Wave Interaction", frame)
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q') or key == 27:  # q or ESC
                    log.info("用户按键退出")
                    state = AppState.STOPPED
                    break

    except KeyboardInterrupt:
        log.info("Ctrl-C 退出")
        state = AppState.STOPPED
    except Exception as e:  # noqa: BLE001
        log.exception(f"主循环异常: {e}")
        state = AppState.ERROR
        return 2
    finally:
        # 退出: 停速度 + 退回 Stance
        try:
            if not robot.is_dry_run:
                log.info("退出中: 停速度 + 退回 Stance (FSM 4)")
                try:
                    robot.stop_move()
                except Exception:  # noqa: BLE001
                    pass
                try:
                    robot.exit_locomotion()
                except Exception:  # noqa: BLE001
                    pass
        except Exception:  # noqa: BLE001
            pass
        try:
            robot.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            if gesture_det:
                gesture_det.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            if cap:
                cap.release()
        except Exception:  # noqa: BLE001
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
