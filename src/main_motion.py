"""R1 EDU 运动调试版 — 手势 → 机器人走两步 (前进/后退)

== 用途 ==
跟 src/main.py 的"安全测试版"分开, 专门调试"手势 → 运动"链路。
main.py 留作只切状态/不发速度的安全版本, 本程序加上"走两步"运动。

== 启动序列 (用户实测的合法路径) ==
    1) ZeroTorque    (FSM 0)    零力矩, 软状态 (机器人被你放倒, 当前就是它)
    2) Damp          (FSM 1)    切电机, 自由落体 — 状态机路径的必经步
    3) Stance        (FSM 4)    锁定站立 (电机锁位置, 但不能抗扰动)
    4) Start         (FSM 811)  进入 Locomotion (走跑模式)
    5) Lie2StandUp   (FSM 701)  从躺姿站起来 (必须在 Start 模式下才能成功)
    6) 速度=0                  动态稳定站立 — Start 模式 + 速度=0 才是"站稳"

    ⚠️ 状态机切换的合法路径 (用户遥控实测):
        ZeroTorque → Damp → Stance → Start → Lie2StandUp
        任何跳跃 (如 ZeroTorque → Lie2StandUp) 都会失败。
        躺下来正好相反: Lie2StandUp ↔ StandUp2Lie (FSM 702) 都需要在 Start 模式。

    ⚠️ Stance vs Start 速度=0 的区别:
        - Stance (FSM 4): 电机锁在位置, 不能抗外部推力
        - Start (FSM 811) + 速度=0: 动态平衡控制, 能抗扰动, 真正"站稳"
        所以最后必须停在 Start 模式 + 速度=0, 而不是 Stance。

== 手势 → 运动 (走两步, 速度/时长都显式写死, 不读 config) ==
    ✊  拳头 (BACKWARD) → 机器人向后走两步  (vx = -0.20 m/s × 1.2s)
    ✋  手掌 (STOP)     → 原地站住           (vx=0, vy=0, vyaw=0, 保持 Locomotion)
    ☝️  食指 (FORWARD)  → 机器人向前走两步  (vx = +0.20 m/s × 1.2s)

    ⚠️ 防连续: 一次前进/后退完成后, 必须中间出现一次 STOP 才能再做下一次前进/后退。
                防止用户"拳头 → 食指" 或 "食指 → 拳头" 连续触发。

== 显式参数 (顶部, 调这里) ==
    WALK_SPEED       = 0.20 m/s
    WALK_DURATION    = 1.2  s      (≈ 0.24m ≈ 2 步, R1 SDK 没步数指令, 用速度×时间)
    DEBOUNCE_FRAMES  = 6          (跟 main.py 一致)

== 跑法 ==
    # Jetson 真机
    python3 src/main_motion.py eth10

    # 干跑 (不连机器人, 用本机 USB 摄像头)
    python3 src/main_motion.py eth10 --dry-run

    # 不显示窗口
    python3 src/main_motion.py eth10 --no-window
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

log = setup_logger("r1.motion")


# ============================================================
#  显式参数 (不读 config, 改这里就行)
# ============================================================
# 运动参数
WALK_SPEED = 0.20         # m/s, 走两步的线速度 (慢一点安全)
WALK_DURATION = 1.2       # 秒, 走两步的持续时间 (≈ 0.24m, R1 单步 ≈ 10-12cm)

# 启动序列时序 (每步切完等一会儿, 让 R1 内部状态稳定)
ZERO_TORQUE_WAIT = 0.5    # ZeroTorque 后等 0.5s
DAMP_WAIT = 0.5           # Damp 后等 0.5s (电机被切, 等它软落完)
STANCE_WAIT = 0.5         # Stance 后等 0.5s
LOCO_WAIT = 1.0           # Start() 后等 1s (进入 locomotion)
STANDUP_WAIT = 3.0        # Lie2StandUp 后等 3s (站起动作慢, 大约 2-3s)

# 视觉参数
DEBOUNCE_FRAMES = 6
ROI = (0.20, 0.10, 0.60, 0.80)   # 中央 60% × 80% 区域
MIN_HAND_AREA_PCT = 3.0
MAX_HAND_AREA_PCT = 50.0


# ============================================================
#  应用状态机
# ============================================================
class AppState(enum.Enum):
    BOOT = "boot"          # 启动序列中
    IDLE = "idle"          # 空闲, 等手势
    WALKING = "walking"    # 正在走两步 (期间忽略手势)
    STOPPED = "stopped"    # 退出中
    ERROR = "error"


# ============================================================
#  启动序列: 6 步走完整条状态机路径
# ============================================================
def startup_sequence(robot: R1Client) -> bool:
    """按用户实测的状态序列执行:
        ZeroTorque (0) → Damp (1) → Stance (4) → Start (811) → Lie2StandUp (701) → 速度=0

    为什么必须按这个顺序? R1 FSM 不是任意两两之间都能切:
      - Stance 模式: 电机锁位置, 不能抗扰动, 不算"真站稳"
      - Start 模式 + 速度=0: 动态平衡控制, 真正抗扰动的"站稳"
      - Lie2StandUp: 动画动作, 必须在 Start 模式下才能成功执行
    """
    log.info("=" * 60)
    log.info("启动序列: ZeroTorque → Damp → Stance → Start → Lie2StandUp → 速度=0")
    log.info("=" * 60)

    # 1) ZeroTorque (FSM 0): 零力矩, 软状态 (你把机器人放倒时它就在这个状态)
    log.info("[1/6] ZeroTorque()  → FSM 0   (零力矩, 软状态)")
    robot.zero_torque()
    time.sleep(ZERO_TORQUE_WAIT)
    # 绕开后续 exit_locomotion() 的安全门 (zero_torque 后 _known_fsm=ZERO_TORQUE)
    robot.set_known_fsm(R1FsmState.STAND)

    # 2) Damp (FSM 1): 切断电机 — ⚠️ 这里 force=True 是状态机路径, 不是急停
    #    force=True 才能调, 因为 sdk_client 默认拒绝 damp() (太危险)
    log.info("[2/6] Damp(force=True)  → FSM 1   (切电机, 状态机路径的一步)")
    robot.damp(force=True)
    time.sleep(DAMP_WAIT)
    # damp() 后 _known_fsm=DAMP, 重置为 STAND 准备下一步
    robot.set_known_fsm(R1FsmState.STAND)

    # 3) Stance (FSM 4): 锁定站立 — 电机锁位置, 但不能抗扰动
    log.info("[3/6] Stance()  → FSM 4   (锁定站立, 电机锁位置, 不能抗扰动)")
    robot.balance_stand()  # 内部调 self._loco.Stance()
    time.sleep(STANCE_WAIT)

    # 4) Start (FSM 811): 进入 Locomotion (走跑模式)
    log.info("[4/6] Start()  → FSM 811 (Locomotion, 走跑模式)")
    robot.enter_locomotion()
    time.sleep(LOCO_WAIT)

    # 5) Lie2StandUp (FSM 701): 从躺姿站起 — 必须在 Start 模式下才能成功
    log.info("[5/6] Lie2StandUp()  → FSM 701 (从躺姿站起, 需在 Start 模式下)")
    robot.stand_up_from_lie()
    log.info(f"       等待 {STANDUP_WAIT}s 让站起动作完成...")
    time.sleep(STANDUP_WAIT)

    # 6) 速度=0 — Start 模式 + 速度=0 = 动态稳定站立
    log.info("[6/6] move(0, 0, 0)  →  Start 模式 + 速度=0  (动态稳定站立)")
    robot.move(0.0, 0.0, 0.0)

    log.info("=" * 60)
    log.info("✓ 启动序列完成 — 在 Locomotion (FSM 811) + 速度=0 状态, 准备接收手势")
    log.info("=" * 60)
    return True


# ============================================================
#  运动控制状态机 (防连续)
# ============================================================
class MotionController:
    """简单的运动状态机: 跟踪上次执行的动作, 实现"前后不能连续"。

    状态:
        IDLE    空闲, 接受新手势
        WALKING 正在走两步, 期间忽略所有手势 (保证两步走完)

    防连续规则:
        一次前进/后退完成后 last_motion 被设为该手势, 下次前进/后退被忽略
        直到用户做一次 STOP, last_motion 重置为 STOP, 才解除
    """

    def __init__(self, robot: R1Client):
        self.robot = robot
        self.state = AppState.IDLE
        self.last_motion: Optional[Gesture] = None
        self.walk_start_t: float = 0.0
        self.walk_dir: int = 0  # +1=前进, -1=后退

    def step(self, gesture: Gesture, now: float) -> str:
        """每帧调用一次. 返回一个状态字符串 (log 用)."""
        # ---- 走路阶段: 持续发速度, 走完回 IDLE ----
        if self.state == AppState.WALKING:
            elapsed = now - self.walk_start_t
            if elapsed < WALK_DURATION:
                self.robot.move(self.walk_dir * WALK_SPEED, 0.0, 0.0)
                return f"walking(t={elapsed:.2f}s/{WALK_DURATION}s)"
            # 走完了: 停
            self.robot.move(0.0, 0.0, 0.0)
            self.state = AppState.IDLE
            direction = "前进" if self.walk_dir > 0 else "后退"
            log.info(f"  ✓ {direction}两步走完, 速度=0, 等下一次手势")

        # ---- 空闲阶段: 处理手势 ----
        if self.state == AppState.IDLE:
            # STOP: 原地站住 (走跑模式速度=0)
            if gesture == Gesture.STOP:
                self.robot.move(0.0, 0.0, 0.0)
                if self.last_motion in (Gesture.FORWARD, Gesture.BACKWARD):
                    log.info("  [stop] 解除防连续锁, 下一动作可执行前进/后退")
                self.last_motion = Gesture.STOP
                return "stopped(vx=0)"

            # FORWARD / BACKWARD: 走两步 (带防连续)
            if gesture in (Gesture.FORWARD, Gesture.BACKWARD):
                if self.last_motion in (Gesture.FORWARD, Gesture.BACKWARD):
                    log.info(
                        f"  ⚠ {gesture.value} 被忽略 — 上一次运动是 "
                        f"{self.last_motion.value}, 必须先 STOP 过渡"
                    )
                    return f"blocked(need_stop, last={self.last_motion.value})"

                # 通过防连续, 开始走两步
                self.state = AppState.WALKING
                self.walk_start_t = now
                self.walk_dir = +1 if gesture == Gesture.FORWARD else -1
                self.last_motion = gesture
                direction = "前进" if self.walk_dir > 0 else "后退"
                log.info(
                    f"  ▶ 开始{direction}: vx={self.walk_dir * WALK_SPEED:+.2f} m/s "
                    f"× {WALK_DURATION}s"
                )
                return f"started:{gesture.value}"

        return "noop"


# ============================================================
#  视频源 (跟 main.py 一致, 简化版)
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
#  简化版 HUD (自己画几个字, 不依赖 overlays.py)
# ============================================================
def draw_hud(
    frame,
    gesture: Gesture,
    state: AppState,
    last_motion: Optional[Gesture],
    info: str,
    fingers_state: Optional[dict],
):
    h, w = frame.shape[:2]
    # 顶部黑条
    cv2.rectangle(frame, (0, 0), (w, 90), (0, 0, 0), -1)
    # 5 指状态 (二进制串: 拇食中无小)
    if fingers_state:
        fs = fingers_state
        bits = "".join("1" if fs.get(k, False) else "0" for k in ("thumb", "index", "middle", "ring", "pinky"))
    else:
        bits = "-----"
    cv2.putText(frame, f"Gesture: {gesture.value}  [{bits}]", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"State:   {state.value}  |  {info}", (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    last_str = last_motion.value if last_motion else "-"
    cv2.putText(frame, f"Last motion: {last_str}", (10, 84),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    # 底部图例
    legend = "[拳=后退两步]  [掌=停]  [食指=前进两步]  [q/ESC=退出]"
    cv2.putText(frame, legend, (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)


# ============================================================
#  CLI
# ============================================================
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="R1 EDU 运动调试 (手势 → 走两步)")
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
    motion: Optional[MotionController] = None
    cap: Optional[cv2.VideoCapture] = None

    try:
        robot.initialize()
        log.info("✓ R1 客户端就绪")

        # 启动序列
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
        motion = MotionController(robot)

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

            # 3) 运动控制
            info = motion.step(stable, now)

            # 4) 显示
            if not args.no_window:
                draw_hud(
                    frame,
                    gesture=stable,
                    state=motion.state,
                    last_motion=motion.last_motion,
                    info=info,
                    fingers_state=gr.fingers_state if gr else None,
                )
                cv2.imshow("R1 Motion Control", frame)
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
        # 退出: 速度清零, 保持 Start (FSM 811) 模式
        # 不能切 Stance (FSM 4), 原因见模块顶部注释:
        #   - Stance: 电机锁位置, R1 不能抗扰动, 一推就倒
        #   - Start + 速度=0: 动态平衡控制, 真正"站稳"
        # 所以 safe-exit = stop_move(), 让 R1 留在 walk/run 模式下原地站着。
        # log 永远打 (dry-run 也打), 让用户能确认 finally 跑过了
        try:
            if robot.is_dry_run:
                log.info("安全退出: [DRY-RUN] stop_move() 走模拟路径")
            else:
                log.info("安全退出: 速度清零, 保持 Start 模式 (FSM 811) — 动态稳定站立")
            try:
                robot.stop_move()
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
