"""R1 EDU 运动调试版 — 手势 → 机器人走两步 (前进/后退)

== 用途 ==
跟 src/main.py 的"安全测试版"分开, 专门调试"手势 → 运动"链路。
main.py 留作只切状态/不发速度的安全版本, 本程序加上"走两步"运动。

== 启动序列 (Damp 起手 — 上电默认就在 Damp) ==
    1) Damp          (FSM 1)    切断电机 (上电默认状态, 显式切一次保证状态干净)
    2) Stance        (FSM 4)    锁定站立 (电机锁位置, 但不能抗扰动)
    3) Start         (FSM 811)  进入 Locomotion (走跑模式)
    4) Lie2StandUp   (FSM 701)  从躺姿站起来 (必须在 Start 模式下才能成功)
    5) 速度=0                  动态稳定站立 — Start 模式 + 速度=0 才是"站稳"

    ⚠️ 为什么是 Damp 起手, 不是 ZeroTorque?
        真实使用场景: 机器人先上电 (R1 上电默认就在 Damp, 电机被切),
        程序启动时直接 ZeroTorque 会让电机"突然上电但零力矩",
        腿会"突突"一下砸下来, 容易撞坏关节或摔坏机器人。
        Damp 起手 → Stance → Start → Lie2StandUp, 电机重新使能的过程
        是连续可控的 (Stance 锁位置, 不会砸)。

    ⚠️ 状态机切换的合法路径:
        Damp → Stance → Start → Lie2StandUp
        任何跳跃 (如 Damp → Lie2StandUp) 都会失败。
        躺下来正好相反: Lie2StandUp ↔ StandUp2Lie (FSM 702) 都需要在 Start 模式。

    ⚠️ Stance vs Start 速度=0 的区别:
        - Stance (FSM 4): 电机锁在位置, 不能抗外部推力
        - Start (FSM 811) + 速度=0: 动态平衡控制, 能抗扰动, 真正"站稳"
        所以最后必须停在 Start 模式 + 速度=0, 而不是 Stance。

== 手势 → 运动 (走两步, 速度/时长都显式写死, 不读 config) ==
    ✊  拳头 (BACKWARD) → 机器人向后走两步  (vx = -0.50 m/s × 2.0s)
    ✋  手掌 (STOP)     → 原地站住           (vx=0, vy=0, vyaw=0, 保持 Locomotion)
    ✌️  食+中指 (FORWARD) → 机器人向前走两步  (vx = +0.50 m/s × 2.0s)

    ⚠️ FORWARD 用 V 字手势 (食指+中指) 而不是单指, 原因:
        单指(☝️)在远处小手掌下容易误识别为 BACKWARD (5 指都弯一点点, 只食指伸出),
        双指(✌️)形状更明确, 抗干扰更强。
        改 v3: 2026-08 — 详见 src/vision/hand_gesture.py _classify_gesture

    ⚠️ 防连续: 一次前进/后退完成后, 必须中间出现一次 STOP 才能再做下一次前进/后退。
                防止用户"拳头 → V字" 或 "V字 → 拳头" 连续触发。

== 显式参数 (顶部, 调这里) ==
    WALK_SPEED       = 0.50 m/s     (用户实测, 比 Go2 例子 0.3 快, 但单步距离还是稳的)
    WALK_DURATION    = 2.0  s       (≈ 1.0m, 用户实测位移符合预期)
    STARTUP_DELAY    = 0.3  s       (开始 move 后, 前 0.3s 不算 WALK_DURATION, 等 R1 进入步态)
    DEBOUNCE_FRAMES  = 6            (跟 main.py 一致)

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
# 运动参数 (用户实测值)
WALK_SPEED = 0.50         # m/s, 走两步的线速度
WALK_DURATION = 2.0       # 秒, 走两步的持续时间 (≈ 1.0m, 用户实测)
STARTUP_DELAY = 0.3       # 秒, 走两步开始后前 0.3s 不算 WALK_DURATION (等 R1 步态真的启动)
DIAG_EVERY = 0.2          # 秒, 走两步过程中每 0.2s 打一次诊断 log (确认 move 真发了)

# 启动序列时序 (每步切完等一会儿, 让 R1 内部状态稳定)
DAMP_WAIT = 0.5           # Damp 后等 0.5s (电机被切, 等它软落完)
STANCE_WAIT = 0.5         # Stance 后等 0.5s
LOCO_WAIT = 1.0           # Start() 后等 1s (进入 locomotion)
STANDUP_WAIT = 3.0        # Lie2StandUp 后等 3s (站起动作慢, 大约 2-3s)

# 视觉参数
DEBOUNCE_FRAMES = 6
ROI = (0.20, 0.05, 0.60, 0.60)   # 中央 60% × 80% 区域
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
#  启动序列: Damp 起手 → 4 步走到站立
# ============================================================
def startup_sequence(robot: R1Client) -> bool:
    """按合法状态序列执行:
        Damp (1) → Stance (4) → Start (811) → Lie2StandUp (701) → 速度=0

    为什么 Damp 起手 (而不是 ZeroTorque)?
        真实使用场景: 机器人先上电 (R1 上电默认就在 Damp, 电机被切),
        程序启动时直接 ZeroTorque 会让电机"突然上电但零力矩",
        腿会"突突"一下砸下来, 容易撞坏关节或摔坏机器人。
        Damp 起手 → Stance → Start → Lie2StandUp, 电机重新使能的过程
        是连续可控的 (Stance 锁位置, 不会砸)。

    为什么必须按这个顺序? R1 FSM 不是任意两两之间都能切:
      - Stance 模式: 电机锁位置, 不能抗扰动, 不算"真站稳"
      - Start 模式 + 速度=0: 动态平衡控制, 真正抗扰动的"站稳"
      - Lie2StandUp: 动画动作, 必须在 Start 模式下才能成功执行
    """
    log.info("=" * 60)
    log.info("启动序列: Damp → Stance → Start → Lie2StandUp → 速度=0")
    log.info("=" * 60)

    # 1) Damp (FSM 1): 切断电机 — R1 上电默认就在 Damp, 显式切一次保证状态干净
    #    ⚠️ 这里 force=True 是状态机路径, 不是急停
    #    force=True 才能调, 因为 sdk_client 默认拒绝 damp() (太危险)
    log.info("[1/5] Damp(force=True)  → FSM 1   (切电机, 上电默认状态, 显式确认)")
    robot.damp(force=True)
    time.sleep(DAMP_WAIT)
    # damp() 后 _known_fsm=DAMP, 重置为 STAND 准备下一步
    robot.set_known_fsm(R1FsmState.STAND)

    # 2) Stance (FSM 4): 锁定站立 — 电机锁位置, 但不能抗扰动
    log.info("[2/5] Stance()  → FSM 4   (锁定站立, 电机锁位置, 不能抗扰动)")
    robot.balance_stand()  # 内部调 self._loco.Stance()
    time.sleep(STANCE_WAIT)

    # 3) Start (FSM 811): 进入 Locomotion (走跑模式)
    log.info("[3/5] Start()  → FSM 811 (Locomotion, 走跑模式)")
    robot.enter_locomotion()
    time.sleep(LOCO_WAIT)

    # 4) Lie2StandUp (FSM 701): 从躺姿站起 — 必须在 Start 模式下才能成功
    log.info("[4/5] Lie2StandUp()  → FSM 701 (从躺姿站起, 需在 Start 模式下)")
    robot.stand_up_from_lie()
    log.info(f"       等待 {STANDUP_WAIT}s 让站起动作完成...")
    time.sleep(STANDUP_WAIT)

    # 5) 速度=0 — Start 模式 + 速度=0 = 动态稳定站立
    log.info("[5/5] move(0, 0, 0)  →  Start 模式 + 速度=0  (动态稳定站立)")
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

    走两步时序 (实测 R1 步态启动有延迟, 直接 WALK_DURATION 计时会让步态还没动就停):
        [0, STARTUP_DELAY)        : 起步期, 不算 WALK_DURATION, 让 R1 进入步态
        [STARTUP_DELAY, STARTUP_DELAY+WALK_DURATION) : 正式走两步
        [结束]                    : stop_move, 回 IDLE
    """

    def __init__(self, robot: R1Client):
        self.robot = robot
        self.state = AppState.IDLE
        self.last_motion: Optional[Gesture] = None
        self.walk_start_t: float = 0.0   # 走两步开始的 monotonic 时间
        self.walk_dir: int = 0            # +1=前进, -1=后退
        self.last_diag_t: float = 0.0    # 上一次打诊断 log 的时间
        self.move_count: int = 0          # 走两步期间发了几次 move (诊断用)

    def step(self, gesture: Gesture, now: float) -> str:
        """每帧调用一次. 返回一个状态字符串 (log 用)."""
        # ---- 走路阶段: 持续发速度, 走完回 IDLE ----
        if self.state == AppState.WALKING:
            elapsed = now - self.walk_start_t
            if elapsed < STARTUP_DELAY + WALK_DURATION:
                # 起步期内也发 move, 让 R1 早进入步态; 但 elapsed 还没进 WALK_DURATION
                vx = self.walk_dir * WALK_SPEED
                self.robot.move(vx, 0.0, 0.0)
                self.move_count += 1

                # 诊断: 每 DIAG_EVERY 秒打一次, 确认 move 真发了
                if now - self.last_diag_t >= DIAG_EVERY:
                    phase = "起步" if elapsed < STARTUP_DELAY else "走步"
                    log.info(
                        f"    [{phase}] move(vx={vx:+.2f}, vy=0.00, vyaw=0.00) "
                        f"t={elapsed:.2f}s (move_count={self.move_count})"
                    )
                    self.last_diag_t = now

                if elapsed < STARTUP_DELAY:
                    return f"starting(t={elapsed:.2f}s/{STARTUP_DELAY:.1f}s)"
                return f"walking(t={elapsed - STARTUP_DELAY:.2f}s/{WALK_DURATION}s)"

            # 走完了: 停
            self.robot.move(0.0, 0.0, 0.0)
            self.state = AppState.IDLE
            direction = "前进" if self.walk_dir > 0 else "后退"
            log.info(
                f"  ✓ {direction}两步走完 (move_count={self.move_count}), "
                f"速度=0, 等下一次手势"
            )
            self.move_count = 0  # 重置, 下一轮用

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
                self.last_diag_t = now
                self.move_count = 0
                self.walk_dir = +1 if gesture == Gesture.FORWARD else -1
                self.last_motion = gesture
                direction = "前进" if self.walk_dir > 0 else "后退"
                log.info(
                    f"  ▶ 开始{direction}: vx={self.walk_dir * WALK_SPEED:+.2f} m/s "
                    f"× {WALK_DURATION}s (起步 {STARTUP_DELAY}s)"
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
    roi_pcts: Optional[tuple] = None,        # (x, y, w, h) 0~1, None=不画 ROI
    hand_pos: Optional[tuple] = None,        # (cx, cy) 像素, None=不画手位置
    hand_in_roi: Optional[bool] = None,      # True/False/None (None=灰)
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
    legend = "[拳=后退两步]  [掌=停]  [食+中指=前进两步]  [q/ESC=退出]"
    cv2.putText(frame, legend, (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    # ROI 识别区 (画在 HUD 之上, 让用户清楚知道手该放在哪儿)
    if roi_pcts is not None:
        x_pct, y_pct, w_pct, h_pct = roi_pcts
        rx = int(w * x_pct)
        ry = int(h * y_pct)
        rw = int(w * w_pct)
        rh = int(h * h_pct)
        # 1) 半透明绿色填充
        overlay = frame.copy()
        cv2.rectangle(overlay, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), -1)
        cv2.addWeighted(overlay, 0.06, frame, 0.94, 0, frame)
        # 2) 绿色边框
        cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
        # 3) 黄色取景角
        corner = 22
        thick = 3
        yellow = (0, 255, 255)
        # top-left
        cv2.line(frame, (rx, ry), (rx + corner, ry), yellow, thick)
        cv2.line(frame, (rx, ry), (rx, ry + corner), yellow, thick)
        # top-right
        cv2.line(frame, (rx + rw, ry), (rx + rw - corner, ry), yellow, thick)
        cv2.line(frame, (rx + rw, ry), (rx + rw, ry + corner), yellow, thick)
        # bottom-left
        cv2.line(frame, (rx, ry + rh), (rx + corner, ry + rh), yellow, thick)
        cv2.line(frame, (rx, ry + rh), (rx, ry + rh - corner), yellow, thick)
        # bottom-right
        cv2.line(frame, (rx + rw, ry + rh), (rx + rw - corner, ry + rh), yellow, thick)
        cv2.line(frame, (rx + rw, ry + rh), (rx + rw, ry + rh - corner), yellow, thick)
        # 4) 顶部 label "RECOGNITION AREA" (被黑条挡了就显示在 ROI 内顶部)
        label = "RECOGNITION AREA"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        label_x = rx + (rw - tw) // 2
        label_y = max(ry + th + 4, 100)  # 避开顶部 90px 黑条
        cv2.rectangle(frame, (label_x - 4, label_y - th - 4),
                      (label_x + tw + 4, label_y + 4), (0, 0, 0), -1)
        cv2.putText(frame, label, (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, yellow, 1)
        # 5) 手位置标记 (小圆 + 状态色)
        if hand_pos is not None:
            cx, cy = hand_pos
            if hand_in_roi is None:
                color = (200, 200, 200)
            elif hand_in_roi:
                color = (0, 255, 0)
            else:
                color = (0, 0, 255)
            cv2.circle(frame, (cx, cy), 8, color, 2)
            cv2.circle(frame, (cx, cy), 3, color, -1)


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
                # 手位置: 在 ROI 内/外/未知
                hand_pos = gr.hand_center if gr else None
                in_roi: Optional[bool]
                if gr is None or hand_pos is None:
                    in_roi = None
                else:
                    fh, fw = frame.shape[:2]
                    rx = int(fw * ROI[0])
                    ry = int(fh * ROI[1])
                    rw = int(fw * ROI[2])
                    rh = int(fh * ROI[3])
                    cx, cy = hand_pos
                    in_roi = (rx <= cx <= rx + rw) and (ry <= cy <= ry + rh)
                draw_hud(
                    frame,
                    gesture=stable,
                    state=motion.state,
                    last_motion=motion.last_motion,
                    info=info,
                    fingers_state=gr.fingers_state if gr else None,
                    roi_pcts=ROI,
                    hand_pos=hand_pos,
                    hand_in_roi=in_roi,
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
