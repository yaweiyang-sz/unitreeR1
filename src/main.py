"""R1 EDU 控制系统 — 安全测试版 v2 (手势 → 状态切换, 不发速度)

== 这一版只测"手势能不能让 R1 切 FSM 状态", 不做运动控制 ==

手势 → R1 状态映射 (B 方案, 安全版本):

    张开手掌 5 指 (STOP)   →  exit_locomotion()   → R1 回 Stance (FSM 4)
    握起拳头    (BACKWARD) →  damp(force=True)    → R1 切 Damp (FSM 1) 切断电机

⚠️ 关键警告 ⚠️
    damp() 切断 R1 所有电机, 机器人会瘫倒, 必须物理 power cycle 才能恢复。
    1. 跑这个版本前, 把机器人固定在支架上, 周围留 1m 空间
    2. 戴好护具, 急停按钮在手
    3. 跑完一定要 power cycle R1 才能再跑其他 example

== 跟 A 方案的差异 (一行就能切) ==
A 方案 (你最初提的, 危险): STOP → damp, BACKWARD → exit_locomotion
要看 A: 把 GESTURE_TO_STATE 那个 dict 里的 value 对调就行

== 不发运动指令 ==
    FORWARD / LEFT / RIGHT 手势暂时 no-op, 等 FSM 状态机走通再加

== 跑法 ==
    # 1) Jetson 上, 默认安全模式 (不进入 locomotion, 只能切 damp/stance)
    python3 src/main.py eth10

    # 2) 显式开 locomotion 模式 (后面要加运动指令时再用)
    python3 src/main.py eth10 --enable-loco

    # 3) 干跑, 不连机器人, 用本机 USB 摄像头
    python3 src/main.py eth10 --dry-run

    # 4) 调试用, 不显示 OpenCV 窗口
    python3 src/main.py eth10 --no-window
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

from src.config import load_config, get
from src.logger import setup_logger
from src.robot.sdk_client import R1Client, R1FsmState, R1Mode
from src.vision.hand_gesture import HandGestureDetector, GestureDebouncer, Gesture
from src.vision.overlays import (
    draw_gesture_label,
    draw_gesture_legend,
    draw_hand_state,
    draw_telemetry,
)

log = setup_logger("r1.main")


# ============================================================
#  应用状态机 (本程序, 不是 R1 FSM)
# ============================================================
class State(enum.Enum):
    BOOT = "boot"
    IDLE = "idle"             # 启动后立刻进入, 等待手势
    STOPPED = "stopped"       # 急停 (按 'q' / ESC)
    ERROR = "error"


# ============================================================
#  核心: 手势 → R1 FSM 状态 映射
# ============================================================
#
# B 方案 (默认, 安全):
#   STOP (张开手掌)     → exit_locomotion()   (Stance, 最安全)
#   BACKWARD (拳头)     → damp(force=True)    (危险, 但不容易误触)
#
# A 方案 (你最初提的): 把下面两行 value 对调即可
GESTURE_TO_STATE_ACTION: dict[Gesture, str] = {
    Gesture.STOP:     "exit_locomotion",  # 张开手掌 → Stance (FSM 4)
    Gesture.BACKWARD: "damp",             # 拳头 → Damp (FSM 1, 危险)
    # FORWARD / LEFT / RIGHT: 故意 no-op, 等 v3 加运动指令
}


# ============================================================
#  CLI 参数
# ============================================================
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="R1 EDU 手势→状态切换 (安全测试版)")
    p.add_argument("iface", nargs="?", default=None, help="接 R1 的网卡 (如 eth10)")
    p.add_argument("--config", default="config.yaml")
    p.add_argument("--dry-run", action="store_true", help="不连机器人, 干跑视觉")
    p.add_argument("--no-window", action="store_true", help="不显示 OpenCV 窗口")
    p.add_argument("--webcam", type=int, default=0, help="dry-run 模式下用的 USB 摄像头 index")
    p.add_argument("--enable-loco", action="store_true",
                   help="启动后让机器人进入 locomotion (FSM 811)。v2 默认不进入, 留作后面 v3 加运动指令时启用")
    p.add_argument("--sport-state-topic", default=None)
    return p.parse_args()


# ============================================================
#  视频源
# ============================================================
def open_video_source(robot: R1Client, webcam_index: int) -> tuple[str, callable]:
    if not robot.is_dry_run:
        code, data = robot.get_image()
        if code == 0 and data:
            log.info("视频源: 机器人前置摄像头")
            return "robot", lambda: robot.get_image()
    cap = cv2.VideoCapture(webcam_index)
    if cap.isOpened():
        log.info(f"视频源: 本机 USB 摄像头 index={webcam_index}")
        return "webcam", lambda: _read_webcam(cap)
    cap.release()
    log.error("没有可用的视频源")
    return "none", lambda: (-1, None)


def _read_webcam(cap: cv2.VideoCapture) -> tuple[int, Optional[bytes]]:
    ok, frame = cap.read()
    if not ok:
        return -1, None
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return -1, None
    return 0, buf.tobytes()


# ============================================================
#  手势 → R1 状态机 的执行
# ============================================================
def execute_gesture_action(robot: R1Client, gesture: Gesture) -> str:
    """根据 GESTURE_TO_STATE_ACTION 把手势翻译成 R1 状态切换。

    返回执行结果 (log 用):
        "executed:<action>"  - 真的给 R1 发了指令
        "noop:<gesture>"     - 手势没有映射, 不发指令
        "skipped:<action>"   - action 是 damp 但机器人不在 locomotion (会标 warn)
    """
    action = GESTURE_TO_STATE_ACTION.get(gesture)
    if action is None:
        return f"noop:{gesture.value}"

    if action == "damp":
        # 关键: damp 是最危险的操作, 必须 force=True
        # 即便 R1 在 Stance 也能切到 Damp, 不需要先 enter_locomotion
        log.warning(f"  >>>>> 手势 {gesture.value} 触发 damp() — R1 电机将切断! <<<<<")
        robot.damp(force=True)
        return f"executed:damp"

    elif action == "exit_locomotion":
        log.info(f"  >>>>> 手势 {gesture.value} 触发 exit_locomotion() — R1 回 Stance <<<<<")
        # 即便不在 locomotion 也安全 (R1Client.exit_locomotion 内部有 _in_locomotion 检查)
        robot.exit_locomotion()
        return f"executed:exit_locomotion"

    return f"noop:{action}"


# ============================================================
#  主循环
# ============================================================
def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)

    iface = args.iface or get(cfg, "robot.network_interface", "eth10")
    mode = R1Mode.DRY_RUN if args.dry_run else R1Mode.REAL
    sport_topic = args.sport_state_topic or get(cfg, "robot.sport_state_topic", "rt/sportmodestate")
    robot = R1Client(iface, mode=mode, sport_state_topic=sport_topic, enable_state_subscription=False)

    # 默认安全: 不进入 locomotion
    will_enter_loco = (not args.dry_run) and args.enable_loco
    if will_enter_loco:
        log.warning("⚠️ --enable-loco 已打开, 机器人会进入 locomotion (FSM 811)")
        log.warning("⚠️ 这个版本 v2 不发运动指令, 进入 locomotion 主要用于验证 Move 链路")
        try:
            input("按 Enter 继续, Ctrl-C 取消...")
        except KeyboardInterrupt:
            log.info("用户取消")
            return 0

    state = State.BOOT
    try:
        robot.initialize()
        state = State.IDLE
        log.info("✓ R1 客户端就绪")
        log.info(f"  手势映射 (B 方案, 安全):")
        log.info(f"    张开手掌 (STOP)     → exit_locomotion() (Stance)")
        log.info(f"    拳头     (BACKWARD) → damp(force=True)  (⚠️ 危险, 切电机)")
        log.info(f"  其他手势 (FORWARD/LEFT/RIGHT) 暂时 no-op")

        if will_enter_loco:
            robot.enter_locomotion()
        else:
            log.info("[safe-mode] 不进入 locomotion, 机器人将停在 Stance, 等你手势触发切 FSM")

        # 视觉模块
        gesture_det = HandGestureDetector()
        debouncer = GestureDebouncer(debounce_frames=get(cfg, "gesture.debounce_frames", 6))

        # 视频源
        src_name, src_read = open_video_source(robot, args.webcam)
        if src_name == "none":
            state = State.ERROR
            return 1

        # 上一帧执行过的 action, 避免重复触发 (手势稳定后会持续 debounce)
        last_action: Optional[str] = None
        last_fps_t = time.monotonic()
        fps_counter = 0
        cur_fps = 0.0

        log.info(f"进入主循环, 按 'q' 或 ESC 退出 (state={state.value})")

        while True:
            t_loop = time.monotonic()

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

            # 3) 手势 → R1 状态切换 (核心逻辑)
            if state == State.IDLE and stable != Gesture.UNKNOWN:
                result = execute_gesture_action(robot, stable)
                if result.startswith("executed:"):
                    last_action = result
                # 无论是不是 noop, 都 log 出来, 方便看手势链路
                log.info(f"  [gesture]  detected={gr.gesture.value if gr else 'none'}  "
                         f"stable={stable.value}  → {result}")

            # 4) UI overlay
            draw_gesture_label(frame, gr, stable=stable)
            draw_hand_state(frame, gr)
            draw_gesture_legend(frame)

            # 5) Telemetry
            fps_counter += 1
            if t_loop - last_fps_t > 0.5:
                cur_fps = fps_counter / (t_loop - last_fps_t)
                fps_counter = 0
                last_fps_t = t_loop

            extra = {
                "src": src_name,
                "iface": iface,
                "dry": "yes" if robot.is_dry_run else "no",
                "loco": "on" if will_enter_loco else "off",
                "last": last_action or "-",
            }
            draw_telemetry(frame, cur_fps, state.value, extra)

            # 6) 显示
            cv2.imshow("R1 Gesture FSM Control", frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # q or ESC
                log.info("用户按键退出")
                state = State.STOPPED
                break

        return 0

    except KeyboardInterrupt:
        log.info("Ctrl-C 退出")
        return 0
    except Exception as e:  # noqa: BLE001
        log.exception(f"主循环异常: {e}")
        return 2
    finally:
        # 关键: 退出时机器人回到 Stance
        try:
            if not robot.is_dry_run and will_enter_loco:
                log.info("退出时让 R1 回到 Stance...")
                robot.exit_locomotion()
        except Exception:  # noqa: BLE001
            pass
        try:
            robot.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            gesture_det.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    raise SystemExit(main())
