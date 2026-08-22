"""主控: 状态机把 视觉 → 决策 → 机器人 串起来。

R1 生命周期 (FSM):
    boot → initialize (DDS + clients)
         → enter_locomotion (FSM 811, 需 --enable-loco 确认)
         → GESTURE / FOLLOW  (持续发 Move)
         → exit_locomotion (FSM 4, Stance)  ←  任何退出路径

    ⚠️ 不传 --enable-loco 不会真正进入 locomotion, 机器人会停在 Stance,
       整个流程只会本地跑视觉 + 把 Move 命令打到 DDS, 但 LocoClient 会被
       真实 robot 拒收 (因为不在 FSM 811)。这是干跑/调试用的安全模式。

状态:
    BOOT       启动, 加载各模块
    IDLE       等待进入 GESTURE/FOLLOW (默认)
    GESTURE    手势控制 (默认)
    FOLLOW     跟随模式
    STOPPED    急停 (按键 'q' / ESC)
    ERROR      出错, 准备退出

切换:
    GESTURE 中持续识别到 STOP 超过 N 秒 -> FOLLOW
    FOLLOW 中识别到 BACKWARD (拳头) -> 回到 GESTURE
    任意状态按 'q' / ESC -> STOPPED → exit_locomotion

视频源:
    1) 优先 R1 前置摄像头 (通过 Go2 VideoClient 复用)
    2) 失败 / dry-run -> 本机 USB 摄像头 (index 0)
    3) 都没有 -> 报错退出

运行:
    python3 src/main.py eth0                      # 真机 (默认干跑, 不让机器人走)
    python3 src/main.py eth0 --enable-loco       # 真机 + 真的进入 locomotion
    python3 src/main.py eth0 --dry-run           # 完全模拟
    python3 src/main.py eth0 --no-window         # 服务器模式
    python3 src/main.py eth0 --follow            # 启动后直接进 FOLLOW
"""
from __future__ import annotations

import argparse
import enum
import sys
import time
from pathlib import Path
from typing import Optional

# 让脚本既可作为模块跑，也可直接跑
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from src.config import load_config, get
from src.logger import setup_logger
from src.robot.sdk_client import R1Client, R1Mode
from src.vision.hand_gesture import HandGestureDetector, GestureDebouncer, Gesture
from src.vision.body_follow import BodyFollower
from src.vision.overlays import (
    draw_gesture_label,
    draw_gesture_legend,
    draw_hand_state,
    draw_follow_target,
    draw_telemetry,
)
from src.control.velocity_smoother import VelocitySmoother
from src.control.gesture_to_command import GestureCommandMapper
from src.control.follow_controller import FollowController
from src.view.viewer import Viewer


log = setup_logger("r1.main")


class State(enum.Enum):
    BOOT = "boot"
    IDLE = "idle"
    GESTURE = "gesture"
    FOLLOW = "follow"
    STOPPED = "stopped"
    ERROR = "error"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Unitree R1 EDU 手势控制主控")
    p.add_argument("iface", nargs="?", default=None, help="机器人所在网卡名 (如 eth0)")
    p.add_argument("--config", default="config.yaml", help="配置文件路径")
    p.add_argument("--dry-run", action="store_true", help="不连真实机器人, 用本机摄像头调试")
    p.add_argument("--no-window", action="store_true", help="不显示 OpenCV 窗口 (服务器环境)")
    p.add_argument("--webcam", type=int, default=0, help="fallback USB 摄像头 index")
    p.add_argument("--follow", action="store_true", help="启动后默认进入跟随模式")
    p.add_argument(
        "--enable-loco",
        action="store_true",
        help="真机模式下, 显式让机器人进入 locomotion (FSM 811)。不传则停在 Stance。",
    )
    p.add_argument(
        "--enter-loco-now",
        action="store_true",
        help="跳过 '按 Enter 确认' 提示, 直接进 locomotion (脚本场景)。",
    )
    p.add_argument(
        "--sport-state-topic",
        default=None,
        help="覆盖 SportModeState_ 订阅主题, 默认 rt/sportmodestate",
    )
    return p.parse_args()


def open_video_source(robot: R1Client, webcam_index: int) -> tuple[str, callable]:
    """选择视频源: 优先机器人摄像头, 失败则本机 USB。"""
    if not robot.is_dry_run:
        # 试探一帧
        code, data = robot.get_image()
        if code == 0 and data:
            log.info("视频源: 机器人前置摄像头")
            return "robot", lambda: robot.get_image()

    cap = cv2.VideoCapture(webcam_index)
    if cap.isOpened():
        log.info(f"视频源: 本机 USB 摄像头 index={webcam_index}")
        return "webcam", lambda: _read_webcam(cap)
    cap.release()

    log.error("没有可用的视频源 (机器人摄像头失败, USB 摄像头也打不开)")
    return "none", lambda: (-1, None)


def _read_webcam(cap: cv2.VideoCapture) -> tuple[int, Optional[bytes]]:
    ok, frame = cap.read()
    if not ok:
        return -1, None
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return -1, None
    return 0, buf.tobytes()


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)

    iface = args.iface or get(cfg, "robot.network_interface", "eth0")
    mode = R1Mode.DRY_RUN if args.dry_run else R1Mode.REAL
    sport_topic = args.sport_state_topic or get(cfg, "robot.sport_state_topic", "rt/sportmodestate")
    robot = R1Client(iface, mode=mode, sport_state_topic=sport_topic)
    state = State.BOOT

    # 是否让真实机器人真的进 locomotion
    will_enter_loco = (
        (not args.dry_run)
        and (args.enable_loco or get(cfg, "robot.auto_enter_locomotion", False))
    )
    if will_enter_loco and not args.enter_loco_now:
        log.warning("即将让 R1 进入 locomotion (FSM 811) — 这是危险操作")
        log.warning("确认条件: 支架悬吊 / 周围 ≥ 2m / 急停按钮在握")
        try:
            input("按 Enter 继续, Ctrl-C 取消...")
        except KeyboardInterrupt:
            log.info("用户取消, 不进入 locomotion")
            will_enter_loco = False

    try:
        robot.initialize()
        state = State.IDLE

        if will_enter_loco:
            robot.enter_locomotion()
        else:
            log.info("[safe-mode] 不进入 locomotion, 机器人将停在 Stance (FSM 4)")

        # 视觉模块
        gesture_det = HandGestureDetector()
        follower = BodyFollower(
            target_center_x=get(cfg, "follow.target_center_x", 320),
            target_bbox_area=get(cfg, "follow.target_bbox_area", 50000),
            kp_yaw=get(cfg, "follow.kp_yaw", 0.0025),
            kp_distance=get(cfg, "follow.kp_distance", 0.00004),
            max_linear_speed=get(cfg, "follow.max_linear_speed", 0.3),
            max_angular_speed=get(cfg, "follow.max_angular_speed", 0.5),
            deadzone_yaw=get(cfg, "follow.deadzone_yaw", 30),
            deadzone_area=get(cfg, "follow.deadzone_area", 8000),
        )
        debouncer = GestureDebouncer(debounce_frames=get(cfg, "gesture.debounce_frames", 6))

        # 控制层
        smoother = VelocitySmoother()
        mapper = GestureCommandMapper(
            forward_speed=get(cfg, "control.forward_speed", 0.3),
            backward_speed=get(cfg, "control.backward_speed", -0.2),
            turn_speed=get(cfg, "control.turn_speed", 0.5),
            turn_direction_sign=get(cfg, "control.turn_direction_sign", 1),
        )
        follow_ctrl = FollowController(robot, follower, smoother)

        # 视野显示
        viewer = Viewer(
            opencv_window=not args.no_window and get(cfg, "viewer.opencv_window", True),
            web_port=get(cfg, "viewer.web_port", 8080),
        )
        viewer.start()

        # 视频源
        src_name, src_read = open_video_source(robot, args.webcam)
        if src_name == "none":
            state = State.ERROR
            return 1

        # 初始进入 GESTURE 模式
        state = State.FOLLOW if args.follow else State.GESTURE
        if state == State.GESTURE:
            log.info("进入 GESTURE 模式 (默认)")

        # 切换状态用的临时变量
        stop_hold_start: Optional[float] = None
        last_fps_t = time.monotonic()
        fps_counter = 0
        cur_fps = 0.0
        last_sent_v = (0.0, 0.0, 0.0)
        send_period = 1.0 / max(1, get(cfg, "control.loop_hz", 30))
        last_send_t = 0.0

        # 主循环
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

            # 2) 手势识别
            gr = gesture_det.detect(frame)
            stable = debouncer.update(gr.gesture if gr else Gesture.UNKNOWN)

            # 3) 状态机: 决定当前行为
            send_vx, send_vy, send_vyaw = 0.0, 0.0, 0.0

            if state == State.GESTURE:
                if gr is None:
                    # 没有手: 切到 IDLE 让机器人稳住
                    send_vx, send_vy, send_vyaw = 0.0, 0.0, 0.0
                else:
                    target_v = mapper.to_velocity(stable)
                    cur = smoother.update(target_v)
                    send_vx, send_vy, send_vyaw = cur.vx, cur.vy, cur.vyaw

                # 状态切换: 持续 STOP 1.5 秒 -> FOLLOW
                if stable == Gesture.STOP:
                    if stop_hold_start is None:
                        stop_hold_start = t_loop
                    elif t_loop - stop_hold_start > 1.5:
                        log.info("STOP 持续 1.5s, 切换到 FOLLOW")
                        state = State.FOLLOW
                        stop_hold_start = None
                else:
                    stop_hold_start = None

            elif state == State.FOLLOW:
                vx, vyaw = follow_ctrl.step(frame)
                send_vx, send_vy, send_vyaw = vx, 0.0, vyaw
                # 拳头手势超过 0.5s -> 回到 GESTURE
                if stable == Gesture.BACKWARD:
                    if stop_hold_start is None:
                        stop_hold_start = t_loop
                    elif t_loop - stop_hold_start > 0.5:
                        log.info("识别拳头, 退出跟随 -> GESTURE")
                        state = State.GESTURE
                        robot.stop_move()
                        smoother.reset()
                        stop_hold_start = None
                else:
                    stop_hold_start = None

            elif state == State.STOPPED:
                send_vx, send_vy, send_vyaw = 0.0, 0.0, 0.0

            # 4) 节流发指令 (避免高频撞 DDS)
            if t_loop - last_send_t > send_period:
                if (send_vx, send_vy, send_vyaw) != last_sent_v or state == State.STOPPED:
                    robot.move(send_vx, send_vy, send_vyaw)
                    last_sent_v = (send_vx, send_vy, send_vyaw)
                last_send_t = t_loop

            # 5) UI
            draw_gesture_label(frame, gr, stable=stable)
            draw_hand_state(frame, gr)            # 左下: 5 根手指状态条
            draw_gesture_legend(frame)            # 右上: 手势 -> 动作 永久图例
            tgt = follower._locked if state == State.FOLLOW else None
            if tgt is not None:
                draw_follow_target(frame, tgt,
                                   target_center_x=follower.target_center_x,
                                   target_bbox_area=follower.target_bbox_area)
            fps_counter += 1
            if t_loop - last_fps_t > 0.5:
                cur_fps = fps_counter / (t_loop - last_fps_t)
                fps_counter = 0
                last_fps_t = t_loop
            extra = {
                "src": src_name,
                "iface": iface,
                "dry": "yes" if robot.is_dry_run else "no",
                "fsm": robot.get_fsm_state().value,
                "loco": "on" if will_enter_loco else "off",
                "cmd": f"({send_vx:+.2f},{send_vy:+.2f},{send_vyaw:+.2f})",
            }
            draw_telemetry(frame, cur_fps, state.value, extra)

            # 6) 显示
            viewer.update(frame)
            if viewer.poll_quit():
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
        # 退出顺序: 停速度 → Stance (FSM 4) → 关 DDS
        try:
            robot.stop_move()
        except Exception:  # noqa: BLE001
            pass
        if will_enter_loco and not robot.is_dry_run:
            try:
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
        try:
            follower.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    raise SystemExit(main())
