"""视觉测试 1: 手势识别 (不连机器人, 用本机 USB 摄像头)。

用法:
    python3 scripts/test_gesture_webcam.py
    python3 scripts/test_gesture_webcam.py --camera 1
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2

from src.logger import setup_logger
from src.vision.hand_gesture import HandGestureDetector, GestureDebouncer
from src.vision.overlays import draw_gesture_legend, draw_hand_state

log = setup_logger("test.gesture")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--debounce", type=int, default=6)
    args = p.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        log.error(f"打不开摄像头 index={args.camera}")
        return 1

    det = HandGestureDetector()
    deb = GestureDebouncer(debounce_frames=args.debounce)

    log.info("把手放到摄像头前, 按 ESC 退出")
    fps_t = time.monotonic()
    fps_n = 0
    fps = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        fps_n += 1
        if time.monotonic() - fps_t > 0.5:
            fps = fps_n / (time.monotonic() - fps_t)
            fps_n = 0
            fps_t = time.monotonic()
        res = det.detect(frame)
        stable = deb.update(res.gesture if res else deb.stable)
        if res is not None:
            det.draw(frame, res)
        # 调试面板: 5 根手指状态 + ratio
        draw_hand_state(frame, res)
        # 手势图例 (永久显示)
        draw_gesture_legend(frame)
        conf = res.confidence if res is not None else 0.0
        inf_ms = res.inference_ms if res is not None else 0.0
        label = f"FPS {fps:.1f}  stable={stable.value}  conf={conf:.2f}  {inf_ms:.0f}ms"
        cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Gesture Test", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    det.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
