"""视觉测试 2: 人体跟随 (不连机器人, 用本机 USB 摄像头)。

用法:
    python3 scripts/test_pose_webcam.py
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2

from src.logger import setup_logger
from src.vision.body_follow import BodyFollower

log = setup_logger("test.pose")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--target-area", type=int, default=50000)
    args = p.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        log.error("打不开摄像头")
        return 1

    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
    fol = BodyFollower(
        target_center_x=w // 2,
        target_bbox_area=args.target_area,
    )

    log.info(f"站在摄像头前, 目标面积={args.target_area} px² (640x480 画面的近景)")
    log.info("按 ESC 退出")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        vx, vyaw, tgt = fol.step(frame)
        if tgt is not None:
            fol.draw(frame, tgt)
            cv2.line(frame, (fol.target_center_x, 0), (fol.target_center_x, h), (0, 200, 0), 1)
        info = f"vx={vx:+.2f}  vyaw={vyaw:+.2f}  area={int(tgt.area) if tgt else 0}"
        cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Follow Test", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    fol.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
