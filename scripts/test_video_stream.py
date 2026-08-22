"""SDK 集成测试 2: 视频流 (从机器人前置摄像头取帧)。

R1 没有自己的 video_client, 走 unitree_sdk2py.go2.video.video_client.VideoClient
(因为 Go2 与 R1 共用同一套 video_service, 都是 GetImageSample RPC)。

用法:
    python3 scripts/test_video_stream.py eth0
    python3 scripts/test_video_stream.py eth0 --preview
    python3 scripts/test_video_stream.py eth0 --save out.jpg
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from src.logger import setup_logger
from src.robot.sdk_client import R1Client, R1Mode

log = setup_logger("test.video")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("iface", nargs="?", default="eth0")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--preview", action="store_true", help="OpenCV 窗口预览 5s")
    p.add_argument("--save", default=None, help="保存一帧为 JPEG 文件")
    args = p.parse_args()

    mode = R1Mode.DRY_RUN if args.dry_run else R1Mode.REAL
    client = R1Client(args.iface, mode=mode)

    try:
        client.initialize()
    except Exception as e:  # noqa: BLE001
        log.error(f"初始化失败: {e}")
        return 1

    log.info("取一帧...")
    code, data = client.get_image()
    if code != 0 or data is None:
        log.error(f"取帧失败 code={code}, data len={len(data) if data else 0}")
        log.error("检查: 1) 视频服务是否启用 (App -> 设置 -> 视频) 2) 机器人是否在调试模式")
        client.shutdown()
        return 2
    arr = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        log.error("JPEG 解码失败")
        client.shutdown()
        return 3
    h, w = frame.shape[:2]
    log.info(f"✓ 收到一帧: {w}x{h}, 字节数 {len(data)}")

    if args.save:
        cv2.imwrite(args.save, frame)
        log.info(f"已保存 {args.save}")

    if args.preview:
        log.info("预览 5 秒, 按 ESC 提前退出...")
        end = time.monotonic() + 5.0
        n = 1
        while time.monotonic() < end:
            code, data = client.get_image()
            if code == 0 and data:
                arr = np.frombuffer(data, dtype=np.uint8)
                f = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if f is not None:
                    cv2.putText(f, f"R1 camera preview n={n}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.imshow("R1 Camera", f)
                    n += 1
            if cv2.waitKey(30) & 0xFF == 27:
                break
        cv2.destroyAllWindows()

    client.shutdown()
    log.info("==== 完成 ====")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
