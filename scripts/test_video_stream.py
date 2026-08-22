"""SDK 集成测试 2: 视频流 (从机器人前置摄像头取帧)。

R1 没有自己的 video_client, 走 unitree_sdk2py.go2.video.video_client.VideoClient
(因为 Go2 与 R1 共用同一套 video_service, 都是 GetImageSample RPC)。

为了确认 client 真的工作 (不是拿到占位 / 黑屏), 默认连续抓 5 帧写到 ./video_frames/
你 scp / rsync 回来肉眼检查。

用法:
    python3 scripts/test_video_stream.py eth0                 # 默认抓 5 帧
    python3 scripts/test_video_stream.py eth0 --num 10        # 抓 10 帧
    python3 scripts/test_video_stream.py eth0 --save one.jpg  # 只抓 1 帧 (旧行为)
    python3 scripts/test_video_stream.py eth0 --preview       # 弹窗预览
    python3 scripts/test_video_stream.py eth0 --out-dir /tmp/r1_frames  # 自定义输出目录
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
    p.add_argument("--num", type=int, default=5, help="连续抓多少帧 (默认 5)")
    p.add_argument("--interval", type=float, default=0.5, help="帧间隔 (秒, 默认 0.5)")
    p.add_argument("--out-dir", default="./video_frames", help="输出目录")
    p.add_argument("--preview", action="store_true", help="同时弹 OpenCV 窗口")
    p.add_argument("--save", default=None, help="只抓 1 帧并另存为指定路径 (旧行为兼容)")
    args = p.parse_args()

    mode = R1Mode.DRY_RUN if args.dry_run else R1Mode.REAL
    client = R1Client(args.iface, mode=mode)

    try:
        client.initialize()
    except Exception as e:  # noqa: BLE001
        log.error(f"初始化失败: {e}")
        return 1

    # ---- 兼容旧 --save: 只抓 1 帧 ----
    if args.save:
        code, data = client.get_image()
        if code != 0 or data is None:
            log.error(f"取帧失败 code={code}")
            client.shutdown()
            return 2
        arr = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            log.error("JPEG 解码失败")
            client.shutdown()
            return 3
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(args.save, frame)
        log.info(f"✓ 已保存 {args.save} ({frame.shape[1]}x{frame.shape[0]})")
        client.shutdown()
        return 0

    # ---- 默认行为: 连续抓 N 帧到 out_dir ----
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 清掉上一次的, 避免混淆
    for old in out_dir.glob("frame_*.jpg"):
        try:
            old.unlink()
        except Exception:  # noqa: BLE001
            pass

    log.info(f"开始抓 {args.num} 帧 → {out_dir.resolve()}")
    log.info(f"间隔 {args.interval}s, 总耗时约 {args.num * args.interval:.1f}s")

    saved: list[tuple[int, int, int, Path]] = []  # (idx, w, h, path)
    failed_codes: list[int] = []
    sizes: list[int] = []

    t_start = time.monotonic()
    for i in range(args.num):
        code, data = client.get_image()
        t_now = time.monotonic() - t_start

        if code != 0 or not data:
            failed_codes.append(code)
            log.warning(f"  [{i+1:02d}/{args.num}] t={t_now:5.2f}s  code={code}  data=None")
            time.sleep(args.interval)
            continue

        arr = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            log.warning(f"  [{i+1:02d}/{args.num}] t={t_now:5.2f}s  JPEG 解码失败 (字节数 {len(data)})")
            time.sleep(args.interval)
            continue

        h, w = frame.shape[:2]
        # 估算亮度: 太黑(<10) 或 太白(>245) 都可疑
        mean_luma = float(frame.mean())
        is_black = mean_luma < 5
        is_white = mean_luma > 250

        path = out_dir / f"frame_{i+1:02d}_t{t_now:05.2f}.jpg"
        cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        sizes.append(len(data))
        saved.append((i + 1, w, h, path))

        flag = ""
        if is_black:
            flag = "  ⚠ 全黑 (mean luma < 5)"
        elif is_white:
            flag = "  ⚠ 全白 (mean luma > 250)"
        log.info(
            f"  [{i+1:02d}/{args.num}] t={t_now:5.2f}s  {w}x{h}  "
            f"{len(data):>7} bytes  mean_luma={mean_luma:6.1f}{flag}"
        )

        if args.preview:
            cv2.putText(
                frame, f"R1 camera frame {i+1}/{args.num}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
            )
            cv2.imshow("R1 Camera", frame)

        time.sleep(args.interval)

    if args.preview:
        cv2.waitKey(500)
        cv2.destroyAllWindows()

    client.shutdown()

    # ---- 总结 ----
    print()
    log.info("=" * 60)
    log.info(f"共抓 {args.num} 帧, 成功 {len(saved)}, 失败 {len(failed_codes)}")
    if failed_codes:
        log.warning(f"失败 code: {failed_codes}")
    if saved:
        log.info(f"输出目录: {out_dir.resolve()}")
        for idx, w, h, p in saved:
            log.info(f"  - {p.name}  ({w}x{h})")

        # 文件字节差异: 如果每张都几乎一样大小, 可能是同一帧 / 占位图
        if len(sizes) >= 2:
            size_set = set(sizes)
            if len(size_set) == 1:
                log.warning("⚠ 所有帧字节数完全相同 — 可能是同一帧 / 占位图, 请肉眼确认")
            else:
                spread = max(sizes) - min(sizes)
                log.info(
                    f"帧字节数范围: {min(sizes)} ~ {max(sizes)} (spread {spread}) — 有变化 = 真视频"
                )

    log.info("=" * 60)
    return 0 if saved else 4


if __name__ == "__main__":
    raise SystemExit(main())
