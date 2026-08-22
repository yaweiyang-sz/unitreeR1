"""SDK 集成测试 2: 视频流 (从机器人前置摄像头取帧)。

R1 没有自己的 video_client, 走 unitree_sdk2py.go2.video.video_client.VideoClient
(因为 Go2 与 R1 共用同一套 video_service, 都是 GetImageSample RPC)。

⚠️ 视频抓取跑在**子进程**里: 某些 R1 固件在 video_service 状态不对时,
   GetImageSample RPC 会让 cyclonedds C 层 segfault, Python try/except 拦不住。
   子进程隔离后, 即便崩了父进程也能拿到 exit code (139 / -11 = SIGSEGV),
   把信号、已抓到的帧、错误信息都给你, 不会拖死主程序。

用法:
    python3 scripts/test_video_stream.py eth10                # 默认抓 5 帧
    python3 scripts/test_video_stream.py eth10 --num 10       # 抓 10 帧
    python3 scripts/test_video_stream.py eth10 --save one.jpg # 只抓 1 帧
    python3 scripts/test_video_stream.py eth10 --out-dir /tmp/r1_frames
    python3 scripts/test_video_stream.py eth10 --probe        # 只跑一次 RPC, 不保存
"""
from __future__ import annotations

import argparse
import multiprocessing as mp
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.logger import setup_logger

log = setup_logger("test.video")


# ============== 子进程 worker (顶层函数, Windows spawn 友好) ==============

def _video_worker(q, iface: str, num: int, interval: float, out_dir_str: str,
                  save_single: str | None) -> None:
    """子进程: 跑 VideoClient 抓帧, 把结果通过 Queue 送给父进程。

    通过 status code 报告:
        0  成功, 全部抓完
        1  初始化失败
        2  抓帧中途失败 (但没崩)
        3  SIGSEGV 等致命信号 (Python 通过异常信息推测, 实际是父进程看 exit code)
    """
    import cv2
    import numpy as np
    from src.robot.sdk_client import R1Client, R1Mode

    out_dir = Path(out_dir_str)
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        client = R1Client(iface, mode=R1Mode.REAL, enable_state_subscription=False)
        client.initialize()
    except Exception as e:  # noqa: BLE001
        q.put({"status": 1, "error": f"init failed: {e}"})
        return

    # ---- --save 单帧模式 ----
    if save_single:
        try:
            code, data = client.get_image()
            if code == 0 and data:
                arr = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    p = Path(save_single)
                    p.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(p), frame)
                    q.put({"status": 0, "saved": [str(p)], "w": frame.shape[1], "h": frame.shape[0]})
                else:
                    q.put({"status": 2, "error": "JPEG decode failed"})
            else:
                q.put({"status": 2, "error": f"get_image code={code}"})
        except Exception as e:  # noqa: BLE001
            q.put({"status": 2, "error": str(e)})
        finally:
            client.shutdown()
        return

    # ---- 默认多帧模式 ----
    saved: list[dict] = []
    sizes: list[int] = []
    t_start = time.monotonic()
    for i in range(num):
        try:
            code, data = client.get_image()
        except Exception as e:  # noqa: BLE001
            q.put({"status": 2, "error": f"get_image raised: {e}", "saved": saved})
            return
        t_now = time.monotonic() - t_start
        if code != 0 or not data:
            time.sleep(interval)
            continue
        try:
            arr = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception as e:  # noqa: BLE001
            time.sleep(interval)
            continue
        if frame is None:
            time.sleep(interval)
            continue
        h, w = frame.shape[:2]
        mean_luma = float(frame.mean())
        path = out_dir / f"frame_{i+1:02d}_t{t_now:05.2f}.jpg"
        cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        sizes.append(len(data))
        saved.append({"path": str(path), "w": w, "h": h, "luma": mean_luma, "bytes": len(data)})
        time.sleep(interval)

    q.put({"status": 0, "saved": saved, "sizes": sizes})


# ============== 父进程 ==============

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("iface", nargs="?", default="eth10")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--num", type=int, default=5)
    p.add_argument("--interval", type=float, default=0.5)
    p.add_argument("--out-dir", default="./video_frames")
    p.add_argument("--save", default=None)
    p.add_argument("--probe", action="store_true", help="只跑一次 RPC, 不保存")
    p.add_argument("--timeout", type=float, default=15.0, help="子进程超时 (秒)")
    args = p.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 兼容旧 --save
    save_single = None if args.probe else args.save

    if args.dry_run:
        log.info("[DRY-RUN] 跳过真机, 写一张占位图到 out_dir")
        import cv2
        import numpy as np
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "DRY-RUN placeholder", (50, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        path = out_dir / "dryrun.jpg"
        cv2.imwrite(str(path), img)
        log.info(f"  → {path}")
        return 0

    # ---- 用子进程跑, 隔离潜在 segfault ----
    q: mp.Queue = mp.Queue(maxsize=1)
    proc = mp.Process(
        target=_video_worker,
        args=(q, args.iface, args.num, args.interval, str(out_dir), save_single),
        daemon=True,
    )
    log.info(f"启动子进程抓视频, iface={args.iface}, num={args.num}, timeout={args.timeout}s")
    proc.start()
    proc.join(timeout=args.timeout)

    # ---- 子进程超时 ----
    if proc.is_alive():
        log.warning(f"子进程 {args.timeout}s 内没完成, terminate")
        proc.terminate()
        proc.join(1.0)
        if proc.is_alive():
            proc.kill()
            proc.join(0.5)
        log.error("视频抓取超时 — R1 video_service 可能没响应")
        return 5

    # ---- 子进程被信号杀掉 (SIGSEGV = -11 on POSIX, 139 on POSIX exit code) ----
    exit_code = proc.exitcode
    if exit_code is not None and exit_code < 0:
        sig = -exit_code
        log.error(f"!!! 子进程被信号 {sig} 杀掉 (很可能是 SIGSEGV={11})")
        log.error("R1 端 video_service 在当前状态下让 cyclonedds C 层段错误")
        log.error("排查方向:")
        log.error("  1) R1 是否在调试模式? (App 切到调试)")
        log.error("  2) R1 App 里'视频'开关是否打开?")
        log.error("  3) R1 firmware 版本是否过旧?")
        log.error("  4) 试 cyclonedds ls 看 R1 是否在发 video 相关主题")
        return 6

    if exit_code not in (0, None):
        log.error(f"子进程异常退出, exit code = {exit_code}")
        return 7

    # ---- 读子进程报告 ----
    if q.empty():
        log.error("子进程没回报结果")
        return 8

    result = q.get_nowait()
    status = result.get("status", -1)
    if status == 1:
        log.error(f"子进程初始化失败: {result.get('error')}")
        return 1
    if status == 2:
        log.warning(f"子进程抓帧失败: {result.get('error')}")
        if result.get("saved"):
            log.info(f"但保存了 {len(result['saved'])} 帧, 检查 {out_dir}")
        return 2

    # ---- 成功, 整理输出 ----
    saved = result.get("saved", [])
    sizes = result.get("sizes", [])
    log.info("=" * 60)
    log.info(f"成功抓到 {len(saved)} 帧")
    for s in saved:
        path = Path(s["path"])
        log.info(
            f"  - {path.name}  {s['w']}x{s['h']}  {s['bytes']:>7} B  "
            f"luma={s['luma']:6.1f}"
        )

    if sizes and len(set(sizes)) > 1:
        log.info(f"帧字节数变化 {min(sizes)} ~ {max(sizes)}  → 确认是真视频流")
    elif sizes:
        log.warning(f"所有帧字节数都是 {sizes[0]} — 可能是同一帧 / 占位图, 请肉眼确认")

    if args.probe:
        log.info("(probe 模式: 上面只是单次探测结果, 没保存)")

    log.info(f"输出目录: {out_dir.resolve()}")
    log.info("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
