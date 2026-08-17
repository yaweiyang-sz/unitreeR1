"""SDK 集成测试 1: 网络/状态/运控 (高层)。

用法:
    python3 scripts/test_sdk_connection.py eth0
    python3 scripts/test_sdk_connection.py eth0 --dry-run
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.logger import setup_logger
from src.robot.sdk_client import R1Client, R1Mode

log = setup_logger("test.sdk")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("iface", nargs="?", default="eth0")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--no-move", action="store_true", help="只测连接和读状态, 不发移动指令")
    args = p.parse_args()

    mode = R1Mode.DRY_RUN if args.dry_run else R1Mode.REAL
    client = R1Client(args.iface, mode=mode)

    log.info("==== SDK 集成测试 ====")
    log.info(f"模式: {mode.value}, 网卡: {args.iface}")
    try:
        client.initialize()
    except Exception as e:  # noqa: BLE001
        log.error(f"初始化失败: {e}")
        log.error("检查: 1) 机器人是否开机 2) 网线/WiFi 是否通 3) 网卡名是否正确")
        return 1

    log.info("✓ 初始化成功")

    # 读状态
    st = client.get_state()
    if st:
        log.info(f"✓ 读到状态: {st}")
    else:
        log.warning("读不到状态 (不影响运控, 可忽略)")

    if not args.no_move and not args.dry_run:
        try:
            log.info("→ StandUp")
            client.stand_up()
            time.sleep(2.0)
            log.info("→ BalanceStand")
            client.balance_stand()
            time.sleep(1.0)
            log.info("→ Move(0.2, 0, 0) x 1.0s")
            client.move(0.2, 0, 0)
            time.sleep(1.0)
            log.info("→ StopMove")
            client.stop_move()
            time.sleep(0.5)
        except Exception as e:  # noqa: BLE001
            log.error(f"运控异常: {e}")
            return 2
        log.info("✓ 运控测试通过")
    else:
        log.info("跳过移动指令 (--no-move 或 dry-run)")

    client.shutdown()
    log.info("==== 完成 ====")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
