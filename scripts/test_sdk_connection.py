"""SDK 集成测试 1: 网络 / 状态 / 运控 (高层)。

按 R1 LocoClient 的真实 FSM 流程跑, 默认不在 Stance 之外发移动指令。
要看机器人真的动起来, 加 --enter-loco (会要求二次确认)。

⚠️ iface 必须是接 R1 机器人 LAN (192.168.123.x) 的那块网卡,
   常见命名: Jetson 上是 eth10 (192.168.123.164) 或 eth0 (看固件),
   不是连外网 / 办公网的那块网卡。

用法:
    python3 scripts/test_sdk_connection.py eth10            # 只测连接 + 状态
    python3 scripts/test_sdk_connection.py eth10 --enter-loco # Stance -> Start -> Move -> Stance
    python3 scripts/test_sdk_connection.py eth10 --dry-run
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.logger import setup_logger
from src.robot.sdk_client import R1Client, R1FsmState, R1Mode

log = setup_logger("test.sdk")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("iface", nargs="?", default="eth10")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument(
        "--enter-loco",
        action="store_true",
        help="进入 locomotion (FSM 811) 跑 Move/Stance 完整流程",
    )
    p.add_argument(
        "--yes",
        action="store_true",
        help="跳过 enter-loco 的二次确认 (脚本场景)",
    )
    p.add_argument(
        "--sport-state-topic",
        default="rt/sportmodestate",
        help="SportModeState_ DDS 主题",
    )
    p.add_argument(
        "--enable-state-sub",
        action="store_true",
        help="启用 SportModeState_ 订阅 (拿到 IMU/位置/速度, 默认关, 避免 cyclonedds 资源冲突)",
    )
    args = p.parse_args()

    mode = R1Mode.DRY_RUN if args.dry_run else R1Mode.REAL
    client = R1Client(
        args.iface,
        mode=mode,
        sport_state_topic=args.sport_state_topic,
        enable_state_subscription=args.enable_state_sub,
    )

    log.info("==== SDK 集成测试 (R1 LocoClient) ====")
    log.info(f"模式: {mode.value}  网卡: {args.iface}  状态主题: {args.sport_state_topic}  订阅: {'开' if args.enable_state_sub else '关(默认)'}")
    try:
        client.initialize()
    except Exception as e:  # noqa: BLE001
        log.error(f"初始化失败: {e}")
        log.error("检查: 1) 机器人是否开机 2) App 切到调试模式 3) 网卡名对 4) cyclonedds 安装")
        return 1

    log.info("✓ 初始化成功")

    # 等最多 3s 让 SportModeState 订阅跑至少一两帧 (默认 1Hz 发布)
    deadline = time.monotonic() + 3.0
    st = client.get_state()
    while not st and time.monotonic() < deadline:
        time.sleep(0.2)
        st = client.get_state()
    if st:
        log.info(
            f"✓ 读到状态: mode={st.get('mode')}, pos={st.get('position')}, "
            f"vel={st.get('velocity')}, imu_rpy={st.get('imu_rpy')}"
        )
    else:
        log.warning("3s 内没读到 SportModeState — 主题名可能不对, 但 FSM 仍可通过 RPC 拿")

    # 主路径: 主动 RPC 拉一次 FSM
    fsm = client.poll_fsm()
    log.info(f"当前 FSM (via LocoClient.GetFsmId RPC): {fsm.value}")

    if not args.enter_loco or args.dry_run:
        if args.dry_run:
            log.info("[DRY-RUN] 模拟进入 locomotion")
            client.enter_locomotion()
            time.sleep(0.3)
            log.info("→ Move(0.2, 0, 0) x 1.0s")
            client.move(0.2, 0, 0)
            time.sleep(1.0)
            log.info("→ StopMove")
            client.stop_move()
            time.sleep(0.2)
            client.exit_locomotion()
        else:
            log.info("跳过 locomotion (用 --enter-loco 才会让机器人走)")
        client.shutdown()
        log.info("==== 完成 ====")
        return 0

    # 真实进入 locomotion — 危险, 二次确认
    if not args.yes:
        log.warning("即将让 R1 进入 locomotion (FSM 811) — 危险操作")
        log.warning("确认: 1) 周围 ≥ 2m  2) 急停按钮在手  3) 机器人已用支架保护")
        try:
            input("按 Enter 继续, Ctrl-C 取消...")
        except KeyboardInterrupt:
            log.info("用户取消")
            client.shutdown()
            return 0

    try:
        client.enter_locomotion()
        # 给 FSM 切换一点时间, RPC 拉一次确认
        time.sleep(0.5)
        log.info(f"FSM after Start: {client.poll_fsm().value}")

        log.info("→ Move(0.2, 0, 0) x 1.0s")
        client.move(0.2, 0, 0)
        time.sleep(1.0)

        log.info("→ Move(0, 0, 0.3)  x 1.0s")
        client.move(0, 0, 0.3)
        time.sleep(1.0)

        log.info("→ StopMove")
        client.stop_move()
        time.sleep(0.5)

        log.info("→ exit_locomotion (Stance)")
        client.exit_locomotion()
        time.sleep(0.5)
        log.info(f"FSM after Stance: {client.poll_fsm().value}")
        log.info("✓ 运控测试通过")
    except Exception as e:  # noqa: BLE001
        log.exception(f"运控异常: {e}")
        try:
            client.exit_locomotion()
        except Exception:  # noqa: BLE001
            pass
        client.shutdown()
        return 2

    client.shutdown()
    log.info("==== 完成 ====")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
