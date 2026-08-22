"""宇树 R1 EDU 机器人客户端封装（运控 + 视频 + 状态订阅）。

设计要点:
1. 真机模式 (R1Mode.REAL) 走 unitree_sdk2py:
   - 运控: unitree_sdk2py.r1.loco.r1_loco_client.LocoClient (FSM: Damp / Stance / Start / Lie2StandUp ...)
   - 视频: unitree_sdk2py.go2.video.video_client.VideoClient (跨机型复用)
   - 状态: ChannelSubscriber("rt/sportmodestate", SportModeState_) (DDS 直订)
2. 模拟模式 (R1Mode.DRY_RUN) 完全不依赖 SDK，可在无机器人环境调试视觉 + 状态机。
3. R1 没有 BalanceStand / StandUp / StandDown 这种 G1/Go2 风格的方法,
   这里同时保留 stand_up / stand_down / balance_stand 作为兼容老代码的别名
   (内部映射到 Lie2StandUp / StandUp2Lie / Stance)。
"""
from .sdk_client import (
    R1Client,
    R1FsmState,
    R1Mode,
    VideoFrame,
    SPORT_STATE_TOPIC_DEFAULT,
    SPORT_STATE_TOPIC_ALT,
)

__all__ = [
    "R1Client",
    "R1Mode",
    "R1FsmState",
    "VideoFrame",
    "SPORT_STATE_TOPIC_DEFAULT",
    "SPORT_STATE_TOPIC_ALT",
]
