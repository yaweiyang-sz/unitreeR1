"""手势 -> 速度指令映射。

设计:
- 持续发送, 不靠增量 (Move 是绝对速度指令)
- STOP 直接清零
- 前进/后退 用同一个线速度幅值
- 左右 用角速度 (vyaw)
- 速度幅值由 config.yaml 控制
- turn_direction_sign: +1 默认 (你站机器人后方), -1 翻转 (你面对机器人)
"""
from __future__ import annotations

from .velocity_smoother import Velocity
from ..vision.hand_gesture import Gesture


class GestureCommandMapper:
    def __init__(
        self,
        forward_speed: float = 0.3,
        backward_speed: float = -0.2,
        lateral_speed: float = 0.0,    # R1 侧移可能不支持, 留 0
        turn_speed: float = 0.5,
        turn_direction_sign: int = 1,  # 翻转让用户从机器人前方发手势时用 -1
    ):
        self.forward_speed = forward_speed
        self.backward_speed = backward_speed
        self.lateral_speed = lateral_speed
        self.turn_speed = turn_speed
        self.turn_direction_sign = turn_direction_sign

    def to_velocity(self, gesture: Gesture) -> Velocity:
        if gesture == Gesture.STOP or gesture == Gesture.UNKNOWN:
            return Velocity(0.0, 0.0, 0.0)
        if gesture == Gesture.FORWARD:
            return Velocity(self.forward_speed, 0.0, 0.0)
        if gesture == Gesture.BACKWARD:
            return Velocity(self.backward_speed, 0.0, 0.0)
        if gesture == Gesture.LEFT:
            return Velocity(0.0, 0.0, self.turn_direction_sign * self.turn_speed)
        if gesture == Gesture.RIGHT:
            return Velocity(0.0, 0.0, -self.turn_direction_sign * self.turn_speed)
        return Velocity(0.0, 0.0, 0.0)
