"""速度平滑器：一阶低通 + 死区 + 限幅。

- max_step: 每次允许的最大速度变化 (m/s per tick)，避免指令跳变
- alpha: 指数平滑系数（0=保持旧值, 1=新值直接覆盖）
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import math


@dataclass
class Velocity:
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.vx, self.vy, self.vyaw)


class VelocitySmoother:
    """把原始目标速度平滑成可发指令的速度。"""

    def __init__(
        self,
        alpha: float = 0.3,
        max_step_vx: float = 0.05,
        max_step_vy: float = 0.05,
        max_step_vyaw: float = 0.15,
        max_abs_vx: float = 0.5,
        max_abs_vy: float = 0.3,
        max_abs_vyaw: float = 1.0,
        decay_to_zero_per_sec: float = 0.8,
    ):
        self.alpha = alpha
        self.max_step_vx = max_step_vx
        self.max_step_vy = max_step_vy
        self.max_step_vyaw = max_step_vyaw
        self.max_abs_vx = max_abs_vx
        self.max_abs_vy = max_abs_vy
        self.max_abs_vyaw = max_abs_vyaw
        self.decay = decay_to_zero_per_sec
        self._cur = Velocity()
        self._last_t: float | None = None

    def reset(self) -> None:
        self._cur = Velocity()
        self._last_t = None

    @property
    def current(self) -> Velocity:
        return self._cur

    def update(self, target: Velocity, now: float | None = None) -> Velocity:
        """推进一步: 平滑 + 限幅 + 衰减到 0。"""
        now = now if now is not None else time.monotonic()
        dt = 0.0 if self._last_t is None else (now - self._last_t)
        self._last_t = now

        # 1) 一阶低通
        cur = self._cur
        smooth = Velocity(
            vx=cur.vx + self.alpha * (target.vx - cur.vx),
            vy=cur.vy + self.alpha * (target.vy - cur.vy),
            vyaw=cur.vyaw + self.alpha * (target.vyaw - cur.vyaw),
        )

        # 2) 单步最大变化
        smooth.vx = _clamp_step(smooth.vx, cur.vx, self.max_step_vx)
        smooth.vy = _clamp_step(smooth.vy, cur.vy, self.max_step_vy)
        smooth.vyaw = _clamp_step(smooth.vyaw, cur.vyaw, self.max_step_vyaw)

        # 3) 绝对值限幅
        smooth.vx = _clamp_abs(smooth.vx, self.max_abs_vx)
        smooth.vy = _clamp_abs(smooth.vy, self.max_abs_vy)
        smooth.vyaw = _clamp_abs(smooth.vyaw, self.max_abs_vyaw)

        # 4) 如果目标接近 0, 衰减到 0
        if abs(target.vx) < 1e-3 and abs(target.vy) < 1e-3 and abs(target.vyaw) < 1e-3:
            decay = self.decay * dt
            smooth.vx = _decay(smooth.vx, decay)
            smooth.vy = _decay(smooth.vy, decay)
            smooth.vyaw = _decay(smooth.vyaw, decay)

        self._cur = smooth
        return self._cur


def _clamp_step(new: float, old: float, max_step: float) -> float:
    delta = new - old
    if abs(delta) <= max_step:
        return new
    return old + math.copysign(max_step, delta)


def _clamp_abs(v: float, max_abs: float) -> float:
    if abs(v) <= max_abs:
        return v
    return math.copysign(max_abs, v)


def _decay(v: float, amount: float) -> float:
    if abs(v) <= amount:
        return 0.0
    return v - math.copysign(amount, v)
