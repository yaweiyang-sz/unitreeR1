"""跟随控制器: 把 BodyFollower 输出的 (vx, vyaw) 接到 R1 客户端。"""
from __future__ import annotations

import time
from dataclasses import dataclass

from ..robot.sdk_client import R1Client
from ..vision.body_follow import BodyFollower
from .velocity_smoother import VelocitySmoother, Velocity
from ..logger import setup_logger

log = setup_logger("r1.follow_ctrl")


class FollowController:
    def __init__(
        self,
        robot: R1Client,
        follower: BodyFollower,
        smoother: VelocitySmoother,
    ):
        self.robot = robot
        self.follower = follower
        self.smoother = smoother

    def step(self, frame) -> tuple[float, float]:
        """单步: 跑 P 控制, 平滑, 发指令。返回 (vx, vyaw) 平滑后值。"""
        vx, vyaw, target = self.follower.step(frame)
        target_v = Velocity(vx=vx, vy=0.0, vyaw=vyaw)
        cur = self.smoother.update(target_v)
        self.robot.move(cur.vx, cur.vy, cur.vyaw)
        return cur.vx, cur.vyaw
