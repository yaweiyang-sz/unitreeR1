"""人体跟随: MediaPipe Pose 33 关键点 -> P 控制输出 (vx, vyaw)。

策略:
    1. 在画面中找出所有完整人体 bbox
    2. 选一个目标（默认: 距离画面中心最近 / 面积最大；可由用户指定）
    3. P 控制:
        - 横向偏差 (目标中心 - 画面中心) -> 角速度 vyaw
        - 面积偏差 (目标面积 - 目标面积) -> 线速度 vx
    4. 死区: 偏差在阈值内不输出 (避免抖动)
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from ..logger import setup_logger

log = setup_logger("r1.follow")


@dataclass
class FollowTarget:
    bbox: tuple[int, int, int, int]   # x, y, w, h
    center: tuple[float, float]       # cx, cy
    area: float                       # 像素²
    landmarks_xy: np.ndarray = field(default=None)  # (33, 2) or None
    score: float = 1.0                # 锁定得分（越大越稳）


class BodyFollower:
    """基于 MediaPipe Pose 的人体检测 + 跟随。"""

    def __init__(
        self,
        target_center_x: int = 320,
        target_bbox_area: int = 50000,
        kp_yaw: float = 0.0025,
        kp_distance: float = 0.00004,
        max_linear_speed: float = 0.3,
        max_angular_speed: float = 0.5,
        deadzone_yaw: int = 30,
        deadzone_area: int = 8000,
    ):
        try:
            import mediapipe as mp
        except ImportError as e:
            raise RuntimeError("缺少 mediapipe，请 pip install mediapipe==0.10.18") from e
        ver = getattr(mp, "__version__", "0.0.0")
        if not hasattr(mp, "solutions") and not hasattr(mp, "tasks"):
            raise RuntimeError(
                f"mediapipe 版本 ({ver}) 缺少 solutions/tasks API。\n"
                f"修复: `pip install -U mediapipe==0.10.18`"
            )
        self._mp = mp
        if hasattr(mp, "solutions"):
            self._mp_pose = mp.solutions.pose
            self._mp_draw = mp.solutions.drawing_utils
            self._mp_styles = mp.solutions.drawing_styles
            self.pose = self._mp_pose.Pose(
                static_image_mode=False,
                model_complexity=0,
                smooth_landmarks=True,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self._backend = "solutions"
        else:
            # tasks API
            from mediapipe.tasks import python as mp_python  # type: ignore
            from mediapipe.tasks.python import vision as mp_vision  # type: ignore
            base = os.environ.get("MEDIAPIPE_POSE_MODEL") or (
                "https://storage.googleapis.com/mediapipe-models/pose_landmarker_lite/"
                "pose_landmarker_lite/float16/1/pose_landmarker_lite.task"
            )
            base_options = mp_python.BaseOptions(model_asset_path=base)
            options = mp_vision.PoseLandmarkerOptions(
                base_options=base_options,
                num_poses=1,
                min_pose_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self._landmarker = mp_vision.PoseLandmarker.create_from_options(options)
            self._mp_draw = None
            self._backend = "tasks"

        self.target_center_x = target_center_x
        self.target_bbox_area = target_bbox_area
        self.kp_yaw = kp_yaw
        self.kp_distance = kp_distance
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.deadzone_yaw = deadzone_yaw
        self.deadzone_area = deadzone_area
        self._locked: Optional[FollowTarget] = None
        self._lost_frames = 0
        log.info("BodyFollower 已加载")

    def lock(self, target: FollowTarget) -> None:
        self._locked = target
        self._lost_frames = 0

    def unlock(self) -> None:
        self._locked = None

    @property
    def is_locked(self) -> bool:
        return self._locked is not None

    def detect(self, frame_bgr: np.ndarray) -> list[FollowTarget]:
        """检测画面里的所有人，返回候选列表（不指定目标时由调用方选）。"""
        if frame_bgr is None or frame_bgr.size == 0:
            return []
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        h, w = frame_bgr.shape[:2]

        if self._backend == "solutions":
            res = self.pose.process(rgb)
            if not res.pose_landmarks:
                return []
            pts = np.array([(p.x * w, p.y * h) for p in res.pose_landmarks.landmark], dtype=np.float32)
        else:
            from mediapipe.tasks.python import vision as mp_vision  # type: ignore
            mp_image = mp_vision.Image(image_format=mp_vision.ImageFormat.SRGB, data=rgb)
            res = self._landmarker.detect(mp_image)
            if not res.pose_landmarks:
                return []
            pts = np.array([(p.x * w, p.y * h) for p in res.pose_landmarks[0]], dtype=np.float32)

        x, y, ww, hh = cv2.boundingRect(pts.astype(np.int32))
        return [FollowTarget(
            bbox=(x, y, ww, hh),
            center=(x + ww / 2.0, y + hh / 2.0),
            area=float(ww * hh),
            landmarks_xy=pts,
            score=1.0,
        )]

    def step(self, frame_bgr: np.ndarray) -> tuple[float, float, Optional[FollowTarget]]:
        """单步控制: 返回 (vx, vyaw, current_target)。

        - 如果没有锁定目标, 自动选最大的一个
        - 锁定后丢失超过 30 帧则解锁
        """
        targets = self.detect(frame_bgr)
        if not targets:
            self._lost_frames += 1
            if self._lost_frames > 30 and self._locked is not None:
                log.info("目标丢失 30 帧, 自动解锁")
                self._locked = None
            return 0.0, 0.0, None

        # 自动锁定: 选距离画面中心最近 + 面积较大
        h, w = frame_bgr.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        if self._locked is None:
            best = min(targets, key=lambda t: (t.center[0] - cx) ** 2 + (t.center[1] - cy) ** 2 - 0.0001 * t.area)
            self._locked = best
            self._lost_frames = 0
        else:
            # 跟踪模式: 找最靠近上一个锁定的目标
            px, py = self._locked.center
            best = min(targets, key=lambda t: (t.center[0] - px) ** 2 + (t.center[1] - py) ** 2)
            self._locked = best
            self._lost_frames = 0

        tgt = self._locked
        # 偏差
        err_yaw = tgt.center[0] - self.target_center_x
        err_area = self.target_bbox_area - tgt.area  # 正: 目标太远, 需前进

        # 死区
        if abs(err_yaw) < self.deadzone_yaw:
            err_yaw = 0.0
        if abs(err_area) < self.deadzone_area:
            err_area = 0.0

        # P 控制
        vyaw = -err_yaw * self.kp_yaw
        vx = err_area * self.kp_distance

        # 限幅
        vx = max(-self.max_linear_speed, min(self.max_linear_speed, vx))
        vyaw = max(-self.max_angular_speed, min(self.max_angular_speed, vyaw))
        return vx, vyaw, tgt

    def draw(self, frame_bgr: np.ndarray, target: Optional[FollowTarget]) -> np.ndarray:
        if target is None or target.landmarks_xy is None or self._mp_draw is None:
            # 即使不画骨架, 也画 bbox
            if target is not None:
                x, y, w, h = target.bbox
                cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
            return frame_bgr
        from mediapipe.framework.formats import landmark_pb2  # type: ignore
        proto = landmark_pb2.NormalizedLandmarkList(
            landmark=[
                landmark_pb2.NormalizedLandmark(x=float(p[0]) / frame_bgr.shape[1],
                                                y=float(p[1]) / frame_bgr.shape[0])
                for p in target.landmarks_xy
            ]
        )
        self._mp_draw.draw_landmarks(
            frame_bgr,
            proto,
            self._mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=self._mp_styles.get_default_pose_landmarks_style(),
        )
        x, y, w, h = target.bbox
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return frame_bgr

    def close(self) -> None:
        try:
            if self._backend == "solutions":
                self.pose.close()
            else:
                self._landmarker.close()
        except Exception:  # noqa: BLE001
            pass
