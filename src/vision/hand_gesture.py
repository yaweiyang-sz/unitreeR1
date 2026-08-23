"""手势识别: MediaPipe Hands 21 关键点 -> 5 类动作。

判定方案（v3, 2026-08）:
    STOP      ✋  5 指全展开 + 手腕水平速度 < 阈值（手保持不动）
    FORWARD   ✌️  食指+中指同时竖起（无名指+小指弯曲）
    BACKWARD  ✊  拳头（5 指全弯）
    LEFT      ✋  5 指全展开 + 手腕水平向左挥动（vx < -thresh）
    RIGHT     ✋  5 指全展开 + 手腕水平向右挥动（vx >  thresh）

设计要点:
    1. 5 指全张开是 "激活态", 用来给出 STOP/LEFT/RIGHT 三种动作
    2. 方向不靠"拇指/小指伸出"判定 (对手心朝向敏感),
       改成靠"手腕的水平速度" (挥动方向), 用户怎么拿手都行
    3. BACKWARD/FORWARD 是独立的形状判定, 不受挥动影响
    4. 手腕速度用 0.3s 滑动平均, 避免单帧抖动误判

依赖: mediapipe>=0.10
"""
from __future__ import annotations

import enum
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from ..logger import setup_logger

log = setup_logger("r1.gesture")


class Gesture(enum.Enum):
    UNKNOWN = "unknown"
    STOP = "stop"
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"


@dataclass
class GestureResult:
    gesture: Gesture
    confidence: float           # 0~1，组合手指状态得分
    handedness: str             # "Left" / "Right"
    hand_bbox: tuple[int, int, int, int]  # x, y, w, h
    landmarks_xy: np.ndarray    # (21, 2) 像素坐标
    inference_ms: float
    fingers_state: dict[str, bool] = None  # 每根手指 True=伸直 / False=弯曲
    fingers_ratio: dict[str, float] = None  # 4 指用 tip-mcp/pip-mcp, 拇指用 tip-wrist/mcp-wrist
    wrist_vx: float = 0.0       # 手腕水平速度 (px/s), 正=向右, 负=向左
    hand_active: bool = False    # True = 5指全张 (激活态, 可走 LEFT/RIGHT/STOP)
    # 位置 / 面积 (相对画面), UI 用来显示手在哪儿、有多大
    hand_center: tuple[int, int] = (0, 0)  # (cx, cy) in pixels
    hand_area_pct: float = 0.0             # 0~100, 占画面面积比例

    @property
    def is_valid(self) -> bool:
        return self.gesture != Gesture.UNKNOWN


class HandGestureDetector:
    """封装 MediaPipe Hands。线程不安全，调用方负责同步。"""

    def __init__(
        self,
        static_image_mode: bool = False,
        max_num_hands: int = 1,
        min_detection_confidence: float = 0.6,
        min_tracking_confidence: float = 0.5,
        use_legacy: bool = False,
        motion_window: int = 8,             # 速度平滑窗口 (帧)
        motion_speed_thresh: float = 150,   # 挥动识别阈值 (px/s)
        direction_hold_sec: float = 1.0,    # 方向持续时间 (s): 挥动停下后保持方向
        # ROI 区域限制 (相对坐标 0~1, 0/0/1/1 = 全画面不过滤)
        roi_x_pct: float = 0.20,
        roi_y_pct: float = 0.10,
        roi_w_pct: float = 0.60,
        roi_h_pct: float = 0.80,
        # 手部大小过滤 (相对画面面积的百分比), 0/100 = 不过滤
        min_hand_area_pct: float = 3.0,     # 太远 (< 3%) 不识别
        max_hand_area_pct: float = 50.0,    # 太近 (> 50%) 不识别
    ):
        try:
            import mediapipe as mp
        except ImportError as e:
            raise RuntimeError(
                "缺少 mediapipe，请先 `pip install mediapipe==0.10.18`\n"
                "或在 (yolov8) 环境里: `pip install -U mediapipe`"
            ) from e

        ver = getattr(mp, "__version__", "0.0.0")
        # 0.8 之前的版本没有 solutions 子模块；0.10.22+ 完全移除 legacy solutions
        if not hasattr(mp, "solutions") and not hasattr(mp, "tasks"):
            raise RuntimeError(
                f"当前 mediapipe 版本 ({ver}) 不支持 Hands API。\n"
                f"修复: `pip install -U mediapipe==0.10.18`\n"
                f"如果你用 conda (yolov8): `conda install -c conda-forge mediapipe=0.10.18`"
            )

        # 优先 legacy `mp.solutions.hands` (0.9 ~ 0.10.21)
        if hasattr(mp, "solutions"):
            self._mp = mp
            self._mp_hands = mp.solutions.hands
            self._mp_draw = mp.solutions.drawing_utils
            self._mp_styles = mp.solutions.drawing_styles
            self.hands = self._mp_hands.Hands(
                static_image_mode=static_image_mode,
                max_num_hands=max_num_hands,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )
            self._backend = "solutions"
        else:
            # fallback to tasks API (0.10.14+)
            from mediapipe.tasks import python as mp_python  # type: ignore
            from mediapipe.tasks.python import vision as mp_vision  # type: ignore
            base = self._model_path() or (
                "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
                "hand_landmarker/float16/1/hand_landmarker.task"
            )
            base_options = mp_python.BaseOptions(model_asset_path=base)
            options = mp_vision.HandLandmarkerOptions(
                base_options=base_options,
                num_hands=max_num_hands,
                min_hand_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )
            self._landmarker = mp_vision.HandLandmarker.create_from_options(options)
            self._mp = mp
            self._mp_draw = mp.solutions.drawing_utils if hasattr(mp, "solutions") else None
            self._backend = "tasks"
        log.info(f"MediaPipe Hands 已加载 (backend={self._backend}, version={ver})")
        # 手腕位置历史: 用于计算挥动方向/速度
        self._wrist_x_hist: deque[float] = deque(maxlen=motion_window)
        self._wrist_t_hist: deque[float] = deque(maxlen=motion_window)
        self._motion_speed_thresh = motion_speed_thresh
        # 方向持续时间: 识别到 LEFT/RIGHT 后, 保持该方向 N 秒
        self._direction_hold_sec = direction_hold_sec
        self._last_dir: Optional[Gesture] = None
        self._last_dir_t: float = 0.0

        # ROI 区域 (相对坐标 0~1, 默认中央 60%x80% 矩形)
        self.roi_x_pct = roi_x_pct
        self.roi_y_pct = roi_y_pct
        self.roi_w_pct = roi_w_pct
        self.roi_h_pct = roi_h_pct
        # 大小过滤 (占画面百分比)
        self.min_hand_area_pct = min_hand_area_pct
        self.max_hand_area_pct = max_hand_area_pct
        # 统计: 上次 detect 是否被 ROI / 大小过滤掉
        self._filtered_by_roi: int = 0
        self._filtered_by_size: int = 0

    def detect(self, frame_bgr: np.ndarray) -> Optional[GestureResult]:
        """对一帧 BGR 图像做手势检测。"""
        if frame_bgr is None or frame_bgr.size == 0:
            return None
        t0 = time.perf_counter()
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        h, w = frame_bgr.shape[:2]

        if self._backend == "solutions":
            res = self.hands.process(rgb)
            if not res.multi_hand_landmarks or not res.multi_handedness:
                return None
            lm = res.multi_hand_landmarks[0]
            handedness = res.multi_handedness[0].classification[0].label
            pts = np.array([(p.x * w, p.y * h) for p in lm.landmark], dtype=np.float32)
        else:
            # tasks API
            from mediapipe.tasks.python import vision as mp_vision  # type: ignore
            mp_image = mp_vision.Image(image_format=mp_vision.ImageFormat.SRGB, data=rgb)
            res = self._landmarker.detect(mp_image)
            if not res.hand_landmarks:
                return None
            lm = res.hand_landmarks[0]
            handedness = "Right" if res.handedness[0][0].category_name.lower().startswith("right") else "Left"
            pts = np.array([(p.x * w, p.y * h) for p in lm], dtype=np.float32)

        dt_ms = (time.perf_counter() - t0) * 1000.0
        x, y, ww, hh = cv2.boundingRect(pts.astype(np.int32))
        hand_bbox = (x, y, ww, hh)
        hand_cx, hand_cy = x + ww // 2, y + hh // 2
        hand_area_pct = (ww * hh) / max(1, w * h) * 100.0

        # ---- ROI 区域过滤 (中央区域内的手才识别) ----
        if not (
            self.roi_x_pct <= 0.0
            and self.roi_y_pct <= 0.0
            and self.roi_w_pct >= 1.0
            and self.roi_h_pct >= 1.0
        ):
            roi_x = int(w * self.roi_x_pct)
            roi_y = int(h * self.roi_y_pct)
            roi_w = int(w * self.roi_w_pct)
            roi_h = int(h * self.roi_h_pct)
            if not (
                roi_x <= hand_cx <= roi_x + roi_w
                and roi_y <= hand_cy <= roi_y + roi_h
            ):
                self._filtered_by_roi += 1
                return None

        # ---- 大小过滤 (太远/太近的不要) ----
        if not (self.min_hand_area_pct <= hand_area_pct <= self.max_hand_area_pct):
            self._filtered_by_size += 1
            return None

        fingers, ratios = _fingers_state(pts, handedness)

        # 计算手腕水平速度 (vx, px/s)
        wrist_x = float(pts[0][0])
        now = time.monotonic()
        self._wrist_x_hist.append(wrist_x)
        self._wrist_t_hist.append(now)
        wrist_vx = 0.0
        if len(self._wrist_x_hist) >= 3:
            dx = self._wrist_x_hist[-1] - self._wrist_x_hist[0]
            dt = self._wrist_t_hist[-1] - self._wrist_t_hist[0]
            if dt > 1e-3:
                wrist_vx = dx / dt

        # 分类: 5 指 ratio + 挥动速度
        hand_active = all(fingers.values())
        raw_gesture, conf = _classify_gesture(
            fingers, ratios, wrist_vx, self._motion_speed_thresh,
        )

        # 方向持续时间: 挥动停下后, 保持 LEFT/RIGHT 一段时间
        # 这样用户不用一直挥手, 1.0s 一次的节奏就能持续转向
        gesture = raw_gesture
        if raw_gesture in (Gesture.LEFT, Gesture.RIGHT):
            self._last_dir = raw_gesture
            self._last_dir_t = now
        elif raw_gesture == Gesture.STOP:
            # STOP: 检查是否在"方向保持"窗口内, 且手仍是激活态
            if (
                self._last_dir in (Gesture.LEFT, Gesture.RIGHT)
                and hand_active
                and (now - self._last_dir_t) < self._direction_hold_sec
            ):
                gesture = self._last_dir
        else:
            # BACKWARD / FORWARD / UNKNOWN: 清空方向记忆
            self._last_dir = None

        return GestureResult(
            gesture=gesture,
            confidence=conf,
            handedness=handedness,
            hand_bbox=hand_bbox,
            landmarks_xy=pts,
            inference_ms=dt_ms,
            fingers_state=fingers,
            fingers_ratio=ratios,
            wrist_vx=wrist_vx,
            hand_active=hand_active,
            hand_center=(hand_cx, hand_cy),
            hand_area_pct=hand_area_pct,
        )

    def draw(self, frame_bgr: np.ndarray, result: Optional[GestureResult]) -> np.ndarray:
        """在画面上画关键点。"""
        if result is None or self._backend != "solutions" or self._mp_draw is None:
            return frame_bgr
        from mediapipe.framework.formats import landmark_pb2  # type: ignore
        proto = landmark_pb2.NormalizedLandmarkList(
            landmark=[
                landmark_pb2.NormalizedLandmark(x=float(p[0]) / frame_bgr.shape[1],
                                                y=float(p[1]) / frame_bgr.shape[0])
                for p in result.landmarks_xy
            ]
        )
        self._mp_draw.draw_landmarks(
            frame_bgr,
            proto,
            self._mp_hands.HAND_CONNECTIONS,
            self._mp_styles.get_default_hand_landmarks_style(),
            self._mp_styles.get_default_hand_connections_style(),
        )
        return frame_bgr

    @staticmethod
    def _model_path() -> Optional[str]:
        """用户可放一个本地 hand_landmarker.task 模型文件; 这里先返回 None."""
        import os
        cand = os.environ.get("MEDIAPIPE_HAND_MODEL")
        return cand if cand and os.path.exists(cand) else None

    def close(self) -> None:
        try:
            if self._backend == "solutions":
                self.hands.close()
            else:
                self._landmarker.close()
        except Exception:  # noqa: BLE001
            pass

    def reset_state(self) -> None:
        """重置方向记忆 / 速度历史 (切换模式时用)."""
        self._wrist_x_hist.clear()
        self._wrist_t_hist.clear()
        self._last_dir = None
        self._last_dir_t = 0.0


# -------------------- 内部工具 --------------------

# MediaPipe Hands 关键点索引 (0-20):
#   0  wrist
#   1  thumb_CMC, 2 thumb_MCP, 3 thumb_IP, 4 thumb_TIP
#   5  index_MCP, 6 index_PIP, 7 index_DIP, 8 index_TIP
#   9  mid_MCP,  10 mid_PIP,  11 mid_DIP,  12 mid_TIP
#  13  ring_MCP, 14 ring_PIP, 15 ring_DIP, 16 ring_TIP
#  17  pinky_MCP,18 pinky_PIP,19 pinky_DIP,20 pinky_TIP


def _fingers_state(pts: np.ndarray, handedness: str) -> tuple[dict[str, bool], dict[str, float]]:
    """判断每根手指是否伸直（True = 伸直）。

    4 指判定: ratio = d(tip, mcp) / d(pip, mcp)
        伸直时 ratio ≈ 2.0 (tip 远离 mcp)
        弯曲时 ratio ≈ 1.0 (tip 靠近 mcp)
        阈值 1.65 介于中间, 实测能稳定区分 ✊/☝️/✋

    拇指判定: 用 tip-wrist 距离 vs mcp-wrist 距离
        伸直时 tip 远离 wrist (ratio 大)
        弯曲/贴在掌心时 tip 接近 wrist
        阈值 1.3 偏宽松, 配合 ratio 输出方便调试
    """
    states: dict[str, bool] = {}
    ratios: dict[str, float] = {}

    # 4 个非拇指手指
    fingers = [
        ("index",  8,  6,  5),
        ("middle", 12, 10, 9),
        ("ring",   16, 14, 13),
        ("pinky",  20, 18, 17),
    ]
    for name, tip_i, pip_i, mcp_i in fingers:
        d_tip_mcp = float(np.linalg.norm(pts[tip_i] - pts[mcp_i]))
        d_pip_mcp = float(np.linalg.norm(pts[pip_i] - pts[mcp_i]))
        ratio = d_tip_mcp / max(d_pip_mcp, 1e-6)
        ratios[name] = ratio
        states[name] = ratio > 1.65

    # 拇指: 用 tip 到 wrist 的距离 vs mcp 到 wrist 的距离
    thumb_tip = pts[4]
    thumb_mcp = pts[2]
    wrist = pts[0]
    d_tip_wrist = float(np.linalg.norm(thumb_tip - wrist))
    d_mcp_wrist = float(np.linalg.norm(thumb_mcp - wrist))
    ratio_thumb = d_tip_wrist / max(d_mcp_wrist, 1e-6)
    ratios["thumb"] = ratio_thumb
    states["thumb"] = ratio_thumb > 1.3

    return states, ratios


def _classify_gesture(
    fingers: dict[str, bool],
    ratios: dict[str, float],
    wrist_vx: float = 0.0,
    speed_thresh: float = 150.0,
) -> tuple[Gesture, float]:
    """根据 5 指 ratio + 手腕水平速度分类手势 (v3, 2026-08).

    阈值策略:
        弯曲 ratio < 1.3   (握拳时 ratio 0.4-1.0, 阈值 1.3 容许半握)
        伸直 ratio > 1.65  (激活态要求 5 指都明显伸直)
        食指/中指伸出 ratio > 1.4 (FORWARD 阈值, 比 1.65 宽松, 让用户稍微弯其它手指)
        拇指: 弯曲/伸直临界值 1.3, 但 BACKWARD 不依赖拇指 (握拳时拇指可能稍微伸)

    判定顺序:
        1. 4 指 (食/中/无/小) 都 < 1.3      -> BACKWARD (拳头, 拇指不参与)
        2. 食指 > 1.4 + 中指 > 1.4 + 无/小 都 < 1.3  -> FORWARD (拇指不参与, v3 起改双指)
        3. 5 指全张 (拇>1.3, 其他>1.65)    -> 激活态: 挥动决定 LEFT/RIGHT/STOP
        4. 其他 -> UNKNOWN
    """
    ri = ratios.get("index", 0.0)
    rm = ratios.get("middle", 0.0)
    rr = ratios.get("ring", 0.0)
    rp = ratios.get("pinky", 0.0)
    rt = ratios.get("thumb", 0.0)

    # 弯曲阈值 (容许半握: 完全握拳 ~0.5, 半握 ~1.0, 卡 1.3 让小指半伸也能识别)
    BENT = 1.3
    # 食指/中指伸直阈值 (比激活态的 1.65 宽松, 让用户稍微弯其它手指也能识别)
    INDEX_OUT = 1.4
    # 激活态伸直阈值 (要求 5 指都明显伸直)
    FULL_EXT = 1.65
    THUMB_EXT = 1.3

    # 1) 4 指全弯 (拇指不参与) -> BACKWARD (拳头)
    #    握拳时拇指可能稍微伸 (贴在食指旁边), 所以不查拇指
    if ri < BENT and rm < BENT and rr < BENT and rp < BENT:
        return Gesture.BACKWARD, 0.95

    # 2) 食指+中指 同时伸出, 无名指+小指弯曲 -> FORWARD (拇指不参与)
    #    v3 改成双指: 单指容易被误识别 (尤其远处小手掌), V 字手势更稳定
    if ri > INDEX_OUT and rm > INDEX_OUT and rr < BENT and rp < BENT:
        return Gesture.FORWARD, 0.95

    # 3) 5 指全张 -> 激活态: 用挥动方向判定
    all_ext = (
        rt > THUMB_EXT
        and ri > FULL_EXT
        and rm > FULL_EXT
        and rr > FULL_EXT
        and rp > FULL_EXT
    )
    if all_ext:
        if wrist_vx < -speed_thresh:
            return Gesture.LEFT, 0.9
        if wrist_vx > speed_thresh:
            return Gesture.RIGHT, 0.9
        return Gesture.STOP, 0.95

    return Gesture.UNKNOWN, 0.0


class GestureDebouncer:
    """连续 N 帧识别到同一手势才触发，避免噪声。"""

    def __init__(self, debounce_frames: int = 6):
        self.n = debounce_frames
        self._buf: list[Gesture] = []
        self._stable: Gesture = Gesture.UNKNOWN

    def update(self, g: Gesture) -> Gesture:
        self._buf.append(g)
        if len(self._buf) > self.n:
            self._buf.pop(0)
        # 统计最高频的非 UNKNOWN
        non_unknown = [x for x in self._buf if x != Gesture.UNKNOWN]
        if len(non_unknown) >= max(1, self.n // 2):
            from collections import Counter
            c = Counter(non_unknown)
            top, count = c.most_common(1)[0]
            if count >= self.n // 2:
                self._stable = top
        else:
            self._stable = Gesture.UNKNOWN
        return self._stable

    @property
    def stable(self) -> Gesture:
        return self._stable

    def reset(self) -> None:
        self._buf.clear()
        self._stable = Gesture.UNKNOWN
